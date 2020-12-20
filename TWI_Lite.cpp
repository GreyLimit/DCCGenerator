//
//	twi.cpp - TWI/I2C library for Wiring & Arduino
//	Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
//
//	This library is free software; you can redistribute it and/or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.
//
//	This library is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//	Lesser General Public License for more details.
//
//	You should have received a copy of the GNU Lesser General Public
//	License along with this library; if not, write to the Free Software
//	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
//	Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts
//	Modified 2020 by Greyson Christoforo (grey@christoforo.net) to implement timeouts
//	Modified 2020 by Jeff Penfold <jeff.penfold@googlemail.com> to add lightness
//

#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <compat/twi.h>
#include "Arduino.h"		// for digitalWrite and micros

#include "pins_arduino.h"
#include "TWI_Lite.h"

static volatile uint8_t twi_state;
static volatile uint8_t twi_slarw;
static volatile uint8_t twi_sendStop;			// should the transaction end with a stop
static volatile uint8_t twi_inRepStart;			// in the middle of a repeated start

//
//	twi_timeout_us > 0 prevents the code from getting stuck in various while loops here
//	if twi_timeout_us == 0 then timeout checking is disabled (the previous Wire lib behavior)
//	at some point in the future, the default twi_timeout_us value could become 25000
//	and twi_do_reset_on_timeout could become true to conform to the SMBus standard
//
//	http://smbus.org/specs/SMBus_3_1_20180319.pdf
//
static volatile uint32_t twi_timeout_us = 0ul;
static volatile bool twi_timed_out_flag = false;	// a timeout has been seen
static volatile bool twi_do_reset_on_timeout = false;	// reset the TWI registers on timeout

static void (*twi_onSlaveTransmit)( void );
static void (*twi_onSlaveReceive)( uint8_t*, int );

//
//	NOTE/
//
//	Need to assess the actaul requirement for three separate
//	buffer areas.  If this is a state machine, then (surely)
//	there is only a need for a single buffer at any single time?
//


//
//	Used in following routines:
//
//		twi_readFrom()
//		twi_writeTo()
//
//	and ISR states:
//
//		TW_MT_SLA_ACK
//		TW_MT_DATA_ACK
//		TW_MR_DATA_ACK
//		TW_MR_DATA_NACK
//
static uint8_t twi_masterBuffer[ TWI_BUFFER_LENGTH ];
static volatile uint8_t twi_masterBufferIndex;
static volatile uint8_t twi_masterBufferLength;

//
//	Used in following routines:
//
//		twi_transmit()
//
//	and ISR states:
//
//		TW_ST_SLA_ACK
//		TW_ST_ARB_LOST_SLA_ACK
//		TW_ST_DATA_ACK
//
static uint8_t twi_txBuffer[ TWI_BUFFER_LENGTH ];
static volatile uint8_t twi_txBufferIndex;
static volatile uint8_t twi_txBufferLength;

//
//	Used in following ISR states:
//
//		TW_SR_DATA_ACK
//		TW_SR_GCALL_DATA_ACK
//		TW_SR_STOP
//
static uint8_t twi_rxBuffer[ TWI_BUFFER_LENGTH ];
static volatile uint8_t twi_rxBufferIndex;

//
//	Global TWI error flag
//
static volatile uint8_t twi_error;

// 
//	Function	twi_init
//	Desc		readys twi pins and sets twi bitrate
//	Input		none
//	Output		none
//
void twi_init( void ) {
	//
	//	Initialize state
	//
	twi_state = TWI_READY;
	twi_sendStop = true;
	twi_inRepStart = false;

	//
	//	Activate internal pullups for twi.
	//
	digitalWrite( SDA, 1 );
	digitalWrite( SCL, 1 );

	//
	//	Initialize twi prescaler and bit rate
	//
	bitClear( _SFR_BYTE( TWSR ), TWPS0 );
	bitClear( _SFR_BYTE( TWSR ), TWPS1 );
	TWBR = (( F_CPU / TWI_FREQ ) - 16 ) / 2;

	//
	//	twi bit rate formula from atmega128 manual pg 204
	//	SCL Frequency = CPU Clock Frequency / (16 + (2 * TWBR))
	//
	//	note: TWBR should be 10 or higher for master mode
	//	It is 72 for a 16mhz Wiring board with 100kHz TWI
	//

	//
	//	Enable twi module, acks, and twi interrupt
	//
	TWCR = bit( TWEN ) | bit( TWIE ) | bit( TWEA );
}

// 
//	Function	twi_disable
//	Desc		disables twi pins
//	Input		none
//	Output		none
//
void twi_disable( void ) {
	//
	//	Disable twi module, acks, and twi interrupt
	//
	TWCR &= ~( bit( TWEN ) | bit( TWIE ) | bit( TWEA ));

	//
	//	Deactivate internal pullups for twi.
	//
	digitalWrite( SDA, 0 );
	digitalWrite( SCL, 0 );
}

//
//	Function twi_slaveInit
//	Desc     sets slave address and enables interrupt
//	Input    none
//	Output   none
//
void twi_setAddress( uint8_t address ) {
	//
	//	Set twi slave address (skip over TWGCE bit)
	//
	TWAR = address << 1;
}

//
//	Function	twi_setClock
//	Desc		sets twi bit rate
//	Input		Clock Frequency
//	Output		none
//
void twi_setFrequency( uint32_t frequency ) {
	TWBR = (( F_CPU / frequency ) - 16) / 2;

	//
	//	twi bit rate formula from atmega128 manual pg 204
	//	SCL Frequency = CPU Clock Frequency / (16 + (2 * TWBR))
	//
	//	note: TWBR should be 10 or higher for master mode
	//	It is 72 for a 16mhz Wiring board with 100kHz TWI */
}

//
//	Function	twi_readFrom
//	Desc		attempts to become twi bus master and read a
//			series of bytes from a device on the bus
//	Input		address: 7bit i2c device address
//			data: pointer to byte array
//			length: number of bytes to read into array
//			sendStop: Boolean indicating whether to send a stop at the end
//	Output		number of bytes read
//
uint8_t twi_readFrom( uint8_t address, uint8_t* data, uint8_t length, bool sendStop ) {
	//
	//	Ensure data will fit into buffer
	//
	if( TWI_BUFFER_LENGTH < length ) {
		return( 0 );
	}

	//
	//	Wait until twi is ready, become master receiver
	//
	uint32_t startMicros = micros();
	
	while( TWI_READY != twi_state ) {
		if(( twi_timeout_us > 0ul ) && (( micros() - startMicros) > twi_timeout_us )) {
			twi_handleTimeout( twi_do_reset_on_timeout );
			return( 0 );
		}
	}
	twi_state = TWI_MRX;
	twi_sendStop = sendStop;
	//
	//	Reset error state (0xFF.. no error occured)
	//
	twi_error = 0xFF;

	//
	//	Initialize buffer iteration vars
	//
	twi_masterBufferIndex = 0;
	twi_masterBufferLength = length-1;  // This is not intuitive, read on...
	//
	//	On receive, the previously configured ACK/NACK setting is transmitted in
	//	response to the received byte before the interrupt is signalled. 
	//	Therefor we must actually set NACK when the _next_ to last byte is
	//	received, causing that NACK to be sent in response to receiving the last
	//	expected byte of data.
	//
	//	Build sla+w, slave device address + w bit
	//
	twi_slarw = TW_READ;
	twi_slarw |= address << 1;

	if( true == twi_inRepStart ) {
		//
		//	if we're in the repeated start state, then we've already sent the start,
		//	(@@@ we hope), and the TWI statemachine is just waiting for the address byte.
		//	We need to remove ourselves from the repeated start state before we enable interrupts,
		//	since the ISR is ASYNC, and we could get confused if we hit the ISR before cleaning
		//	up. Also, don't enable the START interrupt. There may be one pending from the 
		//	repeated start that we sent ourselves, and that would really confuse things.
		//
		twi_inRepStart = false;			// Remember, we're dealing with an ASYNC ISR
		startMicros = micros();
		do {
			TWDR = twi_slarw;
			if(( twi_timeout_us > 0ul ) && (( micros() - startMicros) > twi_timeout_us )) {
				twi_handleTimeout( twi_do_reset_on_timeout );
				return( 0 );
			}
		} while( TWCR & bit( TWWC ));
		TWCR = bit( TWINT ) | bit( TWEA ) | bit( TWEN ) | bit( TWIE );	// enable INTs, but not START
	}
	else {
		//
		//	Send start condition
		//
		TWCR = bit( TWEN ) | bit( TWIE ) | bit( TWEA ) | bit( TWINT ) | bit( TWSTA );
	}

	//
	//	Wait for read operation to complete
	//
	startMicros = micros();
	while( TWI_MRX == twi_state ) {
		if(( twi_timeout_us > 0ul ) && (( micros() - startMicros) > twi_timeout_us )) {
			twi_handleTimeout( twi_do_reset_on_timeout );
			return( 0 );
		}
	}

	if( twi_masterBufferIndex < length ) length = twi_masterBufferIndex;

	//
	//	Copy twi buffer to data
	//
	for( int i = 0; i < length; i++ ) data[ i ] = twi_masterBuffer[ i ];

	return( length );
}

//
//	Function	twi_writeTo
//	Desc		attempts to become twi bus master and write a
//			series of bytes to a device on the bus
//	Input		address: 7bit i2c device address
//			data: pointer to byte array
//			length: number of bytes in array
//			wait: boolean indicating to wait for write or not
//			sendStop: boolean indicating whether or not to send a stop at the end
//	Output		0 .. success
//			1 .. length to long for buffer
//			2 .. address send, NACK received
//			3 .. data send, NACK received
//			4 .. other twi error (lost bus arbitration, bus error, ..)
//			5 .. timeout
//
uint8_t twi_writeTo( uint8_t address, uint8_t* data, uint8_t length, uint8_t wait, bool sendStop ) {
	//
	//	Ensure data will fit into buffer
	//
	if( TWI_BUFFER_LENGTH < length ) return( 1 );


	//
	//	Wait until twi is ready, become master transmitter
	//
	uint32_t startMicros = micros();

	while( TWI_READY != twi_state ) {
		if(( twi_timeout_us > 0ul ) && (( micros() - startMicros ) > twi_timeout_us )) {
			twi_handleTimeout( twi_do_reset_on_timeout );
			return( 5 );
		}
	}
	twi_state = TWI_MTX;
	twi_sendStop = sendStop;
	//
	//	Reset error state (0xFF.. no error occured)
	//
	twi_error = 0xFF;

	//
	//	Initialize buffer iteration vars
	//
	twi_masterBufferIndex = 0;
	twi_masterBufferLength = length;

	//
	//	Copy data to twi buffer
	//
	for( int i = 0; i < length; i++ ) twi_masterBuffer[ i ] = data[ i ];

	//
	//	Build sla+w, slave device address + w bit
	//
	twi_slarw = TW_WRITE;
	twi_slarw |= address << 1;

	//
	//	If we're in a repeated start, then we've already sent the START
	//	in the ISR. Don't do it again.
	//
	if( true == twi_inRepStart ) {
		//
		//	if we're in the repeated start state, then we've already sent the start,
		//	(@@@ we hope), and the TWI statemachine is just waiting for the address byte.
		//	We need to remove ourselves from the repeated start state before we enable interrupts,
		//	since the ISR is ASYNC, and we could get confused if we hit the ISR before cleaning
		//	up. Also, don't enable the START interrupt. There may be one pending from the 
		//	repeated start that we sent outselves, and that would really confuse things.
		//
		twi_inRepStart = false;			// remember, we're dealing with an ASYNC ISR
		startMicros = micros();
		do {
			TWDR = twi_slarw;
			if(( twi_timeout_us > 0ul ) && (( micros() - startMicros ) > twi_timeout_us )) {
				twi_handleTimeout( twi_do_reset_on_timeout );
				return( 5 );
			}
		} while( TWCR & bit( TWWC ));
		TWCR = bit( TWINT ) | bit( TWEA ) | bit( TWEN ) | bit( TWIE );	// enable INTs, but not START
	}
	else {
		//
		//	Send start condition
		//
		TWCR = bit( TWINT ) | bit( TWEA ) | bit( TWEN ) | bit( TWIE ) | bit( TWSTA );	// enable INTs
	}

	//
	//	Wait for write operation to complete
	//
	startMicros = micros();
	while( wait && ( TWI_MTX == twi_state )){
		if(( twi_timeout_us > 0ul ) && (( micros() - startMicros ) > twi_timeout_us )) {
			twi_handleTimeout( twi_do_reset_on_timeout );
			return( 5 );
		}
	}
	switch( twi_error ) {
		case 0xFF:		return( 0 );	// success
		case TW_MT_SLA_NACK:	return( 2 );	// error: address send, nack received
		case TW_MT_DATA_NACK:	return( 3 );	// error: data send, nack received
		default:		break;
	}
	return( 4 );					// other twi error
}

//
//	Function	twi_transmit
//	Desc		fills slave tx buffer with data
//			must be called in slave tx event callback
//	Input		data: pointer to byte array
//			length: number of bytes in array
//	Output		1 length too long for buffer
//			2 not slave transmitter
//			0 ok
//
uint8_t twi_transmit( const uint8_t* data, uint8_t length ) {
	//
	//	Ensure data will fit into buffer
	//
	if( TWI_BUFFER_LENGTH < ( twi_txBufferLength + length )) return( 1 );

	//
	//	Ensure we are currently a slave transmitter
	//
	if( TWI_STX != twi_state ) return( 2 );

	//
	//	Set length and copy data into tx buffer
	//
	for( int i = 0; i < length; i++ ) twi_txBuffer[ twi_txBufferLength + i ] = data[ i ];
	twi_txBufferLength += length;

	return( 0 );
}

//
//	Function	twi_attachSlaveRxEvent
//	Desc		sets function called before a slave read operation
//	Input		function: callback function to use
//	Output		none
//
void twi_attachSlaveRxEvent( void (*function)( uint8_t*, int )) {
	twi_onSlaveReceive = function;
}

//
//	Function	twi_attachSlaveTxEvent
//	Desc		sets function called before a slave write operation
//	Input		function: callback function to use
//	Output		none
//
void twi_attachSlaveTxEvent( void (*function)( void )) {
	twi_onSlaveTransmit = function;
}

//
//	Function	twi_reply
//	Desc		sends byte or readys receive line
//	Input		ack: byte indicating to ack or to nack
//	Output		none
//
void twi_reply( bool ack ) {
	//
	//	Transmit master read ready signal, with or without ack
	//
	TWCR = bit( TWEN ) | bit( TWIE ) | bit( TWINT ) | ( ack? bit( TWEA ): 0 );
}

//
//	Function	twi_stop
//	Desc		relinquishes bus master status
//	Input		none
//	Output		none
//
void twi_stop( void ) {
	//
	//	Send stop condition
	//
	TWCR = bit( TWEN ) | bit( TWIE ) | bit( TWEA ) | bit( TWINT ) | bit( TWSTO );

	//
	//	Wait for stop condition to be exectued on bus
	//	TWINT is not set after a stop condition!
	//
	//	 We cannot use micros() from an ISR, so approximate the timeout with cycle-counted delays
	//
	const uint8_t us_per_loop = 8;
	uint32_t counter = ( twi_timeout_us + us_per_loop - 1 )/ us_per_loop; // Round up
	
	while( TWCR & bit( TWSTO )){
		if( twi_timeout_us > 0ul ) {
			if( counter > 0ul ) {
				_delay_us( 10 );
				counter--;
			}
			else {
				twi_handleTimeout( twi_do_reset_on_timeout );
				return;
			}
		}
	}

	// update twi state
	twi_state = TWI_READY;
}

//
//	Function	twi_releaseBus
//	Desc		releases bus control
//	Input		none
//	Output		none
//
void twi_releaseBus( void ) {
	//
	//	Release bus
	//
	TWCR = bit( TWEN ) | bit( TWIE ) | bit( TWEA ) | bit( TWINT );

	//
	//	Update twi state
	//
	twi_state = TWI_READY;
}

//
//	Function	twi_setTimeoutInMicros
//	Desc		set a timeout for while loops that twi might get stuck in
//	Input		timeout value in microseconds (0 means never time out)
//	Input		reset_with_timeout: true causes timeout events to reset twi
//	Output		none
//
void twi_setTimeoutInMicros( uint32_t timeout, bool reset_with_timeout ) {
	twi_timed_out_flag = false;
	twi_timeout_us = timeout;
	twi_do_reset_on_timeout = reset_with_timeout;
}

//
//	Function	twi_handleTimeout
//	Desc		this gets called whenever a while loop here has lasted longer than
//			twi_timeout_us microseconds. always sets twi_timed_out_flag
//	Input		reset: true causes this function to reset the twi hardware interface
//	Output		none
//
void twi_handleTimeout( bool reset ) {
	twi_timed_out_flag = true;

	if( reset ) {
		//
		//	Remember bitrate and address settings
		//
		uint8_t previous_TWBR = TWBR;
		uint8_t previous_TWAR = TWAR;

		//
		//	Reset the interface
		//
		twi_disable();
		twi_init();

		//
		//	Reapply the previous register values
		//
		TWAR = previous_TWAR;
		TWBR = previous_TWBR;
	}
}

//
//	Function	twi_manageTimeoutFlag
//	Desc		returns true if twi has seen a timeout
//			optionally clears the timeout flag
//	Input		clear_flag: true if we should reset the hardware
//	Output		none
//
bool twi_manageTimeoutFlag( bool clear_flag ) {
	bool flag = twi_timed_out_flag;
	
	if( clear_flag ) twi_timed_out_flag = false;
	return( flag );
}

//
//	Two Wire Interrupt Service Routine.
//
ISR( TWI_vect ) {
	switch( TW_STATUS ) {
		//
		//	All Master
		//
		case TW_START:     	// sent start condition
		case TW_REP_START: {	// sent repeated start condition
			//
			//	Copy device address and r/w bit to output register and ack
			//
			TWDR = twi_slarw;
			twi_reply( true );
			break;
		}
		//
		//	Master Transmitter
		//
		case TW_MT_SLA_ACK:	// slave receiver acked address
		case TW_MT_DATA_ACK: {	// slave receiver acked data
			//
			//	If there is data to send, send it, otherwise stop
			//
			if( twi_masterBufferIndex < twi_masterBufferLength ) {
				//
				//	Copy data to output register and ack
				//
				TWDR = twi_masterBuffer[ twi_masterBufferIndex++ ];
				twi_reply( true );
			}
			else{
				if( twi_sendStop ) {
					twi_stop();
				}
				else {
					twi_inRepStart = true;	// we're gonna send the START
					//
					//	Don't enable the interrupt. We'll generate the start, but we
					//	avoid handling the interrupt until we're in the next transaction,
					//	at the point where we would normally issue the start.
					//
					TWCR = bit( TWINT ) | bit( TWSTA )| bit( TWEN );
					twi_state = TWI_READY;
				}
			}
			break;
		}
		case TW_MT_SLA_NACK: {	// address sent, nack received
			twi_error = TW_MT_SLA_NACK;
			twi_stop();
			break;
		}
		case TW_MT_DATA_NACK: {	// data sent, nack received
			twi_error = TW_MT_DATA_NACK;
			twi_stop();
			break;
		}
		case TW_MT_ARB_LOST: {	// lost bus arbitration
			twi_error = TW_MT_ARB_LOST;
			twi_releaseBus();
			break;
		}
		//
		//	Master Receiver
		//
		case TW_MR_DATA_ACK: {	// data received, ack sent
			//
			//	Put byte into buffer
			//
			twi_masterBuffer[ twi_masterBufferIndex++ ] = TWDR;
			__attribute__ ((fallthrough));
		}
		case TW_MR_SLA_ACK: {	// address sent, ack received
			//
			//	Ack if more bytes are expected, otherwise nack
			//
			twi_reply( twi_masterBufferIndex < twi_masterBufferLength );
			break;
		}
		case TW_MR_DATA_NACK: {	// data received, nack sent
			//
			//	Put final byte into buffer
			//
			twi_masterBuffer[ twi_masterBufferIndex++ ] = TWDR;
			if( twi_sendStop ) {
				twi_stop();
			}
			else {
				twi_inRepStart = true;	// we're gonna send the START
				//
				//	Don't enable the interrupt. We'll generate the start, but we
				//	avoid handling the interrupt until we're in the next transaction,
				//	at the point where we would normally issue the start.
				//
				TWCR = bit( TWINT ) | bit( TWSTA )| bit( TWEN ) ;
				twi_state = TWI_READY;
			}
			break;
		}
		case TW_MR_SLA_NACK: {	// address sent, nack received
			twi_stop();
			break;
		}
		//
		//	TW_MR_ARB_LOST handled by TW_MT_ARB_LOST case
		//
		//	Slave Receiver
		//
		case TW_SR_SLA_ACK:			// addressed, returned ack
		case TW_SR_GCALL_ACK:			// addressed generally, returned ack
		case TW_SR_ARB_LOST_SLA_ACK:		// lost arbitration, returned ack
		case TW_SR_ARB_LOST_GCALL_ACK: {	// lost arbitration, returned ack
			//
			//	Enter slave receiver mode
			//
			twi_state = TWI_SRX;
			//
			//	Indicate that rx buffer can be overwritten and ack
			//
			twi_rxBufferIndex = 0;
			twi_reply( true );
			break;
		}
		case TW_SR_DATA_ACK:		// data received, returned ack
		case TW_SR_GCALL_DATA_ACK: {	// data received generally, returned ack
			//
			//	If there is still room in the rx buffer
			//
			if( twi_rxBufferIndex < TWI_BUFFER_LENGTH ) {
				//
				//	Put byte in buffer and ack
				//
				twi_rxBuffer[ twi_rxBufferIndex++ ] = TWDR;
				twi_reply( true );
			}
			else {
				//
				//	Otherwise nack
				//
				twi_reply( false );
			}
			break;
		}
		case TW_SR_STOP: {		// stop or repeated start condition received
			//
			//	Ack future responses and leave slave receiver state
			//
			twi_releaseBus();
			//
			//	Put a null char after data if there's room
			//
			if( twi_rxBufferIndex < TWI_BUFFER_LENGTH ) {
				twi_rxBuffer[ twi_rxBufferIndex ] = '\0';
			}
			//
			//	Callback to user defined callback
			//
			twi_onSlaveReceive( twi_rxBuffer, twi_rxBufferIndex );
			//
			//	Since we submit rx buffer to "wire" library, we can reset it
			//
			twi_rxBufferIndex = 0;
			break;
		}
		case TW_SR_DATA_NACK:		// data received, returned nack
		case TW_SR_GCALL_DATA_NACK: {	// data received generally, returned nack
			//
			//	Nack back at master
			//
			twi_reply( false );
			break;
		}
		//
		//	Slave Transmitter
		//
		case TW_ST_SLA_ACK:		// addressed, returned ack
		case TW_ST_ARB_LOST_SLA_ACK: {	// arbitration lost, returned ack
			//
			//	Enter slave transmitter mode
			//
			twi_state = TWI_STX;
			//
			//	Ready the tx buffer index for iteration
			//
			twi_txBufferIndex = 0;
			//
			//	Set tx buffer length to be zero, to verify if user changes it
			//
			twi_txBufferLength = 0;
			//
			//	Request for txBuffer to be filled and length to be set
			//
			//	note: user must call twi_transmit(bytes, length) to do this
			//
			twi_onSlaveTransmit();
			//
			//	If they didn't change buffer & length, initialize it
			//
			if( 0 == twi_txBufferLength ) {
				twi_txBufferLength = 1;
				twi_txBuffer[0] = 0x00;
			}
			__attribute__ ((fallthrough));
		}
		//
		//	Transmit first byte from buffer, fall
		case TW_ST_DATA_ACK: {		// byte sent, ack returned
			//
			//	Copy data to output register
			//
			TWDR = twi_txBuffer[ twi_txBufferIndex++ ];
			//
			//	If there is more to send, ack, otherwise nack
			//
			twi_reply( twi_txBufferIndex < twi_txBufferLength );
			break;
		}
		case TW_ST_DATA_NACK:		// received nack, we are done 
		case TW_ST_LAST_DATA: {		// received ack, but we are done already!
			//
			//	Ack future responses
			//
			twi_reply( true );
			//
			//	Leave slave receiver state
			//
			twi_state = TWI_READY;
			break;
		}
		//
		//	All
		//
		case TW_NO_INFO: {	// no state information
			break;
		}
		case TW_BUS_ERROR: {	// bus error, illegal stop/start
			twi_error = TW_BUS_ERROR;
			twi_stop();
			break;
		}
	}
}

//
//	EOF
//

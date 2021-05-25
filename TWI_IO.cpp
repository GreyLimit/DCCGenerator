///
///	TWI_IO.cpp - A simplified TWI/I2C API library for Arduino
///
///	Copyright (c) 2021 Jeff Penfold.  All right reserved.
///
///	This library is free software; you can redistribute it and/or
///	modify it under the terms of the GNU Lesser General Public
///	License as published by the Free Software Foundation; either
///	version 2.1 of the License, or (at your option) any later version.
///
///	This library is distributed in the hope that it will be useful,
///	but WITHOUT ANY WARRANTY; without even the implied warranty of
///	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
///	Lesser General Public License for more details.
///
///	You should have received a copy of the GNU Lesser General Public
///	License along with this library; if not, write to the Free Software
///	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301
///	USA
///

#include <limits.h>
#include <stdlib.h>
#include <inttypes.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <compat/twi.h>

#include "Arduino.h"
#include "pins_arduino.h"
#include "TWI_IO.h"

//
//	Design Discussion
//	=================
//
//	The "SMBus_3_1_20180319.pdf" (http://smbus.org/specs/SMBus_3_1_20180319.pdf)
//	document provides (in a somewhat less than entirely clear manner)
//	two levels of information that can be viewed as different levels
//	(or tiers) of any "networking" protocol stack.  These are:
//
//	o	Abstract data format guide lines for the nature of the
//		conversation between devices on an I2C/TWI bus
//
//	o	Hardware specific action sequences dictating the "how"
//		to enables two devices on an I2C/TWI bus to successfully
//		exchange data which was outlined above.
//
//	Of these two items the second is a mandatory element that must be
//	implemented accurately to ensure communication with the widest
//	range of devices.  These routines do this.
//
//	The implementation of the "content" of the communication will be
//	deligated to the software using these routines.
//

//
//	From "SMBus_3_1_20180319.pdf" The following two structural elements
//	have been specifically extracted for encoding within these routines:
//
//	o	Transaction Diagrams detailing the *exact* hardware level
//		exchanges between master and slave devices.
//
//	o	Exchange "modes" which name and capture the various ways
//		a master and a slave can talk.  Each named mechanism will
//		be detailed using Transaction Diagrams.
//
//		The document above lists the following "protocols" that
//		can be used between master and slave devices:
//
//		6.5.1	Quick Commands
//
//		6.5.2	Send Byte
//		6.5.3	Receive Byte
//		6.5.4	Write Byte/Word
//		6.5.5	Read Byte/Word
//
//		6.5.6	Process Call
//
//		6.5.7	Block Write/Read
//		6.5.8	Block Write-Block Read Process Call
//
//		6.5.9	SMBus Host Notify protocol
//
//		6.5.10	Write 32 protocol
//		6.5.11	Read 32 protocol
//		6.5.12	Write 64 protocol
//		6.5.13	Read 64 protocol
//
//	It is worth noting that the I2C/TWI hardware protocol is significantly
//	asymetric in the following ways:
//
//	o	There is a master/slave relationship between two devices
//		communicating though any device can initiate an exchange
//		and will therefore become the master for that exchange.
//
//	o	The nature of the above means that while it is possible for
//		master to transmit a variable amount of data to the slave
//		with the slave clearly being able to determine the end of the
//		data.  It is (apparently) impossible for the master to
//		receive a variable amount of data as there is no underlying
//		hardware based mechanism which enables the slave to say to
//		the master "end of data".
//
//		The consequence of this is that the API is also asymmetric
//		requiring the master to always know, ahead of time, how much
//		data the slave will send back.  This must become a feature
//		of the application level protocol ensuring both master and
//		slave agree on transfer sizes.
//		

//
//	The following data structures capture a simplified view of the
//	above titles and maps them onto its functional implementation.
//
//	Each will be supported by its Transaction Diagram where the following
//	layout and symbols will be used:
//
//		Master:	master transmission components
//		Slave:	slave transmission components
//
//			time >>>--->
//
//	The components of a transmission are:
//
//		S	Start condition
//		T	Repeat Start condition
//		P	Stop condition
//		R	RW bit set for Reading (1)
//		W	RW bit set for Writing (0)
//		A	Acknowledge bit (?)
//		N	Not-Acknowledge bit (?)
//		X	Value Ignored
//
//		a	7 bits of the slave address
//		d	8 data bits
//		^	Repeated section of the diagram
//
//	The following definitions enable the Transaction Diagrams
//	to be directly encoded and used as the basis for the
//	interrupt service routine and supporting service routines.
//
//	As this is being stored in PROGMEM we have to allocate this to
//	specific data type, let us make that a "byte".
//
typedef byte TWI_HW_ACTION;

//
//		Emit start condition (handled by event loop code)
//		Emit a restart condition.
//		Start or restart has completed.
//
#define TWI_HW_START		1	
#define TWI_HW_RESTART		2
#define TWI_HW_START_COMPLETE	3

//
//		Emit a stop condition.
//
#define TWI_HW_STOP		4

//
//		Emit target address with Read.
//		Emit target address with Write.
//		Wait for address Acknowledgement.
//
#define TWI_HW_ADRS_READ	5
#define TWI_HW_ADRS_WRITE	6
#define TWI_HW_ADRS_ACK		7

//
//		Send one byte of data
//		Wait for slave Acknowledgement and...
//		...loop back 1 if data still to send.
//
#define TWI_HW_SEND_BYTE	8
#define TWI_HW_SEND_ACK_LOOP	9

//
//		Tell TWI hardware that we are waiting
//		for data.  Set to use NAck if this will
//		be the last byte, set for Ack if this is
//		not the last byte.
//		The receive action actually gets the data
//		and loops back if the byte was Ackd or
//		rolls to the next action if it was NAckd.
//
#define TWI_HW_RECV_READY	10
#define TWI_HW_RECV_BYTE_LOOP	11

//
//	These two action indicate the "end" of a set of
//	actions and are used to indicate a transaction
//	is complete (with appropriate callback executed).
//
//		Execute successful master callback routine
//		Execute unsuccessful master callback routine
//
//	 Both handled by event loop code.
//
#define TWI_HW_GOOD_CALLBACK	12
#define TWI_HW_FAIL_CALLBACK	13

//
//	The (6.5.1) Quick Commands boil down to one of two possible
//	single bytes transmission from the master to the slave.  This
//	is effectively just the target slave address with the Read
//	bit either 1 (for reading) or 0 (for writing).  No data is read
//	of transmitted.
//
//	TWI_MODE_QUICK_READ
//	-------------------
//	Master:	SaaaaaaaR P
//	Slave:           A
//
//	TWI_MODE_QUICK_WRITE
//	--------------------
//	Master:	SaaaaaaaW P
//	Slave:           A
//
static const TWI_HW_ACTION twi_mode_quick_read[] PROGMEM = {
	TWI_HW_START, TWI_HW_START_COMPLETE,
	TWI_HW_ADRS_READ, TWI_HW_ADRS_ACK,
	TWI_HW_STOP, TWI_HW_GOOD_CALLBACK
};
static const TWI_HW_ACTION twi_mode_quick_write[] PROGMEM = {
	TWI_HW_START, TWI_HW_START_COMPLETE,
	TWI_HW_ADRS_WRITE, TWI_HW_ADRS_ACK,
	TWI_HW_STOP, TWI_HW_GOOD_CALLBACK
};

//
//	All "just send data" commands (6.5.2 Send Byte, 6.5.4 Write
//	Byte/Word, 6.5.7 Block Write/Read, 6.5.10 Write 32 protocol,
//	6.5.12 Write 64 protocol) conform the following underlying
//	hardware exchange:
//
//	TWI_MODE_SEND_DATA
//	------------------
//	Master:	SaaaaaaaW dddddddd P
//	Slave:	         A        A
//	Repeat:	          ^^^^^^^^^
//
static const TWI_HW_ACTION twi_mode_send_data[] PROGMEM = {
	TWI_HW_START, TWI_HW_START_COMPLETE,
	TWI_HW_ADRS_WRITE, TWI_HW_ADRS_ACK,
	TWI_HW_SEND_BYTE, TWI_HW_SEND_ACK_LOOP,
	TWI_HW_STOP, TWI_HW_GOOD_CALLBACK
};
//
//	The (6.5.3) Receive Byte is the only command which can directly
//	obtain any data form the slave *without* first asking for
//	something:
//
//	TWI_MODE_RECEIVE_BYTE
//	---------------------
//	Master:	SaaaaaaaR         NP
//	Slave:	         Adddddddd
//
static const TWI_HW_ACTION twi_mode_receive_byte[] PROGMEM = {
	TWI_HW_START, TWI_HW_START_COMPLETE,
	TWI_HW_ADRS_READ, TWI_HW_ADRS_ACK,
	TWI_HW_RECV_READY, TWI_HW_RECV_BYTE_LOOP,
	TWI_HW_STOP, TWI_HW_GOOD_CALLBACK
};
//
//	All of the remaining commands (6.5.4 Write Byte/Word, 6.5.5
//	Read Byte/Word, 6.5.6 Process Call, 6.5.8 Block Write-Block
//	Read Process Call, 6.5.9 SMBus Host Notify protocol, 6.5.11
//	Read 32 protocol, 6.5.13 Read 64 protocol) conform to a common
//	data exchange protocol:
//
//	TWI_MODE_DATA_EXCHANGE
//	----------------------
//	Master:	SaaaaaaaW dddddddd TSaaaaaaaR         A          NP
//	Slave:	         A        A          Adddddddd   dddddddd
//	Repeat:	          ^^^^^^^^^           ^^^^^^^^^
//
static const TWI_HW_ACTION twi_mode_data_exchange[] PROGMEM = {
	TWI_HW_START, TWI_HW_START_COMPLETE,
	TWI_HW_ADRS_WRITE, TWI_HW_ADRS_ACK,
	TWI_HW_SEND_BYTE, TWI_HW_SEND_ACK_LOOP,
	TWI_HW_RESTART, TWI_HW_START_COMPLETE,
	TWI_HW_ADRS_READ, TWI_HW_ADRS_ACK,
	TWI_HW_RECV_READY, TWI_HW_RECV_BYTE_LOOP,
	TWI_HW_STOP, TWI_HW_GOOD_CALLBACK
};

//
//	twi_abortTransaction
//	--------------------
//
//	This incomplete sequence is not applied in the first instance
//	but is used to "redirect" a transaction which has failed and
//	needs to completed with subsequent tidy up.
//
static const TWI_HW_ACTION twi_abortTransaction[] PROGMEM = {
	TWI_HW_STOP, TWI_HW_FAIL_CALLBACK
};

//
//	Define a data structure used to queue pending
//	exchange requests initiated from this MCU as a
//	master device.
//
#define TWI_TRANSACTION struct twi_transaction
TWI_TRANSACTION {
	//
	//	The list of actions this needs to implement, and the
	//	reset address if the transaction needs to be restarted
	//	mid flight.
	//
	const TWI_HW_ACTION
			*action;
	//
	//	Who are we trying to exchange data with?
	//
	byte		target;
	//
	//	The address of the buffer space to be used as
	//	the source data and repository for the subsequent
	//	reply.  Next forms a pointer used to access the
	//	buffer space.
	//
	byte		*buffer;
	//
	//	The next field is used to "step through" the buffer
	//	when either sending or receiving data.
	//
	byte		next;
	//
	//	The number of bytes in the buffer space which form
	//	the data to be sent from the master to the slave
	//
	byte		send;
	//
	//	The number of bytes expected back from the slave
	//	which will be placed into the buffer
	//
	byte		recv;
	//
	//	An anonymous pointer available to the calling firmware to be
	//	passed into the reply function when it is called.
	//
	void		*link;
	//
	//	The address of the routine (if not NULL) which is called
	//	when a reply is received.
	//
	void		FUNC( reply )( bool valid, void *link, byte *buffer, byte len );
};

//
//	Define the size of the pending queue implemented by these functions.
//
#define TWI_MAX_QUEUE_LEN	4

//
//	Define the pending exchange queue and indexes into it.
//
static TWI_TRANSACTION	twi_queue[ TWI_MAX_QUEUE_LEN ];	// This is the queue
static byte		twi_queue_len,			// number of queued exchanges
			twi_queue_in,			// index of next free slot
			twi_queue_out;			// index of next pending exchange

//
//	The following pointer is used by the ISR
//	as the pointer to the "active" exchange
//	record.
//
static TWI_TRANSACTION	*twi_active;

//
//	Handling timeout of activities
//
//	These three variables manage the timeout of actions
//	in the following way:
//
//	twi_has_timeout		true if there is a timeout
//				to consider.
//
//	twi_start_time		This is the time (in Micorseconds)
//				at which the timeout STARTED
//
//	twi_timeout_delay	This is the number of Microseconds
//				the action has before it will be timed
//				out.
//
static bool		twi_has_timeout;
static unsigned long	twi_start_time;
static word		twi_timeout_delay;

//
//	TWI Interrupt enable.
//
//	The content of this variable is used to enable the
//	interrupt calling the ISR when the TWINT flag is
//	asserted by the TWI Hardware.
//
//	This variable has only two possible values:
//
//		0		Interrupt DISABLED
//
//		bit( TWIE )	Interrupt ENABLED (default)
//
static byte		twi_enable_interrupt;

//
//	TWI Slave enable
//
//	The content of this variable is used to enable the
//	acceptance of slave connections placing the TWI
//	hardware into either SR or ST modes.
//
//	This variable has only two possible values:
//
//		0		Slave modes DISABLED
//
//		bit( TWEA )	Slave mode ENABLED
//
static byte		twi_enable_slave;

//
//	This variable will contain the value of the last
//	error condition that the code detected.  This will
//	be appropriate when the reply() function is called
//	with valid set to false.
//
byte			twi_error;

//
//	The following variables are used to handle these functions
//	responding to a master exchange "as the slave device".  There
//	is no queue to speak of in this case, we simply need to keep
//	note of a buffer space we can use and a routine to call
//	to handle the data sent to the MCU.
//
//	twi_slave_active	Boolean is true if slave transaction in
//				progress.
//
//	twi_slave_buffer	The address of the buffer space.
//
//	twi_slave_adrsd		The address which the slave is currently
//				responding to.
//
//	twi_slave_size		The total size of the buffer space.
//
//	twi_slave_len		How much of the buffer space has been used
//				when either receiving data or setting data
//				to be sent.
//
//	twi_slave_send		The number of bytes in the buffer to send.
//
//	twi_slave_answer	The call back routine used to generate a
//				reply to the slave request.
//
static bool		twi_slave_active;
static byte		*twi_slave_buffer,
			twi_slave_adrsd,
			twi_slave_size,
			twi_slave_len,
			twi_slave_send,
			FUNC( twi_slave_answer )( byte adrs, byte *buffer, byte size, byte len );

//
//	Finally the pointer to the error reporting routine, if set.
//
static void	FUNC( twi_reportError )( byte error );


//
//	Basic initialisation and configuration routines.
//	================================================
//

//
//	The following PROGMEM table facilitates the implementation of
//	bit rate configuration.
//
//	Note: the TWI clock speed is calculated from:
//
//		TWI_CLOCK = F_CPU / ( 16 + 2 * TWBR * 4 ^ TWPS )
//
//	Re-arranged to give TWBR:
//
//		 TWBR = ( F_CPU / TWI_CLOCK - 16 ) / ( 2 * 4 ^ TWPS )
//
//	All divisions are rounded down (floor'd) to the next lowest integer.
//
//	A table is used as this removes the requirement for the
//	mathematics library, and also allows for a (more) simplified
//	setting of the scale factor at the same time.
//
//	As outlined in the "TWI_IO.h" file the association between the
//	actual clock speed and the representation of it in the software
//	does not have to have a mathematical basis, but for the moment
//	it does.
//
#define TWI_BITRATE struct twi_bitrate
TWI_BITRATE {
	byte	freq,		// freq / 10Khz
		twbr,		// Range 10 .. 255
		twps;		// Range 0 .. 3
};
//
//	The order of the entries in this table is important; they
//	must be in sorted freq order, largest first.  The
//	routine will always return the result which is the speed
//	requested or nearest (but slower) speed available.
//
//	All speeds which require the TWBR to be less than 10
//	are explicitly excluded as these are (allegedly) too fast
//	for the MCU to support reliably.
//
static const TWI_BITRATE twi_bitrates[] PROGMEM = {
#if	F_CPU == 16000000
	{	40,	12,	0	},
	{	35,	14,	0	},
	{	30,	18,	0	},
	{	25,	24,	0	},
	{	20,	32,	0	},
	{	15,	45,	0	},
	{	10,	72,	0	},
	{	9,	80,	0	},
	{	8,	92,	0	},
	{	7,	106,	0	},
	{	6,	125,	0	},
	{	5,	38,	1	},
	{	4,	48,	1	},
	{	3,	64,	1	},
	{	2,	98,	1	},
	{	1,	49,	2	},
	
#elif	F_CPU == 12000000
	{	30,	12,	0	},
	{	25,	16,	0	},
	{	20,	22,	0	},
	{	15,	32,	0	},
	{	10,	52,	0	},
	{	9,	58,	0	},
	{	8,	67,	0	},
	{	7,	77,	0	},
	{	6,	92,	0	},
	{	5,	112,	0	},
	{	4,	35,	1	},
	{	3,	48,	1	},
	{	2,	73,	1	},
	{	1,	37,	2	},
	
#elif	F_CPU == 8000000
	{	20,	12,	0	},
	{	15,	18,	0	},
	{	10,	32,	0	},
	{	9,	36,	0	},
	{	8,	42,	0	},
	{	7,	49,	0	},
	{	6,	58,	0	},
	{	5,	72,	0	},
	{	4,	92,	0	},
	{	3,	125,	0	},
	{	2,	48,	1	},
	{	1,	98,	1	},
	
#elif	F_CPU == 4000000
	{	10,	12,	0	},
	{	9,	14,	0	},
	{	8,	17,	0	},
	{	7,	20,	0	},
	{	6,	25,	0	},
	{	5,	32,	0	},
	{	4,	42,	0	},
	{	3,	58,	0	},
	{	2,	92,	0	},
	{	1,	48,	1	},
	
#elif	F_CPU == 2000000
	{	5,	12,	0	},
	{	4,	17,	0	},
	{	3,	25,	0	},
	{	2,	42,	0	},
	{	1,	92,	0	},
	
#elif	F_CPU == 1000000
	{	2,	17,	0	},
	{	1,	42,	0	},
#else
#error "Unrecognised CPU Clock Speed"
#endif
	//
	//	End of list
	//
	{	0,	0,	0	}
};

//
//	void twi_setFrequency( byte freq )
//	----------------------------------
//
//	Set the TWI Clock frequency by setting the TWBR and
//	TWPS (TWSR 1:0) registers.
//
//	The frequency is (currently) passed in as a multiple
//	of 10KHz (i.e. if freq == 25 then clock = 250,000 Hz)
//
//	There is an assumption, a reasonable one I believe, that
//	the table that this routine accesses contains at least ONE line
//	for a valid TWI/I2C clock rate.  Failure to ensure this will
//	render any use of the TWI hardware pretty much impossible.
//
void twi_setFrequency( byte freq ) {
	const TWI_BITRATE	*look;
	byte			found,
				twbr,
				twps;

	twbr = 0;
	twps = 0;
	look = twi_bitrates;
	while(( found = pgm_read_byte( &( look->freq )))) {
		twbr = pgm_read_byte( &( look->twbr ));
		twps = pgm_read_byte( &( look->twps ));
		if( found <= freq ) break;
		look++;
	}
	TWSR = twps;
	TWBR = twbr;
}

//
//	byte twi_bestFrequency( byte freq )
//	-----------------------------------
//
//	Return the identity of the best available
//	clock speed at (or below) the initial value
//	passed in.
//
byte twi_bestFrequency( byte freq ) {
	const TWI_BITRATE	*look;
	byte			found;

	look = twi_bitrates;
	while(( found = pgm_read_byte( &( look->freq )))) {
		if( found <= freq ) break;
		look++;
	}
	return( found );
}

//
//	Return the identity of the slowest available
//	clock speed.  This could be useful when a master
//	and a slave negotiate the best available
//	common speed .. you need to start from somewhere.
//
//	This *should* always come out as "1", and (currently)
//	reflect a speed of 10Kbps
//
byte twi_lowestFrequency( void ) {
	const TWI_BITRATE	*look;
	byte			found,
				last;

	look = twi_bitrates;
	last = TWI_MAXIMUM_FREQ;
	while(( found = pgm_read_byte( &( look->freq )))) {
		last = found;
		look++;
	}
	return( last );
}

//
//	void twi_init( byte adrs, bool gcall, bool isr, bool pullup = true )
//	--------------------------------------------------------------------
//
//	Call this routine in the setup() function.
//
void twi_init( byte adrs, bool gcall, bool isr, bool pullup ) {
	//
	//	Initialise default timeout.
	//
	twi_has_timeout = false;
	twi_start_time = 0;
	twi_timeout_delay = 0;
	
	//
	//	Prepare the queue as empty.
	//
	twi_queue_len = 0;
	twi_queue_in = 0;
	twi_queue_out = 0;

	//
	//	Initially there is no active exchange.
	//
	twi_active = NULL;

	//
	//	Initialise the Slave transaction details
	//	to empty.
	//
	twi_slave_active = false;
	twi_slave_adrsd = 0;
	twi_slave_buffer = NULL;
	twi_slave_size = 0;
	twi_slave_len = 0;
	twi_slave_answer = NULL;

	//
	//	Are we using an ISR routine?  If "isr" is set then
	//	we set "twi_enable_interrupt" to "bit( TWIE )", if not
	//	then "twi_enable_interrupt" is zero.
	//
	twi_enable_interrupt = isr? bit( TWIE ): 0;

	//
	//	Initialise the Slave configuration
	//
	//	If the address provided is non-zero then apply it to
	//	the TWAR register and set the GCall bit is required.
	//
	//	If not then set everything to zero.
	//
	if( adrs ) {
		//
		//	Set TWI slave address (skip over TWGCE bit)
		//
		TWAR = ( adrs << 1 )|( gcall? 1: 0 );
		twi_enable_slave = bit( TWEA );
	}
	else {
		TWAR = 0;
		twi_enable_slave = 0;
	}

	//
	//	set no error reporting routine.
	//
	twi_reportError = NULL;

	//
	//	Optionally enable internal pull-up resistors required
	//	for TWI protocol.
	//
	pinMode( SDA, ( pullup? INPUT_PULLUP: INPUT ));
	pinMode( SCL, ( pullup? INPUT_PULLUP: INPUT ));

	//
	//	Initialise TWI pre-scaler and default bit rate.
	//
	twi_setFrequency( TWI_FREQ );

	//
	//	Enable TWI module, TWI interrupt and slave mode.
	//
	TWCR = twi_enable_interrupt | twi_enable_slave | bit( TWEN );
}

//
//	void twi_disable( void )
//	------------------------
//
//	Disables the TWI interface completely.
//
void twi_disable( void ) {
	//
	//	Disable TWI module, TWI interrupt and Slave mode.
	//
	TWCR &= ~( bit( TWEN ) | bit( TWIE ) | bit( TWEA ));

	//
	//	Deactivate internal pull-up resistors for TWI.
	//
	pinMode( SDA, INPUT );
	pinMode( SCL, INPUT );
}

//
//	void twi_setTimeout( word ms )
//	------------------------------
//
//	Set the maximum time the software will wait for some
//	action to complete before aborting the activity.
//	Specified in microseconds.
//
void twi_setTimeout( word us ) {
	if( us ) {
		twi_has_timeout = true;
		twi_timeout_delay = us;
	}
	else {
		twi_has_timeout = false;
		twi_timeout_delay = 0;
	}
}

//
//	void twi_errorReporting( void FUNC( report )( byte error ))
//	----------------------------------------------------------
//
//	Provide an API for the users of the software to
//	collect error numbers when the routines detect an
//	error.
//
//	This does not have to be set, but if set the routine
//	called should execute quickly and on the assumption
//	that it *might* be called from within an ISR.
//
void twi_errorReporting( void FUNC( report )( byte error )) {
	twi_reportError = report;
}

//
//	void twi_sendError( byte error )
//	--------------------------------
//
//	This is the routine called by these routines to handle
//	reporting of problems.  Error recovery is handled by the
//	routines themselves, this is purely for reporting purposes.
//
static void twi_sendError( byte error ) {
	if( twi_reportError ) FUNC( twi_reportError )( error );
}


//
//	bool twi_queueTransaction( const TWI_HW_ACTION *action, byte address, byte *buffer, byte send, byte recv, void *link,
//	---------------------------------------------------------------------------------------------------------------------
//						void FUNC( reply )( bool valid, void *link, byte *buffer, byte len ))
//						---------------------------------------------------------------------
//
//	Initiates an asynchronous data exchange with a specified slave device, with the parameters supplying
//	the required details:
//
//		action		The list of actions which implement this transaction
//		address		The slave address being targeted
//		buffer		The address of a byte array where sent and returned data will be stored.
//		send		The number of bytes to send from the buffer area
//		recv		The number of bytes which will be returned into the buffer area
//		link		Abstract pointer to be passed into the called reply routine
//		reply(		The function to call when the reply from the slave is obtained
//			valid	The slave exchange succeeded, or not
//			link	The link pointer passed in above
//			buffer	The start address of the reply
//			len)	The number of bytes in the reply
//
//	Returns true if the exchange has been successfully queued, false otherwise.
//
//	Note:	The buffer space address passed into this routine is kept and accessed
//		asynchronously from the main flow of the program.  To this end the area
//		must be allocated in such a way that it remains valid from the point of
//		calling this routine to the point at which the reply routine (when called)
//		returns.
//
//		Also note that if multiple exchange requests are queued, then each must
//		have its own distinct buffer space to avoid data being over written.
//
//	The API reserves the right to implement a pending exchange queue of arbitrary length
//	from zero entries.  The implication is that with a queue of length zero all
//	exchange requests will fail if any exchange is currently taking place.  The code
//	must be prepared for this possibility as the same situation would apply with any
//	size queue which has been filled with pending requests.
//
static bool twi_queueTransaction( const TWI_HW_ACTION *action, byte address, byte *buffer, byte send, byte recv, void *link, void FUNC( reply )( bool valid, void *link, byte *buffer, byte len )) {
	TWI_TRANSACTION	*ptr;
	
	if( twi_queue_len >= TWI_MAX_QUEUE_LEN ) {
		twi_sendError( TWI_ERR_QUEUE_FULL );
		return( false );
	}
	//
	//	Locate the next available slot and adjust queue
	//	appropiately.
	//
	ptr = &( twi_queue[ twi_queue_in++ ]);
	if( twi_queue_in >= TWI_MAX_QUEUE_LEN ) twi_queue_in = 0;
	//
	//	"ptr" is the address of our queue record, so we can now
	//	fill in this record with the supplied details.
	//
	ptr->action = action;
	ptr->target = address;
	ptr->buffer = buffer;
	ptr->next = 0;		// Also set up by the START and RESTART actions
	ptr->send = send;
	ptr->recv = recv;
	ptr->link = link;
	ptr->reply = reply;
	//
	//	Last action, increase the queue length.
	//
	twi_queue_len++;
	return( true );
}

//
//	Declare the Master API
//	======================
//
//	All master API routines return true if the I2C/TWI action
//	has been successfully scheduled or false otherwise.
//

//
//	The (6.5.1) Quick Commands
//	--------------------------
//
bool twi_cmd_quick_read( byte adrs, void *link, void FUNC( reply )( bool valid, void *link, byte *buffer, byte len )) {
	return( twi_queueTransaction( twi_mode_quick_read, adrs, NULL, 0, 0, link, reply ));
}

bool twi_cmd_quick_write( byte adrs, void *link, void FUNC( reply )( bool valid, void *link, byte *buffer, byte len )) {
	return( twi_queueTransaction( twi_mode_quick_write, adrs, NULL, 0, 0, link, reply ));
}

//
//	All "just send data" commands
//	-----------------------------
//	(6.5.2) Send Byte
//	(6.5.4) Write Byte/Word
//	(6.5.7) Block Write/Read
//	(6.5.10) Write 32 protocol
//	(6.5.12) Write 64 protocol
//
bool twi_cmd_send_data( byte adrs, byte *buffer, byte send, void *link, void FUNC( reply )( bool valid, void *link, byte *buffer, byte len )) {
	return( twi_queueTransaction( twi_mode_send_data, adrs, buffer, send, 0, link, reply ));
}

//
//	The (6.5.3) Receive Byte
//	------------------------
//
bool twi_cmd_receive_byte( byte adrs, byte *buffer, void *link, void FUNC( reply )( bool valid, void *link, byte *buffer, byte len )) {
	return( twi_queueTransaction( twi_mode_receive_byte, adrs, buffer, 0, 1, link, reply ));
}

//
//	All "exchange data" commands
//	----------------------------
//	(6.5.4) Write Byte/Word
//	(6.5.5) Read Byte/Word
//	(6.5.6) Process Call
//	(6.5.8) Block Write-Block Read Process Call
//	(6.5.9) SMBus Host Notify protocol
//	(6.5.11) Read 32 protocol
//	(6.5.13) Read 64 protocol
//
bool twi_cmd_exchange( byte adrs, byte *buffer, byte send, byte recv, void *link, void FUNC( reply )( bool valid, void *link, byte *buffer, byte len )) {
	return( twi_queueTransaction( twi_mode_data_exchange, adrs, buffer, send, recv, link, reply ));
}

//
//	Declare the Slave API
//	=====================
//

//
//	void twi_slaveFunction( byte *buffer, byte size, byte FUNC( answer )( byte *buffer, byte size, byte len ))
//	----------------------------------------------------------------------------------------------------------
//
//	This routine primes the "slave answering a master" exchange request with both
//	buffer space (for the in-coming request and out-going reply) and the reply routine
//	which is called to generate the necessary reply content.
//
//		buffer		The address of a byte array where received and returned data will be stored
//		size		The number of bytes in the buffer area
//		answer(		The function to call when a master starts an exchange with us
//			buffer	The buffer space where the data sent is placed
//			size	The over all, maximum size of the buffer
//			len)	The number of bytes in the buffer making up the master request
//
//	The answer() function returns the number of bytes in the buffer containing the reply to
//	send back to the master.
//
void twi_slaveFunction( byte *buffer, byte size, byte FUNC( answer )( byte adrs, byte *buffer, byte size, byte len )) {
	//
	//	The value of these parameters is critical, any error in them
	//	will cause difficult to trace issues.
	//
	//	Having said that, they are not complicated.  Make sure they
	//	are all valid, or are all zero or null.
	//
	twi_slave_buffer = buffer;
	twi_slave_size = size;
	twi_slave_len = 0;
	twi_slave_answer = answer;
}

//
//	Supporting Routines
//	===================
//

//
//	byte twi_queueLength( void )
//	----------------------------
//
//	Return the number of pending exchange requests.  This (by the
//	way the software works) includes the active transmission.
//
byte twi_queueLength( void ) {
	return( twi_queue_len );
}

//
//	void twi_clearQueue( void )
//	---------------------------
//
//	Delete any queued (but inactive) exchange requests.
//
void twi_clearQueue( void ) {
	if( twi_active ) {
		//
		//	Something is being processed, leave this alone
		//	to complete, but kill off the rest.
		//
		twi_queue_len = 1;
		if(( twi_queue_in = twi_queue_out+1 ) >= TWI_MAX_QUEUE_LEN ) twi_queue_in = 0;
	}
	else {
		//
		//	Easy, just forget everything!
		//
		twi_queue_len = 0;
		twi_queue_in = 0;
		twi_queue_out = 0;
	}
}

//
//	void twi_synchronise( void )
//	----------------------------
//
//	Pause the program until all pending master transactions have
//	completed.
//
void twi_synchronise( void ) {
	//
	//	Exit only when the queu is empty
	//
	while( twi_queueLength()) twi_eventProcessing();
}

//
//	Queue manipulaton routines
//	==========================
//

//
//	void twi_dropCurrentAction( void )
//	----------------------------------
//
//	Routine to capture the dropping of the current master
//	transaction and returning it to the pending queue as
//	a free record.
//
static void twi_dropCurrentAction( void ) {
	if( twi_active ) {
		//
		//	Need to protect the update of twi_active
		//	from possible "mid update" interrupt
		//
		if( twi_enable_interrupt ) noInterrupts();
		twi_active = NULL;
		twi_queue_len -= 1;
		if(( twi_queue_out += 1 ) >= TWI_MAX_QUEUE_LEN ) twi_queue_out = 0;
		if( twi_enable_interrupt ) interrupts();
	}
}

//
//	void twi_loadNextAction( void )
//	----------------------------------
//
//	Routine to capture the dropping of the current master
//	transaction and returning it to the pending queue as
//	a free record.
//
static void twi_loadNextAction( void ) {
	//
	//	Need to protect the update of twi_active
	//	from possible "mid update" interrupt
	//
	if( twi_enable_interrupt ) noInterrupts();
	//
	//	Are we replacing a now finished job?
	//
	if( twi_active ) {
		//
		//	Yes.
		//
		twi_queue_len -= 1;
		if(( twi_queue_out += 1 ) >= TWI_MAX_QUEUE_LEN ) twi_queue_out = 0;
	}
	//
	//	Is there anything left in the queue?
	//
	if( twi_queue_len ) {
		//
		//	Yes.
		twi_active = &( twi_queue[ twi_queue_out ]);
	}
	else {
		//
		//	No.
		//
		twi_active = NULL;
	}
	if( twi_enable_interrupt ) interrupts();
}

//
//	Two Wire Hardware interface routines.
//	=====================================
//

//
//	The following TWI "primitive" operations are based on the
//	content of the document:
//
//		ATmega48A/PA/88A/PA/168A/PA/328/P
//		megaAVR ® Data Sheet
//
//	Produced by Microchip, downloaded from:
//
//	https://ww1.microchip.com/downloads/en/DeviceDoc/ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061B.pdf
//

//
//	void twi_start( void )
//	------------------------
//
//	Send start condition (also doubles up as a restart)
//
static inline void twi_start( void ) {
	//
	//	TWI Control Register, TWCR
	//
	//	Bit	Value	Meaning
	//
	//	TWINT	1	Initiate action (resets bit to 0)
	//	TWEA	slave	TWI Slave enable / (N)Ack reply
	//	TWSTA	1	Transmit Start condition
	//	TWSTO	0	Transmit Stop condition
	//	TWWC	X	TWDR Write Collision detected
	//	TWEN	1	TWI Hardware enable
	//	TWIE	int	TWI Interrupt enable
	//
	TWCR = twi_enable_slave | twi_enable_interrupt |( bit( TWEN ) | bit( TWSTA ) | bit( TWINT ));
}

//
//	bool twi_actionComplete( void )
//	-------------------------------
//
//	Checks the value of the Initiate flag (TWINT) which will
//	return to 1 when the previous activity has been completed.
//
static inline bool twi_actionComplete( void ) {
	//
	//	TWCR	TWI Control Register
	//
	//	TWINT	Action complete flag.
	//
	//		Returns 0 while an action is in
	//		progress, and becomes 1 when the
	//		action has completed and the TWI
	//		is ready for the next action.
	//
	return( TWCR & bit( TWINT ));
}

//
//	void twi_sendByte( byte data )
//	------------------------------
//
//	Transmit a byte of data
//
static inline void twi_sendByte( byte data ) {
	//
	//	TWI data IO register, TWDR
	//
	//	TWI Control Register, TWCR
	//
	//	Bit	Value	Meaning
	//
	//	TWINT	1	Initiate action (resets bit to 0)
	//	TWEA	slave	TWI Slave enable / (N)Ack reply
	//	TWSTA	0	Transmit Start condition
	//	TWSTO	0	Transmit Stop condition
	//	TWWC	X	TWDR Write Collision detected
	//	TWEN	1	TWI Hardware enable
	//	TWIE	int	TWI Interrupt enable
	//
	TWDR = data;
	TWCR = twi_enable_slave | twi_enable_interrupt |( bit( TWEN ) | bit( TWINT ));
}

//
//	void twi_sendTarget( byte adrs, bool writing )
//	----------------------------------------------
//
//	Send the address byte of the slave being targetted by
//	this transmission. The argument writing indicates if
//	the following transmission involves the master writing
//	to the slave (true) or readin from thw slave (false).
//
static void twi_sendTarget( byte adrs, bool writing ) {
	twi_sendByte(( adrs << 1 )|( writing? TW_WRITE: TW_READ ));
}

//
//	void twi_readAck( bool ack )
//	----------------------------
//
//	This routine is only about sending an Acknowledgement
//	or NOT Acknowledgement as a result of a byte being
//	delivered to us.
//
static inline void twi_readAck( bool ack ) {
	//
	//	TWI Control Register, TWCR
	//
	//	Bit	Value	Meaning
	//
	//	TWINT	1	Initiate action (resets bit to 0)
	//	TWEA	ack	TWI Slave enable / (N)Ack reply
	//	TWSTA	0	Transmit Start condition
	//	TWSTO	0	Transmit Stop condition
	//	TWWC	X	TWDR Write Collision detected
	//	TWEN	1	TWI Hardware enable
	//	TWIE	int	TWI Interrupt enable
	//
	TWCR = twi_enable_interrupt |( ack? bit( TWEA ): 0 )|( bit( TWEN ) | bit( TWINT ));
}

//
//	byte twi_readByte( void )
//	-------------------------
//
//	Return the byte which hsa just been delivered to the system
//
static inline byte twi_readByte( void ) {
	//
	//	TWI Data Register, TWDR
	//
	return( TWDR );
}

//
//	void twi_stop( void )
//	---------------------
//
//	Tell the remote slave device that we are done, and
//	release the bus for other devices to use.
//
static inline void twi_stop( void ) {
	//
	//	TWI Control Register, TWCR
	//
	//	Bit	Value	Meaning
	//
	//	TWINT	1	Initiate action (resets bit to 0)
	//	TWEA	X	TWI Slave enable / (N)Ack reply
	//	TWSTA	0	Transmit Start condition
	//	TWSTO	1	Transmit Stop condition
	//	TWWC	X	TWDR Write Collision detected
	//	TWEN	1	TWI Hardware enable
	//	TWIE	X	TWI Interrupt enable
	//
	TWCR = twi_enable_slave | twi_enable_interrupt |( bit( TWEN ) | bit( TWSTO ) | bit( TWINT ));
}

//
//	byte twi_slaveAddress( void )
//	-----------------------------
//
//	Return the slave address as found in TWDR
//
static inline byte twi_slaveAddress( void ) {
	return( TWDR >> 1 );
}

//
//	void twi_clearBus( void )
//	-------------------------
//
//	Return the TWI Bus to a "quiessed" state ready for the next
//	transmission.
//
static inline void twi_clearBus( void ) {
	//
	//	TWI Control Register, TWCR
	//
	//	Bit	Value	Meaning
	//
	//	TWINT	1	Initiate action (resets bit to 0)
	//	TWEA	X	TWI Slave enable / (N)Ack reply
	//	TWSTA	0	Transmit Start condition
	//	TWSTO	0	Transmit Stop condition
	//	TWWC	X	TWDR Write Collision detected
	//	TWEN	1	TWI Hardware enable
	//	TWIE	X	TWI Interrupt enable
	//
	TWCR = twi_enable_slave | twi_enable_interrupt |( bit( TWEN ) | bit( TWINT ));
}

//
//	void twi_resetHardware( void )
//	------------------------------
//
//	This is a final "let us try power off then on" the TWI
//	hardware to regain control of the system.
//
static void twi_resetHardware( void ) {
	byte	twcr;
	
	//
	//	We are only focused on the hardware, we will not
	//	change any firmware settings.
	//
	//	Save current value.
	//
	twcr = TWCR;
	//
	//	Clear the control register to reset hardware.
	//
	TWCR = 0;
	//
	//	Report hardware reset
	//
	twi_reportError( TWI_ERR_HW_RESET );
	//
	//	Pause to allow hardware to complete
	//	power down.
	//
#if TWI_HARDWARE_RESET_DELAY_US > 0
	delayMicroseconds( TWI_HARDWARE_RESET_DELAY_US );
#endif
	//
	//	Now put the control register back, but without
	//	a TWSTA, TWSTO or TWINT set.
	//
	TWCR = twcr & ~( TWSTA | TWSTO | TWINT );
}

//
//	byte twi_state( void )
//	---------------------------
//
static inline byte twi_state( void ) {
	return( TWSR & 0xf8 );
}

//
//	The Master Machine Logic Routine
//	================================
//

//
//	void twi_masterMachine( TWI_TRANSACTION *active, byte twsr )
//	------------------------------------------------------------
//
//	Using the TWI state supplied drive the transaction record forward.
//	When the routine returns something will have been done.
//
static void twi_masterMachine( TWI_TRANSACTION *active, byte twsr ) {
	//
	//	This routine is always handed a TWI state obtained after
	//	TWINT becomes 1 (either through being called by the ISR
	//	or though polling the TWINT flag).
	//
	//	The purpose of this routine is to handle the new state
	//	by moving on the master transaction machine forward.
	//
masterLoop:
	switch( pgm_read_byte( active->action )) {
		case TWI_HW_RESTART: {
			//
			//	Generate a restart condition and move the
			//	action pointer forward to the next step.
			//
			twi_start();
			active->next = 0;	// Reset index into the buffer
			active->action++;
			break;
		}
		case TWI_HW_START_COMPLETE: {
			//
			//	We are waiting for the start (or restart) to complete successfully.
			//	If it has move to the next step in the machine.  If not then redirect
			//	machine to the abort transaction code.
			//
			switch( twsr ) {
				case TW_START:
				case TW_REP_START: {
					//
					//	Move to next step in the machine.
					//
					active->action++;
					break;
				}
				default: {
					//
					//	Not a state we are anticipating, abort transaction
					//
					twi_error = twsr;
					active->action = twi_abortTransaction;
					twi_sendError( TWI_ERR_STARTING );
				}
			}
			//
			//	We have not "done" anything so we go back to the start of the
			//	routine and read the next step in the machine.
			//
			goto masterLoop;
		}
		case TWI_HW_STOP: {
			//
			//	Generate a stop condition and move the
			//	action pointer forward to the next step.
			//
			twi_stop();
			active->action++;
			break;
		}
		case TWI_HW_ADRS_READ: {
			//
			//	We now send out the slave address from
			//	which we want to read data.
			//
			//	If we are only getting a single byte, then
			//	the Ack flag needs to be false, otherwise
			//	we set it to true.
			//
			twi_sendTarget( active->target, false );
			active->action++;
			break;
		}
		case TWI_HW_ADRS_WRITE: {
			//
			//	We now send out the slave address with
			//	WRITE signified.
			//
			twi_sendTarget( active->target, true );
			active->action++;
			break;
		}
		case TWI_HW_ADRS_ACK: {
			//
			//	We are waiting for the address byte to
			//	complete, then we check the status to
			//	see if we were ack'd or nack'd by the slave.
			//
			switch( twsr ) {
				case TW_MT_SLA_ACK:
				case TW_MR_SLA_ACK: {
					//
					//	The slave has Ackd their address byte
					//	for either read or write (this step in the
					//	master machine does not care which).  Move
					//	to the next step in the machine.
					//
					active->action++;
					break;
				}
				case TW_MT_SLA_NACK:
				case TW_MR_SLA_NACK: {
					//
					//	The slave has NOT Ackd their address byte, so
					//	log an error and then move to the next step in
					//	the machine.
					//
					twi_error = twsr;
					active->action = twi_abortTransaction;
					twi_sendError( TWI_ERR_ADDRESS );
					break;
				}
				default: {
					//
					//	..and again, but a different error.
					//
					twi_error = twsr;
					active->action = twi_abortTransaction;
					twi_sendError( TWI_ERR_TRANSACTION );
					break;
				}
			}
			goto masterLoop;
		}
		case TWI_HW_SEND_BYTE: {
			//
			//	We have to send a byte to the slave
			//
			twi_sendByte( active->buffer[ active->next++ ]);
			active->action++;
			break;
		}
		case TWI_HW_SEND_ACK_LOOP: {
			//
			//	Has the data been received?
			//
			switch( twsr ) {
				case TW_MT_DATA_ACK: {
					//
					//	The sent data byte has been Ackd.  If there
					//	are more data bytes to send go BACK one step,
					//	otherwise move forwards to the next step.
					//
					if( active->next < active->send ) {
						active->action--;
					}
					else {
						active->action++;
					}
					break;
				}
				case TW_MT_DATA_NACK: {
					//
					//	The data byte was NAckd, the writ has failed.
					//
					twi_error = twsr;
					active->action = twi_abortTransaction;
					twi_sendError( TWI_ERR_WRITE_FAIL );
					break;
				}
				default: {
					//
					//	Other state, generate another error.
					//
					twi_error = twsr;
					active->action = twi_abortTransaction;
					twi_sendError( TWI_ERR_TRANSACTION );
				}
			}
			goto masterLoop;
		}
		case TWI_HW_RECV_READY: {
			//
			//	Here we let the system know that we are
			//	ready to receive another byte from the
			//	slave and (through Ack/NAck) if this will
			//	be the last byte.
			//
			twi_readAck( active->next < ( active->recv-1 ));
			active->action++;
			break;
		}
		case TWI_HW_RECV_BYTE_LOOP: {
			//
			//	Save that byte of data!
			//
			active->buffer[ active->next++ ] = twi_readByte();
			//
			//	Now choose what to do.
			//
			switch( twsr ) {
				case TW_MR_DATA_ACK: {
					//
					//	We have a byte of data, the system
					//	has already sent an "Ack" (so this
					//	is not the last byte).  We roll
					//	back to the previous action.
					//
					active->action--;
					break;
				}
				case TW_MR_DATA_NACK: {
					//
					//	We have a byte of data, the system
					//	has already sent an "NAck" (so this
					//	IS the last byte).  Move onto the
					//	next action.
					//
					active->action++;
					break;
				}
				default: {
					//
					//	Some sort of transaction error.
					//
					twi_error = twsr;
					active->action = twi_abortTransaction;
					twi_sendError( TWI_ERR_TRANSACTION );
					break;
				}
			}
			goto masterLoop;
		}
		default: {
			//
			//	Anything we do not recognise we take to
			//	be handled by another routine, and so our
			//	roll here is to do nothing
			//
			break;
		}
	}
}

//
//	void twi_slaveMachine( byte twsr )
//	----------------------------------
//
//	Drive the state of the slave machine forward.
//
static void twi_slaveMachine( byte twsr ) {
	//
	//	This routine is always handed a TWI state obtained after
	//	TWINT becomes 1 (either through being called by the ISR
	//	or though polling the TWINT flag).
	//
	//	The purpose of this routine is to handle the new state.
	//
	//	The slave machine is held in the variables detailed
	//	below:
	//
	//	twi_slave_active	Boolean is true if slave transaction in progress
	//
	//	twi_slave_adrsd		The address the slave has replied to		
	//
	//	twi_slave_buffer	The address of the buffer space
	//
	//	twi_slave_size		The total size of the buffer space
	//
	//	twi_slave_len		How much of the buffer space has been used
	//				when either receiving data or setting data
	//				to be sent.
	//
	//	twi_slave_send		The number of bytes in the buffer to send.
	//
	//	twi_slave_answer	The call back routine used to generate a
	//				reply to the slave request.
	//
	//		FUNC( twi_slave_answer )( byte adrs, byte *buffer, byte size, byte len );
	//
	switch( twsr ) {
		//
		//	Slave Receiver
		//	--------------
		//
		case TW_SR_ARB_LOST_SLA_ACK:
		case TW_SR_ARB_LOST_GCALL_ACK: {
			//
			//	This system *was* trying to obtain the bus for a master
			//	transaction.  This has failed with a slave connection being
			//	accepted instead.
			//
			//	For an action perspective, the slave code does not change from
			//	"normal" connection.  However, the failed master connection
			//	will need to be restarted once the slave transmission is completed
			//
			//	To do this we will simply "cancel" the current master transaction.
			//
			twi_active = NULL;
			//
			//	Roll on to normal code handling.
			//
			__attribute__(( fallthrough ));
		}
		case TW_SR_SLA_ACK:
		case TW_SR_GCALL_ACK: {
			//
			//	This device has been addressed as a Slave Device
			//	with the Master sending data to the Slave.
			//
			//	If "twi_slave_active" is true then whatever was
			//	going on will be dropped (with an error being
			//	reported).
			//
			if( twi_slave_active ) {
				//
				//	We thought something was going on but another
				//	transaction has begun.
				//
				twi_sendError( TWI_ERR_TRUNCATED );
			}
			//
			//	Set up everything for reading data then
			//	set twi_slave_active as true.
			//
			twi_slave_len = 0;
			twi_slave_send = 0;
			twi_slave_active = true;
			//
			//	Make a note of the address which caused this
			//	TWI hardware to pick up this communication
			//
			twi_slave_adrsd = twi_slaveAddress();
			break;
		}
		case TW_SR_DATA_ACK:
		case TW_SR_GCALL_DATA_ACK:
		case TW_SR_DATA_NACK:
		case TW_SR_GCALL_DATA_NACK: {
			//
			//	Slave have has received data that has either been
			//	acknowldged or (final byte) not acknowledged
			//
			//	If we have a buffer to fill, and it is not full, add
			//	the byte to it.
			//
			//	We use the "twi_slave_size" variable (the maximum size
			//	of the slave buffer) to indicate if there has been a
			//	slave buffer set.
			//
			if( twi_slave_active ) {
				if( twi_slave_size ) {
					if( twi_slave_len < twi_slave_size ) {
						twi_slave_buffer[ twi_slave_len++ ] = twi_readByte();
					}
					else {
						twi_sendError( TWI_ERR_OVERFLOW );
					}
					twi_readAck( twi_slave_len < ( twi_slave_size - 1 ));
				}
				else {
					twi_sendError( TWI_ERR_NO_BUFFER );
					twi_readAck( false );
				}
			}
			else {
				//
				//	Otherwise nack
				//
				twi_sendError( TWI_ERR_TRANSACTION );
				twi_readAck( false );
			}
			break;
		}
		case TW_SR_STOP: {
			//
			//	Slave receive data ends.  Possible transition to
			//	additional transmission (via master restart), so we
			//	make the slave callback and then reset twi_slave_active.
			//
			if( twi_slave_active ) {
				if( twi_slave_answer ) {
					twi_slave_send = FUNC( twi_slave_answer )( twi_slave_adrsd, twi_slave_buffer, twi_slave_size, twi_slave_len );
				}
				else {
					twi_slave_send = 0;
				}
			}
			else {
				twi_sendError( TWI_ERR_TRANSACTION );
			}
			//
			//	We now clear down the transmission
			//
			twi_slave_active = false;
			twi_clearBus();
			break;
		}
		//
		//	Slave Transmitter
		//	-----------------
		//
		case TW_ST_ARB_LOST_SLA_ACK: {
			//
			//	This system *was* trying to obtain the bus for a master
			//	transaction.  This has failed with a slave connection being
			//	accepted instead.
			//
			//	For an action perspective, the slave code does not change from
			//	"normal" connection.  However, the failed master connection
			//	will need to be restarted once the slave transmission is completed
			//
			//	To do this we will simply "cancel" the current master transaction.
			//
			twi_active = NULL;
			//
			//	Roll on to normal code handling.
			//
			__attribute__(( fallthrough ));
		}
		case TW_ST_SLA_ACK: {
			//
			//	This device has been addressed as a Slave Device
			//	with the Master reading data from the Slave.
			//
			//	If "twi_slave_active" is false then there was no prior
			//	write to the slave, so we clear out the length, and call
			//	the callback to give the software a chance to create a
			//	reply.
			//
			if( twi_slave_active ) {
				//
				//	We thought something was going on but another
				//	transaction has begun.
				//
				twi_sendError( TWI_ERR_TRUNCATED );
			}
			//
			//	If we do not already have some data that needs to be sent
			//	(ie twi_slave_send == 0 ) then we call up the slave answer
			//	routine with an empty buffer and send what ever it puts
			//	into the buffer.
			//
			if( twi_slave_send == 0 ) {
				if( twi_slave_answer ) {
					twi_slave_send = FUNC( twi_slave_answer )( twi_slave_adrsd, twi_slave_buffer, twi_slave_size, 0 );
				}
				else {
					twi_slave_answer = 0;
				}
			}
			//
			//	Set the sending index to 0 so we send from the start of the buffer.
			//
			twi_slave_len = 0;
			//
			//	We have to automatically send the first byte, so we
			//	start from the begining.
			//
			if( twi_slave_len < twi_slave_send ) {
				twi_sendByte( twi_slave_buffer[ twi_slave_len++ ]);
			}
			else {
				twi_sendError( TWI_ERR_UNDERFLOW );
				twi_sendByte( 0 );
			}
			//
			//	Flag slave transaction in progress.
			//
			twi_slave_active = true;
			break;
		}
		case TW_ST_DATA_ACK: {
			//
			//	Previous byte sent was acknowledged, so send another.
			//
			if( twi_slave_active ) {
				if( twi_slave_len < twi_slave_send ) {
					twi_sendByte( twi_slave_buffer[ twi_slave_len++ ]);
				}
				else {
					twi_sendError( TWI_ERR_UNDERFLOW );
					twi_sendByte( 0 );
				}
			}
			else {
				twi_sendError( TWI_ERR_TRANSACTION );
			}
			break;
		}
		case TW_ST_DATA_NACK:
		case TW_ST_LAST_DATA: {	
			//
			//	Last data byte successfully sent, stop using the bus.
			//
			twi_clearBus();
			twi_slave_active = false;
			break;
		}
	}
}

//
//	void twi_masterBusReset( byte twsr )
//	------------------------------------
//
//	Called when (and only when) a master state change is
//	detected in the absence of a master transaction being in play.
//
//	The purpose of this routine is to steer the bus back to a
//	reset status thus allowing new transactions to process.
//	
static void twi_masterBusReset( byte twsr ) {
	//
	//	Review document "Application Note AN-686" from
	//	Analog Devices (www.analog.com) titled:
	//
	//	"Implementing an I2C Reset" by Jim Greene
	//
	//	This document (filename: 54305147357414AN686_0.pdf)
	//	contains a discussion on how a master device can
	//	find itself with an apparently stuck bus, and what
	//	it can do about this.
	//
	//	To be implemented.
	//
}

//
//	void twi_stateChangeHandler( byte twsr )
//	-----------------------------------------
//
//	This routine undertakes the interpretation and resulting
//	actions arising from the arrival of a new TWI state.
//
static void twi_stateChangeHandler( byte twsr ) {
	//
	//	Work out what and respond to it.
	//
	if( TW_MASTER_STATUS( twsr )) {
		//
		//	MASTER Actions
		//	--------------
		//
		//	Use twi_masterStateMachine to action the consequences of this
		//	state change.
		//
		if( twi_active ) {
			//
			//	We have a transaction lined up, so lets move
			//	this on.
			//
			twi_masterMachine( twi_active, twsr );
			if( twi_has_timeout ) twi_start_time = micros();
		}
		else {
			//
			//	We are getting a master state, but without any
			//	associated master transaction: Execute a bus
			//	release and try to return the bus to "normal"
			//
			twi_masterBusReset( twsr );
		}
	}
	else {
		if( TW_SLAVE_STATUS( twsr )) {
			//
			//	SLAVE Actions
			//	-------------
			//
			//	Note that it *is* possible for the firmware
			//	for have a master transaction apparently in
			//	progress and to have slave states being raised.
			//
			//	This will have been cause by the firmware lining
			//	up the next master transaction, but another
			//	device on the bus getting in with their own
			//	master transaction ahead of this one.
			//
			//	So, we do no other checking and simply allow
			//	the slave state machine to proceed as it sees
			//	appropriate.
			//
			twi_slaveMachine( twsr );
			if( twi_has_timeout ) twi_start_time = micros();
		}
		else {
			//
			//	This is a state about the bus which we
			//	need to handle here.
			//
			switch( twsr ) {
				case TW_NO_INFO: {
					//
					//	The status value is "empty", ignore.
					//
					break;
				}
				case TW_BUS_ERROR: {
					//
					//	The bus is malfunctioning.  We will
					//	initiate a full shutdown of anything
					//	currently "in flight".
					//
					twi_stop();
					twi_dropCurrentAction();
					twi_slave_active = false;
					break;
				}
				default: {
					//
					//	An unrecognised status value.  Treat
					//	like a bus error.
					//
					twi_stop();
					twi_dropCurrentAction();
					twi_slave_active = false;
					break;
				}
			}
		}
	}
}

//
//	void twi_eventProcessing( void )
//	--------------------------------
//
//	This routine is to be called as part of the main event loop of an
//	application, ideally as frequently as possible.  This is the
//	routine which will call the "reply" routines ensuring that
//	these are not "caught up" in the time and space restrictions that
//	being part of an ISR call would create.
//
//	This will also, if TWI polling is enabled, drive forward any
//	in progress transaction (slave or master).
//
void twi_eventProcessing( void ) {

	//
	//	Master Processing
	//	-----------------
	//
	//	The value of "twi_active" indicates (if non-zero) that there
	//	transaction in progress with the details of the transaction
	//	is a at the address indicated.
	//
	if( twi_active ) {
		//
		//	This points to a transaction record so we look to see
		//	if there are actions which we have to execute on its
		//	behalf.
		//
		//	These fall into one of two categorise:  Either we need
		//	start the master transmission, or we need to complete it.
		//
		//	We only run the callback when the action indicates this
		//	and then subsequently delete the record.
		//
		switch( pgm_read_byte( twi_active->action )) {
			case TWI_HW_START: {
				//
				//	Generate the start condition and move the
				//	action pointer forward to the next step.
				//
				if( twi_has_timeout ) twi_start_time = micros();
				twi_active->next = 0;	// New index into the buffer
				twi_active->action++;
				twi_start();
				break;
			}
			case TWI_HW_GOOD_CALLBACK: {
				//
				//	Make a good call back (if the callback is not NULL)
				//
				if( twi_active->reply ) FUNC( twi_active->reply )( true, twi_active->link, twi_active->buffer, twi_active->next );
				//
				//	Throw the transaction away.
				//
				twi_loadNextAction();
				break;
			}
			case TWI_HW_FAIL_CALLBACK: {
				//
				//	Make a bad call back (if the callback is not NULL)
				//
				if( twi_active->reply ) FUNC( twi_active->reply )( false, twi_active->link, NULL, 0 );
				//
				//	Throw the transaction away.
				//
				twi_loadNextAction();
				break;
			}
			default: {
				//
				//	Any other action we simply ignore as none
				//	of our business (as this will be dealt with
				//	through another mechanism).
				//
				break;
			}
		}
	}
	else {
		//
		//	There is no active record, so if the queue is not
		//	empty then set the pointer to the next available
		//	record and initialise the time out new value.
		//
		if( twi_queue_len > 0 ) twi_loadNextAction();
	}

	//
	//	TWI Polling Code
	//	----------------
	//
	//	Here we implement TWI polling (if interrupts are not being used).
	//
	if( !twi_enable_interrupt ) {
		//
		//	This code handles the TWI hardware through polling.
		//
		if( twi_actionComplete()) {
			//
			//	The last action on the TWI interface has completed
			//	so we need to drive this forward:
			//
			twi_stateChangeHandler( twi_state());
		}
	}
	
	//
	//	Timeout handling
	//	----------------
	//
	//	Finally we need to see if the current action
	//	has stalled and needs cancelling.
	//
	if( twi_has_timeout ) {
		if(( micros() - twi_start_time ) > twi_timeout_delay ) {
			//
			//	Yup!  Cancel ... something ... we need to work out what has
			//	been going on to work out what we need to cancel
			//
			//	Fortunately there is a sequence we can use to determine what
			//	is going on at this moment.  Essentially if "twi_slave_active"
			//	is true then a Slave Transaction *must* be in progress, and the
			//	time out relates to that.
			//
			//	If "twi_slave_active" is false then we can rely on the "twi_active"
			//	pointer to supply a candidate for a Master Transaction being caught
			//	out by time out.
			//
			//	If neither of the above apply there there has been something
			//	strange going on and we will simply perform a generic reset.
			//
			//	So, Slave Transaction?
			//
			if( twi_slave_active ) {
				//
				//	Actually this is a little tricky.  Literally the slave does not
				//	control the communications which seem to have timed out, and so
				//	there is nothing it can do to "prompt" the master end to carry on.
				//
				//	Suggest simply doing a hardware reset.
				//
				twi_resetHardware();
			}
			else {
				//
				//	Now, Master Transaction?
				//
				if( twi_active ) {
					//
					//	Make a bad call back (if the callback is not NULL)
					//
					if( twi_active->reply ) {
						twi_error = TWI_ERR_TIMED_OUT;
						FUNC( twi_active->reply )( false, twi_active->link, NULL, 0 );
					}
					//
					//	Stop the transaction directly.
					//
					twi_stop();
					twi_dropCurrentAction();
				}
				else {
					//
					//	Ok, Generic response.
					//
					twi_resetHardware();
				}
			}
		}
	}
}


//
//	Two Wire Interrupt Service Routine.
//	===================================
//
//	This routine simply calls the generic event/state handler
//	passing in the current time of the ISR and the state
//	value being notified.
//
//
ISR( TWI_vect ) {
	twi_stateChangeHandler( twi_state());
}


//
//	EOF
//

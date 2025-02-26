///
///	TWI.h - A simplified TWI/I2C API library for Arduino
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

//
//	This collection of routines will (within reasonable efforts) implement
//	the protocol as outlined in the following document:
//
//		http://smbus.org/specs/SMBus_3_1_20180319.pdf
//
//	How this document has been interpreted, and the design decisions which
//	derive from these interpretations will be captured in the its
//	implementation file.
//

#ifndef _TWI_H_
#define _TWI_H_

#include "Environment.h"
#include "Configuration.h"
#include "Parameters.h"
#include "Task_Entry.h"
#include "Console.h"
#include "Trace.h"


//
//	This code divides all TWI clock frequencies by 10K
//	to make them fit inside a single byte.
//
//	This "relationship" between actual frequency and
//	its internal representation is not mathematical as
//	the clock speed applied to the TWI hardware is
//	determined using a lookup table.  It would, therefore,
//	be perfectly feasible to have a non-linear association
//	as long as larger numbers meant faster speeds.
//
//	Default Frequency is 100KHz.
//
#ifndef TWI_FREQ
#define TWI_FREQ		10
#endif

//
//	Allow configuration of the internal queue length.  This
//	define the maximum number of pending TWI actions that the
//	driver can sustain.  In reality this effectively defines
//	the number of TWI attached devices which can be communicated
//	with in a parallel manner, but not the actual maximum number
//	of attached devices.
//
#ifndef TWI_MAX_QUEUE_LEN
#define TWI_MAX_QUEUE_LEN	4
#endif

//
//	Specify a delay period used during a hardware reset.  This is
//	the time between "turning off" the device and the following
//	re-enable actions.  This is (hopefully) enough time for the
//	TWI attached device causing trouble to reset itself and so enable
//	the bus to return to normal operation.
//
#ifndef TWI_HARDWARE_RESET
#define TWI_HARDWARE_RESET	1000
#endif

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
//	Two Wire Status values
//	----------------------
//
//	In the file <util/twi.h> are the definitions for the
//	status values which can be found in the top 5 bits of the
//	TWSR register (with the bottom 3 being used for other purposes).
//
//	The defines made in this file are fixed as they are derived from
//	hardware embedded in the TWI implementation in the AVR chips.
//
//	A quick analysis of these values shows that once shifted right 3
//	bits there is little "logical" meaning to the individual bits, but
//	form a number sequence from 1 to 25.  Within this range they codes
//	are grouped into four groups and a scattering of "others".  These
//	are:
//
//		Master Initialisation	TW_START, TW_REP_START
//		Master Transmit		TW_MT_*
//		Master Receive		TW_MR_*
//		Slave Receive		TW_SR_*
//		Slave Transmit		TW_ST_*
//		"Others"		No Info, Bus Error
//
//	It is worth noting that within TW_MT_ and TW_MR_ there is
//	a duplicated entry: "TW_MT_ARB_LOST" and "TW_MR_ARB_LOST"
//	with the same value: 0x38.  This is, however, unique to
//	the Master set of codes as the Slave codes contain their
//	own specific versions.
//
//	To simplify some handling of the TWI status values the following
//	macros are provided to allow "classification" of the status code
//	before specific code-by-code analysis is undertaken.
//

//
//	Master related status values:
//
//						TWSR & 0xf8	TWSR >> 3
//
//		TW_START			0x08		1
//		TW_REP_START			0x10		2
//
//		TW_MT_SLA_ACK			0x18		3
//		TW_MT_SLA_NACK			0x20		4
//		TW_MT_DATA_ACK			0x28		5
//		TW_MT_DATA_NACK			0x30		6
//
//		TW_MT_ARB_LOST			0x38		7
//		TW_MR_ARB_LOST			0x38		7
//
//		TW_MR_SLA_ACK			0x40		8
//		TW_MR_SLA_NACK			0x48		9
//		TW_MR_DATA_ACK			0x50		10
//		TW_MR_DATA_NACK			0x58		11
//
#define TW_MASTER_STATUS(x)	(((x)>=TW_START)&&((x)<=TW_MR_DATA_NACK))

//
//		Slave related status values:
//
//						TWSR & 0xf8	TWSR >> 3
//
//		TW_SR_SLA_ACK			0x60		12
//		TW_SR_ARB_LOST_SLA_ACK		0x68		13
//		TW_SR_GCALL_ACK			0x70		14
//		TW_SR_ARB_LOST_GCALL_ACK	0x78		15
//		TW_SR_DATA_ACK			0x80		16
//		TW_SR_DATA_NACK			0x88		17
//		TW_SR_GCALL_DATA_ACK		0x90		18
//		TW_SR_GCALL_DATA_NACK		0x98		19
//		TW_SR_STOP			0xA0		20
//
//		TW_ST_SLA_ACK			0xA8		21
//		TW_ST_ARB_LOST_SLA_ACK		0xB0		22
//		TW_ST_DATA_ACK			0xB8		23
//		TW_ST_DATA_NACK			0xC0		24
//		TW_ST_LAST_DATA			0xC8		25
//
#define TW_SLAVE_STATUS(x)	(((x)>=TW_SR_SLA_ACK)&&((x)<=TW_ST_LAST_DATA))

//
//	"Others"
//
//						TWSR & 0xf8	TWSR >> 3
//
//		TW_NO_INFO			0xF8		31	
//		TW_BUS_ERROR			0x00		0
//

//
//	In addition to the values defined in the file <util/twi.h>
//	these code specific value are also set into this variable.
//
//	Looking at the above table it can be seen that the values from
//	26 to 30 are not used, and could be hijacked for synthetic
//	status values, if that were required.
//
#define TW_NO_ERROR	0xff


//
//	Define the Twin Wire Interface (TWI) also known as I2C or IIC.
//
class TWI : public Task_Entry {
public:
	//
	//	Declare the frequency of the TWI interface to the
	//	attached devices.
	//
	static const byte	frequency = TWI_FREQ;

	//
	//	Provide a symbol which will mean "the highest possible
	//	speed"
	//
	//	At the moment "255" would mean approximately 2.5 MHz
	//	which is an improbable TWI speed.
	//
	static const byte	maximum_frequency = 255;

	//
	//	Define a period of Microseconds which the firmware will
	//	wait during a hardware reset of the TWI
	//
	//	This is a last resort attempt to put things back into an
	//	operation state.
	//
	static const byte	reset_delay_us = 50;

	//
	//	Define the Lowest and Highest valid addresses.
	//
	static const byte	lowest_address = 16;
	static const byte	Highest_address = 127;
	
	//
	//	Define the number of micro-seconds we pause operation
	//	of the TWI device in the event of a hardware reset.
	//
	static const word	hardware_reset = TWI_HARDWARE_RESET;

	//
	//	These are the error codes passed to the error reporting
	//	routine.
	//
	enum error_code : byte {
		error_none = 0,		//	No Error, success!
		error_ignored,		
		error_timed_out,	//	Action timed out
		error_queue_full,	//	Master transaction queue full
		error_hw_reset,		//	TWI Hardware has been reset
		error_address,		//	Slave address not acknowledged
		error_write_fail,	//	Write data to slave not acknowledged
		error_read_fail,	//	n/a
		error_transaction,	//	Unexpected state change in transaction
		error_starting,		//	Transaction start failed
		error_truncated,	//	Slave transaction truncated
		error_overflow,		//	Slave recv buffer overflowed, data lost
		error_underflow,	//	Slave send buffer too small, zero filling
		error_no_buffer,	//	No buffer space has been allocated
		error_dropped		//	Transaction was dropped before completion
	};

private:
	//
	//	Declare the data structure the TWI code relies upon.
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
	struct bitrate {
		byte	freq,		// freq / 10Khz
			twbr,		// Range 10 .. 255
			twps;		// Range 0 .. 3
	};
	//
	//	The bitrate table:
	//
	static const bitrate bitrates[] PROGMEM;


	//
	//	The following enumeration enable the Transaction Diagrams
	//	to be directly encoded and used as the basis for the
	//	interrupt service routine and supporting service routines.
	//

	enum machine_state : byte {
		//
		//		Emit start condition (handled by event loop code)
		//		Emit a restart condition.
		//		Start or restart has completed.
		//
		state_start,
		state_restart,
		state_start_complete,

		//
		//		Emit target address with Read.
		//		Emit target address with Write.
		//		Wait for address Acknowledgement.
		//
		state_adrs_read,
		state_adrs_write,
		state_adrs_ack,

		//
		//		Send one byte of data
		//		Wait for slave Acknowledgement and...
		//		...loop back 1 if data still to send.
		//
		state_send_byte,
		state_send_ack_loop,

		//
		//		Tell TWI hardware that we are waiting
		//		for data.  Set to use NAck if this will
		//		be the last byte, set for Ack if this is
		//		not the last byte.
		//		The receive action actually gets the data
		//		and loops back if the byte was Ackd or
		//		rolls to the next action if it was NAckd.
		//
		state_recv_ready,
		state_recv_byte_loop,

		//
		//		Emit a stop condition.
		//
		state_stop,

		//
		//	This action represent the task of sending
		//	back a notification that an action has
		//	completed (to its initiator) and kicking
		//	off the next one.
		//
		state_finish_action
	};

	//
	//	The following static tables provide the state machines
	//	which handle the execution of any given TWI activity
	//	as defined in the SMBus Specification.
	//
	static const machine_state mode_quick_read[] PROGMEM;
	static const machine_state mode_quick_write[] PROGMEM;
	static const machine_state mode_send_data[] PROGMEM;
	static const machine_state mode_receive_byte[] PROGMEM;
	static const machine_state mode_data_exchange[] PROGMEM;
	static const machine_state abort_transaction[] PROGMEM;

	//
	//	Define a data structure used to queue pending
	//	exchange requests initiated from this MCU as a
	//	master device.
	//
	struct transaction {
		//
		//	The list of actions this needs to implement, and the
		//	reset address if the transaction needs to be restarted
		//	mid flight.
		//
		const machine_state	*action;
		//
		//	Who are we trying to exchange data with?
		//
		byte			target;
		//
		//	The address of the buffer space to be used as
		//	the source data and repository for the subsequent
		//	reply.  Next forms a pointer used to access the
		//	buffer space.
		//
		byte			*buffer;
		//
		//	The next field is used to "step through" the buffer
		//	when either sending or receiving data.
		//
		byte			next;
		//
		//	The number of bytes in the buffer space which form
		//	the data to be sent from the master to the slave
		//
		byte			send;
		//
		//	The number of bytes expected back from the slave
		//	which will be placed into the buffer
		//
		byte			recv;
		//
		//	The signal to be released when the action completes.
		//
		Signal			*flag;
		//
		//	The address where the error code of action is placed.
		//
		error_code		*result;
		//
		//	The next transaction to undertake (but cannot use
		//	next as its been used).
		//
		transaction		*tail;
	};

	//
	//	Define the pending exchange queue and indexes into it.
	//
	transaction	*_active,
			**_tail,
			*_free;

	//
	//	Declare the event/process handle to be used with the
	//	interrupt handler.
	//
	static const byte irq_process	= 1;

	//
	//	Declare the flag used to link an interrrupt (and ISR call)
	//	to the state machine that operates the TWI hardware and
	//	a variable to hold the status register at that point
	//	in time.
	//
	Signal		_irq;
	byte		_twsr;

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
	//	void start( void )
	//	------------------
	//
	//	Send start condition (also doubles up as a restart)
	//
	static inline void start( void ) {
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
		TWCR = bit( TWIE ) | bit( TWEN ) | bit( TWSTA ) | bit( TWINT );
	}

	//
	//	byte status( byte twsr )
	//	------------------------
	//
	//	Return the "status component" of the TWSR value.
	//
	static inline byte status( byte twsr ) {
		return( twsr & 0xf8 );
	}

	//
	//	bool action_complete( void )
	//	-------------------------------
	//
	//	Checks the value of the Initiate flag (TWINT) which will
	//	return to 1 when the previous activity has been completed.
	//
	static inline bool action_complete( void ) {
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
	//	void send_byte( byte data )
	//	------------------------------
	//
	//	Transmit a byte of data
	//
	static inline void send_byte( byte data ) {
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
		TWCR = bit( TWIE ) | bit( TWEN ) | bit( TWINT );
	}

	//
	//	void send_target( byte adrs, bool writing )
	//	----------------------------------------------
	//
	//	Send the address byte of the slave being targetted by
	//	this transmission. The argument writing indicates if
	//	the following transmission involves the master writing
	//	to the slave (true) or readin from thw slave (false).
	//
	static inline void send_target( byte adrs, bool writing ) {
		//
		//	send_byte(( adrs << 1 )|( writing? TW_WRITE: TW_READ ));
		//
		TWDR = ( adrs << 1 )|( writing? TW_WRITE: TW_READ );
		TWCR = bit( TWIE ) | bit( TWEN ) | bit( TWINT );
	}

	//
	//	void read_ack( bool ack )
	//	----------------------------
	//
	//	This routine is only about sending an Acknowledgement
	//	or NOT Acknowledgement as a result of a byte being
	//	delivered to us.
	//
	static inline void read_ack( bool ack ) {
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
		TWCR = bit( TWIE ) |( ack? bit( TWEA ): 0 )| bit( TWEN ) | bit( TWINT );
	}

	//
	//	byte read_byte( void )
	//	----------------------
	//
	//	Return the byte which has just been delivered to the system
	//
	static inline byte read_byte( void ) {
		//
		//	TWI Data Register, TWDR
		//
		return( TWDR );
	}

	//
	//	void stop( void )
	//	---------------------
	//
	//	Tell the remote slave device that we are done, and
	//	release the bus for other devices to use.
	//
	static inline void stop( void ) {
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
		TWCR = bit( TWIE ) | bit( TWEN ) | bit( TWSTO ) | bit( TWINT );
	}
	
	//
	//	Disable the TWI interrupt until the next action has been
	//	submitted.  This is used to prevent series interrupts
	//	for the same event if the event notification and event
	//	handler are not part of the ISR.
	//
	static inline void disable( void ) {
		//
		//	TWI Control Register, TWCR
		//
		//	Bit	Value	Meaning
		//
		//	TWINT	X	Initiate action (resets bit to 0)
		//	TWEA	X	TWI Slave enable / (N)Ack reply
		//	TWSTA	X	Transmit Start condition
		//	TWSTO	X	Transmit Stop condition
		//	TWWC	X	TWDR Write Collision detected
		//	TWEN	X	TWI Hardware enable
		//	TWIE	0	TWI Interrupt enable
		//
		TWCR = bit( TWEN );
	}

	//
	//	void clear_bus( void )
	//	-------------------------
	//
	//	Return the TWI Bus to a "quiessed" state ready for the next
	//	transmission.
	//
	static inline void clear_bus( void ) {
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
		TWCR = bit( TWIE ) | bit( TWEN ) | bit( TWINT );
	}

	//
	//	void reset_hardware( void )
	//	------------------------------
	//
	//	This is a final "let us try power off then on" the TWI
	//	hardware to regain control of the system.
	//
	static void reset_hardware( void );


	//
	//	void next_action( void )
	//	------------------------
	//
	//	Routine to handle bringing a action to an end and launching
	//	a new one.
	//
	void next_action( void );


	//
	//	bool queue_transaction( const machine_state *action, byte address, byte *buffer, byte send, byte recv, Signal *flag, error_code *result )
	//	------------------------------------------------------------------------------------------------------------------------------------------
	//
	//	Initiates an asynchronous data exchange with a specified slave device, with the parameters supplying
	//	the required details:
	//
	//		action		The list of actions which implement this transaction
	//		adrs		The slave address being targeted
	//		buffer		The address of a byte array where sent and returned data will be stored.
	//		send		The number of bytes to send from the buffer area
	//		recv		The number of bytes which will be returned into the buffer area
	//		flag		Address of a flag to set to true when the command completes
	//		result		Address of an error_code variable where the status of the completed command is placed
	//
	//	Returns true if the exchange has been successfully queued, false otherwise.
	//
	//	Note:	The buffer space address passed into this routine is kept and accessed
	//		asynchronously from the main flow of the program.  To this end the area
	//		must be allocated in such a way that it remains valid from the point of
	//		calling this routine to the point at which the flag is set.
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
	bool queue_transaction( const machine_state *action, byte adrs, byte *buffer, byte send, byte recv, Signal *flag, error_code *result );

public:
	//
	//	void set_frequency( byte freq )
	//	-------------------------------
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
	void set_frequency( byte freq );

	//
	//	byte best_frequency( byte freq )
	//	--------------------------------
	//
	//	Return the identity of the best available
	//	clock speed at (or below) the value passed in.
	//
	byte best_frequency( byte freq );

	//
	//	byte lowest_frequency( void )
	//	-----------------------------
	//
	//	Return the identity of the slowest available
	//	clock speed.  This could be useful when a master
	//	and a slave negotiate the best available
	//	common speed .. you need to start from somewhere.
	//
	//	This *should* always come out as "1", and (currently)
	//	reflects a speed of 10Kbps
	//
	byte lowest_frequency( void );


	//
	//	Constructor initialises the TWI to the default
	//
	TWI( void );

	//
	//	Call this routine to make things begin.
	//
	void initialise( void );

	//
	//	The (6.5.1) Quick Commands
	//	--------------------------
	//
	bool quick_read( byte adrs, Signal *flag, error_code *result );

	bool quick_write( byte adrs, Signal *flag, error_code *result );

	//
	//	All "just send data" commands
	//	-----------------------------
	//	(6.5.2) Send Byte
	//	(6.5.4) Write Byte/Word
	//	(6.5.7) Block Write/Read
	//	(6.5.10) Write 32 protocol
	//	(6.5.12) Write 64 protocol
	//
	bool send_data( byte adrs, byte *buffer, byte send, Signal *flag, error_code *result );

	//
	//	The (6.5.3) Receive Byte
	//	------------------------
	//
	bool receive_byte( byte adrs, byte *buffer, Signal *flag, error_code *result );

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
	bool exchange( byte adrs, byte *buffer, byte send, byte recv, Signal *flag, error_code *result );

	//
	//	void process( byte handle )
	//	---------------------------
	//
	//	Using the TWI state supplied drive the transaction record forward.
	//	When the routine returns something will have been done.
	//
	virtual void process( byte handle );
	
	//
	//	The routine called by the ISR to indicate an interrupt
	//	has been received.
	//
	void irq( byte twsr );
};



//
//	Define the object controlling the TWI device
//
extern TWI twi;

#endif

//
//	EOF
//

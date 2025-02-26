///
///	TWI.cpp - A simplified TWI/I2C API library for Arduino
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
//	Environment for this module.
//
#include "TWI.h"
#include "Clock.h"
#include "Code_Assurance.h"
#include "Memory_Heap.h"
#include "Task.h"
#include "Trace.h"
#include "Stats.h"

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
//	The (6.5.1) Quick Commands boil down to one of two possible
//	single bytes transmission from the master to the slave.  This
//	is effectively just the target slave address with the Read
//	bit either 1 (for reading) or 0 (for writing).  No data is read
//	of transmitted.
//
//	MODE_QUICK_READ
//	---------------
//	Master:	SaaaaaaaR P
//	Slave:           A
//
//	MODE_QUICK_WRITE
//	----------------
//	Master:	SaaaaaaaW P
//	Slave:           A
//
static const TWI::machine_state TWI::mode_quick_read[] PROGMEM = {
	state_start, state_start_complete,
	state_adrs_read, state_adrs_ack,
	state_stop
};
static const TWI::machine_state TWI::mode_quick_write[] PROGMEM = {
	state_start, state_start_complete,
	state_adrs_write, state_adrs_ack,
	state_stop
};

//
//	All "just send data" commands (6.5.2 Send Byte, 6.5.4 Write
//	Byte/Word, 6.5.7 Block Write/Read, 6.5.10 Write 32 protocol,
//	6.5.12 Write 64 protocol) conform to the following underlying
//	hardware exchange:
//
//	MODE_SEND_DATA
//	--------------
//	Master:	SaaaaaaaW dddddddd P
//	Slave:	         A        A
//	Repeat:	          ^^^^^^^^^
//
static const TWI::machine_state TWI::mode_send_data[] PROGMEM = {
	state_start, state_start_complete,
	state_adrs_write, state_adrs_ack,
	state_send_byte, state_send_ack_loop,
	state_stop
};
//
//	The (6.5.3) Receive Byte is the only command which can directly
//	obtain any data form the slave *without* first asking for
//	something:
//
//	MODE_RECEIVE_BYTE
//	-----------------
//	Master:	SaaaaaaaR         NP
//	Slave:	         Adddddddd
//
static const TWI::machine_state TWI::mode_receive_byte[] PROGMEM = {
	state_start, state_start_complete,
	state_adrs_read, state_adrs_ack,
	state_recv_ready, state_recv_byte_loop,
	state_stop
};
//
//	All of the remaining commands (6.5.4 Write Byte/Word, 6.5.5
//	Read Byte/Word, 6.5.6 Process Call, 6.5.8 Block Write-Block
//	Read Process Call, 6.5.9 SMBus Host Notify protocol, 6.5.11
//	Read 32 protocol, 6.5.13 Read 64 protocol) conform to a common
//	data exchange protocol:
//
//	MODE_DATA_EXCHANGE
//	------------------
//	Master:	SaaaaaaaW dddddddd TSaaaaaaaR         A          NP
//	Slave:	         A        A          Adddddddd   dddddddd
//	Repeat:	          ^^^^^^^^^           ^^^^^^^^^
//
static const TWI::machine_state TWI::mode_data_exchange[] PROGMEM = {
	state_start, state_start_complete,
	state_adrs_write, state_adrs_ack,
	state_send_byte, state_send_ack_loop,
	state_restart, state_start_complete,
	state_adrs_read, state_adrs_ack,
	state_recv_ready, state_recv_byte_loop,
	state_stop
};

//
//	abort_transaction
//	--------------------
//
//	This incomplete sequence is not applied in the first instance
//	but is used to "redirect" a transaction which has failed and
//	needs to completed with subsequent tidy up.
//
static const TWI::machine_state TWI::abort_transaction[] PROGMEM = {
	state_stop
};


//
//	The bitrate table
//	-----------------
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
static const TWI::bitrate TWI::bitrates[] PROGMEM = {
#if	F_CPU == 20000000
	{	40,	17,	0	},
	{	35,	20,	0	},
	{	30,	25,	0	},
	{	25,	32,	0	},
	{	20,	42,	0	},
	{	15,	58,	0	},
	{	10,	92,	0	},
	{	9,	103,	0	},
	{	8,	117,	0	},
	{	7,	33,	1	},
	{	6,	39,	1	},
	{	5,	48,	1	},
	{	4,	60,	1	},
	{	3,	81,	1	},
	{	2,	123,	1	},
	{	1,	62,	2	},
	
#elif	F_CPU == 16000000
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
//	void reset_hardware( void )
//	------------------------------
//
//	This is a final "let us try power off then on" the TWI
//	hardware to regain control of the system.
//
void TWI::reset_hardware( void ) {

	STACK_TRACE( "void TWI::reset_hardware( void )" );
	
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
	//	Pause execution of the TWI code.  For this
	//	to simplify this code we use the "in-line"
	//	delay provided by the event_timer module. 
	//
	event_timer.inline_delay( USECS( hardware_reset ));
	
	//
	//	Now put the control register back, but without
	//	a TWSTA, TWSTO or TWINT set.
	//
	TWCR = twcr & ~( TWSTA | TWSTO | TWINT );
}



//
//	void next_action( void )
//	------------------------
//
//	Routine to handle bringing a action to an end and launching
//	a new one.
//
void TWI::next_action( void ) {

	STACK_TRACE( "void TWI::next_action( void )" );

	transaction	*ptr;

	//
	//	Are we replacing a now finished job?
	//
	if(( ptr = _active )) {
		
		//
		//	Tell creator action has completed.
		//
		TRACE_TWI( console.print( F( "TWI release flag " )));
		TRACE_TWI( console.println( _active->flag->identity()));

		ptr->flag->release();

		//
		//	Forget the action we have completed.
		//
		if(!( _active = ptr->tail )) _tail = &( _active );
		ptr->tail = _free;
		_free = ptr;
	}
	//
	//	Is there anything left in the queue?
	//
	if( _active ) {
		//
		//	Kick off this action.
		//
		//	To do this we fake getting an "initial" interrupt
		//	so that the state machine performs the first
		//	hardware start action.
		//
		
		ASSERT( _irq.value() == 0 );
		
		_irq.release( true );

		TRACE_TWI( console.println( F( "TWI queue started" )));
	}
}


//
//	bool queue_transaction( const machine_state *action, byte address, byte *buffer, byte send, byte recv, bool *flag, error_code *result )
//	------------------------------------------------------------------------------------------------------------------------------------------
//
//	Initiates an asynchronous data exchange with a specified slave device, with the parameters supplying
//	the required details:
//
//		action		The list of actions which implement this transaction
//		address		The slave address being targeted
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
bool TWI::queue_transaction( const TWI::machine_state *action, byte adrs, byte *buffer, byte send, byte recv, Signal *flag, TWI::error_code *result ) {

	STACK_TRACE( "bool TWI::queue_transaction( const TWI::machine_state *action, byte adrs, byte *buffer, byte send, byte recv, Signal *flag, TWI::error_code *result )" );

	transaction	*ptr;

	ASSERT( flag != NIL( Signal ));
	ASSERT( result != NIL( error_code ));

	TRACE_TWI( console.print( F( "TWI flag " )));
	TRACE_TWI( console.println( flag->identity()));

	//
	//	Re-use or create a transaction record.
	//
	if(( ptr = _free )) {
		_free = ptr->tail;
	}
	else {
		if(!( ptr = new transaction )) return( false );
	}
	
	//
	//	"ptr" is the address of our queue record, so we can now
	//	fill in this record with the supplied details.
	//
	ptr->action = action;
	ptr->target = adrs;
	ptr->buffer = buffer;
	ptr->next = 0;			// Also set up by the START and RESTART actions
	ptr->send = send;
	ptr->recv = recv;
	ptr->flag = flag;
	ptr->result = result;
	ptr->tail = NIL( transaction );
	
	//
	//	Last action; append to the queue, and start the queue
	//	if this is the first in the queue!
	//
	*_tail = ptr;
	_tail = &( ptr->tail );
	if( _active == ptr ) {
		TRACE_TWI( console.println( F( "TWI start queue" )));
		
		_irq.release( true );
	}
	
	//
	//	Good to go!
	//

	TRACE_TWI( console.println( F( "TWI transaction queued" )));
		
	return( true );
}

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
void TWI::set_frequency( byte freq ) {

	STACK_TRACE( "void TWI::set_frequency( byte freq )" );

	const bitrate	*look;
	byte		found,
			twbr,
			twps;

	twbr = 0;
	twps = 0;
	look = bitrates;
	while(( found = progmem_read_byte( look->freq ))) {
		twbr = progmem_read_byte( look->twbr );
		twps = progmem_read_byte( look->twps );
		if( found <= freq ) break;
		look++;
	}
	TWSR = twps;
	TWBR = twbr;
}

//
//	byte best_frequency( byte freq )
//	--------------------------------
//
//	Return the identity of the best available
//	clock speed at (or below) the value passed in.
//
byte TWI::best_frequency( byte freq ) {

	STACK_TRACE( "byte TWI::best_frequency( byte freq )" );

	const bitrate	*look;
	byte		found;

	look = bitrates;
	while(( found = progmem_read_byte( look->freq ))) {
		if( found <= freq ) break;
		look++;
	}
	return( found );
}

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
byte TWI::lowest_frequency( void ) {

	STACK_TRACE( "byte TWI::lowest_frequency( void )" );

	const bitrate	*look;
	byte		found,
			last;

	look = bitrates;
	last = maximum_frequency;
	while(( found = progmem_read_byte( look->freq ))) {
		last = found;
		look++;
	}
	return( last );
}


//
//	Constructor initialises the TWI to the default
//
TWI::TWI( void ) {
	//
	//	Prepare the queue as empty.
	//
	_active = NIL( transaction );
	_tail = &( _active );
	_free = NIL( transaction );
}

//
//	Call this routine to make things begin.
//
void TWI::initialise( void ) {

	STACK_TRACE( "void TWI::initialise( void )" );

	//
	//	Attach ourselves to the task manager so that
	//	we process the interrupts asynchronously.
	//
	if( !task_manager.add_task( this, &_irq, irq_process )) ABORT( TASK_MANAGER_QUEUE_FULL );

	//
	//	Disable slave configuration
	//
	TWAR = 0;

	//
	//	Optionally enable internal pull-up resistors required
	//	for TWI protocol.
	//
	pinMode( SDA, INPUT_PULLUP );
	pinMode( SCL, INPUT_PULLUP );

	//
	//	Initialise TWI pre-scaler and default bit rate.
	//
	set_frequency( frequency );

	//
	//	Enable TWI module and TWI interrupt.
	//
	TWCR = bit( TWIE ) | bit( TWEN );
}


//
//	The (6.5.1) Quick Commands
//	--------------------------
//
bool TWI::quick_read( byte adrs, Signal *flag, TWI::error_code *result ) {

	STACK_TRACE( "bool TWI::quick_read( byte adrs, Signal *flag, TWI::error_code *result )" );

	return( queue_transaction( mode_quick_read, adrs, NULL, 0, 0, flag, result ));
}

bool TWI::quick_write( byte adrs, Signal *flag, error_code *result ) {

	STACK_TRACE( "bool TWI::quick_write( byte adrs, Signal *flag, error_code *result )" );

	return( queue_transaction( mode_quick_write, adrs, NULL, 0, 0, flag, result ));
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
bool TWI::send_data( byte adrs, byte *buffer, byte send, Signal *flag, error_code *result ) {

	STACK_TRACE( "bool TWI::send_data( byte adrs, byte *buffer, byte send, Signal *flag, error_code *result )" );

	return( queue_transaction( mode_send_data, adrs, buffer, send, 0, flag, result ));
}

//
//	The (6.5.3) Receive Byte
//	------------------------
//
bool TWI::receive_byte( byte adrs, byte *buffer, Signal *flag, error_code *result ) {

	STACK_TRACE( "bool TWI::receive_byte( byte adrs, byte *buffer, Signal *flag, error_code *result )" );

	return( queue_transaction( mode_receive_byte, adrs, buffer, 0, 1, flag, result ));
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
bool TWI::exchange( byte adrs, byte *buffer, byte send, byte recv, Signal *flag, error_code *result ) {

	STACK_TRACE( "bool TWI::exchange( byte adrs, byte *buffer, byte send, byte recv, Signal *flag, error_code *result )" );

	return( queue_transaction( mode_data_exchange, adrs, buffer, send, recv, flag, result ));
}


//
//	void process( byte handle )
//	---------------------------
//
//	Using the TWI state supplied drive the transaction record forward.
//	When the routine returns something will have been done.
//
void TWI::process( UNUSED( byte handle )) {

	STACK_TRACE( "void TWI::process( byte handle )" );

	TRACE_TWI( console.print( F( "TWI twsr = " )));
	TRACE_TWI( console.println_hex( _twsr ));

	//
	//	Handle strange cases where the TWI hardware emits a
	//	state where we are not processing an action.
	//
	if( _active == NIL( transaction )) {
		TRACE_TWI( console.println( F( "TWI _active is NIL" )));

		errors.log_error( TWI_STATE_CHANGE, _twsr );
		return;
	}
	
	//
	//	The purpose of this routine is to handle the new state
	//	by moving on the master transaction machine forward.
	//
loop:	switch((machine_state)progmem_read_byte_at( _active->action )) {
		case state_start: {
			TRACE_TWI( console.println( F( "TWI state_start" )));

			//
			//	Generate the start condition and move the
			//	action pointer forward to the next step.
			//
			//	Starting the transmission (though calling start())
			//	will cause the TWI interrupt routine to handle
			//	all the asynchronous interactions with the TWI
			//	hardware without any "main line" code execution.
			//
			//	Transfer back to the "main line" code takes place
			//	at the end of the transmission when the stop()
			//	command is reached.
			//
			_active->next = 0;	// New index into the buffer
			_active->action++;
			start();
			break;
		}
		case state_restart: {
			TRACE_TWI( console.println( F( "TWI state_restart" )));

			//
			//	Generate a restart condition and move the
			//	action pointer forward to the next step.
			//
			_active->next = 0;	// Reset index into the buffer
			_active->action++;
			start();
			break;
		}
		case state_start_complete: {
			TRACE_TWI( console.println( F( "TWI state_start_complete" )));

			//
			//	We have waited for the start (or restart) to complete successfully.
			//	If it has completed then move to the next step in the machine.
			//      If not then redirect machine to the abort transaction code.
			//
			switch( _twsr ) {
				case TW_START:
				case TW_REP_START: {
					TRACE_TWI( console.println( F( "TWI start complete" )));
					
					//
					//	Move to next step in the machine.
					//
					_active->action++;
					break;
				}
				default: {
					TRACE_TWI( console.println( F( "TWI start failed" )));
					
					//
					//	Not a state we are anticipating, abort transaction
					//
					_active->action = abort_transaction;
					*( _active->result ) = error_starting;
					break;
				}
			}
			//
			//	We have not "done" anything so we go back to the start of the
			//	routine and read the next step in the machine.
			//
			goto loop;
		}
		case state_adrs_read: {
			TRACE_TWI( console.print( F( "TWI state_adrs_read " )));
			TRACE_TWI( console.println_hex( _active->target ));

			//
			//	We now send out the slave address from
			//	which we want to read data.
			//
			//	If we are only getting a single byte, then
			//	the Ack flag needs to be false, otherwise
			//	we set it to true.
			//
			send_target( _active->target, false );
			_active->action++;
			break;
		}
		case state_adrs_write: {
			TRACE_TWI( console.print( F( "TWI state_adrs_write " )));
			TRACE_TWI( console.println_hex( _active->target ));

			//
			//	We now send out the slave address with
			//	WRITE signified.
			//
			send_target( _active->target, true );
			_active->action++;
			break;
		}
		case state_adrs_ack: {
			TRACE_TWI( console.println( F( "TWI state_adrs_ack" )));

			//
			//	We are waiting for the address byte to
			//	complete, then we check the status to
			//	see if we were ack'd or nack'd by the slave.
			//
			switch( _twsr ) {
				case TW_MT_SLA_ACK:
				case TW_MR_SLA_ACK: {
					//
					//	The slave has Ackd their address byte
					//	for either read or write (this step in the
					//	master machine does not care which).  Move
					//	to the next step in the machine.
					//
					_active->action++;
					break;
				}
				case TW_MT_SLA_NACK:
				case TW_MR_SLA_NACK: {
					//
					//	The slave has NOT Ackd their address byte, so
					//	log an error and then move to the next step in
					//	the machine.
					//
					_active->action = abort_transaction;
					*( _active->result ) = error_address;
					break;
				}
				default: {
					//
					//	..and again, but a different error.
					//
					_active->action = abort_transaction;
					*( _active->result ) = error_transaction;
					break;
				}
			}
			goto loop;
		}
		case state_send_byte: {
			TRACE_TWI( console.print( F( "TWI state_send_byte " )));
			TRACE_TWI( console.println_hex( _active->buffer[ _active->next ]));

			//
			//	We have to send a byte to the slave
			//
			send_byte( _active->buffer[ _active->next++ ]);
			_active->action++;
			break;
		}
		case state_send_ack_loop: {
			TRACE_TWI( console.println( F( "TWI state_send_ack_loop" )));

			//
			//	Has the data been received?  The ack
			//	will tell us.
			//
			switch( _twsr ) {
				case TW_MT_DATA_ACK: {
					//
					//	The sent data byte has been Ackd.  If there
					//	are more data bytes to send go BACK one step,
					//	otherwise move forwards to the next step.
					//
					if( _active->next < _active->send ) {
						_active->action--;
					}
					else {
						_active->action++;
					}
					break;
				}
				case TW_MT_DATA_NACK: {
					//
					//	The data byte was NAckd, the write has failed.
					//
					_active->action = abort_transaction;
					*( _active->result ) = error_write_fail;
					break;
				}
				default: {
					//
					//	Other state, generate another error.
					//
					_active->action = abort_transaction;
					*( _active->result ) = error_transaction;
					break;
				}
			}
			goto loop;
		}
		case state_recv_ready: {
			TRACE_TWI( console.println( F( "TWI state_recv_ready" )));

			//
			//	Here we let the system know that we are
			//	ready to receive another byte from the
			//	slave and (through Ack/NAck) if this will
			//	be the last byte.
			//
			read_ack( _active->next < ( _active->recv-1 ));
			_active->action++;
			break;
		}
		case state_recv_byte_loop: {
			TRACE_TWI( console.print( F( "TWI state_recv_byte_loop " )));

			byte	data = read_byte();

			TRACE_TWI( console.println_hex( data ));

			//
			//	Save that byte of data!
			//
			if( _active->next < _active->recv ) {
				_active->buffer[ _active->next++ ] = data;
			}
			else {
				errors.log_error( TWI_READ_DATA_OVERFLOW, _active->next );
			}
			//
			//	Now choose what to do.
			//
			switch( _twsr ) {
				case TW_MR_DATA_ACK: {
					//
					//	We have a byte of data, the system
					//	has already sent an "Ack" (so this
					//	is not the last byte).  We roll
					//	back to the previous action.
					//
					_active->action--;
					break;
				}
				case TW_MR_DATA_NACK: {
					//
					//	We have a byte of data, the system
					//	has already sent an "NAck" (so this
					//	IS the last byte).  Move onto the
					//	next action.
					//
					_active->action++;
					break;
				}
				default: {
					//
					//	Some sort of transaction error.
					//
					_active->action = abort_transaction;
					*( _active->result ) = error_transaction;
					break;
				}
			}
			goto loop;
		}
		case state_stop: {
			TRACE_TWI( console.println( F( "TWI state_stop" )));

			//
			//	Generate a stop condition and move the
			//	action pointer forward to the next step.
			//
			stop();
			next_action();
			break;
		}
		default: {
			TRACE_TWI( console.println( F( "TWI default" )));

			//
			//	Everything that this module does should be
			//	handled before the default case.  This is
			//	and error!
			//
			ABORT( PROGRAMMER_ERROR_ABORT );
			
			//
			//	In the event that there has been coding
			//	error we could try recovery options...
			//
			_active->action = abort_transaction;
			*( _active->result ) = error_transaction;
			goto loop;
		}
	}
}

//
//	The routine called by the ISR to indicate an interrupt
//	has been received.
//
void TWI::irq( byte twsr ) {
	//
	//	1	Save the status register for later
	//
	//	2	Disable interrupts for now
	//
	//	3	Signal the event so the main line code can handle it.
	//
	_twsr = status( twsr );
	disable();
	_irq.release( true );
}



//
//	The TWI Object
//	==============
//
//	Define the object controlling the TWI device
//
TWI twi;

//
//	Two Wire Interrupt Service Routine.
//	===================================
//
//	This routine simply calls the generic event/state handler
//	passing in the state value being notified.
//
//
ISR( TWI_vect ) {
	COUNT_INTERRUPT;
	twi.irq( TWSR );
}


//
//	EOF
//

///
///	TWI_IO.h - A simplified TWI/I2C API library for Arduino
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

#ifndef _TWI_IO_H_
#define _TWI_IO_H_

#include <Arduino.h>
#include <inttypes.h>
#include <util/twi.h>

#ifndef TWI_FREQ
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
#define TWI_FREQ		10

//
//	Provide a symbol which will mean "the highest possible
//	speed"
//
//	At the moment "255" would mean approximately 2.5 MHz
//	which is an improbable TWI speed.
//
#define TWI_MAXIMUM_FREQ	255
#endif

//
//	Syntactic sugar
//
#ifndef FUNC
#define FUNC(f)	(*(f))
#endif

//
//	Define a period of Microseconds which the firmware will
//	wait during a hardware reset of the TWI
//
//	This is a last resort attempt to put things back into an
//	operation state.
//
#define TWI_HARDWARE_RESET_DELAY_US 50

//
//	The Setup Routines.
//	===================
//

//
//	Core basic configuration routines.
//

//
//	Call this routine in the setup() function.  Initialises the TWI
//	hardware, sets up the slave address, ability to reply to the general
//	call address and (optionally) enables/disables the internal
//	pull-up resistors.
//
//	adrs		The slave address to answer to (if non-zero)
//
//	gcall		If true (and adrs is non-zero) also answer calls
//			to the general call address.
//
//	isr		Set to true if the software has an ISR ready to
//			handle the TWI hardware asynchronously.  If set
//			to false then the software will poll the TWI hardware.
//
//	pullup		Enable or disable the on-board pull-up resistors
//			in the MCU.
//
extern void twi_init( byte adrs, bool gcall, bool isr, bool pullup );

//
//	Disables the TWI interface completely.
//
extern void twi_disable( void );

//
//	Set the TWI Clock frequency.
//
//	The frequency is passed in as a multiple
//	of 10KHz (i.e. if freq == 25 then clock = 250,000 Hz)
//
extern void twi_setFrequency( byte freq );

//
//	Return the identity of the best available
//	clock speed at (or below) the initial value
//	passed in.
//
extern byte twi_bestFrequency( byte freq );

//
//	Return the identity of the slowest available
//	clock speed.  This could be useful when a master
//	and a slave negotiate the best available
//	common speed .. you need to start from somewhere.
//
extern byte twi_lowestFrequency( void );

//
//	Set the software timeout for activities.  Time
//	specified in microseconds.
//
extern void twi_setTimeout( word us );

//
//	Provide an API for the users of the software to
//	collect error numbers when the routines detect an
//	error.
//
//	This does not have to be set, but if set the routine
//	called should execute quickly and on the assumption
//	that it *might* be called from within an ISR.
//
extern void twi_errorReporting( void FUNC( report )( byte error ));

//
//	These are the error codes passed to the error reporting
//	routine.
//
//		TWI_ERR_NONE		No Error
//		TWI_ERR_TIMED_OUT	Action timed out
//		TWI_ERR_QUEUE_FULL	Master transaction queue full
//		TWI_ERR_HW_RESET	TWI Hardware has been reset
//		TWI_ERR_ADDRESS		Slave address not acknowledged
//		TWI_ERR_WRITE_FAIL	Write data to slave not acknowledged
//		TWI_ERR_READ_FAIL	n/a
//		TWI_ERR_TRANSACTION	Unexpected state change in transaction
//		TWI_ERR_STARTING	Transaction start failed
//		TWI_ERR_TRUNCATED	Slave transaction truncated
//		TWI_ERR_OVERFLOW	Slave recv buffer overflowed, data lost
//		TWI_ERR_UNDERFLOW	Slave send buffer too small, zero filling
//		TWI_ERR_NO_BUFFER	No buffer space has been allocated
//
#define TWI_ERR_NONE		0
#define TWI_ERR_TIMED_OUT	1
#define TWI_ERR_QUEUE_FULL	2
#define TWI_ERR_HW_RESET	3
#define TWI_ERR_ADDRESS		4
#define TWI_ERR_WRITE_FAIL	5
#define TWI_ERR_READ_FAIL	6
#define TWI_ERR_TRANSACTION	7
#define TWI_ERR_STARTING	8
#define TWI_ERR_TRUNCATED	9
#define TWI_ERR_OVERFLOW	10
#define TWI_ERR_UNDERFLOW	11
#define TWI_ERR_NO_BUFFER	12


//
//	Declare a Master API
//	====================
//
//	All master API routines return true if the I2C/TWI action
//	has been successfully scheduled or false otherwise.
//

//
//	The (6.5.1) Quick Commands
//	--------------------------
//
extern bool twi_cmd_quick_read( byte adrs, void *link, void FUNC( reply )( bool valid, void *link, byte *buffer, byte len ));
extern bool twi_cmd_quick_write( byte adrs, void *link, void FUNC( reply )( bool valid, void *link, byte *buffer, byte len ));

//
//	All "just send data" commands
//	-----------------------------
//	(6.5.2) Send Byte
//	(6.5.4) Write Byte/Word
//	(6.5.7) Block Write/Read
//	(6.5.10) Write 32 protocol
//	(6.5.12) Write 64 protocol
//
extern bool twi_cmd_send_data( byte adrs, byte *buffer, byte send, void *link, void FUNC( reply )( bool valid, void *link, byte *buffer, byte len ));

//
//	The (6.5.3) Receive Byte
//	------------------------
//
extern bool twi_cmd_receive_byte( byte adrs, byte *buffer, void *link, void FUNC( reply )( bool valid, void *link, byte *buffer, byte len ));

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
extern bool twi_cmd_exchange( byte address, byte *buffer, byte send, byte recv, void *link, void FUNC( reply )( bool valid, void *link, byte *buffer, byte len ));


//
//	Declare the SLAVE API
//	=====================
//

//
//	The Slave API is simply assigning some static buffer space and
//	the address of a slave call back routine to the library.  The
//	use of these facilities is asynchronous from the remainder of the
//	MCU programming.
//

//
//	void twi_slaveFunction( byte *buffer, byte size, byte FUNC( answer )( byte adrs, byte *buffer, byte size, byte len ))
//	---------------------------------------------------------------------------------------------------------------------
//
//	This routine primes the "slave answering a master" exchange request with both
//	buffer space (for the in-coming request and out-going reply) and the reply routine
//	which is called to generate the necessary reply content.
//
//		adrs		The address the code is replying to (this will be either the
//				set slave address or the general call address if enabled)
//		buffer		The address of a byte array where received and returned data will be stored
//		size		The number of bytes in the buffer area
//		answer(		The function to call when a master starts an exchange with us
//			adrs	The address this device is responding to
//			buffer	The buffer space where the data sent is placed
//			size	The over all, maximum size of the buffer
//			len)	The number of bytes in the buffer making up the master request
//
//	The answer() function returns the number of bytes in the buffer containing the reply to
//	send back to the master.
//
extern void twi_slaveFunction( byte *buffer, byte size, byte FUNC( answer )( byte adrs, byte *buffer, byte size, byte len ));

//
//	The Management and Operation Routines.
//	======================================
//

//
//	void twi_eventProcessing( unsigned long now )
//	---------------------------------------------
//
//	IMPORTANT
//	=========
//
//	This routine is to be called as part of the main event loop of an
//	application, ideally as frequently as possible.
//
//	This routine supports key TWI actions:
//
//	1/	For the Master this routine will call the "reply" routine
//		ensuring that these are not caught up in the time and
//		space restrictions that being part of an ISR call could
//		create.
//
//	2/	For exchange requests initiated on the MCU running these
//		functions, this is the routine which moves requests from
//		the pending queue into actively being processed by the
//		hardware.
//
extern void twi_eventProcessing( void );

//
//	byte twi_queueLength( void )
//	----------------------------
//
//	Return the number of pending exchange requests plus one if
//	there is an active exchange.
//
extern byte twi_queueLength( void );

//
//	void twi_clearQueue( void )
//	---------------------------
//
//	Delete any queued (but inactive) exchange requests.
//
extern void twi_clearQueue( void );

//
//	void twi_synchronise( void )
//	----------------------------
//
//	Pause the program until all pending master transactions have
//	completed.
//
extern void twi_synchronise( void );

//
//	The following variable is exposed to provide an insight into
//	what might have failed in the event that an exchange fails
//	(the callback routine is called with valid == false).
//
extern byte twi_error;

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

#endif

//
//	EOF
//

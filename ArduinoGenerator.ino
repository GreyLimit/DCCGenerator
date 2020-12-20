///
///	ArduinoGenerator: Model railway DCC signal generator.
///
/// 	Firmware for an Arduino Uno R3 and Motorshield which
///	can generate a compliant NMRA DCC signal suitable for
///	the operation and control of mobile and accessory DCC
///	decoders.
///
///	Copyright (C) 2020, Jeff Penfold, jeff.penfold@googlemail.com
///	
///	This program is free software: you can redistribute it and/or modify
///	it under the terms of the GNU General Public License as published by
///	the Free Software Foundation, either version 3 of the License, or
///	(at your option) any later version.
///
///	This program is distributed in the hope that it will be useful,
///	but WITHOUT ANY WARRANTY; without even the implied warranty of
///	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
///	GNU General Public License for more details.
///
///	You should have received a copy of the GNU General Public License
///	along with this program.  If not, see <https://www.gnu.org/licenses/>.
///
 
//
//	Arduino DCC Generator
//	=====================
//
//	A sketch for generating the DCC signal through
//	an Arduino motor shield.
//
//	Hardware required:
//
//		Arduino UNO R3
//		Arduino Motor Shield
//		15 volt DC power supply
//

//
//	To implement partial compatibility with the Arduino
//	DCC Plus Plus firmware, located at:
//
//		https://github.com/DccPlusPlus/BaseStation
//
//	You should define DCC_PLUS_PLUS_COMPATIBILITY.
//
//	If not defined then a similar, though different, more
//	extensive and consistent set of commands are defined.
//
//#define DCC_PLUS_PLUS_COMPATIBILITY 1

//
//	Compilation note:
//
//	If when compiled for a target host you get the warning
//	about insufficient memory and potential instability issues
//	then the simplest (safest!) way to reduce memory consumption
//	is to reduce the number of bit buffers for each category
//	of use.  It is possible to reduce each to 1 (though no less),
//	but reducing the number of mobile buffers reduces the number
//	of engines which can be operated in parallel.
//
//	Values are defined at approximately line 320 and currently have
//	the following default values:
//
//		ACCESSORY_TRANS_BUFFERS	3
//		MOBILE_TRANS_BUFFERS	6
//		PROGRAMMING_BUFFERS	1
//

//
//	To implement an I2C connected LCD display, define the
//	following macros and values.
//
//	LCD_DISPLAY_ENABLE	Simple define to include required code.
//	LCD_DISPLAY_ROWS	Number of rows display has.
//	LCD_DISPLAY_COLS	Number of columns available per row.
//	LCD_DISPLAY_ADRS	The I2C address of the display
//
//	The display is assumed to be attached using a generic
//	PCF8575 I2C to Parallel adator.  The following default
//	definitions apply to a generic 20x4 display.
//
#define LCD_DISPLAY_ENABLE
#define LCD_DISPLAY_ROWS	4
#define LCD_DISPLAY_COLS	20
#define LCD_DISPLAY_ADRS	0x27
//
//	Define a set of single charactrer symbols to represet
//	actions/directions applied to decoders/accessories.
//
#define LCD_ACTION_FORWARD	'>'
#define LCD_ACTION_BACKWARDS	'<'
#define LCD_ACTION_ENABLE	'^'
#define LCD_ACTION_DISABLE	'v'

//
//	Finally, on LCDs..
//
//	From a wiring perspective (apart from +Vcc and Ground) the
//	display is attached to the arduino via the pass-through pins
//	D18/A4 and D19/A5 on the motor shield.  These equate to
//	the I2C/TWI interface pins SDA (D18) and SCL (D19).
//
//	To keep memory allocation to a minimum (at least on the basic
//	Arduino UNO) the "Lite" version of the Wire, TWI and LCD_I2C
//	libraries are required (found with this code).
//

//
//	Bring in the necessary IO and Interrupt definitions.
//
#include <avr/io.h>
#include <avr/interrupt.h>

//
//	Include the AVR Progmem access facilities.
//
#include <avr/pgmspace.h>

#ifdef LCD_DISPLAY_ENABLE

//
//	LCD Display libraries.
//
#include "TWI_Lite.h"
#include "Wire_Lite.h"
#include "LCD_I2C_Lite.h"

#endif

//
//	General universal constants.
//
#define NL	'\n'
#define CR	'\r'
#define EOS	'\0'
#define SPACE	' '
#define HASH	'#'
#define MINUS	'-'
#define PLUS	'+'
#define DELETE	'\177'
#define ERROR	(-1)

//
//	Define the size of a generic small textual buffer for
//	use on the stack.
//
#define TEXT_BUFFER 8

//
//	Debugging options
//	-----------------
//
//	Define any of the following macros to optionally include
//	at compile time additional code to output status and data
//	reports to the serial line.
//
//	This data will not be in a valid communications format so
//	it should be possible to run this firmware against the host
//	software.  Timing of the  firmware might be unavoidably
//	impacted.
//
//	DEBUG_BUFFER_MANAGER	Define to get status updates from
//				buffer management routine
//
//	DEBUG_BIT_SLICER	Define to get output from the routine
//				which converts a series of bytes into
//				the bits transitions which the time
//				critical code uses to generate the
//				output signal.
//
//	DEBUG_POWER_MONITOR	Define to get details of the power
//				monitoring code
//
//	DEBUG_STATISTICS	Collect and transmit statistics pertinent
//				to the frequency that sections of the code
//				are executed, per second.
//
//	DEBUG_BUFFER_TABLE	Send an outline breakdown of the bit
//				buffers to the Serial port in the event of
//				one of the bit buffer related errors
//				being generated.
//
//#define DEBUG_BUFFER_MANAGER	1
//#define DEBUG_BIT_SLICER	1
//#define DEBUG_POWER_MONITOR	1
//#define DEBUG_STATISTICS	1
//#define DEBUG_BUFFER_TABLE	1

//
//	Verification Code
//	-----------------
//
//	Define the macros ASSERT_VERIFICATION to include some software
//	verification code in the firmware.  In the event of conformance
//	checks failing errors will be produced.
//
#define ASSERT_VERIFICATION	1

//
//	Compatibility Support
//	---------------------
//
//	Used where "whole code" substitution would make the code
//	unnecessarily obscure.  "a" for native mode, "b" for
//	compatibility mode.
//
#ifdef DCC_PLUS_PLUS_COMPATIBILITY
#define SELECT_ALT(a,b)		b
#else
#define SELECT_ALT(a,b)		a
#endif

//
//	The following definitions allow for the similarity which is
//	present to be capitalised on and not cause the source code to
//	become unnecessarily convoluted.
//
//	PROT_IN_CHAR	Define the start and end characters of a sentence
//	PROT_OUT_CHAR	in the target operating system mode syntax.
//	
#define PROT_IN_CHAR		SELECT_ALT( '[', '<' )
#define PROT_OUT_CHAR		SELECT_ALT( ']', '>' )

//
//	Serial Communications.
//	----------------------
//
//	Define the Speed and buffer size used when accessing the
//	serial port.
//
//	Example baud rates:
//
//		9600 14400 19200 38400 57600 115200
//
#define SERIAL_BAUD_RATE	SELECT_ALT( 38400, 115200 )
#define SERIAL_BUFFER_SIZE	32

//
//	High Level Configuration values.
//	--------------------------------
//
//	These definitions alter operational elements
//	of the solution, but not the functional aspects.
//

//
//	Define the absolute current limit value (0-1023) at which
//	the power is removed from the track (current spike).
//
//	The average current limit is the maximum difference in
//	current and average reading allowed before a overload condition
//	is declared
//
#define INSTANT_CURRENT_LIMIT	750
#define AVERAGE_CURRENT_LIMIT	500

//
//	Define the minimum number of positive delta amps required for the
//	code to recognised a confirmation signal.
//
#define MINIMUM_DELTA_AMPS	35

//
//	Define the watchdog interval in milliseconds.
//
#define WATCHDOG_INTERVAL	2000

//
//	Define the maximum number of bytes in a DCC packet that we
//	can handle.
//
//	Note:	This is different from the number of bit transitions
//		which the system will support.  This figure (see
//		BIT_TRANSITIONS below) is based on a DCC command of
//		just four bytes.
//
#define MAXIMUM_DCC_COMMAND	6

//
//	Define a maximum number of characters that are required to
//	formulate the host reply to a command being received successfully.
//
#define MAXIMUM_DCC_REPLY	16

//
//	Define the number of times various types of packets are repeated.
//
//	TRANSIENT_COMMAND_REPEATS
//	
//		Operational, non-mobile, commands are considered "non-
//		permanent" DCC commands and are repeated before it is
//		automatically dropped.
//
//	SERVICE_MODE_RESET_REPEATS
//
//		The number of times a service mode reset command is
//		repeated before and after a service mode command.
//
//	SERVICE_MODE_COMMAND_REPEATS
//
//		The number of times a service mode action command is
//		repeated.
//
#define TRANSIENT_COMMAND_REPEATS	8
#define SERVICE_MODE_RESET_REPEATS	20
#define SERVICE_MODE_COMMAND_REPEATS	10

//
//	Define the number of buffers set aside for each of the
//	use cases for the buffers:
//
//		Accessory and transient commands
//		Mobile decoder speed and direction
//		Programming track specific
//
//	The program allocates buffers as follows (where A represents
//	the number of Accessory buffers and M the corresponding
//	number of mobile buffers):
//
//		0 .. A-1	Accessory, transient DCC packets
//		A .. A+M-1	Mobile, persistent DCC packets
//		A+M .. A+M+P-1	Programming track buffers.
//
#define ACCESSORY_TRANS_BUFFERS	3
#define MOBILE_TRANS_BUFFERS	6
#define PROGRAMMING_BUFFERS	1

//
//	Define short hand base buffer numbers for each section.
//
#define ACCESSORY_BASE_BUFFER	0
#define MOBILE_BASE_BUFFER	(ACCESSORY_BASE_BUFFER+ACCESSORY_TRANS_BUFFERS)
#define PROGRAMMING_BASE_BUFFER	(MOBILE_BASE_BUFFER+MOBILE_TRANS_BUFFERS)

//
//	Define the total number of transmission buffers that will
//	form the circular transmission loop.
//
//	This is a composition of the number of buffer allocated
//	to each section.
//
#define TRANSMISSION_BUFFERS	(PROGRAMMING_BASE_BUFFER+PROGRAMMING_BUFFERS)

//
//	Define the number of "1"s transmitted by the firmware
//	forming the "preamble" for the DCC packet itself.
//
//	The DCC standard specifies a minimum of 14 for normal
//	commands and 20 for programming commands.  These are the
//	short and long preambles.
//
#define DCC_SHORT_PREAMBLE	15
#define DCC_LONG_PREAMBLE	30

//
//	Various DCC protocol based values
//
#define MAXIMUM_DCC_ADDRESS	10239
#define MAXIMUM_SHORT_ADDRESS	127
#define MINIMUM_DCC_ADDRESS	1
//
#define MAXIMUM_DCC_SPEED	126
#define MINIMUM_DCC_SPEED	0
#define EMERGENCY_STOP		(-1)
//
#define DCC_FORWARDS		1
#define DCC_BACKWARDS		0
//
//	Internal DCC Accessory addresses.  The address structure
//	as defined in the DCC protocol.
//
#define MIN_ACCESSORY_ADDRESS	0
#define MAX_ACCESSORY_ADDRESS	511
#define MIN_ACCESSORY_SUBADRS	0
#define MAX_ACCESSORY_SUBADRS	3
//
//	External combined DCC accessory address.  The address range
//	as commonly used by external software systems.
//
#define MIN_ACCESSORY_EXT_ADDRESS	1
#define MAX_ACCESSORY_EXT_ADDRESS	2048
//
#define ACCESSORY_ON		1
#define ACCESSORY_OFF		0
//
//	The DCC standard specifies CV values between 1 and 1024,
//	but the actual "on wire" protocol utilised the values 0
//	to 1023.
//
#define MINIMUM_CV_ADDRESS	1
#define MAXIMUM_CV_ADDRESS	1024
//
//	Function numbers within a decoder
//
#define MIN_FUNCTION_NUMBER	0
#define MAX_FUNCTION_NUMBER	28
//
#define FUNCTION_ON		1
#define FUNCTION_OFF		0

//
//	Motor Shield definitions.
//	-------------------------
//
//	Hardware characteristics of the "Deek-Robot" Motor
//	Shield, which should be compatible with the Arduino
//	version (and others).
//
//	A and B drivers available, each with 4 pins allocated.
//
//	These are the pin numbers "as per the motor shield".
//
//	The "_ANALOGUE" values are the numerical part of the
//	preceding "_LOAD" pin numbers, and are required as
//	part of the interrupts driven current monitoring. 
//
//	IMPORTANT:
//
//		You *must* cut the VIN CONNECT and BRAKE A
//		and B traces on the back of the Motor Shield.
//		You will need to power the shield separately
//		from the Arduino with 15 volts DC, and leaving
//		the VIN CONNECT in place *will* put 15 volts
//		across the Arduino and cook it.
//
//		The BRAKE feature (for both A and B H-Bridges)
//		is a system for deliberately shorting the
//		output of an H-Bridge to cause a rapid
//		stop of a DC electric motor.  Since we are
//		not directly driving electric motors these too
//		should be cut.
//
//		This firmware requires no additional jumpers
//		for its intended operation.
//
#define SHIELD_DRIVER_A_DIRECTION	12
#define SHIELD_DRIVER_A_INPUT		3
#define SHIELD_DRIVER_A_BRAKE		9
#define SHIELD_DRIVER_A_LOAD		A0
#define SHIELD_DRIVER_A_ANALOGUE	0

#define SHIELD_DRIVER_B_DIRECTION	13
#define SHIELD_DRIVER_B_INPUT		11
#define SHIELD_DRIVER_B_BRAKE		8
#define SHIELD_DRIVER_B_LOAD		A1
#define SHIELD_DRIVER_B_ANALOGUE	1

//
//	Define the physical pins linking this firmware to the
//	motor shield attached to the Arduino.  This is where
//	you can choose which H-Bridge to associate with the
//	"main" track and the "programming" track.
//
//	The "Direction" pins are used to create the Square
//	Wave format of the electrical signal (as per the DCC
//	specification).  The "Enable" pin provide a complete
//	On/Off capability.
//
//	The "load" pins are read as an analogue value (between
//	0 and 1023) and represent (in some fashion) the amps/load
//	passing through the respective H-Bridge.
//
//	As stated above, the "analogue" pins represent an
//	alternative reference to the load pins.
//
#define MAIN_TRACK_DIRECTION	SHIELD_DRIVER_A_DIRECTION
#define MAIN_TRACK_ENABLE	SHIELD_DRIVER_A_INPUT
#define MAIN_TRACK_BRAKE	SHIELD_DRIVER_A_BRAKE
#define MAIN_TRACK_LOAD		SHIELD_DRIVER_A_LOAD
#define MAIN_TRACK_ANALOGUE	SHIELD_DRIVER_A_ANALOGUE
//
#define PROG_TRACK_DIRECTION	SHIELD_DRIVER_B_DIRECTION
#define PROG_TRACK_ENABLE	SHIELD_DRIVER_B_INPUT
#define PROG_TRACK_BRAKE	SHIELD_DRIVER_B_BRAKE
#define PROG_TRACK_LOAD		SHIELD_DRIVER_B_LOAD
#define PROG_TRACK_ANALOGUE	SHIELD_DRIVER_B_ANALOGUE

//
//	Timing, Protocol and Data definitions.
//	--------------------------------------
//

//
//	The following paragraphs and numerical calculations are
//	based on a spread sheet used to analyse the possible
//	subdivisions of the DCC signal timing (spread sheet not
//	included).
//
//	The key point of the analysis is to find a common divisor
//	between the duration of half a "1" bit (58 us) and half a
//	"0" bit (100 us) based on the basic clock frequency of
//	the Arduino and ideally giving an interval count sum that
//	fits into an 8-bit interrupt counter.
//
//	The analysis determined that dividing the "1" unit of time
//	(58 us) by 4 gave a period of 14.5 us, which divides into
//	the "0" unit of time into 7 with an acceptable margin of
//	error (~1.5%).
//

//
//	This macro defines the number of Clock Cycles that the
//	interrupt timer must count before raising an interrupt
//	to create a target interval of 14.5 us.
//
//	This period we are calling a "tick", multiples of which
//	are used to build up the intervals required to create
//	the DCC signal.
//
//	Simplistically the "MHz" clock rate of the board multiplied
//	by the selected "tick" duration (in microseconds) gives the
//	answer:
//
//		16 (MHz) x 14.5 (microseconds) = 232 (clock cycles)
//
#define TIMER_CLOCK_CYCLES	232

//
//	The following two macros return the number of interrupt
//	ticks which are required to generate half of a "1" or
//	half of a "0" bit in the DCC signal.
//
//	Within the DCC standard, the "half time" of a "1" bit is
//	58 us:
//		4 ticks of 14.5 us are exactly 58 us.
//
//	Like wise, the nominal "half time" of a "0" bit is 100 us,
//	so:
//		7 ticks of 14.5 us gives a time of 101.5 us.
//
#define TICKS_FOR_ONE	4
#define TICKS_FOR_ZERO	7

//
//	Assert verification code
//	------------------------
//
//	Are we including basic software verification?
//
//	As this is not software, but firmware, there is no ability to
//	simply stop the software on detection of an assertion failure
//	with the intention of analysing the system.
//
//	A valid option would be to implement a routine which would continuously
//	output an appropriate error but never exit, so freezing the firmware.
//
//	I haven't done this.
//
//	In the event of an assertion failure I have chosen to log an error
//	with the error subsystem and hope that this is sent out before the
//	firmware ceases operating.
//
#ifdef ASSERT_VERIFICATION

//
//	Pre-define the report_error routine.  We have to do this
//	as the error reporting system is much further down the program.
//
static void report_error( int err, int arg );

//
//	Add an assert error number and an assertion macro to
//	use in the code.
//
#define ASSERT_FAILED		99
#define ASSERT(v)		if(!(v))report_error(ASSERT_FAILED,__LINE__)

#else

//
//	...or not.
//
#define ASSERT(v)

#endif

//
//	LCD structure
//	-------------
//
#ifdef LCD_DISPLAY_ENABLE

//
//	Create the LCD interface object.
//
static LCD_I2C_Lite lcd( LCD_DISPLAY_ADRS, LCD_DISPLAY_COLS, LCD_DISPLAY_ROWS );

//
//	Outline description of the  LCD display
//	---------------------------------------
//
//	The dimentions and field sizes for the display are set here as they
//	have size implications for other data structures following.
//
//	The code will *attempt* to be compile time sensitive to the
//	dimensions of the display (as set in LCD_DISPLAY_ROWS and _COLS)
//	This may not result in a pleasing/balanced display in all cases.
//
//	The code has been organised to target a 20 column by 4 row display.
//
//	A "drawing" of the target output display:
//
//	+--------------------+	The STATUS area of the display, showing:
//	|SSSSSS              |	The power (L)oad average
//	|SSSSSS              |	The available (F)ree bit buffers and (P)ower status
//	|SSSSSS              |	DCC packets (T)ransmitted sent per second
//	|SSSSSS              |	The (U)ptime in seconds
//	+--------------------+	
//
//	+--------------------+	The BUFFER area of the display, showing:
//	|      BBBBBBBBBBBBBB|	Buffers in use and the action in place.
//	|      BBBBBBBBBBBBBB|
//	|      BBBBBBBBBBBBBB|
//	|      BBBBBBBBBBBBBB|
//	+--------------------+	
//
//	The following definitions define some parameters which
//	shape the output ot the LCD.
//
#define LCD_DISPLAY_STATUS_WIDTH	6
#define LCD_DISPLAY_BUFFER_WIDTH	7
#define LCD_DISPLAY_BUFFER_COLS		((LCD_DISPLAY_COLS-LCD_DISPLAY_STATUS_WIDTH)/LCD_DISPLAY_BUFFER_WIDTH)


#endif

//
//	Pending DCC Packet data structure.
//	----------------------------------
//
//	The following constant and structure definitions provide
//	a structured and extensible mechanism for inserting one
//	or more (as a series) of packets into a transmission
//	buffer for broadcasting onto either the operations or
//	programming track.
//

//
//	Define the data structure used to hold a single pending
//	DCC packet.  The field defined are:
//
//	target		The new value for target upon setting up a new
//			bit stream.
//
//	lpreamble	true if the packet should have a long preamble.
//
//	duration	This is the new value for duration.
//
//	len		Length of the command in bytes.
//
//	command		This is the series of bytes which form the "byte"
//			version of the command to be sent.
//
//	next		Pointer to next packet to send (or NULL).
//
#define PENDING_PACKET struct pending_packet
PENDING_PACKET {
	int		target;
	bool		lpreamble;
	byte		duration,
			len,
			command[ MAXIMUM_DCC_COMMAND ];
	PENDING_PACKET	*next;
};

//
//	Define the number of pending buffers which will be available.
//
//	For the moment, we will allocate the same number as there are
//	transmission buffers.  This should be ample as persistent commands
//	will not tie up a pending packet leaving the pending packet
//	buffers mostly free for transient commands which by their nature
//	only tie up pending packets for short periods of time.
//
#define PENDING_PACKETS	TRANSMISSION_BUFFERS

//
//	Define the the pending packet records and the head of the free
//	records list.
//
static PENDING_PACKET pending_dcc_packet[ PENDING_PACKETS ];
static PENDING_PACKET *free_pending_packets;

//
//	Copy a DCC command to a new location and append the parity data.
//	Returns length of data in the target location.
//
static byte copy_with_parity( byte *dest, byte *src, byte len ) {
	byte	p, i;

	ASSERT( len > 0 );

	p = 0;
	for( i = 0; i < len; i++ ) p ^= ( *dest++ = *src++ );
	*dest = p;
	return( len+1 );
}

//
//	Define standard routine to obtain and fill in a pending
//	packet record.
//
static bool create_pending_rec( PENDING_PACKET ***adrs, int target, byte duration, bool lpreamble, byte len, byte *cmd ) {
	PENDING_PACKET	*ptr, **tail;

	ASSERT( adrs != NULL );
	ASSERT( *adrs != NULL );
	ASSERT( len > 0 );
	ASSERT( len < MAXIMUM_DCC_COMMAND );
	ASSERT( cmd != NULL );
	
	//
	//	We work on being handed the address of the pointer to
	//	the tail of the list of pending records.  Just a simple
	//	triple indirection definition.
	//
	//	We return true if the record has been created and linked
	//	in correctly, false otherwise. 
	//
	if(( ptr = free_pending_packets ) == NULL ) return( false );
	free_pending_packets = ptr->next;
	//
	//	There is a spare record available so fill it in.
	//
	ptr->target = target;
	ptr->lpreamble = lpreamble;
	ptr->duration = duration;
	ptr->len = copy_with_parity( ptr->command, cmd, len );
	//
	//	Now link it in.
	//
	tail = *adrs;

	ASSERT( *tail == NULL );

	*tail = ptr;
	tail = &( ptr->next );
	*tail = NULL;
	*adrs = tail;
	//
	//	Done.
	//
	return( true );
}
//
//	Define a routine to release one (one=true) or all (one=false) pending packets in a list.
//
static PENDING_PACKET *release_pending_recs( PENDING_PACKET *head, bool one ) {
	PENDING_PACKET	*ptr;

	//
	//	We either release one record and return the address of the remaining
	//	records (one is true) or we release them all and return NULL (one is false).
	//
	while(( ptr = head ) != NULL ) {
		head = ptr->next;
		ptr->next = free_pending_packets;
		free_pending_packets = ptr;
		if( one ) break;
	}
	//
	//	Done.
	//
	return( head );
}

//
//	DCC Accessory Address conversion
//	--------------------------------
//
//	Define a routine which, given an accessory target address and sub-
//	address, returns a numerical value which is the external unified
//	equivalent value.
//

#ifdef DCC_PLUS_PLUS_COMPATIBILITY

static int external_acc_target( int adrs, int subadrs ) {

	ASSERT( adrs >= MIN_ACCESSORY_ADDRESS );
	ASSERT( adrs <= MAX_ACCESSORY_ADDRESS );
	ASSERT( subadrs >= MIN_ACCESSORY_SUBADRS );
	ASSERT( subadrs <= MAX_ACCESSORY_SUBADRS );

	return((( adrs << 2 ) | subadrs ) + 1 );
}

#else

//
//	Define two routines which, given and external accessory number,
//	return the DCC accessory address and sub-address.
//
static int internal_acc_adrs( int target ) {

	ASSERT( target >= MIN_ACCESSORY_EXT_ADDRESS );
	ASSERT( target <= MAX_ACCESSORY_EXT_ADDRESS );

	return(( target - 1 ) >> 2 );
}
static int internal_acc_subadrs( int target ) {

	ASSERT( target >= MIN_ACCESSORY_EXT_ADDRESS );
	ASSERT( target <= MAX_ACCESSORY_EXT_ADDRESS );

	return(( target  - 1 ) & 3 );
}

#endif

#ifndef DCC_PLUS_PLUS_COMPATIBILITY

//
//	Decoder Function value cache
//	----------------------------
//
//	Native DCC Generator code requires a function status cache to
//	support modification of an individual decoder function.  This
//	is required as (as far as I can tell) there is no mechanism to
//	allow individual function adjustment without setting/resetting
//	between 3 to 7 other functions at the same time.
//

//
//	Define a number of cache records and bytes for bits storage in
//	each record.  We calculate FUNCTION_BIT_ARRAY based on the
//	MIN and MAX function numbers provided (the 7+ ensures correct
//	rounding in boundary cases).
//
//	We will base the function cache size on the maximum number of
//	DCC mobile decoders we can have active in parallel.
//
#define FUNCTION_CACHE_RECS	MOBILE_TRANS_BUFFERS
#define FUNCTION_BIT_ARRAY	((7+1+MAX_FUNCTION_NUMBER-MIN_FUNCTION_NUMBER)>>3)

//
//	The structure used to cache function values per decoder, so that
//	the "block" function setting DCC packet can be used.
//
#define FUNCTION_CACHE struct func_cache
FUNCTION_CACHE {
	int		target;
	byte		bits[ FUNCTION_BIT_ARRAY ];
	FUNCTION_CACHE	*next,
			**prev;
};
static FUNCTION_CACHE	function_rec[ FUNCTION_CACHE_RECS ],
			*function_cache;
//
//	Function to initialise the cache records empty.
//
//	We will "pre-fill" the cache with empty records so that the code can
//	always assume that there are records in the cache, because there are.
//
static void init_function_cache( void ) {
	FUNCTION_CACHE	**tail, *ptr;
	
	tail = &function_cache;
	for( byte i = 0; i < FUNCTION_CACHE_RECS; i++ ) {
		//
		//	Note current record.
		//
		ptr = function_rec + i;
		//
		//	Empty the record.
		//
		ptr->target = 0;
		for( byte j = 0; j < FUNCTION_BIT_ARRAY; ptr->bits[ j++ ] = 0 );
		//
		//	Link in the record.
		//
		*tail = ptr;
		ptr->prev = tail;
		tail = &( ptr->next );
	}
	*tail = NULL;
}

//
//	Define the lookup and manage cache code.
//
static FUNCTION_CACHE *find_func_cache( int target ) {
	FUNCTION_CACHE	**adrs,
			*last,
			*ptr;

	ASSERT( target >= MINIMUM_DCC_ADDRESS );
	ASSERT( target <= MAXIMUM_DCC_ADDRESS );

	adrs = &function_cache;
	last = NULL;
	while(( ptr = *adrs ) != NULL ) {
		//
		//	Is this the one?
		//
		if( target == ptr->target ) {
			//
			//	Yes, so move to top of the list (if not already there)
			//	so that access to this record is as quick as possible
			//	for subsequent requests.
			//
			if( function_cache != ptr ) {
				//
				//	Detach from the list.
				//
				if(( *( ptr->prev ) = ptr->next )) ptr->next->prev = ptr->prev;
				//
				//	Add to head of list.
				//
				ptr->next = function_cache;
				function_cache->prev = &( ptr->next );
				function_cache = ptr;
				ptr->prev = &function_cache;
			}
			return( ptr );
		}
		//
		//	Note last record we saw.
		//
		last = ptr;
		adrs = &( ptr->next );
	}
	//
	//	Nothing found, so we re-use the oldest record in the list.
	//	Start by unlinking it from the end of the list.
	//
	*( last->prev ) = NULL;
	//
	//	Replace with new target and empty function settings.
	//
	last->target = target;
	for( byte i = 0; i < FUNCTION_BIT_ARRAY; last->bits[ i++ ] = 0 );
	//
	//	Link onto head of cache.
	//
	last->next = function_cache;
	function_cache->prev = &( last->next );
	function_cache = last;
	last->prev = &function_cache;
	//
	//	Done.
	//
	return( last );
}

//
//	Routine applies a boolean value for a specified function
//	on a specified target number.  Returns true if the update
//	resulted in a change of value.
//
static bool update_function( int target, byte func, bool state ) {
	FUNCTION_CACHE	*ptr;
	byte		i, b; 

	ASSERT( func <= MAX_FUNCTION_NUMBER );

	ptr = find_func_cache( target );
	i = ( func - MIN_FUNCTION_NUMBER ) >> 3;
	b = 1 << (( func - MIN_FUNCTION_NUMBER ) & 7 );

	if( state ) {
		//
		//	Bit set already?
		//
		if( ptr->bits[ i ] & b ) return( false );
		//
		//	Yes.
		//
		ptr->bits[ i ] |= b;
		return( true );
	}
	//
	//	Bit clear already?
	//
	if(!( ptr->bits[ i ] & b )) return( false );
	//
	//	Yes.
	//
	ptr->bits[ i ] &= ~b;
	return( true );
}

//
//	Routine returns 0 or the supplied value based on the
//	supplied function number being off or on.
//
static byte get_function( int target, byte func, byte val ) {
	FUNCTION_CACHE	*ptr;
	byte		i, b; 

	ASSERT( func <= MAX_FUNCTION_NUMBER );

	ptr = find_func_cache( target );
	i = ( func - MIN_FUNCTION_NUMBER ) >> 3;
	b = 1 << (( func - MIN_FUNCTION_NUMBER ) & 7 );

	if( ptr->bits[ i ] & b ) return( val );
	return( 0 );
}

#endif

//
//	Storage of Transmission Data
//	----------------------------
//
//	Output bit patterns are stored as an array of bytes
//	where alternate bytes indicate the number of "1"s or "0"s
//	to output (array index 0 represents an initial series of
//	"1"s).
//
//	While this method is less space efficient than simply
//	storing the bytes required, it has the benefit that the
//	inter-byte protocol "0"s and "1"s are easily captured, and
//	interrupt code required to generate a DCC packet has fewer
//	special case requirements and so will be faster.
//
//	This inevitably (and correctly) offloads processing work
//	from the interrupt routine to the non-time-critical general
//	IO code.
//

//
//	Define limits on the number of bit transitions which a
//	transmission buffer can contain.
//
//	The maximum size for a DCC command that this program can
//	process is defined as MAXIMUM_DCC_COMMAND, however this is
//	and extreme case, and so the bit transition array will be
//	based (for the moment) on a series of 4 bytes.
//
//	Worst case scenario for a DCC packet is a long sequence of
//	bytes of either 0xAA or 0x55 (individually alternating bits).
//
//	In this case the bit array would be filled thus:
//
//	Content					Ones	Zeros
//	-------					----	-----
//	Packets header of 'X' "1"s + 1 "0"	X	1
//	(X is selected preamble length)
//
//	Byte 0xAA + 1 "0"			1	1
//						1	1
//						1	1
//						1	2
//
//	Byte 0xAA + 1 "0"			1	1
//						1	1
//						1	1
//						1	2
//
//	Byte 0xAA + 1 "0"			1	1
//						1	1
//						1	1
//						1	2
//
//	Checksum 0xAA + 1 "1"			1	1
//						1	1
//						1	1
//						1	1
//
//	End of packet "1"			1
//	
//
//	In this situation a minimum 36 byte bit transition array would be
//	required for the data (remember additional space required for end
//	zero byte).
//
//	Statistically this would normally hold a larger DCC packet than
//	this.
//
#define BIT_TRANSITIONS		40

//
//	Define maximum bit iterations per byte of the bit transition array.
//
//	It is possible to prove that this figure can never be reached as this
//	would imply a a series of 28 byte containing just 0 bits which (apart
//	from being an invalid DCC command, is over 4 time longer than the
//	longest DCC command this code will handle.
//
//	This value is applied inside verification assert statements.
//
#define MAXIMUM_BIT_ITERATIONS	255

//
//	Define the state information which is used to control the transmission
//	buffers.
//
//	The states are divided into a number of groups reflecting which section
//	of code monitors or controls it.  The concept here is that (like a game
//	of "ping pong") the responsibility for the buffers is handed back and
//	forth between the bits of code.  The intention here is to avoid requiring
//	locked out sections of code (using cli() and sei()) which might impact
//	the over all timing of the firmware.
//
//	In the following table:
//
//		State		The name of the state in this code.
//		Monitor		Which Code section operates in this state
//				(i.e. acts upon a buffer in that state).
//		Role		Briefly what is going when a buffer is moved
//				from its initial to result state).
//		Result		State into which the code moves the buffer.
//
//	Trying to capture state transitions here:
//
//	State		Monitor		Role			Result
//	-----		-------		----			------
//	TBS_RUN		ISR		Continuous packet	TBS_RUN
//					transmission
//					(duration is zero)
//
//	TBS_RUN		ISR		Limited packet		TBS_RUN
//					transmission
//					(--duration > 0 )
//
//	TBS_RUN		ISR		Limited packet		TBS_LOAD
//					transmission
//					(--duration == 0 )
//
//	TBS_LOAD	Manager		Pending pointer not	TBS_RUN
//					NULL, load next bit
//					sequence from record
//					and drop pending record
//
//	TBS_LOAD	Manager		Pending pointer is	TBS_EMPTY
//					NULL.
//
//	TBS_EMPTY	IO		Create a list of	TBS_LOAD
//					pending records and
//					attach to a buffer
//
//	TBS_RELOAD	ISR		Synchronised load	TBS_LOAD
//					initiated by IO code
//					on a TBS_RUN buffer
//
#define TBS_EMPTY	0
#define TBS_LOAD	1
#define TBS_RUN		2
#define TBS_RELOAD	3

//
//	Define a Transmission Buffer which contains the following
//	elements:
//
//	state		The current state of this buffer.  Primary method
//			synchronise activities between the interrupt driven
//			signal generator code and the iteratively called
//			management code.
//
//	Live Transmission fields:
//	-------------------------
//
//	target		The DCC ID being targetted by the buffer:
//			+ve -> mobile ID, -ve ->  negated External
//			Accessory ID, 0 Broadcast address or programming
//			track.
//
//	duration	Provides a mechanism between the main IO code
//			and the Interrupt driven DCC transmission code
//			for scheduling the disabling the buffer.
//
//			If set as zero the buffer will run continuously.
//
//			If set to a non-zero value, the content
//			of duration will be decreased each time the buffer
//			is sent.  When it reaches zero the buffer is
//			then disabled and made empty.
//
//	bits		The bit definition of a the DCC command string which
//			is to be broadcast as a series of 1/0 transitions.
//			Terminated with a zero byte.
//
//	Pending Transmission Fields:
//	----------------------------
//
//	pending		Address of the pending data record containing the next
//			command to load after this one is completed.  There is
//			an implication that if durations is 0 then this must
//			be NULL.
//
//	Confirmation reply data.
//	------------------------
//
//	reply		Define how the system should manage/generate a reply
//			to the host computer.
//
//			NO_REPLY_REQUIRED	No reply required.
//
//			REPLY_ON_CONFIRM	On detection of response
//						from the decoder.  This is
//						sent when the duration count
//						reaches zero and there are no
//						more pending packets to send.
//						At this point a check is made
//						to see if a confirmation has
//						been noted and a suitable
//						reply sent.
//
//			REPLY_ON_SEND		Send the reply once the
//						last pending record has been
//						loaded into the bit buffer
//						and is about to be sent.
//
#define			NO_REPLY_REQUIRED	0
#define			REPLY_ON_CONFIRM	1
#define			REPLY_ON_SEND		2
//
//	contains	The (EOS terminated) reply string which should
//			form the reply sent.  If this contains a HASH in
//			it and the reply is confirmation based the HASH
//			will be replaced with 1 if there was a confirmation
//			or 0 otherwise.  If there is no HASH in the reply
//			then it is only sent if a confirmation was seen.
//
//	Link to the next buffer, circular fashion:
//	------------------------------------------
//
//	next		The address of the next transmission buffer in
//			the loop
//
#define TRANS_BUFFER struct trans_buffer
TRANS_BUFFER {
	//
	//	The current state of this buffer.  Primary method used to
	//	synchronise activities between the interrupt driven signal
	//	generator code and the iteratively called management code.
	//
	byte		state;
	//
	//	Live Transmission fields:
	//	-------------------------
	//
	//
	//	The target ID of the packet.  Note the following usage:
	//
	//		target < 0	Accessory Decoder (negate to get  ext ID)
	//		target == 0	Broadcast address
	//		target > 0	Mobile Decoder
	//
	int		target;
	//
	//	The duration countdown (if non-zero).  An initial value
	//	of zero indicates no countdown meaning the packet is transmitted
	//	indefinitely.
	//
	byte		duration;
	//
	//	The bit pattern to transmit.  This is only filled
	//	from the command field by the buffer management code
	//	so that it can be synchronised with operation of the
	//	interrupt routine.
	//
	//	Bit transitions are zero byte terminated, no length
	//	need be maintained.
	//
	byte		bits[ BIT_TRANSITIONS ];
	//
	//	Pending Transmission Fields:
	//	----------------------------
	//
	//	Address of the next pending DCC command to send, NULL
	//	if nothing to send after this bit pattern.
	//
	PENDING_PACKET	*pending;
	//
	//	Confirmation reply data.
	//	------------------------
	//
	//	When is a reply required and what does that reply contain?  This
	//	is applied only at the end of a series of pending records (ie when
	//	pending is NULL).
	//
	byte	reply;
	char	contains[ MAXIMUM_DCC_REPLY ];
	
#ifdef LCD_DISPLAY_ENABLE
	//
	//	Liquid Crystal Display Data
	//	---------------------------
	//
	char	display[ LCD_DISPLAY_BUFFER_WIDTH ];
#endif

	//
	//	Buffer linkage.
	//	---------------
	//
	//	Finally, the link to the next buffer
	//
	TRANS_BUFFER	*next;
};

//
//	Statistic collection variables
//	------------------------------
//
//	Variables conditionally defined when the statistics collection
//	has been compiled in.
//

#if defined( DEBUG_STATISTICS )| defined( LCD_DISPLAY_ENABLE )

static int	statistic_packets;

#endif
#ifdef DEBUG_STATISTICS

static int	statistic_power;
static long	statistic_looping;	// This value grows too fast for an int.

#endif

//
//	Output Cache and transmission Code.
//	-----------------------------------
//
//	This code is all about sending data back
//	to the connected computer.  A small buffer
//	is used to decouple the generation of the
//	return messages from the hardware required
//	to send it.
//

//
//	Define the size of the output queue.  This is the number of
//	characters of data that are awaiting an opportunity to be sent
//	to the host computer.
//
#define OUTPUT_QUEUE_SIZE	64

//
//	Define the output queue itself.  This is a
//	circular buffer controlled by 3 variables:
//
//		queue_in:	Where to add data.
//		queue_out:	Where to remove data.
//		queue_len:	How much data pending.
//
//	The buffer itself is queue_data.
//
static char	queue_data[ OUTPUT_QUEUE_SIZE ];
static byte	queue_in = 0,
		queue_out = 0,
		queue_len = 0;

//
//	This routine is called by the main loop
//	when the Serial port has capacity for
//	transmitting additional data.  Count
//	provides the maximum number of bytes which
//	can be sent without blocking the firmware.
//
static void process_output( int count ) {
	//
	//	We loop until until we have either
	//	filled the hardware buffer or used
	//	all the queued data
	//
	while( queue_len && count ) {
		//
		//	Queue_len and count are both
		//	non-zero: output something!
		//
		Serial.write( queue_data[ queue_out++ ]);
		if( queue_out == OUTPUT_QUEUE_SIZE ) queue_out = 0;
		queue_len--;
		count--;
	}
}

//
//	Define a routine which accepts output
//	to be sent (once the Serial port has
//	output capacity available).
//
//	Returns true if all text cached OK,
//	false if some has been dropped.
//
static int queue_output( const char *buf ) {
	char	c;
	
	while(( c = *buf++ )) {
		if( queue_len >= OUTPUT_QUEUE_SIZE ) return( false );
		queue_data[ queue_in++ ] = c;
		if( queue_in == OUTPUT_QUEUE_SIZE ) queue_in = 0;
		queue_len++;
	}
	return( true );
}

//
//	Define a routine which returns the available
//	capacity of the output queue.
//
static int queue_available( void ) {
	return( OUTPUT_QUEUE_SIZE - queue_len );
}

//
//	Generalised data formatting routines.
//	-------------------------------------
//
//	Some routines which provide simple conversion of data to
//	text in a manner that can be simply combined; the routines
//	return the first address after the last character generated.
//
static char *int_to_text( char *buf, int v ) {
	byte	r[ TEXT_BUFFER ], c;
	
	if( v == 0 ) {
		*buf++ = '0';
		return( buf );
	}
	if( v < 0 ) {
		v = -v;
		*buf++ = '-';
	}
	c = 0;
	while( v ) {
		r[ c++ ] = v % 10;
		v /= 10;
	}
	while( c ) *buf++ = '0' + r[ --c ];
	return( buf );
}

#ifdef LCD_DISPLAY_ENABLE

//
//	This "back fill" integer to text routine is used only
//	by the LCD update routine.  Returns true if everything
//	fits, false otherwise
//
static bool backfill_int_to_text( char *buf, int v, byte len ) {
	bool	n;	// Negative flag.

	ASSERT( buf != NULL );
	ASSERT( len > 0 );

	//
	//	Cut out the "0" case as it need special handling.
	//
	if( v ) {
		//
		//	Prepare for handling negative number
		//
		if(( n = ( v < 0 ))) v = -v;
		//
		//	loop round pealing off the digits
		//
		while( len-- ) {
			buf[ len ] = '0' + ( v % 10 );
			if(!( v /= 10 )) break;
		}
		//
		//	If v is not zero, or if len is zero and
		//	the negative flag is set, then we cannot
		//	fit the data into the available space.
		//
		if( v ||( n && ( len < 1 ))) return( false );
		//
		//	Insert negative symbol if required.
		//
		if( n ) buf[ --len ] = '-';
	}
	else {
		//
		//	Zero case easy to handle.
		//
		buf[ --len ] = '0';
	}
	//
	//	Space pad rest of buffer
	//
	while( len-- ) buf[ len ] = SPACE;
	//
	//	Done!
	//
	return( true );
}

#endif

//
//	Define routines to assist with debugging that output
//	a range of value types.  Not compiled in unless one of the
//	debugging macros has been enabled.
//
#if defined( DEBUG_BUFFER_MANAGER )||defined( DEBUG_BIT_SLICER )||defined( DEBUG_POWER_MONITOR )||defined( DEBUG_STATISTICS )||defined( DEBUG_BUFFER_TABLE )

static char to_hex( byte n ) {
	if(( n >= 0 )&&( n < 10 )) return( '0' + n );
	if(( n >= 10 )&&( n < 16 )) return( 'A' + ( n - 10 ));
	return( '?' );
}

static int queue_byte( byte v ) {
	char	t[ TEXT_BUFFER ];

	t[ 0 ] = to_hex( v >> 4 );
	t[ 1 ] = to_hex( v & 0x0f );
	t[ 2 ] = EOS;
	return( queue_output( t ));
}

#ifdef DEBUG_STATISTICS

static int queue_int( int v ) {
	char	t[ TEXT_BUFFER ];

	*int_to_text( t, v ) = EOS;
	return( queue_output( t ));
}

static int queue_long( long v ) {
	char	t[ TEXT_BUFFER ];

	if(( v > -32000 )&&( v < 32000 )) {
		*int_to_text( t, v ) = EOS;
	}
	else {
		//
		//	If the value is too big for an int
		//	then we'll fudge things about a little.
		//
		char	*p;

		p = int_to_text( t, v / 1000 );
		*p++ = '0';
		*p++ = '0';
		*p++ = '0';
		*p = EOS;
	}
	return( queue_output( t ));
}

#endif

#ifdef DEBUG_BUFFER_TABLE

//
//	Define a routine to display high level content of a bit buffer
//
static void queue_bit_buffer( TRANS_BUFFER *ptr ) {
	//
	//	We will output this directly because it's
	//	too big to go through the output buffer and
	//	it should not be enabled in anything other
	//	than a manual debugging context.
	//
	Serial.print( '[' );
	Serial.print( ptr->state );
	Serial.print( ',' );
	Serial.print( ptr->target );
	Serial.print( ',' );
	Serial.print( ptr->track );
	Serial.print( ',' );
	Serial.print( ptr->duration );
	Serial.println( ']' );
}

#endif
#endif


//
//	Current monitoring code.
//	------------------------
//
//	With thanks to Nick Gammon and his post in the thread which can be
//	found at "http://www.gammon.com.au/forum/?id=11488&reply=5#reply5"
//	for those crucial first few steps that enable asynchronous ADC
//	use.
//

//
//	Global variables used to interact with the ADC interrupt service routine.
//
static volatile int	track_load_reading;
static volatile bool	reading_is_ready;

//
//	Macro generating code to restart the ADC process.  These could now
//	probably be subroutines as the earlier requirement for keeping
//	execution time to a minimum has been removed.
//
#define RESTART_ANALOGUE_READ()		track_load_reading=0;reading_is_ready=false;ADCSRA|=bit(ADSC)|bit(ADIE)
//
//	Macro to set the input pin the ADC will continuously read until
//	called to read another pin.
//
#define MONITOR_ANALOGUE_PIN(p)		ADMUX=bit(REFS0)|((p)&0x07);RESTART_ANALOGUE_READ()

//
//	Define the Interrupt Service Routine (ISR) which will
//	read the data and store it.  Restarting another ADC
//	conversion is done elsewhere.
//
ISR( ADC_vect ) {
	byte	low, high;

	//
	//	We have to read ADCL first; doing so locks both ADCL
	//	and ADCH until ADCH is read.  reading ADCL second would
	//	cause the results of each conversion to be discarded,
	//	as ADCL and ADCH would be locked when it completed.
	//
	low = ADCL;
	high = ADCH;
	//
	//	Store the reading away flagging that it is available.
	//
	track_load_reading = ( high << 8 ) | low;
	reading_is_ready = true;
}

//
//	The interrupt driven DCC packet transmission code.
//	--------------------------------------------------
//

//
//	Define the Circular buffer.
//
static TRANS_BUFFER circular_buffer[ TRANSMISSION_BUFFERS ];

//
//	The following variables direct the actions of the interrupt
//	routine.
//

//
//	Define the pointer into the circular buffer
//	used (and updated by) the interrupt routine.
//
//	This always points to the buffer currently being transmitted.
//
static TRANS_BUFFER	*current;

//
//	Contains the number of the pin which is being flipped
//	by the interrupt code whist generating the DCC signal.
//
//	This should always only contain one of either:
//
//		MAIN_TRACK_DIRECTION
//		PROG_TRACK_DIRECTION
//
static byte		output_pin;

//
//	"side" flips between true and false and lets the routine know
//	which "side" of the signal was being generated.  Used as input
//	to "SET_DIRECTION()" in the creation of the output signal.
//
static byte		side;

//
//	"remaining" The number of ticks before the next "side" transition.
//
//	"reload" is the value that should be reloaded into remaining if we are
//	only half way through generating a bit.
//
static byte		remaining,
			reload;

//
//	"one" is a boolean variable which indicates if we are currently
//	transmitting a series of ones (true) or zeros (false).
//
//	"left" is the number of ones or zeros which still need to be sent
//	before we start transmitting the next series of zeros or ones (or
//	the end of this bit transmission).
//
static byte		one,
			left;

//
//	This is the pointer the interrupt routine uses to collect the
//	bit stream data from inside the transmission buffer.  This can
//	point to data outside the bit buffer if the current buffer
//	contains no valid bit stream to transmit.
//
static byte		*bit_string;

//
//	The following array of bit transitions define the "DCC Idle Packet".
//
//	This packet contains the following bytes:
//
//		Address byte	0xff
//		Data byte	0x00
//		Parity byte	0xff
//
//	This is translated into the following bit stream:
//
//		1111...11110111111110000000000111111111
//
static byte dcc_idle_packet[] = {
	DCC_SHORT_PREAMBLE,	// 1s
	1,			// 0s
	8,			// 1s
	10,			// 0s
	9,			// 1s
	0
};

//
//	The following array does not describe a DCC packet, but a
//	filler series of a single "1" which is required while working
//	with decoders in service mode.
//
static byte dcc_filler_data[] = {
	1,			// 1s
	0
};


//
//	The Interrupt Service Routine which generate the DCC signal.
//
ISR( TIMER2_COMPA_vect ) {
	//
	//	The interrupt routine should be as short as possible, but
	//	in this case the necessity to drive forwards the output
	//	of the DCC signal is paramount.  So (while still time
	//	critical) the code has some work to achieve.
	//
	//	If "remaining" is greater than zero we are still counting down
	//	through half of a bit.  If it reaches zero it is time to
	//	flip the signal over.
	//
	if(!( --remaining )) {
		//
		//	Time is up for the current side.  Flip over and if
		//	a whole bit has been transmitted, the find the next
		//	bit to send.
		//
		//	We "flip" the output DCC signal now as this is the most
		//	time consistent position to do so.  The fact that the
		//	"current" output direction is always the opposite of
		//	"side" is actually irrelevant.  This should be more
		//	time efficient.
		//
		digitalWrite( output_pin, ( side? HIGH: LOW ));
		if(( side = !side )) {
			//
			//	Starting a new bit, is it more of the same?
			//
			if(!( --left )) {
				//
				//	No! It is now time to output a series
				//	of the alternate bits (assignment intentional).
				//
				if(( left = *bit_string++ )) {
					//
					//	More bits to send.
					//
					//	Select the correct tick count for the next
					//	next bit (again the assignment is intentional).
					//
					reload = ( one = !one )? TICKS_FOR_ONE: TICKS_FOR_ZERO;
				}
				else {
					//
					//	There are no more bits to transmit
					//	from this buffer, but before we move
					//	on we check the duration flag and act
					//	upon it.
					//
					//	If the current buffer is in RUN mode and duration
					//	is greater than 0 then we decrease duration and
					//	if zero, reset state to LOAD.  This will cause the
					//	buffer management code to check for any pending
					//	DCC commands.
					//
					if( current->duration && ( current->state == TBS_RUN )) {
						if(!( --current->duration )) current->state = TBS_LOAD;
					}
					//
					//	Move onto the next buffer.
					//
					current = current->next;

#if defined( DEBUG_STATISTICS )| defined( LCD_DISPLAY_ENABLE )

					//
					//	Count a successful packet transmission
					//
					statistic_packets++;

#endif

					//
					//	Actions related to the current state of the new
					//	buffer (select bits to output and optional state
					//	change).
					//
					switch( current->state ) {
						case TBS_RUN: {
							//
							//	We just transmit the packet found in
							//	the bit data
							//
							bit_string = current->bits;
							break;
						}
						case TBS_RELOAD: {
							//
							//	We have been asked to drop this buffer
							//	so we output an idle packet while changing
							//	state of buffer to LOAD so the manager can
							//	deal with it.
							//
							bit_string = dcc_idle_packet;
							current->state = TBS_LOAD;
							break;
						}
						case TBS_LOAD: {
							//
							//	This is a little tricky.  While we do not
							//	(and cannot) do anything with a buffer in
							//	load state, there is a requirement for the
							//	signal generator code NOT to output an idle
							//	packet if we are in the middle of a series
							//	of packets on the programming track.
							//
							//	To this end, if (and only if) the pending
							//	pointer is not NULL, then we will output
							//	the dcc filler data instead of the idle
							//	packet
							//
							bit_string = current->pending? dcc_filler_data: dcc_idle_packet;
							break;
						}
						default: {
							//
							//	If we find any other state we ignore the
							//	buffer and output an idle packet.
							//
							bit_string = dcc_idle_packet;
							break;
						}
					}
					//
					//	Initialise the remaining variables required to
					//	output the selected bit stream.
					//
					one = true;
					reload = TICKS_FOR_ONE;
					left = *bit_string++;
				}
			}
		}
		//
		//	Reload "remaining" with the next half bit
		//	tick count down from "reload".  If there has been
		//	a change of output bit "reload" will already have
		//	been modified appropriately.
		//
		remaining = reload;
	}
	//
	//	In ALL cases this routine needs to complete in less than TIMER_CLOCK_CYCLES
	//	(currently 232 for a 16 MHz machine).  This is approximately 100 actual
	//	instructions (assuming most instructions take 2 cycle with some taking 3).
	//
	//	The above code, on the "longest path" through the code (when moving
	//	between transmission buffers) I am estimating that this uses no more
	//	than 80% of this window (crosses fingers, it is tight in the worst case).
	//
	//	This routine would be so much better written in assembler when deployed
	//	on an AVR micro-controller, however this C does work and produces the
	//	required output signal with no loss of accuracy.
	//	
}

//
//	DCC Packet to bit stream conversion routine.
//	--------------------------------------------
//

//
//	Define a routine which will convert a supplied series of bytes
//	into a DCC packet defined as a series of bit transitions (as used
//	in the transmission buffer).
//
//	This routine is not responsible for the meaning (valid or otherwise)
//	of the bytes themselves, but *is* required to construct the complete
//	DCC packet formation including the preamble and inter-byte bits.
//
//	The code assumes it is being placed into a buffer of BIT_TRANSITIONS
//	bytes (as per the transmission buffers).
//
//	Returns true on success, false otherwise.
//
static bool pack_command( byte *cmd, byte clen, bool long_preamble, byte *buf ) {
	byte	l, b, c, v, s;

#ifdef DEBUG_BIT_SLICER
	queue_output( "PACK:" );
	for( l = 0; l < clen; queue_byte( cmd[ l++ ]));
	queue_output( ":" );
	queue_byte( long_preamble? DCC_LONG_PREAMBLE: DCC_SHORT_PREAMBLE );
#endif

	//
	//	Start with a pre-amble of "1"s.
	//
	*buf++ = long_preamble? DCC_LONG_PREAMBLE: DCC_SHORT_PREAMBLE;
	l = BIT_TRANSITIONS-1;

	//
	//	Prime pump with the end of header "0" bit.
	//
	b = 0;			// Looking for "0"s. Will
				// be value 0x80 when looking
				// for "1"s.
	c = 1;			// 1 zero found already.

	//
	//	Step through each of the source bytes one at
	//	a time..
	//
	while( clen-- ) {
		//
		//	Get this byte value.
		//
		v = *cmd++;
		//
		//	Count off the 8 bits in v
		//
		for( s = 0; s < 8; s++ ) {
			//
			//	take bits from from the MSB end
			//
			if(( v & 0x80 ) == b ) {
				//
				//	We simply increase the bit counter.
				//

				ASSERT( c < MAXIMUM_BIT_ITERATIONS );

				c++;
			}
			else {
				//
				//	Time to flip to the other bit.
				//
				if(!( --l )) return( false );

#ifdef DEBUG_BIT_SLICER
				queue_byte( c );
#endif

				*buf++ = c;
				c = 1;
				b ^= 0x80;
			}
			//
			//	Shift v over for the next bit.
			//
			v <<= 1;
		}
		//
		//	Now add inter-byte bit "0", or end of packet
		//	bit "1".  Remember we use the top bit in b to
		//	indicate which bit we are currently counting.
		//
		if(( clen? 0: 0x80 ) == b ) {
			//
			//	On the right bit, add to counter.
			//

			ASSERT( c < MAXIMUM_BIT_ITERATIONS );

			c++;
		}
		else {
			//
			//	Need the other bit, flip.
			//
			if(!( --l )) return( false );

#ifdef DEBUG_BIT_SLICER
			queue_byte( c );
#endif

			*buf++ = c;
			c = 1;
			b ^= 0x80;
		}
	}
	//
	//	Finally place the bit currently being counted, which
	//	must always be a "1" bit (end of packet marker).
	//
	if(!( --l )) return( false );

	ASSERT( c < MAXIMUM_BIT_ITERATIONS );

#ifdef DEBUG_BIT_SLICER
	queue_byte( c );
#endif

	*buf++ = c;
	//
	//	Mark end of the bit data.
	//
	if(!( --l )) return( false );

#ifdef DEBUG_BIT_SLICER
	queue_byte( 0 );
	queue_output( "\n" );
#endif

	*buf = 0;
	return( true );
}

//
//	Reply Construction routines.
//	----------------------------
//
//	The following set of routines provide alternatives to
//	using "sprintf()" for filling in a buffer space with
//	a reply from the firmware.
//
//	This has been done to reduce memory usage from the sprintf
//	template strings in exchange for a larger code base (as
//	there is plenty of code flash available but memory is
//	tight).
//

static char *_reply_in( char *buf, char code ) {
	*buf++ = PROT_IN_CHAR;
	*buf++ = code;
	return( buf );
}

static char *_reply_out( char *buf ) {
	*buf++ = PROT_OUT_CHAR;
	*buf++ = NL;
	return( buf );
}

static char *_reply_char( char *buf, char c ) {
	*buf++ = c;
	return( buf );
}

static void reply_1( char *buf, char code, int a1 ) {
	*_reply_out( int_to_text( _reply_in( buf, code ), a1 )) = EOS;
}

static void reply_2( char *buf, char code, int a1, int a2 ) {
	*_reply_out( int_to_text( _reply_char( int_to_text( _reply_in( buf, code ), a1 ), SPACE ), a2 )) = EOS;
}

#ifndef DCC_PLUS_PLUS_COMPATIBILITY

static void reply_2c( char *buf, char code, int a1, int a2 ) {
	*_reply_out( _reply_char( _reply_char( int_to_text( _reply_char( int_to_text( _reply_in( buf, code ), a1 ), SPACE ), a2 ), SPACE ), HASH )) = EOS;
}

#endif

static void reply_3( char *buf, char code, int a1, int a2, int a3 ) {
	*_reply_out( int_to_text( _reply_char( int_to_text( _reply_char( int_to_text( _reply_in( buf, code ), a1 ), SPACE ), a2 ), SPACE ), a3 )) = EOS;
}

//
//	Error Management and Reporting Code.
//	------------------------------------
//
//	Define a mechanism for the firmware to collate reports
//	of detected errors and forward them back to the host
//	computers as a suitable opportunity arises.
//
//	Once again, this is provided to prevent the firmware
//	from stalling on hardware IO.
//

//
//	Define a list of Error numbers that this code could report
//
#define NO_ERROR		0
#define ERROR_QUEUE_OVERFLOW	1
#define ERROR_REPORT_FAIL	2
#define BIT_TRANS_OVERFLOW	3
#define DCC_COMMAND_OVERFLOW	4
#define UNRECOGNISED_COMMAND	5
#define INVALID_BUFFER_NUMBER	6
#define INVALID_ARGUMENT_COUNT	7
#define INVALID_ADDRESS		8
#define INVALID_SPEED		9
#define INVALID_DIRECTION	10
#define INVALID_STATE		11
#define INVALID_CV_NUMBER	12
#define INVALID_FUNC_NUMBER	13
#define INVALID_BIT_NUMBER	14
#define INVALID_BIT_VALUE	15
#define INVALID_BYTE_VALUE	16
#define INVALID_WORD_VALUE	17
#define COMMAND_REPORT_FAIL	18
#define TRANSMISSION_BUSY	19
#define COMMAND_QUEUE_FAILED	20
#define POWER_NOT_OFF		21
#define POWER_OVERLOAD		22
#define POWER_SPIKE		23

//
//	Define a short queue of errors which are pending transmission
//
#define ERROR_QUEUE_SIZE	5

//
//	Define the type used to store a single error report.
//
#define QUEUED_ERROR struct queued_error
QUEUED_ERROR {
	int	err,
		arg;
};

//
//	Define error queue and control variables
//
static QUEUED_ERROR	error_queue[ ERROR_QUEUE_SIZE ];
static byte		error_in = 0,
			error_out = 0,
			errors_pending = 0;

//
//	Define a routine for capturing reported errors and placing them
//	in the pending error queue.
//
static void report_error( int err, int arg ) {
	QUEUED_ERROR	*e;
	byte		i;

	//
	//	Add error to end of queued errors.
	//
	if( errors_pending < ERROR_QUEUE_SIZE ) {
		//
		//	Locate new error record...
		//
		e = &( error_queue[ error_in++ ]);
		if( error_in == ERROR_QUEUE_SIZE ) error_in = 0;
		errors_pending++;
		//
		//	fill in the record.
		//
		e->err = err;
		e->arg = arg;
		return;
	}
	//
	//	The buffer is full, lets see if we can find
	//	a pre-existing buffer full error message.
	//
	for( i = 0; i < ERROR_QUEUE_SIZE; i++ ) {
		if( error_queue[ i ].err == ERROR_QUEUE_OVERFLOW ) {
			//
			//	Yeah, add one to the argument.
			//
			error_queue[ i ].arg++;
			return;
		}
	}
	//
	//	First time buffer overflow -- need to scrap the youngest
	//	error message and put this in its place.
	//
	if(( i = error_in )) {
		//
		//	non-zero, so we can count back.
		//
		i--;
	}
	else {
		//
		//	zero, so we wrap back.
		//
		i = ERROR_QUEUE_SIZE-1;
	}
	//
	//	Over write the error
	//
	error_queue[ i ].err = ERROR_QUEUE_OVERFLOW;
	error_queue[ i ].arg = 1;
	//
	//	Done.
	//
}

//
//	Define size of the buffer where the returned error messages
//	are constructed.
//
#define ERROR_OUTPUT_BUFFER 32

//
//	Flush the errors routine.
//
static void flush_error_queue( void ) {
	//
	//	Flush a single error to the output queue
	//	if there is an error pending and there is
	//	enough space in the output queue.
	//
	//	The error message is in the form:
	//
	//		"<#NN AAAAA>\n"
	//
	//	or
	//		"[ENN AAAAA]\n"
	//
	//	which should always be shorter than
	//	ERROR_OUTPUT_BUFFER characters.
	//
	if( errors_pending ) {
		char		buffer[ ERROR_OUTPUT_BUFFER ];
		QUEUED_ERROR	*e;
	
		//
		//	Build error report, and send it only if there
		//	is enough space.
		//
		e = &( error_queue[ error_out ]);
		reply_2( buffer, SELECT_ALT( 'E', '#' ), e->err, e->arg );
		//
		//	Can we send this?
		//
		if( queue_available() >= (int)strlen( buffer )) {
			//
			//	Send as space is available.
			//
			if( ++error_out == ERROR_QUEUE_SIZE ) error_out = 0;
			errors_pending--;
			//
			//	This cannot fail as we have checked first.
			//
			(void)queue_output( buffer );
		}
	}
}

//
//	Power selection and control code.
//	---------------------------------
//
//	Simple code to manage which tracks are powered on and off
//	depending on the bit buffer requirements.
//
//	In compatibility mode the firmware will only operate the
//	operations track supporting the "<0>" and "<1>" commands.
//
//	In native mode the firmware can operate either the operations
//	track OR the programming track, but not both at the same time.
//	The firmware requires that you can only jump between off ("[P0]")
//	mode and either one of the powered modes (operations track, "[P1]",
//	or programming track, "[P2]"), and not directly between using the
//	operations track and programming track.
//

//
//	Power ON/OFF status values.
//
#define POWER_STATE enum power_state
POWER_STATE {
	GLOBAL_POWER_OFF,
	GLOBAL_POWER_MAIN,
	GLOBAL_POWER_PROG
};

static POWER_STATE global_power_state = GLOBAL_POWER_OFF;

//
//	Routine to turn ON the operating track.
//	Return true if this actually changed the
//	state of the power.
//
//
static byte power_on_main_track( void ) {
	POWER_STATE	prev;

	prev = global_power_state;
	output_pin = MAIN_TRACK_DIRECTION;
	digitalWrite( MAIN_TRACK_ENABLE, HIGH );
	digitalWrite( PROG_TRACK_ENABLE, LOW );
	global_power_state = GLOBAL_POWER_MAIN;
	return( prev != GLOBAL_POWER_MAIN );
}

//
//	Routine to turn ON the programming track.
//	Return true if this actually changed the
//	state of the power.
//
//

#ifndef DCC_PLUS_PLUS_COMPATIBILITY

static byte power_on_prog_track( void ) {
	POWER_STATE	prev;

	prev = global_power_state;
	output_pin = PROG_TRACK_DIRECTION;
	digitalWrite( MAIN_TRACK_ENABLE, LOW );
	digitalWrite( PROG_TRACK_ENABLE, HIGH );
	global_power_state = GLOBAL_POWER_PROG;
	return( prev != GLOBAL_POWER_PROG );
}

#endif

//
//	Routine to power OFF tracks, return true
//	if this actually changed the state of the
//	power.
//
static byte power_off_tracks( void ) {
	POWER_STATE	prev;

	prev = global_power_state;
	digitalWrite( MAIN_TRACK_ENABLE, LOW );
	digitalWrite( PROG_TRACK_ENABLE, LOW );
	global_power_state = GLOBAL_POWER_OFF;
	return( prev != GLOBAL_POWER_OFF);
}

//
//	Current/Load monitoring system
//	==============================
//

//
//	We are going to use an array of values compounded upon
//	each other to generate a series of average values which
//	represent averages over longer and longer periods (each
//	element of the array twice as long as the previous).
//
//	Define the size of the array.
//
#define COMPOUNDED_VALUES	10

//
//	Define the size of the "short average" we will use to
//	identify a genuine rise in power consumption signifying
//	a confirmation return.  This is an index into the
//	compounded average table.
//
#define SHORT_AVERAGE_VALUE	2

//
//	This is the array of compounded average values.
//
static int	load_compound_value[ COMPOUNDED_VALUES ];
//
//	Finally, this is the flag set if the power code
//	spots a confirmation signal in the power consumption.
//
static bool	load_confirmed;

//
//	This routine is called every time track electrical load data
//	becomes available.  The routine serves two purposes:
//
//		Monitor for overload and spike conditions
//
//		Detect return signals from devices attached
//		to the DCC bus.
//
static void monitor_current_load( int amps ) {

#ifdef DEBUG_POWER_MONITOR
	Serial.print( "AMPS=" );
	Serial.println( amps );
#endif

	//
	//	Compound in the new figure.
	//
	for( byte i = 0; i < COMPOUNDED_VALUES; i++ ) {
		amps = ( load_compound_value[ i ] = ( amps + load_compound_value[ i ]) >> 1 );
	}
	//
	//	Is there a power spike?
	//
	if( load_compound_value[ 1 ] > INSTANT_CURRENT_LIMIT ) {
		//
		//	No question - cut the power now.
		//
		if( power_off_tracks()) {
			//
			//	Now, log an error to give the reason for
			//	cutting the power, and then send back an
			//	appropriate power report letting the software
			//	know this has happened.
			//
			report_error( POWER_SPIKE, amps );
			if( !queue_output( SELECT_ALT( "[P0]\n", "<p0>\n" ))) report_error( COMMAND_REPORT_FAIL, SELECT_ALT( 'P', 'p' ));
			//
			//	We flatten the power averaging data to simplify
			//	power up restarting.
			//
			for( byte i = 0; i < COMPOUNDED_VALUES; load_compound_value[ i++ ] = 0 );
		}
		return;
	}
	//
	//	Has there been a big enough jump compared a short term average?
	//
	if( load_compound_value[ COMPOUNDED_VALUES - 1 ] > AVERAGE_CURRENT_LIMIT ) {
		//
		//	Cut the power here because there is some sort of long
		//	term higher power drain.
		//
		if( power_off_tracks()) {
			report_error( POWER_OVERLOAD, amps );
			if( !queue_output( SELECT_ALT( "[P0]\n", "<p0>\n" ))) report_error( COMMAND_REPORT_FAIL, SELECT_ALT( 'P', 'p' ));
			for( byte i = 0; i < COMPOUNDED_VALUES; load_compound_value[ i++ ] = 0 );
		}
		return;
	}
	//	We now assess if there has been a "jump" in the current draw
	//	which would indicate a confirmation signal being sent back
	//	from an attached decoder.
	//
	if(( load_compound_value[ SHORT_AVERAGE_VALUE ] - load_compound_value[ COMPOUNDED_VALUES-1 ]) > MINIMUM_DELTA_AMPS ) {
		//
		//	We believe that we have seen a confirmation.
		//
		load_confirmed = true;
	}
	//
	//	Kick off the next reading.
	//
	RESTART_ANALOGUE_READ();
}

//
//	Routine called by the watchdog timer code to report
//	track power dynamically.
//
static void report_track_power( void ) {
	char	buffer[ 16 ];

	reply_1( buffer, SELECT_ALT( 'L', 'a' ), load_compound_value[ COMPOUNDED_VALUES-1 ]);
	if( !queue_output( buffer )) report_error( COMMAND_REPORT_FAIL, SELECT_ALT( 'L', 'a' ));
}

//
//	Buffer Control and Management Code.
//	-----------------------------------
//

//
//	The following variable is used by the "Management Processing" code which
//	continuously loops through the transmission buffers looking for buffers
//	which need some processing.
//
static TRANS_BUFFER	*manage;

//
//	This is the routine which controls (and synchronises with the interrupt routine)
//	the transition of buffers between various state.
//
static void management_service_routine( void ) {
	//
	//	This routine only handles the conversion of byte encoded DCC packets into
	//	bit encoded packets and handing the buffer off to the ISR for transmission.
	//
	//	Consequently we are only interested in buffers with a state of LOAD.
	//
	if( manage->state == TBS_LOAD ) {
		PENDING_PACKET	*pp;

		//
		//	Pending DCC packets to process? (assignment intentional)
		//
		if(( pp = manage->pending )) {
			//
			//	Our only task here is to convert the pending data into live
			//	data and set the state to RUN.
			//
			if( pack_command( pp->command, pp->len, pp->lpreamble, manage->bits )) {
				//
				//	Good, set up the remainder of the live parameters.
				//
				manage->target = pp->target;
				manage->duration = pp->duration;
				//
				//	We set state now as this is the trigger for the
				//	interrupt routine to start processing the content of this
				//	buffer (so everything must be completed before hand).
				//
				//	We have done this in this order to prevent a situation
				//	where the buffer has state LOAD and pending == NULL, as
				//	this might (in a case of bad timing) cause the ISR to output
				//	an idle packet when we do not want it to.
				//
				manage->state = TBS_RUN;
				//
				//	Now we dispose of the one pending record we have used.
				//
				manage->pending = release_pending_recs( manage->pending, true );
				//
				//	Finally, if this had a "reply on send" confirmation and
				//	the command we have just lined up is the last one in the
				//	list, then send the confirmation now.
				//
				if(( manage->reply == REPLY_ON_SEND )&&( manage->pending == NULL )) {
					if( !queue_output( manage->contains )) {
						report_error( COMMAND_REPORT_FAIL, manage->target );
					}
					manage->reply = NO_REPLY_REQUIRED;
				}

#ifdef DEBUG_BUFFER_MANAGER
				queue_output( "LOAD:" );
				queue_int( manage->target );
				queue_output( "\n" );
#endif

			}
			else {
				//
				//	Failed to complete as the bit translation failed.
				//
				report_error( BIT_TRANS_OVERFLOW, manage->pending->target );
				//
				//	We push this buffer back to EMPTY, there is nothing
				//	else we can do with it.
				//
				manage->state = TBS_EMPTY;
				//
				//	Finally, we scrap all pending records.
				//
				manage->pending = release_pending_recs( manage->pending, false );

#ifdef DEBUG_BUFFER_MANAGER
				queue_output( "FAIL:" );
				queue_int( manage->target );
				queue_output( "\n" );
#endif

			}
		}
		else {
			//
			//	The pending field is empty.  Before marking the buffer as empty
			//	for re-use, we should check to see if a confirmation is required.
			//
			if( manage->reply == REPLY_ON_CONFIRM ) {
				char	*hash;

				//
				//	Two actions here:
				//
				//		If the reply text contains a HASH then this is
				//		replaced with 1 on success and 0 on failure.
				//
				//		Otherwise send reply as is but only if confirmation
				//		was seen.
				//
				//	Look for the HASH symbol
				//
				if(( hash = strchr( manage->contains, HASH )) != NULL ) {
					//
					//	Found, so update hash and send.
					//
					*hash = load_confirmed? '1': '0';
					if( !queue_output( manage->contains )) {
						report_error( COMMAND_REPORT_FAIL, manage->target );
					}
				}
				else if( load_confirmed ) {
					//
					//	Only send confirmation if confirmation was recieved
					//
					if( !queue_output( manage->contains )) {
						report_error( COMMAND_REPORT_FAIL, manage->target );
					}
				}
			}
			//
			//	Now mark empty.
			//
			manage->reply = NO_REPLY_REQUIRED;
			manage->state = TBS_EMPTY;

#ifdef DEBUG_BUFFER_MANAGER
			queue_output( "EMPTY:" );
			queue_int( manage->target );
			queue_output( "\n" );
#endif

		}
	}
	//
	//	Finally, before we finish, remember to move onto the next buffer in the
	//	circular queue.
	//
	manage = manage->next;
}

//
//	Initial buffer configuration routine and post-init
//	reconfiguration routines (for either main or programming
//	use).
//
static void link_buffer_chain( void ) {
	int	i;
	
	//
	//	All buffers flagged as empty.
	//
	for( i = 0; i < TRANSMISSION_BUFFERS; i++ ) {
		circular_buffer[ i ].state = TBS_EMPTY;
		circular_buffer[ i ].target = 0;
		circular_buffer[ i ].duration = 0;
		circular_buffer[ i ].bits[ 0 ] = 0;
		circular_buffer[ i ].pending = NULL;

#ifdef LCD_DISPLAY_ENABLE
		//
		//	Not really necesary but ensures all buffers
		//	contain something displayable even if it has
		//	not been explicity set.
		//
		memset( circular_buffer[ i ].display, '_', LCD_DISPLAY_BUFFER_WIDTH );
		circular_buffer[ i ].display[ 0 ] = '[';
		circular_buffer[ i ].display[ LCD_DISPLAY_BUFFER_WIDTH-1 ] = ']';	
#endif

	}
	//
	//	Link up all the buffers into a loop in numerical order.
	//
	for( i = 0; i < TRANSMISSION_BUFFERS-1; i++ ) circular_buffer[ i ].next = circular_buffer + ( i + 1 );
	//
	//	point the tail to the head
	//
	circular_buffer[ TRANSMISSION_BUFFERS-1 ].next = circular_buffer;
}

//
//	The following two routines carefully reshape the circular
//	buffer links to ensure that the interrupt and management routines
//	end up only servicing a specific subset of the records, even
//	if at the exact point of modification, they are in the other
//	set of buffers.
//
//	There is an extremely small possibility that the signal ISR will
//	read one of these addresses "as it is being modified", and subsequently
//	pick up the wrong address.  It is a small risk but the table
//	extends across multiple page boundaries (256 byte sections), and so
//	the addresses differ in both the MSB and LSB.  To protect against this
//	possibility the assignments need to be bracketed between cli() and sei().
//
//	These routines are only called when one or other track is being power up.
//
static void link_main_buffers( void ) {
	//
	//	This routine is called to shape the circular buffers
	//	to only contain the operating track buffers.
	//
	cli();
	circular_buffer[ PROGRAMMING_BASE_BUFFER-1 ].next = circular_buffer;
	circular_buffer[ TRANSMISSION_BUFFERS-1 ].next = circular_buffer;
	sei();
}

#ifndef DCC_PLUS_PLUS_COMPATIBILITY

static void link_prog_buffers( void ) {
	//
	//	This routine is called to shape the circular buffers
	//	to only contain the programming track buffer.
	//
	cli();
	circular_buffer[ PROGRAMMING_BASE_BUFFER-1 ].next = circular_buffer + PROGRAMMING_BASE_BUFFER;
	circular_buffer[ TRANSMISSION_BUFFERS-1 ].next = circular_buffer + PROGRAMMING_BASE_BUFFER;
	sei();
}

#endif

//
//	Firmware initialisation routines
//	--------------------------------
//

//
//	Define the circular buffer and interrupt variable initialisation routine.
//
static void initialise_data_structures( void ) {
	int	i;

	//
	//	We start all buffer circularly linked.
	//
	link_buffer_chain();
	//
	//	Initialise the pending packets structures.
	//
	free_pending_packets = NULL;
	for( i = 0; i < PENDING_PACKETS; i++ ) {
		pending_dcc_packet[ i ].target = 0;
		pending_dcc_packet[ i ].duration = 0;
		pending_dcc_packet[ i ].len = 0;
		pending_dcc_packet[ i ].command[ 0 ] = EOS;
		pending_dcc_packet[ i ].next = free_pending_packets;
		free_pending_packets = &( pending_dcc_packet[ i ]);
	}
	//
	//	Now prime the transmission interrupt routine state variables.
	//
	current = circular_buffer;
	output_pin = MAIN_TRACK_DIRECTION;
	side = true;
	remaining = 1;
	bit_string = dcc_idle_packet;
	one = true;
	reload = TICKS_FOR_ONE;
	left = *bit_string++;
	//
	//	Now prime the management code
	//
	manage = circular_buffer;
}

void setup( void ) {
	//
	//	Initialise serial connection
	//
	Serial.begin( SERIAL_BAUD_RATE );
	while( !Serial );
	Serial.println( "Arduino DCC Generator Firmware (" SELECT_ALT( "Native", "DCC++" ) " personality)" );
	//
	//	Initialise the MAIN track H Bridge pins.
	//
	pinMode( MAIN_TRACK_DIRECTION, OUTPUT );
	digitalWrite( MAIN_TRACK_DIRECTION, LOW );
	pinMode( MAIN_TRACK_ENABLE, OUTPUT );
	digitalWrite( MAIN_TRACK_ENABLE, LOW );
	pinMode( MAIN_TRACK_BRAKE, OUTPUT );
	digitalWrite( MAIN_TRACK_BRAKE, LOW );
	pinMode( MAIN_TRACK_LOAD, INPUT );
	(void)analogRead( MAIN_TRACK_LOAD );
	//
	//	Initialise the Programming track H Bridge pins.
	//
	pinMode( PROG_TRACK_DIRECTION, OUTPUT );
	digitalWrite( PROG_TRACK_DIRECTION, LOW );
	pinMode( PROG_TRACK_ENABLE, OUTPUT );
	digitalWrite( PROG_TRACK_ENABLE, LOW );
	pinMode( PROG_TRACK_BRAKE, OUTPUT );
	digitalWrite( PROG_TRACK_BRAKE, LOW );
	pinMode( PROG_TRACK_LOAD, INPUT );
 	(void)analogRead( PROG_TRACK_LOAD );
	//
	//	Disable interrupts.
	//
	cli();
	//
	//		Set Timer2 to count clock cycles 1:1.	
	//
	TCCR2A = 0;	//	Set entire TCCR2A register to 0
	TCCR2B = 0;	//	Same for TCCR2B
	TCNT2  = 0;	//	Initialize counter value to 0
	//
	//		Set compare match register to
	//		generate the correct tick duration.
	//
	OCR2A = TIMER_CLOCK_CYCLES;
	//
	//		Turn on CTC mode
	//
	TCCR2A |= (1 << WGM21);
	//
	//		Set CS20 bit for no pre-scaler
	//
	TCCR2B |= (1 << CS20);   
	//
	//		Enable timer compare interrupt
	//
	TIMSK2 |= (1 << OCIE2A);
	//
	//	Now set up the data structures.
	//	-------------------------------
	//
	initialise_data_structures();

#ifndef DCC_PLUS_PLUS_COMPATIBILITY

	init_function_cache();
	
#endif
  	//
	//	Enable interrupts.
	//
	sei();
	//
	//	Kick of the power management system.
	//
	MONITOR_ANALOGUE_PIN( MAIN_TRACK_ANALOGUE );

	//
	//	Optional hardware initialisations
	//

	//
	//	LCD Display.
	//	------------
	//
	//	This might be a warm restart for the display,
	//	so reset everything to a clean slate.
	//

#ifdef LCD_DISPLAY_ENABLE

	Wire.begin();
	lcd.begin();
	lcd.backlight( true );
	lcd.clear();

#endif
}

#ifdef LCD_DISPLAY_ENABLE

//
//	Liquid Crystal Display
//	======================
//
//	Update the LCD with pertinent data about the operation of the
//	DCC Generator.
//

#include "mul_div.h"

//
//	Time of previous call to display update (ms).
//
static unsigned long last_lcd_update = 0;

//
//	Update routine.
//
static void display_lcd_updates( unsigned long now ) {
	unsigned int	delta,		// Milli-Seconds between calls
			uptime,		// Minutes system has been up
			pps;		// DCC Packets per second.

	char		buffer[ LCD_DISPLAY_STATUS_WIDTH ];

	//
	//	store the seconds uptime.
	//
	uptime = now / 1000;
	//
	//	Establish the time since we last ran the
	//	LCD update funciton
	//
	delta = (unsigned int)( now - last_lcd_update );
	last_lcd_update = now;
	//
	//	Calculate the packets per second rate.
	//
	pps = mul_div( statistic_packets, 1000, delta );
	//
	//	Output the STATUS column data in the following order:
	//
	//	+--------------------+	The STATUS area of the display, showing:
	//	|SSSSSS              |	The power (L)oad average
	//	|SSSSSS              |	The available (F)ree bit buffers and (P)ower status
	//	|SSSSSS              |	DCC packets (T)ransmitted sent per second
	//	|SSSSSS              |	The (U)ptime in seconds
	//	+--------------------+
	//
	//	The code will a visual bounary at the far right of the status
	//	area to create a visual break between the status and buffer areas.
	//
	buffer[ LCD_DISPLAY_STATUS_WIDTH-1 ] = SELECT_ALT( '|', ':' );
	//
	//	Now complete each of the rows in LCD_DISPLAY_STATUS_WIDTH-1 characters.
	//
	{
		//
		//	Row 0, always available, Power Load Average
		//
		buffer[ 0 ] = 'L';
		if( !backfill_int_to_text( buffer+1, load_compound_value[ COMPOUNDED_VALUES-1 ], LCD_DISPLAY_STATUS_WIDTH-2 )) {
			memset( buffer+1, HASH, LCD_DISPLAY_STATUS_WIDTH-2 );
		}
		lcd.position( 0, 0 );
		lcd.write( buffer, LCD_DISPLAY_STATUS_WIDTH );
	}
#if LCD_DISPLAY_ROWS > 1
	{
		byte	c;

		//
		//	Row 1, (F)ree bit buffers and (P)ower status
		//
		c = 0;
		for( byte i = 0; i < TRANSMISSION_BUFFERS; i++ ) {
			if( circular_buffer[ i ].state == TBS_EMPTY ) {
				c++;
			}
		}
		buffer[ 0 ] = 'P';
		switch( global_power_state ) {
			case GLOBAL_POWER_OFF: {
				buffer[ 1 ] = '0';
				break;
			}
			case GLOBAL_POWER_MAIN: {
				buffer[ 1 ] = '1';
				break;
			}
			case GLOBAL_POWER_PROG: {
				buffer[ 1 ] = '2';
				break;
			}
			default: {
				buffer[ 1 ] = HASH;
				break;
			}
		}
		buffer[ 2 ] = 'F';
		if( !backfill_int_to_text( buffer+3, c, LCD_DISPLAY_STATUS_WIDTH-4 )) {
			memset( buffer+3, HASH, LCD_DISPLAY_STATUS_WIDTH-4 );
		}
		lcd.position( 0, 1 );
		lcd.write( buffer, LCD_DISPLAY_STATUS_WIDTH );
	}
#endif
#if LCD_DISPLAY_ROWS > 2
	{
		//
		//	Row 2, DCC packets (T)ransmitted sent per second
		//
		buffer[ 0 ] = 'T';
		if( !backfill_int_to_text( buffer+1, pps, LCD_DISPLAY_STATUS_WIDTH-2 )) {
			memset( buffer+1, HASH, LCD_DISPLAY_STATUS_WIDTH-2 );
		}
		lcd.position( 0, 2 );
		lcd.write( buffer, LCD_DISPLAY_STATUS_WIDTH );
	}
#endif
#if LCD_DISPLAY_ROWS > 3
	{
		//
		//	Row 3, The (U)ptime in seconds
		//
		buffer[ 0 ] = 'U';
		if( uptime < 1000 ) {
			//
			//	Display time in seconds.
			//
			(void)backfill_int_to_text( buffer+1, uptime, LCD_DISPLAY_STATUS_WIDTH-3 );
			buffer[ LCD_DISPLAY_STATUS_WIDTH-2 ] = 's';
		}
		else {
			if(( uptime /= 60 ) < 1000 ) {
				//
				//	Display time in minutes.
				//
				(void)backfill_int_to_text( buffer+1, uptime, LCD_DISPLAY_STATUS_WIDTH-3 );
				buffer[ LCD_DISPLAY_STATUS_WIDTH-2 ] = 'm';
			}
			else {
				//
				//	Display time in HOURS!
				//
				uptime /= 60;
				(void)backfill_int_to_text( buffer+1, uptime, LCD_DISPLAY_STATUS_WIDTH-3 );
				buffer[ LCD_DISPLAY_STATUS_WIDTH-2 ] = 'h';
			}
		}
		lcd.position( 0, 3 );
		lcd.write( buffer, LCD_DISPLAY_STATUS_WIDTH );
	}
#endif
	//
	//	+--------------------+	The BUFFER area of the display, showing:
	//	|      BBBBBBBBBBBBBB|	Buffers in use and the action in place.
	//	|      BBBBBBBBBBBBBB|	This area is broken up into as many one
	//	|      BBBBBBBBBBBBBB|	line areas as will fit.  Each area is
	//	|      BBBBBBBBBBBBBB|	LCD_DISPLAY_BUFFER_WIDTH bytes wide.
	//	+--------------------+	
	//
	{
		byte	r, c;

		r = 0;
		c = 0;
		for( byte i = 0; i < TRANSMISSION_BUFFERS; i++ ) {
			if( circular_buffer[ i ].state == TBS_RUN ) {
				lcd.position( LCD_DISPLAY_STATUS_WIDTH + c * LCD_DISPLAY_BUFFER_WIDTH, r );
				if(( r += 1 ) >= LCD_DISPLAY_ROWS ) {
					r = 0;
					if(( c += 1 ) >= LCD_DISPLAY_BUFFER_COLS ) {
						break;
					}
				}
				lcd.write( circular_buffer[ i ].display, LCD_DISPLAY_BUFFER_WIDTH );
			}
		}
		//
		//	Clear out remaining spaces.
		//
		while( c < LCD_DISPLAY_BUFFER_COLS ) {
			while( r < LCD_DISPLAY_ROWS ) {
				lcd.position( LCD_DISPLAY_STATUS_WIDTH + c * LCD_DISPLAY_BUFFER_WIDTH, r );
				lcd.fill( SPACE, LCD_DISPLAY_BUFFER_WIDTH );
				r++;
			}
			r = 0;
			c++;
		}
	}
	//ZZ//
}

#endif

//
//	Input Processing routines
//	=========================
//

//
//	While it seems pointless to create a different protocol
//	from the DCCpp Firmware, doing so offers a couple of
//	benefits:
//
//	o	Using an Arduino with one firmware against software
//		expecting the other will obviously fail, rather the
//		seem to work, then fail in unexpected ways.
//
//	o	The firmware coded in this sketch is unable to
//		implement some of the higher level primitives
//		that the DCCpp firmware supports.  These commands are
//		not direct DCC primitive operations but synthesized
//		actions which can be implemented within the host computer
//		software.
//
//	o	The syntax and structure of the conversation between
//		the DCC Generator protocol and host computer software
//		will be more aligned with the operation of the
//		firmware itself.
//
//	Having said that, in high level terms, both protocols have
//	basically the same structure.
//

//
//	Define simple "in string" number parsing routine.  This
//	effectively does the bulk of the syntax parsing for the
//	commands so it a little more fiddly than strictly
//	necessary.
//
//	Return the address of the next unparsed character, this
//	be EOS if at the end of the string.
//
static char *parse_number( char *buf, bool *found, int *value ) {
	int	v, n, c;

	//
	//	Skip any leading white space
	//
	while( isspace( *buf )) buf++;
	//
	//	Intentional assignment, remember if we are handling
	//	a negative number.
	//
	if(( n = ( *buf == '-' ))) buf++;
	//
	//	gather up a decimal number
	//
	c = 0;
	v = 0;
	while( isdigit( *buf )) {
		v = v * 10 + ( *buf++ - '0' );
		c++;
	}
	if( n ) v = -v;
	//
	//	Skip any trailing white space
	//
	while( isspace( *buf )) buf++;
	//
	//	Set up returned data
	//
	*found = ( c > 0 );
	*value = v;
	//
	//	Return address of next unprocessed character.
	//
	return( buf );
}

//
//	Define a routine which scans the input text and returns the
//	command character and a series of numeric arguments
//
//	Routine returns the number of arguments (after the command).
//
static int parse_input( char *buf, char *cmd, int *arg, int max ) {
	int	args,
		value;
	bool	found;

	ASSERT( max > 0 );

	//
	//	Copy out the command character and verify
	//
	*cmd  = *buf++;
	if( !isalnum( *cmd )) return( ERROR );
	//
	//	Step through remainder of string looking for
	//	numbers.
	//
	args = 0;
	found = true;
	while( found ) {
		buf = parse_number( buf, &found, &value );
		if( found ) {
			if( args >= max ) return( ERROR );
			arg[ args++ ] = value;
		}
	}
	//
	//	Done.
	//
	return( args );
}

//
//	Define a routine for finding an available buffer within
//	a specified range and return its address, or return NULL.
//
static TRANS_BUFFER *find_available_buffer( byte base, byte count, int target ) {
	byte		i;
	TRANS_BUFFER	*b;

	ASSERT( base < TRANSMISSION_BUFFERS );
	ASSERT(( count > 0 )&&( count <= TRANSMISSION_BUFFERS ));
	ASSERT(( base + count ) <= TRANSMISSION_BUFFERS );
	
	//
	//	Look for possible buffer *already* sending to this
	//	target.
	//
	b = circular_buffer + base;
	for( i = 0; i < count; i++ ) {
		if( b->target == target ) return( b );
		b++;
	}
	//
	//	Nothing found so far, look for an empty one.
	//
	b = circular_buffer + base;
	for( i = 0; i < count; i++ ) {
		if( b->state == TBS_EMPTY ) return( b );
		b++;
	}
	//
	//	No available buffer located.
	//

#ifdef DEBUG_BUFFER_TABLE
	for( byte i = 0; i < TRANSMISSION_BUFFERS; i++ ) queue_bit_buffer( circular_buffer + i );
#endif

	return( NULL );
}

//
//	DCC composition routines
//	------------------------
//
//	The following routines are used to create individual byte oriented
//	DCC commands.
//

//
//	Create a speed and direction packet for a specified target.
//	Returns the number of bytes in the buffer.
//
static byte compose_motion_packet( byte *command, int adrs, int speed, int dir ) {
	byte	len;

	ASSERT( command != NULL );
	ASSERT(( adrs >= MINIMUM_DCC_ADDRESS )&&( adrs <= MAXIMUM_DCC_ADDRESS ));
	ASSERT(( speed == EMERGENCY_STOP )||(( speed >= MINIMUM_DCC_SPEED )&&( speed <= MAXIMUM_DCC_SPEED )));
	ASSERT(( dir == DCC_FORWARDS )||( dir == DCC_BACKWARDS ));

	if( adrs > MAXIMUM_SHORT_ADDRESS ) {
		command[ 0 ] = 0b11000000 | ( adrs >> 8 );
		command[ 1 ] = adrs & 0b11111111;
		len = 2;
	}
	else {
		command[ 0 ] = adrs;
		len = 1;
	}
	command[ len++ ] = 0b00111111;
	switch( speed ) {
		case 0: {
			command[ len++ ] = ( dir << 7 );
			break;
		}
		case EMERGENCY_STOP: {
			command[ len++ ] = ( dir << 7 ) | 1;
			break;
		}
		default: {
			command[ len++ ] = ( dir << 7 )|( speed + 1 );
			break;
		}
	}
	//
	//	Done.
	//
	return( len );
}

//
//	Create an accessory modification packet.  Return number of bytes
//	used by the command.
//
static byte compose_accessory_change( byte *command, int adrs, int subadrs, int state ) {

	ASSERT( command != NULL );
	ASSERT(( adrs >= MIN_ACCESSORY_ADDRESS )&&( adrs <= MAX_ACCESSORY_ADDRESS ));
	ASSERT(( subadrs >= MIN_ACCESSORY_SUBADRS )&&( subadrs <= MAX_ACCESSORY_SUBADRS ));
	ASSERT(( state == ACCESSORY_ON )||( state == ACCESSORY_OFF ));

	command[ 0 ] = 0b10000000 | ( adrs & 0b00111111 );
	command[ 1 ] = ((( adrs >> 2 ) & 0b01110000 ) | ( subadrs << 1 ) | state ) ^ 0b11111000;
	//
	//	Done.
	//
	return( 2 );
}

#ifndef DCC_PLUS_PLUS_COMPATIBILITY

//
//	Create a Function set/reset command, return number of bytes used.
//
//	Until I can find a more focused mechanism for modifying the functions
//	this routine has to be this way.  I guess a smarter data driven approach
//	could be used to generate the same output, but at least this is clear.
//
static byte compose_function_change( byte *command, int adrs, int func, int state ) {

	ASSERT( command != NULL );
	ASSERT(( adrs >= MIN_ACCESSORY_ADDRESS )&&( adrs <= MAX_ACCESSORY_ADDRESS ));
	ASSERT(( func >= MIN_FUNCTION_NUMBER )&&( func <= MAX_FUNCTION_NUMBER ));
	ASSERT(( state == FUNCTION_ON )||( state == FUNCTION_OFF ));

	if( update_function( adrs, func, ( state == FUNCTION_ON ))) {
		byte	len;

		//
		//	Function has changed value, update the corresponding decoder.
		//
		if( adrs > MAXIMUM_SHORT_ADDRESS ) {
			command[ 0 ] = 0b11000000 | ( adrs >> 8 );
			command[ 1 ] = adrs & 0b11111111;
			len = 2;
		}
		else {
			command[ 0 ] = adrs;
			len = 1;
		}
		//
		//	We have to build up the packets depending on the
		//	function number to modify.
		//
		if( func <= 4 ) {
			//
			//	F0-F4		100[F0][F4][F3][F2][F1]
			//
			command[ len++ ] =	0x80	| get_function( adrs, 0, 0x10 )
							| get_function( adrs, 1, 0x01 )
							| get_function( adrs, 2, 0x02 )
							| get_function( adrs, 3, 0x04 )
							| get_function( adrs, 4, 0x08 );
		}
		else if( func <= 8 ) {
			//
			//	F5-F8		1011[F8][F7][F6][F5]
			//
			command[ len++ ] =	0xb0	| get_function( adrs, 5, 0x01 )
							| get_function( adrs, 6, 0x02 )
							| get_function( adrs, 7, 0x04 )
							| get_function( adrs, 8, 0x08 );
		}
		else if( func <= 12 ) {
			//
			//	F9-F12		1010[F12][F11][F10][F9]
			//
			command[ len++ ] =	0xa0	| get_function( adrs, 9, 0x01 )
							| get_function( adrs, 10, 0x02 )
							| get_function( adrs, 11, 0x04 )
							| get_function( adrs, 12, 0x08 );
		}
		else if( func <= 20 ) {
			//
			//	F13-F20		11011110, [F20][F19][F18][F17][F16][F15][F14][F13]
			//
			command[ len++ ] =	0xde;
			command[ len++ ] =		  get_function( adrs, 13, 0x01 )
							| get_function( adrs, 14, 0x02 )
							| get_function( adrs, 15, 0x04 )
							| get_function( adrs, 16, 0x08 )
							| get_function( adrs, 17, 0x10 )
							| get_function( adrs, 18, 0x20 )
							| get_function( adrs, 19, 0x40 )
							| get_function( adrs, 20, 0x80 );
		}
		else {
			//
			//	F21-F28		11011111, [F28][F27][F26][F25][F24][F23][F22][F21]
			//
			command[ len++ ] =	0xdf;
			command[ len++ ] =		  get_function( adrs, 21, 0x01 )
							| get_function( adrs, 22, 0x02 )
							| get_function( adrs, 23, 0x04 )
							| get_function( adrs, 24, 0x08 )
							| get_function( adrs, 25, 0x10 )
							| get_function( adrs, 26, 0x20 )
							| get_function( adrs, 27, 0x40 )
							| get_function( adrs, 28, 0x80 );
		}
		return( len );
	}
	//
	//	We have to "do" something..
	//	.. so we substitute in an idle packet.
	//
	command[ 0 ] = 0xff;
	command[ 1 ] = 0x00;
	return( 2 );
}

//
//	Compose a digital reset packet
//
static byte compose_digital_reset( byte *command ) {

	ASSERT( command != NULL );

	command[ 0 ] = 0;
	command[ 1 ] = 0;
	return( 2 );
}

//
//	Compose a set CV with value packet.
//
static byte compose_set_cv( byte *command, int cv, int value ) {

	ASSERT( command != NULL );
	ASSERT(( cv >= MINIMUM_CV_ADDRESS )&&( cv <= MAXIMUM_CV_ADDRESS ));
	ASSERT(( value >= 0 )&&( value <= 255 ));

	cv -= MINIMUM_CV_ADDRESS;
	command[ 0 ] = 0b01111100 | (( cv >> 8 ) & 0b00000011 );
	command[ 1 ] = cv & 0b11111111;
	command[ 2 ] = value;
	return( 3 );
}

//
//	Compose a verify CV with value packet.
//
static byte compose_verify_cv( byte *command, int cv, int value ) {

	ASSERT( command != NULL );
	ASSERT(( cv >= MINIMUM_CV_ADDRESS )&&( cv <= MAXIMUM_CV_ADDRESS ));
	ASSERT(( value >= 0 )&&( value <= 255 ));

	cv -= MINIMUM_CV_ADDRESS;
	command[ 0 ] = 0b01110100 | (( cv >> 8 ) & 0b00000011 );
	command[ 1 ] = cv & 0b11111111;
	command[ 2 ] = value;
	return( 3 );
}

//
//	Compose a verify bit from CV command
//
static byte compose_verify_cv_bit( byte *command, int cv, int bnum, int value ) {

	ASSERT( command != NULL );
	ASSERT(( cv >= MINIMUM_CV_ADDRESS )&&( cv <= MAXIMUM_CV_ADDRESS ));
	ASSERT(( bnum >= 0 )&&( bnum <= 7 ));
	ASSERT(( value == 0 )||( value == 1 ));

	cv -= MINIMUM_CV_ADDRESS;
	command[ 0 ] = 0b01111000 | (( cv >> 8 ) & 0b00000011 );
	command[ 1 ] = cv & 0b11111111;
	command[ 2 ] = 0b11100000 | ( value << 3 ) | bnum;
	return( 3 );
}

#endif

//
//	Liquid Crystal Display summaries
//	================================

#ifdef LCD_DISPLAY_ENABLE

#if LCD_DISPLAY_BUFFER_WIDTH < 4
	//
	//	If there is not enough space for anything.
	//	This is really a coding error.
	//
#	error "LCD_DISPLAY_BUFFER_WIDTH set too small"
#endif

//
//	The following routines provide the mechanism for filling
//	out the display data for buffers that will be shown on the
//	attached LCD.
//

//
//	Summarise the motion command for an mobile decoder
//
static void lcd_summary_motion( char *buffer, int target, int speed, int dir ) {
	//
	//	Fill in the target number.
	//
	if( !backfill_int_to_text( buffer, target, 4 )) {
		memset( buffer, HASH, 4 );
	}

#if LCD_DISPLAY_BUFFER_WIDTH >= 5
	//
	//	The direction
	//
	buffer[ 4 ] = dir? LCD_ACTION_FORWARD: LCD_ACTION_BACKWARDS;
#endif

#if LCD_DISPLAY_BUFFER_WIDTH >= 7
	//
	//	Finally the speed.
	//
	if( !backfill_int_to_text( buffer + 5, speed, 2 )) memset( buffer + 5, HASH, 2 );
#endif

#if LCD_DISPLAY_BUFFER_WIDTH > 7
	//
	//	Space fill the tail area
	//
	memset( buffer + 7, SPACE, LCD_DISPLAY_BUFFER_WIDTH - 7 );
#endif

}

//
//	Summarise an action upon an accessory
//
static void lcd_summary_accessory( char *buffer, int adrs, int subadrs, int state ) {
	//
	//	Place Accessory main address.
	//
	(void)backfill_int_to_text( buffer, adrs, 3 );

#if LCD_DISPLAY_BUFFER_WIDTH >= 5
	//
	//	Place sub address.
	//
	buffer[ 3 ] = '/';
	(void)backfill_int_to_text( buffer + 4, subadrs, 1 );
#endif

#if LCD_DISPLAY_BUFFER_WIDTH >= 7
	//
	//	Place action applied.
	//
	buffer[ 5 ] = state? LCD_ACTION_ENABLE: LCD_ACTION_DISABLE;
	buffer[ 6 ] = SPACE;
#endif

#if LCD_DISPLAY_BUFFER_WIDTH > 7
	//
	//	Space fill the tail area
	//
	memset( buffer + 7, SPACE, LCD_DISPLAY_BUFFER_WIDTH - 7 );
#endif

}

#ifdef DCC_PLUS_PLUS_COMPATIBILITY

//
//	DDC Generator Compatibility Command LCD summary routines.
//	---------------------------------------------------------
//

//
//	Summarise function application to a target
//
static void lcd_summary_function( char *buffer, int target ) {
	//
	//	Fill in the target number.
	//
	if( !backfill_int_to_text( buffer, target, 4 )) {
		memset( buffer, HASH, 4 );
	}

#if LCD_DISPLAY_BUFFER_WIDTH >= 5
	//
	//	The actions taken
	//
	buffer[ 4 ] = 'A';
#endif

#if LCD_DISPLAY_BUFFER_WIDTH > 5
	//
	//	Space fill the tail area.
	//
	memset( buffer + 5, SPACE, LCD_DISPLAY_BUFFER_WIDTH - 5 );
#endif

}

#else

//
//	DDC Generator Native Command LCD summary routines.
//	--------------------------------------------------
//

//
//	Summarise function application to a target
//
static void lcd_summary_function( char *buffer, int target, int func, int state ) {
	//
	//	Fill in the target number.
	//
	if( !backfill_int_to_text( buffer, target, 4 )) {
		memset( buffer, HASH, 4 );
	}

#if LCD_DISPLAY_BUFFER_WIDTH >= 5
	//
	//	The actions taken
	//
	buffer[ 4 ] = state? LCD_ACTION_ENABLE: LCD_ACTION_DISABLE;
#endif

#if LCD_DISPLAY_BUFFER_WIDTH >= 7
	//
	//	Finally function.
	//
	(void)backfill_int_to_text( buffer + 5, func, 2 );
#endif

#if LCD_DISPLAY_BUFFER_WIDTH > 7
	//
	//	Space fill the tail area
	//
	memset( buffer + 7, SPACE, LCD_DISPLAY_BUFFER_WIDTH - 7 );
#endif
}

//
//	Display setting a CV to a value.
//
static void lcd_summary_setcv( char *buffer, int cv, int value ) {
	//
	//	Fill in the CV number.
	//
	if( !backfill_int_to_text( buffer, cv, 3 )) {
		memset( buffer, HASH, 3 );
	}
	buffer[ 3 ] = '=';
	
#if LCD_DISPLAY_BUFFER_WIDTH >= 7
	//
	//	new value.
	//
	(void)backfill_int_to_text( buffer + 4, value, 3 );
#endif

#if LCD_DISPLAY_BUFFER_WIDTH > 7
	//
	//	Space fill the tail area
	//
	memset( buffer + 7, SPACE, LCD_DISPLAY_BUFFER_WIDTH - 7 );
#endif
}

//
//	Display checking a CV against a value.
//
static void lcd_summary_verifycv( char *buffer, int cv, int value ) {
	//
	//	Fill in the CV number.
	//
	if( !backfill_int_to_text( buffer, cv, 3 )) {
		memset( buffer, HASH, 3 );
	}
	buffer[ 3 ] = '?';
	
#if LCD_DISPLAY_BUFFER_WIDTH >= 7
	//
	//	Check value.
	//
	(void)backfill_int_to_text( buffer + 4, value, 3 );
#endif

#if LCD_DISPLAY_BUFFER_WIDTH > 7
	//
	//	Space fill the tail area
	//
	memset( buffer + 7, SPACE, LCD_DISPLAY_BUFFER_WIDTH - 7 );
#endif
}

//
//	Display reading a CV bit (against a known value).
//
static void lcd_summary_readcv( char *buffer, int cv, int bitno, int value ) {
	//
	//	Fill in the CV number.
	//
	if( !backfill_int_to_text( buffer, cv, 3 )) {
		memset( buffer, HASH, 3 );
	}
	buffer[ 3 ] = '/';
	
#if LCD_DISPLAY_BUFFER_WIDTH >= 5
	//
	//	bit number.
	//
	if(( bitno >= 0 )&&( bitno <= 7 )) {
		buffer[ 4 ] = '0' + bitno;
	}
	else {
		buffer[ 4 ] = HASH;
	}
#endif

#if LCD_DISPLAY_BUFFER_WIDTH >= 7
	//
	//	Check value.
	//
	buffer[ 5 ] = '?';
	buffer[ 6 ] = value? '1': '0';
#endif

#if LCD_DISPLAY_BUFFER_WIDTH > 7
	//
	//	Space fill the tail area
	//
	memset( buffer + 7, SPACE, LCD_DISPLAY_BUFFER_WIDTH - 7 );
#endif
}

#endif


#endif
	
//
//	Command Summary
//	===============
//

#ifdef DCC_PLUS_PLUS_COMPATIBILITY

	//
	//	DCC Plus Plus Command Summary
	//	=============================
	//
	//	The following summary provides the sub-set of DCC commands
	//	supported by the DCCpp Arduino Firmware which this code will
	//	try to emulate.
	//
	//	Descriptions of the DCC Packet format will not include the
	//	pre-amble, the inter-byte bits nor the parity byte.
	//
	//	Multi-function address byte(s) are defined as one of the following
	//	two formats (represented in binary):
	//
	//		Short Address range (1-127):	0AAAAAAA
	//
	//		Long Address range (1-10239):	11AAAAAA AAAAAAAA
	//
	//
	//
	//	Note:	The top 6 bits of the long address are restricted (by the
	//		DCC standard) to be from 000000 to 100111 (39 decimal).
	//		As a result the largest long address is 10 0111 1111 1111
	//		translating to 10239.
	//
	//
	//	Set Engine Throttle (126-Step)
	//	------------------------------
	//    
	//		<t BUFFER ADRS SPEED DIR>
	//
	//		BUFFER:	An internal buffer number, from 1 through to MOBILE_TRANS_BUFFERS
	//		ADRS:	The short (1-127) or long (128-10239) address of the engine decoder
	//		SPEED:	Throttle speed from 0-126, or -1 for emergency stop
	//		DIR:	1=forward, 0=reverse
	//    
	//	returns:
	//
	//		<T BUFFER SPEED DIRECTION>
	//
	//	DCC Packet format:
	//
	//		[0AAAAAAA|11AAAAAA AAAAAAAA] 00111111 DSSSSSSS
	//
	//	Where:
	//		A:		Address data.
	//		D:		Direction; 1=forwards, 0=backwards.
	//		SSSSSSS:	Speed; 0=stop, 1=emergency stop, or target speed+1
	//
	//
	//
	//	Modify Engine Function F0-F28
	//	-----------------------------    
	//
	//		<f ADRS BYTE1 [BYTE2]>
	//
	//		ADRS:	The short (1-127) or long (128-10239) address of the engine decoder
	//
	//	In the following options [Fn] represents the bit for function "n", where a 1 would
	//	activate it and a 0 disable it (byte values shown in binary).
	//
	//	To set functions F0-F4:
	//      
	//		BYTE1:  100[F0][F4][F3][F2][F1]
	//   
	//	To set functions F5-F8:
	//   
	//		BYTE1:  1011[F8][F7][F6][F5]
	//   
	//	To set functions F9-F12:
	//   
	//		BYTE1:  1010[F12][F11][F10][F9]
	//   
	//	To set functions F13-F20:
	//   
	//		BYTE1: 11011110 
	//		BYTE2: [F20][F19][F18][F17][F16][F15][F14][F13]
	//   
	//	To set functions F21-F28:
	//   
	//		BYTE1: 11011111
	//		BYTE2: [F28][F27][F26][F25][F24][F23][F22][F21]
	//   
	//	Returns: Nothing
	//
	//	DCC Packet format:
	//
	//		[0AAAAAAA|11AAAAAA AAAAAAAA] {BYTE1} [{BYTE2}]
	//
	//	The nature of this functions interface lends itself for abuse as it simply
	//	copies the supplied bytes into the output packet.  The caller of this function
	//	could supply any data and so perform any function.
	// 
	//
	//	Operate Accessory Decoder
	//	-------------------------    
	//
	//		<a ADRS SUBADRS STATE>
	//
	//		ADRS:	The primary address of the decoder (0-511)
	//		SUBADRS:The sub-address of the decoder (0-3)
	//		STATE:	1=on (set), 0=off (clear)
	//
	//	Returns: Nothing
	//
	//	DCC Packet format:
	//
	//		10AAAAAA 1aaa1SSC
	//
	//
	//	Where:
	//
	//		aaaAAAAAA:	Accessory address (a's are ~'d)
	//		SS:		Sub address
	//		C:		Target state 1=on, 0=off
	//
	//
	//	Power ON/OFF to the track
	//	-------------------------
	//
	//	Neither of these commands have any functional scope in the DCC
	//	protocol as these directly operate against the Arduino itself.
	//
	//	Power ON the track(s):
	//
	//		<1>
	//
	//	Returns: <p1>
	//
	//	Power OFF the track(s):
	//
	//		<0>
	//
	//	Returns: <p0>
	//

#else

	//
	//	DCC Generator Command Summary
	//	=============================
	//
	//	For the moment the following commands are described in outline
	//	only; details to be provided.
	//
	//	Mobile decoder set speed and direction
	//	--------------------------------------
	//
	//	[M ADRS SPEED DIR] -> [M ADRS SPEED DIR]
	//
	//		ADRS:	The short (1-127) or long (128-10239) address of the engine decoder
	//		SPEED:	Throttle speed from 0-126, or -1 for emergency stop
	//		DIR:	1=Forward, 0=Reverse
	//
	//	Accessory decoder set state
	//	---------------------------
	//
	//	[A ADRS STATE] -> [A ADRS STATE]
	//
	//		ADRS:	The combined address of the decoder (1-2048)
	//		STATE:	1=on (set), 0=off (clear)
	//
	//	Mobile decoder set function state
	//	---------------------------------
	//
	//	[F ADRS FUNC VALUE] -> [F ADRS FUNC STATE]
	//
	//		ADRS:	The short (1-127) or long (128-10239) address of the engine decoder
	//		FUNC:	The function number to be modified (0-21)
	//		VALUE:	1=Enable, 0=Disable
	//		STATE:	1=Confirmed, 0=Failed
	//	
	//	Enable/Disable Power to track
	//	-----------------------------
	//
	//	[P STATE] -> [P STATE]
	//
	//		STATE: 1=On, 0=Off
	//
	//	Set CV value (Programming track)
	//	--------------------------------
	//
	//	[S CV VALUE] -> [S CV VALUE STATE]
	//
	//		CV:	Number of CV to set (1-1024)
	//		VALUE:	8 bit value to apply (0-255)
	//		STATE:	1=Confirmed, 0=Failed
	//
	//	Verify CV value (Programming track)
	//	-----------------------------------
	//
	//	[V CV VALUE] -> [V CV VALUE STATE]
	//
	//		CV:	Number of CV to set (1-1024)
	//		VALUE:	8 bit value to apply (0-255)
	//		STATE:	1=Confirmed, 0=Failed
	//
	//	Verify CV bit value (Programming track)
	//	---------------------------------------
	//
	//	Compare the specified CV bit with the supplied
	//	value, if they are the same, return 1, otherwise
	//	(or in the case of failure) return 0.
	//
	//	[R CV BIT VALUE] -> [R CV BIT STATE]
	//
	//		CV:	Number of CV to set (1-1024)
	//		BIT:	Bit number (0 LSB - 7 MSB)
	//		VALUE:	0 or 1
	//		STATE:	1=Confirmed, 0=Failed
	//
	//	Asynchronous data returned from the firmware
	//	============================================
	//
	//	Change in Power state:
	//
	//		-> [P STATE]
	//
	//	Current power consumption of the system:
	//
	//		-> [L LOAD]
	//
	//		LOAD: Figure between 0 and 1023
	//
	//	Error detected by the firmware
	//
	//		-> [E ERR ARG]
	//
	//		ERR:	Error number giving nature of problem
	//		ARG:	Additional information data, nature
	//			dependant on the error number.
	//

#endif

//
//	The maximum number of arguments after the command letter.
//
#define MAX_DCC_ARGS	4

//
//	Simple compiler time check so we do not need to do pointless
//	check in the run time code.
//
#if MAXIMUM_DCC_COMMAND < 5
#error "MAXIMUM_DCC_COMMAND too small."
#endif

//
//	Define the size of a reply buffer
//
#define MAXIMUM_REPLY_SIZE	32
//
//	The command interpreting routine.
//
static void scan_line( char *buf ) {
	//
	//	Where we build the command
	//
	char	cmd;
	int	arg[ MAX_DCC_ARGS ], args;
	//
	//	Where we construct the DCC packet data.
	//
	byte	command[ MAXIMUM_DCC_COMMAND ];

	//
	//	Take the data proved and parse it into useful pieces.
	//
	if(( args = parse_input( buf, &cmd, arg, MAX_DCC_ARGS )) != ERROR ) {
		//
		//	We have at least a command and (possibly) some arguments.
		//
		switch( cmd ) {

			//
			//	Power management commands
			//	-------------------------
			//
#ifdef DCC_PLUS_PLUS_COMPATIBILITY
			case '0':
			case '1': {
				char	reply[ 8 ];

				//
				//	<1>	Turn track power on
				//	<0>	Turn track power off
				//
				if( args != 0 ) {
					report_error( INVALID_ARGUMENT_COUNT, cmd );
					break;
				}
				if( cmd == '1' ) {
					if( power_on_main_track()) {
						MONITOR_ANALOGUE_PIN( MAIN_TRACK_ANALOGUE );
						link_main_buffers();
					}
					reply_1( reply, 'p', 1 );
					if( !queue_output( reply )) report_error( COMMAND_REPORT_FAIL, cmd );
				}
				else {
					(void)power_off_tracks();
					reply_1( reply, 'p', 0 );
					if( !queue_output( reply )) report_error( COMMAND_REPORT_FAIL, cmd );
				}
				
				break;
			}
			case 'c': {
				//
				//	Report power
				//
				if( args != 0 ) {
					report_error( INVALID_ARGUMENT_COUNT, cmd );
					break;
				}
				report_track_power();
				break;
			}

#else
			case 'P': {
				char	reply[ 8 ];

				//	
				//	Enable/Disable Power to track
				//	-----------------------------
				//
				//	[P STATE] -> [P STATE]
				//
				//		STATE: 0=Off, 1=Main, 2=Prog
				//
				if( args != 1 ) {
					report_error( INVALID_ARGUMENT_COUNT, cmd );
					break;
				}
				switch( arg[ 0 ]) {
					case 0: {	// Power track off
						(void)power_off_tracks();
						reply_1( reply, 'P', 0 );
						if( !queue_output( reply )) report_error( COMMAND_REPORT_FAIL, cmd );
						break;
					}
					case 1: {
						//
						//	Power Main Track on
						//
						if( global_power_state != GLOBAL_POWER_OFF ) {
							report_error( POWER_NOT_OFF, cmd );
							break;
						}
						if( power_on_main_track()) {
							//
							//	Set up power monitor and
							//	circular buffers
							//
							MONITOR_ANALOGUE_PIN( MAIN_TRACK_ANALOGUE );
							link_main_buffers();
						}
						reply_1( reply, 'P', 1 );
						if( !queue_output( reply )) report_error( COMMAND_REPORT_FAIL, cmd );
						break;
					}
					case 2: {
						//
						//	Power Prog Track on
						//
						if( global_power_state != GLOBAL_POWER_OFF ) {
							report_error( POWER_NOT_OFF, cmd );
							break;
						}
						if( power_on_prog_track()) {
							//
							//	Set up power monitor and
							//	circular buffers
							//
							MONITOR_ANALOGUE_PIN( PROG_TRACK_ANALOGUE );
							link_prog_buffers();
						}
						reply_1( reply, 'P', 2 );
						if( !queue_output( reply )) report_error( COMMAND_REPORT_FAIL, cmd );
						break;
					}
					default: {
						report_error( INVALID_STATE, cmd );
						break;
					}
				}
				break;
			}
#endif
			
			//
			//	Cab/Mobile decoder commands
			//	---------------------------
			//
#ifdef DCC_PLUS_PLUS_COMPATIBILITY
			case 't': {
				PENDING_PACKET	**tail;
				TRANS_BUFFER	*buf;
				int		buffer,
						target,
						speed,
						dir;

				//
				//	Set throttle speed for engine ADRS in buffer BUFFER.
				//
				//	<t BUFFER ADRS SPEED DIR>
				//
				//	[0AAAAAAA|11AAAAAA AAAAAAAA] 00111111 DSSSSSSS
				//
				if( args != 4 ) {
					report_error( INVALID_ARGUMENT_COUNT, cmd );
					break;
				}
				//
				//	Save command arguments.
				//
				buffer = arg[ 0 ];
				target = arg[ 1 ];
				speed = arg[ 2 ];
				dir = arg[ 3 ];
				//
				//	Verify ranges
				//
				if(( buffer < 1 )||( buffer > MOBILE_TRANS_BUFFERS )) {
					report_error( INVALID_BUFFER_NUMBER, buffer );
					break;
				}
				if(( target < MINIMUM_DCC_ADDRESS )||( target > MAXIMUM_DCC_ADDRESS )) {
					report_error( INVALID_ADDRESS, target );
					break;
				}
				if((( speed < MINIMUM_DCC_SPEED )||( speed > MAXIMUM_DCC_SPEED ))&&( speed != EMERGENCY_STOP )) {
					report_error( INVALID_SPEED, speed );
					break;
				}
				if(( dir != DCC_FORWARDS )&&( dir != DCC_BACKWARDS )) {
					report_error( INVALID_DIRECTION, dir );
					break;
				}
				//
				//	Locate target buffer, and prepare pending list.
				//
				buf = circular_buffer + ( MOBILE_BASE_BUFFER + buffer - 1 );
				buf->pending = release_pending_recs( buf->pending, false );
				tail = &( buf->pending );
				//
				//	Check buffer is in usable state.
				//
				if(( buf->state != TBS_RUN )&&( buf->state != TBS_EMPTY )) {
					//
					//	We cannot modify this buffer.
					//
					report_error( TRANSMISSION_BUSY, cmd );
					break;
				}
				//
				//	Now create and append the command to the pending list.
				//
				if( !create_pending_rec( &tail, target, 0, false, compose_motion_packet( command, target, speed, dir ), command )) {
					//
					//	Report that no pending record has been created.
					//
					report_error( COMMAND_QUEUE_FAILED, cmd );
					break;
				}

#ifdef LCD_DISPLAY_ENABLE
				//
				//	Complete LCD summary.
				//
				lcd_summary_motion( buf->display, target, speed, dir );
#endif

				//
				//	Construct the reply to send when we get send
				//	confirmation and pass to the manager code to
				//	insert the new packet into the transmission
				//	process.
				//
				reply_3( buf->contains, 'T', buffer, speed, dir );
				buf->reply = REPLY_ON_SEND;
				buf->state = ( buf->state == TBS_EMPTY )? TBS_LOAD: TBS_RELOAD;
				break;
			}
#else
			case 'M': {
				PENDING_PACKET	**tail;
				TRANS_BUFFER	*buf;
				int		target,
						speed,
						dir;

				//
				//	Mobile decoder set speed and direction
				//	--------------------------------------
				//
				//	[M ADRS SPEED DIR] -> [M ADRS SPEED DIR]
				//
				//		ADRS:	The short (1-127) or long (128-10239) address of the engine decoder
				//		SPEED:	Throttle speed from 0-126, or -1 for emergency stop
				//		DIR:	1=Forward, 0=Reverse
				//
				if( args != 3 ) {
					report_error( INVALID_ARGUMENT_COUNT, cmd );
					break;
				}
				//
				//	Save command arguments.
				//
				target = arg[ 0 ];
				speed = arg[ 1 ];
				dir = arg[ 2 ];
				//
				//	Verify ranges
				//
				if(( target < MINIMUM_DCC_ADDRESS )||( target > MAXIMUM_DCC_ADDRESS )) {
					report_error( INVALID_ADDRESS, target );
					break;
				}
				if((( speed < MINIMUM_DCC_SPEED )||( speed > MAXIMUM_DCC_SPEED ))&&( speed != EMERGENCY_STOP )) {
					report_error( INVALID_SPEED, speed );
					break;
				}
				if(( dir != DCC_FORWARDS )&&( dir != DCC_BACKWARDS )) {
					report_error( INVALID_DIRECTION, dir );
					break;
				}
				//
				//	Find a destination buffer
				//
				if(( buf = find_available_buffer( MOBILE_BASE_BUFFER, MOBILE_TRANS_BUFFERS, target )) == NULL ) {
					//
					//	No available buffers
					//
					report_error( TRANSMISSION_BUSY, cmd );
					break;
				}
				buf->pending = release_pending_recs( buf->pending, false );
				tail = &( buf->pending );
				//
				//	Now create and append the command to the pending list.
				//
				if( !create_pending_rec( &tail, target, ((( speed == EMERGENCY_STOP )||( speed == MINIMUM_DCC_SPEED ))? TRANSIENT_COMMAND_REPEATS: 0 ), false, compose_motion_packet( command, target, speed, dir ), command )) {
					//
					//	Report that no pending record has been created.
					//
					report_error( COMMAND_QUEUE_FAILED, cmd );
					break;
				}

#ifdef LCD_DISPLAY_ENABLE
				//
				//	Complete LCD summary.
				//
				lcd_summary_motion( buf->display, target, speed, dir );
#endif

				//
				//	Construct the reply to send when we get send
				//	confirmation and pass to the manager code to
				//	insert the new packet into the transmission
				//	process.
				//
				reply_3( buf->contains, 'M', target, speed, dir );
				buf->reply = REPLY_ON_SEND;
				buf->state = ( buf->state == TBS_EMPTY )? TBS_LOAD: TBS_RELOAD;
				break;
			}
#endif

			//
			//	Accessory Commands
			//	------------------
			//
#ifdef DCC_PLUS_PLUS_COMPATIBILITY
			case 'a': {
				PENDING_PACKET	**tail;
				TRANS_BUFFER	*buf;
				int		adrs,
						subadrs,
						state,
						target;

				//
				//	Modify the state of an accessory.
				//
				//	<a ADRS SUBADRS STATE>
				//
				//		ADRS:	The primary address of the decoder (0-511)
				//		SUBADRS:The sub-address of the decoder (0-3)
				//		STATE:	1=on (set), 0=off (clear)
				//
				//	10AAAAAA 1aaa1SSC
				//
				if( args != 3 ) {
					report_error( INVALID_ARGUMENT_COUNT, cmd );
					break;
				}
				//
				//	Save arguments
				//
				adrs = arg[ 0 ];
				subadrs = arg[ 1 ];
				state = arg[ 2 ];
				//
				//	Verify ranges.
				//
				if(( adrs < MIN_ACCESSORY_ADDRESS )||( adrs > MAX_ACCESSORY_ADDRESS )) {
					report_error( INVALID_ADDRESS, adrs );
					break;
				}
				if(( subadrs < MIN_ACCESSORY_SUBADRS )||( subadrs > MAX_ACCESSORY_SUBADRS )) {
					report_error( INVALID_ADDRESS, subadrs );
					break;
				}
				if(( state != ACCESSORY_ON )&&( state != ACCESSORY_OFF )) {
					report_error( INVALID_STATE, state );
					break;
				}
				//
				//	Generate internal target ID.  We negate it here as
				//	we use negative numbers to distinguish accessory targets
				//	from mobile decoder targets.
				//
				target = -external_acc_target( adrs, subadrs );
				//
				//	Find a destination buffer
				//
				if(( buf = find_available_buffer( ACCESSORY_BASE_BUFFER, ACCESSORY_TRANS_BUFFERS, target )) == NULL ) {
					//
					//	No available buffers
					//
					report_error( TRANSMISSION_BUSY, cmd );
					break;
				}
				buf->pending = release_pending_recs( buf->pending, false );
				tail = &( buf->pending );
				//
				//	Now create and append the command to the pending list.
				//
				if( !create_pending_rec( &tail, target, TRANSIENT_COMMAND_REPEATS, false, compose_accessory_change( command, adrs, subadrs, state ), command )) {
					//
					//	Report that no pending has been record created.
					//
					report_error( COMMAND_QUEUE_FAILED, cmd );
					break;
				}

#ifdef LCD_DISPLAY_ENABLE
				//
				//	Complete LCD summary.
				//
				lcd_summary_accessory( buf->display, adrs, subadrs, state );
#endif

				//
				//	No reply required.
				//
				buf->reply = NO_REPLY_REQUIRED;
				buf->state = ( buf->state == TBS_EMPTY )? TBS_LOAD: TBS_RELOAD;
				break;
			}
#else
			case 'A': {
				PENDING_PACKET	**tail;
				TRANS_BUFFER	*buf;
				int		target,
						adrs,
						subadrs,
						state;

				//
				//	Accessory decoder set state
				//	---------------------------
				//
				//	[A ADRS STATE] -> [A ADRS STATE]
				//
				//		ADRS:	The combined address of the decoder (1-2048)
				//		STATE:	1=on (set), 0=off (clear)
				//
				if( args != 2 ) {
					report_error( INVALID_ARGUMENT_COUNT, cmd );
					break;
				}
				//
				//	Save arguments.
				//
				target = arg[ 0 ];
				state = arg[ 1 ];
				//
				//	Verify ranges
				//
				if(( target < MIN_ACCESSORY_EXT_ADDRESS )||( target > MAX_ACCESSORY_EXT_ADDRESS )) {
					report_error( INVALID_ADDRESS, target );
					break;
				}
				if(( state != ACCESSORY_ON )&&( state != ACCESSORY_OFF )) {
					report_error( INVALID_STATE, state );
					break;
				}
				//
				//	Convert to internal address and sub-address values and
				//	remember to invert the external target number since we
				//	use negative numbers to represent accessories internally.
				//
				adrs = internal_acc_adrs( target );
				subadrs = internal_acc_subadrs( target );
				target = -target;
				//
				//	Find a destination buffer
				//
				if(( buf = find_available_buffer( ACCESSORY_BASE_BUFFER, ACCESSORY_TRANS_BUFFERS, target )) == NULL ) {
					//
					//	No available buffers
					//
					report_error( TRANSMISSION_BUSY, cmd );
					break;
				}
				buf->pending = release_pending_recs( buf->pending, false );
				tail = &( buf->pending );
				//
				//	Now create and append the command to the pending list.
				//
				if( !create_pending_rec( &tail, target, TRANSIENT_COMMAND_REPEATS, false, compose_accessory_change( command, adrs, subadrs, state ), command )) {
					//
					//	Report that no pending has been record created.
					//
					report_error( COMMAND_QUEUE_FAILED, cmd );
					break;
				}

#ifdef LCD_DISPLAY_ENABLE
				//
				//	Complete LCD summary.
				//
				lcd_summary_accessory( buf->display, adrs, subadrs, state );
#endif

				//
				//	Construct the future reply and set the state (un-negate
				//	target value).
				//
				reply_2( buf->contains, 'A', -target, state );
				buf->reply = REPLY_ON_SEND;
				buf->state = ( buf->state == TBS_EMPTY )? TBS_LOAD: TBS_RELOAD;
				break;
			}
#endif

			//
			//	Mobile decoder functions
			//	------------------------
			//
#ifdef DCC_PLUS_PLUS_COMPATIBILITY
			case 'f': {
				PENDING_PACKET	**tail;
				TRANS_BUFFER	*buf;
				int		target,
						byte1,
						byte2,
						len;

				//
				//	Activate/Deactivate functions within a mobile decoder.
				//
				//		<f ADRS BYTE1 [BYTE2]>
				//
				//		ADRS:  the short (1-127) or long (128-10239) address of the engine decoder
				//
				//		BYTE1:	Mandatory initial data byte
				//
				//		BYTE2:	Optional second data byte
				//
				//	[0AAAAAAA|11AAAAAA AAAAAAAA] {BYTE1} [ {BYTE2} ]
				//
				if(( args != 2 )&&( args != 3 )) {
					report_error( INVALID_ARGUMENT_COUNT, cmd );
					break;
				}
				//
				//	Gather parameters
				//
				target = arg[ 0 ];
				byte1 = arg[ 1 ];
				byte2 = ( args == 3 )? arg[ 2 ]: 0;
				//
				//	Validate ranges
				//
				if(( target < MINIMUM_DCC_ADDRESS )||( target > MAXIMUM_DCC_ADDRESS )) {
					report_error( INVALID_ADDRESS, target );
					break;
				}
				//
				//	To sanity check the byte values provided we only need to ensure
				//	that byte 1 is one of a set of formats:
				//
				//	Only validate as one of (checked off in this order):
				//
				//		11011110 
				//		11011111
				//		1011????
				//		1010????
				//		100?????
				//
				if( byte1 != 0b11011110 ) {
					if( byte1 != 0b11011111 ) {
						if(( byte1 & 0b11110000 ) != 0b10110000 ) {
							if(( byte1 & 0b11110000 ) != 0b10100000 ) {
								if(( byte1 & 0b11100000 ) != 0b10000000 ) {
									report_error( INVALID_FUNC_NUMBER, cmd );
									break;
								}
							}
						}
					}
				}
				//
				//	Find a destination buffer
				//
				if(( buf = find_available_buffer( ACCESSORY_BASE_BUFFER, ACCESSORY_TRANS_BUFFERS, arg[ 0 ])) == NULL ) {
					//
					//	No available buffers
					//
					report_error( TRANSMISSION_BUSY, cmd );
					break;
				}
				buf->pending = release_pending_recs( buf->pending, false );
				tail = &( buf->pending );
				//
				//	Build up the packet now the data is validated.
				//
				if( target > MAXIMUM_SHORT_ADDRESS ) {
					command[ 0 ] = 0b11000000 | ( target >> 8 );
					command[ 1 ] = target & 0b11111111;
					len = 2;
				}
				else {
					command[ 0 ] = target;
					len = 1;
				}
				//
				//	Fill in the payload of the DCC packet.
				//
				command[ len++ ] = byte1;
				if( args == 3 ) command[ len++ ] = byte2;
				//
				//	Now append this command to the pending list.
				//
				if( !create_pending_rec( &tail, target, TRANSIENT_COMMAND_REPEATS, false, len, command )) {
					//
					//	Report that no pending has been record created.
					//
					report_error( COMMAND_QUEUE_FAILED, cmd );
					break;
				}
				
#ifdef LCD_DISPLAY_ENABLE
				//
				//	Reduced LCD summary as we have no idea what has been modified.
				//
				lcd_summary_function( buf->display, target );
#endif

				//
				//	There is no future reply so set the state.
				//
				buf->reply = NO_REPLY_REQUIRED;
				buf->state = ( buf->state == TBS_EMPTY )? TBS_LOAD: TBS_RELOAD;
				break;
			}
#else
			case 'F': {
				PENDING_PACKET	**tail;
				TRANS_BUFFER	*buf;
				int		target,
						func,
						state;

				//
				//	Mobile decoder set function state
				//	---------------------------------
				//
				//	[F ADRS FUNC STATE] -> [F ADRS FUNC STATE]
				//
				//		ADRS:	The short (1-127) or long (128-10239) address of the engine decoder
				//		FUNC:	The function number to be modified (0-21)
				//		STATE:	1=Enable, 0=Disable
				//	
				if( args != 3 ) {
					report_error( INVALID_ARGUMENT_COUNT, cmd );
					break;
				}
				//
				//	Gather arguments
				//
				target = arg[ 0 ];
				func = arg[ 1 ];
				state = arg[ 2 ];
				//
				//	Verify ranges
				//
				if(( target < MINIMUM_DCC_ADDRESS )||( target > MAXIMUM_DCC_ADDRESS )) {
					report_error( INVALID_ADDRESS, target);
					break;
				}
				if(( func < MIN_FUNCTION_NUMBER )||( func > MAX_FUNCTION_NUMBER )) {
					report_error( INVALID_FUNC_NUMBER, func );
					break;
				}
				if(( state != FUNCTION_ON )&&( state != FUNCTION_OFF )) {
					report_error( INVALID_STATE, state );
					break;
				}
				//
				//	Find a destination buffer
				//
				if(( buf = find_available_buffer( ACCESSORY_BASE_BUFFER, ACCESSORY_TRANS_BUFFERS, arg[ 0 ])) == NULL ) {
					//
					//	No available buffers
					//
					report_error( TRANSMISSION_BUSY, cmd );
					break;
				}
				buf->pending = release_pending_recs( buf->pending, false );
				tail = &( buf->pending );
				//
				//	Now create and append the command to the pending list.
				//
				if( !create_pending_rec( &tail, target, TRANSIENT_COMMAND_REPEATS, false, compose_function_change( command, target, func, state ), command )) {
					//
					//	Report that no pending has been record created.
					//
					report_error( COMMAND_QUEUE_FAILED, cmd );
					break;
				}

#ifdef LCD_DISPLAY_ENABLE
				//
				//	Complete LCD summary.
				//
				lcd_summary_function( buf->display, target, func, state );
#endif

				//
				//	Prepare the future reply and set new state.
				//
				reply_3( buf->contains, 'F', target, func, state );
				buf->reply = REPLY_ON_SEND;
				buf->state = ( buf->state == TBS_EMPTY )? TBS_LOAD: TBS_RELOAD;
				break;
			}
#endif
			//
			//	Modify CV values on the programming track
			//	-----------------------------------------
			//
#ifdef DCC_PLUS_PLUS_COMPATIBILITY
				//
				//	No compatibility command defined.
				//
#else
			case 'S': {
				PENDING_PACKET	**tail;
				TRANS_BUFFER	*buf;
				int		cv,
						value;
				bool		ok;

				//
				//	Set CV value (Programming track)
				//	--------------------------------
				//
				//	[S CV VALUE] -> [S CV VALUE STATE]
				//
				//		CV:	Number of CV to set (1-1024)
				//		VALUE:	8 bit value to apply (0-255)
				//		STATE:	1=Confirmed, 0=Failed
				//
				if( args != 2 ) {
					report_error( INVALID_ARGUMENT_COUNT, cmd );
					break;
				}
				//
				//	Save arguments
				//
				cv = arg[ 0 ];
				value = arg[ 1 ];
				//
				//	verify ranges
				//
				if(( cv < MINIMUM_CV_ADDRESS )||( cv > MAXIMUM_CV_ADDRESS )) {
					report_error( INVALID_CV_NUMBER, cv );
					break;
				}
				if(( value < 0 )||( value > 255 )) {
					report_error( INVALID_BYTE_VALUE, value );
					break;
				}
				//
				//	Find a destination buffer
				//
				if(( buf = find_available_buffer( PROGRAMMING_BASE_BUFFER, PROGRAMMING_BUFFERS, 0 )) == NULL ) {
					//
					//	No available buffers
					//
					report_error( TRANSMISSION_BUSY, cmd );
					break;
				}
				buf->pending = release_pending_recs( buf->pending, false );
				tail = &( buf->pending );
				//
				//	Build up the command chain..
				//
				ok = create_pending_rec( &tail, 0, SERVICE_MODE_RESET_REPEATS, true, compose_digital_reset( command ), command );
				ok &= create_pending_rec( &tail, 0, SERVICE_MODE_COMMAND_REPEATS, true, compose_set_cv( command, cv, value ), command );
				ok &= create_pending_rec( &tail, 0, SERVICE_MODE_RESET_REPEATS, true, compose_digital_reset( command ), command );
				if( !ok ) {
					//
					//	Report that no pending has been record created.
					//
					buf->pending = release_pending_recs( buf->pending, false );
					report_error( COMMAND_QUEUE_FAILED, cmd );
					break;
				}

#ifdef LCD_DISPLAY_ENABLE
				//
				//	Complete LCD summary.
				//
				lcd_summary_setcv( buf->display, cv, value );
#endif

				//
				//	Construct the future reply and set the state. Use the confirmation version
				//	of the reply routine.
				//
				reply_2c( buf->contains, 'S', cv, value );
				load_confirmed = false;
				buf->reply = REPLY_ON_CONFIRM;
				buf->state = ( buf->state == TBS_EMPTY )? TBS_LOAD: TBS_RELOAD;
				break;
			}
#endif
			//
			//	Verify CV Value
			//	---------------
			//
#ifdef DCC_PLUS_PLUS_COMPATIBILITY
				//
				//	No compatibility command defined.
				//
#else
			case 'V': {
				PENDING_PACKET	**tail;
				TRANS_BUFFER	*buf;
				int		cv,
						value;
				bool		ok;

				//
				//	Verify CV value (Programming track)
				//	-----------------------------------
				//
				//	[V CV VALUE] -> [V CV VALUE STATE]
				//
				//		CV:	Number of CV to set (1-1024)
				//		VALUE:	8 bit value to apply (0-255)
				//		STATE:	1=Confirmed, 0=Failed
				//
				if( args != 2 ) {
					report_error( INVALID_ARGUMENT_COUNT, cmd );
					break;
				}
				//
				//	Save arguments
				//
				cv = arg[ 0 ];
				value = arg[ 1 ];
				//
				//	verify ranges
				//
				if(( cv < MINIMUM_CV_ADDRESS )||( cv > MAXIMUM_CV_ADDRESS )) {
					report_error( INVALID_CV_NUMBER, cv );
					break;
				}
				if(( value < 0 )||( value > 255 )) {
					report_error( INVALID_BYTE_VALUE, value );
					break;
				}
				//
				//	Find a destination buffer
				//
				if(( buf = find_available_buffer( PROGRAMMING_BASE_BUFFER, PROGRAMMING_BUFFERS, 0 )) == NULL ) {
					//
					//	No available buffers
					//
					report_error( TRANSMISSION_BUSY, cmd );
					break;
				}
				buf->pending = release_pending_recs( buf->pending, false );
				tail = &( buf->pending );
				//
				//	Build up the command chain..
				//
				ok = create_pending_rec( &tail, 0, SERVICE_MODE_RESET_REPEATS, true, compose_digital_reset( command ), command );
				ok &= create_pending_rec( &tail, 0, SERVICE_MODE_COMMAND_REPEATS, true, compose_verify_cv( command, cv, value ), command );
				ok &= create_pending_rec( &tail, 0, SERVICE_MODE_RESET_REPEATS, true, compose_digital_reset( command ), command );
				if( !ok ) {
					//
					//	Report that no pending has been record created.
					//
					buf->pending = release_pending_recs( buf->pending, false );
					report_error( COMMAND_QUEUE_FAILED, cmd );
					break;
				}

#ifdef LCD_DISPLAY_ENABLE
				//
				//	Complete LCD summary.
				//
				lcd_summary_verifycv( buf->display, cv, value );
#endif

				//
				//	Construct the future reply and set the state. The '#' in the reply
				//	will be replaced by a 1 or 0 to reflect confirmation.
				//
				reply_2c( buf->contains, 'V', cv, value );
				load_confirmed = false;
				buf->reply = REPLY_ON_CONFIRM;
				buf->state = ( buf->state == TBS_EMPTY )? TBS_LOAD: TBS_RELOAD;
				break;
			}
#endif
			//
			//	Read CV bit value
			//	-----------------
			//
#ifdef DCC_PLUS_PLUS_COMPATIBILITY
				//
				//	No compatibility command defined.
				//
#else
			case 'R': {
				PENDING_PACKET	**tail;
				TRANS_BUFFER	*buf;
				int		cv,
						bnum,
						value;
				bool		ok;

				//
				//	Read CV bit value (Programming track)
				//	-------------------------------------
				//
				//	Compare the specified CV bit with the supplied
				//	value, if they are the same, return 1, otherwise
				//	(or in the case of failure) return 0.
				//
				//	[R CV BIT VALUE] -> [R CV BIT STATE]
				//
				//		CV:	Number of CV to set (1-1024)
				//		BIT:	Bit number (0 LSB - 7 MSB)
				//		VALUE:	0 or 1
				//		STATE:	1=Confirmed, 0=Failed
				//
				//
				if( args != 3 ) {
					report_error( INVALID_ARGUMENT_COUNT, cmd );
					break;
				}
				//
				//	Save arguments
				//
				cv = arg[ 0 ];
				bnum = arg[ 1 ];
				value = arg[ 2 ];
				//
				//	verify ranges
				//
				if(( cv < MINIMUM_CV_ADDRESS )||( cv > MAXIMUM_CV_ADDRESS )) {
					report_error( INVALID_CV_NUMBER, cv );
					break;
				}
				if(( bnum < 0 )||( bnum > 7 )) {
					report_error( INVALID_BIT_NUMBER, bnum );
					break;
				}
				if(( value != 0 )&&( value != 1 )) {
					report_error( INVALID_BIT_VALUE, value );
					break;
				}
				//
				//	Find a destination buffer
				//
				if(( buf = find_available_buffer( PROGRAMMING_BASE_BUFFER, PROGRAMMING_BUFFERS, 0 )) == NULL ) {
					//
					//	No available buffers
					//
					report_error( TRANSMISSION_BUSY, cmd );
					break;
				}
				buf->pending = release_pending_recs( buf->pending, false );
				tail = &( buf->pending );
				//
				//	Build up the command chain..
				//
				ok = create_pending_rec( &tail, 0, SERVICE_MODE_RESET_REPEATS, true, compose_digital_reset( command ), command );
				ok &= create_pending_rec( &tail, 0, SERVICE_MODE_COMMAND_REPEATS, true, compose_verify_cv_bit( command, cv, bnum, value ), command );
				ok &= create_pending_rec( &tail, 0, SERVICE_MODE_RESET_REPEATS, true, compose_digital_reset( command ), command );
				if( !ok ) {
					//
					//	Report that no pending has been record created.
					//
					buf->pending = release_pending_recs( buf->pending, false );
					report_error( COMMAND_QUEUE_FAILED, cmd );
					break;
				}

#ifdef LCD_DISPLAY_ENABLE
				//
				//	Complete LCD summary.
				//
				lcd_summary_readcv( buf->display, cv, bnum, value );
#endif

				//
				//	Construct the future reply and set the state. The '#' in the reply
				//	will be replaced by a 1 or 0 to reflect confirmation.
				//
				reply_2c( buf->contains, 'R', cv, bnum );
				load_confirmed = false;
				buf->reply = REPLY_ON_CONFIRM;
				buf->state = ( buf->state == TBS_EMPTY )? TBS_LOAD: TBS_RELOAD;
				break;
			}
#endif
			default: {
				//
				//	Here we capture any unrecognised command letters.
				//
				report_error( UNRECOGNISED_COMMAND, cmd );
				break;
			}
		}
	}
	//
	//	Done.
	//
}

//
//	This routine gathers input command from the serial port
//	and consolidates it into a buffer ready for processing.
//

//
//	The buffer:
//
static char serial_buf[ SERIAL_BUFFER_SIZE+1 ];
static byte serial_used = 0;
static bool in_cmd_packet = false;

//
//	The character input routine.
//
static void process_input( int count ) {
	while( count-- ) {
		char	c;

		switch(( c = Serial.read())) {
			case PROT_IN_CHAR: {
				//
				//	Found the start of a command, regardless of what we thought
				//	we were doing we clear out the buffer and start collecting
				//	the content of the packet.
				//
				serial_used = 0;
				in_cmd_packet =  true;
				break;
			}
			case PROT_OUT_CHAR: {
				//
				//	If we got here and we were in a command packet then we have
				//	something to work with, maybe.
				//
				if( in_cmd_packet ) {
					//
					//	Process line and reset buffer.
					//
					serial_buf[ serial_used ] = EOS;
					scan_line( serial_buf );
					//
					//	Now reset the buffer space.
					//
					serial_used = 0;
					in_cmd_packet =  false;
				}
				break;
			}
			default: {
				//
				//	If we are inside a command packet then we save character, if there is space.
				//
				if( in_cmd_packet ) {
					if(( c < SPACE )||( c >= DELETE )) {
						//
						//	Invalid character for a command - abandon the current command.
						//
						serial_used = 0;
						in_cmd_packet =  false;
					}
					else {
						//
						//	"Valid" character (at least it is a normal character), so
						//	save it if possible.
						//
						if( serial_used < SERIAL_BUFFER_SIZE-1 ) serial_buf[ serial_used++ ] = c;
					}
				}
				break;
			}
		}
	}
}

//
//	Firmware main processing loop.
//	------------------------------
//

//
//	Define a "watchdog" variable that is used to initiate
//	activities at durations greater than microseconds or
//	milliseconds.
//
static unsigned long watchdog = 0;

//
//	Finally, the main event loop:
//
void loop( void ) {
	int		i;
	unsigned long	now;
	
	//
	//	If there is any data pending on the serial line
	//	grab it
	//
	if(( i = Serial.available())) process_input( i );
	//
	//	Every time we spin through the loop we give the
	//	management service routine a slice of the CPU time
	//	so that buffer transitions between states are
	//	synchronised correctly.
	//
	management_service_routine();
	//
	//	Power related actions triggered only when data is ready
	//
	if( reading_is_ready ) {
		//
		//	Analyse current data.
		//	
		monitor_current_load( track_load_reading );
		
#ifdef DEBUG_STATISTICS
		//
		//	Count power iterations.
		//
		statistic_power++;
#endif

	}
	//
	//	Time for watch dog activities?
	//
	if(( now = millis()) > watchdog ) {
		//
		//	Yes, but before getting on with them, set the
		//	next watchdog time.
		//
		watchdog = now + WATCHDOG_INTERVAL;

#ifndef DCC_PLUS_PLUS_COMPATIBILITY
		//
		//	Forward out an asynchronous power update.
		//
		report_track_power();
#endif

#ifdef LCD_DISPLAY_ENABLE
		//
		//	Send updates to the LCD
		//
		display_lcd_updates( now );
#endif

#ifdef DEBUG_STATISTICS

#define SCALE(v,m,d)	(((long)(v)*(long)(m))/(long)(d))

		//
		//	Output a "per second" statistics analysis
		//	of how often code sections are executing.
		//
		//	This code is *really* slow and should not (for
		//	the moment) be considered for anything other
		//	than manual ad-hoc information gathering.
		//
		queue_output( "STAT:L=" );
		queue_long( SCALE( statistic_looping, 1000, WATCHDOG_INTERVAL ));
		queue_output( ":T=" );
		queue_int( SCALE( statistic_packets, 1000, WATCHDOG_INTERVAL ));
		queue_output( ":P=" );
		queue_int( SCALE( statistic_power, 1000, WATCHDOG_INTERVAL ));
		queue_output( "\n" );
		//
		//	Reset those values to zero
		//
		statistic_looping = 0;
		statistic_power = 0;

#endif
#if defined( DEBUG_STATISTICS )| defined( LCD_DISPLAY_ENABLE )

		statistic_packets = 0;

#endif

	}
	//
	//	Then we give the Error management system an
	//	opportunity to queue some output data.
	//
	flush_error_queue();
	//
	//	Finally, try to return any pending output from
	//	the firmware.
	//
	if(( i = Serial.availableForWrite())) process_output( i );

#ifdef DEBUG_STATISTICS
	//
	//	Count loop iterations.
	//
	statistic_looping++;
#endif
}

//
//	EOF
//

///
///	ArduinoGenerator: Model railway DCC signal generator.
///
/// 	Firmware for an Arduino Uno R3 and Motor shield which
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
//	Arduino DCC Generator V1.3.3
//	============================
//
#define VERSION_NUMBER "1.3.3"

//
//	Modifications to this version:
//
//		Flexible contant handling using the EEPROM to keep
//		'configured' constants.  This has required a migration
//		of constants towards the top of the source file so that
//		the default values are easily tuned.
//
//		Constant configuration accessed and modified via an
//		extension to the communications protocol (see the
//		'Q' command).
//
//	March 2023
//		

//
//	Arduino DCC Generator V1.3.2
//	----------------------------
//
//	Included the set CV bit command '[U cv bnum value]' to the system
//	and also doubled up the command *inside* the command transmission
//	as there is some evidence to suggest that duplicate sequential
//	commands are required before the decode is obliged to reply.
//
//	March 2023
//

//
//	Arduino DCC Generator V1.3.1
//	----------------------------
//
//	Incorporated improved confirmation detections code which (largely)
//	reduces the averaging codes impact in the detection process.
//
//	March 2023
//

//
//
//	Arduino DCC Generator V1.3
//

//
//	This version sees a rewrite of the "programming confirmation
//	mechanism" and an extension of the mechanism that translates
//	DCC byte encoded commands into bit streams as required by the
//	generator code in the interrupt routine.
//
//	Changes in 1.3:
//
//	o	Source code now starting to be placed into separate
//		modules.
//
//	o	System Serial Library replaced with own configurable
//		USART interrupt driven solution.
//
//	o	The "long preamble" flag has been replaced in the DCC
//		command processing (the conversion from byte codes to
//		bit streams) with "preamble bits" and "postamble bits"
//		values.  This change an enabled the CV programming
//		code to introduce a meaningful delay after the specific
//		DCC command in which any reply from the decoder is more
//		readily and reliably determined.
//
//	All-in-all the change in version number to 1.3 is justified as
//	these changes are significant and affect a number of systems in
//	the firmware. 
//
//	Feb 2023
//
//	Arduino DCC Generator V1.2
//	--------------------------
//
//	Feb 2023
//
//	There has been no real version tracking within this sketch for
//	no honestly good reason.  Since version 0.x and through versions
//	1.0 and 1.2 there have been many changes.  This version should
//	probably be called version 1.5 or 1.6, but isn't.  This source
//	code is in retirement phase awaiting replacement with the DCC
//	generator V2.1 (why no V2.0, because that approach was also set
//	to one side).
//
//	Anyway..
//
//
//	A sketch for generating the DCC signal through
//	an Arduino motor shield.
//
//	Minimum hardware required:
//
//		Arduino UNO R3 (or Mega 2560)
//		Arduino Motor Shield or bespoke Driver board
//		15 volt (max) DC power supply
//

//
//	To implement partial compatibility with the Arduino
//	DCC Plus Plus firmware, located at:
//
//		https://github.com/DccPlusPlus/BaseStation
//
//	You should define DCC_PLUS_PLUS_COMPATIBILITY.
//
//	This will only enable main track commands, no programming
//	track commands can be processed.
//
//	If not defined then the default set of commands are defined
//	including a set of programming track commands.
//
//#define DCC_PLUS_PLUS_COMPATIBILITY 1

//
//	To include a programming track as the last DCC output/district
//	(and include the associated programming commands) then
//	define the symbol PROGRAMMING_TRACK.
//
#define PROGRAMMING_TRACK 1

//
//	Program Tuneable Constants
//
#include "Constants.h"

//
//	Include Modules from the "Library" solution...
//
#include "Code_Assurance.h"
#include "Errors.h"
#include "USART.h"

//
//	The CONSOLE device.
//
static Byte_Queue<32>	console_in;
static Byte_Queue<128>	console_out;
static USART_IO		console;
			

//
//	DCC Programming Confirmations
//	=============================
//
//	A "reply" is defined with the following statement from
//	the NMRA Standards document "S 9.2.3" (line numbered 46):
//
//		Basic acknowledgement is defined by the Digital
//		Decoder providing an increased load (positive-delta)
//		on the programming track of at least 60 mA for 6 ms
//		+/-1 ms. It is permissible to provide this increased
//		load by applying power to the motor or other similar
//		device controlled by the Digital Decoder.
//
//
//	The code that does this (see towards the bottom of the
//	monitor_current_load() routine) performs an analysis of
//	the short term current average against the longest term
//	average.  If the short average is larger (by more than a
//	fixed amount) than the long average, then there is a
//	confirmation.
//
//	Currently the delta required between the two *is* fixed, but
//	the levels of the short and long averages are floating.
//
//	I have the following thoughts (re power monitoring and decoder
//	to generator confirmation signal):
//
//	1/	I am probably not actually "looking" at the power levels
//		in a meaningful way.  The use of the averaging system has
//		allowed me to smear the number across a period of time
//		but not allow me to specify what period of time.
//
//	2/	Would, for the programming track, it be better to keep
//		a running maximum over the specified period?
//
//	3/	There is a requirement for a proper delay between issuing
//		the command and seeing if there has been a reply returned
//		from the decoder.
//
//	This has been facilitated through the incorporating firmware
//	applied counts for the number of '1's broadcast for the pre-amble
//	*and* the post-amble.  This has provided a mechanism that enables
//	any reasonable pause to be placed after any command if that
//	command might generate a reply.
//
//	A note on timing (with respect to the post-amble):
//
//	A single '1' bit is 116us long (2 x 58us) while the confirmation
//	window that we are looking over is 6000us long.  This requires a
//	series of a little under 52 extra '1's be appended to specific
//	DCC commands.
//
//	This, for the time being, will be encoded in the macro
//	CONFIRMATION_PAUSE value and added to the end of all commands
//	that reasonably anticipate a reply.
//
#define CONFIRMATION_PAUSE 52

//
//	Debugging options
//	=================
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
//#define DEBUG_BUFFER_MANAGER	1
//#define DEBUG_BIT_SLICER	1
//#define DEBUG_POWER_MONITOR	1

//
//	Quick options sanity check and define selection macro for
//	programming track options.
//
#if defined( DCC_PLUS_PLUS_COMPATIBILITY ) && defined( PROGRAMMING_TRACK )
#error "PROGRAMMING_TRACK cannot be defined when compiling for Compatibility mode"
#endif

#ifdef PROGRAMMING_TRACK
#define SELECT_PROG(y,n)	y
#else
#define SELECT_PROG(y,n)	n
#endif

//
//	Hardware Specific Configuration Definitions
//	===========================================
//
//	The following definitions are used to abstract the differences
//	between each of the boards.
//
//	The Macro "SELECT_SML(s,m,l)" will be used to select alternate
//	configuration values based on the apparent "size" of the target
//	micro controller.  The parameter "s" represents small MCUs with
//	2 KBytes SRAM.  "m" represents systems with between
//	2 and 4 KBytes SRAM.  All other systems will have the "l" value
//	applied.
//
#if defined( __AVR_ATmega328__ )| defined( __AVR_ATmega328P__ )| defined( __AVR_ATmega328PB__ )
//
//	Standard Nano or Uno R3 configuration
//	-------------------------------------
//
#define HW_TITLE		"AVR ATmega328"

//
//	SRAM = 2 KBytes
//
#define SELECT_SML(s,m,l)	s

//
//	Map Timer symbols onto target Timer hardware
//
#define HW_TCCRnA		TCCR2A
#define HW_TCCRnB		TCCR2B
#define HW_TIMERn_COMPA_vect	TIMER2_COMPA_vect
#define HW_TCNTn		TCNT2
#define HW_OCRnA		OCR2A
#define HW_WGMn1		WGM21
#define HW_CSn0			CS20
#define HW_CSn1			CS21
#define HW_TIMSKn		TIMSK2
#define HW_OCIEnA		OCIE2A

#elif defined( __AVR_ATmega2560__ )
//
//	Standard Mega 2560 configuration
//	--------------------------------
//
#define HW_TITLE		"AVR ATmega2560"

//
//	SRAM = 8 KBytes
//
#define SELECT_SML(s,m,l)	l

//
//	Map Timer symbols onto target Timer hardware
//
#define HW_TCCRnA		TCCR2A
#define HW_TCCRnB		TCCR2B
#define HW_TIMERn_COMPA_vect	TIMER2_COMPA_vect
#define HW_TCNTn		TCNT2
#define HW_OCRnA		OCR2A
#define HW_WGMn1		WGM21
#define HW_CSn0			CS20
#define HW_CSn1			CS21
#define HW_TIMSKn		TIMSK2
#define HW_OCIEnA		OCIE2A

#elif defined( __AVR_ATmega4809__ )
//
//	Arduino Every ATmega4809 configuration
//	--------------------------------------
//
#define HW_TITLE		"AVR ATmega4809"
//
//	SRAM = 6 KBytes
//
#define SELECT_SML(s,m,l)	l

//
//	This *Will Not* compile, yet.  This is due
//	to the substantially different TWI interface.
//
//	Timer macros to be defined too.
//
#error "AVR ATmega4809 not supported yet, in development"

#elif defined( __AVR_ATmega32U4__ )
//
//	Arduino Every ATmega32U4 configuration
//	--------------------------------------
//
#define HW_TITLE		"AVR ATmega32U4"

//
//	SRAM = 2.5 KBytes
//
#define SELECT_SML(s,m,l)	m

//
//	Map Timer symbols onto target Timer hardware
//
#define HW_TCCRnA		TCCR0A
#define HW_TCCRnB		TCCR0B
#define HW_TIMERn_COMPA_vect	TIMER0_COMPA_vect
#define HW_TCNTn		TCNT0
#define HW_OCRnA		OCR0A
#define HW_WGMn1		WGM01
#define HW_CSn0			CS00
#define HW_CSn1			CS01
#define HW_TIMSKn		TIMSK0
#define HW_OCIEnA		OCIE0A

#else
//
//	Firmware has not been configured for this board.
//
#error "Firmware has not been configured for this board"

#endif

//
//	Liquid Crystal Display
//	======================
//
//	To implement an I2C connected LCD display, define the
//	following macros and values.
//
//	LCD_DISPLAY_ENABLE	Simple define to include required code.
//	LCD_DISPLAY_ROWS	Number of rows the display has.
//	LCD_DISPLAY_COLS	Number of columns available per row.
//	LCD_DISPLAY_ADRS	The I2C address of the display
//
//	The display is assumed to be attached using a generic
//	PCF8575 I2C to Parallel adaptor.  The following default
//	definitions apply to a generic 20x4 display.
//
#define LCD_DISPLAY_ENABLE
#define LCD_DISPLAY_ROWS	4
#define LCD_DISPLAY_COLS	20
#define LCD_DISPLAY_ADRS	0x27
//
//	Define a set of single character symbols to represent
//	actions/directions applied to decoders/accessories when
//	displayed on the LCD.
//
#define LCD_ACTION_FORWARD	'>'
#define LCD_ACTION_BACKWARDS	'<'
#define LCD_ACTION_ENABLE	'^'
#define LCD_ACTION_DISABLE	'v'
#define LCD_ACTION_TOGGLE	'~'

//
//	Finally, on LCDs..
//
//	From a wiring perspective (apart from +5V and Ground) the
//	display is attached to the Arduino Uno* via the pass-through pins
//	D18/A4 and D19/A5 on the motor shield.  These equate to
//	the I2C/TWI interface pins SDA (D18) and SCL (D19).  These will
//	be different on other platforms.
//
//	To keep memory allocation to a minimum (at least on the basic
//	Arduino UNO) these "Lite" version of the Wire, TWI and LCD_I2C
//	libraries are required (found with this code).
//
//	*/ Arduino Mega uses different pins: D20/SDA and D21/SCL.  These
//	are outside the footprint of a standard motor shield and need to
//	picked up directly from the Mega itself.
//
//	Note that the bespoke multi-district DCC board has a specific screw
//	terminal broken out for the LCD display.
//

//
//	Include Local and System Libraries
//	==================================
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
//	These are the original "non-blocking" libraries
//	developed by Jeff Penfold (jeff.penfold@googlemail.com).
//
//	It should be noted that these have been superseded but
//	have not been replaced.
//

#include "TWI_IO.h"
#include "LCD_TWI_IO.h"

#endif

//
//	General universal constants
//	===========================
//
#define NL	'\n'
#define CR	'\r'
#define EOS	'\0'
#define SPACE	' '
#define HASH	'#'
#define MINUS	'-'
#define PLUS	'+'
#define USCORE	'_'
#define DELETE	'\177'
#define ERROR	(-1)

//
//	Define the size of a generic small textual buffer for
//	use on the stack.
//
#define TEXT_BUFFER SELECT_SML(8,12,16)

//
//	Compatibility Support
//	=====================
//
//	Used where "whole code" substitution would make the code
//	unnecessarily obscure.  "a" for native mode, "b" for
//	compatibility mode.
//
#ifdef DCC_PLUS_PLUS_COMPATIBILITY
#define SELECT_COMPAT(a,b)		b
#else
#define SELECT_COMPAT(a,b)		a
#endif

//
//	Boot Splash Screen
//	==================
//
//	Do we want a splash screen on power up and
//	what should it say?
//
//	Define SPLASH_ENABLE to include splash code,
//	and SPLASH_LINE_1, _2, _3, _4 (as appropriate
//	for the target display) as text string to
//	display on boot.  Define SPLASH_WAIT as a
//	pause period (ms) before rolling on to the
//	the firmware.
//
#define SPLASH_ENABLE
#define SPLASH_LINE_1	"Vers: " VERSION_NUMBER ", " __DATE__
#define SPLASH_LINE_2	"Mode: " SELECT_COMPAT( "Native " SELECT_PROG( "+", "-" ) "PT", "Compat" ) " " SELECT_SML( "(S)", "(M)", "(L)" )
#define SPLASH_LINE_3	"Model: " HW_TITLE
#define SPLASH_LINE_4	"Baud: " SERIAL_BAUD_RATE_STR
#define SPLASH_WAIT	3000

//
//	"NOW" - a local copy of millis()
//
//	We don't need to know "exactly" what the time is,
//	only maintain a feeling for the on going passage of
//	time.  Updating this in "loop()" is sufficient for the
//	reset of the firmware to track time and schedule actions.
//
static unsigned long now;

//
//	Serial Host Connectivity
//	========================
//

//
//	The following definitions allow for the similarity which is
//	present to be capitalised on and not cause the source code to
//	become unnecessarily convoluted.
//
//	PROT_IN_CHAR	Define the start and end characters of a sentence
//	PROT_OUT_CHAR	in the target operating system mode syntax.
//	
#define PROT_IN_CHAR		SELECT_COMPAT( '[', '<' )
#define PROT_OUT_CHAR		SELECT_COMPAT( ']', '>' )

//
//	Define the Speed and buffer size used when accessing the
//	serial port.
//
//	Example baud rates:
//
//		9600 14400 19200 38400 57600 115200
//
//	The new USART module uses specific constant values for
//	the various supported baud rates.  These are all preceeded
//	with a 'B'.
//
#define SERIAL_BAUD_RATE	SELECT_COMPAT( B38400, B115200 )
#define SERIAL_BAUD_RATE_STR	SELECT_COMPAT( "38400", "115200" )

//
//	High Level Configuration values.
//	================================
//

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
//	Define the maximum number of bytes that can be accepted as
//	a complete input command.
//
#define MAXIMUM_DCC_CMD		16

//
//	Define a maximum number of characters that are required to
//	formulate the host reply to a command being received successfully.
//
#define MAXIMUM_DCC_REPLY	16

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
#define ACCESSORY_TRANS_BUFFERS	SELECT_SML( 5, 6, 8 )
#define MOBILE_TRANS_BUFFERS	SELECT_SML( 4, 6, 8 )
//
//	Note, number of buffers for programming only valid as 1
//	if a programming track is supported, or 0 if it is
//	explicitly not required.
//
#define PROGRAMMING_BUFFERS	SELECT_PROG( 1, 0 )


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
#define TRANSMISSION_BUFFERS	(ACCESSORY_TRANS_BUFFERS+MOBILE_TRANS_BUFFERS+PROGRAMMING_BUFFERS)

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
#define FUNCTION_OFF		0
#define FUNCTION_ON		1
#define FUNCTION_TOGGLE		2

//
//	Motor Shield definitions.
//	=========================
//
//	This DCC Generator firmware now supports multiple DCC
//	Districts in addition to an optional programming track.
//
//	This create a selection of coding, performance and optimisation
//	challenges which are all closely related to the type and
//	configuration of the Motor Shield installed upon the Arduino.
//
//	The following definitions provide the higher level guidance
//	to the source code allowing alternative methods and
//	optimisations to be selected as permitted by the installed
//	motor shield.
//
//	SHIELD_OUTPUT_DRIVERS	This is set to the number of independent
//				H-Bridge driver circuits which the
//				motor shield contains.
//
//	SHIELD_PORT_DIRECT	Define this with the port name of the
//				pin register that contains all of the
//				direction pins to enable code which accesses
//				the port directly.
//
//	Note:	Enabling SHIELD_PORT_DIRECT code will only work for shields
//		where all direction function pins are all grouped within a
//		single pin register.  It is also a requirement that this
//		register is not used for any other functions.
//
//	The configuration of the motor shield is captured using the following
//	structure definition which should be stored in the Arduino program
//	memory.
//
//	Where a brake pin is not available (specifically on the bespoke DCC
//	Arduino shield) use BRAKE_NOT_AVAILABLE as its pin value.  This will
//	prevent the firmware from trying to force the pin LOW to make the
//	pin safe.
//
#define SHIELD_DRIVER struct shield_driver
SHIELD_DRIVER {
	bool		main;		// True if this is an operational
					// track driver, false if it is the
					// programming track.
	byte		direction,	// Which pin controls the polarity
					// of the output.  When SHIELD_PORT_DIRECT
					// is defined this becomes the binary
					// mask of the bit in that port that
					// controls the direction for this
					// driver (ie bit 3 == 0b00001000)
					// The "bit()" can be used to simplify
					// this definition process.
			enable,		// Which pin is used to enable the
					// output from the driver.
			brake,		// Pin to clear on startup.
			load,		// Pin to read driver loading from.
			analogue;	// Same pin as above but numerically
					// suitable for use with setting up
					// the asynchronous ADC interrupt.
};
#define BRAKE_NOT_AVAILABLE 255

//
//	Set one of the following definitions to
//	select the motor shield installed.
//
//	SHIELD_DEFAULT_ARDUINO	The standard (or compatible) motor shield
//				available widely from Arduino or other
//				vendors.
//
//	SHIELD_GENERATOR_DRIVER	The bespoke shield built specifically to
//				support multiple (>2) DCC Districts with
//				pin register optimised specification.
//
//				The table defined below for this configuration
//				has been laid out for the DCC Driver Backplane
//				and three DCC Driver Modules giving a
//				total of 6 DCC Districts (or 5 + 1 programming
//				track).
//
#define SHIELD_DEFAULT_ARDUINO
//#define SHIELD_GENERATOR_DRIVER

//
//	Arduino Motor Shield
//	--------------------
//
#ifdef SHIELD_DEFAULT_ARDUINO

//
//	The following definitions are based on the hardware
//	characteristics of the "Deek-Robot" Motor Shield, which
//	should be compatible with the Arduino version (and others).
//
#define SHIELD_OUTPUT_DRIVERS	2
//
//	A and B drivers available, with 4 pins allocated to each.
//
//	These are the pin numbers "as per the motor shield".
//
//	IMPORTANT:
//
//		You *must* cut the VIN CONNECT traces on the
//		back of the Motor Shield.  You will need to
//		power the shield separately from the Arduino
//		with 15 volts DC, and leaving the VIN CONNECT
//		in place will put 15 volts across the Arduino
//		and probably damage it.
//
//		Previous versions of this firmware (and the DCC++
//		firmware) also required that the BRAKE feature
//		(for both A and B H-Bridges) should be cut too.
//		This is no longer required as this firmware
//		explicitly sets these LOW and are not touched
//		after that.
//
//		Also (unlike the DCC++ firmware) this firmware
//		requires no additional jumpers to support its
//		intended operation.
//
#define SHIELD_DRIVER_A_DIRECTION	12
#define SHIELD_DRIVER_A_ENABLE		3
#define SHIELD_DRIVER_A_BRAKE		9
#define SHIELD_DRIVER_A_LOAD		A0
#define SHIELD_DRIVER_A_ANALOGUE	0

#define SHIELD_DRIVER_B_DIRECTION	13
#define SHIELD_DRIVER_B_ENABLE		11
#define SHIELD_DRIVER_B_BRAKE		8
#define SHIELD_DRIVER_B_LOAD		A1
#define SHIELD_DRIVER_B_ANALOGUE	1

//
//	Define the motor shield attached to the Arduino for
//	the firmware.
//
static const SHIELD_DRIVER shield_output[ SHIELD_OUTPUT_DRIVERS ] PROGMEM = {
	{
		//
		//	Main track (main district)
		//
		true,
		SHIELD_DRIVER_A_DIRECTION,
		SHIELD_DRIVER_A_ENABLE,
		SHIELD_DRIVER_A_BRAKE,
		SHIELD_DRIVER_A_LOAD, SHIELD_DRIVER_A_ANALOGUE
	},
	{
		//
		//	Programming track / Last Main Track
		//
		SELECT_PROG( false, true ),
		SHIELD_DRIVER_B_DIRECTION,
		SHIELD_DRIVER_B_ENABLE,
		SHIELD_DRIVER_B_BRAKE,
		SHIELD_DRIVER_B_LOAD, SHIELD_DRIVER_B_ANALOGUE
	}
};

#else

//
//	Arduino Generator Shield
//	------------------------
//
#ifdef SHIELD_GENERATOR_DRIVER

//
//	The bespoke DCC Driver Backplane and DCC Driver Module
//	solution.
//
#define SHIELD_OUTPUT_DRIVERS		6
#define SHIELD_PORT_DIRECT		PORTB
#define SHIELD_PORT_DIRECT_DIR		DDRB
//
#define SHIELD_DRIVER_1_DIRECTION	bit(0)
#define SHIELD_DRIVER_1_ENABLE		2
#define SHIELD_DRIVER_1_LOAD		A0
#define SHIELD_DRIVER_1_ANALOGUE	0
//
#define SHIELD_DRIVER_2_DIRECTION	bit(1)
#define SHIELD_DRIVER_2_ENABLE		3
#define SHIELD_DRIVER_2_LOAD		A1
#define SHIELD_DRIVER_2_ANALOGUE	1
//
#define SHIELD_DRIVER_3_DIRECTION	bit(2)
#define SHIELD_DRIVER_3_ENABLE		4
#define SHIELD_DRIVER_3_LOAD		A2
#define SHIELD_DRIVER_3_ANALOGUE	2
//
#define SHIELD_DRIVER_4_DIRECTION	bit(3)
#define SHIELD_DRIVER_4_ENABLE		5
#define SHIELD_DRIVER_4_LOAD		A3
#define SHIELD_DRIVER_4_ANALOGUE	3
//
#define SHIELD_DRIVER_5_DIRECTION	bit(4)
#define SHIELD_DRIVER_5_ENABLE		6
#define SHIELD_DRIVER_5_LOAD		A6
#define SHIELD_DRIVER_5_ANALOGUE	6
//
#define SHIELD_DRIVER_6_DIRECTION	bit(5)
#define SHIELD_DRIVER_6_ENABLE		7
#define SHIELD_DRIVER_6_LOAD		A7
#define SHIELD_DRIVER_6_ANALOGUE	7
//
static const SHIELD_DRIVER shield_output[ SHIELD_OUTPUT_DRIVERS ] PROGMEM = {
	{
		//
		//	Main track district 1
		//
		true,
		SHIELD_DRIVER_1_DIRECTION,
		SHIELD_DRIVER_1_ENABLE,
		BRAKE_NOT_AVAILABLE,
		SHIELD_DRIVER_1_LOAD, SHIELD_DRIVER_1_ANALOGUE
	},
	{
		//
		//	Main track district 2
		//
		true,
		SHIELD_DRIVER_2_DIRECTION,
		SHIELD_DRIVER_2_ENABLE,
		BRAKE_NOT_AVAILABLE,
		SHIELD_DRIVER_2_LOAD, SHIELD_DRIVER_2_ANALOGUE
	},
	{
		//
		//	Main track district 3
		//
		true,
		SHIELD_DRIVER_3_DIRECTION,
		SHIELD_DRIVER_3_ENABLE,
		BRAKE_NOT_AVAILABLE,
		SHIELD_DRIVER_3_LOAD, SHIELD_DRIVER_3_ANALOGUE
	},
	{
		//
		//	Main track district 4
		//
		true,
		SHIELD_DRIVER_4_DIRECTION,
		SHIELD_DRIVER_4_ENABLE,
		BRAKE_NOT_AVAILABLE,
		SHIELD_DRIVER_4_LOAD, SHIELD_DRIVER_4_ANALOGUE
	},
	{
		//
		//	Main track district 5
		//
		true,
		SHIELD_DRIVER_5_DIRECTION,
		SHIELD_DRIVER_5_ENABLE,
		BRAKE_NOT_AVAILABLE,
		SHIELD_DRIVER_5_LOAD, SHIELD_DRIVER_5_ANALOGUE
	},
	{
		//
		//	Programming track / Last Main Track
		//
		SELECT_PROG( false, true ),
		SHIELD_DRIVER_6_DIRECTION,
		SHIELD_DRIVER_6_ENABLE,
		BRAKE_NOT_AVAILABLE,
		SHIELD_DRIVER_6_LOAD, SHIELD_DRIVER_6_ANALOGUE
	}
};

#else

//
//	No motor shield board selected.
//
#error "No motor shield board type selected."

#endif
#endif

//
//	Timing, Protocol and Data definitions.
//	======================================
//

//
//	The following paragraphs and numerical calculations are
//	based on a spread sheet used to analyse the possible
//	subdivisions of the DCC signal timing (spreadsheet not
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
//	These macros define the number of Interrupt Cycles that the
//	interrupt timer must count before raising an interrupt
//	to create a target interval of 14.5 us.
//
//	This period we are calling a "tick", multiples of which
//	are used to build up the intervals required to create
//	the DCC signal.
//
//	Simplistically the "MHz" clock rate of the board multiplied
//	by the selected "tick" duration (in microseconds) gives the
//	base starting point.
//

#if F_CPU == 16000000
//
//	Arduino Uno (or equivalent)
//
//		16 (MHz) x 14.5 (microseconds) = 232 (clock cycles)
//
//	This fits inside the 8-bit interrupt timer limit (255), so
//	no interrupt pre-scaler is required (so equal to 1).
//
//	Within the DCC standard, the half time of a "1" bit is
//	58 us:
//		4 ticks of 14.5 us are exactly 58 us (0% error).
//
//	Like wise, the nominal half time of a "0" bit is 100 us,
//	so:
//		7 ticks of 14.5 us gives a time of 101.5 us (1.5% error).
//
#define TIMER_INTERRUPT_CYCLES	232
#define TIMER_CLOCK_PRESCALER	1

#else

#if F_CPU == 20000000
//
//	Arduino Mega (or equivalent)
//
//		20 (MHz) x 14.5 (microseconds) = 290 (clock cycles)
//
//	This is too big for the 8 bit timer we are using, so we select
//	the smallest available pre-scaler: 8
//
//		290 (clock cycles) / 8 = 36.25
//
//	Round down to 36 and multiply out to determine the resulting
//	tick interval:
//
//		36 * 8 / 20 = 14.4 us
//
//	This makes the time interval the Interrupt Service Routine
//	must complete in marginally shorter, but is easily offset with
//	the higher intrinsic clock rate.
//
//	Therefore the we get the "half time" of a "1" as:
//
//		4 ticks of 14.4 us is 57.6 us (0.6% error)
//
//	Like wise, the nominal "half time" of a "0" bit is 100 us,
//	so:
//		7 ticks of 14.4 us gives a time of 100.8 us (0.8% error)
//
#define TIMER_INTERRUPT_CYCLES	36
#define TIMER_CLOCK_PRESCALER	8

#else

//
//	The target MCU clock frequency has not been accounted for.
//
#error "MCU Clock speed calculation needs to be calculate for this clock rate."

//
//	The calculations outlined above need to be carried out and appropriate
//	results captured in the definitions of TIMER_INTERRUPT_CYCLES and
//	TIMER_CLOCK_PRESCALER values.
//

#endif
#endif

//
//	The following two macros return the number of interrupt
//	ticks which are required to generate half of a "1" or
//	half of a "0" bit in the DCC signal.
//
#define TICKS_FOR_ONE	4
#define TICKS_FOR_ZERO	7


//
//	LCD structure
//	-------------
//
#ifdef LCD_DISPLAY_ENABLE

//
//	Create the LCD interface object.
//
static LCD_TWI_IO lcd( LCD_DISPLAY_ADRS, LCD_DISPLAY_COLS, LCD_DISPLAY_ROWS );

//
//	Allocate a static frame buffer
//
#define LCD_BUFFER	(LCD_DISPLAY_COLS*LCD_DISPLAY_ROWS)
static byte lcd_buffer[ LCD_BUFFER ];

//
//	Outline description of the  LCD display
//	---------------------------------------
//
//	The dimensions and field sizes for the display are set here as they
//	have size implications for other data structures following.
//
//	The code will compile itself to match the dimensions of the display
//	(as set in LCD_DISPLAY_ROWS and _COLS).  This may not result in a
//	pleasing/balanced display in all cases.
//
//	The code has been optimised to target a 20 column by 4 row display.
//
//	A drawing of the target output display:
//
//	+--------------------+	The STATUS area of the display, showing:
//	|SSSSSS              |	The highest district power (L)oad
//	|SSSSSS              |	The available (F)ree bit buffers and (P)ower status
//	|SSSSSS              |	DCC packets (T)ransmitted sent per second
//	|SSSSSS              |	The (U)ptime in seconds
//	+--------------------+	Right hand column set to '|'
//
//	+--------------------+	The DISTRICT area of the display, showing:
//	|      DDDDDDD       |	Details of each districts status and power
//	|      DDDDDDD       |	output (the source data for the Load value
//	|      DDDDDDD       |	in the Status area).
//	|      DDDDDDD       |
//	+--------------------+	Right hand column set to '|'
//
//	+--------------------+	The BUFFER area of the display, showing:
//	|             BBBBBBB|	Buffers in use and the action in place.
//	|             BBBBBBB|
//	|             BBBBBBB|
//	|             BBBBBBB|
//	+--------------------+	
//
//	The following definitions define some parameters which
//	shape the output ot the LCD.  The fixed values here should
//	not be modified unless the appropriate code is adjusted
//	accordingly.
//
#define LCD_DISPLAY_STATUS_WIDTH	6
#define LCD_DISPLAY_STATUS_COLUMN	0
//
#define LCD_DISPLAY_BUFFER_WIDTH	7
#define LCD_DISPLAY_BUFFER_COLUMN	(LCD_DISPLAY_COLS-LCD_DISPLAY_BUFFER_WIDTH)
//
#define LCD_DISPLAY_DISTRICT_COLUMN	(LCD_DISPLAY_STATUS_COLUMN+LCD_DISPLAY_STATUS_WIDTH)
#define LCD_DISPLAY_DISTRICT_WIDTH	(LCD_DISPLAY_BUFFER_COLUMN-LCD_DISPLAY_DISTRICT_COLUMN)

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
//	DCC packet.  The fields defined are:
//
//	target		The new value for target upon setting up a new
//			bit stream.
//
//	preamble	The number of '1's to send as the lead-in preamble
//			to a DCC command.
//
//	postamble	The number of '1's to send at the end of a DCC
//			command.  This is normally 1, but for programming
//			command this can be used to create a pause in
//			command transmission (required to 'see' any
//			decoder reply).
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
	byte		preamble,
			postamble,
			duration,
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
static bool create_pending_rec( PENDING_PACKET ***adrs, int target, byte duration, byte preamble, byte postamble, byte len, byte *cmd ) {
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
	ptr->preamble = preamble;
	ptr->postamble = postamble;
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
//	Define a number of cache records and bytes for bit storage in
//	each record.  We calculate FUNCTION_BIT_ARRAY based on the
//	MIN and MAX function numbers provided (the 7+ ensures correct
//	rounding in boundary cases).
//
//	We will base the function cache size on the maximum number of
//	DCC mobile decoders we can have active in parallel.
//
#define FUNCTION_CACHE_RECS	MOBILE_TRANS_BUFFERS
#define FUNCTION_BIT_ARRAY	((( 1 + MAX_FUNCTION_NUMBER - MIN_FUNCTION_NUMBER )+7 ) >> 3 )

//
//	The structure used to cache function values per decoder so that
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
//	on a specified target number.
//
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
#define BIT_TRANSITIONS		SELECT_SML( 36, 48, 64 )

//
//	Define maximum bit iterations per byte of the bit transition array.
//
//	It is possible to prove that this figure can never be reached as this
//	would imply a a series of 28 byte containing just 0 bits which (apart
//	from being an invalid DCC command, is over 4 times longer than the
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
//	locked out sections of code (using noInterrupts() and interrupts()) which
//	might impact the over all timing of the firmware.
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
//			an implication that if duration is 0 then this must
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
	//		target < 0	Accessory Decoder (negate to get ID)
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
	//	is applied only at the end of a series of pending records (i.e. when
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

#ifdef LCD_DISPLAY_ENABLE

static int	lcd_statistic_packets;

#endif


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
//	by the LCD update routine.  Returns true if there was
//	an issue with the conversion (and remedial action needs
//	to be done) or false if everything worked as planned.
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
		if( v ||( n && ( len < 1 ))) return( true );
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
	return( false );
}

#endif

//
//	Define routines to assist with debugging that output
//	a range of value types.  Not compiled in unless one of the
//	debugging macros has been enabled.
//
#if defined( DEBUG_BUFFER_MANAGER )||defined( DEBUG_BIT_SLICER )||defined( DEBUG_POWER_MONITOR )

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
	return( console.print( t ));
}

#endif


//
//	Current monitoring code.
//	========================
//
//	With thanks to Nick Gammon and his post in the thread which can be
//	found at "http://www.gammon.com.au/forum/?id=11488&reply=5#reply5"
//	for those crucial first few steps that enabled this asynchronous ADC
//	implementation.
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
//	==================================================
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
//	Define a variable to control the direction output
//
#ifdef SHIELD_PORT_DIRECT

//
//	We are controlling the output signal using direct port
//	and bitmap control.  We define a pair of variables which
//	each hold one side of the output pattern so that
//	the Interrupt Service Routine has a simple task flipping
//	all direction pins on/off as quickly as possible.
//
//	output_mask_on		bit mask for "in phase" pins
//
//	output_mask_off		bit mask for "anti-phase" pins
//
static volatile byte	output_mask_on,
			output_mask_off;

#else

//
//	If we are not using direct port access then we define
//	a short array (with associated length) of the pin numbers
//	which need to be flipped.
//
//	output_pin		The individual pin numbers
//
//	output_phase		"in phase" or "anti-phase"
//
//	output_pins		Number of pins being controlled
//
static byte		output_pin[ SHIELD_OUTPUT_DRIVERS ];
static bool		output_phase[ SHIELD_OUTPUT_DRIVERS ];
static byte		output_pins;

#endif

//
//	"side" flips between true and false and lets the routine know
//	which "side" of the signal was being generated.
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
//	filler of a single "1" which is required while working
//	with decoders in service mode.
//
static byte dcc_filler_data[] = {
	1,			// 1s
	0
};

//
//	The Interrupt Service Routine which generates the DCC signal.
//
ISR( HW_TIMERn_COMPA_vect ) {
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
		//	time consistent position to do so.
		//

#ifdef SHIELD_PORT_DIRECT
		//
		//	Code supporting the DCC Generator Driver hardware accessed
		//	via a single port variable:
		//
		//	We flip all pins in the port according to the value of 'side'.
		//
		//	The in/out phase (as a result of auto-phase adjustment) are all
		//	handled by modifying the values found in output_mask_on and
		//	output_mask_off.
		//
		SHIELD_PORT_DIRECT = side? output_mask_on: output_mask_off;
#else
		//
		//	Code supporting the Arduino Motor Shield hardware where
		//	all pins need to individually flipped.
		//
		//	The data necessary to do the task is gathered into the two
		//	arrays output_pin[] (containing the actual pin numbers to
		//	change) and output_phase[] which details if the pin is in/out of
		//	phase.  output_pins gives the number of pins which are captured
		//	in these arrays.
		//
		{
			//
			//	We run through the array as fast as possible.
			//
			register byte *op, oc;
			register bool *ob;

			op = output_pin;
			oc = output_pins;
			ob = output_phase;
			//
			//	Replicated code is used to remove unnecessary computation
			//	from inside the loop to maximise speed through the pin
			//	adjustments.
			//
			//	Question: Can we remove the "?:" code and use the boolean
			//	value in the output_phase array directly?  Maybe, but not yet.
			//
			//	Use side to select broad logic choice..
			//
			if( side ) {
				while( oc-- ) digitalWrite( *op++, ( *ob++? HIGH: LOW ));
			}
			else {
				while( oc-- ) digitalWrite( *op++, ( *ob++? LOW: HIGH ));
			}
		}
#endif

		//
		//	Now undertake the logical flip and subsequent actions.
		//
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

#ifdef LCD_DISPLAY_ENABLE
					//
					//	Count a successful packet transmission for LCD display
					//
					lcd_statistic_packets++;
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
							//	the state of buffer to LOAD so the manager
							//	can deal with it.
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
	//	In ALL cases this routine needs to complete in less than TIMER_INTERRUPT_CYCLES
	//	(currently 232 for a 16 MHz machine).  This is (huge guestimation) approximately
	//	100 actual instructions (assuming most instructions take 1 cycle with some taking
	//	2 or 3).
	//
	//	The above code, on the "longest path" through the code (when moving
	//	between transmission buffers) I am estimating that this uses no more
	//	than 50 to 75% of this window.
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
static bool pack_command( byte *cmd, byte clen, byte preamble, byte postamble, byte *buf ) {
	byte	l, b, c, v, s;

	ASSERT( preamble >= DCC_SHORT_PREAMBLE );
	ASSERT( postamble >= 1 );

#ifdef DEBUG_BIT_SLICER
	console.print( "PACK:" );
	for( l = 0; l < clen; queue_byte( cmd[ l++ ]));
	console.print( ":" );
	queue_byte( long_preamble? DCC_LONG_PREAMBLE: DCC_SHORT_PREAMBLE );
#endif

	//
	//	Start with a preamble of "1"s.
	//
	*buf++ = preamble;
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
				if( c == MAXIMUM_BIT_ITERATIONS ) return( false );
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
		//	bit "1" (clen will be 0 on the last byte).
		//	Remember we use the top bit in b to indicate
		//	which bit we are currently counting.
		//
		if(( clen? 0: 0x80 ) == b ) {
			//
			//	On the right bit (which ever bit that
			//	is) so we simply need to add to the
			//	current bit counter.
			//
			if( c == MAXIMUM_BIT_ITERATIONS ) return( false );
			c++;
		}
		else {
			//
			//	Need the other bit so save the count so
			//	far then flip to the other bit.
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

	ASSERT( b == 0x80 );
	
	if(!( --l )) return( false );

	//
	//	Here we have the post amble to append.  Rather than
	//	abort sending the command if adding the postamble
	//	exceeds the value of MAXIMUM_BIT_ITERATIONS, we will
	//	simply trim the value down to MAXIMUM_BIT_ITERATIONS.
	//
	if(( b = ( MAXIMUM_BIT_ITERATIONS - c )) < postamble ) postamble = b;
	c += postamble;
	
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
	console.print( "\n" );
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
//	tight on the smaller MCUs).
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
	buf = _reply_in( buf, code );
	buf = int_to_text( buf, a1 );
	buf = _reply_out( buf );
	*buf = EOS;
}

static void reply_2( char *buf, char code, int a1, int a2 ) {
	buf = _reply_in( buf, code );
	buf = int_to_text( buf, a1 );
	buf = _reply_char( buf, SPACE );
	buf = int_to_text( buf, a2 );
	buf = _reply_out( buf );
	*buf = EOS;
}

#ifndef DCC_PLUS_PLUS_COMPATIBILITY
#ifdef PROGRAMMING_TRACK

static void reply_2c( char *buf, char code, int a1, int a2 ) {
	buf = _reply_in( buf, code );
	buf = int_to_text( buf, a1 );
	buf = _reply_char( buf, SPACE );
	buf = int_to_text( buf, a2 );
	buf = _reply_char( buf, SPACE );
	buf = _reply_char( buf, HASH );
	buf = _reply_out( buf );
	*buf = EOS;
}

static void reply_3c( char *buf, char code, int a1, int a2, int a3 ) {
	buf = _reply_in( buf, code );
	buf = int_to_text( buf, a1 );
	buf = _reply_char( buf, SPACE );
	buf = int_to_text( buf, a2 );
	buf = _reply_char( buf, SPACE );
	buf = int_to_text( buf, a3 );
	buf = _reply_char( buf, SPACE );
	buf = _reply_char( buf, HASH );
	buf = _reply_out( buf );
	*buf = EOS;
}

#endif

static void reply_n( char *buf, char code, int n, int *a ) {
	buf = _reply_in( buf, code );
	while( n-- ) {
		buf = int_to_text( buf, *a++ );
		if( n ) buf = _reply_char( buf, SPACE );
	}
	buf = _reply_out( buf );
	*buf = EOS;
}

#endif

static void reply_3( char *buf, char code, int a1, int a2, int a3 ) {
	buf = _reply_in( buf, code );
	buf = int_to_text( buf, a1 );
	buf = _reply_char( buf, SPACE );
	buf = int_to_text( buf, a2 );
	buf = _reply_char( buf, SPACE );
	buf = int_to_text( buf, a3 );
	buf = _reply_out( buf );
	*buf = EOS;
}

//
//	Error Reporting Code.
//	---------------------
//
//	Define a mechanism for the firmware to collate reports
//	of detected errors and forward them back to the host
//	computers as a suitable opportunity arises.
//


//
//	Define size of the buffer where the returned error messages
//	are constructed.
//
#define ERROR_OUTPUT_BUFFER 32

//
//	Flush the errors routine.
//
static void flush_error_queue( void ) {
	word	error, arg;
	byte	repeats;
	
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
	if( errors.peek_error( &error, &arg, &repeats )) {

		char		buffer[ ERROR_OUTPUT_BUFFER ];
	
		//
		//	Build error report, and send it only if there
		//	is enough space.
		//
		reply_2( buffer, SELECT_COMPAT( 'E', '#' ), error, arg );
		//
		//	Can we send this?
		//
		if( console.space() >= (int)strlen( buffer )) {
			//
			//	Send as space is available.
			//
			errors.drop_error();
			(void)console.print( buffer );
		}
	}
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
//	Define the size of the "spike average" we will use to
//	identify a critical rise in power consumption signifying
//	a short circuit.  This is an index into the
//	compounded average table.
//
#define SPIKE_AVERAGE_VALUE	1

//
//	Define the size of the "short average" we will use to
//	identify a genuine rise in power consumption signifying
//	a confirmation return.  This is an index into the
//	compounded average table.
//
#define SHORT_AVERAGE_VALUE	2

//
//	Define a structure used to track one of the drivers
//	that the firmware is managing.  This contains the state
//	of that driver and the compounded averaging system.
//
//	compound_value:	The array of compound average values for the
//			specified driver.
//
//	status:		Enumeration representing the current status of
//			the driver.  The status values are:
//
//			DRIVER_ON_GRACE	Output is ON, but operating inside
//					the "power on grace period" where
//					overloading is ignored before being
//					moved to the DRIVER_ON state.
//			DRIVER_ON	Output is good, nothing to do.
//			DRIVER_FLIPPED	Output phase has been swapped,
//					recheck has been set.  If nothing
//					changes by recheck time then move
//					driver back to DRIVER_ON_GRACE.
//			DRIVER_BLOCKED	Output phase *wants* to be changed,
//					but another district has locked
//					exclusive access to do so.
//			DRIVER_OFF	Output is disabled and will be
//					retried at recheck time.
//			DRIVER_DISABLED	This driver is not currently being
//					used for the creation of a DCC
//					signal.
//
//	recheck:	If non-zero then this driver has been modified
//			in response to a power condition.  This is the
//			"future time" at which this needs to be reviewed.
//
#define DRIVER_STATUS enum driver_status
DRIVER_STATUS {
	DRIVER_ON,
	DRIVER_ON_GRACE,
	DRIVER_FLIPPED,
	DRIVER_BLOCKED,
	DRIVER_OFF,
	DRIVER_DISABLED
};
#define DRIVER_LOAD struct driver_load
DRIVER_LOAD {
	word		compound_value[ COMPOUNDED_VALUES ];
	DRIVER_STATUS	status;
	unsigned long	recheck;
	
#ifdef DEBUG_POWER_MONITOR
	int		old_amps;
#endif
};

//
//	Define the array of structures used to track each of the
//	drivers loads.
//
static DRIVER_LOAD	output_load[ SHIELD_OUTPUT_DRIVERS ];

//
//	This is the phase flipping code lock flag.  Normally
//	NULL, set to the address of a load record when a district
//	is trying an phase inversion to resolve a short situation.
//
static DRIVER_LOAD	*flip_lock = NULL;

//
//	Keep an index into the output_load array so that each of
//	the drivers can have its load assessed in sequence.
//
static byte		output_index;

//
//	Finally, these flags set if the power code into "check
//	fora confirmation signal" mode in the power consumption,
//	and records if one has been seen.
//
static bool		confirmation_enabled,
			load_confirmed;

//
//	We keep a copy of the last track power reported to
//	save recalculating it multiple times.
//
static int		last_highest_power;

//
//	This routine is called during setup() to ensure the
//	driver load code is initialised and the first ADC
//	is initialised.
//
static void init_driver_load( void ) {
	//
	//	Ensure all monitoring data is empty
	//
	output_index = 0;
	for( byte d = 0; d < SHIELD_OUTPUT_DRIVERS; d++ ) {
		for( byte c = 0; c < COMPOUNDED_VALUES; c++ ) {
			output_load[ d ].compound_value[ c ] = 0;
		}
		output_load[ d ].status = DRIVER_DISABLED;
		output_load[ d ].recheck = 0;
#ifdef DEBUG_POWER_MONITOR
		output_load[ d ].old_amps = 0;
#endif
	}

	//
	//	Start checking current on the first driver.
	//
	MONITOR_ANALOGUE_PIN( pgm_read_byte( &( shield_output[ output_index ].analogue )));
}

//
//	In native mode we update the host computer with the district status
//	as changes occur.  This is the routine used to do that.
//
static void report_driver_status( void ) {
#ifndef DCC_PLUS_PLUS_COMPATIBILITY
	//
	//	Actually only do this in Native mode.
	//
	char	buffer[ 4 + SHIELD_OUTPUT_DRIVERS * 2 ];
	int	v[ SHIELD_OUTPUT_DRIVERS ];

	for( byte i = 0; i < SHIELD_OUTPUT_DRIVERS; i++ ) {
		switch( output_load[ i ].status ) {
			case DRIVER_ON:
			case DRIVER_ON_GRACE: {
				v[ i ] = 1;
				break;
			}
			case DRIVER_FLIPPED: {
				v[ i ] = 2;
				break;
			}
			case DRIVER_BLOCKED: {
				v[ i ] = 3;
				break;
			}
			case DRIVER_OFF: {
				v[ i ] = 4;
				break;
			}
			default: {
				v[ i ] = 0;
				break;
			}
		}
	}
	reply_n( buffer, 'D', SHIELD_OUTPUT_DRIVERS, v );
	if( !console.print( buffer )) errors.log_error( COMMAND_REPORT_FAIL, 'D' );

#endif
}

//
//	This routine is called every time track electrical load data
//	becomes available.  The routine serves several purposes:
//
//		Monitor for overload and spike conditions
//
//		Manage the phase flipping protocol
//
//		Detect return signals from devices attached
//		to the DCC bus.
//
//	How this code operates is dependent on the status of driver, outlined
//	in the following explanation:
//
//	o	Each driver output is managed in one of four states:
//
//		ON		District supplying power, no issues
//		OFF		District enabled, but temporarily suspended
//		FLIPPED		System is trying phase flip approach to
//				address an issue
//		BLOCKED		System *wants* to flip this district, but another
//				district has this code locked out.
//		DISABLED	District is not scheduled to supply power.
//
//	o	Ignoring DISABLED, the following state change table indicates
//		an individual district should progress through the states
//		each time it has it power/load measured.  The intersection in
//		the table gives the new state and accompanying actions.
//
//						Load Condition
//				Retry		--------------
//		In State	Time	Normal		Overloaded	Spike
//		--------	-----	------		----------	-----
//
//		ON		n/a	ON		OFF+		FLIPPED+
//							Schedule	invert
//							retry		phasing
//
//		FLIPPED		OFF	ON		OFF+		OFF+
//							Schedule	Schedule
//							retry		retry
//
//		BLOCKED		OFF	ON		OFF+		FLIPPED+
//							Schedule	Schedule
//							retry		retry
//
//		OFF		ON	OFF		OFF		OFF
//				
//
//
static void monitor_current_load( int amps ) {
	DRIVER_LOAD	*dp;

	//
	//	Set shortcut pointer into the output_load[] array.
	//
	dp = &( output_load[ output_index ]);

	//
	//	Check status of the flip lock:
	//
	//	*	If flip_lock is the same as this record, then
	//		this record must be in FLIPPED state
	//
	//	*	If the flip_lock is different to the current
	//		record (and this includes being NULL) then
	//		this record must NOT be in FLIPPED state.
	//
	//	The net result of this test is that there should be
	//	only ONE record in FLIPPED state when flip_lock is
	//	not NULL, and that record must be the one flip_lock
	//	points to.
	//
	ASSERT((( flip_lock == dp )&&( dp->status == DRIVER_FLIPPED ))||(( flip_lock != dp )&&( dp->status != DRIVER_FLIPPED )));

	//
	//	If overload detection is enabled we do our thing.
	//
	if( dp->status != DRIVER_ON_GRACE ) {
		if( confirmation_enabled ) {
			//
			//	Confirmation is being tested, so we
			//	modify the average as little as
			//	possible to keep base line values
			//	stable.
			//
			for( byte i = 0; i <= SHORT_AVERAGE_VALUE; i++ ) {
				amps = dp->compound_value[ i ] = ( amps + dp->compound_value[ i ]) >> 1;
			}
			
		}
		else {
			//
			//	Compound the new figure into the averages.
			//
			for( byte i = 0; i < COMPOUNDED_VALUES; i++ ) {
				amps = dp->compound_value[ i ] = ( amps + dp->compound_value[ i ]) >> 1;
			}
		}

#ifdef DEBUG_POWER_MONITOR
		//
		//	Display to Serial power data if the debug
		//	flag DEBUG_POWER_MONITOR is set.
		//
		if( amps != dp->old_amps ) {
			dp->old_amps = amps;

			console.print( "AMPS " );
			console.print( output_index );
			console.print( "=" );
			console.print( amps );
			for( byte i = 0; i < COMPOUNDED_VALUES; i++ ) {
				console.print( "_" );
				console.print( dp->compound_value[ i ]);
			}
			console.println( ";" );
		}
#endif

		//
		//	The code in the remainder of this routine needs to cover
		//	all of the cases outlined in the table described above.
		//
		//	Firstly, an instantaneous short circuit?
		//
		//	This covers off all of the rows under the column "Spike".
		// 
		if( dp->compound_value[ SPIKE_AVERAGE_VALUE ] >  INSTANT_CURRENT_LIMIT ) {
			//
			//	How we handle this is dependent on what
			//	has happened before.
			//
			switch( dp->status ) {
				case DRIVER_ON: {
					//
					//	This is the first time this district has seen
					//	a power spike.  What we do here is dependent
					//	on the state of the flip lock.
					//
					//	If the flip lock is taken, we need to start a
					//	controlled period of waiting before we simply
					//	close down the district.
					//
					//	If it is clear then we can head into the trying
					//	to invert the phase of this output before doing
					//	anything more drastic (like closing down the
					//	district).
					//
					if( flip_lock ) {
						//
						//	Some other district is already "flipping"
						//
						//	Put this district into BLOCKED state and
						//	set a short timeout before we set the
						//	district to OFF.
						//
						dp->status = DRIVER_BLOCKED;
						dp->recheck = now + DRIVER_PHASE_PERIOD;
					}
					else {
						//
						//	Nobody has the flip lock, so this district
						//	can initiate the phase flipping logic.
						//
					
#ifdef SHIELD_PORT_DIRECT
						byte	mask;

						//
						//	For a direct port shield we simply "flip" the corresponding
						//	bits in the output_mask_on and output_mask_off bit masks, once
						//	we have worked out which bit to flip.
						//
						mask = pgm_read_byte( &( shield_output[ output_index ].direction ));
						//
						//	.. and flip.
						//
						output_mask_on ^= mask;
						output_mask_off ^= mask;
#else
						//
						//	For an Arduino motor shield solution we simply invert the
						//	corresponding output_phase[] value.
						//
						output_phase[ output_index ] ^= true;
#endif
						//
						//	Lock the flip code and note change of state.
						//
						flip_lock = dp;
						dp->status = DRIVER_FLIPPED;
						dp->recheck = now + DRIVER_PHASE_PERIOD;
					}
					//
					//	Now, log an error to give the reason for the
					//	change in status.
					//
					errors.log_error( POWER_SPIKE, output_index );
					break;
				}
				case DRIVER_FLIPPED: {
					//
					//	If we get here then the power condition has persisted, despite
					//	this district having had the phase flipped.
					//
					//	What we do depends on the time.
					//
					//	If the DRIVER_PHASE_PERIOD has elapsed since this district went
					//	into FLIPPED state, then we must shutdown this district.
					//
					if( now > dp->recheck ) {
						//
						//	This has failed long enough.  Turn it OFF.
						//
						digitalWrite( pgm_read_byte( &( shield_output[ output_index ].enable )), LOW );
						//
						//	We flatten the power averaging data to simplify
						//	power up restarting and schedule the restart.
						//
						for( byte i = 0; i < COMPOUNDED_VALUES; dp->compound_value[ i++ ] = 0 );
						//
						//	Clear the flip lock.
						//
						ASSERT( flip_lock == dp );
						flip_lock = NULL;
						//
						//	Note changes in status and schedule re checking status.
						//
						dp->status = DRIVER_OFF;
						dp->recheck = now + DRIVER_RESET_PERIOD;
						//
						//	Report new driver state
						//
						report_driver_status();
					}
					break;
				}
				case DRIVER_BLOCKED: {
					//
					//	We have got here because the power condition has persisted
					//	despite some district (other than this one) going through a
					//	process of flipping its phase.
					//
					//	First we test to see if the flip_lock has become available.
					//
					if( flip_lock ) {
						//
						//	Flip lock still held.  Is it time to stop waiting?
						//
						if( now > dp->recheck ) {
							//
							//	This has failed long enough so we turn it OFF.
							//
							digitalWrite( pgm_read_byte( &( shield_output[ output_index ].enable )), LOW );
							//
							//	We flatten the power averaging data to simplify
							//	power up restarting and schedule the restart.
							//
							for( byte i = 0; i < COMPOUNDED_VALUES; dp->compound_value[ i++ ] = 0 );
							//
							//	Note changes in status and schedule re checking status.
							//
							dp->status = DRIVER_OFF;
							dp->recheck = now + DRIVER_RESET_PERIOD;
							//
							//	Report change in status
							//
							report_driver_status();
						}
					}
					else {
						//
						//	We can phase flip because the flip lock has become free.
						//
#ifdef SHIELD_PORT_DIRECT
						byte	mask;

						//
						//	For a direct port shield we simply "flip" the corresponding
						//	bits in the output_mask_on and output_mask_off bit masks, once
						//	we have worked out which bit to flip.
						//
						mask = pgm_read_byte( &( shield_output[ output_index ].direction ));
						output_mask_on ^= mask;
						output_mask_off ^= mask;
#else
						//
						//	For an Arduino motor shield solution we simply invert the
						//	corresponding output_phase[] value.
						//
						output_phase[ output_index ] ^= true;
#endif
						//
						//	Lock the flip code and note change of state but
						//	do not reset the recheck time.  Any time lost
						//	in BLOCKED state is lost.
						//
						flip_lock = dp;
						dp->status = DRIVER_FLIPPED;
						//
						//	Report new driver state
						//
						report_driver_status();
					}
					break;
				}
				default: {
					//
					//	Turn off the output, but do not
					//	change the state information.
					//
					digitalWrite( pgm_read_byte( &( shield_output[ output_index ].enable )), LOW );
					break;
				}
			}
		}
		else {
			//
			//	Secondly, basic overload?
			//
			//	This covers off all of the rows under the column "Overloaded".
			//
			if( dp->compound_value[ COMPOUNDED_VALUES - 1 ] >  AVERAGE_CURRENT_LIMIT ) {
				//
				//	Cut the power here because there is some sort of long
				//	term higher power drain.
				//
				switch( dp->status ) {
					case DRIVER_DISABLED:
					case DRIVER_OFF: {
						//
						//	Turn off the output, but do not
						//	change the state information or
						//	any rechecking time which might
						//	have been set.
						//
						digitalWrite( pgm_read_byte( &( shield_output[ output_index ].enable )), LOW );
						break;
					}
					default: {
						//
						//	For all other states, we turn off the
						//	output, set a re-test time and clear the
						//	flip lock if necessary.
						//
						digitalWrite( pgm_read_byte( &( shield_output[ output_index ].enable )), LOW );
						//
						//	We flatten the power averaging data to simplify
						//	power up restarting and schedule the restart.
						//
						for( byte i = 0; i < COMPOUNDED_VALUES; dp->compound_value[ i++ ] = 0 );
						//
						//	Flip lock needs clearing, if it pointed
						//	to this record.
						//
						if( flip_lock == dp ) flip_lock = NULL;
						//
						//	Note what we have done and schedule rechecking
						//
						dp->status = DRIVER_OFF;
						dp->recheck = now + DRIVER_RESET_PERIOD;
						//
						//	Report error/action to host computer
						//
						errors.log_error( POWER_OVERLOAD, amps );
						report_driver_status();
					}
				}
			}
			else {
				//
				//	Lastly, this is a case where the power load reported
				//	is "nominal"; in the normal operating range of values.
				//
				//	Are we seeing a "reply" from an on-track device?
				//
				//	Before we check out the different between the short term average
				//	and the long term average, we should check that it is actually
				//	larger than the long term average.
				//
				word	a, b;

				if(( a = dp->compound_value[ SHORT_AVERAGE_VALUE ]) > ( b = dp->compound_value[ COMPOUNDED_VALUES-1 ])) {
					if(( a - b ) > MINIMUM_DELTA_AMPS ) {
						//
						//	We believe that we have seen a confirmation.
						//
						load_confirmed = true;
					}
				}
				//
				//	Finally, power level are not of any concern, but still
				//	other things to do.
				//
				//	If the state is either FLIPPED or BLOCKED we can
				//	reset it to ON and clear any retry.  Do not forget
				//	to clear the flip lock if necessary.
				//
				switch( dp->status ) {
					case DRIVER_FLIPPED: {
						//
						//	Phase flipping must have worked!
						//

						ASSERT( flip_lock == dp );

						flip_lock = NULL;
						//
						//	We are deliberately going to fall through
						//	to the following case to complete the
						//	common actions.
						//
						__attribute__(( fallthrough ));
					}
					case DRIVER_BLOCKED: {
						//
						//	Another district flipping must have fixed this.
						//
						dp->status = DRIVER_ON_GRACE;
						dp->recheck = now + POWER_GRACE_PERIOD;

						//
						//	Let the world know the good news.
						//
						report_driver_status();
						break;
					}
					case DRIVER_OFF: {
						//
						//	We expect to get here every time with a district
						//	which is OFF.  All we do is check to see if we
						//	need to restart the district.
						//
						if( now > dp->recheck ) {
							//
							//	Restart the district.
							//
							//	Turn on output.
							//
							digitalWrite( pgm_read_byte( &( shield_output[ output_index ].enable )), HIGH );
							//
							//	reset status etc.
							//
							dp->status = DRIVER_ON_GRACE;
							dp->recheck = now + POWER_GRACE_PERIOD;
							//
							//	Let the world know the good news.
							//
							report_driver_status();
						}
						break;
					}
					default: {
						//
						//	Nothing to do here
						//
						break;
					}
				}
			}
		}
	}
	else {
		//
		//	We are in the Power On Grace period.
		//
		if( now > dp->recheck ) {
			dp->status = DRIVER_ON;
			dp->recheck = 0;
		}
	}
	
	//
	//	Move to the next driver..
	//
	if(( output_index += 1 ) >= SHIELD_OUTPUT_DRIVERS ) output_index = 0;

	//
	//	..and start a new reading on that.
	//
	MONITOR_ANALOGUE_PIN( pgm_read_byte( &( shield_output[ output_index ].analogue )));
}

//
//	Routine called by the periodic timer code to report
//	track power dynamically.
//
static void report_track_power( void ) {
	char	buffer[ 16 ];

	//
	//	Here we will simply find the highest load value and return that figure.
	//
	last_highest_power = 0;
	for( byte i = 0; i < SHIELD_OUTPUT_DRIVERS; i++ ) {
		int t = output_load[ i ].compound_value[ COMPOUNDED_VALUES-1 ];
		if( t > last_highest_power ) last_highest_power = t;
	}
	reply_1( buffer, SELECT_COMPAT( 'L', 'a' ), last_highest_power );
	if( !console.print( buffer )) errors.log_error( COMMAND_REPORT_FAIL, SELECT_COMPAT( 'L', 'a' ));
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
	
#ifdef SHIELD_PORT_DIRECT
	{
		byte	new_mask;
		
		//
		//	Rebuild the output mask to reflect the new output
		//	pin mask.  We "buffer" the change to the output_mask
		//	value to ensure that it moves directly from its old value
		//	to zero then to its completed new value without containing
		//	any intermediate values.  This ensures that the ISR
		//	only ever sees valid and complete bit masks.
		//
		//	When we power on the track we assume *all* districts are
		//	in "forward" mode (all the same phase) so output_mask_on
		//	contains all "1"s and output_mask_off "0"s.  If a phase
		//	change condition is detected the distict impacted will move
		//	its "1" from _on to _off (or the otherway).
		//
		output_mask_on = output_mask_off = new_mask = 0;
		for( byte i = 0; i < SHIELD_OUTPUT_DRIVERS; i++ ) {
			if( pgm_read_byte( &( shield_output[ i ].main ))) {
				//
				//	Enable the specific operating track driver..
				//
				digitalWrite( pgm_read_byte( &( shield_output[ i ].enable )), HIGH );
				output_load[ i ].status = DRIVER_ON_GRACE;
				output_load[ i ].recheck = now + POWER_GRACE_PERIOD;
				//
				//	Add the pin to the mask
				//
				new_mask |= pgm_read_byte( &( shield_output[ i ].direction ));
			}
			else {
				//
				//	Disable the specific programming track driver.
				//
				digitalWrite( pgm_read_byte( &( shield_output[ i ].enable )), LOW );
				output_load[ i ].status = DRIVER_DISABLED;
			}
			//
			//	Clear load array.
			//
			for( byte j = 0; j < COMPOUNDED_VALUES; j++ ) {
				output_load[ i ].compound_value[ j ] = 0;
			}
			output_load[ i ].recheck = 0;
		}
		output_mask_on = new_mask;
	}
#else
	{
		//
		//	Set up the output pin array for the main track.
		//
		output_pins = 0;
		for( byte i = 0; i < SHIELD_OUTPUT_DRIVERS; i++ ) {
			if( pgm_read_byte( &( shield_output[ i ].main ))) {
				//
				//	Enable the specific operating track driver..
				//
				digitalWrite( pgm_read_byte( &( shield_output[ i ].enable )), HIGH );
				output_load[ i ].status = DRIVER_ON_GRACE;
				output_load[ i ].recheck = now + POWER_GRACE_PERIOD;
				//
				//	Add the direction pin to the output array..
				//
				output_pin[ output_pins ] = pgm_read_byte( &( shield_output[ i ].direction ));
				//
				//	Kick off in the "normal" phase alignment.
				//
				output_phase[ output_pins ] = true;
				//
				//	..and then (after updating the arrays) increase the pin count.
				//
				output_pins++;
			}
			else {
				//
				//	Disable the specific programming track driver.
				//
				digitalWrite( pgm_read_byte( &( shield_output[ i ].enable )), LOW );
				output_load[ i ].status = DRIVER_DISABLED;
			}
			//
			//	Clear load array.
			//
			for( byte j = 0; j < COMPOUNDED_VALUES; j++ ) {
				output_load[ i ].compound_value[ j ] = 0;
			}
			output_load[ i ].recheck = 0;
		}
	}
#endif

	report_driver_status();
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
#ifdef PROGRAMMING_TRACK

static byte power_on_prog_track( void ) {
	POWER_STATE	prev;

	prev = global_power_state;

#ifdef SHIELD_PORT_DIRECT
	{
		byte	new_mask;
		
		//
		//	Same code as the above.
		//
		output_mask_on = output_mask_off = new_mask = 0;
		for( byte i = 0; i < SHIELD_OUTPUT_DRIVERS; i++ ) {
			if( pgm_read_byte( &( shield_output[ i ].main ))) {
				//
				//	Disable the specific operating track driver.
				//
				digitalWrite( pgm_read_byte( &( shield_output[ i ].enable )), LOW );
				output_load[ i ].status = DRIVER_DISABLED;
			}
			else {
				//
				//	Enable the specific programming track driver..
				//
				digitalWrite( pgm_read_byte( &( shield_output[ i ].enable )), HIGH );
				output_load[ i ].status = DRIVER_ON_GRACE;
				output_load[ i ].recheck = now + POWER_GRACE_PERIOD;
				//
				//	Add the pin to the mask
				//
				new_mask |= pgm_read_byte( &( shield_output[ i ].direction ));
			}
			//
			//	Clear load array.
			//
			for( byte j = 0; j < COMPOUNDED_VALUES; j++ ) {
				output_load[ i ].compound_value[ j ] = 0;
			}
			output_load[ i ].recheck = 0;
		}
		output_mask_on = new_mask;
	}
#else
	{
		//
		//	Set up the output pin array for the main track.
		//
		output_pins = 0;
		for( byte i = 0; i < SHIELD_OUTPUT_DRIVERS; i++ ) {
			if( pgm_read_byte( &( shield_output[ i ].main ))) {
				//
				//	Disable the specific operating track driver.
				//
				digitalWrite( pgm_read_byte( &( shield_output[ i ].enable )), LOW );
				output_load[ i ].status = DRIVER_DISABLED;
			}
			else {
				//
				//	Enable the specific programming track driver..
				//
				digitalWrite( pgm_read_byte( &( shield_output[ i ].enable )), HIGH );
				output_load[ i ].status = DRIVER_ON_GRACE;
				output_load[ i ].recheck = now + POWER_GRACE_PERIOD;
				//
				//	Add the direction pin to the output array..
				//
				output_pin[ output_pins ] = pgm_read_byte( &( shield_output[ i ].direction ));
				//
				//	Kick off in the "normal" phase alignment.
				//
				output_phase[ output_pins ] = true;
				//
				//	..and then (after updating the arrays) increase the pin count.
				//
				output_pins++;
			}
			//
			//	Clear load array.
			//
			for( byte j = 0; j < COMPOUNDED_VALUES; j++ ) {
				output_load[ i ].compound_value[ j ] = 0;
			}
			output_load[ i ].recheck = 0;
		}
	}
#endif
	report_driver_status();
	global_power_state = GLOBAL_POWER_PROG;
	return( prev != GLOBAL_POWER_PROG );
}

#endif
#endif

//
//	Routine to power OFF tracks, return true
//	if this actually changed the state of the
//	power.
//
static byte power_off_tracks( void ) {
	POWER_STATE	prev;

	prev = global_power_state;
	
#ifdef SHIELD_PORT_DIRECT
	//
	//	Clear the output pin mask.
	//
	output_mask_on = 0;
	output_mask_off = 0;
#else
	//
	//	Mark the output pin array as empty.
	//
	output_pins = 0;
#endif

	for( byte i = 0; i < SHIELD_OUTPUT_DRIVERS; i++ ) {
		digitalWrite( pgm_read_byte( &( shield_output[ i ].enable )), LOW );
		for( byte j = 0; j < COMPOUNDED_VALUES; j++ ) {
			output_load[ i ].compound_value[ j ] = 0;
		}
		output_load[ i ].status = DRIVER_DISABLED;
		output_load[ i ].recheck = 0;
	}
	report_driver_status();
	global_power_state = GLOBAL_POWER_OFF;
	return( prev != GLOBAL_POWER_OFF);
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
			if( pack_command( pp->command, pp->len, pp->preamble, pp->postamble, manage->bits )) {
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
					if( !console.print( manage->contains )) {
						errors.log_error( COMMAND_REPORT_FAIL, manage->target );
					}
					manage->reply = NO_REPLY_REQUIRED;
				}

#ifdef DEBUG_BUFFER_MANAGER
				console.print( "LOAD:" );
				queue_int( manage->target );
				console.print( "\n" );
#endif

			}
			else {
				//
				//	Failed to complete as the bit translation failed.
				//
				errors.log_error( BIT_TRANS_OVERFLOW, manage->pending->target );
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
				console.print( "FAIL:" );
				queue_int( manage->target );
				console.print( "\n" );
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
					if( !console.print( manage->contains )) {
						errors.log_error( COMMAND_REPORT_FAIL, manage->target );
					}
				}
				else if( load_confirmed ) {
					//
					//	Only send confirmation if confirmation was received
					//
					if( !console.print( manage->contains )) {
						errors.log_error( COMMAND_REPORT_FAIL, manage->target );
					}
				}
				//
				//	Clear confirmation mode.
				//
				confirmation_enabled = false;

			}
			//
			//	Now mark empty.
			//
			manage->reply = NO_REPLY_REQUIRED;
			manage->state = TBS_EMPTY;

#ifdef DEBUG_BUFFER_MANAGER
			console.print( "EMPTY:" );
			queue_int( manage->target );
			console.print( "\n" );
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
		//	Not really necessary but ensures all buffers
		//	contain something displayable even if it has
		//	not been explicitly set.
		//
		memset( circular_buffer[ i ].display, USCORE, LCD_DISPLAY_BUFFER_WIDTH );
		circular_buffer[ i ].display[ 0 ] = '[';
		circular_buffer[ i ].display[ LCD_DISPLAY_BUFFER_WIDTH-1 ] = ']';	
#endif

	}
	//
	//	Link up *all* the buffers into a loop in numerical order.
	//
	//	If the programming track is not enabled, then this is the
	//	only time the circular buffer is formed, and must include
	//	all the buffers.
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
//	possibility the assignments need to be bracketed between noInterrupts()
//	and interrupts().
//
//	These routines are only called when one or other track is being power up.
//
//	These routines are *only* required when the firmware is required to support
//	a programming track in addition to the main operations track.
//
static void link_main_buffers( void ) {
	//
	//	This routine is called to shape the circular buffers
	//	to only contain the operating track buffers.
	//
#ifdef PROGRAMMING_TRACK
	noInterrupts();
	circular_buffer[ PROGRAMMING_BASE_BUFFER-1 ].next = circular_buffer;
	circular_buffer[ TRANSMISSION_BUFFERS-1 ].next = circular_buffer;
	interrupts();
#endif
}

#ifndef DCC_PLUS_PLUS_COMPATIBILITY

#ifdef PROGRAMMING_TRACK

static void link_prog_buffers( void ) {
	//
	//	This routine is called to shape the circular buffers
	//	to only contain the programming track buffer.
	//
	noInterrupts();
	circular_buffer[ PROGRAMMING_BASE_BUFFER-1 ].next = circular_buffer + PROGRAMMING_BASE_BUFFER;
	circular_buffer[ TRANSMISSION_BUFFERS-1 ].next = circular_buffer + PROGRAMMING_BASE_BUFFER;
	interrupts();
}

#endif
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
	//	We start all buffer circularly linked, however, this
	//	is not the "correct" linkage (depending on which track
	//	is being operated).  When operating the main track only
	//	the main track packets are linked.  When operating the
	//	(optional) programming track, only the programming track
	//	packet (there is only 1) is linked to itself.
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

	//
	//	Make sure the "pin out" data is empty as we are initially not
	//	driving current to any track.
	//
#ifdef SHIELD_PORT_DIRECT
	output_mask_on = 0;
	output_mask_off = 0;
#else
	output_pins = 0;
#endif

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
	//
	//	Initialise confirmation variables.
	//
	confirmation_enabled = false;
	load_confirmed = false;
}

void setup( void ) {
	//
	//	Initialise all the constant values
	//
	initialise_constants();
	
	//
	//	First things first: Set up the serial connection.  This
	//	takes place regardless of if we are encoding to use the
	//	serial port for application purposes.
	//
	console.initialise( 0, SERIAL_BAUD_RATE, CS8, PNone, SBOne, &console_in, &console_out );

	//
	//	Initialise our view of the current time.
	//
	now = millis();

	//
	//	Initialise all of the H Bridge Drivers from the
	//	motor shield configuration table.
	//
#ifdef SHIELD_PORT_DIRECT
	//
	//	If accessing the direction port directly, we can
	//	set it up in one go.
	//
	SHIELD_PORT_DIRECT_DIR = 0xff;
	SHIELD_PORT_DIRECT = 0;
#endif

	for( byte i = 0; i < SHIELD_OUTPUT_DRIVERS; i++ ) {
		byte	p;

#ifndef SHIELD_PORT_DIRECT
		//
		//	If *not* accessing the direction port directly,
		//	then we individually configure the direction
		//	pins.
		//
		p = pgm_read_byte( &( shield_output[ i ].direction ));
		pinMode( p, OUTPUT );
		digitalWrite( p, LOW );
#endif
		p = pgm_read_byte( &( shield_output[ i ].enable ));
		pinMode( p, OUTPUT );
		digitalWrite( p, LOW );
		p = pgm_read_byte( &( shield_output[ i ].brake ));
		if( p != BRAKE_NOT_AVAILABLE ) {
			pinMode( p, OUTPUT );
			digitalWrite( p, LOW );
		}
		p = pgm_read_byte( &( shield_output[ i ].load ));
		pinMode( p, INPUT );
		(void)analogRead( p );
	}
	//
	//	Set up the data structures.
	//
	initialise_data_structures();

#ifndef DCC_PLUS_PLUS_COMPATIBILITY

	init_function_cache();
	
#endif

	//
	//	Set up the Interrupt Service Routine
	//	------------------------------------
	//
	//	Disable interrupts.
	//
	noInterrupts();
	//
	//		Set Timer to default empty values.	
	//
	HW_TCCRnA = 0;	//	Set entire HW_TCCRnA register to 0
	HW_TCCRnB = 0;	//	Same for HW_TCCRnB
	HW_TCNTn  = 0;	//	Initialize counter value to 0
	//
	//		Set compare match register to
	//		generate the correct tick duration.
	//
	HW_OCRnA = TIMER_INTERRUPT_CYCLES;
	//
	//		Turn on CTC mode
	//
	HW_TCCRnA |= ( 1 << HW_WGMn1 );

#if TIMER_CLOCK_PRESCALER == 1

	//
	//		Set HW_CSn0 bit for no pre-scaler (factor == 1 )
	//
	HW_TCCRnB |= ( 1 << HW_CSn0 );
	
#else
#if TIMER_CLOCK_PRESCALER == 8

	//
	//		Set HW_CSn1 bit for pre-scaler factor 8
	//
	HW_TCCRnB |= ( 1 << HW_CSn1 );
#else

	//
	//	Pre-scaler value not supported.
	//
#error "Selected interrupt clock pre-scaler not supported"

#endif
#endif

	//
	//		Enable timer compare interrupt
	//
	HW_TIMSKn |= ( 1 << HW_OCIEnA );
  	//
	//	Enable interrupts.
	//
	interrupts();

	//
	//	Kick off the power monitor and management system.
	//
	init_driver_load();

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
	//
	//	First enable the TWI interface:
	//
	//	0	No Slave address
	//	false	Do not accept general calls (broadcasts)
	//	true	Enable the I2C/TWI interrupt handler
	//	true	Use internal pull up resistors
	//
	twi_init( 0, false, true, true );
	//
	//	Kick off the LCD display, now that the TWI
	//	interface is enabled, and add the frame
	//	buffer.
	//
	lcd.begin();
	lcd.setBuffer( lcd_buffer, LCD_BUFFER );
#endif

#ifdef SPLASH_ENABLE
	{
		static const char banner_line[] PROGMEM = {
			SPLASH_LINE_1 "\n" SPLASH_LINE_2 "\n" SPLASH_LINE_3 "\n" SPLASH_LINE_4 "\n"
		};
		byte	l;
		char	*s, c;

		//
		//	Display banner to the LCD for a short period.
		//
		l = 0;
		s = (char *)banner_line;
		
#ifdef LCD_DISPLAY_ENABLE
		lcd.clear();
#endif

		console.println();
		while(( c = pgm_read_byte( s++ )) != EOS ) {
			if( c == '\n' ) {
				
#ifdef LCD_DISPLAY_ENABLE
				lcd.setPosn( 0, ++l );
#endif

				console.println();
			}
			else {
				
#ifdef LCD_DISPLAY_ENABLE
				lcd.writeChar( c );
#endif

				console.print( c );
			}
		}
		console.println();
		
#ifdef LCD_DISPLAY_ENABLE
		lcd.synchronise( SPLASH_WAIT );
		lcd.clear();
#else
		delay( SPLASH_WAIT );
#endif
	}

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
//	Routine that draws a stylised bar-graph of a 16 bit value
//	in a supplied number of characters.
//
static void bar_graph( word val, word scale, char *buf, byte len ) {
	byte	div, a, b, c;

	div = scale / len;
	a = div >> 2;
	b = div >> 1;
	c = a + b;

	while( len-- ) {
		if( val > c ) {
			*buf++ = '#';
		}
		else {
			if( val > b ) {
				*buf++ = '=';
			}
			else {
				if( val < a ) {
					*buf++ = USCORE;
				}
				else {
					*buf++ = '-';
				}
			}
		}
		if( val >= div ) {
			val -= div;
		}
		else {
			val = 0;
		}
	}
}

//
//	Time of next call to display update (ms).
//
static unsigned long next_lcd_update = 0;

//
//	The next line of screen to be updated.
//
static byte next_lcd_line = 0;

//
//	LCD Update routine.
//
//	This is called frequently, and is the responcibility of this
//	routine to return immediately 99% of the time.
//
//	The update of the LCD has been spread over a number of calls
//	to reduce the "dead time" that the MCU spends in this routine
//	at any single time.
//
//	The variable "next_lcd_line" is stepped from 0 to LCD_DISPLAY_ROWS
//	(inclusive), on divisions of LINE_REFRESH_INTERVAL microseconds.
//
//	During steps 0 to LCD_DISPLAY_ROWS-1 the corresponding lines of
//	the STATUS and DISTRICT areas are updated.  On the final count of
//	"LCD_DISPLAY_ROWS" the BUFFER area is fully updated.
//
static void display_lcd_updates( void ) {

	//
	//	Is it time to update the LCD again?
	//
	if( now < next_lcd_update ) return;
	next_lcd_update += LINE_REFRESH_INTERVAL;

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
	//	Now complete each of the rows in LCD_DISPLAY_STATUS_WIDTH-1 characters.
	//
	switch( next_lcd_line ) {
		case 0:	{
			char		buffer[ LCD_DISPLAY_STATUS_WIDTH ];

			//
			//	Row 0, always available, Power Load Average
			//
			buffer[ 0 ] = 'L';
			if( backfill_int_to_text( buffer+1, last_highest_power, LCD_DISPLAY_STATUS_WIDTH-2 )) {
				memset( buffer+1, HASH, LCD_DISPLAY_STATUS_WIDTH-2 );
			}
			buffer[ LCD_DISPLAY_STATUS_WIDTH-1 ] = SELECT_COMPAT( '|', ':' );
			lcd.setPosn( LCD_DISPLAY_STATUS_COLUMN, 0 );
			lcd.writeBuf( buffer, LCD_DISPLAY_STATUS_WIDTH );
			break;
		}

#if LCD_DISPLAY_ROWS > 1
		case 1: {
			char		buffer[ LCD_DISPLAY_STATUS_WIDTH ];
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
			if( backfill_int_to_text( buffer+3, c, LCD_DISPLAY_STATUS_WIDTH-4 )) {
				memset( buffer+3, HASH, LCD_DISPLAY_STATUS_WIDTH-4 );
			}
			buffer[ LCD_DISPLAY_STATUS_WIDTH-1 ] = SELECT_COMPAT( '|', ':' );
			lcd.setPosn( LCD_DISPLAY_STATUS_COLUMN, 1 );
			lcd.writeBuf( buffer, LCD_DISPLAY_STATUS_WIDTH );
			break;
		}
#endif

#if LCD_DISPLAY_ROWS > 2
		case 2: {
			char		buffer[ LCD_DISPLAY_STATUS_WIDTH ];

			//
			//	Row 2, DCC packets (T)ransmitted sent per second
			//
			buffer[ 0 ] = 'T';
			if( backfill_int_to_text( buffer+1, mul_div<int>( lcd_statistic_packets, 1000, LCD_UPDATE_INTERVAL ), LCD_DISPLAY_STATUS_WIDTH-2 )) {
				memset( buffer+1, HASH, LCD_DISPLAY_STATUS_WIDTH-2 );
			}
			buffer[ LCD_DISPLAY_STATUS_WIDTH-1 ] = SELECT_COMPAT( '|', ':' );
			lcd.setPosn( LCD_DISPLAY_STATUS_COLUMN, 2 );
			lcd.writeBuf( buffer, LCD_DISPLAY_STATUS_WIDTH );
			lcd_statistic_packets = 0;
			break;
		}
#endif

#if LCD_DISPLAY_ROWS > 3
		case 3: {
			unsigned int		uptime;
			char			buffer[ LCD_DISPLAY_STATUS_WIDTH ];
			byte			odd;

			//
			//	Row 3, The (U)ptime in seconds
			//
			//
			//	Store the uptime as seconds.
			//
			uptime = now / 1000;

			buffer[ 0 ] = 'U';
			odd = uptime & 1;
			if( uptime < 1000 ) {
				//
				//	Display time in seconds.
				//
				(void)backfill_int_to_text( buffer+1, uptime, LCD_DISPLAY_STATUS_WIDTH-3 );
				buffer[ LCD_DISPLAY_STATUS_WIDTH-2 ] = odd? 's': SPACE;
			}
			else {
				if(( uptime /= 60 ) < 1000 ) {
					//
					//	Display time in minutes.
					//
					(void)backfill_int_to_text( buffer+1, uptime, LCD_DISPLAY_STATUS_WIDTH-3 );
					buffer[ LCD_DISPLAY_STATUS_WIDTH-2 ] = odd? 'm': SPACE;
				}
				else {
					//
					//	Display time in HOURS!
					//
					uptime /= 60;
					(void)backfill_int_to_text( buffer+1, uptime, LCD_DISPLAY_STATUS_WIDTH-3 );
					buffer[ LCD_DISPLAY_STATUS_WIDTH-2 ] = odd? 'h': SPACE;
				}
			}
			buffer[ LCD_DISPLAY_STATUS_WIDTH-1 ] = SELECT_COMPAT( '|', ':' );
			lcd.setPosn( LCD_DISPLAY_STATUS_COLUMN, 3 );
			lcd.writeBuf( buffer, LCD_DISPLAY_STATUS_WIDTH );
			break;
		}
#endif
		default: {
			//
			//	On this count we are updating the buffer section of the screen.
			//
			break;
		}
	}

	//
	//	+--------------------+	The DISTRICT area of the display, showing:
	//	|      DDDDDDD       |	Details of each districts status and power
	//	|      DDDDDDD       |	output (the source data for the Load value
	//	|      DDDDDDD       |	in the Status area).
	//	|      DDDDDDD       |
	//	+--------------------+	Right hand column set to '|'
	//
	if( next_lcd_line < LCD_DISPLAY_ROWS ) {
		char		buffer[ LCD_DISPLAY_DISTRICT_WIDTH ];
		byte		d, c;

		//
		//	Get the compiler to calculate the parameters
		//	which the display updates need to operate within.
		//
		//	LCD_DISPLAY_DISTRICT_COLS	How many columns are required
		//	LCD_DISPLAY_DISTRICT_COLW	How wide each column is
		//	LCD_DISPLAY_DISTRICT_LEFT	How many columns are left over
		//					including the edge bar
		//
#define LCD_DISPLAY_DISTRICT_COLS ((SHIELD_OUTPUT_DRIVERS+LCD_DISPLAY_ROWS-1)/LCD_DISPLAY_ROWS)
#define LCD_DISPLAY_DISTRICT_COLW ((LCD_DISPLAY_DISTRICT_WIDTH-1)/LCD_DISPLAY_DISTRICT_COLS)
#define LCD_DISPLAY_DISTRICT_LEFT (LCD_DISPLAY_DISTRICT_WIDTH-(LCD_DISPLAY_DISTRICT_COLS*LCD_DISPLAY_DISTRICT_COLW))

		//
		//	Display the columns which do not contain valid data (the edge and
		//	any unused spaces).
		//
#if ( LCD_DISPLAY_DISTRICT_LEFT-1 ) > 1
		memset( buffer, SPACE, LCD_DISPLAY_DISTRICT_LEFT-1 );
#elif ( LCD_DISPLAY_DISTRICT_LEFT-1 ) == 1
		buffer[ 0 ] = SPACE;
#endif
		buffer[ LCD_DISPLAY_DISTRICT_LEFT-1 ] = '|';
		//
		//	display..
		//
		lcd.setPosn( LCD_DISPLAY_BUFFER_COLUMN - LCD_DISPLAY_DISTRICT_LEFT, next_lcd_line );
		lcd.writeBuf( buffer, LCD_DISPLAY_DISTRICT_LEFT );
			
		//
		//	We cheat a little here; we know drivers and LCD rows are all numbered
		//	from 0, so we can use the LCD data to move the driver number through
		//	the driver array picking out only those that occupy the LCD line we
		//	are interested in.
		//
		c = 0;
		for( d = next_lcd_line; d < SHIELD_OUTPUT_DRIVERS; d += LCD_DISPLAY_ROWS ) {
			//
			//	pick where we are drawing...
			//
			lcd.setPosn( LCD_DISPLAY_DISTRICT_COLUMN + c, next_lcd_line );
			//
			//	Fill in the buffer and draw it.
			//
			buffer[ 0 ] = 'A' + d;
			switch( output_load[ d ].status ) {
				case DRIVER_OFF: {
					//
					//	This driver/district is in a restart
					//	caused by a power exception.
					//
					memset( buffer+1, '*', LCD_DISPLAY_DISTRICT_COLW-1 );
					break;
				}
				case DRIVER_FLIPPED: {
					//
					//	This driver/district has been "flipped" to
					//	the opposite phase.
					//
					memset( buffer+1, '~', LCD_DISPLAY_DISTRICT_COLW-1 );
					break;
				}
				case DRIVER_DISABLED: {
					//
					//	This driver/district is disabled.
					//
					memset( buffer+1, SPACE, LCD_DISPLAY_DISTRICT_COLW-1 );
					break;
				}
				default: {
					//
					//	Draw the bar as a reflection of power consumption
					//
					bar_graph( output_load[ d ].compound_value[ COMPOUNDED_VALUES-1 ], AVERAGE_CURRENT_LIMIT, buffer+1, LCD_DISPLAY_DISTRICT_COLW-1 );
					break;
				}
			}
			//
			//	display and move on
			//
			lcd.writeBuf( buffer, LCD_DISPLAY_DISTRICT_COLW );
			c += LCD_DISPLAY_DISTRICT_COLW;
		}
		//
		//	Empty any remaining spaces
		//
		if( c < ( LCD_DISPLAY_DISTRICT_WIDTH - LCD_DISPLAY_DISTRICT_LEFT )) {
			//
			//	prep spaces
			//
			memset( buffer, SPACE, LCD_DISPLAY_DISTRICT_COLW );
			//
			//	draw them as required.
			//
			while( c < ( LCD_DISPLAY_DISTRICT_WIDTH - LCD_DISPLAY_DISTRICT_LEFT )) {
				lcd.setPosn( LCD_DISPLAY_DISTRICT_COLUMN + c, next_lcd_line );
				lcd.writeBuf( buffer, LCD_DISPLAY_DISTRICT_COLW );
				c += LCD_DISPLAY_DISTRICT_COLW;
			}
		}
	}

	//
	//	+--------------------+	The BUFFER area of the display, showing:
	//	|             BBBBBBB|	Buffers in use and the action in place.
	//	|             BBBBBBB|	This area is broken up into as many one
	//	|             BBBBBBB|	line areas as will fit.  Each area is
	//	|             BBBBBBB|	LCD_DISPLAY_BUFFER_WIDTH bytes wide.
	//	+--------------------+	
	//
	if( next_lcd_line == LCD_DISPLAY_ROWS ){
		byte	r;

		r = 0;
		for( byte i = 0; i < TRANSMISSION_BUFFERS; i++ ) {
				if(( circular_buffer[ i ].state == TBS_RUN )||( circular_buffer[ i ].state == TBS_RELOAD )) {
					lcd.setPosn( LCD_DISPLAY_BUFFER_COLUMN, r );
					lcd.writeBuf( circular_buffer[ i ].display, LCD_DISPLAY_BUFFER_WIDTH );
					if(( r += 1 ) >= LCD_DISPLAY_ROWS ) break;
				}
		}
		//
		//	Clear out remaining spaces.
		//
		while( r < LCD_DISPLAY_ROWS ) {
			lcd.setPosn( LCD_DISPLAY_BUFFER_COLUMN, r );
			lcd.fill( SPACE, LCD_DISPLAY_BUFFER_WIDTH );
			r++;
		}
	}

	//
	//	Move to the next line, *but* also step through
	//	count LCD_DISPLAY_ROWS as we use this to update
	//	whole buffer area.
	//
	if( ++next_lcd_line > LCD_DISPLAY_ROWS ) next_lcd_line = 0;
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
	ASSERT(( adrs >= MINIMUM_DCC_ADDRESS )&&( adrs <= MAXIMUM_DCC_ADDRESS ));
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
//	The following commands are only require on the Programming
//	Track.
//
#ifdef PROGRAMMING_TRACK

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
//	Compose a write bit to CV command
//
static byte compose_set_cv_bit( byte *command, int cv, int bnum, int value ) {

	ASSERT( command != NULL );
	ASSERT(( cv >= MINIMUM_CV_ADDRESS )&&( cv <= MAXIMUM_CV_ADDRESS ));
	ASSERT(( bnum >= 0 )&&( bnum <= 7 ));
	ASSERT(( value == 0 )||( value == 1 ));

	//
	//	0111CCVV 0 VVVVVVVV 0 DDDDDDDD
	//
	//	VVVVVVVVVV:		CV address - 1.
	//
	//	CC:		00	Reserved for future use
	//			01	Verify byte
	//			11	Write byte
	//			10*	Bit manipulation
	//
	//	DDDDDDDD:	For BIT MANIPULATION
	//	-> 111CDBBB:	D	0|1*	Contains the value of the bit to be verified or written
	//			C	1*	WRITE BIT
	//				0	VERIFY BIT
	//			BBB	0-7*	Bit Number
	//
	cv -= MINIMUM_CV_ADDRESS;
	command[ 0 ] = 0b01111000 | (( cv >> 8 ) & 0b00000011 );
	command[ 1 ] = cv & 0b11111111;
	command[ 2 ] = 0b11110000 | ( value << 3 ) | bnum;
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

	//
	//	0111CCVV 0 VVVVVVVV 0 DDDDDDDD
	//
	//	VVVVVVVVVV:	0-1023	CV address - 1.
	//
	//	CC:		00	Reserved for future use
	//			01	Verify byte
	//			11	Write byte
	//			10*	Bit manipulation
	//
	//	DDDDDDDD:	For BIT MANIPULATION
	//	-> 111CDBBB:	D	0|1*	Contains the value of the bit to be verified or written
	//			C	1	WRITE BIT
	//				0*	VERIFY BIT
	//			BBB	0-7*	Bit Number
	//
	cv -= MINIMUM_CV_ADDRESS;
	command[ 0 ] = 0b01111000 | (( cv >> 8 ) & 0b00000011 );
	command[ 1 ] = cv & 0b11111111;
	command[ 2 ] = 0b11100000 | ( value << 3 ) | bnum;
	return( 3 );
}

//
//	End programming track
//
#endif

//
//	End non-compatible
//
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
	if( backfill_int_to_text( buffer, target, 4 )) memset( buffer, HASH, 4 );

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
	if( backfill_int_to_text( buffer + 5, speed, 2 )) memset( buffer + 5, HASH, 2 );
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
	if( backfill_int_to_text( buffer, target, 4 )) memset( buffer, HASH, 4 );

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
	if( backfill_int_to_text( buffer, target, 4 )) memset( buffer, HASH, 4 );

#if LCD_DISPLAY_BUFFER_WIDTH >= 5
	//
	//	The actions taken
	//
	switch( state ) {
		case FUNCTION_OFF: {
			buffer[ 4 ] = LCD_ACTION_DISABLE;
			break;
		}
		case FUNCTION_ON: {
			buffer[ 4 ] = LCD_ACTION_ENABLE;
			break;
		}
		case FUNCTION_TOGGLE: {
			buffer[ 4 ] = LCD_ACTION_TOGGLE;
			break;
		}
		default: {
			buffer[ 4 ] = '?';
			break;
		}
	}
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
//	Include summary command for Programming Track commands.
//
#ifdef PROGRAMMING_TRACK

//
//	Display setting a CV to a value.
//
static void lcd_summary_setcv( char *buffer, int cv, int value ) {
	//
	//	Fill in the CV number.
	//
	if( backfill_int_to_text( buffer, cv, 3 )) memset( buffer, HASH, 3 );
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
//	Display setting a CV to a value.
//
static void lcd_summary_setcvbit( char *buffer, int cv, int bnum, int value ) {
	//
	//	Fill in the CV number.
	//
	if( backfill_int_to_text( buffer, cv, 3 )) memset( buffer, HASH, 3 );
	buffer[ 3 ] = '/';
	
#if LCD_DISPLAY_BUFFER_WIDTH >= 5
	//
	//	bit number.
	//
	buffer[ 4 ] = '0' + bnum;
#endif

#if LCD_DISPLAY_BUFFER_WIDTH >= 6
	buffer[ 5 ] = '=';
#endif
	
#if LCD_DISPLAY_BUFFER_WIDTH >= 7
	//
	//	new value.
	//
	buffer[ 6 ] = '0' + value;
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
	if( backfill_int_to_text( buffer, cv, 3 )) memset( buffer, HASH, 3 );
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
static void lcd_summary_readcv( char *buffer, int cv, int bnum, int value ) {
	//
	//	Fill in the CV number.
	//
	if( backfill_int_to_text( buffer, cv, 3 )) memset( buffer, HASH, 3 );
	buffer[ 3 ] = '/';
	
#if LCD_DISPLAY_BUFFER_WIDTH >= 5
	//
	//	bit number.
	//
	buffer[ 4 ] = '0' + bnum;
#endif

#if LCD_DISPLAY_BUFFER_WIDTH >= 7
	//
	//	Check value.
	//
	buffer[ 5 ] = '?';
	buffer[ 6 ] = '0' + value;
#endif

#if LCD_DISPLAY_BUFFER_WIDTH > 7
	//
	//	Space fill the tail area
	//
	memset( buffer + 7, SPACE, LCD_DISPLAY_BUFFER_WIDTH - 7 );
#endif
}

//
//	End Programming Track summary
//
#endif

//
//	End DCC Compatibility
//
#endif

//
//	End LCD Display Enabled
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
	//	Set CV bit value (Programming track)
	//	------------------------------------
	//
	//	Set the specified CV bit with the supplied
	//	value.
	//
	//	[U CV BIT VALUE] -> [U CV BIT VALUE STATE]
	//
	//		CV:	Number of CV to set (1-1024)
	//		BIT:	Bit number (0 LSB - 7 MSB)
	//		VALUE:	0 or 1
	//		STATE:	1=Confirmed, 0=Failed
	//
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
	//	[R CV BIT VALUE] -> [R CV BIT VALUE STATE]
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
	//	Report status of individual districts
	//
	//		[D a b ...]
	//
	//		Reported numbers (a, b, c ...) reflect
	//		the individual districts A, B C etc (independent
	//		of the role of the district).  The values
	//		provided follow the following table:
	//
	//			0	Disabled
	//			1	Enabled
	//			2	Phase Flipped
	//			3	Overloaded
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
					errors.log_error( INVALID_ARGUMENT_COUNT, cmd );
					break;
				}
				if( cmd == '1' ) {
					if( power_on_main_track()) link_main_buffers();
					reply_1( reply, 'p', 1 );
					if( !console.print( reply )) errors.log_error( COMMAND_REPORT_FAIL, cmd );
				}
				else {
					(void)power_off_tracks();
					reply_1( reply, 'p', 0 );
					if( !console.print( reply )) errors.log_error( COMMAND_REPORT_FAIL, cmd );
				}
				
				break;
			}
			case 'c': {
				//
				//	Report power
				//
				if( args != 0 ) {
					errors.log_error( INVALID_ARGUMENT_COUNT, cmd );
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
					errors.log_error( INVALID_ARGUMENT_COUNT, cmd );
					break;
				}
				switch( arg[ 0 ]) {
					case 0: {	// Power track off
						(void)power_off_tracks();
						reply_1( reply, 'P', 0 );
						if( !console.print( reply )) errors.log_error( COMMAND_REPORT_FAIL, cmd );
						break;
					}
					case 1: {
						//
						//	Power Main Track on
						//
						if( global_power_state != GLOBAL_POWER_OFF ) {
							errors.log_error( POWER_NOT_OFF, cmd );
							break;
						}
						if( power_on_main_track()) link_main_buffers();
						reply_1( reply, 'P', 1 );
						if( !console.print( reply )) errors.log_error( COMMAND_REPORT_FAIL, cmd );
						break;
					}
					case 2: {
						//
						//	Power Prog Track on
						//
#ifdef PROGRAMMING_TRACK
						if( global_power_state != GLOBAL_POWER_OFF ) {
							errors.log_error( POWER_NOT_OFF, cmd );
							break;
						}
						if( power_on_prog_track()) link_prog_buffers();
						reply_1( reply, 'P', 2 );
						if( !console.print( reply )) errors.log_error( COMMAND_REPORT_FAIL, cmd );
#else
						errors.log_error( NO_PROGRAMMING_TRACK, cmd );
#endif
						break;
					}
					default: {
						errors.log_error( INVALID_STATE, cmd );
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
					errors.log_error( INVALID_ARGUMENT_COUNT, cmd );
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
					errors.log_error( INVALID_BUFFER_NUMBER, buffer );
					break;
				}
				if(( target < MINIMUM_DCC_ADDRESS )||( target > MAXIMUM_DCC_ADDRESS )) {
					errors.log_error( INVALID_ADDRESS, target );
					break;
				}
				if((( speed < MINIMUM_DCC_SPEED )||( speed > MAXIMUM_DCC_SPEED ))&&( speed != EMERGENCY_STOP )) {
					errors.log_error( INVALID_SPEED, speed );
					break;
				}
				if(( dir != DCC_FORWARDS )&&( dir != DCC_BACKWARDS )) {
					errors.log_error( INVALID_DIRECTION, dir );
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
					errors.log_error( TRANSMISSION_BUSY, cmd );
					break;
				}
				//
				//	Now create and append the command to the pending list.
				//
				if( !create_pending_rec( &tail, target, 0, DCC_SHORT_PREAMBLE, 1, compose_motion_packet( command, target, speed, dir ), command )) {
					//
					//	Report that no pending record has been created.
					//
					errors.log_error( COMMAND_QUEUE_FAILED, cmd );
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
					errors.log_error( INVALID_ARGUMENT_COUNT, cmd );
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
					errors.log_error( INVALID_ADDRESS, target );
					break;
				}
				if((( speed < MINIMUM_DCC_SPEED )||( speed > MAXIMUM_DCC_SPEED ))&&( speed != EMERGENCY_STOP )) {
					errors.log_error( INVALID_SPEED, speed );
					break;
				}
				if(( dir != DCC_FORWARDS )&&( dir != DCC_BACKWARDS )) {
					errors.log_error( INVALID_DIRECTION, dir );
					break;
				}
				//
				//	Find a destination buffer
				//
				if(( buf = find_available_buffer( MOBILE_BASE_BUFFER, MOBILE_TRANS_BUFFERS, target )) == NULL ) {
					//
					//	No available buffers
					//
					errors.log_error( TRANSMISSION_BUSY, cmd );
					break;
				}
				buf->pending = release_pending_recs( buf->pending, false );
				tail = &( buf->pending );
				//
				//	Now create and append the command to the pending list.
				//
				if( !create_pending_rec( &tail, target, ((( speed == EMERGENCY_STOP )||( speed == MINIMUM_DCC_SPEED ))? TRANSIENT_COMMAND_REPEATS: 0 ), DCC_SHORT_PREAMBLE, 1, compose_motion_packet( command, target, speed, dir ), command )) {
					//
					//	Report that no pending record has been created.
					//
					errors.log_error( COMMAND_QUEUE_FAILED, cmd );
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
					errors.log_error( INVALID_ARGUMENT_COUNT, cmd );
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
					errors.log_error( INVALID_ADDRESS, adrs );
					break;
				}
				if(( subadrs < MIN_ACCESSORY_SUBADRS )||( subadrs > MAX_ACCESSORY_SUBADRS )) {
					errors.log_error( INVALID_ADDRESS, subadrs );
					break;
				}
				if(( state != ACCESSORY_ON )&&( state != ACCESSORY_OFF )) {
					errors.log_error( INVALID_STATE, state );
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
					errors.log_error( TRANSMISSION_BUSY, cmd );
					break;
				}
				buf->pending = release_pending_recs( buf->pending, false );
				tail = &( buf->pending );
				//
				//	Now create and append the command to the pending list.
				//
				if( !create_pending_rec( &tail, target, TRANSIENT_COMMAND_REPEATS, DCC_SHORT_PREAMBLE, 1, compose_accessory_change( command, adrs, subadrs, state ), command )) {
					//
					//	Report that no pending has been record created.
					//
					errors.log_error( COMMAND_QUEUE_FAILED, cmd );
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
					errors.log_error( INVALID_ARGUMENT_COUNT, cmd );
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
					errors.log_error( INVALID_ADDRESS, target );
					break;
				}
				if(( state != ACCESSORY_ON )&&( state != ACCESSORY_OFF )) {
					errors.log_error( INVALID_STATE, state );
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
					errors.log_error( TRANSMISSION_BUSY, cmd );
					break;
				}
				buf->pending = release_pending_recs( buf->pending, false );
				tail = &( buf->pending );
				//
				//	Now create and append the command to the pending list.
				//
				if( !create_pending_rec( &tail, target, TRANSIENT_COMMAND_REPEATS, DCC_SHORT_PREAMBLE, 1, compose_accessory_change( command, adrs, subadrs, state ), command )) {
					//
					//	Report that no pending has been record created.
					//
					errors.log_error( COMMAND_QUEUE_FAILED, cmd );
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
					errors.log_error( INVALID_ARGUMENT_COUNT, cmd );
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
					errors.log_error( INVALID_ADDRESS, target );
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
									errors.log_error( INVALID_FUNC_NUMBER, cmd );
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
					errors.log_error( TRANSMISSION_BUSY, cmd );
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
				if( !create_pending_rec( &tail, target, TRANSIENT_COMMAND_REPEATS, DCC_SHORT_PREAMBLE, 1, len, command )) {
					//
					//	Report that no pending has been record created.
					//
					errors.log_error( COMMAND_QUEUE_FAILED, cmd );
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
				bool		ok;

				//
				//	Mobile decoder set function state
				//	---------------------------------
				//
				//	[F ADRS FUNC STATE] -> [F ADRS FUNC STATE]
				//
				//		ADRS:	The short (1-127) or long (128-10239) address of the engine decoder
				//		FUNC:	The function number to be modified (0-21)
				//		STATE:	1=Enable, 0=Disable, 2=Toggle
				//
				//	Encoding a new "state": 2.  This is the act of turning
				//	on a function then almost immediately turning it off
				//	again (as a mnemonic this is, in binary, a 1 followed
				//	by a 0)
				//	
				if( args != 3 ) {
					errors.log_error( INVALID_ARGUMENT_COUNT, cmd );
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
					errors.log_error( INVALID_ADDRESS, target);
					break;
				}
				if(( func < MIN_FUNCTION_NUMBER )||( func > MAX_FUNCTION_NUMBER )) {
					errors.log_error( INVALID_FUNC_NUMBER, func );
					break;
				}
				if(( state != FUNCTION_ON )&&( state != FUNCTION_OFF )&&( state != FUNCTION_TOGGLE )) {
					errors.log_error( INVALID_STATE, state );
					break;
				}
				//
				//	Find a destination buffer
				//
				if(( buf = find_available_buffer( ACCESSORY_BASE_BUFFER, ACCESSORY_TRANS_BUFFERS, arg[ 0 ])) == NULL ) {
					//
					//	No available buffers
					//
					errors.log_error( TRANSMISSION_BUSY, cmd );
					break;
				}
				buf->pending = release_pending_recs( buf->pending, false );
				tail = &( buf->pending );
				if( state == FUNCTION_TOGGLE ) {
					//
					//	Create a pair of DCC commands to turn the function
					//	on then off:- the toggle option.
					//
					ok = create_pending_rec( &tail, target, TRANSIENT_COMMAND_REPEATS, DCC_SHORT_PREAMBLE, 1, compose_function_change( command, target, func, FUNCTION_ON ), command );
					ok &= create_pending_rec( &tail, target, TRANSIENT_COMMAND_REPEATS, DCC_SHORT_PREAMBLE, 1, compose_function_change( command, target, func, FUNCTION_OFF ), command );
					if( !ok ) {
						//
						//	Report that no pending has been record created.
						//
						buf->pending = release_pending_recs( buf->pending, false );
						errors.log_error( COMMAND_QUEUE_FAILED, cmd );
						break;
					}
				}
				else {
					//
					//	Now create and append the command to the pending list.
					//
					if( !create_pending_rec( &tail, target, TRANSIENT_COMMAND_REPEATS, DCC_SHORT_PREAMBLE, 1, compose_function_change( command, target, func, state ), command )) {
						//
						//	Report that no pending has been record created.
						//
						errors.log_error( COMMAND_QUEUE_FAILED, cmd );
						break;
					}
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
				if( state == FUNCTION_TOGGLE ) {
					reply_3( buf->contains, 'F', target, func, FUNCTION_OFF );
				}
				else {
					reply_3( buf->contains, 'F', target, func, state );
				}
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
#ifdef PROGRAMMING_TRACK
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
					errors.log_error( INVALID_ARGUMENT_COUNT, cmd );
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
					errors.log_error( INVALID_CV_NUMBER, cv );
					break;
				}
				if(( value < 0 )||( value > 255 )) {
					errors.log_error( INVALID_BYTE_VALUE, value );
					break;
				}
				//
				//	Find a destination buffer
				//
				if(( buf = find_available_buffer( PROGRAMMING_BASE_BUFFER, PROGRAMMING_BUFFERS, 0 )) == NULL ) {
					//
					//	No available buffers
					//
					errors.log_error( TRANSMISSION_BUSY, cmd );
					break;
				}
				buf->pending = release_pending_recs( buf->pending, false );
				tail = &( buf->pending );
				//
				//	Build up the command chain..
				//
				ok = create_pending_rec( &tail, 0, SERVICE_MODE_RESET_REPEATS, DCC_LONG_PREAMBLE, 1, compose_digital_reset( command ), command );
				ok &= create_pending_rec( &tail, 0, SERVICE_MODE_COMMAND_REPEATS, DCC_LONG_PREAMBLE, CONFIRMATION_PAUSE, compose_set_cv( command, cv, value ), command );
				ok &= create_pending_rec( &tail, 0, SERVICE_MODE_COMMAND_REPEATS, DCC_LONG_PREAMBLE, CONFIRMATION_PAUSE, compose_set_cv( command, cv, value ), command );
				ok &= create_pending_rec( &tail, 0, SERVICE_MODE_RESET_REPEATS, DCC_LONG_PREAMBLE, 1, compose_digital_reset( command ), command );
				if( !ok ) {
					//
					//	Report that no pending has been record created.
					//
					buf->pending = release_pending_recs( buf->pending, false );
					errors.log_error( COMMAND_QUEUE_FAILED, cmd );
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
				confirmation_enabled = true;
				load_confirmed = false;
				buf->reply = REPLY_ON_CONFIRM;
				buf->state = ( buf->state == TBS_EMPTY )? TBS_LOAD: TBS_RELOAD;
#else
				errors.log_error( NO_PROGRAMMING_TRACK, cmd );
#endif
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
#ifdef PROGRAMMING_TRACK
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
					errors.log_error( INVALID_ARGUMENT_COUNT, cmd );
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
					errors.log_error( INVALID_CV_NUMBER, cv );
					break;
				}
				if(( value < 0 )||( value > 255 )) {
					errors.log_error( INVALID_BYTE_VALUE, value );
					break;
				}
				//
				//	Find a destination buffer
				//
				if(( buf = find_available_buffer( PROGRAMMING_BASE_BUFFER, PROGRAMMING_BUFFERS, 0 )) == NULL ) {
					//
					//	No available buffers
					//
					errors.log_error( TRANSMISSION_BUSY, cmd );
					break;
				}
				buf->pending = release_pending_recs( buf->pending, false );
				tail = &( buf->pending );
				//
				//	Build up the command chain..
				//
				ok = create_pending_rec( &tail, 0, SERVICE_MODE_RESET_REPEATS, DCC_LONG_PREAMBLE, 1, compose_digital_reset( command ), command );
				ok &= create_pending_rec( &tail, 0, SERVICE_MODE_COMMAND_REPEATS, DCC_LONG_PREAMBLE, CONFIRMATION_PAUSE, compose_verify_cv( command, cv, value ), command );
				ok &= create_pending_rec( &tail, 0, SERVICE_MODE_COMMAND_REPEATS, DCC_LONG_PREAMBLE, CONFIRMATION_PAUSE, compose_verify_cv( command, cv, value ), command );
				ok &= create_pending_rec( &tail, 0, SERVICE_MODE_RESET_REPEATS, DCC_LONG_PREAMBLE, 1, compose_digital_reset( command ), command );
				if( !ok ) {
					//
					//	Report that no pending has been record created.
					//
					buf->pending = release_pending_recs( buf->pending, false );
					errors.log_error( COMMAND_QUEUE_FAILED, cmd );
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
				confirmation_enabled = true;
				load_confirmed = false;
				buf->reply = REPLY_ON_CONFIRM;
				buf->state = ( buf->state == TBS_EMPTY )? TBS_LOAD: TBS_RELOAD;
#else
				errors.log_error( NO_PROGRAMMING_TRACK, cmd );
#endif
				break;
			}
#endif
			//
			//	Set (Update) CV bit value
			//	-------------------------
			//
#ifdef DCC_PLUS_PLUS_COMPATIBILITY
				//
				//	No compatibility command defined.
				//
#else
			case 'U': {
#ifdef PROGRAMMING_TRACK
				PENDING_PACKET	**tail;
				TRANS_BUFFER	*buf;
				int		cv,
						bnum,
						value;
				bool		ok;

				//
				//	Set CV bit value (Programming track)
				//	------------------------------------
				//
				//	Set the specified CV bit with the supplied
				//	value.
				//
				//	[U CV BIT VALUE] -> [U CV BIT VALUE]
				//
				//		CV:	Number of CV to set (1-1024)
				//		BIT:	Bit number (0 LSB - 7 MSB)
				//		VALUE:	0 or 1
				//
				if( args != 3 ) {
					errors.log_error( INVALID_ARGUMENT_COUNT, cmd );
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
					errors.log_error( INVALID_CV_NUMBER, cv );
					break;
				}
				if(( bnum < 0 )||( bnum > 7 )) {
					errors.log_error( INVALID_BIT_NUMBER, bnum );
					break;
				}
				if(( value != 0 )&&( value != 1 )) {
					errors.log_error( INVALID_BIT_VALUE, value );
					break;
				}
				//
				//	Find a destination buffer
				//
				if(( buf = find_available_buffer( PROGRAMMING_BASE_BUFFER, PROGRAMMING_BUFFERS, 0 )) == NULL ) {
					//
					//	No available buffers
					//
					errors.log_error( TRANSMISSION_BUSY, cmd );
					break;
				}
				buf->pending = release_pending_recs( buf->pending, false );
				tail = &( buf->pending );
				//
				//	Build up the command chain..
				//
				ok = create_pending_rec( &tail, 0, SERVICE_MODE_RESET_REPEATS, DCC_LONG_PREAMBLE, 1, compose_digital_reset( command ), command );
				ok &= create_pending_rec( &tail, 0, SERVICE_MODE_COMMAND_REPEATS, DCC_LONG_PREAMBLE, CONFIRMATION_PAUSE, compose_set_cv_bit( command, cv, bnum, value ), command );
				ok &= create_pending_rec( &tail, 0, SERVICE_MODE_COMMAND_REPEATS, DCC_LONG_PREAMBLE, CONFIRMATION_PAUSE, compose_set_cv_bit( command, cv, bnum, value ), command );
				ok &= create_pending_rec( &tail, 0, SERVICE_MODE_RESET_REPEATS, DCC_LONG_PREAMBLE, 1, compose_digital_reset( command ), command );
				if( !ok ) {
					//
					//	Report that no pending has been record created.
					//
					buf->pending = release_pending_recs( buf->pending, false );
					errors.log_error( COMMAND_QUEUE_FAILED, cmd );
					break;
				}

#ifdef LCD_DISPLAY_ENABLE
				//
				//	Complete LCD summary.
				//
				lcd_summary_setcvbit( buf->display, cv, bnum, value );
#endif

				//
				//	Construct the future reply and set the state. The '#' in the reply
				//	will be replaced by a 1 or 0 to reflect confirmation.
				//
				reply_3c( buf->contains, 'U', cv, bnum, value );
				confirmation_enabled = true;
				load_confirmed = false;
				buf->reply = REPLY_ON_CONFIRM;
				buf->state = ( buf->state == TBS_EMPTY )? TBS_LOAD: TBS_RELOAD;
#else
				errors.log_error( NO_PROGRAMMING_TRACK, cmd );
#endif
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
#ifdef PROGRAMMING_TRACK
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
					errors.log_error( INVALID_ARGUMENT_COUNT, cmd );
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
					errors.log_error( INVALID_CV_NUMBER, cv );
					break;
				}
				if(( bnum < 0 )||( bnum > 7 )) {
					errors.log_error( INVALID_BIT_NUMBER, bnum );
					break;
				}
				if(( value != 0 )&&( value != 1 )) {
					errors.log_error( INVALID_BIT_VALUE, value );
					break;
				}
				//
				//	Find a destination buffer
				//
				if(( buf = find_available_buffer( PROGRAMMING_BASE_BUFFER, PROGRAMMING_BUFFERS, 0 )) == NULL ) {
					//
					//	No available buffers
					//
					errors.log_error( TRANSMISSION_BUSY, cmd );
					break;
				}
				buf->pending = release_pending_recs( buf->pending, false );
				tail = &( buf->pending );
				//
				//	Build up the command chain..
				//
				ok = create_pending_rec( &tail, 0, SERVICE_MODE_RESET_REPEATS, DCC_LONG_PREAMBLE, 1, compose_digital_reset( command ), command );
				ok &= create_pending_rec( &tail, 0, SERVICE_MODE_COMMAND_REPEATS, DCC_LONG_PREAMBLE, CONFIRMATION_PAUSE, compose_verify_cv_bit( command, cv, bnum, value ), command );
				ok &= create_pending_rec( &tail, 0, SERVICE_MODE_COMMAND_REPEATS, DCC_LONG_PREAMBLE, CONFIRMATION_PAUSE, compose_verify_cv_bit( command, cv, bnum, value ), command );
				ok &= create_pending_rec( &tail, 0, SERVICE_MODE_RESET_REPEATS, DCC_LONG_PREAMBLE, 1, compose_digital_reset( command ), command );
				if( !ok ) {
					//
					//	Report that no pending has been record created.
					//
					buf->pending = release_pending_recs( buf->pending, false );
					errors.log_error( COMMAND_QUEUE_FAILED, cmd );
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
				reply_3c( buf->contains, 'R', cv, bnum, value );
				confirmation_enabled = true;
				load_confirmed = false;
				buf->reply = REPLY_ON_CONFIRM;
				buf->state = ( buf->state == TBS_EMPTY )? TBS_LOAD: TBS_RELOAD;
#else
				errors.log_error( NO_PROGRAMMING_TRACK, cmd );
#endif
				break;
			}
#endif
			//
			//	EEPROM configurable constants
			//
			case 'Q': {
				char	*n;
				word	*w;
				byte	*b;
				
				//
				//	Accessing EEPROM configurable constants
				//
				//	[Q] -> [Q N]			Return number of tunable constants
				//	[Q C] ->[Q C V NAME]		Access a specific constant C (range 0..N-1)
				//	[Q C V V] -> [Q C V NAME]	Set a specific constant C to value V,
				//					second V is to prevent accidental
				//					update.
				//	[Q -1 -1] -> [Q -1 -1]		Reset all constants to default.
				//
				switch( args ) {
					case 0: {
						console.print( PROT_IN_CHAR );
						console.print( 'Q' );
						console.print( CONSTANTS );
						console.print( PROT_OUT_CHAR );
						console.println();
						break;
					}
					case 1: {
						if( find_constant( arg[ 0 ], &n, &b, &w ) != ERROR ) {
							console.print( PROT_IN_CHAR );
							console.print( 'Q' );
							console.print( arg[ 0 ]);
							console.print( SPACE );
							if( b ) {
								console.print( (word)(*b ));
							}
							else {
								console.print( *w );
							}
							console.print( SPACE );
							console.print_PROGMEM( n );
							console.print( PROT_OUT_CHAR );
							console.println();
						}
						break;
					}
					case 2: {
						if(( arg[ 0 ] == -1 )&&( arg[ 1 ] == -1 )) {
							reset_constants();
							console.print( PROT_IN_CHAR );
							console.print( 'Q' );
							console.print( -1 );
							console.print( SPACE );
							console.print( -1 );
							console.print( PROT_OUT_CHAR );
							console.println();
						}
						break;
					}
					case 3: {
						if(( arg[ 1 ] == arg[ 2 ])&&( find_constant( arg[ 0 ], &n, &b, &w ) != ERROR )) {
							if( b ) {
								*b = (byte)arg[ 1 ];
							}
							else {
								*w = arg[ 1 ];
							}
							record_constants();
							console.print( PROT_IN_CHAR );
							console.print( 'Q' );
							console.print( arg[ 0 ]);
							console.print( SPACE );
							if( b ) {
								console.print( (word)( *b ));
							}
							else {
								console.print( *w );
							}
							console.print( SPACE );
							console.print_PROGMEM( n );
							console.print( PROT_OUT_CHAR );
							console.println();
						}
						break;
					}
					default: {
						errors.log_error( INVALID_ARGUMENT_COUNT, cmd );
						break;
					}
				}
				break;
			}
			default: {
				//
				//	Here we capture any unrecognised command letters.
				//
				errors.log_error( UNRECOGNISED_COMMAND, cmd );
				break;
			}
		}
	}
	//
	//	Done.
	//
}

//
//	The command buffer:
//
static char cmd_buf[ MAXIMUM_DCC_CMD+1 ];	// "+1" to allow for EOS with checking code.
static byte cmd_used = 0;
static bool in_cmd_packet = false;

//
//	The character input routine.
//
static void process_input( byte count ) {
	while( count-- ) {
		char	c;

		switch(( c = console.read())) {
			case PROT_IN_CHAR: {
				//
				//	Found the start of a command, regardless of what we thought
				//	we were doing we clear out the buffer and start collecting
				//	the content of the packet.
				//
				cmd_used = 0;
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
					cmd_buf[ cmd_used ] = EOS;
					scan_line( cmd_buf );
					//
					//	Now reset the buffer space.
					//
					cmd_used = 0;
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
						cmd_used = 0;
						in_cmd_packet =  false;
					}
					else {
						//
						//	"Valid" character (at least it is a normal character), so
						//	save it if possible.
						//
						if( cmd_used < MAXIMUM_DCC_CMD-1 ) {
							cmd_buf[ cmd_used++ ] = c;
						}
						else {
							//
							//	we have lost some part of the command.  Report error and
							//	drop the malformed command.
							//
							errors.log_error( DCC_COMMAND_OVERFLOW, MAXIMUM_DCC_CMD );
							cmd_used = 0;
							in_cmd_packet =  false;
						}
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
//	Define a the periodic variable that is used to initiate
//	activities at durations greater than microseconds or
//	milliseconds.
//
static unsigned long periodic = 0;

//
//	Finally, the main event loop:
//
void loop( void ) {
	byte		ready;

	//
	//	Grab a copy of the time "now" as several routines
	//	require a notion of how time has passed.
	//
	now = millis();

	//
	//	Initially service those background facilities which
	//	need regular attention.
	//
	twi_eventProcessing();
	lcd.service();

	//
	//	Is there Serial data to be processed...
	//
	if(( ready = console.available())) process_input( ready );
	
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
	}
	//
	//	Time for periodic activities?
	//
	if( now > periodic ) {
		//
		//	Yes, but before getting on with them, set the
		//	next periodic time.
		//
		periodic = now + PERIODIC_INTERVAL;

#ifndef DCC_PLUS_PLUS_COMPATIBILITY
		//
		//	Forward out an asynchronous power update.
		//
		report_track_power();
#endif
	}

#ifdef LCD_DISPLAY_ENABLE
	//
	//	Send updates to the LCD.
	//
	display_lcd_updates();
#endif


	//
	//	Then we give the Error management system an
	//	opportunity to queue some output data.
	//
	flush_error_queue();

}

//
//	EOF
//

//
//	DCC.h
//	=====
//
//	Define the engine that generates a DCC signal through a
//	virtual class interface.
//

#ifndef _DCC_H_
#define _DCC_H_

//
//	Bring in the Driver class as we require this inside the DCC
//	Generator class.
//
#include "Environment.h"
#include "Parameters.h"
#include "Configuration.h"

#include "Protocol.h"
#include "Driver.h"
#include "Task_Entry.h"
#include "Critical.h"
#include "Driver.h"
#include "DCC_Constant.h"
#include "Pin_IO.h"
#include "Function.h"
#include "Buffer.h"
#include "Constants.h"
#include "Average.h"
#include "Memory_Heap.h"

//
//	Hardware specific DCC configuration
//	-----------------------------------
//
//	Define the timer device we will be using for this module.
//
#if defined( __AVR_ATmega328__ )| defined( __AVR_ATmega328P__ )| defined( __AVR_ATmega328PB__ )

//
//	Standard Nano or Uno R3 configuration
//
#define DCC_TCCRnA		TCCR2A
#define DCC_TCCRnB		TCCR2B
#define DCC_TIMERn_COMPA_vect	TIMER2_COMPA_vect
#define DCC_TCNTn		TCNT2
#define DCC_OCRnA		OCR2A
#define DCC_WGMn1		WGM21
#define DCC_CSn0		CS20
#define DCC_CSn1		CS21
#define DCC_TIMSKn		TIMSK2
#define DCC_OCIEnA		OCIE2A


#elif defined( __AVR_ATmega2560__ )
//
//	Standard Mega 2560 configuration
//
#define DCC_TCCRnA		TCCR2A
#define DCC_TCCRnB		TCCR2B
#define DCC_TIMERn_COMPA_vect	TIMER2_COMPA_vect
#define DCC_TCNTn		TCNT2
#define DCC_OCRnA		OCR2A
#define DCC_WGMn1		WGM21
#define DCC_CSn0		CS20
#define DCC_CSn1		CS21
#define DCC_TIMSKn		TIMSK2
#define DCC_OCIEnA		OCIE2A

#else
//
//	Firmware has not been configured for this board.
//
#error "DCC has not been configured for this board"

#endif

//
//	Set an alias for the Timer Compare Register.  This is
//	not essential given that the actual register name is already
//	hidden behind an alias.  However, the "english" stlye alias
//	make the source code easier to read and understand.  Got to be
//	a good thing.
//
#define DCC_COMPARE_REGISTER	DCC_OCRnA

//
//	Do similar for the actual counter register
//
#define DCC_COUNTER_REGISTER	DCC_TCNTn


//
//	Define the DCC Generator class.
//
class DCC : public Task_Entry, Memory_Recovery {
public:
	//
	//	Timing definitions
	//	==================
	//

	//
	//	The following discussion and calculations are the second
	//	system this firmwware will have used for the correct
	//	generation of the DCC data signal.  I feel that an
	//	explanation of the first methods approach and how it
	//	worked will help with the generation and understanding
	//	of the second, new, system,
	//
	//	Verion 1
	//	--------
	//
	//	The original timing generation is based on my early
	//	and simplistic understanding of the Clock/Timing hardware
	//	of the AVR MCU.  Essentially being unclear on how flexible
	//	it was and what was possible I looked for a solution where
	//	the hardware could be configured a single time and all
	//	the necessary signal data could be generated from this.
	//
	//	Hense the hardware was configured to generate a 14.5us
	//	interrupt which formed whole sub-sections of half a DCC
	//	digit where 4 "ticks" formed the 1 and 7 the 0.  This
	//	lead to a lot of interrupts but the earlier firmware being
	//	simpler and more limited, retained the capacity to support
	//	these in a timely way.
	//
	//	A valid DCC signal was generated.
	//
	//	The "full on" re-write of the firmware (caused by the aim
	//	of producing a DCC Controller that could be directly used
	//	by a human operator) pushed the MCU to the point where
	//	it was simply unable to keep time.
	//
	//	No valid DCC was generated.
	//
	//	Version 2
	//	---------
	//
	//	The key change from version one is that the Timer will
	//	now be modified dynamically to generate an interrupt
	//	*only* when it is time for the DCC signal to "flip" and
	//	transition from "+/-" to "-/+" or back again.
	//
	//	To do this we will, once again, require some form of
	//	interval that factors into the timing for half of a DCC
	//	1 (58us) or 0 digit (100us) and we can base this
	//	calculation on the work done for version of the timing
	//	code.
	//
	//	This is actually simpler because we can count (with the
	//	hardware) much, much faster than software.  So lets look
	//	at some blindingly obvious options:
	//
	//	The CPU clock, as an input to the timer, can be divided
	//	by the following factors:
	//
	//		1, 8, 64, 256, or 1024
	//
	//	For an *ideal* fit we need to configure the clock and
	//	divider to produce 1 or 2 micro second increments.  The
	//	following table show what can be done with the CPU clock
	//	and dividers available (value are in micro seconds):
	//
	//	Clock		8Mhz	16MHz	20Mhz
	//	Divider		----	-----	-----
	//		1	0.125	0.0625	0.05
	//		8	1	0.5	0.4
	//		64	8	4	3.2
	//		256	32	16	12.8
	//		1024	128	64	51.2
	//
	//	Our aim is to find a balance between keeping the interval
	//	counters inside the 8-bit range while keeping the actual
	//	intervals being counted as small as possible.  The
	//	following tables break down each of the Clock speeds
	//	while analysing the divider options.  The optimal divider
	//	ratio is marked with an asterix (*).
	//
	//	8 MHz
	//	-----
	//
	//	This is an unlikely speed, but might be required if the
	//	AVR MCU is to be run at 3.3 volts rather than 5 volts.
	//
	//	Digit		Interval	1 (58us)	0 (100us)
	//	Divider		--------	--------	---------
	//		1	0.125		464		800
	//		8*	1		58		100
	//		64	8		7.25		12.5
	//		256	32		1.81..		3.125
	//		1024	128		0.45..		0.78..
	//
	//	16 MHz
	//	------
	//
	//	Digit		Interval	1 (58us)	0 (100us)
	//	Divider		--------	--------	---------
	//		1	0.0625		928		1600
	//		8*	0.5		116		200
	//		64	4		14.5		25
	//		256	16		3.625		6.25
	//		1024	64		0.90..		1.56..
	//	20 MHz
	//	------
	//
	//	Digit		Interval	1 (58us)	0 (100us)
	//	Divider		--------	--------	---------
	//		1	0.05		1160		2000
	//		8*	0.4		145		250
	//		64	3.2		18.125		31.25
	//		256	12.8		4.53..		7.81..
	//		1024	51.2		1.13..		1.95..
	//
	//	So, for the "norminal" range of AVR MCU Clock speeds the
	//	best clock divider is always the same: 8.
	//
	//	This is a surprise to me, but seems correct, so on this
	//	basis we continue:
	//
	//	The following table summarises the counts required to
	//	time the 0 and 1 DCC digits:
	//
	//	Clock		8	16	20
	//	Digit		-	--	--
	//		0	100	200	250
	//		1	58	116	145
	//
	//	These counts all fall inside the 8-bit timer counter
	//	*and* have a 100% accuracy!
	//
	
	//
	//	Encode this data into the DCC Class.
	//
	static const byte	timer_clock_prescaler = 8;
	
#if F_CPU == 8000000
	static const byte	timer_digit_0_cycles = 100;
	static const byte	timer_digit_1_cycles = 58;
	
#elif F_CPU == 16000000
	static const byte	timer_digit_0_cycles = 200;
	static const byte	timer_digit_1_cycles = 116;
	
#elif F_CPU == 20000000
	static const byte	timer_digit_0_cycles = 250;
	static const byte	timer_digit_1_cycles = 145;
	
#else
	//
	//	The target MCU clock frequency has not been accounted for.
	//
#error "DCC Clock speed calculation needs to be calculate for this clock rate."

	//
	//	The calculations outlined above need to be carried out and appropriate
	//	results captured in the definitions of timer_interrupt_cycles and
	//	timer_clock_prescaler values.
	//
#endif

	//
	//	DCC Constant Definitions
	//	------------------------
	//

	//
	//	Define the maximum number of bytes in a DCC packet that we
	//	can handle.
	//
	//	Note:	This is different from both:
	//
	//	o	The number of bit transitions which the system will
	//		support.
	//
	//	o	The number of characters used to communicate a DCC
	//		command to/from the firmware in textual form.
	//
	static const byte	maximum_command		= 6;

	//
	//	Define the maximum number of bytes that can be accepted as
	//	a complete input command.
	//
	static const byte	maximum_input		= 32;

	//
	//	Define a maximum number of characters that are required to
	//	formulate the host reply to a command being received successfully.
	//
	static const byte	maximum_output		= 16;

	//
	//	In contradiction to the above maxima, the follow is required
	//	specifically for the support of the EEPROM manipulation
	//	commands where they reply with long (too long!) human
	//	names for the constants.  The above space simply is not
	//	sufficient to encompass these test strings.
	//
	static const byte	eeprom_maximum_output	= 48;

#if defined( ENABLE_DCC_DELAY_REPORT ) || defined( ENABLE_DCC_SYNCHRONISATION )
	//
	//	DCC Delay reporting and tuning
	//	------------------------------
	//
	//	If either of the delay reporting or delay tuning code has
	//	been included then the following constant definition is
	//	created to capture the extent of the data window used
	//	to create the average delay information.
	//
#ifdef DCC_AVERAGE_DELAYS
	static const byte	delays			= DCC_AVERAGE_DELAYS;
#else
	static const byte	delays			= 16;
#endif

#if defined( ENABLE_DCC_SYNCHRONISATION )
	//
	//	Define a ceiling for the maxiumum amount of time that the
	//	DCC IRQ routine will synchronise to.
	//
	static constexpr byte	max_sync		= timer_digit_1_cycles / 2;
#endif
#endif

	//
	//	Declare the number of milliseconds between the DCC system
	//	trimming the IRQ synchronisation value.
	//
#ifdef DCC_RECALIBRATION_PERIOD
	static const word dcc_recalibration_period	= DCC_RECALIBRATION_PERIOD;
#else
	static const word dcc_recalibration_period	= 1000;
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
	//	The result of the above example is that 8 bytes are
	//	required for each data byte in the packet, 8 for the
	//	checksum, 1 for the end of packet, 2 for the pre-amble
	//	and 1 for the post-amble.
	//
#ifdef BIT_TRANSITIONS
	static const byte	bit_transitions = BIT_TRANSITIONS;
#else
	static constexpr byte	bit_transitions = (( maximum_command + 1 ) * 8 + 3 ) / 2;
#endif

	//
	//	Define maximum bit iterations per byte of the bit transition array.
	//
	//	It is possible to prove that this figure can never be reached as this
	//	would imply a a series of 28 byte containing just 0 bits which (apart
	//	from being an invalid DCC command, is over 4 times longer than the
	//	longest DCC command this code will handle.
	//
	//	This value is applied inside verification assert statements.
	//	In a real sense this is pointless given that the count is held
	//	in a byte value.
	//
	static const byte	maximum_bit_iterations	= 255;

	//
	//	Define the number of "1"s transmitted by the firmware
	//	forming the "preamble" for the DCC packet itself.
	//
	//	The DCC standard specifies a minimum of 14 for normal
	//	commands and 20 for programming commands.
	//
	//	These are the short and long preambles.
	//
	static const byte	short_preamble		= 14;
	static const byte	long_preamble		= 20;

	//
	//	Define an enumeration that captures the possible times when
	//	a command reply is desired.
	//
	//	reply_none	No reply from the system is expected.
	//
	//	reply_at_start	A reply is expected as the *last* DCC packet
	//			is loaded and transmission begins.
	//
	//	reply_at_end	A reply is expected only when the transmission
	//			of the DCC packet has been completed and the
	//			buffer is released.
	//
	enum reply_time {
		reply_none	= 0,
		reply_at_start,
		reply_at_end
	};

private:
	//
	//	Pending Transmissions
	//	---------------------
	//
	//	Define the data structure used to hold a single pending
	//	DCC packet.  The fields defined are:
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
	struct pending_packet {
		byte		preamble,
				postamble,
				duration,
				len,
				command[ maximum_command ];
		pending_packet	*next;
	};
	
	//
	//	Define the the pending packet records and the head of the free
	//	records list.
	//
	pending_packet		*_free_packets;


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
	//	state_run	ISR		Continuous packet	state_run
	//					transmission
	//					(duration is zero)
	//
	//	state_run	ISR		Limited packet		state_run
	//					transmission
	//					(--duration > 0 )
	//
	//	state_run	ISR		Limited packet		state_load
	//					transmission
	//					(--duration == 0 )
	//
	//	state_reload	ISR		Synchronised load	state_load
	//					initiated by IO code
	//					on a state_run buffer
	//
	//	state_load	Manager		Pending pointer not	state_run
	//					NULL, load next bit
	//					sequence from record
	//					and drop pending record
	//
	//	state_load	Manager		Pending pointer is	state_empty
	//					NULL.
	//
	//	state_empty	System		Create a list of	state_load
	//					pending records and
	//					attach to a new buffer
	//
	//	state_run	System		Create a list of	state_reload
	//					pending records and
	//					attach to a running
	//					buffer (how engines
	//					change speed).
	//
	enum buffer_state : byte {		// One line summary of meaning.
		state_empty	= 0,		// Record empty available for use.
		state_fixed,			// This is the fixed record.
		state_load,			// Current transmission completed, load next pending record.
		state_run,			// Current transmission in progress.
		state_reload			// Terminate current transmission (ISR to flip to state_load)
	};
	
	//
	//	Define a Transmission Buffer which contains the following
	//	elements:
	//
	//	state		The current state of this buffer.  Primary method
	//			synchronise activities between the interrupt driven
	//			signal generator code and the table management code.
	//
	//	Live Transmission fields:
	//	-------------------------
	//
	//	target		The DCC ID being targeted by the buffer:
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
	//	reply_when	Define how the system should manage/generate a reply
	//			to the host computer.
	//
	//	contains	The (EOS terminated) reply string which should
	//			form the reply sent (if requested).
	//
	//	Link to the next buffer, circular fashion:
	//	------------------------------------------
	//
	//	next		The address of the next transmission buffer in
	//			the loop
	//
	struct trans_buffer {
		//
		//	The current state of this buffer.
		//
		buffer_state	state;
		//
		//	Live Transmission fields:
		//	-------------------------
		//
		//
		//	The target ID of the packet, and an indication
		//	of what the target is and some "data" giving
		//	internal details of that action.
		//
		word		target;
		bool		mobile;
		word		action;
		
		//
		//	The duration countdown (if non-zero).  An initial value
		//	of zero indicates no countdown meaning the packet is transmitted
		//	indefinitely.
		//
		byte		duration;
		//
		//	The bit pattern to transmit.  This is only filled in
		//	from the command field by the buffer management code
		//	so that it can be synchronised with operation of the
		//	interrupt routine.
		//
		//	Bit transitions are zero byte terminated, no length
		//	need be maintained.
		//
		byte		bits[ bit_transitions ];

		//
		//	Pending Transmission Fields:
		//	----------------------------
		//
		//	Address of the next pending DCC command to send, NULL
		//	if nothing to send after this bit pattern.
		//
		pending_packet	*pending;

		//
		//	Confirmation reply data.
		//	------------------------
		//
		//	When is a reply required and what does that reply contain?  This
		//	is applied only at the end of a series of pending records (i.e.
		//	when pending is NULL).
		//
		reply_time	reply_when;
		char		reply[ maximum_output ];

		//
		//	Buffer linkage.
		//	---------------
		//
		//	Finally, the link to the next buffer and the
		//	link back to the previous buffer.
		//
		trans_buffer	*next,
				**prev;
	};

	//
	//	Define the transmission buffers to be used, and the
	//	pointers into it for various purposes.
	//
	//	To ensure that the circular list of active buffers is
	//	never empty (and so that the cide which deals with the
	//	circular list *never* has to deal with the empty list
	//	scenario) the DCC module will create a *special* static
	//	buffer which always sends a single idle packet.
	//
	//	The constructor and initialiser will ensure that when the
	//	DCC signal is started this record will form the *only*
	//	record in the circular list.
	//
	trans_buffer		*_current,	// Current buffer being transmitted.
				*_circle,	// A fixed point to the fixed record.
				*_free_trans;	// Free transmission buffers.
	//
	//	These two lists are used to hand off records between the
	//	ISR and the manager code so they need to be set as volatile.
	//
	volatile trans_buffer	*_run,		// A list of records that the ISR
						// needs to run.
				*_manage,	// Next buffer to be managed.
				*_scan;		// The next buffer to be scanned
						// for its action data.
	//
	//	Signal generation variables, not volatile as they are
	//	only modified by the interrupt routine.
	//
	byte			_left,
				*_bit_string;
	bool			_side,
				_one;

	//
	//	For statistical purposes the following variables are
	//	maintained:
	//
	byte			_free_buffers;
	word			_packets_sent;
	
#if defined( ENABLE_DCC_DELAY_REPORT ) || defined( ENABLE_DCC_SYNCHRONISATION )
	Average< delays, byte >	_delay;
#endif

#if defined( ENABLE_DCC_SYNCHRONISATION )
	byte			_irq_sync;
#endif

	//
	//	Declare the task manager handle for managment
	//
	static const byte management_process = 1;

	//
	//	Declare the link back to the "Manager" task.  The
	//	ISR will "release()" _manager for every record in
	//	the transmission buffers that needs management
	//	attention.
	//
	//	This system creates a simple and effective asynchronous
	//	supplier/consumer between the two systems.
	//
	Signal			_manager;

#ifdef ENABLE_DCC_SYNCHRONISATION
	//
	//	Declare the task manager handle for recalibration
	//
	static const byte recalibrate_process = 2;

	//
	//	This is the signal used to schedule an IRQ synchonisation
	//	recalibration.
	//
	Signal			_recalibrate;
#endif
	
	//
	//	Private support routines.
	//	-------------------------
	//

	//
	//	Copy a DCC command to a new location and append the parity data.
	//	Returns length of data in the target location.
	//
	byte copy_with_parity( byte *dest, byte *src, byte len );

	//
	//	Define a routine to release one (one=true) or all (one=false) pending packets in a list.
	//
	pending_packet *release_pending_recs( pending_packet *head, bool one );

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
	bool pack_command( byte *cmd, byte clen, byte preamble, byte postamble, byte *buf );

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
	byte compose_motion_packet( byte *command, word adrs, byte speed, byte dir );

	//
	//	Create an accessory modification packet.  Return number of bytes
	//	used by the command.
	//
	byte compose_accessory_change( byte *command, word adrs, byte subadrs, byte state );

	//
	//	Create a Function set/reset command, return number of bytes used.
	//
	//	Until I can find a more focused mechanism for modifying the functions
	//	this routine has to be this way.  I guess a smarter data driven approach
	//	could be used to generate the same output, but at least this is clear.
	//
	byte compose_function_change( byte *command, word adrs, byte func, bool on );

	//
	//	Create one part of the bulk function setting command.  Which part
	//	is set by the range argument.
	//
	byte compose_function_block( byte *command, word adrs, byte *state, byte fn[ DCC_Constant::bit_map_array ]);

	//
	//	Request an empty buffer to be set aside for building
	//	a new transmission activity.
	//
	trans_buffer *acquire_buffer( word target, bool mobile, word action, bool overwrite );

	//
	//	This is the routine which adds a pending DCC packet to the
	//	pending queue on the buffer being constructed.  Returns
	//	true if a new pending record has been created or false
	//	if this was not possible.
	//
	bool extend_buffer( trans_buffer *rec, byte duration, byte preamble, byte postamble, byte *cmd, byte len );

	//
	//	Complete a buffer and prepare it for transmission.
	//
	bool complete_buffer( trans_buffer *rec );

	//
	//	Cancel construction of a new transmission buffer.
	//
	void cancel_buffer( trans_buffer *rec );

	//
	//	Declare some static routines used by the public action
	//	value routines.
	//
	//	These routines are used to encode three pieces of information
	//	into a 16 bit word value:
	//
	//	o	A 4 bit opcode (bits 8-11)
	//	o	A boolean flag (bit 7)
	//	o	A 7 bit unsigned value (bits 0-6)
	//
	static inline word create_action( byte op, byte flag, byte value ) {
		return((((word)op & 0x000f ) << 8 )|(((word)flag & 0x0001 ) << 7 )|((word)value & 0x007f ));
	}
	static inline byte action_op( word action ) {
		return((byte)(( action >> 8 ) & 0x000f ));
	}
	static inline byte action_flag( word action ) {
		return((byte)(( action >> 7 ) & 0x0001 ));
	}
	static inline byte action_value( word action ) {
		return((byte)( action & 0x007f ));
	}


public:
	//
	//	Constructor
	//
	DCC( void );

	//
	//	Call this routine to get the object going.  After
	//	this call is completed the dcc driver object *will*
	//	start getting calls to toggle the direction pins to
	//	generate the output signal.
	//
	//	clock out is a pin specifically used export the internal
	//	DCC clock from the ISR routine to enable basic operational
	//	state verification (ie, is there even a clock working,
	//	and what speed is it?).
	//
	void initialise( void );

	//
	//	This routine is the "manager" of the DCC packets being
	//	transmitted.  This is called asynchronously from the
	//	irq() routine by releasing a resource on the "_manager"
	//	signal.
	//
	//	The Signal object manages all aspects of of concurrency
	//	where the ISR might release multiple transmission records
	//	before the manager routine is scheduled.
	//
	virtual void process( byte handle );

	//
	//	Define the Interrupt Service Routine, where we do the work.  This is
	//	a static routine as it shuold only ever access the static variables
	//	defined in the class.
	//
	void irq( void );
	
	//
	//	API for recovering and decoding data from the active
	//	buffers.
	//
	void reset_scan( void );
	bool scan_next( word *target, bool *mobile, word *action );
	
	//
	//	action values can be created and decoded using the following
	//	static inline functions.
	//
	static inline word speed_and_dir( byte speed, byte direction ) {
		return( create_action( 1, direction, speed ));
	}
	//
	static inline bool is_speed_and_dir( word action ) {
		return( action_op( action ) == 1 );
	}
	static inline byte get_speed( word action ) {
		return( action_value( action ));
	}
	static inline byte get_dir( word action ) {
		return( action_flag( action ));
	}
	//
	static inline word func_and_state( byte func, byte state ) {
		return( create_action( 2, state, func ));
	}
	//
	static inline bool is_func_and_state( word action ) {
		return( action_op( action ) == 2 );
	}
	static inline byte get_func( word action ) {
		return( action_value( action ));
	}
	static inline byte get_state( word action ) {
		return( action_flag( action ));
	}
	//
	static inline word accessory_state( byte state ) {
		return( create_action( 3, state, 0 ));
	}
	//
	static inline bool is_accessory_state( word action ) {
		return( action_op( action ) == 3 );
	}
	static inline byte get_accessory_state( word action ) {
		return( action_flag( action ));
	}

	//
	//	API for sending commands out through DCC.  *ALL* paramters
	//	are the DCC specification values.  Note this especially
	//	applies to the speeds.
	//
	bool mobile_command( word target, byte speed, byte direction, Buffer_API *reply = NIL( Buffer_API ));
	bool accessory_command( word target, byte state, Buffer_API *reply = NIL( Buffer_API ));
	bool function_command( word target, byte func, byte state, Buffer_API *reply = NIL( Buffer_API ));
	bool state_command( word target, byte speed, byte dir, byte fn[ DCC_Constant::bit_map_array ], Buffer_API *reply = NIL( Buffer_API ));

	//
	//	The memory recovery API
	//	=======================
	//
	virtual bool release_memory( void );

	//
	//	Routines used to access statistical analysis
	//
	byte free_buffers( void );
	word packets_sent( void );
	
#if defined( ENABLE_DCC_DELAY_REPORT ) || defined( ENABLE_DCC_SYNCHRONISATION )
	byte irq_delay( void );
#endif

#if defined( ENABLE_DCC_SYNCHRONISATION )
	byte irq_sync( void );
#endif

};

//
//	Declare the DCC Generator.
//
extern DCC dcc_generator;

#endif

//
//	EOF
//

//
//	Clock
//	=====
//
//	The module provides alternative mechanism to the built-in
//	millis() and micros() system calls, by providing a system
//	based on relative timing over absolute timing so that "roll
//	round" errors/issues are avoided.
//
//	This also removes the necessity for many, many unsigned 32 bit
//	comparisons, and should be much faster.
//

#ifndef _CLOCK_H_
#define _CLOCK_H_

//
//	We will need to Environment and Critical modules to define
//	the clock code.
//
#include "Environment.h"
#include "Parameters.h"
#include "Configuration.h"
#include "Task_Entry.h"
#include "Signal.h"

//
//	Define (if not already defined) the maximum number of clock
//	events which the system will handle.
//
#ifndef CLOCK_EVENTS
#define CLOCK_EVENTS	12
#endif

//
//	Define our base clock tick dependent on the underlying hardware.
//
#if defined( __AVR_ATmega328__ )| defined( __AVR_ATmega328P__ )| defined( __AVR_ATmega328PB__ )
//
//	Nano or Uno R3 configuration
//	============================
//

//
//	Map Clock symbols onto target Timer hardware
//
#define CLK_COUNTER_BITS	8
#define CLK_MAX_COUNTER		250
#define CLK_TCCRnA		TCCR0A
#define CLK_TCCRnB		TCCR0B
#define CLK_TIMERn_COMPA_vect	TIMER0_COMPA_vect
#define CLK_TCNTn		TCNT0
#define CLK_OCRnA		OCR0A
#define CLK_WGMn1		WGM01
#define CLK_CSn0		CS00
#define CLK_CSn1		CS01
#define CLK_CSn2		CS02
#define CLK_TIMSKn		TIMSK0
#define CLK_OCIEnA		OCIE0A


#elif defined( __AVR_ATmega2560__ )
//
//	Mega 2560 configuration
//	=======================
//

//
//	Map Clock symbols onto target Timer hardware
//
#define CLK_COUNTER_BITS	8
#define CLK_MAX_COUNTER		250
#define CLK_TCCRnA		TCCR0A
#define CLK_TCCRnB		TCCR0B
#define CLK_TIMERn_COMPA_vect	TIMER0_COMPA_vect
#define CLK_TCNTn		TCNT0
#define CLK_OCRnA		OCR0A
#define CLK_WGMn1		WGM01
#define CLK_CSn0		CS00
#define CLK_CSn1		CS01
#define CLK_CSn2		CS02
#define CLK_TIMSKn		TIMSK0
#define CLK_OCIEnA		OCIE0A

#else

#error "Clock timer has not been configured for this board"

#endif

//
//	From the above declare aliases for the clock hardware counter
//	register and the clock hardware comparison register.
//
#define CLK_COUNTER_REG		CLK_TCNTn
#define CLK_COMPARE_REG		CLK_OCRnA


//
//	Define the software interface to the clock routines.  These
//	routines facilitate the setting of boolean flags "at a set
//	distance" into the future.  These are not intended for "cycle
//	accurate" timed events, but for the sequencing of software
//	based activities which are paced through time in a controlled
//	fashion.
//
class Clock : public Task_Entry {
public:
	//
	//	Timing the Clock
	//	================
	//
	//	The original version of this module used a regular (very regular)
	//	interrupt to manage a chronologically sorted list of pending
	//	Signals awaiting a call to release them.
	//
	//	New timer requests were simply inserted into the queue (or
	//	appended as appropiate) meaning that the sum of all the events
	//	pending times plus its own pending time gave the time left
	//	before that events time.
	//
	//	This worked, but would seem to have had extracted too much cost
	//	from the MCU, leaving little time for other events/interrupts
	//	to execute in a timely manner.
	//
	//	This new version will generate an interrupt only when required,
	//	making the insertion of timed events at the head of the queue
	//	now much more tricky.
	//
	//	We start with the two options : one we cannot change and
	//	one we can:  MCU clock speed and clock divider.
	//
	//	The MCU clock, as an input to the timer, can be divided
	//	by the following factors:
	//
	//		1, 8, 64, 256, or 1024
	//
	//	For our purposes we would like the clock/divider combination
	//	to provide a "useful" balance between too often (giving
	//	higher accuracy but too little range) and too infrequently
	//	(giving poor accuracy but ample range).
	//
	//	The target for the clock module is to time a whole second
	//	accurately, but with enough accuracy that driver pauses
	//	in the 10us to 50us range can be reasonably approximated.
	//
	//	This therefore leads to the first conclusion:  An 8-bit
	//	timer is not able to achieve this.  With only ~250 steps
	//	this would mean that the smallest interval that could be
	//	tracked being 4ms (1/250th of a second).
	//
	//	The solution required will need to be clever.  The system
	//	does not have to be able to time a whole second; the
	//	introduction of "synthetic" events matching the maximum
	//	countable period of time will allow the module to "step"
	//	through time in countable pieces.
	//
	//	So, what is possible with the hardware available?
	//
	//	The following table show what can be done with the CPU
	//	clock and dividers available (value are in micro seconds
	//	1/1,000,000th of a seocnd):
	//
	//	Clock		8Mhz	16MHz	20Mhz
	//	Divider		----	-----	-----
	//		1	0.125	0.0625	0.05
	//		8	1	0.5	0.4
	//		64	8	4	3.2
	//		256*	32	16	12.8
	//		1024	128	64	51.2
	//
	//	This table extrapolates these times by 250 to understand
	//	the maximum countable period by combination (times
	//	converted from micro seconds to milliseconds, truncated):
	//
	//	Clock		8Mhz	16MHz	20Mhz
	//	Divider		----	-----	-----
	//		1	 0.03	 0.01	0.01
	//		8	 0.25	 0.12	0.10
	//		64	 2.00	 1.00	0.80
	//		256*	 8.00	 4.00	3.20
	//		1024	32.00	16.00	12.8
	//
	//	It is noteable that the "optimal" divider is the same
	//	across all clock speeds (see entries marked with an
	//	asterix, '*').  This is mostly because the granularity
	//	of the divider is significantly larger than the
	//	graduation between processor speeds.
	//
	//	The following, conditional, sections of code will pull
	//	the necessary calculations together from the above
	//	tables and the compile time F_CPU parameter.
	//
	//	Even though they are all identical.
	//
	//	It is worth noting that, although the range parameter
	//	becomes shorter as the clock speed increases, it always
	//	represents the same number of clock cycles and hense
	//	executed instructions.  Therefore the smaller periods
	//	do not represent a heavier load on the MCU.
	//

#if F_CPU == 8000000
	//
	//	8 MHz: Step 32us, Range 8ms.
	//	
	static const word	clock_divider = 256;

#elif F_CPU == 16000000
	//
	//	16 MHz: Step 16us, Range 4ms.
	//	
	static const word	clock_divider = 256;

#elif F_CPU == 20000000
	//
	//	20 MHz: Step 12.8us, Range 3.2ms.
	//	
	static const word	clock_divider = 256;

#else

#error "CPU/MCU Clock speed not recognised."

#endif

	//
	//	Setting the Output Compare Register value.
	//
	//	This is not necessarily that obvious.  The Atmel docs
	//	give the following, backwards, formula for working out
	//	the frequency of an interrupt based on the value in the
	//	output compare register:
	//
	//					CPU_freq
	//		Output_freq = -----------------------------------
	//				2 x N x ( 1 + Compare_Register )
	//
	//	The '2' in the bottom does make immediate sense until
	//	you realise that 'Output_freq' represents the complete
	//	output wave form (both the "up" and "down" sections)
	//	which takes 2 compare matches to create.  Therefore to
	//	get the frequency of the compare matches you remove the
	//	'2'.  Thus the above is re-written as:
	//
	//					CPU_freq
	//		Compare_freq = ------------------------------
	//				N x ( 1 + Compare_Register )
	//
	//	Now, divide both side by 'CPU_freq', and cancel out:
	//
	//		Compare_freq 			CPU_freq
	//		------------- = ---------------------------------------------
	//		CPU_freq	( N x ( 1 + Compare_Register )) * CPU_freq
	//
	//	thus:
	//
	//		Compare_freq 			1
	//		------------- = ------------------------------
	//		CPU_freq	N x ( 1 + Compare_Register )
	//
	//	Multiply both side by N and cancel out:
	//
	//		N x Compare_freq		N
	//		---------------- = ------------------------------
	//		CPU_freq	     N x ( 1 + Compare_Register )
	//
	//	thus:
	//
	//		N x Compare_freq		1
	//		---------------- = ------------------------
	//		CPU_freq	     1 + Compare_Register
	//
	//	Finally we invert the formula and move the '1' across:
	//
	//		CPU_freq	
	//		---------------- =  1 + Compare_Register
	//		N x Compare_freq
	//
	//	Thus:
	//
	//		CPU_freq
	//		---------------- - 1 = Compare_Register
	//		N x Compare_freq
	//
	//	This final version gives us the actual value that needs
	//	to be placed into the compare register to obtains a
	//	specific *frequency*.  For the purposes of this clock
	//	code all calculations are made in time units, not basic
	//	frequency.  Therefore, noting the following relationship:
	//
	//			  1			   1
	//		freq = -------	  or	period = ------
	//			period			  freq
	//
	//	Be careful to track units; the basic units are Hz and
	//	Seconds.
	//
	//	Substituting the second equation above into the compare
	//	register equation we get:
	//
	//				    CPU_freq x period
	//		Compare_Register = -------------------  -1
	//					    N
	//
	//	It would seem that the above is a terrific labour to get
	//	to the obvious end: a simply way to convert time into
	//	known counts.
	//

	//
	//	Define, in microseconds, the duration of a "tick", the
	//	indivisible unit of time which this module count in.
	//
	//	clock_tick_hz		Convert the base CPU frequency
	//				into the frequency of the counter.
	//
	//	However, converting the counted frequency into the
	//	obvious microseconds value (as an integer) would be
	//	subject to rounding issues.
	//
	//	So we actually convert "clock_tick_hz" into 10ths of a
	//	microsecond to create an extra digit of accuracy.
	//
	//	clock_tick_10ths	Counted frequency as 10ths of a
	//				microsecond.
	//
	static const dword	milliseconds		= 1000;
	static const dword	microseconds		= 1000000;
	static constexpr dword	clock_tick_hz		= (dword)F_CPU / (dword)clock_divider;
	static constexpr word	clock_tick_10ths	= ( 10 *  microseconds ) / clock_tick_hz;

	//
	//	Maximum consecutive clock ticks which can be counted.
	//
	static const byte	maximum_count		= CLK_MAX_COUNTER;
	
	//
	//	The following two macros convert firmware expressed times
	//	in Milliseconds or Microseconds into a number of counted
	//	clock ticks (as a word value).
	//
	//	Working on the premise that microsecond delays are for
	//	the purpose of timing delays with hardware, then the calculation
	//	will always round up to the next clock_tick; A microsecond
	//	duration should be read as "not less than X microseconds".
	//
	//	The milliseconds calculation will round down and should
	//	therefore be read as "not mroe than X milliseconds".
	//
	//	It is advised that these macros are only ever used with
	//	constant values that can be pre-calculated at compile
	//	time (or the calculation could overwhelm the delays
	//	requested!)
	//
#define MSECS(t)	((((dword)(t))*10000)/Clock::clock_tick_10ths)
#define USECS(t)	((((dword)(t))*10+(Clock::clock_tick_10ths-1))/Clock::clock_tick_10ths)


private:
	//
	//	How many events are we prepared to work with?
	//
	static const byte	clock_events = CLOCK_EVENTS;
	
	//
	//	Our task managing structure.
	//
	struct clock_event {
		word		left,		// Ticks left to event.
				repeats;	// Repeating value if non-zero.
		Signal		*gate;		// The signal to release.
		clock_event	*next;
	};

	//
	//	Our task managing pointers and the array of records
	//	which we have to manage.
	//
	volatile clock_event	*_active,
				*_free;
	clock_event		_events[ clock_events ];
	
	//
	//	Declare the Signal used as a link between the interrupt
	//	and the routine schedulling the work.
	//
	Signal			_irq;

	//
	//	Insert an event into the active list according to the
	//	number of ticks specified in the left field.
	//
	void insert( clock_event *ptr );
	
	//
	//	Stop the timer running until start time has been called again.
	//
	void inline stop_timer( void ) {
		//
		//	Bring the timer to a halt.
		//
		//	Disable timer compare interrupt
		//
		CLK_TIMSKn &= ~bit( CLK_OCIEnA );
		//
		//	Reset counter and compare registers
		//
		CLK_COUNTER_REG = 0;
		CLK_COMPARE_REG = maximum_count;
	}
		
	//
	//	Restart the timer for a fixed number of counts.
	//
	void inline start_timer( byte delay ){
		//
		//	Set compare match register to
		//	generate the correct tick duration.
		//
		CLK_COMPARE_REG = delay;
		CLK_COUNTER_REG = 0;
		//
		//	Enable timer compare interrupt
		//
		CLK_TIMSKn |= bit( CLK_OCIEnA );
	}

public:
	//
	//	Object creation and initialisation routines
	//
	Clock( void );
	void initialise( void );

	//
	//	This is the interface into the clock allowing new
	//	"one time" or "repeating" events to be created.
	//
	bool delay_event( word ticks, Signal *gate, bool repeating );

	//
	//	An "in-line" delay of a specified number of ticks.
	//
	//	This call will automatically return if it is running
	//	inside an interrupt or critical code section without
	//	any pause being experienced.
	//
	void inline_delay( word ticks );

	//
	//	provide inline functions to convert time to time steps.
	//
	static inline word msecs( word ms ) {
		return( MSECS( ms ));
	}
	static inline word usecs( word us ) {
		return( USECS( us ));
	}
	
	//
	//	Routine called to record an interrupt event.
	//
	void irq( void );
	
	//
	//	This is the service routine, called as a result of the
	//	clock tick interrupt.
	//
	virtual void process( byte handle );

};


//
//	Finally we define the actual clock instance.
//
extern Clock	event_timer;


#endif

//
//	EOF
//

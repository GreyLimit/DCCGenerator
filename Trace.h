///
///	trace.h		A file where control of code and data tracing
///			options are consolidated.
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

#ifndef _TRACE_H_
#define _TRACE_H_

//
//	Define any number of the following macros (in Parameters.h)
//	to facilitate the control of trace output to the console.
//
//	ENABLE_COUNT_INTERRUPTS	Display the number of interupts per second.
//	ENABLE_DCC_DELAY_REPORT	Display the "delay" between the DCC interrrupt
//				trigger and ISR being called.
//
//	ENABLE_DCC_SYNCHRONISATION
//				Enable code specifically included to fine
//				tune the execution of the DCC interrupt
//				handler (at the expense of wasting CPU
//				cycles inside interrupt handler).
//
//	ENABLE_TRACE_ADC	Trace the analogue to digital conversions.
//	ENABLE_TRACE_CLOCK	Trace activities relating to the RTC.
//	ENABLE_TRACE_CONSOLE	Trace some details in the console (limited).
//	ENABLE_TRACE_DCC	Trace activities in the DCC generator.
//	ENABLE_TRACE_DISTRICT	Trace actions upon a district.
//	ENABLE_TRACE_DRIVER	Trace actions within the DCC output driver.
//	ENABLE_TRACE_FBUFFER	Trace action within the frame buffer.
//	ENABLE_TRACE_FUNCTION	Trace action in the function cache.
//	ENABLE_TRACE_HCI	Trace actions in the human computer interface.
//	ENABLE_TRACE_HEAP	Trace action in the memory heap module.
//	ENABLE_TRACE_KEYPAD	Trace actions with the keypad.
//	ENABLE_TRACE_LCD	Trace the screen buffer actions.
//	ENABLE_TRACE_SIGNAL	Trace activities on signals.
//	ENABLE_TRACE_SPI	Trace activities on the SPI device.
//	ENABLE_TRACE_STATS	Trace activities on the statistics module.
//	ENABLE_TRACE_ROTARY	Trace actions on the rotary dial.
//	ENABLE_TRACE_TASK	Trace task/event manager.
//	ENABLE_TRACE_TOD	Trace actions in the Time Of Day module.
//	ENABLE_TRACE_TWI	Trace Data IO across the IIC bus.
//
//	ENABLE_STACK_TRACE	Enable the STACK_TRACE macro analysis
//				of the call stack.
//
//	Definition of any of these will automatically disable the
//	output of "proper" console traffic from the firmware to
//	avoid "mixing up" the output and rendering any trace output
//	undecipherable.
//
//	If any of the above definition are made then the symbol
//	DEBUGGING_ENABLED will also be defined and can be used to
//	control inclusion of code specifically required for debug
//	purposes.
//

//
//	The macros definitions for each traceable unit.
//
#ifdef ENABLE_TRACE_TWI
#define TRACE_TWI(c)	c
#ifndef DEBUGGING_ENABLED
#define DEBUGGING_ENABLED
#endif
#else
#define TRACE_TWI(c)
#endif

#ifdef ENABLE_TRACE_HCI
#define TRACE_HCI(c)	c
#ifndef DEBUGGING_ENABLED
#define DEBUGGING_ENABLED
#endif
#else
#define TRACE_HCI(c)
#endif

#ifdef ENABLE_TRACE_HEAP
#define TRACE_HEAP(c)	c
#ifndef DEBUGGING_ENABLED
#define DEBUGGING_ENABLED
#endif
#else
#define TRACE_HEAP(c)
#endif

#ifdef ENABLE_TRACE_TOD
#define TRACE_TOD(c)	c
#ifndef DEBUGGING_ENABLED
#define DEBUGGING_ENABLED
#endif
#else
#define TRACE_TOD(c)
#endif

#ifdef ENABLE_TRACE_ROTARY
#define TRACE_ROTARY(c)	c
#ifndef DEBUGGING_ENABLED
#define DEBUGGING_ENABLED
#endif
#else
#define TRACE_ROTARY(c)
#endif

#ifdef ENABLE_TRACE_SPI
#define TRACE_SPI(c)	c
#ifndef DEBUGGING_ENABLED
#define DEBUGGING_ENABLED
#endif
#else
#define TRACE_SPI(c)
#endif

#ifdef ENABLE_TRACE_STATS
#define TRACE_STATS(c)	c
#ifndef DEBUGGING_ENABLED
#define DEBUGGING_ENABLED
#endif
#else
#define TRACE_STATS(c)
#endif

#ifdef ENABLE_TRACE_DCC
#define TRACE_DCC(c)	c
#ifndef DEBUGGING_ENABLED
#define DEBUGGING_ENABLED
#endif
#else
#define TRACE_DCC(c)
#endif

#ifdef ENABLE_TRACE_CONSOLE
#define TRACE_CONSOLE(c)	c
#ifndef DEBUGGING_ENABLED
#define DEBUGGING_ENABLED
#endif
#else
#define TRACE_CONSOLE(c)
#endif

#ifdef ENABLE_TRACE_KEYPAD
#define TRACE_KEYPAD(c)	c
#ifndef DEBUGGING_ENABLED
#define DEBUGGING_ENABLED
#endif
#else
#define TRACE_KEYPAD(c)
#endif

#ifdef ENABLE_TRACE_LCD
#define TRACE_LCD(c)	c
#ifndef DEBUGGING_ENABLED
#define DEBUGGING_ENABLED
#endif
#else
#define TRACE_LCD(c)
#endif

#ifdef ENABLE_TRACE_TASK
#define TRACE_TASK(c)	c
#ifndef DEBUGGING_ENABLED
#define DEBUGGING_ENABLED
#endif
#else
#define TRACE_TASK(c)
#endif

#ifdef ENABLE_TRACE_ADC
#define TRACE_ADC(c)	c
#ifndef DEBUGGING_ENABLED
#define DEBUGGING_ENABLED
#endif
#else
#define TRACE_ADC(c)
#endif

#ifdef ENABLE_TRACE_CLOCK
#define TRACE_CLOCK(c)	c
#ifndef DEBUGGING_ENABLED
#define DEBUGGING_ENABLED
#endif
#else
#define TRACE_CLOCK(c)
#endif

#ifdef ENABLE_TRACE_SIGNAL
#define TRACE_SIGNAL(c)	c
#ifndef DEBUGGING_ENABLED
#define DEBUGGING_ENABLED
#endif
#else
#define TRACE_SIGNAL(c)
#endif

#ifdef ENABLE_TRACE_DISTRICT
#define TRACE_DISTRICT(c)	c
#ifndef DEBUGGING_ENABLED
#define DEBUGGING_ENABLED
#endif
#else
#define TRACE_DISTRICT(c)
#endif

#ifdef ENABLE_TRACE_DRIVER
#define TRACE_DRIVER(c)	c
#ifndef DEBUGGING_ENABLED
#define DEBUGGING_ENABLED
#endif
#else
#define TRACE_DRIVER(c)
#endif

#ifdef ENABLE_TRACE_FBUFFER
#define TRACE_FBUFFER(c)	c
#ifndef DEBUGGING_ENABLED
#define DEBUGGING_ENABLED
#endif
#else
#define TRACE_FBUFFER(c)
#endif

#ifdef ENABLE_TRACE_FUNCTION
#define TRACE_FUNCTION(c)	c
#ifndef DEBUGGING_ENABLED
#define DEBUGGING_ENABLED
#endif
#else
#define TRACE_FUNCTION(c)
#endif


//
//	Configure/define the stack trace facility.
//
//	Individual modules can disable this explicitly through the
//	definition of the DISABLE_STACK_TRACE macro prior to the
//	inclusion of this file.
//
#ifdef ENABLE_STACK_TRACE

//
//	We will need this.
//
#include "Byte_Queue_API.h"

//
//	We declare our stack frame class.
//
class stack_frame {
public:
	//
	//	Our name and our link down.
	//
	const char	*_name;
	stack_frame	*_down;

	//
	//	Keeping a depth counter
	//
	static byte	_depth;
	
	//
	//	Are we displaying the stack frames in real time?
	//
	static bool	_display;

	//
	//	Cons and Destruct routines which do the work
	//
	stack_frame( void );
	stack_frame( const char *func );
	~stack_frame();

	//
	//	Display the stack frame!
	//
	void show( Byte_Queue_API *to );
	
	//
	//	Display the caller of this routine.
	//
	void caller( Byte_Queue_API *to );
	
	//
	//	Set the display flag
	//
	void display( bool on );
};

//
//	Declare the GLOBAL pointer to the top stack frame.
//
extern stack_frame	*top_stack_frame;

#if !defined( DISABLE_STACK_TRACE )
//
//	When we *are* following the stack we declare a variable
//	of type stack_frame that we use to keep track of stack
//	frames on the .. erm .. stack.
//
#define STACK_TRACE(f)		static const char function_name[] PROGMEM = f;\
				stack_frame _stack_frame_( function_name )

#define STACK_DUMP(t)		top_stack_frame->show(t)

#define STACK_DISPLAY(y)	top_stack_frame->display(y)

#define STACK_CALLER(t)		top_stack_frame->caller(t)

#ifndef DEBUGGING_ENABLED
#define DEBUGGING_ENABLED
#endif

#else
//
//	Generally the stack trace facility is enabled, but specifically
//	in the current target file it is disabled.
//
#define	STACK_TRACE(f)
#define STACK_DUMP(t)
#define STACK_DISPLAY(y)
#define STACK_CALLER(t)

#endif

#else

//
//	When *not* following the stack frame then we
//	define the STACK_TRACE as empty.
//
#define	STACK_TRACE(f)
#define STACK_DUMP(t)
#define STACK_DISPLAY(y)
#define STACK_CALLER(t)

#endif


//
//	The following macros enable the source code to modify itself
//	in a syntactically clean method based on the leve of debugging
//	selected.
//
//	DEBUGGING_ENABLED	Defined if *any* debugging is enabled.
//
//	DEBUGGING_OPTION(y,n)	Expands to the 'y' option if debugging
//				is enabled or the 'n' option otherwise.
//	
#ifdef DEBUGGING_ENABLED
#define DEBUGGING_OPTION(y,n)		y
#else
#define DEBUGGING_OPTION(y,n)		n
#endif


#endif

//
//	EOF
//

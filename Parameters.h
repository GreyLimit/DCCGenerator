//
//	Parameters.h
//
//	The setting of compile time tune-able parameters
//

#ifndef _PARAMETERS_H_
#define _PARAMETERS_H_

//
//	The headline configuration parameters.
//
#define VERSION_NAME		"DCC Gen"
#define VERSION_NUMBER		"0.4.1"
#define SERIAL_BAUD_RATE	B115200
#define SERIAL_BAUD_RATE_STR	"115K"


//
//	The following list describes the parameters which can
//	be over-ridden from within this files.
//
//	CLOCK_EVENTS			The maximum number of time related events which can be handled.
//	CONSOLE_INPUT			The size of the console input buffer
//	CONSOLE_OUTPUT			The size of the console output buffer
//	TIME_OF_DAY_TASKS		The maximum number of tasks that the time of day code can handle
//
//	DCC_AVERAGE_DELAYS		If defined this is the number of DCC delay counts used to establish
//					and average delay value
//
//	DCC_RECALIBRATION_PERIOD	Milliseconds between the DCC system re-assessing the IRQ sync period.
//
//	LCD_LOOKAHEAD_LIMIT		The number of characters the LCD frame buffer looks ahead to find
//					an update to send.
//
#define LCD_LOOKAHEAD_LIMIT		20

//
//	For Debugging Purposes
//	======================
//
//	DISABLE_ASSERTIONS	If defined then (assuming that DEBUGGING_ENABLED
//				is not defined) all of the in-line code
//				ASSERT() verification checks will be excluded.
//
//	ENABLE_COUNT_INTERRUPTS	Display the number of interupts per second.
//	ENABLE_DCC_DELAY_REPORT	Display the "delay" between the DCC interrrupt
//				trigger and ISR being called.
//	ENABLE_IDLE_COUNT	Display a count of the timmes the task
//				manager was "idle" in the last period.
//	ENABLE_HEAP_STATS	Display usage of the heap.
//
//	ENABLE_DCC_SYNCHRONISATION
//				Enable code specifically included to fine
//				tune the execution of the DCC interrupt
//				handler (at the expense of wasting CPU
//				cycles inside interrupt handler).
//			Note/	Do not disable this option (by undefining
//				this symbol) as this is the key code that
//				ensures the DCC signal is time accurate.
//				In time this symbol will be removed.
//
//#define DISABLE_ASSERTIONS
//#define ENABLE_COUNT_INTERRUPTS
//#define ENABLE_DCC_DELAY_REPORT
//#define ENABLE_IDLE_COUNT
//#define ENABLE_HEAP_STATS
#define ENABLE_DCC_SYNCHRONISATION

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
//	ENABLE_TRACE_ROTARY	Trace actions on the rotary dial.
//	ENABLE_TRACE_SIGNAL	Trace activities on signals.
//	ENABLE_TRACE_SPI	Trace activities on the SPI device.
//	ENABLE_TRACE_STATS	Trace activities on the statistics module.
//	ENABLE_TRACE_TASK	Trace task/event manager.
//	ENABLE_TRACE_TOD	Trace actions in the Time Of Day module.
//	ENABLE_TRACE_TWI	Trace Data IO across the IIC bus.
//
//	ENABLE_STACK_TRACE	Enable the STACK_TRACE macro analysis
//

//#define ENABLE_TRACE_ADC
//#define ENABLE_TRACE_CLOCK
//#define ENABLE_TRACE_CONSOLE
//#define ENABLE_TRACE_DCC
//#define ENABLE_TRACE_DISTRICT
//#define ENABLE_TRACE_DRIVER
//#define ENABLE_TRACE_FBUFFER
//#define ENABLE_TRACE_FUNCTION
//#define ENABLE_TRACE_HCI
//#define ENABLE_TRACE_HEAP
//#define ENABLE_TRACE_KEYPAD
//#define ENABLE_TRACE_LCD
//#define ENABLE_TRACE_ROTARY
//#define ENABLE_TRACE_SIGNAL
//#define ENABLE_TRACE_SPI
//#define ENABLE_TRACE_STATS
//#define ENABLE_TRACE_TASK
//#define ENABLE_TRACE_TOD
//#define ENABLE_TRACE_TWI
//
//#define ENABLE_STACK_TRACE


#endif

//
//	EOF
//

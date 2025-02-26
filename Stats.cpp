//
//	Stats.cpp
//	=========
//
//	A system where stats are gathered at regular intervals
//	from various parts of the environment.
//

#include "Stats.h"
#include "Task.h"
#include "Clock.h"
#include "DCC.h"
#include "Memory_Heap.h"
#include "Trace.h"
#include "Console.h"

//
//	Call initialise to get the system going.
//
void Stats::initialise( void ) {

	STACK_TRACE( "void Stats::initialise( void )" );

	TRACE_STATS( console.print( F( "STS flag " )));
	TRACE_STATS( console.println( _flag.identity()));

#ifdef ENABLE_COUNT_INTERRUPTS
	//
	//	Count interrupts as cheaply as possible.
	//
	_interrupt_over = false;
	_interrupt_count = 0;
#endif

	if( !task_manager.add_task( this, &_flag )) ABORT( TASK_MANAGER_QUEUE_FULL );
	if( !event_timer.delay_event( MSECS( STATS_AVERAGE_PERIOD ), &_flag, true )) ABORT( EVENT_TIMER_QUEUE_FULL );
}

//
//	The routine called once a "period" to gather
//	in more stats and .. process them.
//
void Stats::process( UNUSED( byte handle )) {

	STACK_TRACE( "void Stats::process( byte handle )" );

	//
	//	For the moment only the DCC stats are gather this way.
	//
	_packets_sent.add( dcc_generator.packets_sent());
	_free_buffers.add( dcc_generator.free_buffers());

#ifdef ENABLE_COUNT_INTERRUPTS
	//
	//	Capture the interrupts handled, and send to the console.
	//
	_interrupts.add( _interrupt_over? MAXIMUM_WORD: _interrupt_count );
	_interrupt_count = 0;
	_interrupt_over = false;

	console.print( F( "STS interrupts " ));
	console.println( _interrupts.last());
#endif

#ifdef ENABLE_DCC_DELAY_REPORT
	//
	//	Dsiplay DCC delay
	//
	console.print( F( "STS DCC delay " ));
	console.println( dcc_generator.irq_delay());
	
#ifdef ENABLE_DCC_SYNCHRONISATION
	//
	//	Display DCC synchronisation
	//
	console.print( F( "STS DCC sync " ));
	console.println( dcc_generator.irq_sync());
#endif
#endif

#ifdef ENABLE_IDLE_COUNT
	//
	//	Display task idle counter.
	//
	console.print( F( "STS idle count " ));
	console.println( task_manager.idle_count());
#endif

#ifdef ENABLE_HEAP_STATS
	//
	//	Display heap stats.
	//
	console.print( F( "STS free heap " ));
	console.println( (word)heap.free_memory());
	console.print( F( "STS free block " ));
	console.println( (word)heap.free_block());
#endif
}


//
//	Return the packets set in the last time period.
//
word Stats::packets_sent( void ) {

	STACK_TRACE( "word Stats::packets_sent( void )" );

	return( _packets_sent.last());
}

//
//	Return the free buffers in the last time period.
//
byte Stats::free_buffers( void ) {

	STACK_TRACE( "word Stats::free_buffers( void )" );

	return( _free_buffers.last());
}

#ifdef ENABLE_COUNT_INTERRUPTS
	//
	//	Return the average interrupts processed.
	//
	word Stats::interrupts_caught( void ) {
		return( _interrupts.last());
	}
#endif


//
//	The stats object.
//
Stats stats;

//
//	EOF
//

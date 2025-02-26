//
//	Clock
//	=====
//
//	The implementation of the Clock module.
//

#include "Clock.h"

#include "Critical.h"
#include "Errors.h"
#include "Code_Assurance.h"
#include "Task.h"
#include "Trace.h"
#include "Stats.h"

#ifdef DEBUGGING_ENABLED
#include "Console.h"
#endif

//
//	Insert an event into the active list according to the
//	number of ticks specified in the left field.
//
//	This routine will need to suspend interrupts while handling
//	access to the queued events depending on the state of the
//	queue.
//
void Clock::insert( clock_event *ptr ) {
	
	STACK_TRACE( "void Clock::insert( clock_event *ptr )" );

	ASSERT( ptr != NIL( clock_event ));

	clock_event	*look,
			**adrs;

	//
	//	Insert new event somewhere down the queue.
	//
	ptr->next = NIL( clock_event );
	adrs = (clock_event **)&_active;
	while(( look = *adrs )) {
		if( ptr->left < look->left ) {
			ptr->next = look;
			look->left -= ptr->left;
			break;
		}
		ptr->left -= look->left;
		adrs = &( look->next );
	}
	*adrs = ptr;
}

//
//	Object creation and initialisation routines
//
Clock::Clock( void ) {
	//
	//	Set up the memory space we have been granted.
	//
	_active = NIL( clock_event );
	_free = NIL( clock_event );
	for( byte i = 0; i < clock_events; i++ ) {
		_events[ i ].next = (clock_event *)_free;
		_free = &( _events[ i ]);
	}
}

void Clock::initialise( void ) {
	
	STACK_TRACE( "void Clock::initialise( void )" );
	
	Critical	code;

	//
	//		Set Timer to default empty values.
	//
	CLK_TCCRnA = 0;		// Set entire HW_TCCRnA register to 0
	CLK_TCCRnB = 0;		// Same for HW_TCCRnB

	//
	//		Turn on CTC mode by setting this one bit
	//
	CLK_TCCRnA |= bit( CLK_WGMn1 );
	
	//
	//	Set the CSn bits (0..2) in HW_TCCRnB to select
	//	the desired pre-scaler.
	//
	switch( clock_divider ) {
		case 1: {
			CLK_TCCRnB |= bit( CLK_CSn0 );
			break;
		}
		case 8: {
			CLK_TCCRnB |= bit( CLK_CSn1 );
			break;
		}
		case 64: {
			CLK_TCCRnB |= bit( CLK_CSn1 ) | bit( CLK_CSn0 );
			break;
		}
		case 256: {
			CLK_TCCRnB |= bit( CLK_CSn2 );
			break;
		}
		case 1024: {
			CLK_TCCRnB |= bit( CLK_CSn2 ) | bit( CLK_CSn0 );
			break;
		}
		default: {
			ABORT( CLOCK_INVALID_DIVIDER );
			break;
		}
	}
	
	//
	//	Reset count and compare register to default values.
	//
	CLK_COUNTER_REG = 0;
	CLK_COMPARE_REG = maximum_count;
	
	//
	//	With no events recorded at this point (assumption)
	//	we leave the clock stopped.
	//
	ASSERT( _active == NIL( clock_event ));
	
	//
	//	Link this object and it signal together
	//
	if( !task_manager.add_task( this, &_irq )) ABORT( TASK_MANAGER_QUEUE_FULL );
}

//
//	This is the interface into the clock allowing new
//	"one time" or "repeating" events to be created.
//
bool Clock::delay_event( word ticks, Signal *gate, bool repeating ) {
	
	STACK_TRACE( "bool Clock::delay_event( word ticks, Signal *gate, bool repeating )" );
	
	clock_event	*ptr;

	ASSERT( ticks > 0 );
	ASSERT( gate != NIL( Signal ));

	TRACE_CLOCK( console.print( F( "CLK delay " )));
	TRACE_CLOCK( console.print( ticks ));
	TRACE_CLOCK( console.print( F( " gate " )));
	TRACE_CLOCK( console.print( gate->identity()));
	TRACE_CLOCK( console.print( F( " repeats " )));
	TRACE_CLOCK( console.println( repeating ));

	//
	//	Find an empty record or return false.
	//
	if(( ptr = (clock_event *)_free ) == NIL( clock_event )) return( false );

	//
	//	Unlink free record...
	//
	_free = ptr->next;
	
	//
	//	...and fill it in.
	//
	ptr->left = ticks;
	ptr->repeats = repeating? ticks: 0;
	ptr->gate = gate;
	ptr->next = NIL( clock_event );

	//
	//	Normalise the time status of the head record (if there
	//	is one).
	//
	if( _active ) {
		byte stepped = CLK_COUNTER_REG;

		if( stepped < _active->left ) {
			_active->left -= stepped;
		}
		else {
			_active->left = 1;
		}
		//
		//	Insert record into the ordered list of pending
		//	timer events.
		//
		insert( ptr );
	}
	else {
		//
		//	No queue means we just make this the queue and
		//	start the timer for this event.
		//
		_active = ptr;
		start_timer(( _active->left < maximum_count )? _active->left: maximum_count );
	}

	//
	//	Done!
	//
	return( true );
}

//
//	An "in-line" delay of a specified number of ticks.
//
//	This call will automatically return if it is running
//	inside an interrupt or critical code section without
//	any pause being experienced.
//
void Clock::inline_delay( word ticks ) {
	
	STACK_TRACE( "void Clock::inline_delay( word ticks )" );
	
	TRACE_CLOCK( console.print( F( "CLK delay " )));
	TRACE_CLOCK( console.println( ticks ));

	if( Critical::normal_code()) {
		Signal	gate;
		
		if( delay_event( ticks, &gate, false )) {
			while( !gate.acquire()) task_manager.pole_task();
		}
	}
}


//
//	Routine called to record an interrupt event.
//
void Clock::irq( void ) {

	STACK_TRACE( "void Clock::irq( void )" );
	
	stop_timer();
	_irq.release( true );
}

//
//	This is the interrupt processing routine.
//
void Clock::process( UNUSED( byte handle )) {

	STACK_TRACE( "void Clock::process( byte handle )" );
	
	ASSERT( _active != NIL( clock_event ));
	
	clock_event	*ptr;
	byte		step;

	//
	//	Now we recover the size of the compare register into
	//	"step" (as this is the step in time towards the next
	//	event being kicked off).
	//
	//	JEFF/	There is a good argument for adding the value
	//		of the COUNTER register to the value of the
	//		COMPARE register as there are time when the
	//		firmware will take some time to get to this
	//		routine, and the counter counts this time.
	//
	//		eg	step -= CLK_COMPARE_REG + CLK_COUNTER_REG;
	//
	//		However, there is serious risk of over flow
	//		here with both registers being 8 bit and the
	//		system frequently counting to 250....
	//
	step = CLK_COMPARE_REG;

	//
	//	The queue of pending events is sorted in chronological
	//	order where the content of the 'left' field is the
	//	number of ticks left before the event is required. But,
	//	it must be remembered that this applies only for the
	//	event at the head of the queue.  For subsequent events
	//	the 'left' field is the time difference from the previous
	//	event.
	//
	if( step < _active->left ) {
		//
		//	The step to get here is less than the count left
		//	on the event, so we deduct the step from left
		//	and start the timer again.
		//
		//	This introduces an inaccuracy (see note above)
		//	but this is not meant to be a cycle accurate time
		//	source at the end of the day.
		//
		if(( _active->left -= step ) < maximum_count ) {
			start_timer( _active->left );
		}
		else {
			start_timer( maximum_count );
		}
		return;
	}

	//
	//	We set the left time explicitly to zero to
	//	simplify the following code.
	//
	_active->left = 0;

	//
	//	At this point we *know* that we have at least one event
	//	to process, so get on with it.
	//

	TRACE_CLOCK( console.println( F( "CLK events!" )));

	//
	//	Now loop through all the events with a left value of
	//	zero these being the events which are require now.
	//
	while(( ptr = (clock_event *)_active )&&( ptr->left == 0 )) {
		//
		//	This event time has arrived, unhook it
		//	from the _active list.
		//
		_active = ptr->next;
		
		//
		//	Signal that the timer has passed.  This will
		//	(eventually) cause the task manager to call
		//	up the appropriate process function.
		//
		ptr->gate->release( true );

		//
		//	If the repeat field is non-zero then...
		//
		if(( ptr->left = ptr->repeats ) > 0 ) {
			//
			//	... then reschedule the event.
			//
			insert( ptr );
			
			TRACE_CLOCK( console.print( F( "CLK Gate " )));
			TRACE_CLOCK( console.print( ptr->gate->identity()));
			TRACE_CLOCK( console.print( F( " repeats " )));
			TRACE_CLOCK( console.println( ptr->repeats ));
		}
		else {
			//
			//	.. or put it back in the free list.
			//
			ptr->next = (clock_event *)_free;
			_free = ptr;
			
			TRACE_CLOCK( console.print( F( "CLK Gate " )));
			TRACE_CLOCK( console.print( ptr->gate->identity()));
			TRACE_CLOCK( console.println( F( " once" )));
		}
	}

	//
	//	Start the timer for the next remaining event.
	//
	if( _active ) start_timer(( _active->left < maximum_count )? _active->left: maximum_count );
	
	//
	//	Done.
	//
	TRACE_CLOCK( console.println( F( "CLK Done" )));
}



//
//	The actual clock instance.
//
Clock	event_timer;

//
//	and its associated ISR code.
//
ISR( CLK_TIMERn_COMPA_vect ) {
	COUNT_INTERRUPT;
	event_timer.irq();
}


//
//	EOF
//

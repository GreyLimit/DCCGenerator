//
//	TOD.cpp
//	=======
//
//	The implementation of the time of day tracker.
//

#include "Code_Assurance.h"
#include "TOD.h"
#include "Task.h"
#include "Clock.h"
#include "Critical.h"
#include "Memory_Heap.h"

#ifdef DEBUGGING_ENABLED
#include "Console.h"
#endif

//
//	Declare our limits array.
//
const byte TOD::limit[ TOD::stages ] PROGMEM = {
	60,	// Seconds
	60,	// Minutes
	24,	// Hours
	100	// Days (arbitrary).
};

//
//	Constructor and initialise routine.
//
TOD::TOD( void ) {
	//
	//	The TOD values.
	//
	for( byte i = 0; i < stages; _stage[ i++ ] = 0 );
	_elapsed = 0;
	
	//
	//	The pending signals data.
	//
	_active = NIL( pending_tod );
	_free = NIL( pending_tod );
}

//
//	Call to initialise the TOD system within the Clock and
//	Task sub-system.
//
void TOD::initialise( void ) {
	
	STACK_TRACE( "void TOD::initialise( void )" );

	TRACE_TOD( console.print( F( "TOD flag " )));
	TRACE_TOD( console.println( _flag.identity()));
	
	if( !task_manager.add_task( this, &_flag )) ABORT( TASK_MANAGER_QUEUE_FULL );
	if( !event_timer.delay_event( MSECS( 1000 ), &_flag, true )) ABORT( EVENT_TIMER_QUEUE_FULL );
}

//
//	Provide access to the TOD data.
//
byte TOD::read( byte index ) {
	
	STACK_TRACE( "byte TOD::read( byte index )" );
	
	if( index < stages ) return( _stage[ index ]);
	return( 0 );
}
bool TOD::write( byte index, byte value ) {
	
	STACK_TRACE( "bool TOD::write( byte index, byte value )" );
	
	if(( index < stages )&&( value < progmem_read_byte( limit[ index ]))) {
		_stage[ index ] = value;
		return( true );
	}
	return( false );
}

//
//	Provide an *indication* of time since boot
//	in seconds.  This value, an unsigned 16 bit word
//	will wrap to 0 every 18 hours, 12 minutes and
//	(roughly) 15 seconds.
//
word TOD::elapsed( void ) {
	return( _elapsed );
}

//
//	Add a flag to the list of pending flag updates.  The
//	duration is specified in whole seconds upto 65535
//	seconds into the future.  There is no "time of day"
//	based clock scheduling.
//
bool TOD::add( word duration, Signal *flag ) {
	
	STACK_TRACE( "bool TOD::add( word duration, Signal *flag )" );

	ASSERT( flag != NIL( Signal ));
	
	TRACE_TOD( console.print( F( "TOD delay " )));
	TRACE_TOD( console.print( duration ));
	TRACE_TOD( console.print( F( " flag " )));
	TRACE_TOD( console.println( flag->identity()));

	pending_tod	*ptr, **adrs, *look;

	if(( ptr = _free )) {
		_free = ptr->next;
	}
	else {
		if(!( ptr = new pending_tod )) return( false );
	}
	ptr->left = duration;
	ptr->flag = flag;
	adrs = &_active;
	while(( look = *adrs )) {
		if( ptr->left < look->left ) {
			look->left -= ptr->left;
			break;
		}
		ptr->left -= look->left;
		adrs = &( look->next );
	}
	ptr->next = look;
	*adrs = ptr;
	return( true );
}

//
//	The TASK entry point, called each time the flag is
//	set true by the clock system.
//
void TOD::process( UNUSED( byte handle )) {
	STACK_TRACE( "void TOD::process( byte handle )" );

	pending_tod	*ptr;

	//
	//	Called every second to update the TOD and look
	//	at the pending list.
	//
	//	Update the elapsed time and time of day.
	//
	_elapsed++;
	for( byte i = 0; i < stages; i++ ) {
		if( ++_stage[ i ] < progmem_read_byte( limit[ i ])) break;
		_stage[ i ] = 0;
	}

	//
	//	Is there a list of pending actions awaiting our attention.
	//
	if( _active == NIL( pending_tod )) {

		TRACE_TOD( console.println( F( "TOD queue empty" )));
		
		return;
	}

	//
	//	Now we check off all actions with no pending time left
	//	and execute their signal.
	//
	while(( ptr = _active )) {
		//
		//	Has this counter run out?  If not (not zero)
		//	then reduce and leave the loop.
		//
		if( ptr->left ) {

			TRACE_TOD( console.print( F( "TOD pending " )));
			TRACE_TOD( console.println( ptr->left ));
			
			//
			//	Yes!  Decrease it and finish.
			//
			_active->left -= 1;
			break;
		}

		TRACE_TOD( console.print( F( "TOD release flag " )));
		TRACE_TOD( console.println( ptr->flag->identity()));

		//
		//	Counter was zero, so release the flag and return the
		//	record to the free list.
		//
		ptr->flag->release();
		_active = ptr->next;
		ptr->next = _free;
		_free = ptr;
	}

	TRACE_TOD( console.println( F( "TOD done" )));

	//
	//	Done.
	//
}

//
//	A human scale inline delay routine.
//
void TOD::inline_delay( word seconds ) {
	STACK_TRACE( "void TOD::inline_delay( word seconds )" );

	Signal	flag;

	TRACE_TOD( console.print( F( "TOD inline delay " )));
	TRACE_TOD( console.print( seconds ));
	TRACE_TOD( console.print( F( " flag " )));
	TRACE_TOD( console.println( flag.identity()));

	ASSERT( Critical::normal_code());

	if( seconds > 0 ) {
		if( add( seconds, &flag )) {
			while( !flag.acquire()) task_manager.pole_task();
		}
		else {
			errors.log_error( TIME_OF_DAY_QUEUE_FULL, 0 );
		}
	}
}


//
//	Declare the TOD object.
//
TOD time_of_day;


//
//	EOF
//

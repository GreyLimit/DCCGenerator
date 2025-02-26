//
//	Signal.cpp
//	==========
//
//	The implementation of the Signal module.
//


#include "Signal.h"
#include "Task.h"
#include "Critical.h"
#include "Code_Assurance.h"

#ifdef DEBUGGING_ENABLED
#include "Console.h"
#endif

//
//	Declare the static task control elements of the Signal
//	system.
//
Signal::signal_queue	Signal::_fast = { NIL( Signal ), &( Signal::_fast.queue )};
Signal::signal_queue	Signal::_slow = { NIL( Signal ), &( Signal::_slow.queue )};;

//
//	Associate this signal with a specific task and handle.
//
bool Signal::associate( Task_Entry *process, byte handle ) {
	
	STACK_TRACE( "void Signal::associate( Task_Entry *process, byte handle )" );

	TRACE_SIGNAL( console.print( F( "SIG signal " )));
	TRACE_SIGNAL( console.print( identity()));
	TRACE_SIGNAL( console.print( F( " handle " )));
	TRACE_SIGNAL( console.println( handle ));

	ASSERT( process != NIL( Task_Entry ));
	ASSERT( handle > 0 );
	ASSERT( _handle == 0 );
	ASSERT( _process == NIL( Task_Entry ));
	ASSERT( _count == 0 );

	_handle = handle;
	_process = process;

	return( true );
}

//
//	Provide the back end routine targeting a queue.
//
//	o	Remove the signal from the queue.
//	o	Execute the associated process routines.
//	o	Re-queue the signal if the count is non-zero.
//
//	Return true is a task has been executed, false otherwise.
//
bool Signal::run_task( bool fast ) {
	
	STACK_TRACE( "bool Signal::run_task( bool fast )" );

	Signal	*sig;
	bool	requeue;

	signal_queue *queue = fast? &_fast: &_slow;
	
	//
	//	Pull the top item off the queue, if there is one.
	//
	//	This has to be critical code to avoid the possibility
	//	of the signal being updated when we are playing with
	//	its position in the queue.
	//
	//	Variable queue will tell us if we need to re-queue the
	//	signal after the task process has been called.
	//
	{
		Critical code;
		
		if(( sig = (Signal *)queue->queue ) == NIL( Signal )) {
			//
			//	Leave if nothing to do.
			//
			return( false );
		}
		//
		//	Unhook from the queue...
		//
		if(( queue->queue = sig->_next ) == NIL( Signal )) queue->tail = &( queue->queue );
		
		//
		//	...and tidy up.  We will be asserting that the
		//	content of the next field remains NIL if we
		//	are re-queueing it ourselves as a check on the
		//	logic of this code.
		//
		sig->_next = NIL( Signal );

		//
		//	There should never be a signal in the queue with
		//	a count of zero!
		//
		ASSERT( sig->_count > 0 );

		//
		//	Reduce the count and determine if we are putting
		//	it back into the queue at the end of this process.
		//
		//	We do the "queue" decision before making the call
		//	to the task as the act of making this call may release
		//	the signal we are currently processing, and so it may
		//	*already* have been put back into the queue.  But this
		//	would only happen if the release call raised the count
		//	to exactly 1 so if the count is already 1 or more then
		//	we have to re-queue the signal.
		//
		requeue = (( sig->_count -= 1 ) > 0 );
	}

	//
	//	Some sanity checks for debugging purposes.
	//
	ASSERT( sig->_handle > 0 );
	ASSERT( sig->_process != NIL( Task_Entry ));

	//
	//	We now call the process routine for this task.
	//

	TRACE_TASK( console.print( F( "SIG run " )));
	TRACE_TASK( console.println( sig->identity()));

	sig->_process->process( sig->_handle );

	//
	//	Now we put the Signal to the back of the queue if the
	//	requeue variable says so.
	//
	{
		Critical code;

		if( requeue ) {

			TRACE_TASK( console.print( F( "SIG requeue " )));
			TRACE_TASK( console.println( sig->identity()));

			//
			//	Check that the next field is still NIL
			//	as we are tagging this to the tail end
			//	of the fast queue.
			//
			ASSERT( sig->_next == NIL( Signal ));
			
			//
			//	Append to the tail end of the fast queue.
			//
			*queue->tail = sig;
			queue->tail = &( sig->_next );
		}
	}

	//
	//	Done.
	//
	return( true );
}


//
//	Constructor: initialise the signal to
//	zero, forcing the system to wait until a "signal"
//	is raised.
//
Signal::Signal( void ) {
	//
	//	The initial setup of a Signal is a "free standing"
	//	type, not associated with any handle/processing
	//	routine.
	//
	_next = NIL( Signal );
	_handle = 0;
	_process = NIL( Task_Entry );
	_count = 0;
}

//
//	This routine "returns" a resource to the signal; the internal
//	count is increased by one and the signal (if flagged as being
//	managed by the task system) is appended to one of the task
//	queues as specified by the fast argument.
//
void Signal::release( bool fast ) {
	
	STACK_TRACE( "void Signal::release( void )" );
	
	TRACE_SIGNAL( console.print( F( "SIG Release signal " )));
	TRACE_SIGNAL( console.print( identity()));
	TRACE_SIGNAL( console.println( fast? F( "fast" ): F( "slow" )));
	
	Critical code;
	
	//
	//	Simply add 1 to the counter and check to see if we
	//	have "wrapped".
	//
	//	Saying that this "should not" happen does
	//	not mean it will never happen but bugs happen.
	//
	if(( _count += 1 ) == 0 ) {
		//
		//	Report the value wrapping (if debugging)
		//	then abort.
		//
#ifdef DEBUGGING_ENABLED
		errors.log_error( SIGNAL_RANGE_ERROR, identity());
#endif

		//
		//	Abort programming error.
		//
		ABORT( PROGRAMMER_ERROR_ABORT );
	}

	//
	//	Finally, if the count value is now 1 and the
	//	handle value is non-zero, then we need to add
	//	this signal to the queue.
	//
	if(( _count == 1 )&&( _handle > 0 )) {

		TRACE_TASK( console.print( F( "SIG queue " )));
		TRACE_TASK( console.println( identity()));

		ASSERT( _next == NIL( Signal ));

		signal_queue *queue = fast? &_fast: &_slow;

		*queue->tail = this;
		queue->tail = &_next;
	}
}


//
//	This routine is used to test and claim a resource in
//	a single activity.  If the function returns true then
//	a resource *has already been* claimed.
//
//	This routine is *only* to be used on "free standing"
//	Signals.
//
bool Signal::acquire( void ) {
	
	STACK_TRACE( "bool Signal::acquire( void )" );
	
	bool	result;

	ASSERT( Critical::normal_code());
	ASSERT( _handle == 0 );
	ASSERT( _process == NIL( Task_Entry ));

	{
		Critical code;

		if(( result = ( _count > 0 ))) _count--;
	}

	if( result ) {
		TRACE_SIGNAL( console.print( F( "SIG Acquired signal " )));
		TRACE_SIGNAL( console.println( identity()));
	}

	return( result );
}

//
//	Return the value inside the Signal - may not be aaccurate
//	even before the function returns!
//
byte Signal::value( void ) {
	
	STACK_TRACE( "byte Signal::value( void )" );
	
	return( _count );
}


#ifdef DEBUGGING_ENABLED
	//
	//	For debugging purposes this routine returns a unique
	//	handle on the signal.  This will, effectively, be the
	//	address of the handle but serves no functional purpose
	//	other than enabling debugging to track signal actions
	//	through logged output.
	//
	word Signal::identity( void ) {
		return( (word)this );
	}
#endif


//
//	EOF
//

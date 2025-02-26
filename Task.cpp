//
//	Task.cpp
//	========
//
//	Declare the task manager
//

#include "Code_Assurance.h"
#include "Trace.h"
#include "Task.h"

#include "Console.h"

//
//	Constructor and Destructor.
//
TaskManager::TaskManager( void ) {
	//
	//	Just note we start at depth 0.
	//
	_depth = 0;
	
	//
	//	Initialise the balance value.
	//
	_balance = balance;
	
	//
	//	Start with idel empty.
	//
	_idle = 0;
}

//
//	The "in sequence" initialisation routine.
//
void TaskManager::initialise( void ) {

	STACK_TRACE( "void TaskManager::initialise( void )" );

	//
	//	For the moment this is empty!
	//
}

//
//	This is a task polling routine and is called to see
//	if a single task can be executed before returning.
//
void TaskManager::pole_task( void ) {

	STACK_TRACE( "void TaskManager::pole_task( void )" );
	
	TRACE_TASK( console.print( F( "TM depth " )));
	TRACE_TASK( console.println( _depth ));
	
	ASSERT( Critical::normal_code());

	//
	//	Perform our "anti-recursion" depth trap.
	//
	if( _depth >= maximum_depth ) {
		errors.log_error( TASK_DEPTH_EXCEEDED, _depth );
		return;
	}

	//
	//	Note our new nesting depth.
	//
	_depth++;
	
	//
	//	Run a queue from one of either the fast or slow
	//	queue.  We take some guidance from the state of
	//	the balance value in choosing which queue to
	//	pick from.
	//
	if( _balance ) {
		//
		//	While balance is non-zero we choose to
		//	start with the fast queue.
		//
		if( Signal::run_task( true )) {
			//
			//	Reduce the balance as a result of
			//	running the fast process.
			//
			_balance--;
		}
		else {
			//
			//	Try the slow queue instead.
			//
			if( Signal::run_task( false )) {
				//
				//	Successfully running a slow
				//	taks resets the balance.
				//
				_balance = balance;
			}
			else {
				//
				//	Chalk up another time of idleness.
				//
				_idle += 1;
			}
		}
	}
	else {
		//
		//	The balance has to be swung over towards the
		//	slow queue (as _balance is zero).
		//
		if( Signal::run_task( false )) {
			//
			//	Successfully running a slow
			//	task resets the balance.
			//
			_balance = balance;
		}
		else {
			//
			//	Otherwise just try the fast queue.
			//
			if( !Signal::run_task( true )) {
				//
				//	Chalk up another time of idleness.
				//
				_idle += 1;
			}
		}
	}

	//
	//	Restore nesting depth to previous value.
	//
	_depth--;

}

//
//	This is the task scheduler interface (called from the
//	main loop continuously.
//
void TaskManager::run_tasks( void ) {

	STACK_TRACE( "void TaskManager::run_tasks( void )" );
	
	TRACE_TASK( console.println( F( "TM run tasks" )));

	ASSERT( _depth == 0 );

	//
	//	All we do is call the pole_task routine forever.
	//
	while( true ) pole_task();
	
	//
	//	This function never exits.
	//
}

//
//	This is the access point where tasks are added to the system.
//
bool TaskManager::add_task( Task_Entry *call, Signal *trigger, byte handle ) {

	STACK_TRACE( "bool TaskManager::add_task( Task_Entry *call, Signal *trigger, byte handle )" );

	ASSERT( trigger != NIL( Signal ));

	TRACE_TASK( console.print( F( "TM add flag " )));
	TRACE_TASK( console.print( trigger->identity()));
	TRACE_TASK( console.print( F( " handle " )));
	TRACE_TASK( console.print( handle ));
	TRACE_TASK( console.print( F( " from " )));
	TRACE_TASK( STACK_CALLER( &console ));

	//
	//	Update the Signal with its task data.
	//
	return( trigger->associate( call, handle ));
}

//
//	Report (and reset) the idel counter for statistics purposes.
//
word TaskManager::idle_count( void ) {
	word	was;
	
	was = _idle;
	_idle = 0;
	return( was );
}


//
//	define the task_manager itself.
//
TaskManager task_manager;

//
//	EOF
//

//
//	Task.h
//	======
//
//	Define a simple mechanism which can be used to juggle a set
//	of activities, calling each when a condition flag is set.
//

#ifndef _TASK_H_
#define _TASK_H_

//
//	We will need the Environment.
//
#include "Environment.h"
#include "Parameters.h"
#include "Configuration.h"
#include "Signal.h"
#include "Errors.h"
#include "Task_Entry.h"

//
//	My initial thoughts were that this code would need to be
//	"Critical" code (at least in some sections), but *if* the
//	code which creates tasks is *always* outside an interrupt
//	routine then this requirement is unnecessary.
//

//
//	Declare a value for the "maximum number of nested calls" to
//	this object.
//
#ifndef TASK_MAXIMUM_DEPTH
#define TASK_MAXIMUM_DEPTH	3
#endif


//
//	Declare a value for the "balance" between the fast and slow
//	task queues.
//
#ifndef TASK_BALANCE
#define TASK_BALANCE		4
#endif



//
//	Define the task manager class where the argument passed in is
//	the maximum number of tasks which it will handle.
//
class TaskManager {
private:
	//
	//	Define what we consider the maximum nesting depth for a task.
	//
	static const byte maximum_depth = TASK_MAXIMUM_DEPTH;
	
	//
	//	Declare the "balance" allowed between the fast and slow queues.
	//
	static const byte balance	= TASK_BALANCE;
	
	//
	//	Define a "depth indicator"; a counter which tracks
	//	how many times the object has been called in a "nested"
	//	fashion.  The primary purpose is to avoid cyclic/recursive
	//	calls to this module blowing up the stack space and
	//	causing the firmware to fail in a unpredictable way.
	//
	byte		_depth;
	
	//
	//	Counter used to ensure that the fast queue does not
	//	flood the system and prevent slow queue events from
	//	getting any time.
	//
	byte		_balance;
	
	//
	//	Count the number of times that the task manager attempts
	//	to do *something* but finds nothing to do.
	//
	word		_idle;

public:
	//
	//	Constructor and Destructor.
	//
	TaskManager( void );

	//
	//	The "in sequence" initialisation routine.
	//
	void initialise( void );

	//
	//	This is a task polling routine and is called to see
	//	if a single task can be executed before returning.
	//
	void pole_task( void );

	//
	//	This is the task scheduler interface (called from the
	//	main loop continuously).
	//
	void run_tasks( void );

	//
	//	This is the access point where tasks are added to the system.
	//
	//	Here we syntactically allow the handle value to be ignored
	//	when an object calls add_task, forcing the handle to be
	//	non-zero as this is a hard requirement of the Signal class
	//	(zero being used to flag a Signal not accessed by the Task
	//	system).
	//
	bool add_task( Task_Entry *call, Signal *trigger, byte handle = 1 );
	
	//
	//	Report (and reset) the idle counter for statistics purposes.
	//
	word idle_count( void );
};

//
//	define the task_manager itself.
//
extern TaskManager task_manager;

#endif

//
//	EOF
//

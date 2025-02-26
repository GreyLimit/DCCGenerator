//
//	Signal.h
//	========
//
//
//	Declare the "signal" class which is used to send notifications
//	between feeder objects and the consumer objects.
//
//	The "control" of processing power between the feed/consumer
//	code is via the Task Manager.
//
//	The name "Signal" is a nod towards the P/V signal system created
//	by the Edsger W. Dijkstra, after which this system is formed. 
//

#ifndef _SIGNAL_H_
#define _SIGNAL_H_

//
//	We *must* exclusive control over the CPU inside this class
//	so we include the Critical code module.
//
#include "Environment.h"
#include "Parameters.h"
#include "Configuration.h"
#include "Task_Entry.h"
#include "Trace.h"

//
//	Declare the signal type forming the basis for the system.
//
class Signal {
private:
	//
	//	Signalling for Task Control
	//	---------------------------
	//
	//	Here are defined the static components of the Signal
	//	(and therefore common across all signals) which will
	//	provide functions designed to, effectively, replace
	//	task managers inner mechanism with a distributed
	//	system encapsulated within the Signals themselves.
	//
	
	//
	//	Define a structure to control a single signal queue.
	//
	struct signal_queue {
		volatile Signal		*queue,
					**tail;
	};
	
	//
	//	Fast and Slow queues allow events instigated from within
	//	an ISR (hardware related ) to be expedited ahead of
	//	software events.
	//
	//	This is the point where an array of queues could be
	//	used.  For the moment this is adequate for purpose.
	//
	static signal_queue		_fast,
					_slow;
	
	//
	//	Specific Signal Controls
	//	========================
	//
	//	Define the "per Signal" task elements required.  This
	//	needs to be volatile as it is adjusted asychronously
	//	from within interrupt routines.
	//
	volatile Signal		*_next;

	//
	//	Define the handle indicating a Signal managed through
	//	the task management system (if non-zero).
	//
	byte			_handle;
	
	//
	//	The pointer to the process routine associated with this
	//	Signal if this has been brought into the Task Management
	//	system.
	//
	Task_Entry		*_process;
	
public:
	//
	//	Associate this signal with a specific task and handle.
	//
	bool associate( Task_Entry *process, byte handle );
	
	//
	//	The entry point into the task queues that will, if
	//	a ready-to-run task is available:
	//
	//	o	Remove the signal from the queue.
	//	o	Execute the associated process routines.
	//	o	Re-queue the signal if the count is non-zero.
	//
	//	Return true if a task has been executed, false otherwise.
	//	This gives the potential for a "idle time" task to be
	//	created which never executes unless there are no other
	//	tasks pending.
	//
	static bool run_task( bool fast );

private:
	//
	//	Basic Signal functionality
	//	--------------------------
	//
	//	The following pieces capture the core function
	//	of the Signal.
	//
	
	//
	//	Define the value limits for the counter.
	//
	static const byte maximum_count_value = MAXIMUM_BYTE;
	
	//
	//	This is the Counter that forms the controlled data
	//	element.  This needs to be volatile as it is adjusted
	//	asychronously from within interrupt routines.
	//
	volatile byte	_count;

public:
	//
	//	Constructors - default initialises the signal to
	//	zero, forcing the system to wait until a "signal"
	//	is raised.
	//
	Signal( void );

	//
	//	This routine "returns" a resource to the signal; the internal
	//	count is increased by one and the signal (if flagged as being
	//	managed by the task system) is appended to one of the task
	//	queues.
	//
	void release( bool fast = false );

	//
	//	This routine is used to test and claim a resource in
	//	an atomic action.  If the function returns true then
	//	a resource *has already been* claimed (the Signal
	//	counter will have been decremented for you).
	//
	//	Only valid for non-task signals.
	//
	bool acquire( void );

	//
	//	Return the value inside the Signal - may not be aaccurate
	//	even before the function returns!
	//
	byte value( void );
	
#ifdef DEBUGGING_ENABLED
	//
	//	For debugging purposes this routine returns a unique
	//	handle on the signal.  This will, effectively, be the
	//	address of the handle but serves no functional purpose
	//	other than enabling debugging to track signal actions
	//	through logged output.
	//
	word identity( void );
	
#endif

};

#endif

//
//	EOF
//

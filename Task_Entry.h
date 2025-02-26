//
//	Task_Entry.h
//	============
//
//	Define the entry point of a Task managed by the
//	task manager system.
//

#ifndef _TASK_ENTRY_H_
#define _TASK_ENTRY_H_


//
//	Define a virtual class which forms the interface between
//	the implementation of a task and the task manager.
//
class Task_Entry {
public:
	//
	//	This routine is called every time the associated signal
	//	shows that a resource is available.  Handle is a value
	//	assigned when the task is introduced into the task
	//	manager enabling a single object to have multiple flags
	//	associated with it (each with a unique handle).
	//
	virtual void process( byte handle ) = 0;
};


#endif

//
//	EOF
//

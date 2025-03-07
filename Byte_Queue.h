//
//	Byte_Queue - implement a simple byte based queue.
//

#ifndef _BYTE_QUEUE_H_
#define _BYTE_QUEUE_H_

//
//	Bring in the Instrumentation package to facilitate
//	data collection and tracking.
//
#include "Environment.h"
#include "Parameters.h"
#include "Configuration.h"
#include "Code_Assurance.h"
#include "Critical.h"
#include "Signal.h"
#include "Task_Entry.h"
#include "Task.h"
#include "Errors.h"

//
//	Dymanic queue needs the memory heap.
//
#include "Memory_Heap.h"

//
//	This is the generic API these are based on.
//
#include "Byte_Queue_API.h"


//
//	Define a class to implement a "stretchy" dynamically allocated
//	byte queue.
//
class Byte_Queue : public Byte_Queue_API, Memory_Recovery, Task_Entry {
private:
	//
	//	Declare a constant for the size of a temporary byte
	//	buffer which is used by the class when
	//
#ifdef DYNAMIC_BLOCK_SIZE
	static const byte	block_size = DYNAMIC_BLOCK_SIZE;
#else
	static const byte	block_size = 8;
#endif

	//
	//	Define the structure used store a section of the
	//	queue as allocated in the heap.
	//
	struct queue_block {
		byte		queue[ block_size ];
		queue_block	*next;
	};
	
	//
	//	Blocks are allocated and managed from here.
	//
	//	The system will *always* keep a block in the
	//	queue and so the _tail pointer will always
	//	point to a block being filled up, and never
	//	points to the *address* of the last next field.
	//
	volatile queue_block	*_queue,
				*_tail,
				*_free;
	
	//
	//	Remember where we are filling in on the tail block
	//	and picking out on the head block.
	//
	volatile byte		_in,
				_out,
				_content;

	//
	//	The signal used to facilitate block allocation.
	//
	Signal			_flag;

	//
	//	Optional data ready signal.
	//
	Signal			*_ready;

	//
	//	This is the handle number used for memory allocation.
	//
	static const byte	allocate_block = 1;

public:
	//
	//	Constructor only.
	//	
	Byte_Queue( void );

	//
	//	This is the initialisation routine that must be called
	//	before the queue can be used.  The argument can be either
	//	NIL( Signal ) for no data ready signals or the address
	//	of a signal to be released on data being ready.
	//
	void initialise( Signal *ready );

	//
	//	This processing routine is called only when there
	//	has been a request (from inside a Critical code
	//	section) for an additional queue_block to be allocated.
	//
	virtual void process( byte handle );
	
	//
	//	The Byte Queue API
	//	==================
	//
	//	The method for writing to the queue depends to some
	//	extent if we are already in an ISR or Critical code
	//	section.
	//
	virtual bool write( byte data );
	virtual byte read( void );

	//
	//	Perform a "reset" of the underlying system.  This
	//	is used only to recover from an unknown condition
	//	with the expectation that upon return the queue
	//	can be reliably used.
	//
	virtual void reset( void );
	virtual byte space( void );
	virtual byte available( void );
	virtual byte pending( void );

	//
	//	The memory reclamation API.
	//	---------------------------
	//

	//
	//	Return the number of bytes memory being "cached" and
	//	available for release if required.  This is a statistical
	//	call to allow tracking of memory usage.
	//
	virtual size_t cache_memory( void );

	//
	//	Tell the object to clear all cached memory and release it
	//	to the heap.
	//
	virtual bool clear_cache( void );

	//
	//	Ask the object how much memory, as a single block, it
	//	would release to satisfy a specified allocation request.
	//	Return 0 if this object cannot satisfy the request.
	//
	virtual size_t test_cache( size_t bytes );

	//
	//	Request that an object release, as a single block,
	//	enough memory to cover the specified allocation.
	//	Return true on success, false on failure.
	//
	virtual bool release_cache( size_t bytes );


};


#endif

//
//	EOF
//

//
//	Memory_Heap.h
//	=============
//
//	The definition of  a generic memory heap allocation (and
//	deallocation) system designed to be a used in place of
//	the C++ new() and delete().
//
//	Note:
//
//	This system is explicitly granted a fixed memory space
//	upon initialisation and also provides a blanket "memory
//	reset" function which sets the memory provided back to
//	the empty state.
//

#ifndef _MEMORY_HEAP_H_
#define _MEMORY_HEAP_H_

//
//	Bring in some consistent types.
//
#include "Environment.h"
#include "Parameters.h"
#include "Configuration.h"
#include "Library_Types.h"

//
//	Make an estimation of the heap size based on the hardware and
//	configuration data.  The source of these values input values
//	are distributed across a number of files though primarily the
//	hardware and parameters header files.
//
#define HEAP_SIZE	(BOARD_SRAM-BOARD_REGISTERS-BOARD_STACK-STATIC_VARIABLES )

//
//	The Memory Recovery class
//	=========================
//
//	This virtual class is (can be) used as a doorway into classes
//	that allocate memory from the Memory Heap but are prepared to
//	release any spare memory if the need arises.
//
//	To facilitate this the defined class needs to do two things:
//
//	*	include this virtual class and provide an implementation
//		of the request routine.
//
//	*	register itself with the memory heap so that it can be
//		called at a point of need.
//
class Memory_Recovery {
protected:
	//
	//	The linkage allowing for recovery enabled classes to
	//	be linked.
	//
	Memory_Recovery		*_next;

public:
	//
	//	The memory reclamation API.
	//	---------------------------
	//
	//	These are a set of calls which, when supported by a class,
	//	allow the Heap system to claw back saved memory blocks
	//	using a controlled mechanism (as opposed to just telling
	//	all classes to return everything).
	//

	//
	//	Return the number of bytes memory being "cached" and
	//	available for release if required.  This is a statistical
	//	call to allow tracking of memory usage.
	//
	virtual size_t cache_memory( void ) = 0;

	//
	//	Tell the object to clear all cached memory and release it
	//	to the heap.
	//
	virtual bool clear_cache( void ) = 0;

	//
	//	Ask the object how much memory, as a single block, it
	//	would release to satisfy a specified allocation request.
	//	Return 0 if this object cannot satisfy the request.
	//
	virtual size_t test_cache( size_t bytes ) = 0;

	//
	//	Request that an object release, as a single block,
	//	enough memory to cover the specified allocation.
	//	Return true on success, false on failure.
	//
	virtual bool release_cache( size_t bytes ) = 0;
	
	//
	//	These routines provide the API for the Memory Heap code
	//	to manage the object list.
	//
	Memory_Recovery *linkup( Memory_Recovery *list );
	Memory_Recovery *next( void );
};

//
//	The Memory Heap class.
//	======================
//
//	The Heap itself will be managed in the most simple minded
//	manner possible.  This is specifically to promote a solution
//	which is simple to write and so (hopefully) simple to
//	prove correct.  The solution should have a low memory over
//	head at the expense of a high CPU overhead working on the
//	assumption that heap activity will be low volume and
//	predominantly (if not entirely) on an allocate once basis.
//
//	Memory Heap data memory structure:
//
//	Heap->
//	0						size-1
//	Allocation	Allocation
//	[[size][data]]	[[size][data]]	... [0]
//
//	Where:
//
//		size	The number of heap "units" in the allocation
//			including the size.
//
//		data	The actual memory area handed over to the calling
//			code.
//
class Memory_Heap {
public:
	//
	//	Define the base type which the heap system works with
	//	internally.  This is expected to be an unsigned integer
	//	container capable of containing a pointer value.
	//
	//	This does not impact the callers use of this object.
	//
#ifdef ARDUINO
#if HEAP_SIZE < 128
	typedef byte storage_unit;
#else
	typedef word storage_unit;
#endif
#else
#if HEAP_SIZE < 32768
	typedef word storage_unit;
#else
	typedef dword storage_unit;
#endif
#endif

private:
	//
	//	Declare the total size of the heap area in terms of the
	//	storage unit being used.  This means that we have to
	//	divide the specified heap size by the sizeof the storage
	//	unit.
	//
	static const storage_unit	size = HEAP_SIZE / sizeof( storage_unit );
	
	//
	//	The memory area we will manage.  This is a fixed area of
	//	memory which will be the source of dynamically allocated
	//	memory.
	//
	storage_unit			_heap[ size ];
	
	//
	//	For statistical purposes we will keep a running track on
	//	the available space in the heap.
	//
	storage_unit			_free;

	//
	//	The list of memory users which have registered for the
	//	memory recovery facility.
	//
	Memory_Recovery			*_recovery;

	//
	//	Define a number of constants based on the storage_unit type.
	//
	//	We can use the top bit safely as we are always working
	//	in units of "storage_unit", a data item declared to be
	//	no less than *twice* the size of the whole heap area.
	//
	//	The "top bit" flag is set when a block is deallocated
	//	and empty when the block is allocated.
	//
	static const storage_unit heap_flag	= (storage_unit)1 << (( sizeof( storage_unit ) << 3 ) -1 );
	static const storage_unit heap_data	= heap_flag -1;
	//
	static const storage_unit empty		= (storage_unit)0;
	//
	static const storage_unit invalid	= ~empty;
	//
	static const storage_unit rounding	= sizeof( storage_unit ) -1;
	
	//
	//	Define some simple "bit twiddling" functions which the
	//	heap code will require to manage the heap records.
	//
	static inline bool is_flag_set( storage_unit v ) { return( BOOL( v & heap_flag )); }
	static inline storage_unit set_flag( storage_unit v ) { return( v | heap_flag ); }
	static inline storage_unit reset_flag( storage_unit v ) { return( v & heap_data ); }
	static inline storage_unit block_size( storage_unit v ) { return( v & heap_data ); }

	//
	//	Allocation verification routine.
	//
	//	Returns true if the index is referencing a valid block,
	//	allocated or deallocated, or false if there is an issue
	//	with the index.
	//
	bool valid( storage_unit index );
	
	//
	//	Free space location routine
	//
	//	Find the best fit deallocated block and return its index
	//	or return the index of the first "never allocated"
	//	piece of heap memory.
	//
	storage_unit find( storage_unit rqd );

public:
	//
	//	The constructor
	//	===============
	//
	Memory_Heap( void );
	
	//
	//	The memory allocation routine, argument is in bytes.
	//
	void *alloc( size_t rqst );

	//
	//	The memory deallocation routine.
	//
	void free( void *block );

	//
	//	Erase the heap memory, make ready for a new cycle of
	//	allocations etc.
	//
	void erase( void );
	
	//
	//	Return the amount of free space in the heap, though not
	//	necessarily the largest free space.
	//
	size_t free_memory( void );
	
	//
	//	Return the largest memory block available.  This might be
	//	the same as the value free_memory() returns, but is in
	//	no garanteed to be so.
	//
	//	This routine will be slow, so do not include in any time
	//	critical activity or use with any real frequency.
	//
	size_t free_block( void );

	//
	//	Calculate the amount of memory being held "in cache" by
	//	objects in the system.
	//
	size_t cache_memory( void );

	//
	//	Classes which are prepared to respond to the memory
	//	recovery request need to register themselves through
	//	this API call.
	//
	void recover_from( Memory_Recovery *user );
};

//
//	Declare the replacements to the "built in" new and delete operators.
//
extern void *operator new( size_t bytes );
extern void operator delete( void *ptr );

//
//	Declare the memory heap itself.
//
extern Memory_Heap heap;




#endif

//
//	EOF
//

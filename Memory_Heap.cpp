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


#include "Memory_Heap.h"
#include "Errors.h"
#include "Code_Assurance.h"
#include "Trace.h"
#ifdef DEBUGGING_ENABLED
#include "Console.h"
#endif

//
//	Allocation verification routine.
//
//	Returns true if the index is referencing a valid block,
//	allocated or de-allocated, or false if there is an issue
//	with the index.
//
bool Memory_Heap::valid( storage_unit index ) {

	STACK_TRACE( "bool Memory_Heap::valid( storage_unit index )" );

	TRACE_HEAP( console.print( F( "MHP valid " )));
	TRACE_HEAP( console.print( (word)index ));

	for( storage_unit look = 0; _heap[ look ]; look += block_size( _heap[ look ])) {
		if( look == index ) {
			
			TRACE_HEAP( console.println( F( " Yes" )));
			
			return( true );
		}
	}
	
	TRACE_HEAP( console.println( F( " No" )));
	
	return( false );
}
//
//	Space location routine
//
//	Find the best fit de-allocated block and return its index
//	or return the index of the first "never allocated"
//	piece of heap memory.
//
//	Yes - this is very slow - we are not meant to be doing
//	this that often.
//
Memory_Heap::storage_unit Memory_Heap::find( storage_unit rqd ) {

	STACK_TRACE( "storage_unit Memory_Heap::find( storage_unit rqd )" );

	TRACE_HEAP( console.print( F( "MHP find " )));
	TRACE_HEAP( console.println( (word)rqd ));

	storage_unit	look,
			sz,
			nxt,
			bst,
			fnd;

	look = 0;
	bst = invalid;
	fnd = 0;
	while(( sz = _heap[ look ])) {
		if( is_flag_set( sz )) {
			//
			//	Found a block with the flag set
			//	(de-allocated), so extract the size
			//	to make testing and moving on simpler.
			//
			sz = block_size( sz );
			//
			//	We will do some space consolidation
			//	here by looking ahead to see if the
			//	next block can be joined into this
			//	block.
			//
			//	This loop will completely consolidate
			//	all consecutive deallocated blocks in
			//	the heap so no other approach is
			//	required.
			//
			while( is_flag_set( nxt = _heap[ look + sz ])) {
				//
				//	Yes, so do this now as it does
				//	not cost much at this point in time.
				//
				_heap[ look ] = set_flag(( sz += block_size( nxt )));
			}
			//
			//	If this is large enough but smaller
			//	than previous best, then this is
			//	the new best.
			//
			if(( sz >= rqd )&&( sz < bst )) {
				bst = sz;
				fnd = look;
				if( bst == rqd ) break;
			}
		}
		look += sz;
	}
	return(( bst != invalid )? fnd: look );
}

//
//	The constructor
//	===============
//
Memory_Heap::Memory_Heap( void ) {
	//
	//	Initialise heap space as empty.
	//
	for( storage_unit i = 0; i < size; _heap[ i++ ] = 0 );
	_free = size;
	//
	//	Done
	//
};

//
//	The memory allocation routine, argument is in bytes.
//
void *Memory_Heap::alloc( size_t rqst ) {

	STACK_TRACE( "void *Memory_Heap::alloc( size_t rqst )" );

	TRACE_HEAP( console.print( F( "MHP alloc " )));
	TRACE_HEAP( console.println( (word)rqst ));

	storage_unit	rqd,
			fnd,
			sz;

	//
	//	Space required is that for the allocation itself
	//	plus one for the length information all converted
	//	to a number of "storage_unit" data items.
	//
	rqd = (( rqst + rounding ) / sizeof( storage_unit )) + 1;
	
	//
	//	Generally, do we consider this a valid request?
	//
	if(( rqd < 2 )||( rqd >= size )) {
		errors.log_error( HEAP_ERR_INVALID_ALLOCATION, (word)rqd );
		return( NIL( void ));
	}
	
retry_alloc:
	//
	//	Can we find something already de-allocated?
	//
	if(( sz = block_size( _heap[ fnd = find( rqd )]))) {
		//
		//	Found an existing, but de-allocated, allocation.
		//
		//	This is the "least waste" whole block which can
		//	be reused.  If we are using less than half of it
		//	then we will break the block up.
		//
		// JEFF
		//	With the introduction of the recovery mechanism
		//	this approach of *deliberately* over allocating
		//	memory to avoid creating massive fragmentation
		//	might be a poor choice of algorithm.
		//
		//	Need to think this through; my immediate thought
		//	is that only memory which is to be actually used
		//	should be released and even the smallest unused
		//	piece should be kept int he free list.
		//
		if( rqd < ( sz >> 1 )) {
			//
			//	Sub divide the space.
			//
			_heap[ fnd + rqd ] = set_flag( sz - rqd );
			_heap[ fnd ] = reset_flag( rqd );
			_free -= rqd;
		}
		else {
			//
			//	Allocate whole space as once piece.
			//
			_heap[ fnd ] = reset_flag( sz );
			_free -= sz;
		}
	}
	else {
		//
		//	Found the end of the allocated blocks (size == 0).
		//
		//	Is there space for this block?
		//
		//	There needs to be at least enough space for the
		//	required block PLUS one for the "0" at the end
		//	of the block list.
		//
		if( rqd >= ( size - fnd )) {
			//
			//	There is not enough space for the allocation.
			//	lets try to release some memory and try again
			//	if that is successful.
			//
			if( _recovery->request_recovery()) goto retry_alloc;

			//
			//	No recovery successful, produce the error.
			//
			errors.log_error( HEAP_ERR_OUT_OF_MEMORY, (word)rqd );

			TRACE_HEAP( console.println( F( "MHP ret 0" )));

			return( NIL( void ));
		}
		//
		//	Allocation fits so we over write the zero length
		//	at the end of the allocations, and write a new one
		//	further down the memory.
		//
		_heap[ fnd ] = rqd;
		_heap[ fnd + rqd ] = 0;
		_free -= rqd;
	}
	//
	//	Return the allocated memory remembering that the caller
	//	is only interested in the memory after the length data.
	//
	
	TRACE_HEAP( console.print( F( "MHP ret " )));
	TRACE_HEAP( console.println( (word)fnd ));
	
	return( (void *)&( _heap[ fnd + 1 ]));
};

//
//	The memory de-allocation routine.
//
void Memory_Heap::free( void *block ) {

	STACK_TRACE( "void Memory_Heap::free( void *block )" );

	storage_unit	look;

	if( valid( look = ((storage_unit *)block - _heap ) - 1 )) {

		TRACE_HEAP( console.print( F( "MHP free " )));
		TRACE_HEAP( console.println( (word)look ));
		
		storage_unit	sz;
		
		if( is_flag_set( sz = _heap[ look ])) {
			//
			//	Cannot release what has already been released.
			//
			errors.log_error( HEAP_ERR_DUP_DEALLOCATE, look );
		}
		else {
			//
			//	Now, just mark as released/free.
			//
			//	We could enter into a minor consolidation of free
			//	space at this point, but the find() routine does
			//	a completely effective consolidation every time
			//	it is called in alloc(). 
			//
			_heap[ look ] = set_flag( sz );
			_free += sz;
		}
	}
	else {

		TRACE_HEAP( console.print( F( "MHP invalid " )));
		TRACE_HEAP( console.println( (word)look ));
		
		errors.log_error( HEAP_ERR_INVALID_ADRS, look );
	}
};

//
//	Erase the heap memory, make ready for a new cycle of
//	allocations etc.
//
void Memory_Heap::erase( void ) {

	STACK_TRACE( "void Memory_Heap::erase( void )" );

	//
	//	Simply set the whole heap to zeros.
	//
	for( storage_unit i = 0; i < size; _heap[ i++ ] = 0 );
	_free = size;
}

//
//	Return the amount of free space in the heap, though not
//	necessarily the largest free space.
//
Memory_Heap::storage_unit Memory_Heap::free_memory( void ) {

	STACK_TRACE( "Memory_Heap::storage_unit Memory_Heap::free_memory( void )" );

	//
	//	Remember to deduct 1 from the free count as
	//	there is a managment value to account for.
	//
	return(( _free - 1 ) * sizeof( storage_unit ));
}

//
//	Return the largest memory block available.  This might be
//	the same as the value free_memory() returns, but is in
//	no garanteed to be so.
//
//	This routine will be slow, so do not include in any time
//	critical activity or use with any real frequency.
//
Memory_Heap::storage_unit Memory_Heap::free_block( void ) {

	STACK_TRACE( "Memory_Heap::storage_unit Memory_Heap::free_block( void )" );

	storage_unit	best, look, test;

	//
	//	Start at the beginning with no best value.
	//	
	best = 0;
	look = 0;
	//
	//	Step through the allocations checking status and size.
	//
	while(( test = _heap[ look ])) {
		//
		//	Free block?
		//
		if( is_flag_set( test )) {
			//
			//	Remove flag and compare.
			//
			test = reset_flag( test );
			if( test > best ) best = test;
		}
		//
		//	move on to next block.
		//
		look += test;
	}
	//
	//	Finally check the free space at the end of the memory.
	//
	if(( test = size - look ) > best ) best = test;
	//
	//	Remember to deduct 1 from the best count as
	//	there is a managment value to allow for.
	//
	return(( best - 1 ) * sizeof( storage_unit ));
}

//
//	Classes which are prepared to respond to the memory
//	recovery request need to register themselves through
//	this API call.
//
void Memory_Heap::recover_from( Memory_Recovery *user ) {

	STACK_TRACE( "void Memory_Heap::recover_from( Memory_Recovery *user )" );

	ASSERT( user != NIL( Memory_Recovery ));

	_recovery = user->linkup( _recovery );
}


//
//	Declare the replacements to the "built in" new and delete operators.
//
void *operator new( size_t bytes ) {

	STACK_TRACE( "void *operator new( size_t bytes )" );

	return( heap.alloc( bytes ));
}
void operator delete( void *ptr ) {

	STACK_TRACE( "void operator delete( void *ptr )" );

	heap.free( ptr );
}


//
//	Declare the memory heap itself.
//
Memory_Heap heap;


//
//	EOF
//

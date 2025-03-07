//
//	Byte_Queue - implement a simple byte based queue.
//

//
//	This is the generic API these are based on.
//
#include "Byte_Queue.h"

//
//	Constructor only.
//
Byte_Queue::Byte_Queue( void ) {
	//
	//	Queue indexes and length
	//
	_in = 0;
	_out = 0;
	_content = 0;
	_ready = NIL( Signal );

	//
	//	Invalid set up of pointers, as these must
	//	be initialised explicitly in the ...
	//	initialise() routine.
	//
	_queue = _tail = _free = NIL( queue_block );

}

//
//	This is the initialisation routine that must be called
//	before the queue can be used.  The argument can be either
//	NIL( Signal ) for no data ready signals or the address
//	of a signal to be released on data being ready.
//
void Byte_Queue::initialise( Signal *ready ) {

	STACK_TRACE( "void Byte_Queue::initialise( void )" );

	ASSERT( _queue == NIL( queue_block ));
	ASSERT( _tail == NIL( queue_block ));
	ASSERT( _free == NIL( queue_block ));

	//
	//	Remember the data ready signal.
	//
	_ready = ready;

	//
	//	Pre-allocate the essential blocks.
	//
	if(!( _tail = new queue_block )) ABORT( QUEUE_ALLOCATION_FAILED );
	_tail->next = NIL( queue_block );
	//
	if(!( _free = new queue_block )) ABORT( QUEUE_ALLOCATION_FAILED );
	_free->next = NIL( queue_block );
	//
	_queue = _tail;

	//
	//	Link in the memory allocation service
	//
	task_manager.add_task( this, &_flag, allocate_block );

	//
	//	And also the heap management service.
	//
	heap.recover_from( this );
}

//
//	This processing routine is called only when there
//	has been a request (from inside a Critical code
//	section) for an additional queue_block to be allocated.
//
void Byte_Queue::process( UNUSED( byte handle )) {

	STACK_TRACE( "void Byte_Queue::process( byte handle )" );

	//
	//	We don't add a free block if there is all ready
	//	one in the free list.
	//
	if( _free == NIL( queue_block )) {
		queue_block	*ptr;

		if(( ptr = new queue_block )) {
			Critical code;
			
			ptr->next = NIL( queue_block );
			_free = ptr;
		}
		else {
			errors.log_error( QUEUE_ALLOCATION_FAILED, 0 );
		}
	}
}

//
//	The Byte Queue API
//	==================
//
//	The method for writing to the queue depends to some
//	extent if we are already in an ISR or Critical code
//	section.
//
bool Byte_Queue::write( byte data ) {

	STACK_TRACE( "bool Byte_Queue::write( byte data )" );

	Critical code;

	//
	//	If _in is at block size we try to extend the
	//	queue with a block from the free list.
	//
	if( _in == block_size ) {
		if( _free == NIL( queue_block )) {
			queue_block	*free;
			
			//
			//	We need another queue_block, but the free
			//	list is empty.
			//
			//	At this point if we were already in a
			//	critical mode (an ISR for example) before
			//	calling this then there is noting we can do.
			//
			if( code.was_critical()) {
				_flag.release( true );
				return( false );
			}
			//
			//	We know the code calling us *was* normal
			//	code so we know we can suspend the Critical
			//	nature of this for enough time to allocate
			//	some memory.
			//
			{
				Normal code;

				free = new queue_block;
			}
			//
			//	Now we link in the free block we obtained
			//	from the memory allocation into the free list.
			//	(assuming that worked!)
			//
			if( free == NIL( queue_block )) return( false );
			free->next = (queue_block *)_free;
			_free = free;
		}

		//
		//	We believe, and intend, that the block pointed
		//	to by tail is always the last.
		//
		ASSERT( _tail->next == NIL( queue_block ));
		
		//
		//	1  Append the _free list after the _tail block.
		//	2  Move _free list on and raise the _flag signal if empty.
		//	3  Move _tail pointer to new last block.
		//	4  Reset _in to start of the block queue.
		//
		_tail->next = (queue_block *)_free;
		if(!( _free = _free->next )) _flag.release( true );
		_tail = _tail->next;
		_tail->next = NIL( queue_block );

		_in = 0;
	}
	//
	//	We add the data to the tail block.
	//
	_tail->queue[ _in++ ] = data;
	_content++;

	//
	//	Signal the arrival of data?
	//
	if( _ready ) _ready->release();
	
	//
	//	Done.
	//
	return( true );
}

byte Byte_Queue::read( void ) {

	STACK_TRACE( "byte Byte_Queue::read( void )" );

	Critical	code;
	
	if( _content ) {
		byte	data;

		//
		//	There is data so get it.
		//
		data = _queue->queue[ _out++ ];
		_content--;
		//
		//	Do we need to roll into the
		//	next block?
		//
		if( _out == block_size ) {
			//
			//	We are rolling out of this block.
			//
			//	If there is no more data (_content
			//	== 0 ) then we can reset both _in
			//	and _out to the start of this block.
			//
			//	If there is more data we will release
			//	this block to the free list.
			//
			if( _content ) {
				queue_block	*ptr;
				
				//
				//	If we have run off this block
				//	and content is non-zero then
				//	we must have another block so
				//	move on and free this block.
				//
				ASSERT( _queue != _tail );

				ptr = (queue_block *)_queue;
				_queue = ptr->next;
				 ptr->next = (queue_block *)_free;
				_free = ptr;

				_out = 0;
			}
			else {
				//
				//	should be just a single block
				//	so we can push _in and _out to
				//	the start of it.
				//
				ASSERT( _queue == _tail );

				_in = _out = 0;
			}
		}
		//
		//	Done.
		//
		return( data );
	}
	return( 0 );
}

//
//	Perform a "reset" of the underlying system.  This
//	is used only to recover from an unknown condition
//	with the expectation that upon return the queue
//	can be reliably used.
//
void Byte_Queue::reset( void ) {

	STACK_TRACE( "void Byte_Queue::reset( void )" );

	Critical code;
	
	queue_block	*ptr;

	//
	//	Empty *everything* into the free list.  This
	//	*must* be at least a single queue block.
	//
	while(( ptr = (queue_block *)_queue )) {
		_queue = ptr->next;
		ptr->next = (queue_block *)_free;
		_free = ptr;
	}

	//
	//	The pull one back, there should *always*
	//	be a single block allocated in the queue
	//	(as we should have just put it in there!)
	//
	ASSERT( _free != NIL( queue_block ));

	_tail = _queue = _free;
	_free = _free->next;
	_tail->next = NIL( queue_block );
	
	//
	//	Queue indexes and length
	//
	_in = 0;
	_out = 0;
	_content = 0;
}

byte Byte_Queue::space( void ) {

	STACK_TRACE( "byte Byte_Queue::space( void )" );

	if( _free ) return( block_size );
	return( block_size - _in );
}

byte Byte_Queue::available( void ) {

	STACK_TRACE( "byte Byte_Queue::available( void )" );

	return( _content );
}

byte Byte_Queue::pending( void ) {

	STACK_TRACE( "byte Byte_Queue::pending( void )" );

	return( _content );
}

//
//	The memory reclamation API.
//	---------------------------
//

//
//	Return the number of bytes memory being "cached" and
//	available for release if required.  This is a statistical
//	call to allow tracking of memory usage.
//
size_t Byte_Queue::cache_memory( void ) {

	STACK_TRACE( "size_t Byte_Queue::cache_memory( void )" );

	word total = 0;
	if( _free ) {
		for( queue_block *look = _free->next; look != NIL( queue_block ); look = look->next ) {
			total += sizeof( queue_block );
		}
	}
	return( total );
}

//
//	Tell the object to clear all cached memory and release it
//	to the heap.
//
bool Byte_Queue::clear_cache( void ) {

	STACK_TRACE( "bool Byte_Queue::clear_cache( void )" );

	queue_block	*ptr;
	bool		r;

	r = false;
	if( _free ) {
		while( _free->next ) {
			ptr = (queue_block *)_free;
			_free = ptr->next;
			delete ptr;
			r = true;
		}
	}
	return( r );
}

//
//	Ask the object how much memory, as a single block, it
//	would release to satisfy a specified allocation request.
//	Return 0 if this object cannot satisfy the request.
//
size_t Byte_Queue::test_cache( size_t bytes ) {

	STACK_TRACE( "size_t Byte_Queue::test_cache( size_t bytes )" );

	if(( sizeof( queue_block ) >= bytes ) && _free && _free->next ) return( sizeof( queue_block ));
	return( 0 );
}

//
//	Request that an object release, as a single block,
//	enough memory to cover the specified allocation.
//	Return true on success, false on failure.
//
bool Byte_Queue::release_cache( size_t bytes ) {

	STACK_TRACE( "bool Byte_Queue::release_cache( size_t bytes )" );

	if( _free ) {
		if( _free->next &&( sizeof( queue_block ) >= bytes )) {
			queue_block *ptr;

			ptr = (queue_block *)_free;
			_free = ptr->next;
			delete ptr;

			return( true );
		}
	}
	return( false );
}

//
//	EOF
//

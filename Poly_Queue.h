//
//	Poly_Queue.h
//	============
//
//	Define a simple queue systems for basic types of any sort.
//

#ifndef _POLY_QUEUE_H_
#define _POLY_QUEUE_H_

#include "Environment.h"
#include "Parameters.h"
#include "Configuration.h"
#include "Critical.h"


template< class TYPE, byte size >
class Poly_Queue {
private:
	//
	//	save our buffer size.
	//
	static const byte	queue_size = size;

	//
	//	Here is the queue.
	//
	TYPE			_queue[ queue_size ];
	byte			_in,
				_out,
				_size;

public:
	//
	//	Construct as empty.
	//
	Poly_Queue( void ) {
		_in = 0;
		_out = 0;
		_size = 0;
	}

	//
	//	Add to the queue
	//
	bool write( TYPE data ) {
		Critical code;
		
		if( _size >= queue_size ) return( false );
		_queue[ _in++ ] = data;
		if( _in >= queue_size ) _in = 0;
		_size++;
		return( true );
	}

	//
	//	Read from the queue
	//
	bool read( TYPE *adrs ) {
		Critical code;
		
		if( _size == 0 ) return( false );
		*adrs = _queue[ _out++ ];
		if( _out >= queue_size ) _out = 0;
		_size--;
		return( true );
	}

	//
	//	A simple non intrusive test.
	//
	byte available( void ) {
		return( _size );
	}
};

#endif

//
//	EOF
//

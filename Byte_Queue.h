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
#include "Critical.h"
#include "Signal.h"

//
//	We need the API for the base.
//
#include "Byte_Queue_API.h"


//
//	Define a template class where the argument passed in is
//	the size of the buffer area.
//
//	We use a template class as the queue sizes are statically
//	defined at compile time and avoid the requirement for using
//	heap memory and unnecessary double indirection.
//
template< byte QUEUE_SIZE >
class Byte_Queue : public Byte_Queue_API {
	private:
		//
		//	Declare a constant for the size of the queue area.
		//
		static const byte	queue_size = QUEUE_SIZE;
		
		//
		//	Create the memory that is used for the queue area
		//
		byte			_queue[ queue_size ];
		
		//
		//	Define where the data in added and removed and
		//	how many bytes are in the queue.
		//
		volatile byte		_in,
					_out,
					_content;
		
	public:
		//
		//	Constructor only.
		//	
		Byte_Queue( void ) {
			//
			//	Queue indexes and length
			//
			_in = 0;
			_out = 0;
			_content = 0;
		}
		//
		//	The Byte Queue API
		//	==================
		//
		
		virtual bool write( byte data ) {
			Critical	code;

			//
			//	Is there space for the additional byte?
			//
			if( _content >= queue_size ) return( false );
			//
			//	There is sufficient space.
			//
			_queue[ _in++ ] = data;
			if( _in >= queue_size ) _in = 0;
			_content++;
			//
			//	Done.
			//
			return( true );
		}

		virtual byte read( void ) {
			Critical	code;
			
			if( _content ) {
				byte	data;

				//
				//	There is data so get it.
				//
				data = _queue[ _out++ ];
				if( _out >= queue_size ) _out = 0;
				_content--;
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
		virtual void reset( void ) {
			Critical code;
			
			//
			//	Queue indexes and length
			//
			_in = 0;
			_out = 0;
			_content = 0;
		}

		virtual byte space( void ) {
			return( queue_size - _content );
		}

		virtual byte available( void ) {
			return( _content );
		}

		virtual byte pending( void ) {
			return( _content );
		}

};

//
//	Define a template class similar to the above class but including
//	the use of a Signal object to control data collection.
//
template< byte QUEUE_SIZE >
class Byte_Queue_Signal : public Byte_Queue_API {
	private:
		//
		//	Declare a constant for the size of the queue area.
		//
		static const byte	queue_size = QUEUE_SIZE;
		
		//
		//	Create the memory that is used for the queue area
		//
		byte			_queue[ queue_size ];
		
		//
		//	Define where the data in added and removed and
		//	how many bytes are in the queue.
		//
		volatile byte		_in,
					_out,
					_content;

		//
		//	Declare the Signal we will be using to control
		//	reading from the queue.
		//
		Signal			_gate;
		
	public:
		//
		//	Constructor only.
		//	
		Byte_Queue_Signal( void ) {
			//
			//	Queue indexes and length
			//
			_in = 0;
			_out = 0;
			_content = 0;
		}
		//
		//	The Byte Queue API
		//	==================
		//
		
		virtual bool write( byte data ) {
			Critical	code;

			//
			//	Is there space for the additional byte?
			//
			if( _content >= queue_size ) return( false );
			//
			//	There is sufficient space.
			//
			_queue[ _in++ ] = data;
			if( _in >= queue_size ) _in = 0;
			_content++;
			//
			//	Announce extra data.
			//
			_gate.release();
			//
			//	Done.
			//
			return( true );
		}

		virtual byte read( void ) {
			Critical	code;
			
			if( _content ) {
				byte	data;

				//
				//	There is data so get it.
				//
				data = _queue[ _out++ ];
				if( _out >= queue_size ) _out = 0;
				_content--;
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
		virtual void reset( void ) {
			Critical code;
			
			//
			//	Queue indexes and length
			//
			_in = 0;
			_out = 0;
			_content = 0;
		}
		virtual byte space( void ) {
			return( queue_size - _content );
		}

		virtual byte available( void ) {
			return( _content );
		}

		virtual byte pending( void ) {
			return( _content );
		}

		//
		//	For this version of the class we allow
		//	access to the gate
		//
		Signal *control_signal( void ) {
			return( &_gate );
		}

};

#endif

//
//	EOF
//

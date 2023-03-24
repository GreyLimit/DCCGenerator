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
#include "Critical.h"

//
//	Define the virtualised Byte Queue API which can be used
//	as a generic access mechanism to a byte queue of any size.
//
class Byte_Queue_API {
	private:
		//
		//	Define a number buffer size.
		//
		//	Note this value MUST be at least as large
		//	as the maximum number of decimal digits
		//	it takes to display a maximal word value.
		//
		//	This is 65535, so five is the minimum value.
		//
		static const byte number_buffer = 6;
		
	public:
		//
		//	Here are the standard IO routines into
		//	a byte queue class.
		//
		
		//
		//	Insert new data byte into queue.
		//
		virtual bool write( byte data ) = 0;

		//
		//	Return the next available byte; use available()
		//	to determine if there is data pending.
		//
		virtual byte read( void ) = 0;

		//
		//	Return the available capacity in the buffer
		//
		virtual data_size space( void ) = 0;

		//
		//	Return the number of available bytes.
		//
		virtual data_size available( void ) = 0;

		//
		//	Data Output Support
		//	===================
		//

		//
		//	For the moment the following routines are
		//	provided here as a mechanism for creating
		//	textual representations of basic data types.
		//
		//	This is probably not the right place for
		//	this, but fits with the console device
		//	being based on this API, for the moment.
		//
		//	I will need to implement some form of blocking
		//	output here as printing too much at once
		//	simply drops anything which does not fit the
		//	buffer/queue.
		//
		bool print( char c ) {
			return( write( c ));
		}
		
		bool println( void ) {
			return( print( '\r' ) && print( '\n' ));
		}

		bool print( byte b ) {
			return( print( (word)b ));
		}

		bool print_nybble( byte b ) {
			b &= 0x0f;
			if( b < 10 ) {
				return( print((char)( '0' + b )));
			}
			else {
				return( print((char)(( 'A' -10 ) + b )));
			}
			return( false );
		}
		
		bool print_hex( byte b ) {
			return( print_nybble( b >> 4 ) && print_nybble( b ));
		}
		
		bool print( bool b ) {
			return( print( b? 'T': 'F' ));
		}
		
		bool print( word w ) {
			if( w ) {
				char	b[ number_buffer ];
				byte	i;

				for( i = 0; w; i++ ) {
					b[ i ] = '0' + ( w % 10 );
					w /= 10;
				}
				while( i ) if( !print( b[ --i ])) return( false );
				return( true );
			}
			return( print( '0' ));
		}
		
		bool print_hex( word w ) {
			return( print_hex((byte)( w >> 8 )) && print_hex((byte)( w & 0xff )));
		}
		
		bool print( int i ) {
			if( i < 0 ) {
				if( !print( '-' )) return( false );
				i = -i;
			}
			return( print( (word)i ));
		}
		bool println( int i ) {
			return( print( i ) && println());
		}
		
		bool print( const char *s ) {
			while( *s != EOS ) if( !write( *s++ )) return( false );
			return( true );
		}
		
		bool println( const char *s ) {
			return( print( s ) && println());
		}

		//
		//	The following command supports printing
		//	directly out of program memory.
		//
		void print_PROGMEM( const char *pm ) {
			char	c;

			while(( c = progmem_read_byte_at( pm++ )) != EOS ) if( !print( c )) return;
		}
};

//
//	Define a template class where the argument passed in is
//	the size of the buffer area.
//
//	We use a template class as the queue sizes are statically
//	defined at compile time and avoid the requirement for using
//	heap memory and unnecessary double indirections.
//
template< data_size QUEUE_SIZE >
class Byte_Queue : public Byte_Queue_API {
	private:
		//
		//	Declare a constant for the size of the queue area.
		//
		static const data_size	queue_size = QUEUE_SIZE;
		
		//
		//	Create the memory that is used for the queue area
		//
		byte			_queue[ queue_size ];
		
		//
		//	Define where the data in added and removed and
		//	how many bytes are in the queue.
		//
		volatile data_size	_in,
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

		virtual data_size space( void ) {
			return( queue_size - _content );
		}

		virtual data_size available( void ) {
			return( _content );
		}

};
				
#endif

//
//	EOF
//

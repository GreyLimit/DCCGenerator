//
//	Byte_Queue_API.h
//	================
//
//	Declare the universal interface to a queue for bytes.
//

#ifndef _BYTE_QUEUE_API_H_
#define _BYTE_QUEUE_API_H_

//
//	Bring in the Instrumentation package to facilitate
//	data collection and tracking.
//
#include "Environment.h"
#include "Parameters.h"
#include "Configuration.h"
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

		//
		//	Implementation of the "cooked" synchronous mode.
		//
		bool		_synchronous;
		
	public:

		//
		//	Constructor.
		//
		Byte_Queue_API( void ) {
			_synchronous = false;
		}

		//
		//	Control the synchronous flag.
		//
		bool synchronous( bool on ) {
			bool	prev;

			prev = _synchronous;
			_synchronous = on;
			return( prev );
		}
	
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
		virtual byte space( void ) = 0;

		//
		//	Return the number of available bytes.
		//
		virtual byte available( void ) = 0;

		//
		//	Return the number of bytes pending in the buffer.
		//	For "looped back" or simple "internal" buffers
		//	this will be the same value as the "available()"
		//	bytes.
		//
		//	However, for devices that are presented as a buffer
		//	(bi-directional devices) this will return the number
		//	of data bytes still awaiting (pending) being sent.
		//
		virtual byte pending( void ) = 0;

		//
		//	Perform a "reset" of the underlying system.  This
		//	is used only to recover from an unknown condition
		//	with the expectation that upon return the queue
		//	can be reliably used.
		//
		virtual void reset( void ) = 0;

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
		bool print( char c ) {
			if( _synchronous && Critical::normal_code()) while( space() == 0 );
			return( write( c ));
		}
		
		bool println( void ) {
			return( print( '\r' ) && print( '\n' ));
		}
		
		bool println( char c ) {
			return( print( c ) && println());
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
		
		bool println_hex( byte b ) {
			return( print_nybble( b >> 4 ) && print_nybble( b ) && println());
		}
		
		bool print_hex( word w ) {
			return( print_hex((byte)( w >> 8 )) && print_hex((byte)( w & 0xff )));
		}
		
		bool println_hex( word w ) {
			return( print_hex( w ) && println());
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
		
		bool println( word w ) {
			return( print( w ) && println());
		}
		
		bool print( byte b ) {
			return( print( (word)b ));
		}
		bool println( byte b ) {
			return( print( (word)b ) && println());
		}

		bool print( sbyte b ) {
			return( print( (int)b ));
		}
		bool println( sbyte b ) {
			return( print( (int)b ) && println());
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
			while( *s != EOS ) if( !print( *s++ )) return( false );
			return( true );
		}
		
		bool println( const char *s ) {
			return( print( s ) && println());
		}

		bool print( const char *s, byte l ) {
			if( l > space()) return( false );
			while( l-- ) print( *s++ );
			return( true );
		}

		void synchronise( void ) {
			//
			//	This is a *dangerous* routine, and should
			//	(in the normal course of events) not be
			//	called.  This routine will stall the firmware
			//	until the underlying "pending" data has
			//	been cleared.  If this clearance is not
			//	interrupt driven then this will stop the
			//	firmware dead.
			//
			if( _synchronous ) while( pending());
		}

		//
		//	The following command supports printing
		//	directly out of program memory.
		//
		void print_PROGMEM( const char *pm ) {
			char	c;

			while(( c = progmem_read_byte_at( pm++ )) != EOS ) if( !print( c )) return;
		}
		void println_PROGMEM( const char *pm ) {
			print_PROGMEM( pm );
			println();
		}

		//
		//	Provide routines which will support the Arduino
		//	F() macro used to place literal strings into the
		//	PROGMEM area in a programmer friendly manner.
		//
		void print( const __FlashStringHelper *fs ) {
			print_PROGMEM( (const char *)fs );
		}
		void println( const __FlashStringHelper *fs ) {
			println_PROGMEM( (const char *)fs );
		}

		//
		//	Boolean output.
		//
		void print( bool val ) {
			print( val? F( "yes" ): F( "no" ));
		}
		void println( bool val ) {
			print( val );
			println();
		}
};


#endif

//
//	EOF
//

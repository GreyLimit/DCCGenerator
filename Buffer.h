//
//	Buffer.h
//	========
//
//	A set of routines specifically for the encoding of
//	replies through the Serial/USB connection.
//

#ifndef _BUFFER_H_
#define _BUFFER_H_

#include "Environment.h"
#include "Parameters.h"
#include "Configuration.h"
#include "Protocol.h"
#include "Byte_Queue.h"
#include "Trace.h"

//
//	Define a generic Buffer API that will allow the content
//	of a buffer to be extracted from *any* form of Buffer
//	instance (noting that "Buffer" is a template class
//	allowing for different sizes of buffer).
//
class Buffer_API {
public:
	virtual char *buffer( void ) = 0;
	virtual byte size( void ) = 0;
	virtual void copy( char *to, byte len ) = 0;
	virtual bool send( Byte_Queue_API *to ) = 0;
};

//
//	Reply Construction routines.
//	----------------------------
//
//	The following set of routines provide alternatives to
//	using "sprintf()" for filling in a buffer space with
//	a reply from the firmware.
//
//	This has been done to reduce memory usage from the sprintf
//	template strings in exchange for a larger code base (as
//	there is plenty of code flash available but memory is
//	tight on the smaller MCUs).
//
template< byte SIZE >
class Buffer : public Buffer_API {
private:
	//
	//	Declare the size of the internal buffer
	//
	static const byte	buffer_size = SIZE;

	//
	//	Define the maximum number of digits required to convert
	//	an integer value into its text form.
	//
	//	A digit stack of 10 is sufficient for a signed (or
	//	unsigned) 32 bit number conversion.
	//
	static const byte	digit_stack = 10;

	//
	//	This is the internal buffer created.
	//
	char			_buffer[ buffer_size ];

	//
	//	Our pointers into our buffer.
	//
	char			*_ptr;
	byte			_left;

	//
	//	The primitives which are combined to fill in the buffer.
	//
	bool start( char code ) {
		if( _left > 2 ) {
			*_ptr++ = Protocol::lead_in;
			*_ptr++ = code;
			_left -= 2;
			return( true );
		}
		return( false );
	}

	bool end( void ) {
		if( _left > 2 ) {
			*_ptr++ = Protocol::lead_out;
			*_ptr++ = NL;
			*_ptr = EOS;
			_left -= 2;
			return( true );
		}
		return( false );
	}

	bool add( char c ) {
		if( _left > 1 ) {
			*_ptr++ = c;
			_left -= 1;
			return( true );
		}
		return( false );
	}
	
	bool add( int v ) {
		char	r[ digit_stack ];
		byte	c;
		bool	n;

		c = 0;
		if(( n = ( v < 0 ))) v = -v;
		if( v == 0 ) {
			r[ c++ ] = 0;
		}
		else {
			while( v ) {
				r[ c++ ] = v % 10;
				v /= 10;
			}
		}
		if( n ) {
			if( _left < 2 ) return( false );
			*_ptr++ = MINUS;
			_left -= 1;
		}
		if( _left <= c ) return( false );
		_left -= c;
		while( c ) *_ptr++ = '0' + r[ --c ];
		return( true );
	}

	bool add( word v ) {
		char	r[ digit_stack ];
		byte	c;

		c = 0;
		if( v == 0 ) {
			r[ c++ ] = 0;
		}
		else {
			while( v ) {
				r[ c++ ] = v % 10;
				v /= 10;
			}
		}
		if( _left <= c ) return( false );
		_left -= c;
		while( c ) *_ptr++ = '0' + r[ --c ];
		return( true );
	}
	
	bool add( byte v ) {
		char	r[ digit_stack ];
		byte	c;

		c = 0;
		if( v == 0 ) {
			r[ c++ ] = 0;
		}
		else {
			while( v ) {
				r[ c++ ] = v % 10;
				v /= 10;
			}
		}
		if( _left <= c ) return( false );
		_left -= c;
		while( c ) *_ptr++ = '0' + r[ --c ];
		return( true );
	}

	bool add_PROGMEM( const char *str ) {
		char	c;

		while(( c = progmem_read_byte_at( str++ ))) {
			if( !add( c )) return( false );
		}
		return( true );
	}
	
public:
	Buffer( void ) {
		_ptr = _buffer;
		_left = buffer_size;
	}

	bool format( char code, int a1 ) {
		return( start( code ) && add( a1 ) && end());
	}

	bool format( char code, int a1, int a2 ) {
		return( start( code ) && add( a1 ) && add( SPACE ) && add( a2 ) && end());
	}

	bool format( char code, int a1, int a2, int a3 ) {
		return( start( code ) && add( a1 ) && add( SPACE ) && add( a2 ) && add( SPACE ) && add( a3 ) && end());
	}

	bool format( char code, int a1, int a2, const char *a3 ) {
		return( start( code ) && add( a1 ) && add( SPACE ) && add( a2 ) && add( SPACE ) && add_PROGMEM( a3 ) && end());
	}

	virtual char *buffer( void ) {
		return( _buffer );
	}

	virtual byte size( void ) {
		return( buffer_size - _left );
	}

	virtual void copy( char *to, byte len ) {
		byte	s;

		if(( s = size()) < len ) {
			memcpy( to, _buffer, s );
			to[ s ] = EOS;
		}
		else {
			memcpy( to, _buffer, len-1 );
			to[ len-1 ] = EOS;
		}
	}

	virtual bool send( Byte_Queue_API *to ) {
		return( to->print( _buffer, size()));
	}
};

#endif

//
//	EOF
//

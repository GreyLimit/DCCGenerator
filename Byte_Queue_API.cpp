//
//	Byte_Queue_API.cpp
//	==================
//
//	Define the universal interface to a queue for bytes.
//

//
//	Bring in the Instrumentation package to facilitate
//	data collection and tracking.
//
#include "Byte_Queue_API.h"


//
//	Constructor.
//
Byte_Queue_API::Byte_Queue_API( void ) {
	_synchronous = false;
}

//
//	Control the synchronous flag.
//
bool Byte_Queue_API::synchronous( bool on ) {
	bool	prev;

	prev = _synchronous;
	_synchronous = on;
	return( prev );
}


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
bool Byte_Queue_API::print( char c ) {
	if( _synchronous && Critical::normal_code()) while( space() == 0 );
	return( write( c ));
}

bool Byte_Queue_API::println( void ) {
	return( print( '\r' ) && print( '\n' ));
}

bool Byte_Queue_API::println( char c ) {
	return( print( c ) && println());
}

bool Byte_Queue_API::print_nybble( byte b ) {
	b &= 0x0f;
	if( b < 10 ) {
		return( print((char)( '0' + b )));
	}
	else {
		return( print((char)(( 'A' -10 ) + b )));
	}
	return( false );
}

bool Byte_Queue_API::print_hex( byte b ) {
	return( print_nybble( b >> 4 ) && print_nybble( b ));
}

bool Byte_Queue_API::println_hex( byte b ) {
	return( print_nybble( b >> 4 ) && print_nybble( b ) && println());
}

bool Byte_Queue_API::print_hex( word w ) {
	return( print_hex((byte)( w >> 8 )) && print_hex((byte)( w & 0xff )));
}

bool Byte_Queue_API::println_hex( word w ) {
	return( print_hex( w ) && println());
}

bool Byte_Queue_API::print( word w ) {
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

bool Byte_Queue_API::println( word w ) {
	return( print( w ) && println());
}

bool Byte_Queue_API::print( byte b ) {
	return( print( (word)b ));
}
bool Byte_Queue_API::println( byte b ) {
	return( print( (word)b ) && println());
}

bool Byte_Queue_API::print( sbyte b ) {
	return( print( (int)b ));
}
bool Byte_Queue_API::println( sbyte b ) {
	return( print( (int)b ) && println());
}

bool Byte_Queue_API::print( int i ) {
	if( i < 0 ) {
		if( !print( '-' )) return( false );
		i = -i;
	}
	return( print( (word)i ));
}

bool Byte_Queue_API::println( int i ) {
	return( print( i ) && println());
}

bool Byte_Queue_API::print( const char *s ) {
	while( *s != EOS ) if( !print( *s++ )) return( false );
	return( true );
}

bool Byte_Queue_API::println( const char *s ) {
	return( print( s ) && println());
}

bool Byte_Queue_API::print( const char *s, byte l ) {
	if( l > space()) return( false );
	while( l-- ) print( *s++ );
	return( true );
}

void Byte_Queue_API::synchronise( void ) {
	//
	//	This is a *dangerous* routine, and should
	//	not (in the normal course of events) be
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
void Byte_Queue_API::print_PROGMEM( const char *pm ) {
	char	c;

	while(( c = progmem_read_byte_at( pm++ )) != EOS ) if( !print( c )) return;
}
void Byte_Queue_API::println_PROGMEM( const char *pm ) {
	print_PROGMEM( pm );
	println();
}

//
//	Provide routines which will support the Arduino
//	F() macro used to place literal strings into the
//	PROGMEM area in a programmer friendly manner.
//
void Byte_Queue_API::print( const __FlashStringHelper *fs ) {
	print_PROGMEM( (const char *)fs );
}
void Byte_Queue_API::println( const __FlashStringHelper *fs ) {
	println_PROGMEM( (const char *)fs );
}

//
//	Boolean output.
//
void Byte_Queue_API::print( bool val ) {
	print( val? F( "yes" ): F( "no" ));
}
void Byte_Queue_API::println( bool val ) {
	print( val );
	println();
}


//
//	EOF
//

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
	//	The Byte_Queue_API.
	//	===================
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
	//	Constructor.
	//	============
	//
	Byte_Queue_API( void );

	//
	//	Control the synchronous flag.
	//
	bool synchronous( bool on );

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
	bool print( char c );
	bool println( void );
	bool println( char c );
	bool print_nybble( byte b );
	bool print_hex( byte b );
	bool println_hex( byte b );
	bool print_hex( word w );
	bool println_hex( word w );
	bool print( word w );
	bool println( word w );
	bool print( byte b );
	bool println( byte b );
	bool print( sbyte b );
	bool println( sbyte b );
	bool print( int i );
	bool println( int i );
	bool print( const char *s );
	bool println( const char *s );
	bool print( const char *s, byte l );
	void synchronise( void );

	//
	//	The following command supports printing
	//	directly out of program memory.
	//
	void print_PROGMEM( const char *pm );
	void println_PROGMEM( const char *pm );

	//
	//	Provide routines which will support the Arduino
	//	F() macro used to place literal strings into the
	//	PROGMEM area in a programmer friendly manner.
	//
	void print( const __FlashStringHelper *fs );
	void println( const __FlashStringHelper *fs );

	//
	//	Boolean output.
	//
	void print( bool val );
	void println( bool val );
};


#endif

//
//	EOF
//

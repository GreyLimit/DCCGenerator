//
//	Trace.cpp
//	=========
//
//	Implementation of the stack trace macro code.
//

#include "Code_Assurance.h"
#include "Critical.h"
#include "Trace.h"

//
//	Configure/define the stack trace facility.
//
#ifdef ENABLE_STACK_TRACE

//
//	Bring in the Console.
//
#include "Console.h"

//
//	Keeping a depth counter
//
byte	stack_frame::_depth = 0;

//
//	Are we displaying the stack frames in real time?  From boot
//	we keep track of the frames but do not display them until we
//	are told we can.
//
bool	stack_frame::_display = false;

//
//	An "empty" stack frame.
//
stack_frame::stack_frame( void ) {
	//
	//	This is an invalid constructor and should not be used.
	//
	ABORT( PROGRAMMER_ERROR_ABORT );
}

//
//	Cons and Destruct routines which do the work
//
stack_frame::stack_frame( const char *func ) {
	{
		Critical code;
		
		_name = func;
		_down = top_stack_frame;
		top_stack_frame = this;
		_depth++;
	}
	if( _display && Critical::normal_code()) {
		for( byte i = 0; i < _depth; i++ ) console.print(( i & 3 )? SPACE: '|' );
		console.print( F( "E:" ));
		console.println_PROGMEM( _name );
		console.synchronise();
	}
}
stack_frame::~stack_frame() {
	if( _display && Critical::normal_code()) {
		for( byte i = 0; i < _depth; i++ ) console.print(( i & 3 )? SPACE: '|' );
		console.print( F( "L:" ));
		console.println_PROGMEM( _name );
		console.synchronise();
	}
	{
		Critical code;
		
		_depth--;
		top_stack_frame = _down;
	}
}

//
//	Display the stack frame!
//
void stack_frame::show( Byte_Queue_API *to ) {
	stack_frame	*here;
	
	if(( here = this ) == NIL( stack_frame )) {
		to->println( F( "Stack Empty" ));
	}
	else {
		while( here != NIL( stack_frame )) {
			to->println_PROGMEM( here->_name );
			here = here->_down;
		}
	}
}

//
//	Display the caller of this routine.
//
void stack_frame::caller( Byte_Queue_API *to ) {
	stack_frame	*here;
	
	if(( here = this ) == NIL( stack_frame )) {
		to->println( F( "Stack Empty" ));
	}
	else {
		if(( here = here->_down ) == NIL( stack_frame )) {
			to->println( F( "No Caller" ));
		}
		else {
			to->println_PROGMEM( here->_name );
		}
	}
}

//
//	Set the display flag
//
void stack_frame::display( bool on ) {
	_display = on;
}


//
//	Declare the GLOBAL pointer to the top stack frame.
//
stack_frame	*top_stack_frame = NIL( stack_frame );

#endif


//
//	EOF
//

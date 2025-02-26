//
//	Gate.h
//	======
//
//	Define a primitive versions of the Wait and Signal functionality.
//
//	On the Arduino, with a single CPU and no "proper" multi-tasking
//	these calls must not actually wait in any form, and so they
//	are replaced with a pair of routines which can be used to achieve
//	same end.
//

#ifndef _GATE_H
#define _GATE_H

//
//	We need the Critical code to manage interrupts.
//
#include "Configuration.h"
#include "Environment.h"
#include "Critical.h"

//
//	This is the definition of the Gate class which permits
//	only a single point of execution past a given point.
//
class Gate {
private:
	//
	//	This is the lock for the gate.
	//
	volatile bool	_locked;

public:
	Gate( void ) {
		_locked = false;
	}
	bool acquired( void ) {
		Critical code;
		
		if( _locked ) return( false );
		return(( _locked = true ));
	}
	void release( void ) {
		_locked = false;
	}
};

#endif

//
//	EOF
//

//
//	Critical
//
//	This file defines a class which is used to control access to
//	a section of code (through the C++ constructor/destructor
//	mechanism).
//


#ifndef _CRTITICAL_H_
#define _CRTITICAL_H_

#include "Environment.h"
#include "Library_Types.h"

//
//	A variable of class Critical is simply declared at the start of
//	a block of code.  The Constructor and Destructor actions of the
//	class ensure that the whole block remains atomic and cannot be
//	interrupted.
//
//	Example:
//
//		{
//			Critical code;
//
//			...blah blah blah...
//
//		}
//
//	Simples.
//

#if defined( ARDUINO_ARCH_AVR ) || defined( ARDUINO_ARCH_MEGAAVR )

//
//	The Atmel AVR Implementation.
//
class Critical {
	private:
		//
		//	Somewhere to save a copy of the Status
		//	Register
		//
		byte	_sreg;

	public:
		//
		//	Called when entering the block
		//
		Critical( void ) {
			_sreg = SREG;
			noInterrupts();
		}
		//
		//	Called when leaving the block
		//
		~Critical() {
			SREG = _sreg;
		}
};

#else

#error "Define Class Critical for your architecture."


#endif



#endif

//
//	EOF
//

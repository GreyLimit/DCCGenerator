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
//	In situations where an ISR wants to release its Interrupt status
//	for a section of its operation the class Controlled has been
//	provided.  This class performs two actions:
//
//	o	Enables interrupts for the duration of its existence.
//
//	o	Toggles a boolean variable to "true" (ie LOCKED) while
//		the variable exists, restoring its value when destoryed.
//
//	Example code:
//
//
//		ISR( xxxxx ) {
//			static volatile bool lock = false;
//			if( !lock ) {
//				Controlled code( &lock );
//
//				.....Code executing with interrupts enabled....
//			}
//		}
//
//

#if defined( ARDUINO_ARCH_AVR ) || defined( ARDUINO_ARCH_MEGAAVR )

//
//	The Atmel AVR Implementation.
//
class Critical {
public:
	//
	//	The bit of the status byte which controls the
	//	handling of interrupts and enables code to be
	//	flagged as critical.
	//
	//	This is SREG bit 7: ""I".
	//
	static const byte global_interrupt_flag = bit( 7 );
	
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
		SREG &= ~global_interrupt_flag;
	}
	
	//
	//	Called when leaving the block
	//
	~Critical() {
		SREG = _sreg;
	}

	//
	//	Provide a function enabling code executing inside
	//	a Critical section the ability to query the mode
	//	"outside" this section.
	//
	bool was_critical( void ) {
		return( !BOOL( _sreg & global_interrupt_flag ));
	}
	bool was_normal( void ) {
		return( !BOOL( _sreg & global_interrupt_flag ));
	}
	
	//
	//	Simple boolean functions return true if
	//
	//		execution is taking place inside an interrupt or Critical section.
	//
	//		execution is taking place as normal code.
	//
	static inline bool critical_code( void ) {
		return( !BOOL( SREG & global_interrupt_flag ));
	}
	static inline bool normal_code( void ) {
		return( BOOL( SREG & global_interrupt_flag ));
	}

	//
	//	Routines that provide direct access to interrupts controls.
	//	These should not be used under any normal circumstances.
	//
	static inline void enable_interrupts( void ) {
		SREG |= global_interrupt_flag;
	}
	static inline void disable_interrupts( void ) {
		SREG &= ~global_interrupt_flag;
	}
};

//
//	Provide a limited ability for code to be executed in "normal"
//	(ie interruptible) mode.
//
class Normal {
public:
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
	Normal( void ) {
		_sreg = SREG;
		SREG |= Critical::global_interrupt_flag;
	}
	
	//
	//	Called when leaving the block
	//
	~Normal() {
		SREG = _sreg;
	}

};


#else

#error "Define Class Critical/Normal for your architecture."

#endif



#endif

//
//	EOF
//

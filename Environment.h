//
//	Environment -	The definition of a set of types
//			and values which are broadly generic
//	and applicable to multiple modules.
//

#ifndef _ENVIRONMENT_H_
#define _ENVIRONMENT_H_

//
//	To complete some of these tasks we have to bring in
//	the Arduino environment first.
//
#include <Arduino.h>

//
//	Bring in the necessary IO and Interrupt definitions.
//
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <compat/twi.h>

#include <limits.h>
#include <stdlib.h>
#include <inttypes.h>


//
//	Include the base library types.
//
#include "Library_Types.h"

//////////////////////////////////////////////////
//						//
//	PROGMEM Support code and macros		//
//	===============================		//
//						//
//////////////////////////////////////////////////

#if defined( ARDUINO_ARCH_AVR ) || defined( ARDUINO_ARCH_MEGAAVR ) || defined( ARDUINO_ARCH_SAMD )

//
//	AVR MCU Specific PROGMEM code
//
//	Include the AVR program space ability or, if not an
//	AVR system, alias the keyword to function in the knowledge
//	that the data can only be in real memory.
//

#define progmem_read_byte(v)		pgm_read_byte(&(v))
#define progmem_read_byte_at(a)		pgm_read_byte(a)
#define progmem_read_word(v)		pgm_read_word(&(v))
#define progmem_read_word_at(a)		pgm_read_word(a)
#define progmem_read_dword(v)		pgm_read_dword(&(v))
#define progmem_read_dword_at(a)	pgm_read_dword(a)
//
#define progmem_read_address(v)		pgm_read_word(&(v))
#define progmem_read_address_at(a)	pgm_read_word(a)


#else

//
//	Generic non-PROGMEM code
//
#define PROGMEM

#define progmem_read_byte(v)		(v)
#define progmem_read_byte_at(a)		(*(a))
#define progmem_read_word(v)		(v)
#define progmem_read_word_at(a)		(*(a))
#define progmem_read_dword(v)		(v)
#define progmem_read_dword_at(a)	(*(a))
//
#define progmem_read_address(v)		(v)
#define progmem_read_address_at(a)	(*(a))


#endif

//
//	We are going to set CODE_ASSURANCE here
//
#define CODE_ASSURANCE 1

//
//	More Syntactic Sugar
//
#ifndef UNUSED
#define UNUSED(x)	__attribute__((unused)) x
#endif

//
//	This definition should be entirely un-required
//	following a complete move the C++.
//
#ifndef FUNC
#define FUNC(x)		(*(x))
#endif

//
//	Define macro that will convert bit pattern
//	tests (using '&', '^' or '|') into primary
//	true/false values of type bool.
//
#define BOOL(e)		((bool)((e)!=0))

//
//	Define a symbol that will indicate that a case
//	selection (inside a switch statement) should
//	(and is expected to) fall through to the case
//	statement that follows it in the source code.
//
//	This is used so that, according to the compiler
//	selected, the correct fall through can be used.
//
#define FALL_THROUGH	__attribute__((fallthrough))

//
//	An integer error value
//
#ifndef ERROR
#define ERROR		(-1)
#endif

//
//	A type specific NULL
//
#define NIL(t)		((t *)NULL)

//
//	This value is used to indicate an error where a byte, word
//	or dword value is returned
//
#ifndef ERROR_BYTE
#define ERROR_BYTE	((byte)0xff)
#endif
#ifndef ERROR_WORD
#define ERROR_WORD	((word)0xffff)
#endif
#ifndef ERROR_DWORD
#define ERROR_DWORD	((word)0xffffffff)
#endif

//
//	This value is maximum value which can be held in one of the
//	three basic types.
//
#ifndef MAXIMUM_BYTE
#define MAXIMUM_BYTE	((byte)0xff)
#endif
#ifndef MAXIMUM_WORD
#define MAXIMUM_WORD	((word)0xffff)
#endif
#ifndef MAXIMUM_DWORD
#define MAXIMUM_DWORD	((word)0xffffffff)
#endif

//
//	Some Most Significant Bit values.
//
#ifndef BYTE_MSB
#define BYTE_MSB	((byte)0x80)
#endif
#ifndef WORD_MSB
#define WORD_MSB	((word)0x8000)
#endif
#ifndef DWORD_MSB
#define DWORD_MSB	((dword)0x80000000)
#endif

//////////////////////////////////////////////////
//						//
//	Hardware Endian-ness support macros 	//
//	===================================	//
//						//
//////////////////////////////////////////////////

//
//	The following section of definitions deals specifically with
//	how to handle "endianness" of the data between the processor
//	executing the code and hardware devices which will have their
//	data order set (and not necessarily the same as the MCU)
//
//	We will see if the compiler environment contains some standard
//	symbols which we can leverage:
//
//		__ORDER_LITTLE_ENDIAN__
//		__ORDER_BIG_ENDIAN__
//
#if defined( __ORDER_LITTLE_ENDIAN__ ) || defined( __ORDER_BIG_ENDIAN__ )
//
//	Great, we know what we are and need do no more.
//
#else
//
//	Chuck out an error
//
#error "Target Platform Endianness not identified"
//
#endif

//
//	Platform independent conversion from word to bytes and bytes
//	to word
//
#ifndef HL_TO_W
#define HL_TO_W(h,l)	(word)(((h)<<8)|(l))
#endif
#ifndef W_TO_H	
#define W_TO_H(w)	(byte)(((w)>>8)&0xff)
#endif
#ifndef W_TO_L	
#define W_TO_L(w)	(byte)((w)&0xff)
#endif

//
//	And the same with respect to bytes and nybbles.
//
#ifndef HL_TO_B
#define HL_TO_B(h,l)	(byte)(((h)<<4)|(l))
#endif
#ifndef B_TO_H	
#define B_TO_H(b)	(byte)(((b)>>4)&0x0f)
#endif
#ifndef B_TO_L	
#define B_TO_L(b)	(byte)((b)&0x0f)
#endif


//////////////////////////////////////////////////
//						//
//	Generic constant definitions		//
//	============================		//
//						//
//////////////////////////////////////////////////
//
//
//	Finally define character constants used across the board.
//
#define EOS		'\0'
#define SPACE		' '
#define COMMA		','
#define TAB		'\t'
#define NL		'\n'
#define CR		'\r'
#define HASH		'#'
#define MINUS		'-'
#define PLUS		'+'
#define USCORE		'_'
#define SLASH		'/'
#define COLON		':'
#define DELETE		'\177'
#define ZERO		'0'
#define ERROR		(-1)


#endif

//
//	EOF
//

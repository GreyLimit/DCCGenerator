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
#ifndef __linux__
#include <Arduino.h>
#endif

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

#if defined( ARDUINO_ARCH_AVR ) || defined( ARDUINO_ARCH_MEGAAVR )

//
//	AVR MCU Specific PROGMEM code
//
//	Include the AVR program space ability or, if not an
//	AVR system, alias the keyword to nothing.
//

#include <avr/pgmspace.h>

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
//	An integer error value
//
#ifndef ERROR
#define ERROR		(-1)
#endif

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
//	Data Size type definition		//
//	=========================		//
//						//
//////////////////////////////////////////////////
//
//	Define a scalar type to represent the size of the value
//	used when handling buffer lengths.
//
//	The default mode (for small systems with limited memory)
//	is to use an unsigned byte value.  This limits (obviously)
//	8 bit byte buffers to 255 values and 16 bit word buffers
//	(hiding inside a byte buffer) to 127 values.
//
//	A system with more than a few KBytes of memory could reasonably
//	want and need to handle data larger than this, so the type
//	would need to be changed to a word based value.
//

#if defined( ARDUINO_ARCH_AVR ) || defined( ARDUINO_ARCH_MEGAAVR )

//
//	AVR systems are really short of memory, a byte size buffer and
//	queue length should be more than enough.
//
typedef byte data_size;
#define DATA_SIZE_SIZE	1

#else

//
//	Systems other than the AVR chip are assumed to have capability
//	and capacity to support word sizes buffer/queue lengths.
//
typedef word data_size;
#define DATA_SIZE_SIZE	2

#endif

//
//	Definitions used around the data_size type.
//
//	DATA_SIZE_SIZE		Size of type in bytes
//
//	DATA_SIZE_BITS		Number of bits required to index
//				the bytes within the size value.
//
//	DATA_SIZE_MASK		Bit mask matching above bit count
//
//	DATA_SIZE_MAX		The maximum unsigned value for the
//				data_size type.
//
//	I know there are better ways of doing this, but at the moment
//	the concept of the "data_size" type and it's purpose is a little
//	flexible.  Essentially it is meant to reflect the "optimal" size
//	of a data type used to represent the "size" of something (a
//	packet, a buffer, a string .. something).  On the smaller MCUs
//	using anything other than a byte is pointless, they simply have
//	not got the memory.  On the larger MCUs with more memory then a
//	byte value is too restrictive, but a 16-bit word is almost
//	certainly sufficient (for most cases).
//
#define DATA_SIZE_BITS	((DATA_SIZE_SIZE==1)?0:((DATA_SIZE_SIZE==2)?1:((DATA_SIZE_SIZE==4)?2:3)))
#define DATA_SIZE_MASK	((1<<DATA_SIZE_BITS)-1)
#define DATA_SIZE_MAX	((1<<(DATA_SIZE_SIZE<<3))-1)


//////////////////////////////////////////////////
//						//
//	Library type definitions		//
//	========================		//
//						//
//////////////////////////////////////////////////
//


//
//	Define an enumerated type providing the various
//	mirroring options for displays:
//
//	MirrorNone		Display is only rotated
//
//	MirrorHorizontal	Display is rotated then mirrored on a horizontal axis
//
//	MirrorVertical		Display is rotated then mirrored on a Vertical axis
//
//	typing the above comment I actually cannot remember if my comments
//	vis the mirror axis are correct.  I need to test and fix (no doubt).
//
typedef enum {
	MirrorNone,
	MirrorHorizontal,
	MirrorVertical
} MirrorMode;

//
//	Define an enumerated type providing
//	three possible "synchronisation"
//	options:
//
//	Synchronised		Routine only returns when the action is
//				completed.
//
//	HalfSynchronised	Routine only returns when the command has
//				been successfully queued for execution.
//
//	Asynchronous		Routine returns false if it cannot even queue
//				the command.
//
typedef enum {
	Synchronised,
	HalfSynchronised,
	Asynchronous
} SyncMode;

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


#endif

//
//	EOF
//

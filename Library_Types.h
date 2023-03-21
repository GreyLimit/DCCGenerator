//
//	A simple header file to "simulate" a set of Arduino
//	like data types to non-Arduino environments.
//

#ifndef _LIBRARY_TYPES_H_
#define _LIBRARY_TYPES_H_

#if defined( ARDUINO )

//
//	If we are compiling on an Arduino, lets explicitly
//	bring in all the key definitions.
//
#include <Arduino.h>

//
//	We will create a double word type.
//
typedef unsigned long dword;
typedef word address;

#else

//
//	If we are *not* on an Arduino, lets try to do our best.
//
//	We are now trying to determine how to best simulate
//	the basic types of an Arduino.
//

typedef unsigned char byte;
typedef unsigned int word;
typedef unsigned long dword;
typedef unsigned long address;

#endif

#endif

//
//	EOF
//

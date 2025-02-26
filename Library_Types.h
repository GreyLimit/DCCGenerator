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
//	We will create some missing types.
//
typedef signed char	sbyte;
typedef signed int	sword;
typedef unsigned long	dword;
typedef word		address;

#else

//
//	If we are *not* on an Arduino, lets try to do our best.
//
//	We are now trying to determine how to best simulate
//	the basic types of an Arduino.
//

typedef unsigned char	byte;
typedef signed char	sbyte;
typedef unsigned int	word;
typedef signed int	sword;
typedef unsigned long	dword;
typedef unsigned long	address;

#endif

#endif

//
//	EOF
//

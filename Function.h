//
//	Function.h
//	==========
//
//	Module to Cache and manage the "function state" of mobile
//	decoders.
//

#ifndef _FUNCTION_H_
#define _FUNCTION_H_

//
//	We will need the following files
//
#include "Environment.h"
#include "Parameters.h"
#include "Configuration.h"
#include "DCC_Constant.h"
#include "Memory_Heap.h"

//
//	Decoder Function value cache
//	----------------------------
//
//	Native DCC Generator code requires a function status cache to
//	support modification of an individual decoder function.  This
//	is required as (as far as I can tell) there is no mechanism to
//	allow individual function adjustment without setting/resetting
//	between 3 to 7 other functions at the same time.
//


//
//	Declare a "database" which will be used to keep the functional
//	state of the "most recently" accessed mobile decoders.
//
class Function {
private:
	//
	//	How many byte do we need for the bit array?
	//
	static constexpr byte	bit_array	= (( 1 + DCC_Constant::maximum_func_number - DCC_Constant::minimum_func_number ) + 7 ) >> 3;

	//
	//	The structure used to cache function values per decoder
	//	so that the "block" function setting DCC packet can be
	//	used (because that is the only way).
	//
	struct cache {
		word		target;
		byte		bits[ bit_array ];
		cache		*next;
	};

	//
	//	This is the array of cache records and the pointer into the head
	//	of it (the most recently accessed).
	//
	cache			*_cache;

	//
	//	Define the lookup and manage cache code.
	//
	cache *find( word target );


public:
	//
	//	Function to initialise the cache records empty.
	//
	//	We will "pre-fill" the cache with empty records so that the code can
	//	always assume that there are records in the cache, because there are.
	//
	Function( void );

	//
	//	Routine applies a boolean value for a specified function
	//	on a specified target number.
	//
	//	This returns true if the function status changed, false
	//	otherwise.
	//
	bool update( word target, byte func, bool state );

	//
	//	Routine returns 0 or the supplied value based on the
	//	supplied function number being off or on.
	//
	byte get( word target, byte func, byte val );


};

//
//	Define the function cache.
//
extern Function function_cache;

#endif

//
//	EOF
//





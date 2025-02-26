//
//	Function.cpp
//	============
//
//	Module to Cache and manage the "function state" of mobile
//	decoders.
//


#include "Function.h"
#include "Trace.h"

//
//	Define the lookup and manage cache code.
//
Function::cache *Function::find( word target ) {

	STACK_TRACE( "Function::cache *Function::find( word target )" );

	cache	**adrs,
		*last,
		*ptr;

	ASSERT( target >= DCC_Constant::minimum_address );
	ASSERT( target <= DCC_Constant::maximum_address );

	//
	//	Start at the top, the most recently accessed record.
	//
	adrs = &_cache;
	last = NIL( cache );

	//
	//	Extract address of the next record to check..
	//
	while(( ptr = *adrs )) {
		//
		//	Is this the one?
		//
		if( target == ptr->target ) {
			//
			//	Yes, so move to top of the list so that
			//	access to this record is as quick as
			//	possible for subsequent requests.
			//
			if( _cache != ptr ) {
				*adrs = ptr->next;
				ptr->next = _cache;
				_cache = ptr;
			}
			//
			//	Return the pointer to the desired cache record.
			//
			return( ptr );
		}
		//
		//	Note last record we saw.
		//
		last = ptr;
		adrs = &( ptr->next );
	}

	//
	//	Nothing found.
	//
	//	Try to allocate a new record..
	//
	if(( ptr = new cache )) {
		//
		//	 ...and fill it in as empty.
		//
		ptr->target = target;
		for( byte i = 0; i < bit_array; ptr->bits[ i++ ] = 0 );
		ptr->next = _cache;
		_cache = ptr;
		
		//
		//	Done.
		//
		return( ptr );
	}

	//
	//	re-purpose the last record in the queue.  It'll get
	//	moved to the head of the queue on the next call.
	//
	last->target = target;
	for( byte i = 0; i < bit_array; last->bits[ i++ ] = 0 );

	//
	//	Done.
	//
	return( last );
}


//
//	Constructor to start system as empty.
//
Function::Function( void ) {
	_cache = NIL( cache );
}

//
//	Routine applies a boolean value for a specified function
//	on a specified target number.
//
//	This returns true if the function status changed, false
//	otherwise.
//
bool Function::update( word target, byte func, bool state ) {

	STACK_TRACE( "bool Function::update( word target, byte func, bool state )" );

	cache	*ptr;
	byte	i, b; 

	//ASSERT( func >= DCC_Constant::minimum_func_number );
	ASSERT( func <= DCC_Constant::maximum_func_number );

	ptr = find( target );
	i = ( func - DCC_Constant::minimum_func_number ) >> 3;
	b = 1 << (( func - DCC_Constant::minimum_func_number ) & 7 );

	if( state ) {
		//
		//	Bit set already?
		//
		if( ptr->bits[ i ] & b ) return( false );
		//
		//	Yes.
		//
		ptr->bits[ i ] |= b;
		return( true );
	}
	//
	//	Bit clear already?
	//
	if(!( ptr->bits[ i ] & b )) return( false );
	//
	//	Yes.
	//
	ptr->bits[ i ] &= ~b;
	return( true );
}

//
//	Routine returns 0 or the supplied value based on the
//	supplied function number being off or on.
//
byte Function::get( word target, byte func, byte val ) {

	STACK_TRACE( "byte Function::get( word target, byte func, byte val )" );

	cache	*ptr;
	byte	i, b; 

	//ASSERT( func >= DCC_Constant::minimum_func_number );
	ASSERT( func <= DCC_Constant::maximum_func_number );

	ptr = find( target );
	i = ( func - DCC_Constant::minimum_func_number ) >> 3;
	b = 1 << (( func - DCC_Constant::minimum_func_number ) & 7 );

	if( ptr->bits[ i ] & b ) return( val );
	return( 0 );
}


//
//	Declare the function cache.
//
Function function_cache;

//
//	EOF
//

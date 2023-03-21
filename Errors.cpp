//
//	Errors:		A consolidated error logging system
//			to enable errors to be noted at point
//			of detection, but processed at a future
//			point of convenience.
//

//
//	Bring in our environment
//
#include "Environment.h"
#include "Errors.h"

//
//	Initialise in the constructor.
//
Errors::Errors( void ) {
	for( byte i = 0; i < cache_size; i++ ) {
		_cache[ i ].error = 0;
		_cache[ i ].arg = 0;
		_cache[ i ].repeats = 0;
	}
	_count = 0;
	_in = 0;
	_out = 0;
}

//
//	Log an error with the system
//
void Errors::log_error( word error, word arg ) {
	byte	i, j;

	//
	//	Has this error been reported before?
	//
	i = _out;
	for( j = 0; j < _count; j++ ) {
		if(( _cache[ i ].error == error )&&( _cache[ i ].arg == arg )) {
			if( _cache[ i ].repeats < ERROR_BYTE ) _cache[ i ].repeats++;
			return;
		}
		if( ++i >= cache_size ) i = 0;
	}
	//
	//	No, this is a new error.
	//
	if( _count >= cache_size ) {
		//
		//	Cache full, look for one of our own
		//	errors we can update.
		//
		for( i = 0; i < cache_size; i++ ) {
			if( _cache[ i ].error == ERRORS_ERR_OVERFLOW ) {
				//
				//	We increment the argument (count of
				//	overflows) but only if it does not
				//	wrap round to zero.
				//
				if( _cache[ i ].arg < ERROR_WORD ) _cache[ i ].arg++;
				return;
			}
		}
		//
		//	Nothing found so we ditch the newest error
		//	(on the grounds that older errors are more
		//	pertinent to working out what's gone wrong).
		//
		if(( i = _in )) {	// Assign intentional
			i--;
		}
		else {
			i = cache_size-1;
		}
		//
		//	What ever it was is gone.
		//
		_cache[ i ].error = ERRORS_ERR_OVERFLOW;
		_cache[ i ].repeats = 1;
		_cache[ i ].arg = 1;
		return;
	}
	//
	//	Create new record.
	//
	_cache[ _in ].error = error;
	_cache[ _in ].repeats = 0;
	_cache[ _in ].arg = arg;
	if( ++_in >= cache_size ) _in = 0;
	_count++;
	
	//
	//	Done.
	//
}

//
//	Log a terminal system error with the system.
//
void Errors::log_terminate( word error, UNUSED( const char *file_name ), word line_number ) {
	//
	//	This is a termination error where we are supplied the error, filename and
	//	line number.
	//
	log_error( error, line_number );
}

//
//	Return count of errors pending
//
int Errors::pending_errors( void ) {
	return( _count );
}

//
//	Peek at the top error.
//
//	Return true if there was one, false otherwise.
//
bool Errors::peek_error( word *error, word *arg, byte *repeats ) {
	if( _count ) {
		*error = _cache[ _out ].error;
		*arg = _cache[ _out ].arg;
		*repeats = _cache[ _out ].repeats;
		return( true );
	}
	return( false );
}

//
//	Drop the next error.  It is assumed that peek was
//	used to obtain the content of the error so this
//	allows an error which has been handled to be
//	discarded.
//
void Errors::drop_error( void ) {
	if( _count ) {
		if( ++_out >= cache_size ) _out = 0;
		_count--;
	}
}


//
//	Declare the errors object.
//
Errors errors;


//
//	EOF
//

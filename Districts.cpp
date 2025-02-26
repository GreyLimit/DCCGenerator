//
//	Districts.cpp
//	=============
//
//	The declaration of the districts manager.
//


#include "Districts.h"
#include "Code_Assurance.h"
#include "Trace.h"

#ifdef DEBUGGING_ENABLED
#include "Console.h"
#endif

//
//	Allow this to build itself empty first.
//
Districts::Districts( void ) {
	_zone = 0;
}

//
//	Initialise the districts and set them into action.
//
void Districts::initialise( void ) {
	
	STACK_TRACE( "void Districts::initialise( void )" );
	
	//
	//	Set up the districts according to the built in
	//	table.
	//
	for( byte i = 0; i < districts; i++ ) {
		const DCC_District	*d;
		Pin_IO			brake;
		byte			b;

		d = &( DCC_District::district[ i ]);
		
		if(( b = progmem_read_byte( d->brake )) != DCC_District::no_brake ) {
			brake.configure( b, false );
			brake.low();
		}

		//
		//	Ensure zone 0 is never used (zone 0 is all off).
		//
		ASSERT( progmem_read_byte( d->zone ) > 0 );
		
		_district[ i ].assign(	progmem_read_byte( d->enable ),
					progmem_read_byte( d->direction ),
					progmem_read_byte( d->adc_pin ),
					progmem_read_byte( d->adc_test ));
	}
	
	//
	//	Ensure everything is off.
	//
	for( byte i = 0; i < districts; _district[ i++ ].power( false ));
	_zone = 0;
}


//
//	Return the number of the zone currently being operated.
//
byte Districts::zone( void ) {
	
	STACK_TRACE( "byte Districts::zone( void )" );
	
	return( _zone );
}



//
//	Set the power to target zone.
//
bool Districts::power( byte zone ) {
	
	STACK_TRACE( "bool Districts::power( byte zone )" );
	
	byte	count;

	count = 0;
	for( byte i = 0; i < districts; i++ ) {
		if( progmem_read_byte( DCC_District::district[ i ].zone ) == zone ) {
			count++;
			_district[ i ].power( true );
		}
		else {
			_district[ i ].power( false );
		}
	}
	_zone = count? zone: 0;
	return(( zone == 0 )||( count > 0 ));
}

//
//	Return current load average (0-100) for indicated district
//
byte Districts::load_average( byte index ) {
	
	STACK_TRACE( "byte Districts::load_average( byte index )" );
	
	if( index >= districts ) return( 0 );
	return( _district[ index ].load_average());
}

//
//	Return the state of this district
//
District::district_state Districts::state( byte index ) {
	
	STACK_TRACE( "District::district_state Districts::state( byte index )" );
	
	if( index >= districts ) return( District::state_unassigned );
	return( _district[ index ].state());
}


//
//	The districts manager.
//
Districts	districts;


//
//	EOF
//

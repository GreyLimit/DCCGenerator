//
//	Driver.cpp
//	==========
//
//	Declare the driver object.
//

#include "Driver.h"
#include "Trace.h"

#ifdef DEBUGGING_ENABLED
#include "Console.h"
#endif

//
//	Constructor
//
Driver::Driver( void ) {
	_districts = 0;
}

//
//	Add a new pin to the configuration of the driver where
//	the pin is referenced by the "platform" pin number.
//
bool Driver::add( byte *index, byte enable, byte direction ) {
	
	STACK_TRACE( "bool Driver::add( byte *index, byte enable, byte direction )" );

	TRACE_DRIVER( console.print( F( "enable " )));
	TRACE_DRIVER( console.println( enable ));
	TRACE_DRIVER( console.print( F( "direction " )));
	TRACE_DRIVER( console.println( enable ));
	
	driver_record	*d;
	
	//
	//	Any space left?
	//
	if( _districts >= maximum_districts ) return( false );

	//
	//	Locate a new record (and save its index number).
	//
	d = &( _district[(( *index = _districts++ ))]);

	TRACE_DRIVER( console.print( F( "index " )));
	TRACE_DRIVER( console.println( *index ));

	//
	//	Update record and initialise the pins.
	//
	d->enable.configure( enable, false );
	d->enable.low();
	d->direction.configure( direction, false );
	d->direction.low();

	//
	//	Done!
	//
	return( true );
}

//
//	Add a new pin to the configuration of the driver where
//	the pin is referenced by port number and bit within port.
//
bool Driver::add( byte *index, byte enable_dev, byte enable_bitno, byte direction_dev, byte direction_bitno ) {
	
	STACK_TRACE( "bool Driver::add( byte *index, byte enable_dev, byte enable_bitno, byte direction_dev, byte direction_bitno )" );
	
	TRACE_DRIVER( console.print( F( "enable_dev " )));
	TRACE_DRIVER( console.println( enable_dev ));
	TRACE_DRIVER( console.print( F( "enable_bitno " )));
	TRACE_DRIVER( console.println( enable_bitno ));
	TRACE_DRIVER( console.print( F( "direction_dev " )));
	TRACE_DRIVER( console.println( direction_dev ));
	TRACE_DRIVER( console.print( F( "direction_bitno " )));
	TRACE_DRIVER( console.println( direction_bitno ));
	
	driver_record	*d;
	
	//
	//	Any space left?
	//
	if( _districts >= maximum_districts ) return( false );

	//
	//	Locate a new record (and save its index number).
	//
	d = &( _district[(( *index = _districts++ ))]);

	TRACE_DRIVER( console.print( F( "index " )));
	TRACE_DRIVER( console.println( *index ));

	//
	//	Update record and initialise the pins.
	//
	d->enable.configure( enable_dev, enable_bitno, false );
	d->enable.low();
	d->direction.configure( direction_dev, direction_bitno, false );
	d->direction.low();

	//
	//	Done.
	//
	return( true );
}

//
//	Now the DCC generator will use the following API calls.
//

//
//	Turn on all or one district
//
void Driver::on( void ) {
	
	STACK_TRACE( "void Driver::on( void )" );
	
	for( byte i = 0; i < _districts; _district[ i++ ].enable.high());
}
void Driver::on( byte index ) {
	
	STACK_TRACE( "void Driver::on( byte index )" );
	
	if( index < _districts ) _district[ index ].enable.high();
}

//
//	Turn off all or one district
//
void Driver::off( void ) {
	
	STACK_TRACE( "void Driver::off( void )" );
	
	for( byte i = 0; i < _districts; _district[ i++ ].enable.low());
}
void Driver::off( byte index ) {
	
	STACK_TRACE( "void Driver::off( byte index )" );
	
	if( index < _districts ) _district[ index ].enable.low();
}

//
//	Toggle the output signal of all or a single district.
//
void Driver::toggle( void ) {
	
	STACK_TRACE( "void Driver::toggle( void )" );
	
	for( byte i = 0; i < _districts; _district[ i++ ].direction.toggle());
}
void Driver::toggle( byte index ) {
	
	STACK_TRACE( "void Driver::toggle( byte index )" );
	
	if( index < _districts ) _district[ index ].direction.toggle();
}

//
//	Generic power on/off call.
//
void Driver::power( bool on ) {
	
	STACK_TRACE( "void Driver::power( bool on )" );
	
	for( byte i = 0; i < _districts; _district[ i++ ].enable.set( on ));
}
void Driver::power( byte index, bool on ) {
	
	STACK_TRACE( "void Driver::power( byte index, bool on )" );
	
	if( index < _districts ) _district[ index ].enable.set( on );
}

//
//	Here we declare the driver object.
//
Driver dcc_driver;

//
//	EOF
//

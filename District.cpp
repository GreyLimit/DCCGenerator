//
//	District.cpp
//	============
//
//	Implementation of the District module.
//


#include "District.h"
#include "Code_Assurance.h"
#include "ADC_Manager.h"
#include "TOD.h"
#include "Constants.h"
#include "Driver.h"
#include "Task.h"
#include "Trace.h"

#ifdef DEBUGGING_ENABLED
#include "Console.h"
#endif

//
//	A little math support.
//
#include "mul_div.h"

//
//	Declare the exclusivity gate control.
//
Gate District::exclusive_access;


//
//	Initialise the district as unassigned.
//
District::District( void ) {
	_state = state_unassigned;
}

//
//	Assign enable and direction pins to this district
//	(which will then automatically configure the
//	driver object and initiate use of ADC manager)
//
void District::assign( byte enable, byte direction, byte adc_pin, byte adc_number ) {
	
	STACK_TRACE( "void District::assign( byte enable, byte direction, byte adc_pin, byte adc_number )" );

	TRACE_DRIVER( console.print( F( "DST enable " )));
	TRACE_DRIVER( console.println( enable ));
	TRACE_DRIVER( console.print( F( "direction " )));
	TRACE_DRIVER( console.println( direction ));
	TRACE_DRIVER( console.print( F( "adc_pin " )));
	TRACE_DRIVER( console.println( adc_pin ));
	TRACE_DRIVER( console.print( F( "adc_number " )));
	TRACE_DRIVER( console.println( adc_number ));
	TRACE_DRIVER( console.print( F( "signal " )));
	TRACE_DRIVER( console.println( _flag.identity()));
	
	//
	//	Save details for this district.
	//
	_pin.configure( adc_pin, true );
	_test = adc_number;
	
	//
	//	Add our pins to the driver object.
	//
	if( !dcc_driver.add( &_driver, enable, direction )) ABORT( DCC_DRIVER_CONFIGURATION_FULL );
	
	TRACE_DRIVER( console.print( F( "DST driver " )));
	TRACE_DRIVER( console.println( _driver ));
	//
	//	Add the district to the task manager.
	//
	if( !task_manager.add_task( this, &_flag )) ABORT( TASK_MANAGER_QUEUE_FULL );
	
	//
	//	Now initiate the first ADC reading.
	//
	if( !adc_manager.read( _test, &_flag, &_reading )) ABORT( ADC_QUEUE_FULL );
	
	//
	//	Done.
	//
}

//
//	Task Entry point from the task manager.
//
void District::process( UNUSED( byte handle )) {
	
	STACK_TRACE( "void District::process( void )" );

	TRACE_DISTRICT( console.print( F( "DST driver " )));
	TRACE_DISTRICT( console.println( _driver ));
	TRACE_DISTRICT( console.print( F( "amps " )));
	TRACE_DISTRICT( console.println( _reading ));
	
	//
	//	We get here because a reading is available, so
	//	add this to the average.  There are some cases
	//	where the "reading" has not been made (during
	//	a pause), but the reading variable will have been
	//	reset to zero for this period.
	//
	_average.add( _reading );

	
	//
	//	What we do really depends on our state.
	//
	switch( _state ) {
		case state_off: {

			TRACE_DISTRICT( console.println( F( "DST state_off" )));

			//
			//	Nothing to do here.  The state
			//	will be changed by an external
			//	source.
			//
			break;
		}
		case state_on: {

			TRACE_DISTRICT( console.println( F( "DST state_on" )));

			//
			//	This is the "normal" state of a
			//	district.  We should be testing to
			//	see how the load is looking.
			//
			if( _reading > INSTANT_CURRENT_LIMIT ) {
				//
				//	This is declared an immediate short, we
				//	need to head into the recovery process:
				//	We try to invert the district before we
				//	head into a power off.  However, the
				//	"risk" is that multiple districts see the
				//	same short, we have to ensure only one
				//	district tries the reverse trick.
				//
				if( exclusive_access.acquired()) {
					//
					//	we have exclusive access to this code
					//	so jolly well get on with it.
					//
					dcc_driver.toggle( _driver );
					_state = state_inverted;
				}
				else {
					//
					//	Another driver has already got hold of the
					//	exclusive access, we transition to shorted
					//	state to monitor its success.
					//
					_state = state_shorted;
				}
			}
			else if( _average.read( average_current_index ) > AVERAGE_CURRENT_LIMIT ) {
				//
				//	This is an on going overload situation.  This
				//	is not the result of a short (we believe), so
				//	we go straight for the power off option.
				//
				dcc_driver.off( _driver );
				_reading = 0;
				_state = state_paused;
			}
			//
			//	Normal processing completed, state changed if necessary.
			//
			break;
		}
		case state_shorted: {

			TRACE_DISTRICT( console.println( F( "DST state_shorted" )));

			//
			//	We are here because there was a short but we could not
			//	try flipping this district.  The short may have been resolved
			//	so test the reading for that first.
			//
			//	If still shorted try the exclusive flag one more time before
			//	heading into a full pause mode.
			//
			if( _reading > INSTANT_CURRENT_LIMIT ) {
				if( exclusive_access.acquired()) {
					//
					//	we have exclusive access to this code
					//	so jolly well get on with it.
					//
					dcc_driver.toggle( _driver );
					_state = state_inverted;
				}
				else {
					//
					//	There's nothing we can do here - the
					//	district needs to be turned off.
					//
					dcc_driver.off( _driver );
					_reading = 0;
					_state = state_paused;
				}
			}
			else {
				//
				//	We're all good now - back to on state.
				//
				_state = state_on;
			}
			//
			//	Short handling completed, state determines next action.
			//
			break;
		}
		case state_inverted: {

			TRACE_DISTRICT( console.println( F( "DST state_inverted" )));

			//
			//	We are here because we claimed the exclusive lock and
			//	flipped the phase on this district. 
			//
			if( _reading > INSTANT_CURRENT_LIMIT ) {
				//
				//	There's nothing we can do here - the
				//	district needs to be turned off.
				//
				dcc_driver.off( _driver );
				_reading = 0;
				_state = state_paused;
			}
			else {
				//
				//	We're all good now - back to the normal state.
				//
				_state = state_on;
			}
			//
			//	Final action is to release the exclusive lock
			//	before allowing the state variable to control
			//	next actions.
			//
			exclusive_access.release();
			break;
		}
		case state_paused: {

			TRACE_DISTRICT( console.println( F( "DST state_paused" )));

			//
			//	We have come in here after being paused for
			//	a period of time.  Time to restart the district
			//	and try for normal operation.
			//
			dcc_driver.on( _driver );
			_average.reset();
			_state = state_on;
			break;
		}
		default: {
			//
			//	Nothing we can do, this is a programmer error.
			//
			ABORT( PROGRAMMER_ERROR_ABORT );
			break;
		}
	}
	//
	//	Pausing or reading the ADC - depends on the
	//	new value state we are in.
	//
	if( _state == state_paused ) {

		TRACE_DISTRICT( console.println( F( "DST Paused" )));

		time_of_day.add( DRIVER_RESET_PERIOD, &_flag );
	}
	else {

		TRACE_DISTRICT( console.println( F( "DST Request ADC" )));

		adc_manager.read( _test, &_flag, &_reading );
	}
}

//
//	Control the power on this district.
//
void District::power( bool on ) {
	
	STACK_TRACE( "void District::power( bool on )" );
	
	TRACE_DISTRICT( console.print( F( "DST district " )));
	TRACE_DISTRICT( console.print( _driver ));
	TRACE_DISTRICT( console.print( F( " enabled " )));
	TRACE_DISTRICT( console.println( on ));

	dcc_driver.power( _driver, on );
	_state = on? state_on: state_off;
}

//
//	Return current load average 0-100
//
byte District::load_average( void ) {
	
	STACK_TRACE( "byte District::load_average( void )" );
	
	return( (byte)mul_div<word>( _average.read( AVERAGE_CURRENT_INDEX ), 100, AVERAGE_CURRENT_LIMIT ));
}

//
//	Return the state of this district
//
District::district_state District::state( void ) {
	
	STACK_TRACE( "District::district_state District::state( void )" );
	
	return( _state );
}


//
//	EOF
//

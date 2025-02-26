//
//	District.h
//	==========
//
//	Declare the class which manages to actions of a single
//	district in coordination with other districts.
//

#ifndef _DISTRICT_H_
#define _DISTRICT_H_

//
//	We will need the follow systems.
//
#include "Configuration.h"
#include "Parameters.h"
#include "Environment.h"

#include "Task_Entry.h"
#include "Average.h"
#include "Pin_IO.h"
#include "Gate.h"
#include "Signal.h"

//
//	Declare the class for handling a single district.
//
class District : public Task_Entry {
public:
	//
	//	Define the parameters to the district management code:
	//
	//
	//	We are going to use an array of values compounded upon
	//	each other to generate a series of average values which
	//	represent averages over longer and longer periods (each
	//	element of the array twice as long as the previous).
	//
	//	Define the size of the array.
	//
	static const byte	compounded_values = 10;

	//
	//	Define where the "average current load" reading is taken from.
	//
	static const byte	average_current_index = compounded_values - 1;

	//
	//	Define the size of the "spike average" we will use to
	//	identify a critical rise in power consumption signifying
	//	a short circuit.  This is an index into the
	//	compounded average table.
	//
	static const byte	spike_average_value = 1;

	//
	//	Define the size of the "short average" we will use to
	//	identify a genuine rise in power consumption signifying
	//	a confirmation return.  This is an index into the
	//	compounded average table.
	//
	static const byte	short_average_value = 2;

	//
	//	Declare the set of states in which a district can be
	//	sitting in.
	//
	enum district_state : byte {	// District...
		state_unassigned = 0,	// ...	is not attached to any hardware.
		state_off,		// ...	is explicitly powered down.
		state_on,		// ...	is running normally.
		state_shorted,		// ...	is shorted and waiting for
					//	slot to try phase inversion.
		state_inverted,		// ...	was shorted and is waiting
					//	for confirmation that the
					//	inversion worked
		state_paused		// ...	is down as result of short
					//	or overload.
	};
	
private:
	//
	//	Store the state of this district.
	//
	district_state			_state;

	//
	//	Record the link into the driver so we can directly
	//	control our output pins.
	//
	byte				_driver;

	//
	//	Keep a copy of the ADC pin we need to check for the
	//	power monitoring facility.
	//
	Pin_IO				_pin;
	byte				_test;

	//
	//	The control area from the ADC manager.
	//
	word				_reading;
	Signal				_flag;

	//
	//	Where we gather our readings over time.
	//
	Average<compounded_values,word>	_average;

	//
	//	Declare a single Gateway, common to all defined
	//	Districts, that controls access to code which
	//	only a single district should execute at any
	//	single time.
	//
	static Gate			exclusive_access;

public:
	//
	//	Initialise the district as unassigned.
	//
	District( void );

	//
	//	Assign enable and direction pins to this district
	//	(which will then automatically configure the
	//	driver object and initiate use of ADC manager)
	//
	void assign( byte enable, byte direction, byte adc_pin, byte adc_number );

	//
	//	Task Entry point from the task manager.
	//
	virtual void process( byte handle );

	//
	//	Control the power on this district.
	//
	void power( bool on );

	//
	//	Return current load average 0-100
	//
	byte load_average( void );

	//
	//	Return the state of this district
	//
	district_state state( void );
};


#endif


//
//	EOF
//

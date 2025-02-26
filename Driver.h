//
//	Driver.h
//	========
//
//	Define the interface between the DCC Generator code and the
//	physical pins through which the signal is modulated.
//

#ifndef _DRIVER_H_
#define _DRIVER_H_

//
//	Get the environment.
//
#include "Configuration.h"
#include "Environment.h"
#include "Pin_IO.h"
#include "Districts.h"

//
//	Define the class that encapsulates the output pin code.
//
//
//	This implementation adopts the "pin at a time" mechanism
//	so allowing for output pins to be arranged to suite the MCU.
//
class Driver {
public:
	//
	//	Define the maximum number of districts which the
	//	class will accommodate.
	//
	static const byte	maximum_districts = Districts::districts;
	
private:
	//
	//	What we need to know about each "district" of the DCC
	//	environment.
	//
	struct driver_record {
		Pin_IO		enable,
				direction;
	};
	//
	//	Define the array of pins which we will be using to
	//	access to real world.
	//
	driver_record	_district[ maximum_districts ];
	byte		_districts;


public:
	//
	//	Constructor
	//
	Driver( void );

	//
	//	Add a new pin to the configuration of the driver where
	//	the pin is referenced by the "platform" pin number.
	//
	bool add( byte *index, byte enable, byte direction );
	
	//
	//	Add a new pin to the configuration of the driver where
	//	the pin is referenced by port number and bit within port.
	//
	bool add( byte *index, byte enable_dev, byte enable_bitno, byte direction_dev, byte direction_bitno );

	//
	//	Now the DCC generator will use the following API calls.
	//
	
	//
	//	Turn on all or one district
	//
	void on( void );
	void on( byte index );

	//
	//	Turn off all or one district
	//
	void off( void );
	void off( byte index );

	//
	//	Toggle the output signal of all or a single district.
	//
	void toggle( void );
	void toggle( byte index );

	//
	//	Generic power on/off call.
	//
	void power( bool on );
	void power( byte index, bool on );
};

//
//	Here we declare the driver object.
//
extern Driver dcc_driver;

#endif

//
//	EOF
//

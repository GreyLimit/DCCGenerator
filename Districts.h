//
//	Districts.h
//	===========
//
//	Declare the module which defines and creates the set
//	of districts supported by the firmware.
//

#ifndef _DISTRICTS_H_
#define _DISTRICTS_H_

//
//	Bring in the stuff we need.
//
#include "Configuration.h"
#include "Environment.h"
#include "District.h"
#include "DCC_District.h"

//
//	Define the object holding all of the districts and providing
//	the "high level" access and control of the districts.
//
class Districts {
public:
	//
	//	Define the number of districts which we are going to handle.
	//
	//	This number here is really limited by the number of ADC lines which
	//	the MCU has available.  On Arduino UNO/Nano this is typically 6
	//	(that is 8 less the I2C SDA/SCL lines).
	//
	//	That being said, this is also limited by the number of H-Bridge
	//	driver devices which can be attached to the MCU.  With the
	//	standard "Arduino Motor Driver Shield" This is actually a
	//	much lower number: 2.
	//
	static const byte districts = DCC_District::districts;
	
private:
	//
	//	Declare the set of districts which we will be managing.
	//
	District	_district[ districts ];

	//
	//	Which zone is powered on.
	//
	byte		_zone;

public:
	//
	//	Allow this to build itself empty first.
	//
	Districts( void );

	//
	//	Initialise the districts and set them into action.
	//
	void initialise( void );

	//
	//	Return the number of the zone currently being operated.
	//
	byte zone( void );

	//
	//	Set the power on for the districts "in zone".
	//
	bool power( byte zone );

	//
	//	Return current load average (0-100) for indicated district
	//
	byte load_average( byte index );

	//
	//	Return the state of this district
	//
	District::district_state state( byte index );
};

//
//	Declare the controlling object.
//
extern Districts districts;

#endif

//
//	EOF
//

//
//	DCC_District.h
//	===============
//
//	Declare the module which defines the hardware interface
//	to the DCC districts.
//

#ifndef _DCC_DISTRICT_H_
#define _DCC_DISTRICT_H_

//
//	Check that the value of DCC_DISTRICTS is valid.
//
#if ( DCC_DISTRICTS != 2 )&&( DCC_DISTRICTS != 6 )
#error "Invalid hardware specification implied"
#endif

//
//	This is the structure used to capture a district.
//
class DCC_District {
public:
	//
	//	Declare the number of districts
	//
	static const byte districts = DCC_DISTRICTS;
	
	//
	//	Flag indicating there is no brake pin to manage.
	//
	static const byte no_brake = 255;
	
	//
	//	The table is built from these fields.
	//
	byte	enable,
		direction,
		adc_pin,
		adc_test,
		brake,
		zone;

	//
	//	The table is defined here.
	//
	static const DCC_District district[ districts ] PROGMEM;
}; 

#endif


//
//	EOF
//

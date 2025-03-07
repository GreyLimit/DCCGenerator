//
//	DCC_District.cpp
//	================
//
//	Specify the table declared in the header file.
//

#include "Configuration.h"
#include "Environment.h"
#include "Parameters.h"

#include "DCC_District.h"


#if DCC_DISTRICTS == 2
	//
	//	Arduino Motor Shield
	//	====================
	//
	//	A and B drivers available, with 4 pins allocated to each.
	//
	//	These are the pin numbers "as per the motor shield".
	//
	//	IMPORTANT:
	//
	//		You *must* cut the VIN CONNECT traces on the
	//		back of the Motor Shield.  You will need to
	//		power the shield separately from the Arduino
	//		with 15 volts DC, and leaving the VIN CONNECT
	//		in place will put 15 volts across the Arduino
	//		and probably damage it.
	//
	//		Previous versions of this firmware (and the DCC++
	//		firmware) also required that the BRAKE feature
	//		(for both A and B H-Bridges) should be cut too.
	//		This is no longer required as this firmware
	//		explicitly sets these LOW and are not touched
	//		after that.
	//
	//		Also (unlike the DCC++ firmware) this firmware
	//		requires no additional jumpers to support its
	//		intended operation.
	//
	#define SHIELD_DRIVER_A_DIRECTION	12
	#define SHIELD_DRIVER_A_ENABLE		3
	#define SHIELD_DRIVER_A_BRAKE		9
	#define SHIELD_DRIVER_A_LOAD		A0
	#define SHIELD_DRIVER_A_ANALOGUE	0
	#define SHIELD_DRIVER_A_ZONE		Zone_Main_Track

	#define SHIELD_DRIVER_B_DIRECTION	13
	#define SHIELD_DRIVER_B_ENABLE		11
	#define SHIELD_DRIVER_B_BRAKE		8
	#define SHIELD_DRIVER_B_LOAD		A1
	#define SHIELD_DRIVER_B_ANALOGUE	1
	#define SHIELD_DRIVER_B_ZONE		Zone_Programming_Track


	const DCC_District DCC_District::district[ DCC_District::districts ] PROGMEM = {
		//
		//	enable			direction			adc_pin			adc_test			brake			Zone
		//	------			---------			-------			--------			-----			----
		//
		{	SHIELD_DRIVER_A_ENABLE,	SHIELD_DRIVER_A_DIRECTION,	SHIELD_DRIVER_A_LOAD,	SHIELD_DRIVER_A_ANALOGUE,	SHIELD_DRIVER_A_BRAKE,	SHIELD_DRIVER_A_ZONE	},
		{	SHIELD_DRIVER_B_ENABLE,	SHIELD_DRIVER_B_DIRECTION,	SHIELD_DRIVER_B_LOAD,	SHIELD_DRIVER_B_ANALOGUE,	SHIELD_DRIVER_B_BRAKE,	SHIELD_DRIVER_B_ZONE	}
	};

#endif
#if DCC_DISTRICTS == 6

	//
	//	The interface details for the bespoke
	//	DCC six district back-plane and Nano.
	//
	#define SHIELD_DRIVER_1_DIRECTION	bit(0)
	#define SHIELD_DRIVER_1_ENABLE		2
	#define SHIELD_DRIVER_1_BRAKE		DCC_District::no_brake
	#define SHIELD_DRIVER_1_LOAD		A0
	#define SHIELD_DRIVER_1_ANALOGUE	0
	#define SHIELD_DRIVER_1_ZONE		Zone_Main_Track
	//
	#define SHIELD_DRIVER_2_DIRECTION	bit(1)
	#define SHIELD_DRIVER_2_ENABLE		3
	#define SHIELD_DRIVER_2_BRAKE		DCC_District::no_brake
	#define SHIELD_DRIVER_2_LOAD		A1
	#define SHIELD_DRIVER_2_ANALOGUE	1
	#define SHIELD_DRIVER_2_ZONE		Zone_Main_Track
	//
	#define SHIELD_DRIVER_3_DIRECTION	bit(2)
	#define SHIELD_DRIVER_3_ENABLE		4
	#define SHIELD_DRIVER_3_BRAKE		DCC_District::no_brake
	#define SHIELD_DRIVER_3_LOAD		A2
	#define SHIELD_DRIVER_3_ANALOGUE	2
	#define SHIELD_DRIVER_3_ZONE		Zone_Main_Track
	//
	#define SHIELD_DRIVER_4_DIRECTION	bit(3)
	#define SHIELD_DRIVER_4_ENABLE		5
	#define SHIELD_DRIVER_4_BRAKE		DCC_District::no_brake
	#define SHIELD_DRIVER_4_LOAD		A3
	#define SHIELD_DRIVER_4_ANALOGUE	3
	#define SHIELD_DRIVER_4_ZONE		Zone_Main_Track
	//
	#define SHIELD_DRIVER_5_DIRECTION	bit(4)
	#define SHIELD_DRIVER_5_ENABLE		6
	#define SHIELD_DRIVER_5_BRAKE		DCC_District::no_brake
	#define SHIELD_DRIVER_5_LOAD		A6
	#define SHIELD_DRIVER_5_ANALOGUE	6
	#define SHIELD_DRIVER_5_ZONE		Zone_Main_Track
	//
	#define SHIELD_DRIVER_6_DIRECTION	bit(5)
	#define SHIELD_DRIVER_6_ENABLE		7
	#define SHIELD_DRIVER_6_BRAKE		DCC_District::no_brake
	#define SHIELD_DRIVER_6_LOAD		A7
	#define SHIELD_DRIVER_6_ANALOGUE	7
	#define SHIELD_DRIVER_6_ZONE		Zone_Programming_Track
	//
	const DCC_District DCC_District::district[ DCC_District::districts ] PROGMEM = {
		//
		//	enable			direction			adc_pin			adc_test			brake			Zone
		//	------			---------			-------			--------			-----			----
		//
		{	SHIELD_DRIVER_1_ENABLE,	SHIELD_DRIVER_1_DIRECTION,	SHIELD_DRIVER_1_LOAD,	SHIELD_DRIVER_1_ANALOGUE,	SHIELD_DRIVER_1_BRAKE,	SHIELD_DRIVER_1_ZONE	},
		{	SHIELD_DRIVER_2_ENABLE,	SHIELD_DRIVER_2_DIRECTION,	SHIELD_DRIVER_2_LOAD,	SHIELD_DRIVER_2_ANALOGUE,	SHIELD_DRIVER_2_BRAKE,	SHIELD_DRIVER_2_ZONE	},
		{	SHIELD_DRIVER_3_ENABLE,	SHIELD_DRIVER_3_DIRECTION,	SHIELD_DRIVER_3_LOAD,	SHIELD_DRIVER_3_ANALOGUE,	SHIELD_DRIVER_3_BRAKE,	SHIELD_DRIVER_3_ZONE	},
		{	SHIELD_DRIVER_4_ENABLE,	SHIELD_DRIVER_4_DIRECTION,	SHIELD_DRIVER_4_LOAD,	SHIELD_DRIVER_4_ANALOGUE,	SHIELD_DRIVER_4_BRAKE,	SHIELD_DRIVER_4_ZONE	},
		{	SHIELD_DRIVER_5_ENABLE,	SHIELD_DRIVER_5_DIRECTION,	SHIELD_DRIVER_5_LOAD,	SHIELD_DRIVER_5_ANALOGUE,	SHIELD_DRIVER_5_BRAKE,	SHIELD_DRIVER_5_ZONE	},
		{	SHIELD_DRIVER_6_ENABLE,	SHIELD_DRIVER_6_DIRECTION,	SHIELD_DRIVER_6_LOAD,	SHIELD_DRIVER_6_ANALOGUE,	SHIELD_DRIVER_6_BRAKE,	SHIELD_DRIVER_6_ZONE	}
	};


#endif


//
//	EOF
//

//
//	DCC_Constant.h
//	==============
//
//	Define an independent class that captures all the constants
//	that are a key component of the DCC environment.  This is
//	done to decouple classes from the DCC class itself, with
//	uses other classes (creating cyclic definitions).
//

#ifndef _DCC_CONSTANT_H_
#define _DCC_CONSTANT_H_

//
//	For assertions.
//
#include "Code_Assurance.h"

//
//	The DCC Constants
//
class DCC_Constant {
public:
	//
	//	DCC protocol constants
	//	----------------------
	//
	static const word	broadcast_address	= 0;
	static const word	minimum_address		= 1;
	static const word	maximum_short_address	= 127;
	static const word	maximum_address		= 10239;
	//
	static const byte	stationary		= 0;
	static const byte	emergency_stop		= 1;
	static const byte	minimum_speed		= 2;
	static const byte	maximum_speed		= 127;
	//
	static const byte	direction_backwards	= 0;
	static const byte	direction_forwards	= 1;
	//
	//	Internal DCC Accessory addresses.  The address structure
	//	as defined in the DCC protocol.
	//
	static const word	minimum_acc_address	= 0;
	static const word	maximum_acc_address	= 511;
	static const word	minimum_acc_sub_address	= 0;
	static const word	maximum_acc_sub_address	= 3;
	//
	//	External combined DCC accessory address.  The
	//	address range as commonly used by external
	//	software systems.
	//
	static const word	minimum_ext_address	= 1;
	static const word	maximum_ext_address	= 2044;
	//
	static const byte	accessory_off		= 0;
	static const byte	accessory_on		= 1;
	//
	//	The DCC standard specifies CV values between 1 and 1024,
	//	but the actual "on wire" protocol utilised the values 0
	//	to 1023.
	//
	static const word	minimum_cv_address	= 1;
	static const word	maximum_cv_address	= 1024;
	//
	//	Function numbers within a decoder
	//
	static const byte	minimum_func_number	= 0;
	static const byte	maximum_func_number	= 28;
	//
	static const byte	bit_map_array		= 4;
	//
	static const byte	function_off		= 0;
	static const byte	function_on		= 1;
	static const byte	function_toggle		= 2;		// This is specific to this firmware.

	//
	//	Data verification routines
	//
	static bool valid_mobile_target( word target ) {
		return(( target >= minimum_address )||( target <= maximum_address ));
	}
	
	static bool valid_mobile_speed( byte speed ) {
		//
		//	Any value less than or equal to maximum speed is
		//	a valid speed.
		//
		return( speed <= maximum_speed );
	}

	static bool stationary_speed( byte speed ) {
		return(( speed == stationary )||( speed == emergency_stop ));
	}

	static bool in_motion_speed( byte speed ) {
		return(( speed >= minimum_speed )&&( speed <= maximum_speed ));
	}
	
	static bool valid_mobile_direction( byte direction ) {
		return(( direction == direction_forwards )||( direction == direction_backwards ));
	} 

	static bool valid_accessory_ext_address( word adrs ) {
		return(( adrs >= minimum_ext_address )&&( adrs <= maximum_ext_address ));
	}

	static bool valid_accessory_address( word adrs ) {
		//
		//	Since variable is unsigned and base address is
		//	zero there is no test for this.
		//
		return( adrs <= maximum_acc_address );
	}
	static bool valid_accessory_sub_address( word adrs ) {
		//
		//	Since variable is unsigned and base address is
		//	zero there is no test for this.
		//
		return( adrs <= maximum_acc_sub_address );
	}

	static bool valid_accessory_state( byte state ) {
		return(( state == accessory_on )||( state == accessory_off ));
	}

	static bool valid_function_number( byte func ) {
		//
		//	Since variable is unsigned and base function
		//	number is zero there is no test for this.
		//
		return( func <= maximum_func_number );
	}

	static bool valid_function_state( byte state ) {
		return(( state == function_off )||( state == function_on )||( state == function_toggle ));
	}

	//
	//	DCC Accessory Address conversion
	//	--------------------------------
	//
	//	Define a routine which, given an accessory target address and sub-
	//	address, returns a numerical value which is the external unified
	//	equivalent value.
	//

	//
	//	Define two routines which, given and external accessory number,
	//	return the DCC accessory address and sub-address.
	//
	static word internal_acc_adrs( word target ) {

		ASSERT(( target >= minimum_ext_address )&&( target <= maximum_ext_address ));

		return((( target - 1 ) >> 2 ) + 1 );
	}
	static byte internal_acc_subadrs( word target ) {

		ASSERT(( target >= minimum_ext_address )&&( target <= maximum_ext_address ));

		return(( target - 1 ) & 3 );
	}
};


#endif


//
//	EOF
//

//
//	Protocol.h
//	==========
//
//	Declare a set of symbols which capture the fixed elements
//	of the communications protocol between a controlling
//	computer and the Train Controller.
//

#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include "Environment.h"
#include "Parameters.h"
#include "Configuration.h"
#include "Task_Entry.h"
#include "DCC_Constant.h"

//
//	This class defines and handles the communications protocol
//	between a computer and the Train Controller (via the USB link).
//
class Protocol : public Task_Entry {
public:
	//
	//	The lead-in and lead-out characters.
	//
	static const char	lead_in = '[';
	static const char	lead_out = ']';
	//
	//	DCC command letters.
	//
	static const char	mobile = 'M';		// Mobile speed and direction.
	static const char	accessory = 'A';	// Accessory control.
	static const char	function = 'F';		// Mobile function control.
	static const char	rewrite_state = 'W';	// Mobile decoder state re-write.
	//
	//	Controller reporting.
	//
	static const char	error = 'E';		// Returned error report.
	//
	//	Controller configuration.
	//
	static const char	power = 'P';		// Track power control.
	static const char	eeprom ='Q';		// EEPROM accessing command.
	
	//
	//	Mobile device ranges
	//
	static inline bool	valid_mobile_target( int t ) { return(( t >= (int)DCC_Constant::minimum_address )&&( t <= (int)DCC_Constant::maximum_address )); }
	//
	//	Note that we will need to adjust the protocol idea of speed (-1 = emergency, 0-126 speed)
	//	to the DCC generators view (the actual DCC packet view) where '1' is emergency stop.
	//
	static const int	minimum_mobile_speed = 0;
	static const int	maximum_mobile_speed = DCC_Constant::maximum_speed-1;
	static const int	emergency_mobile_stop = -1;
	static inline bool	valid_mobile_speed( int s ) { return(( s >= minimum_mobile_speed )&&( s <= maximum_mobile_speed )); }
	//
	static inline bool	valid_mobile_dir( int d ) { return(( d == (int)DCC_Constant::direction_forwards )||( d == (int)DCC_Constant::direction_backwards )); }
	
	//
	//	Accessory veroification
	//
	static inline bool	valid_accessory_address( int a ) { return(( a >= (int)DCC_Constant::minimum_ext_address )&&( a <= (int)DCC_Constant::maximum_ext_address )); }
	static inline bool	valid_accessory_state( int s ) { return(( s == (int)DCC_Constant::accessory_on )||( s == (int)DCC_Constant::accessory_off )); }
	
	//d
	//	Function control verification
	//
	static inline bool	valid_function_number( int f ) { return(( f >= (int)DCC_Constant::minimum_func_number )&&( f <= (int)DCC_Constant:: maximum_func_number )); }
	static inline bool	valid_function_state( int s ) { return(( s == (int)DCC_Constant::function_on )||( s == (int)DCC_Constant::function_off )||( s == (int)DCC_Constant::function_toggle )); }

	//
	//	Valid bitmap values for rewrite state command
	//
	static inline bool	valid_bitmap_value( int b ) { return(( b >= 0 )||( b <= 255 )); }
	
	//
	//	Power Zones
	//
	static const int	power_off_zone = 0;
	static const int	power_main_zone = 1;
	static const int	power_program_zone = 2;
	static inline bool	valid_power_zone( int z ) { return(( z >= power_off_zone )&&( z <= power_program_zone )); }

	//
	//	Define the maximum number of numeric arguments
	//	after the command character.
	//
	static const byte	maximum_arguments = 10;

	//
	//	Define the size of the input buffer area.
	//
	static const byte	buffer_size = 32;

private:
	//
	//	Define the input state and buffer variables.
	//
	bool		_inside,
			_valid;
	char		_buffer[ buffer_size ];
	byte		_len;
	
	//
	//	Define simple "in string" number parsing routine.
	//
	char *parse_number( char *buf, bool *found, int *value );

	//
	//	Break a received packet into its logical elements.
	//
	byte parse_input( char *buf, char *cmd, int *arg, int max );
	
	//	Parse an input buffer.
	//
	void parse_buffer( char *buf );

public:
	//
	//	Constructor.
	//
	Protocol( void );
	
	//
	//	Set up this object and link into other systems.
	//
	void initialise( void );

	//
	//	The task entry point.
	//
	virtual void process( byte handle );
};

//
//	Here is the protocol engine.
//
extern Protocol protocol;

#endif

//
//	EOF
//

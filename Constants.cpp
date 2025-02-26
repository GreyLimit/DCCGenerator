///
///	Constants.cpp	--	A configurable constants module
///
///	Copyright (c) 2021 Jeff Penfold.  All right reserved.
///
///	This library is free software; you can redistribute it and/or
///	modify it under the terms of the GNU Lesser General Public
///	License as published by the Free Software Foundation; either
///	version 2.1 of the License, or (at your option) any later version.
///
///	This library is distributed in the hope that it will be useful,
///	but WITHOUT ANY WARRANTY; without even the implied warranty of
///	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
///	Lesser General Public License for more details.
///
///	You should have received a copy of the GNU Lesser General Public
///	License along with this library; if not, write to the Free Software
///	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301
///	USA
///

//
//	Bring in the EEPROM access mechanism.
//
#include <EEPROM.h>

//
//	The configuration of the firmware.
//
#include "Configuration.h"
#include "Trace.h"

//
//	Bring in our constant interface definition.
//
#include "Constants.h"

//
//	The declaration of the constants space:
//
Constants constant;

//
//	Define the static configurational data that enables
//	us to perform initial setup of the constants and subsequent
//	updates.
//
typedef struct {
	const char	*name;
	const word	initial,
			*loc_w;
	const byte	*loc_b;
} ConstantValue;

//
//	Constant names kept in program memory.
//
static const char string_im[] PROGMEM = "identification_magic";
static const char string_icl[] PROGMEM = "instant_current_limit";
static const char string_aci[] PROGMEM = "average_current_index";
static const char string_acl[] PROGMEM = "average_current_limit";
static const char string_pgp[] PROGMEM = "power_grace_period";
static const char string_pi[] PROGMEM = "periodic_interval";
static const char string_lui[] PROGMEM = "lcd_update_interval";
static const char string_lri[] PROGMEM = "line_refresh_interval";
static const char string_kpri[] PROGMEM = "keypad_reading_interval";
static const char string_lkp[] PROGMEM = "long_key_press";
static const char string_drp[] PROGMEM = "driver_reset_period";
static const char string_dpp[] PROGMEM = "driver_phase_period";
static const char string_rsp[] PROGMEM = "rotary_scan_period";
static const char string_rup[] PROGMEM = "rotary_update_period";
static const char string_dlp[] PROGMEM = "dynamic_load_period";
static const char string_dlr[] PROGMEM = "dynamic_load_reports";
static const char string_bdt[] PROGMEM = "banner_display_time";
static const char string_tcr[] PROGMEM = "transient_command_repeats";
static const char string_smrr[] PROGMEM = "service_mode_reset_repeats";
static const char string_smcr[] PROGMEM = "service_mode_command_repeats";

//
//	This is the static table of constants support information.
//
static const ConstantValue constant_value[ CONSTANTS ] PROGMEM = {
// 0
	{ string_im,	DEFAULT_IDENTIFICATION_MAGIC,		&IDENTIFICATION_MAGIC,		NULL					},
	{ string_icl,	DEFAULT_INSTANT_CURRENT_LIMIT,		&INSTANT_CURRENT_LIMIT,		NULL					},
	{ string_aci,	DEFAULT_AVERAGE_CURRENT_INDEX,		NULL,				&AVERAGE_CURRENT_INDEX			},
	{ string_acl,	DEFAULT_AVERAGE_CURRENT_LIMIT,		&AVERAGE_CURRENT_LIMIT,		NULL					},
	{ string_pgp,	DEFAULT_POWER_GRACE_PERIOD,		&POWER_GRACE_PERIOD,		NULL					},
// 5
	{ string_pi,	DEFAULT_PERIODIC_INTERVAL,		&PERIODIC_INTERVAL,		NULL					},
	{ string_lui,	DEFAULT_LCD_UPDATE_INTERVAL,		&LCD_UPDATE_INTERVAL,		NULL					},
	{ string_lri,	DEFAULT_LINE_REFRESH_INTERVAL,		&LINE_REFRESH_INTERVAL,		NULL					},
	{ string_kpri,	DEFAULT_KEYPAD_READING_INTERVAL,	&KEYPAD_READING_INTERVAL,	NULL					},
	{ string_lkp,	DEFAULT_LONG_KEY_PRESS,			&LONG_KEY_PRESS,		NULL					},
// 10
	{ string_drp,	DEFAULT_DRIVER_RESET_PERIOD,		&DRIVER_RESET_PERIOD,		NULL					},
	{ string_dpp,	DEFAULT_DRIVER_PHASE_PERIOD,		&DRIVER_PHASE_PERIOD,		NULL					},
	{ string_rsp,	DEFAULT_ROTARY_SCAN_PERIOD,		&ROTARY_SCAN_PERIOD,		NULL					},
	{ string_rup,	DEFAULT_ROTARY_UPDATE_PERIOD,		&ROTARY_UPDATE_PERIOD,		NULL					},
	{ string_dlp,	DEFAULT_DYNAMIC_LOAD_PERIOD,		&DYNAMIC_LOAD_PERIOD,		NULL					},
// 15
	{ string_dlr,	DEFAULT_DYNAMIC_LOAD_REPORTS,		NULL,				&DYNAMIC_LOAD_REPORTS			},
	{ string_bdt,	DEFAULT_BANNER_DISPLAY_TIME,		NULL,				&BANNER_DISPLAY_TIME			},
	{ string_tcr,	DEFAULT_TRANSIENT_COMMAND_REPEATS,	NULL,				&TRANSIENT_COMMAND_REPEATS		},
	{ string_smrr,	DEFAULT_SERVICE_MODE_RESET_REPEATS,	NULL,				&SERVICE_MODE_RESET_REPEATS		},
	{ string_smcr,	DEFAULT_SERVICE_MODE_COMMAND_REPEATS,	NULL,				&SERVICE_MODE_COMMAND_REPEATS		}
};

//
//	word checksum_consts( void );
//	-----------------------------
//
//	Return the checksum value of the memory constant array
//	for either setting or checking the values in the structure.
//
static word checksum_consts( void ) {
	
	STACK_TRACE( "static word checksum_consts( void )" );
	
	word	s;

	s = 0xffff;
	for( int i = 0; i < ConstantArea; i++ ) {

		static const int cs_slide = 3;
		static const int wd_bits = sizeof( word ) * 8;
		static const int cs_revs = wd_bits - cs_slide;

		s = ( s << cs_slide )|( s >> cs_revs );
		s ^= constant.var.check.memory[ i ];
	}
	return( s );
}

//
//	void record_constants( void );
//	------------------------------
//
//	Re-write the constants back to the EEPROM.
//
void record_constants( void ) {
	
	STACK_TRACE( "void record_constants( void )" );
	
	constant.var.check.sum = checksum_consts();
	EEPROM.put( 0, constant );
}

//
//	RESET all Constants to default values.
//
void reset_constants( void ) {
	
	STACK_TRACE( "void reset_constants( void )" );
	
	//
	//	Resetting the entire constants area to zero before
	//	initialising  the "variable" constants from EEPROM.
	//
	{
		byte	*ptr;
		int	left;
		
		ptr = constant.var.check.memory;
		left = ConstantArea;
		while( left-- ) *ptr++ = 0;
	}
	//
	//	Initialise the variable constants.
	//
	for( int i = 0; i < CONSTANTS; i++ ) {
		word	*w;
		byte	*b;

		if(( w = progmem_read_address( constant_value[ i ].loc_w ))) {
			*w = progmem_read_word( constant_value[ i ].initial );
		}
		else {
			b = progmem_read_address( constant_value[ i ].loc_b );
			*b = (byte)progmem_read_word( constant_value[ i ].initial );
		}
	}
	record_constants();
}

//
//	void initialise_constants( void )
//	---------------------------------
//
//	Perform *immediately* as the first item in setup() to load
//	verified constan value or reset constants to initial default
//	values.
//
void initialise_constants( void ) {
	
	STACK_TRACE( "void initialise_constants( void )" );
	
	//
	//	Read in any saved constant values..
	//
	EEPROM.get( 0, constant );
	//
	//	Verify memory read is acceptable and
	//	reset all if not valid.
	//
	if(( constant.var.check.sum != checksum_consts())||( IDENTIFICATION_MAGIC != DEFAULT_IDENTIFICATION_MAGIC )) reset_constants();
}

//
//	int find_constant( int index, char **name, word **adrs );
//	---------------------------------------------------------
//
//	Place the name of the constant at index "i" (initial value
//	as 0) at "name" and where its current value is in "adrs".
//
//	Returns either the index of the next constant or ERROR if
//	there is no constant at this index.
//
//	The name will be in PROGMEM.
//
int find_constant( int index, char **name, byte **adrs_b, word **adrs_w ) {
	
	STACK_TRACE( "int find_constant( int index, char **name, byte **adrs_b, word **adrs_w )" );
	
	if(( index < 0 )||( index >= CONSTANTS )) return( ERROR );
	*name = (char *)progmem_read_address( constant_value[ index ].name );
	*adrs_b = (byte *)progmem_read_address( constant_value[ index ].loc_b );
	*adrs_w = (word *)progmem_read_address( constant_value[ index ].loc_w );
	return( index + 1 );
}

//
//	EOF
//

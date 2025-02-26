///
///	Constants.h		A configurable constants module
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
//	This is the module that captures the specifics of constant
//	values which might need tuning, but without the requirement
//	to recompile the firmware.
//

#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include "Environment.h"
#include "Menu.h"
#include "Magic.h"

//
//	The constants will be managed through a two tier system of
//	static data (embedded in the program memory) and a data
//	structure containing constant values (held in the variable
//	space).
//
//	Define the number of constants we have to manage:
//
#define CONSTANTS	20

//
//	The following structure is the variable space definition
//	from which constant values are extracted at execution time.
//
typedef struct {
	//
	//	The version/format identification symbol alllowing the
	//	firmware to confirm that the data is suitable for it or
	//	if the constants need to be reset.
	//
	word		identification_magic;
	//
	//	The following word and byte values form the constants
	//	that modify the functional behaviour of the firmware.
	//
	word		instant_current_limit,			// 1
			average_current_limit,
			power_grace_period,
			periodic_interval,
			lcd_update_interval,			// 5
			line_refresh_interval,
			keypad_reading_interval,
			long_key_press,
			driver_reset_period,
			driver_phase_period,			// 10
			rotary_scan_period,
			rotary_update_period,
			dynamic_load_period;
	byte		average_current_index,
			dynamic_load_reports,			// 15
			banner_display_time,
			transient_command_repeats,
			service_mode_reset_repeats,
			service_mode_command_repeats;
	//
	//	The following area contains the "Page Memory" managed by
	//	the user through the menu system.
	//
	page_memory	pages;
} ConstantValues;

static const int ConstantArea = sizeof( ConstantValues );

typedef struct {
	byte	memory[ ConstantArea ];
	word	sum;
} ConstantMemory;

typedef struct {
	union {
		ConstantValues	value;
		ConstantMemory	check;
	} var;
} Constants;

//
//	This is the definition of that space:
//
extern Constants constant;

//
//	High Level Configuration values.
//	================================
//
//	These definitions alter operational elements
//	of the solution, but not the functional aspects.
//

//
//	Define the "Magic" number which provides a secondary
//	verification that the content of the constants is
//	suitable for the operation of the firmware.
//
//	The default value is "built" using the MAGIC() macro
//	defined in "Magic.h".
//
#define DEFAULT_IDENTIFICATION_MAGIC	MAGIC(2024,12,9)
#define IDENTIFICATION_MAGIC		constant.var.value.identification_magic

//
//	Define the absolute current limit value (0-1023) at which
//	the power is removed from the track (current spike).
//
//	The average current limit is the maximum difference in
//	current and average reading allowed before a overload condition
//	is declared
//
#define DEFAULT_INSTANT_CURRENT_LIMIT	800
#define INSTANT_CURRENT_LIMIT		constant.var.value.instant_current_limit
//
#define DEFAULT_AVERAGE_CURRENT_INDEX	8
#define AVERAGE_CURRENT_INDEX		constant.var.value.average_current_index
//
#define DEFAULT_AVERAGE_CURRENT_LIMIT	700
#define AVERAGE_CURRENT_LIMIT		constant.var.value.average_current_limit

//
//	The grace period during which, after applying power to a district,
//	any overloads are ignored (in milliseconds).
//
#define DEFAULT_POWER_GRACE_PERIOD	1000
#define POWER_GRACE_PERIOD		constant.var.value.power_grace_period

//
//	Define the periodic interval in milliseconds.
//
#define DEFAULT_PERIODIC_INTERVAL	1000
#define PERIODIC_INTERVAL		constant.var.value.periodic_interval

//
//	Define the overall interval at which the LCD is updated,
//	in milliseconds.
//
#define DEFAULT_LCD_UPDATE_INTERVAL	1000
#define LCD_UPDATE_INTERVAL		constant.var.value.lcd_update_interval

//
//	Define the display line refresh interval.  This is the number
//	of milliseconds paused between each of the status lines being
//	updated.
//
#define DEFAULT_LINE_REFRESH_INTERVAL	250
#define LINE_REFRESH_INTERVAL		constant.var.value.line_refresh_interval

//
//	Define the interval the HCI uses to check for keyboard input.
//
#define DEFAULT_KEYPAD_READING_INTERVAL	500
#define KEYPAD_READING_INTERVAL		constant.var.value.keypad_reading_interval
//
//	Define the duration after which a key press is considered "long".
//
#define DEFAULT_LONG_KEY_PRESS		750
#define LONG_KEY_PRESS			constant.var.value.long_key_press

//
//	Define the Driver/District reset period (in milliseconds)
//	This is the duration through which an individual driver is
//	disabled as a result of a power exception (spike/overload)
//	before a restart is attempted.
//
#define DEFAULT_DRIVER_RESET_PERIOD	10000
#define DRIVER_RESET_PERIOD		constant.var.value.driver_reset_period

//
//	Define the Driver/District phase test period.  This is the
//	time after the firmware flips a driver/district phasing
//	before concluding that the new phase is all OK.
//
#define DEFAULT_DRIVER_PHASE_PERIOD	100
#define DRIVER_PHASE_PERIOD		constant.var.value.driver_phase_period

//
//	Define the number of times various types of packets are repeated.
//
//	TRANSIENT_COMMAND_REPEATS
//	
//		Operational, non-mobile, commands are considered "non-
//		permanent" DCC commands and are repeated before it is
//		automatically dropped.
//
//	SERVICE_MODE_RESET_REPEATS
//
//		The number of times a service mode reset command is
//		repeated before and after a service mode command.
//
//	SERVICE_MODE_COMMAND_REPEATS
//
//		The number of times a service mode action command is
//		repeated.
//
#define DEFAULT_TRANSIENT_COMMAND_REPEATS	8
#define TRANSIENT_COMMAND_REPEATS		constant.var.value.transient_command_repeats
//
#define DEFAULT_SERVICE_MODE_RESET_REPEATS	20
#define SERVICE_MODE_RESET_REPEATS		constant.var.value.service_mode_reset_repeats
//
#define DEFAULT_SERVICE_MODE_COMMAND_REPEATS	10
#define SERVICE_MODE_COMMAND_REPEATS		constant.var.value.service_mode_command_repeats

//
//	Rotary scan period defines how often the rotary
//	controller is check for changes in the knobs
//	rotation / button press.
//
#define DEFAULT_ROTARY_SCAN_PERIOD		5
#define ROTARY_SCAN_PERIOD			constant.var.value.rotary_scan_period

//
//	Rotary Update Period controls how fast input from
//	the rotary control is applied to the mobile controller
//	being adjusted.
//
#define DEFAULT_ROTARY_UPDATE_PERIOD		350
#define ROTARY_UPDATE_PERIOD			constant.var.value.rotary_update_period

//
//	Dynamic Load period specifies the frequency that
//	the system calculates the "maximum" loading, and
//	hense (if enabled) how often a report is sent to
//	the console.
//
#define DEFAULT_DYNAMIC_LOAD_PERIOD		1000
#define DYNAMIC_LOAD_PERIOD			constant.var.value.dynamic_load_period

//
//	Dynamic Load reports controls if the the Arduino
//	Generator sends dynamic loading reports to the host
//	system (the '[L#]' replies).  0 = off, 1 = on.
//
#define DEFAULT_DYNAMIC_LOAD_REPORTS		0
#define DYNAMIC_LOAD_REPORTS			constant.var.value.dynamic_load_reports

//
//	How long do we display the banner for (on the LCD)
//
#define DEFAULT_BANNER_DISPLAY_TIME		3
#define BANNER_DISPLAY_TIME			constant.var.value.banner_display_time

//
//	Provide the alias through which the page memory can be accessed
//
#define PAGE_MEMORY				constant.var.value.pages


//
//	The Constants API
//	=================
//

//
//	void initialise_constants( void )
//	---------------------------------
//
//	Perform *immediately* as the first item in setup() to load
//	verified constan value or reset constants to initial default
//	values.
//
extern void initialise_constants( void );

//
//	int find_constant( int index, char **name, byte **adrs_b, word **adrs_w );
//	--------------------------------------------------------------------------
//
//	Place the name of the constant at index "i" (initial value
//	as 0) at "name" and where its current value is in either
//	"adrs_b" or "adrs_w" (for BYTE or WORD value) with the
//	unused pointer set to NULL.
//
//	Returns either the index of the next constant or ERROR if
//	there is no constant at this index.
//
//	The name will be in PROGMEM.
//
extern int find_constant( int index, char **name, byte **adrs_b, word **adrs_w );

//
//	void record_constants( void );
//	------------------------------
//
//	Re-write the constants back to the EEPROM.
//
extern void record_constants( void );

//
//	RESET all Constants to default values.
//
extern void reset_constants( void );

#endif

//
//	EOF
//

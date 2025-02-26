//
//	Layout.cpp
//	==========
//
//	Static data associated with the keypad layout.
//

//
//	Firmware configuration
//
#include "Configuration.h"
#include "Trace.h"

//
//	Our definitions
//
#include "Environment.h"
#include "Configuration.h"
#include "Parameters.h"

#include "Layout.h"

//
//	Define the mapping between "row x col" and the ASCII (7-bit)
//	value returned.
//

//
//	Select the keypad orientation that is implied by the position
//	of the IO expansion interface.
//
const byte keypad_mapping[ LAYOUT_KEYS ] PROGMEM = {

#if defined( MIRROR_KEYPAD )
	'D',	'C',	'B',	'A',
	'#',	'9',	'6',	'3',
	'0',	'8',	'5',	'2',
	'*',	'7',	'4',	'1'
#else
	'1',	'2',	'3',	'A',
	'4',	'5',	'6',	'B',
	'7',	'8',	'9',	'C',
	'*',	'0',	'#',	'D'
#endif

};


//
//	EOF
//

//
//	Pin_IO.cpp
//	==========
//

//
//	Bring in our definitions.
//
#include "Pin_IO.h"

//
//	Device Hardware Addresses.
//	==========================
//

//
//	Declare the AVR GPIO address lookup table.
//
static const GPIO_Registers * const gpio_addresses[] PROGMEM = {
	//
	//	Arduino UNO and Nano
	//
#if defined( ARDUINO_AVR_UNO )||defined( ARDUINO_AVR_NANO )
	//
	NIL( GPIO_Registers ),		// GPIO A -- Not implemented.
	(GPIO_Registers *)0x0023,	// GPIO B
	(GPIO_Registers *)0x0026,	// GPIO C
	(GPIO_Registers *)0x0029	// GPIO D

	//
	//	Arduino Mega2560
	//
#elif defined( ARDUINO_AVR_MEGA2560 )
	//
	(GPIO_Registers *)0x0020,	// GPIO A
	(GPIO_Registers *)0x0023,	// GPIO B
	(GPIO_Registers *)0x0026,	// GPIO C
	(GPIO_Registers *)0x0029,	// GPIO D
	(GPIO_Registers *)0x002C,	// GPIO E
	(GPIO_Registers *)0x002F,	// GPIO F
	(GPIO_Registers *)0x0032,	// GPIO G
	(GPIO_Registers *)0x0100,	// GPIO H
	NIL( GPIO_Registers ),		// GPIO I -- Not implemented.
	(GPIO_Registers *)0x0103,	// GPIO J
	(GPIO_Registers *)0x0106,	// GPIO K
	(GPIO_Registers *)0x0109	// GPIO L

	//
	//	Arduino Nano Every
	//
#elif defined( ARDUINO_AVR_NANO_EVERY )
	//
	(GPIO_Registers *)0x0400,	// GPIO A
	(GPIO_Registers *)0x0420,	// GPIO B
	(GPIO_Registers *)0x0440,	// GPIO C
	(GPIO_Registers *)0x0460,	// GPIO D
	(GPIO_Registers *)0x0480,	// GPIO E
	(GPIO_Registers *)0x04A0	// GPIO F

#else
#error "Platform.cpp: Target platform not recognised for GPIO addresses."
#endif
};

//
//	The routine which converts an instance number
//	to the hardware address (or NULL) of the associated
//	GPIO registers.
//
static GPIO_Registers *GPIO_address( byte instance ) {
	if( instance >= sizeof( gpio_addresses )) return( NULL );
	return( progmem_read_address( gpio_addresses[ instance ]));
}

//
//	Device Hardware Characteristics
//	===============================
//
//	The "database" of possible pin roles needs to encapsulate a
//	couple of functional factors over what a pin can be used for
//	(see Pin_Role).  This is that an MCU might have multiple
//	separate hardware devices performing a ggiven role, and that
//	each of these devices might be presented at different locations
//	on the pins of the MCU.
//
//	Therefore when a device is required it needs to be specified
//	by the namture of the device (Pin_Role), the instance of this
//	device (0,1,2...) and the possible presentation locations for
//	this device (again, 0,1,2...).
//

//
//	How is the database organised?  Essentially, rather than working
//	backwards for a pin towards a role/device which might be using
//	it, the database starts with a role and works towards the pins.
//
//	Declare a structure that is the head for the data for a specific
//	role
//
//	JEFF
// 


//
//	Define the static, program memory based database of Platform
//	pin numbers to GPIO instance numbers with bit numbers.
//
//	Define some macros to make the creation and use of the
//	database table simpler and less error prone.
//
#define	GPIO_BIT(g,b)	(((g)<<3)|((b)&0x07))
#define GPIO_NUMBER(x)	(((x)>>3)&0x1f)
#define BIT_NUMBER(x)	((x)&0x07)
//
//	Shorthand for simplified table entry:
//
//	Yes, we un-define PI and redefine it
//	to suit our needs.
//
#define	GB(g,b)		GPIO_BIT(g,b)
#define PA		0
#define PB		1
#define PC		2
#define PD		3
#define PE		4
#define PF		5
#define PG		6
#define PH		7
#undef PI
#define PI		8
#define PJ		9
#define PK		10
#define PL		11

//
//	The Translation table is defined as an array of bytes whose
//	content is created using the above GB() macro.
//
static const byte translate_table[] PROGMEM = {

	//
	//	Arduino UNO and Nano
	//
#if defined( ARDUINO_AVR_UNO )||defined( ARDUINO_AVR_NANO )
	//
	GB(PD,0),GB(PD,1),GB(PD,2),GB(PD,3),GB(PD,4),GB(PD,5),GB(PD,6),GB(PD,7),	//  0 ->  7
	GB(PB,0),GB(PB,1),GB(PB,2),GB(PB,3),GB(PB,4),GB(PB,5),GB(PC,0),GB(PC,1),	//  8 -> 15
	GB(PC,2),GB(PC,3),GB(PC,4),GB(PC,5)						// 16 -> 19

	//
	//	Arduino Mega2560
	//
#elif defined( ARDUINO_AVR_MEGA2560 )
	//
	GB(PE,0),GB(PE,1),GB(PE,4),GB(PE,5),GB(PG,5),GB(PE,3),GB(PH,3),GB(PH,4),	//  0 ->  7
	GB(PH,5),GB(PH,6),GB(PB,4),GB(PB,5),GB(PB,6),GB(PB,7),GB(PJ,1),GB(PJ,0),	//  8 -> 15
	GB(PH,1),GB(PH,0),GB(PD,3),GB(PD,2),GB(PD,1),GB(PD,0),GB(PA,0),GB(PA,1),	// 16 -> 23
	GB(PA,2),GB(PA,3),GB(PA,4),GB(PA,5),GB(PA,6),GB(PA,7),GB(PC,7),GB(PC,6),	// 24 -> 31
	GB(PC,5),GB(PC,4),GB(PC,3),GB(PC,2),GB(PC,1),GB(PC,0),GB(PD,7),GB(PG,2),	// 32 -> 39
	GB(PG,1),GB(PG,0),GB(PL,7),GB(PL,6),GB(PL,5),GB(PL,4),GB(PL,3),GB(PL,2),	// 40 -> 47
	GB(PL,1),GB(PL,0),GB(PB,3),GB(PB,2),GB(PB,1),GB(PB,0),GB(PF,0),GB(PF,1),	// 48 -> 55
	GB(PF,2),GB(PF,3),GB(PF,4),GB(PF,5),GB(PF,6),GB(PF,7),GB(PK,0),GB(PK,1),	// 56 -> 63
	GB(PK,2),GB(PK,3),GB(PK,4),GB(PK,5),GB(PK,6),GB(PK,7)				// 64 -> 69

	//
	//	Arduino Nano Every
	//
#elif defined( ARDUINO_AVR_NANO_EVERY )
	//
	GB(PC,5),GB(PC,4),GB(PA,0),GB(PF,5),GB(PC,6),GB(PB,2),GB(PF,4),GB(PA,1),	//  0 ->  7
	GB(PE,3),GB(PB,0),GB(PB,1),GB(PE,0),GB(PE,1),GB(PE,2),GB(PD,3),GB(PD,2),	//  8 -> 15
	GB(PD,1),GB(PD,0),GB(PA,2),GB(PA,3),GB(PD,4),GB(PD,5)				// 16 -> 21

#else
#error "Pin_IO.cpp: Target platform not recognised."
#endif
};

//
//	Routine validates and performs the
//	translation from a Platform Pin number
//	to a GPIO instance and bit number.
//
bool Pin_IO::translate( byte pin, byte *instance, byte *bit_no ) {
	byte	x;
	
	if( pin >= sizeof( translate_table )) return( false );
	x = progmem_read_byte( translate_table[ pin ]);
	*instance = GPIO_NUMBER( x );
	*bit_no = BIT_NUMBER( x );
	return( true );
}

//
//	Default Constructor
//	-------------------
//
//	Produces an object which is not aligned with
//	any device, and should not be used in this mode.
//
Pin_IO::Pin_IO( void ) {
	_adrs = NULL;
	_bit = 0;
}

//
//	Device Constructor
//	--------------------
//
//	Define a GPIO pin via the GPIO instance number
//	combined with the pin bit number (0..7).
//
//	pin	Platform pin number.
//
//	input	True for input pin, false for output pin.
//
//	pull_up	True if the internal 3.3/5 volt pull up
//		resister should be enabled.
//
Pin_IO::Pin_IO( byte dev, byte bit_num, bool input, bool pull_up ) {
	(void)configure( dev, bit_num, input, pull_up );
}

//
//	Platform Constructor
//	--------------------
//
//	Define a GPIO pin via the Platform defined abstract
//	pin numbering scheme.
//
//	pin	Platform pin number.
//
//	input	True for input pin, false for output pin.
//
//	pull_up	True if the internal 3.3/5 volt pull up
//		resister should be enabled.
//
Pin_IO::Pin_IO( byte pin, bool input, bool pull_up ) {
	(void)configure( pin, input, pull_up );
}

//
//	Set/Reset the target of the Pin Object using a
//	device instance number and pin bit number.
//
bool Pin_IO::configure( byte instance, byte bit_no, bool input, bool pull_up ) {
	if(( _adrs = GPIO_address( instance ))) {
		_bit = bit( bit_no );
		if( input ) {
			if( pull_up ) {
				_adrs->input_pullup( _bit );
			}
			else {
				_adrs->input( _bit );
			}
		}
		else {
			_adrs->output( _bit );
		}
		return( true );
	}
	_bit = 0;
	return( false );
}


//
//	Set/Reset the target of the Pin Object using a
//	Platform pin number.
//
bool Pin_IO::configure( byte pin, bool input, bool pull_up ) {
	byte	instance,
		bit_no;
	
	if( !translate( pin, &instance, &bit_no )) return( false );
	return( configure( instance, bit_no, input, pull_up ));
}

//
//	Determine if a pin has been configured.
//
bool Pin_IO::configured( void ) {
	return( _adrs != NULL );
}

//
//	The following functions provide the mechanism
//	to alter, read or write the pin and its config.
//
void Pin_IO::input( void ) {
	_adrs->input( _bit );
}

void Pin_IO::input_pullup( void ) {
	_adrs->input_pullup( _bit );
}

void Pin_IO::output( void ) {
	_adrs->output( _bit );
}

void Pin_IO::high( void ) {
	_adrs->high( _bit );
}

void Pin_IO::low( void ) {
	_adrs->low( _bit );
}

void Pin_IO::set( bool high ) {
	if( high ) {
		_adrs->high( _bit );
	}
	else {
		_adrs->low( _bit );
	}
}

void Pin_IO::toggle( void ) {
	_adrs->toggle( _bit );
}

byte Pin_IO::read( void ) {
	return( _adrs->read( _bit ));
}		


//
//	Default Constructor
//	-------------------
//
//	Produces an object which is not aligned with
//	any port, and should not be used in this mode.
//
Port_IO::Port_IO( void ) {
	_adrs = NIL( GPIO_Registers );
}

//
//	Platform Constructor
//	--------------------
//
//	Define a GPIO pin via the Platform defined abstract
//	pin numbering scheme.
//
//	dev	The port number A=0, B=1, ...
//
//	input	True for input port, false for output port.
//
//	pull_up	True if the internal 3.3/5 volt pull up
//		resister should be enabled.
//
Port_IO::Port_IO( byte dev, bool input, bool pull_up ) {
	(void)configure( dev, input, pull_up );
}

//
//	Set/Reset the target of the Pin Object using a
//	Platform pin number.
//
bool Port_IO::configure( byte dev, bool input, bool pull_up ) {
	if(( _adrs = GPIO_address( dev ))) {
		if( input ) {
			if( pull_up ) {
				_adrs->input_pullup();
			}
			else {
				_adrs->input();
			}
		}
		else {
			_adrs->output();
		}
		return( true );
	}
	return( false );
}

//
//	Determine if a pin has been configured.
//
bool Port_IO::configured( void ) {
	return( _adrs != NIL( GPIO_Registers ));
}

//
//	The following functions provide the mechanism
//	to alter, read or write whole port value.
//
bool Port_IO::read_data( byte *data ) {
	if( _adrs ) {
		*data = _adrs->read();
		return( true );
	}
	return( false );
}

bool Port_IO::write_data( byte data ) {
	if( _adrs ) {
		_adrs->write( data );
		return( true );
	}
	return( false );
}


//
//	EOF
//

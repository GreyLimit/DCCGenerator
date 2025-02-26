//
//	Pin_IO.h
//	========
//

//
//	A version of the high speed Pin IO template class
//	written by Mikael Patel.
//
//	See: https://github.com/mikaelpatel/Arduino-GPIO
//
//	This code has been written to align with the rest
//	of the Library code, but, where most of the other
//	hardware related drivers are based on virtual API
//	classes (and this could be) for speed purposes this
//	is defined as a monolithic, free standing class
//	where the content of the class and methods are set
//	at compile time
//

#ifndef _PIN_IO_H_
#define _PIN_IO_H_

//
//	Configuration and stuff
//
#include "Environment.h"
#include "Parameters.h"
#include "Configuration.h"

//
//	The definitions following are broken into separate
//	functional areas with the aim of simplifying the
//	inclusion of additional MCU platforms without the
//	need to modify the overall API or compromise the
//	performance of the final code.
//

//
//	Declare an enumerated type that captures the IO roles that a
//	given GPIO pin has been assigned.
//
enum Pin_Role : byte {
	role_none		= 0,	// No roles assigned to pin
	role_gpio		= 1,	// General IO pin
	role_twi		= 2,	// TWI/I2C interface
	role_spi		= 3,	// SPI interface
	role_adc		= 4,	// Analogue to Digital conversion
	role_usart		= 5,	// UART/USART interface
	role_pwm		= 6	// PWM/Timer interface
};

//////////////////////////////////////////////////////////
//							//
//	Memory Map for "Port" of 8 bits			//
//	===============================			//
//							//
//////////////////////////////////////////////////////////

//
//	The GPIO_Registers is a class which implements the
//	memory presentation of a GPIO port in memory.  This
//	class should not be "allocated" in the traditional
//	manner, rather it is used as the target of a fixed
//	address and provides abstracted methods for access
//	to the IO pins directly.
//
//	The functions to be provided are:-
//
//	input(b)		Set as input, no pull-up resistor.
//	input_pullup(b)		Set as input with pull-up resistor.
//	output(b)		Set as output.
//
//	high(b)			Set output high
//	low(b)			Set output low
//	toggle(b)		Flip output high<->low
//
//	read(b)			Return pin values.
//
//	In all functions the argument b is a byte value
//	where the set bits indicate the pin elements
//	of the port which are to be modified.
//

class GPIO_Registers {
		//////////////////////////////////////////
		//					//
		//	AVR Architectures		//
		//	=================		//
		//					//
		//////////////////////////////////////////
#if defined( ARDUINO_ARCH_AVR )

	private:
		//
		//	Define the Memory Map of a set
		//	of pins as present in memory.
		//
		//	+0 DDR	Data Direction Register
		//		0 -> Input Pin
		//		1 -> Output Pin
		//
		//	+1 PIN	Data INPUT Register for
		//		all pins where (regardless
		//		of declared direction)
		//
		//		also,
		//
		//		can be used to toggle an
		//		individual pin if 1 written
		//		to the corresponding bit.
		//
		//	+2 PORT	Data OUTPUT Register for
		//		pins where DDR bit is 1,
		//
		//		or,
		//
		//		Where DDR bit 0 (input)
		//		writing a 1 here enables
		//		the internal pull-up
		//		resistor.
		//
		volatile byte	_pin;
		volatile byte	_ddr;
		volatile byte	_port;

	public:
		//
		//	The IO routines required.
		//

		//
		//	The following routines are BYTE
		//	focused and enable the port to
		//	be access/modified as a whole value.
		//
		inline void input( void ) {
			_ddr = 0x00;
			_port = 0x00;
		}
		inline void input_pullup( void ) {
			_ddr = 0x00;
			_port = 0xff;
		}
		inline void output( void ) {
			_ddr = 0xff;
			_port = 0;
		}
		inline void write( byte v ) {
			_port = v;
		}
		inline void toggle( void ) {
			_pin = 0xff;
		}
		inline byte read( void ) {
			return( _pin );
		}
		

		//
		//	The following routines are BIT
		//	focused, taking in an 8 bit
		//	byte where the set bits indicate
		//	the bits to access/modify.
		//
		inline void input( byte b ) {
			_ddr &= ~b;
			_port &= ~b;
		}
		inline void input_pullup( byte b ) {
			_ddr &= ~b;
			_port |= b;
		}
		inline void output( byte b ) {
			_ddr |= b;
			_port &= ~b;
		}
		inline void high( byte b ) {
			_port |= b;
		}
		inline void low( byte b ) {
			_port &= ~b;
		}
		inline void toggle( byte b ) {
			_pin = b;
		}
		inline byte read( byte b ) {
			return( _pin & b );
		}

		//////////////////////////////////////////
		//					//
		//	Mega AVR Architectures		//
		//	======================		//
		//					//
		//////////////////////////////////////////
#elif  defined( ARDUINO_ARCH_MEGAAVR )

	private:
		//
		//	Mega AVR GPIO registers have been extended from
		//	the original AVR registers, thus:
		//
		//	+0 DIR		The direction of each bit in the port.
		// 
		//			0 -> Input
		//			1 -> Output
		//
		//			This port can be READ for the data
		//			Direction information, but the actual
		//			bits in the register should be set or
		//			reset using DIRSET and DIRCLR.
		//
		//	+1 DIRSET	Any bit set to 1 here will make the
		//			corresponding bit in DIR become 1.
		//			Writing a 0 to a bit here does nothing.
		//
		//	+2 DIRCLR	Any bit set to 1 here will make the
		//			corresponding bit in DIR become 0.
		//			Writing a 0 to a bit here does nothing.
		//
		//	+3 DIRTGL	Any bit set to 1 here will make the
		//			corresponding bit in DIR toggle 0<->1.
		//			Writing a 0 to a bit here does nothing.
		//
		//	The Output spect of the GPIO port is managed in the
		//	same manner as the DIRection data.
		//	
		//	+4 OUT		Data being output through the GPIO can
		//			be written as a whole byte value.
		//	+5 OUTSET	Set specific bits in OUT.
		//	+6 OUTCLR	Clear specific bits in OUT.
		//	+7 OUTTGL	Toggle specific bits in OUT.
		//
		//	+8 IN		Return the value of any input pins.
		//
		//	+9 INTFLAGS	Interrupt flags for the pins.  Writing
		//			1 to a bit clears the associated interrupt.
		//
		//	+10 PORTCTRL	Enable/Disable slew rate limitation for the
		//			whole GPIO port.
		//
		//	+11 -> +15	Reserved.
		//
		//	+16[0..7]
		//		PINCTRL	Control specific aspects on an individual
		//			pin.
		//
		//			bit		Role
		//			7		Value inverted (0-no, 1-yes).
		//
#define PIN_INVERTED	(1<<7)
		//
		//			3		Pull up enabled(0-no, 1-yes).
		//
#define PIN_PULL_UP	(1<<3)
		//
		//			2-0		Interrupt configuration:
		//
		//					000	Interrupt disabled
		//					001	Both edges
		//					010	Rising
		//					011	Falling
		//					100	Input disabled
		//					101	Interrupt on low
		//					110	Reserved
		//					111	Reserved
		//
#define PIN_INT_DISABLED	0
#define PIN_INT_BOTH		1
#define PIN_INT_RISING		2
#define PIN_INT_FALLING		3
#define PIN_DISABLED		4
#define PIN_INT_LOW		5
		//
		volatile byte	_dir, _dirset, _dirclr, _dirtgl,
				_out, _outset, _outclr, _outtgl,
				_in,
				_intflags,
				_portctrl,
				_reserved_11,
				_reserved_12,
				_reserved_13,
				_reserved_14,
				_reserved_15,
				_pinctrl[ 8 ];

		//
		//	As the routines below can work on multiple
		//	pins at a time, the following macros proved
		//	simple iterative structures to assist with
		//	the pin control adjustments.
		//
		//	FORALL(v)	Will step the supplied variable,
		//			v, through 0 to 7 (yes, it is a
		//			for loop).
		//
		//	FOREACH(v,s)	Will step the supplied variable,
		//			v, through 0 to 7 but will only
		//			execute the following statement if
		//			the corresponding bit in s is set.
		//
		//			Note: s will get mangled in this.
		//
#define FORALL(v)	for(byte v=0;v<8;v++)
#define FOREACH(v,s)	for(byte v=0;v<8;v++,s>>=1)if(s&1)

	public:
		//
		//	The IO routines required.
		//

		//
		//	BYTE Focused routines.
		//
		inline void input( void ) {
			_dir = 0x00;
			FORALL( i ) _pinctrl[ i ] &= ~PIN_PULL_UP;
		}
		inline void input_pullup( void ) {
			_dir = 0x00;
			FORALL( i ) _pinctrl[ i ] |= PIN_PULL_UP;
		}
		inline void output( void ) {
			_dir = 0xff;
			_out = 0x00;
		}
		inline void write( byte v ) {
			_out = v;
		}
		inline void toggle( void ) {
			_outtgl = 0xff;
		}
		inline byte read( void ) {
			return( _in );
		}
		

		//
		//	BIT Focused routines.
		//
		inline void input( byte b ) {
			_dirclr = b;
			FOREACH( i, b ) _pinctrl[ i ] &= ~PIN_PULL_UP;
		}
		inline void input_pullup( byte b ) {
			_dirclr = b;
			FOREACH( i, b ) _pinctrl[ i ] |= PIN_PULL_UP;
		}
		inline void output( byte b ) {
			_dirset = b;
			_outclr = b;
		}
		inline void high( byte b ) {
			_outset = b;
		}
		inline void low( byte b ) {
			_outclr = b;
		}
		inline void toggle( byte b ) {
			_outtgl = b;
		}
		inline byte read( byte b ) {
			return( _in & b );
		}

#else
#error "GPIO_Registers has no memory map definition for this architecture."
#endif
};

//
//	The following class provides a hardware independent
//	method for specifying the target for the pin action
//	required.
//
class Pin_IO {
	private:
		//
		//	Routine validates and performs the
		//	translation from a Platform Pin number
		//	to a GPIO instance and bit number.
		//
		bool translate( byte pin, byte *instance, byte *bit_no );
		
		//
		//	We need to keep the address of the
		//	port of which the pin is a member,
		//	and the bit map which specifies the
		//	pin being accessed.
		//
		GPIO_Registers	*_adrs;
		byte		_bit;

	public:
		//
		//	Default Constructor
		//	-------------------
		//
		//	Produces an object which is not aligned with
		//	any device, and should not be used in this mode.
		//
		Pin_IO( void );

		//
		//	Device Constructor
		//	--------------------
		//
		//	Define a GPIO pin via the GPIO instance number
		//	combined with the pin bit number (0..7).
		//
		//	dev	Platform GPIO number.
		//
		//	bit_num	Bit number within the GPIO port
		//
		//	input	True for input pin, false for output pin.
		//
		//	pull_up	True if the internal 3.3/5 volt pull up
		//		resister should be enabled.
		//
		Pin_IO( byte dev, byte bit_num, bool input, bool pull_up = false );

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
		Pin_IO( byte pin, bool input, bool pull_up = false );

		//
		//	Set/Reset the target of the Pin Object using a
		//	device instance number and pin bit number.
		//
		bool configure( byte instance, byte bit_no, bool input, bool pull_up = false );

		//
		//	Set/Reset the target of the Pin Object using a
		//	Platform pin number.
		//
		bool configure( byte pin, bool input, bool pull_up = false );

		//
		//	Determine if a pin has been configured.
		//
		bool configured( void );

		//
		//	The following functions provide the mechanism
		//	to alter, read or write the pin and its config.
		//
		void input( void );
		void input_pullup( void );
		void output( void );
		void high( void );
		void low( void );
		void set( bool high );
		void toggle( void );
		byte read( void );
};


//
//	As a "Side Product" of the Digital_Pin code comes the
//	Port_IO class enabling direct 8-bit byte IO through
//	an individual port.
//
class Port_IO {
	private:
		//
		//	We need to keep the address of the
		//	port of which is the target of the
		//	IO.
		//
		GPIO_Registers	*_adrs;
		
	public:
		//
		//	Default Constructor
		//	-------------------
		//
		//	Produces an object which is not aligned with
		//	any port, and should not be used in this mode.
		//
		Port_IO( void );

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
		Port_IO( byte dev, bool input, bool pull_up = false );

		//
		//	Set/Reset the target of the Pin Object using a
		//	Platform pin number.
		//
		bool configure( byte dev, bool input, bool pull_up = false );

		//
		//	Determine if a Port has been configured.
		//
		bool configured( void );

		//
		//	The following functions provide the mechanism
		//	to alter, read or write whole port value.
		//
		bool read_data( byte *data );
		bool write_data( byte data );
};



#endif

//
//	EOF
//

//
//	Configuration.h
//	===============
//
//	Settings pertinent to the configuration of the whole firmware
//	package.
//
//	This file should contain only defintions that configure the
//	*compilation* of the firmware directing the firmware to suit
//	specific hardware configurations (hense the name).
//


#ifndef _CONFIGURATION_H_
#define _CONFIGURATION_H_

//
//	Include hardware definitions.
//
#include "Hardware.h"

//
//	Define, roughly, how big the memory foot print of the
//	firmware is, excluding the memory heap.
//
//	This is *absolutely* depenedent on the firmware
//
#if defined( __AVR_ATmega328__ )| defined( __AVR_ATmega328P__ )| defined( __AVR_ATmega328PB__ )
#define STATIC_VARIABLES	730
#elif defined( __AVR_ATmega2560__ )
#define STATIC_VARIABLES	1000
#else
#error "No variable space estimate available for this board"
#endif


//
//	Arduino Uno R3 Pin Allocations
//	==============================
//
//	Logical	Physical	device		Role
//	-------	--------	------		----
//	D0/RX	PD0		Serial		UART Rx
//	D1/TX	PD1		Serial		UART Tx
//	D2	PD2		GPIO		DCC clock output
//	D3	PD3		Motor Shield	SHIELD_DRIVER_A_ENABLE
//	D4	PD4		
//	D5	PD5		Rotary Control	Button
//	D6	PD6		Rotary Control	Signal A
//	D7	PD7		Rotary Control	Signal A
//	D8	PB0		Motor Shield	SHIELD_DRIVER_B_BRAKE
//	D9	PB1		Motor Shield	SHIELD_DRIVER_A_BRAKE
//	D10	PB2
//	D11	PB3		Motor Shield	SHIELD_DRIVER_B_ENABLE
//	D12	PB4		Motor Shield	SHIELD_DRIVER_A_DIRECTION
//	D13	PB5		Motor Shield	SHIELD_DRIVER_B_DIRECTION
//	D14/A0	PC0		Motor Shield	SHIELD_DRIVER_A_LOAD
//	D15/A1	PC1		Motor Shield	SHIELD_DRIVER_B_LOAD
//	D16/A2	PC2
//	D17/A3	PC3
//	D18/A4	PC4				I2C SCL
//	D19/A5	PC5				I2C SDA
//
//	I2C
//	===
//
//	Address		Interface		Device		Note
//	-------		---------		------		----
//	0x27		I2C to parallel		LCD		This is the default address on PCF8475T cards
//	0x28		I2C to Parallel		Keypad		Interface will need address physically setting
//
//
//	Arduino Mega2560 Pin Allocations
//	================================
//
//	There are (currently) identical to the UNO R3 allocations
//	in order to simplify the firmware.
//

//
//	Serial Host Connectivity
//	========================
//

//
//	Define the Speed and buffer size used when accessing the
//	serial port.
//
//	Example baud rates:
//
//		9600 14400 19200 38400 57600 115200
//
//	The new USART module uses specific constant values for
//	the various supported baud rates.  These are all preceded
//	with a 'B'.
//
#ifndef SERIAL_BAUD_RATE
#define SERIAL_BAUD_RATE	B115200
#endif

//
//	The I2C bus frequency
//	=====================
//
//	Defined to be the frequency divided by 10K then used as a lookup
//	into a table in the TWI code.
//
//	See TWI_IO.[ch]
//
//	This has been set for 100 KBits/s (10), but can be slowed down
//	to determine if there is a timing issue causing communications
//	issues with the LCD.
//
#define TWI_FREQ		10

//
//	DCC Districts
//	=============
//
//	2	Using the Arduino Motor Shield
//	6	Using the bespoke backplane with Nano
//
#define DCC_DISTRICTS		2

//
//	20x4 LCD
//	========
//
//	The following definitions specify the key hardware character-
//	istics of the LCD display being used.
//
//	LCD_DISPLAY_ROWS	Number of rows the display has.
//	LCD_DISPLAY_COLS	Number of columns available per row.
//	LCD_DISPLAY_ADRS	The I2C address of the display
//
//	The display is assumed to be attached using a LCD specific
//	PCF8575 I2C to Parallel adaptor.  The following default
//	definitions apply to a generic 20x4 display.
//
#define LCD_DISPLAY_ROWS	4
#define LCD_DISPLAY_COLS	20
#define LCD_DISPLAY_ADRS	0x27

//
//	Define the macro _LCD_USE_READ_BUSY_READY_ to cause the LCD
//	code to used the "busy ready" status flag from the LCD to time
//	the data and instruction commands to the LCD.
//
//	Note:	At the moment (V0.1.7) this code is written (in the
//		form of "micro code" in the LCD_TWI_IO module), but the
//		data read back from the LCD does not seem functional
//		and the earlier, and simpler, timed delay approach
//		works reliably.
//
//#define _LCD_USE_READ_BUSY_READY_

//
//	Text Buffer Space
//	=================
//
//	Define the size of a generic small textual buffer for
//	use on the stack.
//
#define TEXT_BUFFER			SELECT_SML(8,12,16)



#endif

//
//	EOF
//

//
//	LCD_I2C_Lite - Arduino library to control an LCD via an I2C adapter based on PCF8574
//
//	Copyright(C) 2020 Blackhack <davidaristi.0504@gmail.com>
//
//	This program is free software : you can redistribute it and /or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation, either version 3 of the License, or
//	(at your option) any later version.
//
//	This program is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
//	GNU General Public License for more details.
//
//	You should have received a copy of the GNU General Public License
//	along with this program.If not, see < https://www.gnu.org/licenses/>.
//
//
//	Library originally based on the LCD_I2C Library by the above Blackhack.
//
//	Modified December 2020 by Jeff Penfold <jeff.penfold@googlemail.com>
//

//
//	This library has been "lite"ned for simpler interface, smaller memory
//	and program footprint with support for "standard" LCD sizes to 20x4.
//


#ifndef _LCD_I2C_LITE_H_
#define _LCD_I2C_LITE_H_

#include "Arduino.h"

//
//	Default I2C address for a Liquid Crystal Display
//
#define LCD_I2C_LITE_ADDRESS 0x27

//
//	This struct helps us constructing the I2C output based on data and control outputs.
//	Because the LCD is set to 4-bit mode, 4 bits of the I2C output are for the control outputs
//	while the other 4 bits are for the 8 bits of data which are send in parts using the enable output.
//

//
//	Define some constants surrounding the three "state" variables:
//
//		outputState
//			rs
//			rw
//			enable
//			LED
//		displayState
//		entryState
//
#define LCD_I2C_LITE_OUTPUT_STATE	0b00000000

#define LCD_I2C_LITE_REGISTER_SELECT	0
#define LCD_I2C_LITE_READ_WRITE		1
#define LCD_I2C_LITE_ENABLE		2
#define LCD_I2C_LITE_BACKLIGHT		3

#define LCD_I2C_LITE_ENTRY_STATE	0b00000100

#define LCD_I2C_LITE_AUTO_SCROLL	0
#define LCD_I2C_LITE_LEFT_RIGHT		1

#define LCD_I2C_LITE_DISPLAY_STATE	0b00001000

#define LCD_I2C_LITE_BLINK_ON		0
#define LCD_I2C_LITE_CURSOR_ON		1
#define LCD_I2C_LITE_DISPLAY_ON		2

//
//	Direct LCD commands
//
#define LCD_I2C_LITE_CLEAR_SCREEN	0b00000001
#define LCD_I2C_LITE_HOME_SCREEN	0b00000010
#define LCD_I2C_LITE_DISPLAY_LEFT	0b00011000
#define LCD_I2C_LITE_DISPLAY_RIGHT	0b00011100
#define LCD_I2C_LITE_SET_POSITION	0b10000000

//
//	Macros to extract high and low nybble of a value
//	suitable for sending to display.
//
#define LCD_I2C_LITE_LOW_NYBBLE(v)	(((v)&0x0f)<<4)
#define LCD_I2C_LITE_HIGH_NYBBLE(v)	((v)&0xf0)

//
//	The controlling class for the module.
//
class LCD_I2C_Lite {
private:
	uint8_t	_address,
		_cols,
		_rows;

	uint8_t	_outputState = 0x00;
	uint8_t	_displayState = 0x00;
	uint8_t	_entryState = 0x00;

	void	InitializeLCD( void );
	void	I2C_Write( uint8_t output, int usec );
	void	LCD_Init( uint8_t output, int usec );
	void	LCD_Write( uint8_t output, int usec );

public:
	LCD_I2C_Lite( uint8_t address, uint8_t cols, uint8_t rows );

	void begin( void );
	void backlight( bool on );
	void clear( void );
	void home( void );
	void leftToRight( bool l2r );
	void autoscroll( bool on );
	void display( bool on );
	void cursor( bool on );
	void blink( bool on );
	void scrollDisplayLeft( void );
	void scrollDisplayRight( void );
	void position( uint8_t col, uint8_t row );
	void write( uint8_t val );
	void write( const char *str );
	void write( const char *str, uint8_t len );
	void fill( char val, uint8_t len );
};

#endif

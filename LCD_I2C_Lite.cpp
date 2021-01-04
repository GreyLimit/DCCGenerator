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

#include "LCD_I2C_Lite.h"
#include "Wire_Lite.h"

void LCD_I2C_Lite::InitializeLCD()
{
	// See HD44780U datasheet "Initializing by Instruction" Figure 24 (4-Bit Interface)

	LCD_Init( 0b00110000, 4200 );
	LCD_Init( 0b00110000, 150 );
	LCD_Init( 0b00110000, 37 );
	LCD_Init( 0b00100000, 37 );	// Function Set - 4 bits mode
	LCD_Write( 0b00101000, 37 );	// Function Set - 4 bits(Still), 2 lines, 5x8 font

	display( true );
	clear();
	leftToRight( true );
}

void LCD_I2C_Lite::I2C_Write( uint8_t output, int usec ) {
	Wire.beginTransmission( _address );
	Wire.write( output );
	Wire.endTransmission();
	delayMicroseconds( usec );
}

void LCD_I2C_Lite::LCD_Init( uint8_t output, int usec ) {
	I2C_Write( LCD_I2C_LITE_OUTPUT_STATE | _outputState | bit( LCD_I2C_LITE_ENABLE ) | LCD_I2C_LITE_HIGH_NYBBLE( output ), 1 );
	I2C_Write( LCD_I2C_LITE_OUTPUT_STATE | _outputState | LCD_I2C_LITE_HIGH_NYBBLE( output ), usec );
}

void LCD_I2C_Lite::LCD_Write( uint8_t output, int usec ) {
	I2C_Write( LCD_I2C_LITE_OUTPUT_STATE | _outputState | bit( LCD_I2C_LITE_ENABLE )| LCD_I2C_LITE_HIGH_NYBBLE( output ), 1 );
	I2C_Write( LCD_I2C_LITE_OUTPUT_STATE | _outputState | LCD_I2C_LITE_HIGH_NYBBLE( output ), 37 );

	I2C_Write( LCD_I2C_LITE_OUTPUT_STATE | _outputState | bit( LCD_I2C_LITE_ENABLE )| LCD_I2C_LITE_LOW_NYBBLE( output ), 1 );
	I2C_Write( LCD_I2C_LITE_OUTPUT_STATE | _outputState | LCD_I2C_LITE_LOW_NYBBLE( output ), usec );
}


LCD_I2C_Lite::LCD_I2C_Lite( uint8_t address, uint8_t cols, uint8_t rows ) {
	_address = address;
	_cols = cols;
	_rows = rows;
}

void LCD_I2C_Lite::begin()
{
	I2C_Write( 0b00000000, 0 );			// Clear i2c adapter
	delay( 50 );					// Wait more than 40ms after powerOn.
	InitializeLCD();
}

void LCD_I2C_Lite::backlight( bool on )
{
	bitWrite( _outputState, LCD_I2C_LITE_BACKLIGHT, on );
	I2C_Write( LCD_I2C_LITE_OUTPUT_STATE | _outputState, 37 );
}

void LCD_I2C_Lite::clear( void )
{
	LCD_Write( LCD_I2C_LITE_CLEAR_SCREEN, 1600 );
}

void LCD_I2C_Lite::home( void )
{
	LCD_Write( LCD_I2C_LITE_HOME_SCREEN, 1600 );
}

void LCD_I2C_Lite::leftToRight( bool l2r )
{
	bitWrite( _entryState, LCD_I2C_LITE_LEFT_RIGHT, l2r );
	LCD_Write( LCD_I2C_LITE_ENTRY_STATE | _entryState, 37 );
}


void LCD_I2C_Lite::autoscroll( bool on )
{
	bitWrite( _entryState, LCD_I2C_LITE_AUTO_SCROLL, on );
	LCD_Write( LCD_I2C_LITE_ENTRY_STATE | _entryState, 37 );
}

void LCD_I2C_Lite::display( bool on )
{
	bitWrite( _displayState, LCD_I2C_LITE_DISPLAY_ON, on );
	LCD_Write( LCD_I2C_LITE_DISPLAY_STATE | _displayState, 37 );
}

void LCD_I2C_Lite::cursor( bool on )
{
	bitWrite( _displayState, LCD_I2C_LITE_CURSOR_ON, on );
	LCD_Write( LCD_I2C_LITE_DISPLAY_STATE | _displayState, 37 );
}

void LCD_I2C_Lite::blink( bool on )
{
	bitWrite( _displayState, LCD_I2C_LITE_BLINK_ON, on );
	LCD_Write( LCD_I2C_LITE_DISPLAY_STATE | _displayState, 37 );
}

void LCD_I2C_Lite::scrollDisplayLeft( void )
{
	LCD_Write( LCD_I2C_LITE_DISPLAY_LEFT, 37 );
}

void LCD_I2C_Lite::scrollDisplayRight( void )
{
	LCD_Write( LCD_I2C_LITE_DISPLAY_RIGHT, 37 );
}

void LCD_I2C_Lite::position( uint8_t col, uint8_t row ) {
	uint8_t newAddress;

	switch( row ) {
		case 0: {
			newAddress = 0;
			break;
		}
		case 1: {
			newAddress = 0x40;
			break;
		}
		case 2: {
			newAddress = 0 + _cols;
			break;
		}
		case 3: {
			newAddress = 0x40 + _cols;
			break;
		}
		default: {
			newAddress = 0; // default to top line.
			break;
		}
	}
	newAddress += col;
	LCD_Write( LCD_I2C_LITE_SET_POSITION | newAddress, 37 );
}

void LCD_I2C_Lite::write( uint8_t val ) {
	bitSet( _outputState, LCD_I2C_LITE_REGISTER_SELECT );
	LCD_Write( val, 41 );
	if( do_poll ) (*poll_func)();
	bitClear( _outputState, LCD_I2C_LITE_REGISTER_SELECT );
}

void LCD_I2C_Lite::write( const char *str ) {
	char	c;

	bitSet( _outputState, LCD_I2C_LITE_REGISTER_SELECT );
	while(( c = *str++ )) {
		LCD_Write( (uint8_t)c, 41 );
		if( do_poll ) (*poll_func)();
	}
	bitClear( _outputState, LCD_I2C_LITE_REGISTER_SELECT );
}

void LCD_I2C_Lite::write( const char *str, uint8_t len ) {
	bitSet( _outputState, LCD_I2C_LITE_REGISTER_SELECT );
	while( len-- ) {
		LCD_Write( (uint8_t)( *str++ ), 41 );
		if( do_poll ) (*poll_func)();
	}
	bitClear( _outputState, LCD_I2C_LITE_REGISTER_SELECT );
}

void LCD_I2C_Lite::fill( char val, uint8_t len ) {
	bitSet( _outputState, LCD_I2C_LITE_REGISTER_SELECT );
	while( len-- ) {
		LCD_Write( val, 41 );
		if( do_poll ) (*poll_func)();
	}
	bitClear( _outputState, LCD_I2C_LITE_REGISTER_SELECT );
}

void LCD_I2C_Lite::enable_poll( void (*func)( void )) {
	do_poll = true;
	poll_func = func;
}
void LCD_I2C_Lite::disable_poll( void ) {
	do_poll = false;
}


//
//	EOF
//

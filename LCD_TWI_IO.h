//
//	LCD_TWI_IO - Arduino library to control an LCD via the TWI_IO Library
//
//	Copyright(C) 2021 Jeff Penfold <jeff.penfold@googlemail.com>
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
//	along with this program.  If not, see < https://www.gnu.org/licenses/>.
//


#ifndef _LCD_TWI_IO_H_
#define _LCD_TWI_IO_H_

#include "Arduino.h"

//
//	Default I2C address for the PCF8574 Remote 8-Bit I/O Expander
//	through which the LCD is attached.
//
//	This is based on the initial "fixed" binary address
//	of:
//		0b0100000 (0x20) or 32 in decimal.
//
//	However the board supports the hard specification of the
//	bottom 3 bits of the 7-bit address using solder-able pads.
//	Left un-soldered (i.e., not shorted) each bit (A0, A1 and A2)
//	pulls that bit high in the address and become a "1".
//
//	Therefore, the default, unmodified address is 32 plus 7,
//	as all three bits will show as ones, giving 39 (0x27).
//
#define LCD_TWI_IO_ADDRESS		0x27

//
//	The controlling class for the module.
//
class LCD_TWI_IO {
private:
	//
	//	Capture some hard details about the LCD this object is
	//	addressing.
	//
	byte	_address,
		_cols,
		_rows;

	//
	//	Define a structure to be used to deliver a single data
	//	byte or instruction to the LCD.
	//
	struct pending {
		byte		value;
		const byte	*program;
	};
	static const byte max_pending = 8;
	//
	//	This is the queue of pending bytes heading out of the
	//	object towards the LCD.
	//
	struct pending	_queue[ max_pending ];
	byte		_queue_len,
			_queue_in,
			_queue_out;
	//
	//	Routine used to add another action to the queue
	//
	bool	queueTransfer( const byte *program, byte value );
	void	queueTransferWait( const byte *program, byte value );
	
	//
	//	Define the variables holding the "state" details of the
	//	LCD.
	//
	byte	_backLight;	// Backlight? (obviously)
	byte	_displayState;	// Display on? Cursor on? Blinking?
	byte	_entryState;	// Text L->R? Auto Scroll?
	
	//
	//	The "machine" which will be used to transmit the data
	//	to the LCD is formed from the following instructions:
	//
	static const byte mc_idle		= 0;	// Machine at idle
	static const byte mc_reset		= 1;	// Send reset byte (value=0x00)
	static const byte mc_inst_high_enable	= 2;	// send high nybble with E=1 as inst
	static const byte mc_inst_high_disable	= 3;	// send high nybble with E=0 as inst
	static const byte mc_inst_low_enable	= 4;	// send low nybble with E=1 as inst
	static const byte mc_inst_low_disable	= 5;	// send low nybble with E=0 as inst
	static const byte mc_data_high_enable	= 6;	// send high nybble with E=1 as data
	static const byte mc_data_high_disable	= 7;	// send high nybble with E=0 as data
	static const byte mc_data_low_enable	= 8;	// send low nybble with E=1 as data
	static const byte mc_data_low_disable	= 9;	// send low nybble with E=0 as data
	static const byte mc_transmit_buffer	= 10;	// send content of the buffer
	static const byte mc_wait_on_done	= 11;	// Wait for TWI confirmation
	static const byte mc_set_delay_40000us	= 12;	// Set delay countdown to 40000us
	static const byte mc_set_delay_4200us	= 13;	// 4200us
	static const byte mc_set_delay_1600us	= 14;	// 1600us
	static const byte mc_set_delay_150us	= 15;	// 150us
	static const byte mc_set_delay_41us	= 16;	// 41us
	static const byte mc_set_delay_37us	= 17;	// 37us
	static const byte mc_set_delay_10us	= 18;	// 10us
	static const byte mc_delay_wait		= 19;	// Wait until the delay period has expired

	//
	//	The machine "programs" which tell the system how to
	//	handle each of the mechanisms for talking to the LCD.
	//
	static const byte mc_idle_program[] PROGMEM;
	static const byte mc_reset_program[] PROGMEM;
	static const byte mc_init_long_delay[] PROGMEM;
	static const byte mc_init_medium_delay[] PROGMEM;
	static const byte mc_init_short_delay[] PROGMEM;
	static const byte mc_inst_long_delay[] PROGMEM;
	static const byte mc_inst_short_delay[] PROGMEM;
	static const byte mc_data_short_delay[] PROGMEM;

	//
	//	The variables which the current "program" are operating against.
	//
	const byte	*_fsm_instruction;
	byte		_fsm_data_byte,
			_fsm_buffer;
	word		_fsm_delay;
	unsigned long	_fsm_time_starts;
	bool		_fsm_twi_returns,
			_fsm_twi_success;

	//
	//	The following variables contain and manage the current "frame buffer"
	//	for the LCD.  These are only activated if a piece of memory is provided
	//	to be the frame buffer (the object does not allocate or contain any
	//	memory space for this purpose).
	//
	//	_frame_buffer		Where the frame buffer is in memory (or NULL)
	//
	//	_frame_size		The size of the frame buffer (or 0 if not defined,
	//				use as indication of a buffer being available)
	//
	//	_frame_last		Index of the last position updated.
	//
	//	_frame_next		Index of the next position to check.
	//
	//	_frame_cursor		Insert point for next output text.
	//
	byte		*_frame_buffer,
			_frame_size,
			_frame_last,
			_frame_next,
			_frame_cursor;
			

public:
	//
	//	Constructor for the LCD object.
	//
	LCD_TWI_IO( byte address, byte cols, byte rows );
	
	//
	//	Routine to "kick off" the LCD code
	//	and a service routine which needs to be
	//	called regularly in the main loop code.
	//
	void begin( void );		// Use inside setup();

	//
	//	IMPORTANT
	//	=========
	//
	//	This routine must be called at least once
	//	each time through the "loop()" routine
	//	to ensure that the processes supporting the
	//	LCD are driven forwards generating the
	//	intended output.
	//
	void service( void );		// Use inside loop();

	//
	//	Return space available in the queue
	//
	byte	queueCapacity( void );

	//
	//	This is an "interrupt" style routine called
	//	when TWI transactions have been completed
	//
	//	Never call directly.
	//
	void done( bool ok );
	
	//
	//	Routines to act directly upon the LCD.
	//
	//	These return true if the command/action was
	//	successfully queued, false otherwise.
	//
	bool backlight( bool on );
	bool clear( void );
	bool home( void );
	bool leftToRight( bool l2r );
	bool autoscroll( bool on );
	bool display( bool on );
	bool cursor( bool on );
	bool blink( bool on );
	bool position( byte col, byte row );
	bool index( byte posn );
	bool write( byte val );

	//
	//	Routines which act upon the frame buffer.
	//
	//	The frame buffer hijacks the top bit of each
	//	byte to indicate update status.  As a result
	//	only 7-ASCII can be displayed through this
	//	mechanism.
	//
	bool setBuffer( byte *buffer, byte size );
	void setPosn( byte col, byte row );
	void writeChar( char val );
	void writeStr( const char *str );
	void writeBuf( const char *buf, byte len );
	void fill( char val, byte len );

	//
	//	Pause activities and wait until the
	//	frame buffer has been updated.
	//
	//	Either run until all jobs apear to have
	//	been completed (no arguments), or
	//	run for a fixed period of time specified
	//	as milliseconds in the argument.
	//
	void synchronise( void );
	void synchronise( word ms );
};

#endif

//
//	EOF
//

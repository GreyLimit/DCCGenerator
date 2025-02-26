//
//	LCD - Arduino library to control an LCD via the TWI_IO Library
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


#ifndef _LCD_H_
#define _LCD_H_

#include "Environment.h"
#include "Parameters.h"
#include "Configuration.h"
#include "Task_Entry.h"
#include "Clock.h"
#include "Signal.h"
#include "TWI.h"

//
//	Set up default value for the target address and size of the LCD.
//
#ifndef LCD_DISPLAY_ROWS
#define LCD_DISPLAY_ROWS	4
#endif
#ifndef LCD_DISPLAY_COLS
#define LCD_DISPLAY_COLS	20
#endif
#ifndef LCD_DISPLAY_ADRS
#define LCD_DISPLAY_ADRS	0x27
#endif
#ifndef LCD_PROCESSING_DELAY
#define LCD_PROCESSING_DELAY	5
#endif

//
//	The controlling class for the module.
//
class LCD : public Task_Entry {
private:
	//
	//	The I2C 8-Bit IO Expander directly connects to 8 of the available
	//	LCD IO pins.  These are:
	//
	//	Bit	Name	Meaning
	//
	//	0	RS	Register Select
	//			0	Instruction Register (IR) on WRITE
	//			0	Address Counter (AC) on READ
	//			1	Data Register (DR) for READ or WRITE
	//
	//	1	RW	0	Write Data
	//			1	Read Data
	//
	//	2	E	Enable data read/write
	//
	//			It is the *transition* from 1 -> 0 in this bit which
	//			triggers activity on the LCD.  There for all activity
	//			with the LCD involves writing a "data byte" with this
	//			bit as 1 followed by writing the *same* byte, but with
	//			this bit as 0.
	//
	//	3	LED	Backlight off
	//			0	Off
	//			1	On
	//
	//	4	D0	Data bit 0
	//	5	D1	Data bit 1
	//	6	D2	Data bit 2
	//	7	D3	Data bit 3
	//		 
	//	Because the LCD is set to 4-bit mode, 4 bits of the I2C output are for the control outputs
	//	while the other 4 bits are for the 8 bits of data which are send in parts using the enable
	//	output.
	//

	//
	//	"Output State"
	//	==============
	//
	//	This definition deals with the "top level" access to the
	//	LCD.  All other access has to go through the "output state"
	//	definition.
	//
	//	These values enable the software to "operate" the LCD at
	//	its *hardware* interface level.  ALl of the definitions
	//	after this relate to "operating" the firmware within the
	//	LCD.
	//
	//	The following sequences indicate "how" to access the LCDs
	//	internal registers and so (using the instructions outlined
	//	below) how to make the LCD do what you want!
	//
	//	The following show the steps required to execute specific
	//	actions upon the LCD, and represet the bytes of data which
	//	have to be written to the I2C I/O expander which is attached
	//	to the LCD.
	//
	//	In the following:
	//		"?" means according to requirements
	//		"hhhh" means the high nybble of the data
	//		"llll" means the low nybble of the data
	//
	//	Some form of delay will be required between each data byte
	//	transmitted.
	//
	//		Data	LED	Enable	RW	RS
	//	Bit	7654	3	2	1	0
	//		----	---	------	--	--
	//
	//	Write Byte to Instruction Register
	//
	//		hhhh	?	1	0	0
	//		hhhh	?	0	0	0
	//		llll	?	1	0	0
	//		llll	?	0	0	0
	//
	//
	//	Place a character onto the LCD
	//
	//		hhhh	?	1	0	1
	//		hhhh	?	0	0	1
	//		llll	?	1	0	1
	//		llll	?	0	0	1
	//
	//	
	//
	static const byte	output_state	= 0b00000000;
	//
	//	Name the meaning of each bit position
	//
	static const byte	register_select	= 0;
	static const byte	read_write	= 1;
	static const byte	enable		= 2;
	static const byte	LED_backlight	= 3;

	//
	//	All of the following definitions relate to
	//	values which are applied to the LCD Instruction
	//	Register, and act upon the display rather than
	//	place data into the display.
	//

	//
	//	"Entry State"
	//	=============
	//
	//	The "Entry State" instructions dictate how new
	//	text is inserted into the LCD display. Is text
	//	added left to right or right to left.  Does the
	//	LCD automatically scroll when the bottom is reached?
	//
	//	entry_state		000001ab
	//
	//	a	Left/right additon of new text
	//	b	Auto scrolling enabled
	//
	static const byte	entry_state	= 0b00000100;
	static const byte	auto_scroll	= 0;
	static const byte	left_right	= 1;

	//
	//	"Display State"
	//	===============
	//
	//	The "Display State" instructions control how the
	//	LCD display "looks".  Is it on? Is there a cursor?
	//	Does the cursor flash?
	//
	//	display_state	00001abc
	//
	//	a	Display on?
	//	b	Cursor on?
	//	c	Blinking cursor?
	//
	static const byte	display_state	= 0b00001000;
	static const byte	blink_on	= 0;
	static const byte	cursor_on	= 1;
	static const byte	display_on	= 2;

	//
	//	"Direct Instructions"
	//	=====================
	//
	//	Direct LCD instructions
	//
	static const byte	clear_screen	= 0b00000001;
	static const byte	home_screen	= 0b00000010;
	static const byte	scroll_left	= 0b00011000;
	static const byte	scroll_right	= 0b00011100;
	static const byte	set_position	= 0b10000000;

	//
	//	Macros to extract high and low nybble of a value
	//	suitable for sending to display.
	//
	inline byte low_nybble( byte v ) { return(( v & 0x0f ) << 4 ); }
	inline byte high_nybble( byte v ) { return( v & 0xf0 ); }


	//
	//	The "machine" which will be used to transmit the data
	//	to the LCD is formed from the following instructions:
	//
	enum mc_state : byte {
		mc_idle = 0,		// Machine at idle
		mc_reset,		// Send reset byte (value=0x00)
		mc_inst_high_enable,	// send high nybble with E=1 as inst
		mc_inst_high_disable,	// send high nybble with E=0 as inst
		mc_inst_low_enable,	// send low nybble with E=1 as inst
		mc_inst_low_disable,	// send low nybble with E=0 as inst
		mc_data_high_enable,	// send high nybble with E=1 as data
		mc_data_high_disable,	// send high nybble with E=0 as data
		mc_data_low_enable,	// send low nybble with E=1 as data
		mc_data_low_disable,	// send low nybble with E=0 as data
		mc_status_enable,	// send status request with E=1 as inst
		mc_status_disable,	// send status request with E=0 as inst
		mc_transmit_buffer,	// send content of the buffer
		mc_wait_on_done,	// Wait for TWI confirmation
		mc_read_buffer,		// read the interface into the buffer
		mc_store_high_data,	// place buffer into data high bits
		mc_store_low_data,	// place buffer into data high bits
		mc_begin_wait,		// Note the top of the wait loop
		mc_wait_loop,		// If not ready loop again
		mc_finish_up,		// Complete any transaction tidy up

		//
		//	These should be redundant, eventually.
		//
		mc_delay_40000us,	// Set delay countdown to 40000us
		mc_delay_4200us,	// 4200us
		mc_delay_1600us,	// 1600us
		mc_delay_150us,		// 150us
		mc_delay_41us,		// 41us
		mc_delay_37us,		// 37us
		mc_delay_10us		// 10us
	};

	//
	//	Define a "short period of time" which the LCD driver will
	//	pause for if it is unable to obtain resources required to
	//	complete its processing.
	//
	static const word	processing_delay = MSECS( LCD_PROCESSING_DELAY );

	//
	//	Capture some hard details about the LCD this object is
	//	addressing.
	//
	//	_adrs		I2C/TWI target address.
	//
	//	_rows / _cols	Display size.
	//
	byte			_adrs;
	byte			_rows;
	byte			_cols;

	//
	//	Define a structure to be used to deliver a single data
	//	byte or instruction to the LCD.  The queue is provided
	//	enable a calling system to push out a sequence of
	//	actions asynchronously from the LCD driver function.
	//
	struct pending_lcd {
		byte		value;
		const mc_state	*program;
		Signal		*flag;
		pending_lcd	*next;
	};

	//
	//	This is the queue of pending bytes heading out of the
	//	object towards the LCD.
	//
	pending_lcd	*_active,
			**_tail,
			*_free;

	//
	//	bool queue_transfer( const mc_state *program, byte value, Signal *flag )
	//	-----------------------------------------------------
	//
	//	Routine used to add another action to the queue, return true
	//	if the action has been queued, false otherwise.
	//
	bool queue_transfer( const mc_state *program, byte value, Signal *flag );

	//
	//	bool queue_transfer_wait( const mc_state *program, byte value )
	//	---------------------------------------------------------
	//
	//	Routine used to add another action to the queue but waits until the
	//	action has been successfully added to the queue.  This will block
	//	the execution of the firmware at this point.
	//
	void queue_transfer_wait( const mc_state *program, byte value );

	
	//
	//	Define the variables holding the "state" details of the
	//	LCD.
	//
	byte	_back_light;	// Backlight? (obviously)
	byte	_display_state;	// Display on? Cursor on? Blinking?
	byte	_entry_state;	// Text L->R? Auto Scroll?
	
	//
	//	The machine "programs" which tell the system how to
	//	handle each of the mechanisms for talking to the LCD.
	//
	static const mc_state mc_idle_program[] PROGMEM;
	static const mc_state mc_reset_program[] PROGMEM;
	static const mc_state mc_init_long_delay[] PROGMEM;
	static const mc_state mc_init_medium_delay[] PROGMEM;
	static const mc_state mc_init_short_delay[] PROGMEM;
	static const mc_state mc_send_inst[] PROGMEM;
	static const mc_state mc_send_data[] PROGMEM;

	//
	//	The variables which the current "program" are operating against.
	//
	const mc_state	*_fsm_instruction,
			*_fsm_loop;
	byte		_fsm_data_byte,
			_fsm_buffer;

	//
	//	Task control signal.
	//
	Signal		_flag;
	TWI::error_code	_error;

public:
	//
	//	Constructor!
	//	============
	//
	LCD( void );

	//
	//	Initialise the LCD and get it running.
	//
	void initialise( byte adrs, byte rows, byte cols );
	
	//
	//	This routine called each time round loop to support going
	//	LCD processes.
	//
	virtual void process( byte handle );

	//
	//	Functional Routines
	//	===================
	//
	//	These return true if the command/action was
	//	successfully queued, false otherwise.  The completion
	//	of the command is signalled through the flag passed in.
	//
	bool backlight( bool on, Signal *flag );
	bool clear( Signal *flag );
	bool home( Signal *flag );
	bool leftToRight( bool l2r, Signal *flag );
	bool autoscroll( bool on, Signal *flag );
	bool display( bool on, Signal *flag );
	bool cursor( bool on, Signal *flag );
	bool blink( bool on, Signal *flag );
	bool position( byte row, byte col, Signal *flag );
	bool index( byte posn, Signal *flag );
	bool write( byte val, Signal *flag );
	//
	//	These return true if the command/action was
	//	successfully completed, false otherwise.
	//
	void backlight( bool on );
	void clear( void );
	void home( void );
	void leftToRight( bool l2r );
	void autoscroll( bool on );
	void display( bool on );
	void cursor( bool on );
	void blink( bool on );
	void position( byte row, byte col );
	void index( byte posn );
	void write( byte val );
};


#endif

//
//	EOF
//

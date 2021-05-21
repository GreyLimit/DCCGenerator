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

#include <avr/pgmspace.h>

#include "TWI_IO.h"
#include "LCD_TWI_IO.h"

//
//	Generic constants we use.
//
#define SPACE		' '
#define EOS		'\0'

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
#define LCD_TWI_IO_OUTPUT_STATE		0b00000000
//
//	Name the meaning of each bit position
//
#define LCD_TWI_IO_REGISTER_SELECT	0
#define LCD_TWI_IO_READ_WRITE		1
#define LCD_TWI_IO_ENABLE		2
#define LCD_TWI_IO_BACKLIGHT		3

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
//	LCD_TWI_IO_ENTRY_STATE		000001ab
//
//	a	Left/right additon of new text
//	b	Auto scrolling enabled
//
#define LCD_TWI_IO_ENTRY_STATE		0b00000100

#define LCD_TWI_IO_AUTO_SCROLL		0
#define LCD_TWI_IO_LEFT_RIGHT		1

//
//	"Display State"
//	===============
//
//	The "Display State" instuctions control how the
//	LCD display "looks".  Is it on? Is there a cursor?
//	Does the cursor flash?
//
//	LCD_TWI_IO_DISPLAY_STATE	00001abc
//
//	a	Display on?
//	b	Cursor on?
//	c	Blinking cursor?
//
#define LCD_TWI_IO_DISPLAY_STATE	0b00001000

#define LCD_TWI_IO_BLINK_ON		0
#define LCD_TWI_IO_CURSOR_ON		1
#define LCD_TWI_IO_DISPLAY_ON		2

//
//	"Direct Instructions"
//	=====================
//
//	Direct LCD instructions
//
#define LCD_TWI_IO_CLEAR_SCREEN		0b00000001
#define LCD_TWI_IO_HOME_SCREEN		0b00000010
#define LCD_TWI_IO_SCROLL_LEFT		0b00011000
#define LCD_TWI_IO_SCROLL_RIGHT		0b00011100
#define LCD_TWI_IO_SET_POSITION		0b10000000

//
//	Macros to extract high and low nybble of a value
//	suitable for sending to display.
//
#define LCD_TWI_IO_LOW_NYBBLE(v)	(((v)&0x0f)<<4)
#define LCD_TWI_IO_HIGH_NYBBLE(v)	((v)&0xf0)

//
//	The LCD "programs" for sending data to the
//	display via the TWI IO module.
//
static const byte LCD_TWI_IO::mc_idle_program[] PROGMEM = {
	mc_idle
};
static const byte LCD_TWI_IO::mc_reset_program[] PROGMEM = {
	mc_reset, mc_transmit_buffer, mc_wait_on_done, mc_set_delay_40000us, mc_delay_wait,
	mc_idle
};
static const byte LCD_TWI_IO::mc_init_long_delay[] PROGMEM = {
	mc_inst_high_enable, mc_transmit_buffer, mc_wait_on_done, mc_set_delay_10us, mc_delay_wait,
	mc_inst_high_disable, mc_transmit_buffer, mc_wait_on_done, mc_set_delay_4200us, mc_delay_wait,
	mc_idle
};
static const byte LCD_TWI_IO::mc_init_medium_delay[] PROGMEM = {
	mc_inst_high_enable, mc_transmit_buffer, mc_wait_on_done, mc_set_delay_10us, mc_delay_wait,
	mc_inst_high_disable, mc_transmit_buffer, mc_wait_on_done, mc_set_delay_150us, mc_delay_wait,
	mc_idle
};
static const byte LCD_TWI_IO::mc_init_short_delay[] PROGMEM = {
	mc_inst_high_enable, mc_transmit_buffer, mc_wait_on_done, mc_set_delay_10us, mc_delay_wait,
	mc_inst_high_disable, mc_transmit_buffer, mc_wait_on_done, mc_set_delay_37us, mc_delay_wait,
	mc_idle
};
static const byte LCD_TWI_IO::mc_inst_long_delay[] PROGMEM = {
	mc_inst_high_enable, mc_transmit_buffer, mc_wait_on_done, mc_set_delay_10us, mc_delay_wait,
	mc_inst_high_disable, mc_transmit_buffer, mc_wait_on_done, mc_set_delay_37us, mc_delay_wait,
	mc_inst_low_enable, mc_transmit_buffer, mc_wait_on_done, mc_set_delay_10us, mc_delay_wait,
	mc_inst_low_disable, mc_transmit_buffer, mc_wait_on_done, mc_set_delay_1600us, mc_delay_wait,
	mc_idle
};
static const byte LCD_TWI_IO::mc_inst_short_delay[] PROGMEM = {
	mc_inst_high_enable, mc_transmit_buffer, mc_wait_on_done, mc_set_delay_10us, mc_delay_wait,
	mc_inst_high_disable, mc_transmit_buffer, mc_wait_on_done, mc_set_delay_37us, mc_delay_wait,
	mc_inst_low_enable, mc_transmit_buffer, mc_wait_on_done, mc_set_delay_10us, mc_delay_wait,
	mc_inst_low_disable, mc_transmit_buffer, mc_wait_on_done, mc_set_delay_37us, mc_delay_wait,
	mc_idle
};
static const byte LCD_TWI_IO::mc_data_short_delay[] PROGMEM = {
	mc_data_high_enable, mc_transmit_buffer, mc_wait_on_done, mc_set_delay_10us, mc_delay_wait,
	mc_data_high_disable, mc_transmit_buffer, mc_wait_on_done, mc_set_delay_37us, mc_delay_wait,
	mc_data_low_enable, mc_transmit_buffer, mc_wait_on_done, mc_set_delay_10us, mc_delay_wait,
	mc_data_low_disable, mc_transmit_buffer, mc_wait_on_done, mc_set_delay_41us, mc_delay_wait,
	mc_idle
};

//
//	bool queueTransfer( const byte *program, byte value )
//	-----------------------------------------------------
//
//	Routine used to add another action to the queue, return true
//	if the action has been queued, false otherwise.
//
bool LCD_TWI_IO::queueTransfer( const byte *program, byte value ) {
	if( _queue_len >= max_pending ) return( false );
	_queue[ _queue_in ].value = value;
	_queue[ _queue_in ].program = program;
	if( ++_queue_in >= max_pending ) _queue_in = 0;
	_queue_len++;
	return( true );
}

//
//	bool queueTransferWait( const byte *program, byte value )
//	---------------------------------------------------------
//
//	Routine used to add another action to the queue but waits until the
//	action has been successfully added to the queue.  This will block
//	the execution of the firmware at this point.
//
void LCD_TWI_IO::queueTransferWait( const byte *program, byte value ) {
	while( !queueTransfer( program, value )) {
		twi_eventProcessing();
		service();
	}
}


//
//	byte queueCapacity( void )
//	--------------------------
//
//	Return space available in the queue
//
byte LCD_TWI_IO::queueCapacity( void ) {
	return( max_pending - _queue_len );
}

//
//	Constructor!
//	============
//
LCD_TWI_IO::LCD_TWI_IO( byte address, byte cols, byte rows ) {
	//
	//	Remember the details of the LCD we are managing.
	//
	_address = address;
	_cols = cols;
	_rows = rows;
	//
	//	Empty the pending queue in preparation for the
	//	initial initialisation code to execute.
	//
	_queue_len = 0;
	_queue_in = 0;
	_queue_out = 0;
	//
	//	Set the back light off and the state variables
	//	as empty.
	//
	_backLight = 0;
	_displayState = 0;
	_entryState = 0;
	//
	//	Initialise the program machine which controls
	//	data transmission to the LCD
	//
	_fsm_instruction = mc_idle_program;
	_fsm_data_byte = 0;
	_fsm_buffer = 0;
	_fsm_delay = 0;
	_fsm_time_starts = 0;
	_fsm_twi_returns = false;
	_fsm_twi_success = false;
	//
	//	Set up the frame buffer as empty.
	//
	_frame_buffer = NULL;
	_frame_size = 0;
	_frame_last = 0;
	_frame_next = 0;
	_frame_cursor = 0;
}

void LCD_TWI_IO::begin( void ) {
	//
	//	Clear i2c adapter, then wait more than 40ms after powerOn.
	//
	queueTransferWait( mc_reset_program, 	0b00000000 );
	//
	//	See HD44780U datasheet "Initializing by Instruction" Figure 24 (4-Bit Interface)
	//
	//	Now set up the LCD
	//
	//	Function set flags
	//
	//	DL
	//	--
	//	0	4-bit data IO
	//	1	8-bit data IO
	//
	//	4-bit "Function Set" opcode is
	//
	//		0-0-1-DL
	//
	//	N F	Lines	Font
	//	- -	-----	----
	//	0 0	1	5x8
	//	0 1	1	5x10
	//	1 0	2	5x8
	//
	//	8-bit "Function Set" opcode is
	//
	//		0-0-1-DL-N-F-X-X
	//
	//	These are all written to the Instruction Register
	//	of the LCD (RS = 0, R/W = 0)
	//	
	queueTransferWait( mc_init_long_delay,	0b00110000 );	// Function Set + 8 bit mode (As a 4 bit instruction)
	queueTransferWait( mc_init_medium_delay,0b00110000 );	// Function Set + 8 bit mode (As a 4 bit instruction)
	queueTransferWait( mc_init_short_delay,	0b00110000 );	// Function Set + 8 bit mode (As a 4 bit instruction)
	queueTransferWait( mc_init_short_delay,	0b00100000 );	// Function Set + 4 bit mode (As a 4 bit instruction)
	if( _rows == 1 ) {
		//
		//	1 line displays get initialised here
		//
		queueTransferWait( mc_inst_short_delay, 0b00100000 );	// Function Set + 4 bit mode + 1 line + 5x8 font (As an 8 bit instruction)
	}
	else {
		//
		//	2 and 4 line displays here (as 4 line displays
		//	are doubled up 2 line displays).
		//
		queueTransferWait( mc_inst_short_delay, 0b00101000 );	// Function Set + 4 bit mode + 2 lines + 5x8 font (As an 8 bit instruction)
	}
	//
	//	And pause..
	//
	synchronise( 1000 );
	//
	//	Now some ordinary tidy up steps.
	//
	display( true );
	clear();
	leftToRight( true );
	cursor( false );
	blink( false );
	backlight( true );
}

//
//	This is an "interrupt" style routine called
//	when TWI transactions have been completed
//
void LCD_TWI_IO::done( bool ok ) {
	//
	//	Note last TWI action completed
	//
	_fsm_twi_success = ok;
	_fsm_twi_returns = true;
}

//
//	This is the callback routine which is passed into the TWI IO
//	system along with a (void *) cast copy of the "this" pointer
//
//	This will allow the LCD code to know when the data has been
//	actually sent and the machine can be moved forwards.
//
//	This routine cannot be part of the object itself as it gets
//	called "outside" the objects framework.  Hence the need for
//	casting the link point back to the right type.
//
static void twi_callback( bool valid, void *link, byte *buffer, byte len ) {
	((LCD_TWI_IO *)link)->done( valid );
}


void LCD_TWI_IO::service( void ) {
	//
	//	Everything done in this routine is under the control
	//	of the current LCD program.  This is, effectively,
	//	where all the actual work gets done.
	//
	switch( pgm_read_byte( _fsm_instruction )) {
		case mc_idle: {			// machine at idle
			//
			//	If we are here then if there is something in the queue
			//	we load up the necessary components and run the
			//	program.
			//
			if( _queue_len ) {
				//
				//	Load the _fsm_ variables with the required
				//	data to run this job and remove the job from
				//	the queue.
				//
				_fsm_data_byte = _queue[ _queue_out ].value;
				_fsm_instruction = _queue[ _queue_out ].program;
				if( ++_queue_out >= max_pending ) _queue_out = 0;
				_queue_len--;
			}
			break;
		};
		case mc_reset: {		// Resetting the device
			_fsm_buffer = 0;
			_fsm_instruction++;
			break;
		}
		case mc_inst_high_enable: {	// send high nybble with E=1 as inst
			//
			//	RS = 0	(Select IR)
			//	R/W = 0 (Write)
			//	E = 1	(Enable)
			//
			_fsm_buffer = LCD_TWI_IO_HIGH_NYBBLE( _fsm_data_byte ) | _backLight | bit( LCD_TWI_IO_ENABLE );
			_fsm_instruction++;
			break;
		};
		case mc_inst_high_disable: {	// send high nybble with E=0 as inst
			//
			//	RS = 0	(Select IR)
			//	R/W = 0 (Write)
			//	E = 0	(Disable)
			//
			_fsm_buffer = LCD_TWI_IO_HIGH_NYBBLE( _fsm_data_byte ) | _backLight;
			_fsm_instruction++;
			break;
		};
		case mc_inst_low_enable: {	// send low nybble with E=1 as inst
			//
			//	RS = 0	(Select IR)
			//	R/W = 0 (Write)
			//	E = 1	(Enable)
			//
			_fsm_buffer = LCD_TWI_IO_LOW_NYBBLE( _fsm_data_byte ) | _backLight | bit( LCD_TWI_IO_ENABLE );
			_fsm_instruction++;
			break;
		};
		case mc_inst_low_disable: {	// send low nybble with E=0 as inst
			//
			//	RS = 0	(Select IR)
			//	R/W = 0 (Write)
			//	E = 0	(Disable)
			//
			_fsm_buffer = LCD_TWI_IO_LOW_NYBBLE( _fsm_data_byte ) | _backLight;
			_fsm_instruction++;
			break;
		};
		case mc_data_high_enable: {	// send high nybble with E=1 as data
			//
			//	RS = 1	(Select DR)
			//	R/W = 0 (Write)
			//	E = 1	(Enable)
			//
			_fsm_buffer = LCD_TWI_IO_HIGH_NYBBLE( _fsm_data_byte ) | _backLight | bit( LCD_TWI_IO_REGISTER_SELECT ) | bit( LCD_TWI_IO_ENABLE );
			_fsm_instruction++;
			break;
		};
		case mc_data_high_disable: {	// send high nybble with E=0 as data
			//
			//	RS = 1	(Select DR)
			//	R/W = 0 (Write)
			//	E = 0	(Disable)
			//
			_fsm_buffer = LCD_TWI_IO_HIGH_NYBBLE( _fsm_data_byte ) | _backLight | bit( LCD_TWI_IO_REGISTER_SELECT );
			_fsm_instruction++;
			break;
		};
		case mc_data_low_enable: {	// send low nybble with E=1 as data
			//
			//	RS = 1	(Select DR)
			//	R/W = 0 (Write)
			//	E = 1	(Enable)
			//
			_fsm_buffer = LCD_TWI_IO_LOW_NYBBLE( _fsm_data_byte ) | _backLight | bit( LCD_TWI_IO_REGISTER_SELECT ) | bit( LCD_TWI_IO_ENABLE );
			_fsm_instruction++;
			break;
		};
		case mc_data_low_disable: {	// send low nybble with E=0 as data
			//
			//	RS = 1	(Select DR)
			//	R/W = 0 (Write)
			//	E = 0	(Disable)
			//
			_fsm_buffer = LCD_TWI_IO_LOW_NYBBLE( _fsm_data_byte ) | _backLight | bit( LCD_TWI_IO_REGISTER_SELECT );
			_fsm_instruction++;
			break;
		};
		case mc_transmit_buffer: {	// send the content of the buffer
			//
			//	We send and send again until this returns true then
			//	move to the next instruction
			//
			_fsm_twi_returns = false;
			if( twi_cmd_send_data( _address, &_fsm_buffer, 1, (void *)this, twi_callback )) _fsm_instruction++;
			break;
		}
		case mc_wait_on_done: {		// Wait for the TWI action to complete
			if( _fsm_twi_returns ) {
				//
				//	Transmission completed
				//
				if( _fsm_twi_success ) {
					//
					//	Successfully
					//
					_fsm_instruction++;
				}
				else {
					//
					//	Unsuccessfully
					//
					//	What do we do?  We are (probably)
					//	in the middle of a series of a series
					//	of data transfers such that "losing"
					//	one will break the activity.
					//
					//	Best option is to drop all pending
					//	activities and execute a reset.
					//
					_queue_len = 0;
					_queue_in = 0;
					_queue_out = 0;
					_fsm_instruction = mc_reset_program;
				}
			}
			break;
		}
		case mc_set_delay_40000us: {	// Set delay countdown to 40000us
			_fsm_delay = 40000;
			_fsm_time_starts = micros();
			_fsm_instruction++;
			break;
		}
		case mc_set_delay_4200us: {	// 4200us
			_fsm_delay = 4200;
			_fsm_time_starts = micros();
			_fsm_instruction++;
			break;
		};
		case mc_set_delay_1600us: {	// 1600us
			_fsm_delay = 1600;
			_fsm_time_starts = micros();
			_fsm_instruction++;
			break;
		};
		case mc_set_delay_150us: {	// 150us
			_fsm_delay = 150;
			_fsm_time_starts = micros();
			_fsm_instruction++;
			break;
		};
		case mc_set_delay_41us: {	// 41us
			_fsm_delay = 41;
			_fsm_time_starts = micros();
			_fsm_instruction++;
			break;
		};
		case mc_set_delay_37us: {	// 37us
			_fsm_delay = 37;
			_fsm_time_starts = micros();
			_fsm_instruction++;
			break;
		};
		case mc_set_delay_10us: {	// 10us
			_fsm_delay = 10;
			_fsm_time_starts = micros();
			_fsm_instruction++;
			break;
		};
		case mc_delay_wait: {	// Wait until the delay period has expired
			if(( micros() - _fsm_time_starts ) > _fsm_delay ) _fsm_instruction++;
			break;
		};
		default: {
			//
			//	This should not happen, but if it does then
			//	there is a real problem.  All I can do is
			//	log an error (if possible) and reset the program.
			//
			_fsm_instruction = mc_idle_program;
			break;
		}
	}
	//
	//	If there is frame buffer defined, then we need to continue a scan
	//	of its content to find updated elements, and then work to get these
	//	transmitted to the LCD.
	//
	if( _frame_size ) {
		byte	c;
		
		//
		//	The _frame_last variable gives the index of the current
		//	output cursor on the LCD, the _frame_next gives the current
		//	scan position.
		//
		//	Is there something to do at the current scan position?
		//
		c = _frame_buffer[ _frame_next ];
		if( c & 0x80 ) {
			//
			//	All "pending" updates in the frame buffer have
			//	their top bit set. Once a position has been updated
			//	This bit is reset.
			//
			if( _frame_next == _frame_last ) {
				//
				//	We can just directly output the character
				//	to the display.
				//
				c &= 0x7f;
				if( write( c )) {
					//
					//	Success, update frame buffer, move
					//	next and last onwards.
					//
					_frame_buffer[ _frame_next ] = c;
					//
					//	Now move on the next and last values.
					//
					//	But .. remembering that the LCD does
					//	not wrap across the edge of the display.
					//
					_frame_next += 1;
					if( _frame_next >= _frame_size ) {
						_frame_next = 0;
					}
					else {
						if( _frame_next % _cols ) {
							_frame_last = _frame_next;
						}
					}
				}
			}
			else {
				//
				//	We need to move the output cursor to where we need it
				//
				if( index( _frame_next )) {		
					_frame_last = _frame_next;
				}
			}
		}
		else {
			//
			//	move on to next byte
			//
			if( ++_frame_next >= _frame_size ) _frame_next = 0;
		}
	}
}

//
//	Functional Routines
//	===================
//
bool LCD_TWI_IO::backlight( bool on ) {
	//
	//	The back light is explicitly controlled by
	//	one of the pins on the LCDs primary hardware
	//	interface.  The _backLight variable is
	//	used as part of the mask when writing to
	//	the LCD so the LCD is controlled simply by
	//	setting (or resetting) this specific bit.
	//
	bitWrite( _backLight, LCD_TWI_IO_BACKLIGHT, on );
	//
	//	The change in backlight status will be
	//	implemented on any following command to
	//	the LCD
	//
	return( true );
}

bool LCD_TWI_IO::clear( void ) {
	if( _frame_size ) {
		for( byte i = 0; i < _frame_size; _frame_buffer[ i++ ] = ( SPACE | 0x80 ));
		_frame_last = 0;
		_frame_next = 0;
	}
	return( queueTransfer( mc_inst_long_delay, LCD_TWI_IO_CLEAR_SCREEN ));
}

bool LCD_TWI_IO::home( void ) {
	if( _frame_size ) {
		_frame_last = 0;
		_frame_next = 0;
	}
	return( queueTransfer( mc_inst_long_delay, LCD_TWI_IO_HOME_SCREEN ));
}

bool LCD_TWI_IO::leftToRight( bool l2r ) {
	//
	//	Note that this facility does not apply consistently
	//	to "wrapping" between lines.  If you write off the end
	//	of line 0 it rolls into line 2.  Like wise rolling off
	//	line 1 rolls into line 3.
	//
	//	This is cause entirely by the memory mapping between
	//	the driver chip and the actual display.
	//
	bitWrite( _entryState, LCD_TWI_IO_LEFT_RIGHT, l2r );
	return( queueTransfer( mc_inst_short_delay, LCD_TWI_IO_ENTRY_STATE | _entryState ));
}

bool LCD_TWI_IO::autoscroll( bool on ) {
	bitWrite( _entryState, LCD_TWI_IO_AUTO_SCROLL, on );
	return( queueTransfer( mc_inst_short_delay, LCD_TWI_IO_ENTRY_STATE | _entryState ));
}

bool LCD_TWI_IO::display( bool on ) {
	bitWrite( _displayState, LCD_TWI_IO_DISPLAY_ON, on );
	return( queueTransfer( mc_inst_short_delay, LCD_TWI_IO_DISPLAY_STATE | _displayState ));
}

bool LCD_TWI_IO::cursor( bool on ) {
	bitWrite( _displayState, LCD_TWI_IO_CURSOR_ON, on );
	return( queueTransfer( mc_inst_short_delay, LCD_TWI_IO_DISPLAY_STATE | _displayState ));
}

bool LCD_TWI_IO::blink( bool on ) {
	bitWrite( _displayState, LCD_TWI_IO_BLINK_ON, on );
	return( queueTransfer( mc_inst_short_delay, LCD_TWI_IO_DISPLAY_STATE | _displayState ));
}

//bool LCD_TWI_IO::scrollDisplayLeft( void ) {
//	return( queueTransfer( mc_inst_short_delay, LCD_TWI_IO_SCROLL_LEFT ));
//}

//bool LCD_TWI_IO::scrollDisplayRight( void ) {
//	return( queueTransfer( mc_inst_short_delay, LCD_TWI_IO_SCROLL_RIGHT ));
//}

bool LCD_TWI_IO::position( byte col, byte row ) {
	byte newAddress;

	if( row < _rows ) {
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
	}
	else {
		newAddress = 0;
	}
	return( queueTransfer( mc_inst_short_delay, LCD_TWI_IO_SET_POSITION | newAddress ));
}

bool LCD_TWI_IO::index( byte posn ) {
	//
	//	similar to the position() routine, but views
	//	the buffer simply as a series of bytes starting
	//	at the top left and finishing at the bottom
	//	right.
	//
	//	Line 0
	//
	if( posn < _cols ) return( queueTransfer( mc_inst_short_delay, LCD_TWI_IO_SET_POSITION | posn ));
	//
	//	Line 1
	//
	posn -= _cols;
	if( posn < _cols ) return( queueTransfer( mc_inst_short_delay, LCD_TWI_IO_SET_POSITION |( 0x40 + posn )));
	//
	//	Line 2
	//
	posn -= _cols;
	if( posn < _cols ) return( queueTransfer( mc_inst_short_delay, LCD_TWI_IO_SET_POSITION |( _cols + posn )));
	//
	//	Line 3
	//
	posn -= _cols;
	if( posn < _cols ) return( queueTransfer( mc_inst_short_delay, LCD_TWI_IO_SET_POSITION |( 0x40 + _cols + posn )));
	//
	//	Out of bounds, put it top left, position 0.
	//
	return( queueTransfer( mc_inst_short_delay, LCD_TWI_IO_SET_POSITION ));
}


bool LCD_TWI_IO::write( byte val ) {
	return( queueTransfer( mc_data_short_delay, val ));
}

//
//	Routines which act upon the frame buffer
//	========================================
//

//
//	Provide the object with some buffer space in which
//	the frame buffer can be handled.
//
bool LCD_TWI_IO::setBuffer( byte *buffer, byte size ) {
	byte	required;

	if( buffer == NULL ) {
		//
		//	Releasing the frame buffer
		//
		_frame_size = 0;
		_frame_buffer = NULL;
		_frame_last = 0;
		_frame_next = 0;
		_frame_cursor = 0;
		return( true );
	}
	//
	//	Verify buffer is big enough
	//
	if( size < ( required = _rows * _cols )) return( false );
	//
	//	Set up frame buffer and empty it.
	//
	_frame_buffer = buffer;
	_frame_last = _rows * _cols;
	_frame_next = 0;
	_frame_cursor = 0;
	for( byte i = 0; i < _frame_size; _frame_buffer[ i++ ] = ( (byte)SPACE | 0x80 ));
	//
	//	Set frame size last as this triggers actions on the buffer.
	//
	_frame_size = required;
	//
	//	Wait ...
	//
	synchronise();
	//
	//	Done
	//
	return( true );
}

void LCD_TWI_IO::setPosn( byte col, byte row ) {
	//
	//	Simply set the cursor index if a frame buffer
	//	has been allocated.
	//
	if( _frame_size ) if(( _frame_cursor = ( row * _cols + col )) >=  _frame_size ) _frame_cursor = 0;
}

void LCD_TWI_IO::writeChar( char val ) {
	//
	//	Place character into the frame buffer at the cursor
	//	position and move the cursor forward one position.
	//	Wrap the cursor round to the top of the screen if
	//	you head off the end of the last line.
	//
	if( _frame_size ) {
		byte v = val & 0x7f;
		if( _frame_buffer[ _frame_cursor ] != v ) _frame_buffer[ _frame_cursor ] = v | 0x80;
		if(( _frame_cursor += 1 ) >= _frame_size ) _frame_cursor = 0;
	}
}

void LCD_TWI_IO::writeStr( const char *str ) {
	while( *str != EOS ) writeChar( *str++ );
}

void LCD_TWI_IO::writeBuf( const char *buf, byte len ) {
	while( len-- ) writeChar( *buf++ );
}

void LCD_TWI_IO::fill( char val, byte len ) {
	while( len-- ) writeChar( val );
}

//
//	Pause activities and wait until the
//	frame buffer has been updated.
//
void LCD_TWI_IO::synchronise( void ) {
	while( _queue_len ) {
		service();
		twi_eventProcessing();
	}
}
void LCD_TWI_IO::synchronise( word ms ) {
	unsigned long	start;

	start = millis();
	while(( millis() - start ) < ms ) synchronise();
}


//
//	EOF
//

//
//	FrameBuffer.h
//	=============
//
//	Capture the functionality of a directly addressed
//	LCD display in a "memory mapped" methodology.
//

#ifndef _FRAME_BUFFER_H_
#define _FRAME_BUFFER_H_


#include "Environment.h"
#include "Parameters.h"
#include "Configuration.h"

#include "LCD.h"
#include "Signal.h"
#include "Task_Entry.h"

#ifndef LCD_LOOKAHEAD_LIMIT
#define LCD_LOOKAHEAD_LIMIT 10
#endif

//
//	Declare the frame buffer
//
class FrameBuffer : public Task_Entry {
private:
	//
	//	Set constants pertinent to the display we hare handling.
	//
	static const byte	rows = LCD_DISPLAY_ROWS;
	static const byte	cols = LCD_DISPLAY_COLS;
	static constexpr byte	size = rows * cols;

	//
	//	Declare a "lookahead" limit in the search for cells which
	//	need an update.
	//
	static const byte	lookahead = LCD_LOOKAHEAD_LIMIT;
	
	//
	//	Declare the number of times (per second) that the frame
	//	buffer should target updating the LCD?
	//
	static const byte	refresh_rate = 5;
	
	//
	//	Declare the period (in milliseconds) which the frame buffer
	//	software will sleep for when there are no pending updates.
	//
	static constexpr word	idle_pause = ( 1000 / refresh_rate )/( size / lookahead );
	
	//
	//	Our target LCD.
	//
	LCD		*_lcd;
	
	//
	//	The following variables contain and manage the current "frame buffer"
	//	for the LCD.  These are only activated if a piece of memory is provided
	//	to be the frame buffer (the object does not allocate or contain any
	//	memory space for this purpose).
	//
	//	_buffer		Where the frame buffer is in memory.
	//
	//
	byte		_buffer[ size ];
	
	//
	//	Dynamic data relating to the content of the buffer.
	//
	//	_chk_r		The Row and Col position of the buffer
	//	_chk_c		scanning code.
	//
	//	_cursor		Insert point for next output text.
	//
	//	_pending	The number of positions in the frame buffer
	//			which contain updates that need to be
	//			propagated.
	//
	//	_sync		True if the LCD position is synchronised
	//			with the buffer scan position (r,c).
	//
	byte		_chk_r,
			_chk_c,
			_cursor,
			_pending;
	bool		_sync;

	//
	//	Declare the "State Machine" states that the Frame Buffer
	//	has, and the state variable which tracks them.
	//
	enum fsm_states {
		state_empty	= 0,		// An invalid state
		state_scan,			// Scanning for work
		state_locate,			// Updating LCD position
		state_transmit			// Transmitting to LCD
	}		_state;

	//
	//	Declare the flag used to control access to the
	//	state machine code.
	//
	Signal			_flag;
	
public:
	FrameBuffer( void );
	void initialise( LCD *lcd );
	void set_posn( byte row, byte col );
	void clear( void );
	void write_char( char val );
	void write_str( const char *str );
	void write_buf( const char *buf, byte len );
	void write_PROGMEM( const char *buf, byte len );
	void fill( char val, byte len );

	virtual void process( byte handle );
};


#endif

//
//	EOF
//

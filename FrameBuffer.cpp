//
//	FrameBuffer.cpp
//	===============
//
//	Implementation of the frame buffer
//

#include "FrameBuffer.h"
#include "Code_Assurance.h"
#include "Task.h"
#include "Trace.h"

#ifdef DEBUGGING_ENABLED
#include "Console.h"
#endif

//
//	Declare simple macros for tagging and clearing the "dirty"
//	bit in the frame buffer
//
#define TAG_DIRTY(b)	((b)|(byte)0x80)
#define IS_DIRTY(b)	((b)&(byte)0x80)
#define TAG_CLEAN(b)	((b)&(byte)0x7F)
#define IS_CLEAN(b)	(!IS_DIRTY(b))

FrameBuffer::FrameBuffer( void ) {
	_lcd = NIL( LCD );
	_chk_r = 0;
	_chk_c = 0;
	_cursor = 0;
	_sync = false;
	_state = state_empty;
}

void FrameBuffer::initialise( LCD *lcd ) {

	STACK_TRACE( "void FrameBuffer::initialise( LCD *lcd )" );

	ASSERT( lcd != NIL( LCD ));

	TRACE_FBUFFER( console.print( F( "FB flag " )));
	TRACE_FBUFFER( console.println( _flag.identity()));

	//
	//	Save the LCD driver.
	//
	_lcd = lcd;

	//
	//	Empty the buffer and tag data as
	//	needing to be updated.
	//
	for( byte i = 0; i < size; _buffer[ i++ ] = TAG_DIRTY( SPACE ));
	_pending = size;

	//
	//	Add ourselves to the task list.
	//
	if( !task_manager.add_task( this, &_flag )) ABORT( TASK_MANAGER_QUEUE_FULL );

	//
	//	We now initiate the FSM state and start processing
	//	by releasing that flag so the scheduler makes the
	//	first call to the updating process.
	//
	_state = state_scan;
	_flag.release();
}

void FrameBuffer::clear( void ) {

	STACK_TRACE( "void FrameBuffer::clear( void )" );

	//
	//	Empty the buffer and tag data as
	//	needing to be updated.
	//
	for( byte i = 0; i < size; _buffer[ i++ ] = TAG_DIRTY( SPACE ));
	_pending = size;
	//
	//	Set cursor to home.
	//
	_cursor = 0;
}

	
void FrameBuffer::set_posn( byte row, byte col ) {

	STACK_TRACE( "void FrameBuffer::set_posn( byte row, byte col )" );

	//
	//	Simply set the cursor index somewhere inside the frame
	//	buffer.  Resort to home if outside the display.
	//
	if(( _cursor = ( row * cols + col )) >= size ) _cursor = 0;
}

void FrameBuffer::write_char( char val ) {

	STACK_TRACE( "void FrameBuffer::write_char( char val )" );

	TRACE_FBUFFER( console.print( F( "FB print " )));
	TRACE_FBUFFER( console.print( val ));
	TRACE_FBUFFER( console.print( F( " at " )));
	TRACE_FBUFFER( console.println( _cursor ));

	ASSERT( _cursor < size );

	//
	//	Place character into the frame buffer at the cursor
	//	position and move the cursor forward one position.
	//	Wrap the cursor round to the top of the screen if
	//	you head off the end of the last line.
	//

	byte v = TAG_CLEAN( val );		// make sure the dirty bit is clear.
	byte w = _buffer[ _cursor ];		// pick out what we are over writing.
	
	//
	//	Place value into the frame buffer but only if
	//	it is different from the value already there.
	//
	if( TAG_CLEAN( w ) != v ) {

		TRACE_FBUFFER( console.println( F( "FB Update cell" )));
		
		//
		//	Put in new value having marked it as dirty...
		//
		_buffer[ _cursor ] = TAG_DIRTY( v );
		//
		//	...and increment the pending count only if the
		//	previous value was clean.
		//
		if( IS_CLEAN( w )) _pending += 1;
	}
	//
	//	Move the cursor forward, and wrap to the top
	//	if we fall off the bottom.
	//
	if(( _cursor += 1 ) >= size ) _cursor = 0;
}

void FrameBuffer::write_str( const char *str ) {

	STACK_TRACE( "void FrameBuffer::write_str( const char *str )" );

	char	c;
	
	while(( c = *str++ ) != EOS ) write_char( c );
}

void FrameBuffer::write_buf( const char *buf, byte len ) {

	STACK_TRACE( "void FrameBuffer::write_buf( const char *buf, byte len )" );

	while( len-- ) write_char( *buf++ );
}

void FrameBuffer::write_PROGMEM( const char *buf, byte len ) {

	STACK_TRACE( "void FrameBuffer::write_PROGMEM( const char *buf, byte len )" );

	while( len-- ) write_char( progmem_read_byte_at( buf++ ));
}

void FrameBuffer::fill( char val, byte len ) {

	STACK_TRACE( "void FrameBuffer::fill( char val, byte len )" );

	while( len-- ) write_char( val );
}


//
//	The procedure that provides the "dynamic" background updates
//	from the frame buffer to the LCD thus disconnecting the firmware
//	requirement to output data from the displays ability to accept it.
//
void FrameBuffer::process( UNUSED( byte handle )) {

	STACK_TRACE( "void FrameBuffer::process( byte handle )" );

	TRACE_FBUFFER( console.print( F( "FB state " )));
	TRACE_FBUFFER( console.print( (byte)_state ));
	TRACE_FBUFFER( console.print( F( " pending " )));
	TRACE_FBUFFER( console.println( _pending ));

	//
	//	Frame buffer support.
	//	=====================
	//
	//	This is, a little, fiddly.  To avoid this code scanning
	//	the whole frame buffer every time it is called we will
	//	only scan a limited number of locations before giving
	//	up our processor time.
	//
	//	If we find a cell needing an update then this gets done
	//	there and then and the CPU gets released.
	//

	//
	//	Note/
	//		At the moment the value of the _pending counter
	//		is used *only* to choose whether the code performs
	//		an immediate reschedule if it finds nothign to
	//		update or a delayed reschedule.
	//
	//		Once the content of _pending is seen to be correct
	//		(at I think it is now) then we can use its value
	//		to "stop/start" the scheduling of this update
	//		routine and save unnecessary calls to this routine.
	//

	//
	//	Run FSM code inside an infinite loop to enable
	//	the state machine to switch states without needing
	//	to release the CPU and be rescheduled.
	//
	while( true ) {
		switch( _state ) {
			case state_scan: {
				byte	i,	// index of cell being examined.
					c;	// content of the cell examined.

				TRACE_FBUFFER( console.println( F( "FB state_scan" )));
				
				//
				//	Scan Frame Buffer
				//	-----------------
				//
				//	We loop through this so we can look ahead a number of
				//	cells with each process call.  We do this to improve
				//	the "snappyness" of the LCD updates.
				//
				for( byte l = lookahead; l; l-- ) {
					//
					//	The _chk_r and _chk_c variables give the current position
					//	of where we are looking in the buffer, extract the
					//	content of that cell (and save its index).
					//
					c = _buffer[( i = _chk_r * cols + _chk_c )];

					TRACE_FBUFFER( console.print( F( "FB cell " )));
					TRACE_FBUFFER( console.print( _chk_r ));
					TRACE_FBUFFER( console.print( COMMA ));
					TRACE_FBUFFER( console.print( _chk_c ));
					TRACE_FBUFFER( console.print( F( " = " )));
					TRACE_FBUFFER( console.print_hex( c ));

					//
					//	Is there something to do at the current scan position?
					//	if so then break out of this loop to handle it.
					//
					if( IS_DIRTY( c )) {
						
						TRACE_FBUFFER( console.println( F( " dirty" )));

						break;
					}

					TRACE_FBUFFER( console.println( F( " clean" )));

					//
					//	We skip clean cells - nothing to do.
					//
					//	Any character we are skipping over means that
					//	the output LCD cursor is in the wrong place, so
					//	we reset the sync flag to false.
					//
					_sync = false;

					//
					//	Move on to the next byte, and wrap if we get to
					//	edge of the LCD or return go home at the bottom
					//	of the display.
					//
					if(( _chk_c += 1 ) >= cols ) {
						_chk_c = 0;
						if(( _chk_r += 1 ) >= rows ) {
							_chk_r = 0;
						}
					}
				}

				//
				//	If we get here on a clean cell then we
				//	need to re-schedule a continuation of
				//	the search for dirty cells.
				//
				if( IS_CLEAN( c )) {
					//
					//	Schedule the scan code again because we haven't
					//	found anything to do yet.
					//
					if( _pending > 0 ) {
						//
						//	If there are dirty cells awaiting display
						//	then we self schedule by raising our own
						//	signal before returning.
						//
						_flag.release();
					}
					else {
						//
						//	.. otherwise we make a longer schedule time.
						//
						if( !event_timer.delay_event( MSECS( idle_pause ), &_flag, false )) {
							//
							//	Failure to schedule event means logging an error
							//	and self scheduling.
							//
							errors.log_error( EVENT_TIMER_QUEUE_FULL, idle_pause );
							_flag.release();
						}
					}

					//
					//	End processing.
					//
					return;
				}

				//
				//	Is the scan position synchronised with the LCD output
				//	(cursor) location?  If not then we need to take a
				//	moment to relocate the LCD cursor.
				//
				if( !_sync ) {

					TRACE_FBUFFER( console.print( F( "FB resync to " )));
					TRACE_FBUFFER( console.println( i ));

					//
					//	Reposition the LCD Cursor
					//	-------------------------
					//
					//	We need to move the output cursor on the LCD
					//	to where we need to output a pending character.
					//
					if( _lcd->index( i, &_flag )) {
						//
						//	re-position LCD cursor submitted - change
						//	state for next process call (when the cursor
						//	has been moved).
						//
						_state = state_locate;
					}
					else {
						//
						//	Log an error (this ought not happen really).
						//
						errors.log_error( LCD_QUEUE_FULL, i );
						
						//
						//	Failed to schedule a write cursor movement on the
						//	LCD - we need to reschedule the scan code.
						//
						_flag.release();
					}

					//
					//	End processing.
					//
					return;
				}
				

				TRACE_FBUFFER( console.print( F( "FB update cell at " )));
				TRACE_FBUFFER( console.println( i ));
				
				//
				//	All "pending" updates in the frame buffer have
				//	their top bit set. Once a position has been updated
				//	This bit is reset.
				//
				//	We can just directly output the character
				//	to the display.
				//
				c = TAG_CLEAN( c );
				if( _lcd->write( c, &_flag )) {
					//
					//	Success, update frame buffer
					//	with the cleaned character cell.
					//
					_buffer[ i ] = c;
					_pending -= 1;

					//
					//	Change state for next process call.
					//
					_state = state_transmit;
				}
				else {
					//
					//	Log an error (this ought not happen really).
					//
					errors.log_error( LCD_QUEUE_FULL, i );
						
					//
					//	Failed to write the character to the
					//	LCD - we need to reschedule our attempt
					//	to do so.
					//
					_flag.release();
				}
				
				//
				//	End processing.
				//
				return;
			}
			case state_locate: {
				
				TRACE_FBUFFER( console.println( F( "FB state_locate" )));
				
				//
				//	We get here because the scan case discovered that
				//	we need to tell the LCD where the next character
				//	has to go (frane buffer and LCD cursor positions
				//	are not synchronised).
				//
				//	However, they are synchronised now.
				//
				_sync = true;
				
				//
				//	Re-scan this cell now we are position synchronised.
				//
				_state = state_scan;
				
				//
				//	Return to the top of the state machine processing
				//	by leaving the switch statement and re-entering from
				//	the top.
				//
				break;
			}
			case state_transmit: {
				
				TRACE_FBUFFER( console.println( F( "FB state_transmit" )));
				
				//
				//	When we get here the LCD has been updated
				//	so we can move on to the next cell.
				//
				if(( _chk_c += 1 ) >= cols ) {
					//
					//	If we "run off" the edge of the
					//	line then we will have to get the
					//	LCD to re-position its cursor
					//	on the next logical cell.
					//
					_sync = false;
					_chk_c = 0;
					if(( _chk_r += 1 ) >= rows ) {
						_chk_r = 0;
					}
				}
				
				//
				//	Scan another position.
				//
				_state = state_scan;
				
				//
				//	Return to the top of the state machine processing
				//	by leaving the switch statement and re-entering from
				//	the top.
				//
				break;
			}
			default: {
				//
				//	Here is proof that I have got something wrong!
				//
				ABORT( PROGRAMMER_ERROR_ABORT );
				return;
			}
		}
	}
}

//
//	EOF
//

//
//	HCI.h
//	=====
//
//	The Human Computer Interface.
//

#include "HCI.h"
#include "DCC_Constant.h"
#include "DCC.h"
#include "Errors.h"
#include "Clock.h"
#include "Constants.h"
#include "Task_Entry.h"
#include "Memory_Heap.h"
#include "Banner.h"
#include "District.h"
#include "Districts.h"
#include "Stats.h"
#include "Function.h"
#include "TOD.h"
#include "Task.h"
#include "Trace.h"

#ifdef DEBUGGING_ENABLED
#include "Console.h"
#endif

//
//	Do some compile time sanity checks.
//
#if ( LCD_DISPLAY_COLS != 20 )||( LCD_DISPLAY_ROWS != 4 )
#error "The HCI has assumed an LCD for 20x4 dimensions"
#endif

//
//	Define a set of single character symbols to represent
//	actions/directions applied to decoders/accessories when
//	displayed on the LCD.
//
#define LCD_ACTION_FORWARDS	'>'
#define LCD_ACTION_BACKWARDS	'<'
#define LCD_ACTION_STATIONARY	'='
#define LCD_ACTION_ENABLE	'+'
#define LCD_ACTION_DISABLE	'-'

#define LCD_CAB_OBJECT		'C'
#define LCD_ACCESSORY_OBJECT	'A'

#define LCD_DIVIDER_SYMBOL	'|'

//
//	Dimensions of the various areas.
//
#define LCD_DISPLAY_STATUS_WIDTH	5
#define LCD_DISPLAY_STATUS_COLUMN	0
//
#define LCD_DISPLAY_DISTRICT_WIDTH	6
#define LCD_DISPLAY_DISTRICT_HALF	(LCD_DISPLAY_DISTRICT_WIDTH/2)
#define LCD_DISPLAY_DISTRICT_COLUMN	(LCD_DISPLAY_STATUS_COLUMN+LCD_DISPLAY_STATUS_WIDTH+1)
//
#define LCD_DISPLAY_BUFFER_WIDTH	7
#define LCD_DISPLAY_BUFFER_COLUMN	(LCD_DISPLAY_DISTRICT_COLUMN+LCD_DISPLAY_DISTRICT_WIDTH+1)


//
//	Helper routine for displaying district data
//
bool HCI::fill_district( char *buf, byte len, byte dist ) {

	STACK_TRACE( "bool HCI::fill_district( char *buf, byte len, byte dist )" );

	ASSERT( len > 2 );
	ASSERT( dist < DCC_District::districts );

	char	f;

	buf[ 0 ] = 'A' + dist;
	switch( districts.state( dist )) {
		case District::state_on: {	// Running normally.
			//
			//	Return from here to simplify the
			//	code around this switch statement.
			//
			if( backfill_byte_to_text( buf+1, len-1, districts.load_average( dist ))) return( true );
			//
			//	Doesn't fit!!
			//
			f = '#';
			break;
		}
		case District::state_off: {	// Powered down.
			f = '_';
			break;
		}
		case District::state_shorted: {	// Shorted (Waiting).
			f = '!';
			break;
		}
		case District::state_inverted: {// Shorted (Inverted).
			f = '?';
			break;
		}
		case District::state_paused: {	// Paused
			f = '*';
			break;
		}
		default: {
			//
			//	Opps.
			//
			ABORT( PROGRAMMER_ERROR_ABORT );
			break;
		}
	}
	memset( buf+1, f, len-1 );
	return( false );
}

//
//	The update a line of the LCD with the status of the DCC Generator.
//
void HCI::update_lcd_line( byte line ) {

	STACK_TRACE( "void HCI::update_lcd_line( byte line )" );

	//
	//	We will include a spinner effect here.
	//
	static bool	spinner = false;

	//
	//	The STATUS data
	//	---------------
	//
	//	  0....0....1....1....2
	//	  0    5    0    5    0
	//	 +--------------------+
	//	0|SSSSS|              |
	//	1|SSSSS|              |
	//	2|SSSSS|              |
	//	3|SSSSS|              |
	//	 +--------------------+
	//
	//		F##Z#		Free buffers available, Zone/Power state
	//		T####		Packets Transmitted/second
	//		##:##		The Uptime in minutes and seconds
	//		M####		Free unassigned memory
	//
	//	The District Data
	//	-----------------
	//
	//	  0....0....1....1....2
	//	  0    5    0    5    0
	//	 +--------------------+
	//	0|     |DDDDDD|       |
	//	1|     |DDDDDD|       |
	//	2|     |DDDDDD|       |
	//	3|     |DDDDDD|       |
	//	 +--------------------+
	//
	//	If district count 4 (or less) then each row is a district
	//	using the following format using all 6 characters available.
	//	If the district count is 5 (or mroe) then each row is two
	//	districts using 3 characters.
	//
	//	The Buffer Data
	//	---------------
	//
	//	  0....0....1....1....2
	//	  0    5    0    5    0
	//	 +--------------------+
	//	0|            |BBBBBBB|
	//	1|            |BBBBBBB|
	//	2|            |BBBBBBB|
	//	3|            |BBBBBBB|
	//	 +--------------------+
	//
	//	The BUFFER area of the display, showing:
	//	Buffers in use and the action in place.
	//	This area is broken up into as many one
	//	line areas as will fit.
	//    

	//
	//	The STATUS display
	//
	switch( line ) {
		case 0: {
			//
			//	Flip the spinner only on the top line.
			//
			spinner = !spinner;
			
			//	
			//
			//	F##Z#		Free buffers available, Zone/Power state
			//
			char	buf[ LCD_DISPLAY_STATUS_WIDTH ];

			buf[ 0 ] = 'F';
			if( !backfill_byte_to_text( buf+1, 2, dcc_generator.free_buffers())) memset( buf+1, HASH, 2 );
			buf[ 3 ] = 'P';
			buf[ 4 ] = '0' + districts.zone();
			//
			//	Place the data.
			//
			_display.set_posn( 0, LCD_DISPLAY_STATUS_COLUMN );
			_display.write_buf( buf, LCD_DISPLAY_STATUS_WIDTH );
			break;
		}
		case 1: {
			//
			//	T####		Packets Transmitted/second
			//
			char	buf[ LCD_DISPLAY_STATUS_WIDTH ];

			buf[ 0 ] = 'T';
			if( !backfill_word_to_text( buf+1, 4, stats.packets_sent())) memset( buf+1, HASH, 2 );
			//
			//	Place the data.
			//
			_display.set_posn( 1, LCD_DISPLAY_STATUS_COLUMN );
			_display.write_buf( buf, LCD_DISPLAY_STATUS_WIDTH );
			break;
		}
		case 2: {
			//
			//	M####		Free unassigned memory
			//
			char	buf[ LCD_DISPLAY_STATUS_WIDTH ];
			word	f;

			buf[ 0 ] = 'M';
			if(( f = heap.free_memory()) < 10000 ) {
				(void)backfill_word_to_text( buf+1, 4, f );
			}
			else {
				(void)backfill_word_to_text( buf+1, 3, f >> 10 );
				buf[ 4 ] = 'K';
			}
			
			//
			//	Place the data.
			//
			_display.set_posn( 2, LCD_DISPLAY_STATUS_COLUMN );
			_display.write_buf( buf, LCD_DISPLAY_STATUS_WIDTH );
			break;
		}
		case 3: {
			//
			//	##:##		The Uptime in minutes and seconds
			//
			char	buf[ LCD_DISPLAY_STATUS_WIDTH ],
				h, m;

			//
			//	We always need the minutes.
			//
			m = time_of_day.read( TOD::minutes );
			//
			//	Grab hours and if non-zero use alternate
			//	display format.
			//
			if(( h = time_of_day.read( TOD::hours ))) {
				buf[ 2 ] = spinner? 'h': ':';
				(void)backfill_byte_to_text( buf, 2, h );
				(void)backfill_byte_to_text( buf+3, 2, m, ZERO );
			}
			else {
				buf[ 2 ] = spinner? 'm': ':';
				(void)backfill_byte_to_text( buf, 2, m );
				(void)backfill_byte_to_text( buf+3, 2, time_of_day.read( TOD::seconds ), ZERO );
			}
			
			//
			//	Place the data.
			//
			_display.set_posn( 3, LCD_DISPLAY_STATUS_COLUMN );
			_display.write_buf( buf, LCD_DISPLAY_STATUS_WIDTH );
			break;
		}
		default: {
			//
			//	Should never get here.
			//
			ABORT( PROGRAMMER_ERROR_ABORT );
			break;
		}
	}

	//
	//	The DISTRICTS data
	//
	if( DCC_District::districts > 4 ) {
		char	buf[ LCD_DISPLAY_DISTRICT_WIDTH ];
		byte	d2;

		d2 = line + 4;
		(void)fill_district( buf, LCD_DISPLAY_DISTRICT_HALF, line );
		if( d2 < DCC_District::districts ) {
			(void)fill_district( buf+LCD_DISPLAY_DISTRICT_HALF, LCD_DISPLAY_DISTRICT_HALF, d2 );
		}
		else {
			memset( buf+LCD_DISPLAY_DISTRICT_HALF, SPACE, LCD_DISPLAY_DISTRICT_HALF );
		}
		_display.set_posn( line, LCD_DISPLAY_DISTRICT_COLUMN );
		_display.write_buf( buf, LCD_DISPLAY_DISTRICT_WIDTH );
	}
	else {
		char	buf[ LCD_DISPLAY_DISTRICT_WIDTH ];
		
		if( line < DCC_District::districts ) {
			buf[ LCD_DISPLAY_DISTRICT_WIDTH-1 ] = fill_district( buf, LCD_DISPLAY_DISTRICT_WIDTH-1, line )? '%': SPACE;
		}
		else {
			memset( buf, SPACE, LCD_DISPLAY_DISTRICT_WIDTH );
		}
		_display.set_posn( line, LCD_DISPLAY_DISTRICT_COLUMN );
		_display.write_buf( buf, LCD_DISPLAY_DISTRICT_WIDTH );
	}


	//
	//	The BUFFERS data
	//
#if LCD_DISPLAY_BUFFER_WIDTH < 7
#error "Buffer display area too narrow"
#endif
	{
		word	target, action;
		bool	mobile;
		
		if( line == 0 ) dcc_generator.reset_scan();
		_display.set_posn( line, LCD_DISPLAY_BUFFER_COLUMN );
		if( dcc_generator.scan_next( &target, &mobile, &action )) {
			if( DCC::is_speed_and_dir( action )) {
				char	buf[ LCD_DISPLAY_BUFFER_WIDTH ];
				byte	s;
				
				if( !backfill_word_to_text( buf, 4, target )) memset( buf, HASH, 4 );
				buf[ 4 ] = DCC::get_dir( action )? LCD_ACTION_FORWARDS: LCD_ACTION_BACKWARDS;
				if(( s = DCC::get_speed( action )) > 1 ) s--;
				if( !backfill_byte_to_text( buf+5, 2, s )) memset( buf+5, HASH, 2 );
				_display.write_buf( buf, LCD_DISPLAY_BUFFER_WIDTH );
			}
			else if( DCC::is_func_and_state( action )) {
				char	buf[ LCD_DISPLAY_BUFFER_WIDTH ];
				
				if( !backfill_word_to_text( buf, 4, target )) memset( buf, HASH, 4 );
				buf[ 4 ] = DCC::get_state( action )? LCD_ACTION_ENABLE: LCD_ACTION_DISABLE;
				if( !backfill_byte_to_text( buf+5, 2, DCC::get_func( action ))) memset( buf+5, HASH, 2 );
				_display.write_buf( buf, LCD_DISPLAY_BUFFER_WIDTH );
			}
			else if( DCC::is_accessory_state( action )) {
				char	buf[ LCD_DISPLAY_BUFFER_WIDTH ];

				buf[ 0 ] = LCD_ACCESSORY_OBJECT;
				if( !backfill_word_to_text( buf+1, 3, target )) memset( buf+1, HASH, 3 );
				buf[ 4 ] = DCC::get_accessory_state( action )? LCD_ACTION_ENABLE: LCD_ACTION_DISABLE;
				if( !backfill_byte_to_text( buf+5, 2, DCC::get_func( action ))) memset( buf+5, HASH, 2 );
				_display.write_buf( buf, LCD_DISPLAY_BUFFER_WIDTH );
			}
			else {
				_display.fill( SPACE, LCD_DISPLAY_BUFFER_WIDTH );
			}
		}
		else {
			_display.fill( SPACE, LCD_DISPLAY_BUFFER_WIDTH );
		}
	}
}

void HCI::process( UNUSED( byte handle )) {

	STACK_TRACE( "void HCI::process( byte handle )" );

	update_lcd_line( _display_line++ );
	if( _display_line >= LCD_DISPLAY_ROWS ) _display_line = 0;
}

//
//	Organise the HCI into action!
//
void HCI::initialise( void ) {

	STACK_TRACE( "void HCI::initialise( void )" );

	TRACE_HCI( console.print( F( "HCI display flag " )));
	TRACE_HCI( console.println( _display_flag.identity()));

	//
	//	Set up all the elements that form to HCI control.
	//
	_lcd.initialise( LCD_DISPLAY_ADRS, LCD_DISPLAY_ROWS, LCD_DISPLAY_COLS );
	_display.initialise( &_lcd );

	//
	//	Display the banner screen on the LCD.
	//
	framebuffer_banner( &_display );
	time_of_day.inline_delay( BANNER_DISPLAY_TIME );

	//
	//	Clear and initialise in the display with the static symbols.
	//
	_display.clear();
	for( byte r = 0; r < LCD_DISPLAY_ROWS; r++ ) {
		_display.set_posn( r, LCD_DISPLAY_DISTRICT_COLUMN-1 );
		_display.write_char( LCD_DIVIDER_SYMBOL );
		_display.set_posn( r, LCD_DISPLAY_BUFFER_COLUMN-1 );
		_display.write_char( LCD_DIVIDER_SYMBOL );
	}

	//
	//	Now kick off the events processing.
	//
	_display_line = 0;
	if( !event_timer.delay_event( MSECS( LINE_REFRESH_INTERVAL ), &_display_flag, true )) ABORT( EVENT_TIMER_QUEUE_FULL );
	if( !task_manager.add_task( this, &_display_flag, display_handle )) ABORT( TASK_MANAGER_QUEUE_FULL );
}


//
//	Declare the HCI object itself.
//
HCI hci_control;


//
//	EOF
//

//
//	Banner.cpp
//	==========
//
//	Implement the banner stuff.
//

#include "Banner.h"

//
//	Boot Splash Screen
//	==================
//
#define SPLASH_LINE_1	VERSION_NAME " V" VERSION_NUMBER
#define SPLASH_LINE_2	"MCU: " HW_TITLE
#define SPLASH_LINE_3	"Baud: " SERIAL_BAUD_RATE_STR
#define SPLASH_LINE_4	"Build: " __DATE__

//
//	Declare the program memory splash data.
//
static const char splash1[] PROGMEM = SPLASH_LINE_1;
static const char splash2[] PROGMEM = SPLASH_LINE_2;
static const char splash3[] PROGMEM = SPLASH_LINE_3;
static const char splash4[] PROGMEM = SPLASH_LINE_4;

void serial_banner( Byte_Queue_API *console ) {
	bool	s;

	s = console->synchronous( true );
	console->println();
	console->println_PROGMEM( splash1 );
	console->print_PROGMEM( splash2 );
		console->print( SPACE );
		console->print((word)( F_CPU / 1000000L ));
		console->println( F( "MHz" ));
	console->println_PROGMEM( splash3 );
	console->println_PROGMEM( splash4 );
	console->synchronous( s );
}

static void show_on_line( FrameBuffer *display, byte line, const char *text ) {
	byte	l;
	char	c;
	
	display->set_posn( line, 0 );
	l = LCD_DISPLAY_COLS;
	while(( l-- )&&(( c = progmem_read_byte_at( text++ )))) display->write_char( c );
}

void framebuffer_banner( FrameBuffer *display ) {
	show_on_line( display, 0, splash1 );
	show_on_line( display, 1, splash2 );
	show_on_line( display, 2, splash3 );
	show_on_line( display, 3, splash4 );
}

//
//	EOF
//

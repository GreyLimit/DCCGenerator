//
//	HCI.h
//	=====
//
//	The Human Computer Interface.
//

#ifndef _HCI_H_
#define _HCI_H_

#include "Environment.h"
#include "Parameters.h"
#include "Configuration.h"
#include "Menu.h"
#include "LCD.h"
#include "FrameBuffer.h"
#include "Formatting.h"

//
//	Declare the class containing the HCI control systems
//
class HCI : public Task_Entry {
private:
	//
	//	Define the handle used (via the task manager).
	//
	static const byte	display_handle = 1;

	//
	//	Define the associated flags used by the events.
	//
	Signal			_rotary_flag,
				_keypad_flag,
				_display_flag;
	
	//
	//	These are the other components which form the HCI.
	//
	LCD			_lcd;
	byte			_display_line;
	FrameBuffer		_display;

	//
	//	Helper routine for displaying district data.  Returns
	//	true if the district is running/ON.
	//
	bool fill_district( char *buf, byte len, byte dist );

public:
	//
	//	Redrawing..
	//
	void update_lcd_line( byte line );

	//
	//	The initialisation routine
	//
	void initialise( void );

	//
	//	The task entry routine.
	//
	virtual void process( byte handle );
};


//
//	Declare the HCI object itself.
//
extern HCI hci_control;

#endif

//
//	EOF
//

//
//	Menu
//	====
//
//	Define a class which implements the menu tree.
//

#ifndef _MENU_H_
#define _MENU_H_

//
//	The menus are controlled by the size of the screen
//	and the number of letters available on the 4x4 keypad.
//
#include "Configuration.h"
#include "Layout.h"

//
//	Menu / HCI design
//	=================
//
//	The display will have two areas available for the user interface;
//	a rectangle for the "menu" options, and another for the "objects"
//	being controlled.
//
//	The key restriction on the HCI is not the display but the input
//	mechanisms available.  For this firmware the key HCI input device
//	is the keypad.
//
//	The letters, when used directly, simply select a specific on
//	screen object to be the current object tied to the rotary control.
//
//	When a letter is used in combination with the "page shift" key
//	then this changes the page being displayed and so the objects
//	available for direct control.
//
//	The menu options can be selected when using a letter with the
//	"menu shift" key causing the assigned action to be executed.
//	The menus (yes, multiple) are not organised into pages, but are
//	rather linked into a series where the last option in every menu
//	is the option to move to the next menu.
//
//	To confuse things a little, I'll still call each group of menus
//	a "menu page".
//

//
//	Declare the number of objects per page and the total number of
//	pages.  This is directly controlled by the keypads ability to
//	generate letters from 'A' onwards  (and typically ends with 'D').
//
#define OBJECT_COUNT	LAYOUT_LETTERS
#define PAGE_COUNT	LAYOUT_LETTERS

//
//	Declare the record holding data for a single object.
//
struct object_data {
	//
	//	The DCC address of the object stored in this object.
	//
	//	+ve values	Mobile decoders, Engines.
	//	0		Empty
	//	-ve values	Static accessories, points/signals etc.
	//
	int		adrs;
	//
	//	The current state off the object.
	//
	//	The content of this is defined by the HCI code, and
	//	is dependent on type of object being represented here.
	//
	word		state;
};

//
//	This is a single page of objects:
//
struct page_data {
	object_data	object[ OBJECT_COUNT ];
};

//
//	Finally we define the structure which will be embeded into
//	the constants data that is restored from EEPROM when the 
//	firmware begins.
//
struct page_memory {
	page_data	page[ PAGE_COUNT ];
};

//
//	Defintions associated with the Menus
//	====================================
//

//
//	Define the number of menu items "per page" and the total
//	number of menus in the system
//
#define ITEM_COUNT	LAYOUT_LETTERS
#define MENU_COUNT	3

//
//	Declare the size of a menu entry.
//
#define MENU_ITEM_SIZE	4

//
//	Define the set of menu handles associated with the menu
//	entries available in the menu pages.
//
struct menu_item {
	char		text[ MENU_ITEM_SIZE ];
	byte		action;
};

//
//	A set of menu items as a page.
//
struct menu_page {
	menu_item	item[ ITEM_COUNT ];
};

//
//	And all of the menus
//
struct menu_memory {
	menu_page	page[ MENU_COUNT ];
};

//
//	Finally we define the set of actions which the menus
//
//	None		No action
//	NEW_MOBILE	Assign a cab/mobile decoder to the current
//			object.
//	NEW_STATIC	Assign a static decoder address to the current
//			object.
//	ERASE		Empty the current object.
//	SAVE		Take a snapshot of the controller status and
//			objects.
//	STOP		Turn power off
//	START		Turn power on
//	TOGGLE		Toggle the power status ON to OFF or OFF to ON.
//	STATUS		Toggle the status page of the display.
//
#define ACTION_NONE		0
#define ACTION_NEW_MOBILE	1
#define ACTION_NEW_STATIC	2
#define ACTION_ERASE		3
#define ACTION_NEXT		4
#define ACTION_SAVE		5
#define ACTION_STOP		6
#define ACTION_START		7
#define ACTION_TOGGLE		8
#define ACTION_STATUS		9

//
//	And here are the menus themselves.
//
extern const menu_memory	menus PROGMEM;

#endif

//
//	EOF
//

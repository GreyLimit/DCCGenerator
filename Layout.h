//
//	Layout.h
//	========
//
//	Specify key facts about the keypad layout.
//
//

#ifndef _LAYOUT_H_
#define _LAYOUT_H_

//
//	Define the size of the keypad in rows and cols.  The
//	rows (and cols) are indexed from 0 to N-1.
//
#define LAYOUT_ROWS			4
#define LAYOUT_COLS			4

//
//	Define the total number of keys on the keypad
//
#define LAYOUT_KEYS			(LAYOUT_ROWS*LAYOUT_COLS)

//
//	Define the number of bits required to hold a row and column
//	index.
//
#define LAYOUT_ROW_BITS			2
#define LAYOUT_COL_BITS			2

//
//	Binary bit values related to the row and column values and
//	how they map into the 8 bit device IO port.
//
#define LAYOUT_ROW_MASK			0x0f
#define LAYOUT_ROW_LSB			4
#define LAYOUT_COL_MASK			0x0f
#define LAYOUT_COL_LSB			0

//
//	Define a short delay between scanning activites (milliseconds).
//
#define LAYOUT_SCAN_DELAY		10

//
//	Define MIRROR_LAYOUT if the percieved results are flipped and
//	rotated relative to the real keybopard.  This means the parallel
//	interface to the scanning pins is in the reverse order.
//
//#define MIRROR_LAYOUT			1


//
//	Declare the mapping between "row x col" and the ASCII (7-bit)
//	value returned.
//
extern const byte 			keypad_mapping[ LAYOUT_KEYS ] PROGMEM;

//
//	Declare symbolic values for elements of the keypad:
//
//	Note the number of letters ('A'..'D' in this example) and also
//	the two "control/shift" keys providing alternate meanings to
//	keypad buttons.
//
#define LAYOUT_LETTERS			4
#define LAYOUT_PAGE_SHIFT		'#'
#define LAYOUT_MENU_SHIFT		'*'

#define LAYOUT_IS_LETTER(k)		(((k)>='A')&&((k)<='D'))
#define LAYOUT_LETTER_INDEX(k)		((k)-'A')

#define LAYOUT_IS_NUMBER(k)		(((k)>='0')&&((k)<='9'))
#define LAYOUT_NUMBER_INDEX(k)		((k)-'0')


#endif

//
//	EOF
//

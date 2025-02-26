//
//	Formatting.h
//	============
//
//	Routines for conversion of numeric data into human text.
//

#ifndef _FORMATTING_H_
#define _FORMATTING_H_

//
//	Numerical output formatting routines
//	------------------------------------
//
//	This "back fill" integer to text routine is used only
//	by the LCD update routine.  Returns false if there was
//	an issue with the conversion (and remedial action needs
//	to be done) or true if everything worked as planned.
//
//	The "int" version handles signed 16 bit numbers, the word
//	version unsigned 16 bit numbers and the byte version unsigned
//	8 bit values.
//
extern bool backfill_int_to_text( char *buf, byte len, int v, char fill = SPACE );
extern bool backfill_word_to_text( char *buf, byte len, word v, char fill = SPACE );
extern bool backfill_byte_to_text( char *buf, byte len, byte v, char fill = SPACE );

//
//	Simple front loading routines, return number of bytes used,
//	or zero if the buffer is too small.
//
extern byte byte_to_text( char *buf, byte len, byte v );
extern byte word_to_text( char *buf, byte len, word v );


#endif

//
//	EOF
//

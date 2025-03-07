//
//	DCC_CV_DB.h
//	===========
//
//	Define the data structures that capture the higher level
//	meaning of the individual CVs.
//


#ifndef _DCC_CV_DB_H_
#define _DCC_CV_DB_H_

#include "Environment.h"

//
//	Capture how a value (or part of a larger value) is
//	encoded in a single CV
//
struct CV_Element {
	word			cv;		// Which CV
	byte			bits,		// How many bits
				lsb;		// Least significant bit
};

//
//	The following structure captures the requirement for
//	CVs to be set in particular ways for a specific CV
//	to be enabled.  Specifically the switch between the
//	long and short form of addresses has multiple other
//	CV elements which need updating.
//
struct CV_Update {
	const CV_Element	*cv;
	word			value;
};

//
//	Capture a complete binary value across one or more
//	CV values.
//
struct CV_Value {
	const char		*name;		// What we call this value.
	byte			read_write,	// Variable can be updated, it is NOT
						// a read-only CV.
				combined;	// True if the data is a single value
						// (potentially spread over multiple locations)
						// false if this is an array of values.
	word			start, end;	// Range of valid values for this CV
						// inclusive.
	const CV_Element	*data;		// Details about where the value is
						// stored.
						//
						// If the value is combined across multiple
						// CVs then list the LSB first then through
						// to the MSB last.
						//
						// Arrays of data are store in array number
						// order, index '0' first.
						//
	const CV_Update		*update;	// A series of updates which must also be
						// actioned before the update to this CV
						// becomes enabled.
						//
};

//
//	Define a data structure and supporting routine to facilitate
//	the possible consolidation of CV updates into the smallest
//	possible number of DCC commands.
//
struct CV_Change {
	int		cv;		// CV number to change
	byte		mask,		// Bits to change
			value;		// New value of bits
};


//
//	Return a record for a configuration variable in PROGMEM
//
const CV_Value *find_cv_variable( char *name );

//
//	Clear a set of CV change records.
//
void clear_cv_change( CV_Change *list, int len );

//
//	Record a pending CV change
//
bool add_cv_change( CV_Change *list, int len, int cv, int b, int v );

//
//	Push an update of a specific CV Element into the pending changes.
//
bool make_cv_change( CV_Change *list, int len, const CV_Element *cve, word val, bool combined );



#endif

//
//	EOF
//

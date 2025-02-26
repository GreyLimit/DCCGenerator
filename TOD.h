//
//	TOD.h
//	=====
//
//	The definition of the "time of day" module.
//
//	This has two roles:
//
//	To track the time of day at a "per second" granularity to
//	allow the passing of time to be assessed at human scales.
//
//	To provide a flag management system (used as input the
//	the task management code) where flags can be set true at a
//	human period of time in the future (65535 seconds ahead,
//	approximately 18 hours).
//

#ifndef _TOD_H_
#define _TOD_H_

//
//	Bring in the necessary definitions.
//
#include "Environment.h"
#include "Parameters.h"
#include "Configuration.h"
#include "Signal.h"
#include "Task_Entry.h"

//
//	Define how many TOD Tasks we will manage.
//
#ifndef TIME_OF_DAY_TASKS
#define TIME_OF_DAY_TASKS	8
#endif

//
//	Declare the TOD class.
//
class TOD : public Task_Entry {
	//
	//	The TIME OF DAY element of the class
	//	------------------------------------
	//
public:
	//
	//	Define how many "steps" the counter contains.  Each step
	//	represents a progressively larger unit of time, starting
	//	with seconds.
	//
	//	Defined, so far:
	//		0		seconds,
	//		1		minutes,
	//		2		hours,
	//		3		days.
	//
	static const byte stages = 4;

	//
	//	Define constants for the various stages:
	//
	static const byte seconds	= 0;
	static const byte minutes	= 1;
	static const byte hours		= 2;
	static const byte days		= 3;
	
private:
	//
	//
	//	Here we have the counter limits.
	//
	static const byte limit[ stages ] PROGMEM;

	//
	//	Here are the counters.
	//
	byte		_stage[ stages ];
	word		_elapsed;

	//
	//	Here is the flag required as part of the scheduling
	//	process.
	//
	Signal		_flag;

	//
	//	The TOD Flag Manager components.
	//
	struct pending_tod {
		word		left;
		Signal		*flag;
		pending_tod	*next;
	};
	
	//
	//	flag records and pointers.
	//
	pending_tod	*_active,
			*_free;

public:
	//
	//	Constructor and initialise routine.
	//
	TOD( void );
	void initialise( void );

	//
	//	Provide access to the TOD data.
	//
	byte read( byte index );
	bool write( byte index, byte value );

	//
	//	Provide an *indication* of time since boot
	//	in seconds.  This value, an unsigned 16 bit word
	//	will wrap to 0 every 18 hours, 12 minutes and
	//	(roughly) 15 seconds.
	//
	word elapsed( void );

	//
	//	Add a flag to the list of pending flag updates.  The
	//	duration is specified in whole seconds upto 65535
	//	seconds into the future.  There is no "time of day"
	//	based clock scheduling.
	//
	bool add( word duration, Signal *flag );

	//
	//	The TASK entry point, called each time the flag is
	//	set true by the clock system.
	//
	virtual void process( byte handle );

	//
	//	A human scale inline delay routine.
	//
	void inline_delay( word seconds );
};

//
//	And here is the TOD object.
//
extern TOD time_of_day;

#endif

//
//	EOF
//

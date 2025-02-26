//
//	Stats.h
//	=======
//
//	A system where stats are gathered at regular intervals
//	from various parts of the environment.
//


#ifndef _STATS_H_
#define _STATS_H_

#include "Environment.h"
#include "Parameters.h"
#include "Configuration.h"
#include "Average.h"
#include "Task_Entry.h"
#include "Signal.h"

//
//	Define the averaging range we are going to empty for the
//	stats.
//
#ifndef STATS_AVERAGE_READINGS
#define STATS_AVERAGE_READINGS	4
#endif

//
//	Define the number of Milliseconds overwhich the readings are
//	spanned.
//
#ifndef STATS_AVERAGE_PERIOD
#define STATS_AVERAGE_PERIOD	1000
#endif

//
//	Declare the stats object.
//
class Stats : public Task_Entry {
private:

#ifdef ENABLE_COUNT_INTERRUPTS
	//
	//	Collect stats about the number of interrupts
	//	which we are collecting.  Here we provide only
	//	variable into which the data is gathered.
	//
	bool	_interrupt_over;
	word	_interrupt_count;

	//
	//	The average mechanism
	//
	Average< STATS_AVERAGE_READINGS, word >	_interrupts;
#endif

	//
	//	The internal stats we are keeping.
	//
	Average< STATS_AVERAGE_READINGS, byte >	_packets_sent;
	Average< STATS_AVERAGE_READINGS, byte >	_free_buffers;

	//
	//	The control signal used to schedule this object.
	//
	Signal		_flag;

public:
	//
	//	Call initialise to get the system going.
	//
	void initialise( void );

	//
	//	The routine called once a "period" to gather
	//	in more stats and .. process them.
	//
	virtual void process( byte handle );

	//
	//	Return the packets set in the last time period.
	//
	word packets_sent( void );

	//
	//	Return the free buffers in the last time period.
	//
	byte free_buffers( void );

#ifdef ENABLE_COUNT_INTERRUPTS
	//
	//	Count an interrupt.
	//
	inline void count_interrupt( void ) {
		if( ++_interrupt_count == 0 ) _interrupt_over = true;
	}
	
	//
	//	Return the average interrupts processed.
	//
	word interrupts_caught( void );
#endif


};

//
//	The interrupt stats are gathered through this macro.
//
#ifdef ENABLE_COUNT_INTERRUPTS
#define COUNT_INTERRUPT		stats.count_interrupt()
#else
#define COUNT_INTERRUPT
#endif

//
//	DCC delay capture.
//
#ifdef ENABLE_DCC_DELAY_REPORT
#define SAVE_DCC_DELAY(v)	stats.add_dcc_delay(v)
#else
#define SAVE_DCC_DELAY(v)
#endif

//
//	The stats object.
//
extern Stats stats;

#endif

//
//	EOF
//

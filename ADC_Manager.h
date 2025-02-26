//
//	ADC_Manager.h:	This AVR implementation of the ADC reading
//			environment.
//

#ifndef _AVR_MANAGER_H_
#define _AVR_MANAGER_H_

#include "Environment.h"
#include "Parameters.h"
#include "Configuration.h"
#include "Task_Entry.h"
#include "Signal.h"


//
//	The AVR Analogue to Digital Conversion management class.
//
class ADC_Manager : public Task_Entry {
private:
	//
	//	The following structure is used by the ADC manager to
	//	queue up multiple conversion requests so that the ADC
	//	hardware can be 100% utilised.
	//
	struct pending_adc {
		//
		//	The actual pin that is to be read.  This is the
		//	analogue pin number (not the digital equivalent). 
		//
		byte		pin;
		//
		//	Where the value of the reading is to be placed
		//	when obtained.
		//
		word		*save;
		//
		//	The flag to be set when the reading has been
		//	collected.
		//
		Signal		*flag;
		//
		//	The address of the next record.
		//
		pending_adc	*next;
	};

	//
	//	Define the queue and free record list.
	//
	pending_adc	*_active,
			**_tail,
			*_free;

	//
	//	The signal used to hand off between the ISR and
	//	the processing code, and the variable that holds
	//	the reading value at the time of the ISR.
	//
	Signal		_irq;
	word		_reading;

	//
	//	This is the routine which initiates a single analogue
	//	conversion on the hardware.  This uses none of the
	//	internal data structures of this class.
	//
	void start_conversion( byte pin );

public:
	//
	//	Constructor
	//
	ADC_Manager( void );

	//
	//	Provide the hardware initialisation routine.
	//
	void initialise( void );

	//
	//	Request that a pin has its voltage read.  When
	//	the reading is complete the value obtained is
	//	stored at result and the flag is set.
	//
	bool read( byte pin, Signal *flag, word *result );

	//
	//	The entry point from the task manager.
	//
	virtual void process( byte handle );
	
	//
	//	The routine called by the ISR.
	//
	void irq( word reading );
};


//
//	Declare the ADC Manager itself.
//
extern ADC_Manager adc_manager;


#endif

//
//	EOF
//

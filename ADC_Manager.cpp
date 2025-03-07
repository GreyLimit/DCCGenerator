//
//	ADC_IO.cpp:	The implementation of the AVR ADC_IO object
//

//
//	Bring in the library elements required.
//
#include "ADC_Manager.h"
#include "Code_Assurance.h"
#include "Memory_Heap.h"
#include "Trace.h"
#include "Stats.h"

#ifdef DEBUGGING_ENABLED
#include "Console.h"
#endif

//
//	This is the routine which initiates a single analogue
//	conversion on the hardware.  This uses none of the
//	internal data structures of this class.
//
void ADC_Manager::start_conversion( byte pin ) {
	//
	//	This is the *basic* ADC conversion which work on both
	//	the ATmega328 and ATmega2560 and supports only the
	//	first 8 sample pins (0..7).
	//
	//	Future hardware will need its own entries in here.
	//

	ADMUX = bit( REFS0 )|( pin & 0x07 );
	ADCSRA |= bit( ADSC )|bit( ADIE );

	//
	//	That is all there is to it.
	//
}

//
//	Constructor, only set up the internal data structures.
//
ADC_Manager::ADC_Manager( void ) {
	_active = NIL( pending_adc );
	_tail = &_active;
	_free = NIL( pending_adc );
}

//
//	Provide the hardware initialisation routine.
//
void ADC_Manager::initialise( void ) {

	STACK_TRACE( "void ADC_Manager::initialise( void )" );

	//
	//	There's nothing to do here that isn't done
	//	when a conversion is requested.
	//
}

//
//	Request that a pin has its voltage read.  When
//	the reading is complete the value obtained is
//	stored at result and the flag is set.
//
bool ADC_Manager::read( byte pin, Signal *flag, word *result ) {

	STACK_TRACE( "bool ADC_Manager::read( byte pin, Signal *flag, word *result )" );

	pending_adc	*ptr;
	bool		initiate;

	TRACE_ADC( console.print( F( "ADC read pin " )));
	TRACE_ADC( console.print( pin ));
	TRACE_ADC( console.print( F( " flag " )));
	TRACE_ADC( console.println( flag->identity()));

	//
	//	Have we got (can we get) a spare pending record.
	//
	if(( ptr = _free )) {
		//
		//	Re-use an old one.
		//
		_free = ptr->next;
	}
	else {
		//
		//	Try to mint a new one.
		//
		if(!( ptr = new pending_adc )) return( false );
	}

	//
	//	Will we need to "kick off" the conversion?
	//
	initiate = ( _active == NIL( pending_adc ));

	//
	//	Fill in the pending record.
	//
	ptr->pin = pin;
	ptr->save = result;
	ptr->flag = flag;
	ptr->next = NIL( pending_adc );
	*_tail = ptr;
	_tail = &( ptr->next );

	ASSERT( _active != NIL( pending_adc ));

	//
	//	Fire off conversion if required.
	//
	if( initiate ) start_conversion( _active->pin );

	//
	//	Success!
	//
	return( true );
}

//
//	The entry point from the task manager.
//
void ADC_Manager::process( UNUSED( byte handle )) {

	STACK_TRACE( "void ADC_Manager::process( byte handle )" );

	pending_adc	*ptr;

	//
	//	Was this a valid conversion?
	//
	if(( ptr = _active ) == NIL( pending_adc )) {
		//
		//	Nope.
		//
		errors.log_error( ADC_UNEXPECTED_RESULT, _reading );
		return;
	}

	TRACE_ADC( console.print( F( "ADC pin " )));
	TRACE_ADC( console.print( _active->pin ));
	TRACE_ADC( console.print( F( " = " )));
	TRACE_ADC( console.println( _reading ));

	//
	//	Remove from the list.
	//
	if(( _active = ptr->next ) == NIL( pending_adc )) _tail = &_active;
	
	//
	//	Save the reading and signal that the data is ready.
	//
	//	This is fast signal as reading the load is a time
	//	sensitive activity.
	//
	*( ptr->save ) = _reading;
	ptr->flag->release( true );

	//
	//	Return to the free list.
	//
	ptr->next = _free;
	_free = ptr;

	//
	//	Kick off the next ADC request.
	//
	if( _active ) start_conversion( _active->pin );
}

//
//	The routine called by the ISR.
//
void ADC_Manager::irq( word reading ) {
	
	STACK_TRACE( "void ADC_Manager::irq( word reading )" );
	
	_reading = reading;
	_irq.release( true );
}

//
//	The memory reclamation API.
//	---------------------------
//
//	Return the number of bytes memory being "cached" and
//	available for release if required.  This is a statistical
//	call to allow tracking of memory usage.
//
size_t ADC_Manager::cache_memory( void ) {
	
	STACK_TRACE( "size_t ADC_Manager::cache_memory( void )" );

	word total = 0;
	for( pending_adc *look = _free; look != NIL( pending_adc ); look = look->next ) total += sizeof( pending_adc );
	return( total );
}

//
//	Tell the object to clear all cached memory and release it
//	to the heap.
//
bool ADC_Manager::clear_cache( void ) {
	
	STACK_TRACE( "bool ADC_Manager::clear_cache( void )" );

	pending_adc	*ptr;
	bool		r;

	r = false;
	while(( ptr = _free )) {
		_free = ptr->next;
		delete ptr;
		r = true;
	}
	return( r );
}

//
//	Ask the object how much memory, as a single block, it
//	would release to satisfy a specified allocation request.
//	Return 0 if this object cannot satisfy the request.
//
size_t ADC_Manager::test_cache( size_t bytes ) {
	
	STACK_TRACE( "size_t ADC_Manager::test_cache( size_t bytes )" );

	if( _free &&( sizeof( pending_adc ) >= bytes )) return( sizeof( pending_adc ));
	return( 0 );
}

//
//	Request that an object release, as a single block,
//	enough memory to cover the specified allocation.
//	Return true on success, false on failure.
//
bool ADC_Manager::release_cache( size_t bytes ) {

	STACK_TRACE( "bool ADC_Manager::release_cache( size_t bytes )" );

	if( _free &&( sizeof( pending_adc ) >= bytes )) {
		pending_adc *ptr;

		ptr = _free;
		_free = ptr->next;
		delete ptr;

		return( true );
	}
	return( false );
}


//
//	Declare the ADC Manager itself.
//
ADC_Manager adc_manager;

//
//	Define the Interrupt Service Routine (ISR) which will
//	read the data and store it.  Restarting another ADC
//	conversion is done elsewhere.
//
ISR( ADC_vect ) {
	byte	high, low;
	
	COUNT_INTERRUPT;

	//
	//	We have to read ADCL first; doing so locks both ADCL
	//	and ADCH until ADCH is read.  reading ADCL second would
	//	cause the results of each conversion to be discarded,
	//	as ADCL and ADCH would be locked when it completed.
	//
	low = ADCL;
	high = ADCH;
	
	//
	//	Trigger the main line processing routine.
	//
	adc_manager.irq( HL_TO_W( high, low ));
}

//
//	EOF
//




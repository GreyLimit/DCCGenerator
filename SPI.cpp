//
//	SPI.h
//	=====
//
//	Provide a generic interface to the SPI driver hardware
//	proivding the "controller" functionality.  There is,
//	currently, no "peripheral" functionality.
//


#include "SPI.h"
#include "Code_Assurance.h"
#include "Memory_Heap.h"
#include "Errors.h"
#include "Stats.h"

#include "Trace.h"
#ifdef DEBUGGING_ENABLED
#include "Console.h"
#endif

//
//	The table itself.  These figures reflect the the 8-bit AVR
//	system hardware.
//
//	Note/	The field rate is only used as a consistency check.
//		Also note that the entries must be increasing order
//		of rate divisor (ie faster clocks are tested before
//		slower clocks).
//
static const SPI_Device::clock_divisor SPI_Device::clock_table[] PROGMEM = {
	//
	//	Rate	Shift,	SPI2X	SPR1	SPR0
	//
	{	2,	1,	true,	false,	false	},	// F(osc)/2
	{	4,	2,	false,	false,	false	},	// F(osc)/4
	{	8,	3,	true,	false,	true	},	// F(osc)/8
	{	16,	4,	false,	false,	true	},	// F(osc)/16
	{	32,	5,	true,	true,	false	},	// F(osc)/32
	{	64,	6,	false,	true,	false	},	// F(osc)/64
	{	128,	7,	false,	true,	true	},	// F(osc)/128
	//
	//	Mark end of table (rate and shift zero).
	//
	{	0,	0,	false,	false,	false	}
};

//
//	Select the clock divisor to be used (with respect to the
//	base clock frequency F(osc).
//
//	Return true on success, false on invalid divisor.
//
bool SPI_Device::set_clock( SPI_Registers *device, word clock ) {

	STACK_TRACE( "bool SPI_Device::set_clock( SPI_Registers *device, word clock )" );
	
	const clock_divisor	*c;
	byte			s;
		
	c = clock_table;
	while(( s = progmem_read_byte( c->shift ))) {

		//
		//	Just a little consistency checking.
		//
		ASSERT(( 1 << s ) == c->rate );

		//
		//	Check to see if the requested clock speed is
		//	greater than the divided clock speed.  If so
		//	then we have found the first configurable speed
		//	slower than the requested speed.
		//
		if(( cpu_hz >> s ) <= clock ) {
			//
			//	Found, so apply the changes.
			//
			device->spi2x((bool)( progmem_read_byte( c->spi2x )));
			device->spr1((bool)( progmem_read_byte( c->spr1 )));
			device->spr0((bool)( progmem_read_byte( c->spr0 )));
			return( true );
		}
		c++;
	}
	return( false );
}

//
//	Start transaction puts the transaction addressed by
//	_active into action.
//
void SPI_Device::start_trans( void ) {

	STACK_TRACE( "void SPI_Device::start_trans( void )" );

	//
	//	This must *never* be called outside an interrupt
	//	safe code section.  This is either an ISR() routine
	//	or a "Critical code" section.
	//
	ASSERT( Critical::critical_code());

	//
	//	Of course we anticipate that there is a transaction
	//	to be started and its target is valid.
	//
	ASSERT( _active != NIL( spi_trans ));

	//
	//	Pointer to the new target.
	//
	register spi_target *q = _active->target;

	//
	//	We now load the SPI hardware with our configuration
	//
	_dev->load( &( q->configuration ));

	//
	//	Now enable the target chip
	//
	q->pin->set( q->enable );

	//
	//	Kick off the transaction by sending a byte of data.
	//
	if( _active->send ) {
		//
		//	We have data to send, which is normal, so we can
		//	simply let the process roll using the normal
		//	exchange process.
		//
		spi_event();
	}
	else {
		//
		//	Here we have a special case where no data is being sent.
		//	We must initiate this by sending an initial packing byte.
		//
		_dev->write( packing_byte );
	}
	
	//
	//	We're off!
	//
}

//
//	Stop transaction end the transaction addressed by
//	_active and initiates a following if available.
//
void SPI_Device::stop_trans( void ) {

	STACK_TRACE( "void SPI_Device::stop_trans( void )" );

	//
	//	This must *never* be called outside an interrupt
	//	safe code section.
	//
	ASSERT( Critical::critical_code());

	//
	//	Of course we anticipate that there is a transaction
	//	to be stopped.
	//
	ASSERT( _active != NIL( spi_trans ));

	//
	//	Disable the target chip SPI interface
	//
	_active->target->pin->set( !_active->target->enable );

	//
	//	Shutdown the SPI interface.
	//
	_dev->clear();
	
	//
	//	Signal the flag that we're done.
	//
	_active->flag->release();

	//
	//	Remove from the active list and add to free list.
	//
	spi_trans *ptr = (spi_trans *)_active;
	
	if(( _active = ptr->next ) == NIL( spi_trans )) _tail = &_active;
	ptr->next = (spi_trans *)_free;
	_free = ptr;

	//
	//	If there is another transaction pending
	//	then we can fire that off now.
	//
	if( _active ) start_trans();
}

//
//	This is the constructor which is passed in the
//	address of the registers for the SPI device to
//	be managed and the interrupt redirection register
//	for the device.
//
SPI_Device::SPI_Device( SPI_Registers *device ) {

	ASSERT( device != NIL( SPI_Registers ));

	//
	//	Save our hardware address and clear targets table.
	//
	_dev = device;
	
	//
	//	Organise the transactions queue.
	//
	_active = NIL( spi_trans );
	_tail = &_active;
	_free = NIL( spi_trans );
	
	//
	//	We are ready to go.
	//
}

//
//	Convert an unsigned long clock speed (in Hz) to the
//	word value used by the new_target routine.
//
word SPI_Device::hz( unsigned long clock ) {

	STACK_TRACE( "word SPI_Device::hz( unsigned long clock )" );

	ASSERT( clock <= KHZ_TO_HZ( MAXIMUM_WORD ));
	ASSERT( clock >= KHZ_TO_HZ( 1 ));

	return( HZ_TO_KHZ( clock ));
}


//
//	Add a new target to the SPI manager.
//
//	speed		The bus speed the device requires in
//			units of 1024 Hz.  This gives a bus
//			speed from (approximately) 1 KHz upto
//			65 MHz.
//
//	lsb		True if the remote device is least
//			significant bit first, false if it is
//			most significant bit first.
//
//	cpol, cpha	Bus signal configuration.
//
//	pin		The pin used to enable the target device
//
//	enable		The "state" to set the pin to enable
//			the target device.
//
//	Returns the address of a control record for this device
//	on success, or NIL if there was an issue with the definition.
//
//	This is dynamically created and can be released if not required
//	again (unliekly)
//
SPI_Device::spi_target *SPI_Device::new_target( word speed, bool lsb, bool cpol, bool cpha, Pin_IO *pin, bool enable ) {

	STACK_TRACE( "SPI_Device::spi_target *SPI_Device::new_target( word speed, bool lsb, bool cpol, bool cpha, Pin_IO *pin, bool enable )" );
	
	ASSERT( pin != NIL( Pin_IO ));

	spi_target	*target;
	SPI_Registers	configuration;

	//
	//	Set the clock speed/divisor
	//
	if( !set_clock( &configuration, speed )) {
		errors.log_error( SPI_INVALID_CLOCK_SPEED, speed );
		return( NIL( spi_target ));
	}

	//
	//	Set bit ordering.
	//
	configuration.dord( lsb );

	//
	//	Set up clock characteristics...
	//
	configuration.cpol( cpol );
	configuration.cpha( cpha );

	//
	//	Finally we pre-set the interrupt and SPI enable bits
	//	so that loading the configuration automatically starts
	//	the SPI interface.
	//
	configuration.spie( true );
	configuration.spe( true );

	//
	//	Create new target record and copy in the configuration.
	//
	if(!( target = new spi_target )) return( NIL( spi_target ));

	//
	//	Save the connection details.
	//	Save pin chip enable details and perform initial
	//	chip disable.
	//
	target->configuration.load( &configuration );
	target->pin = pin;
	target->enable = enable;
	pin->output();
	pin->set( !enable );
	
	//
	//	Done now.
	//
	return( target );
}

//
//	Exchange data with a registered target.  Returns true
//	if the request is queued, false otherwise.
//
bool SPI_Device::exchange( SPI_Device::spi_target *target, byte send, byte recv, byte *buffer, Signal *flag ) {

	STACK_TRACE( "bool SPI_Device::exchange( SPI_Device::spi_target *target, byte send, byte recv, byte *buffer, Signal *flag )" );

	spi_trans	*ptr;

	ASSERT( flag != NIL( Signal ));
	ASSERT( buffer != NIL( byte ));
	ASSERT( target != NIL( spi_target ));

	//
	//	Get a free record that we can build the request into.
	//
	{
		Critical code;

		//
		//	Critical code because the ISR also modifies
		//	this data.
		//
		if(( ptr = (spi_trans *)_free )) _free = ptr->next;
	}
	if( ptr == NIL( spi_trans )) {
		if(!( ptr = new spi_trans )) return( false );
	}

	//
	//	Fill in the record...
	//
	ptr->target = target;
	ptr->send = send;
	ptr->recv = recv;
	ptr->sending = ptr->receiving = buffer;
	ptr->flag = flag;
	ptr->next = NIL( spi_trans );

	//
	//	Now append to the end of the queue.
	//
	{
		Critical code;

		//
		//	Link this into the queue.
		//
		*_tail = ptr;
		_tail = (volatile spi_trans **)&( ptr->next );

		//
		//	Now, if the queue *was* empty then we need to
		//	nudge the SPI system into performing it.
		//
		if( _active == ptr ) start_trans();
	}

	//
	//	Done!
	//
	return( true );
}

//
//	Exchange a single byte with the attached device.
//
void SPI_Device::spi_event( void ) {

	STACK_TRACE( "void SPI_Device::spi_event( void )" );

	//
	//	This must *never* be called outside an interrupt
	//	safe code section.  This is either an ISR() routine
	//	or a "Critical code" section.
	//
	ASSERT( Critical::critical_code());

	//
	//	Data left to send?
	//
	if( _active->send ) {
		//
		//	Decrement counter and send the next byte.
		//
		_active->send--;
		_dev->write( *( _active->sending++ ));

		//
		//	A byte of data has been sent; there is nothing
		//	more to do until this routine is called again
		//	by the ISR handler.
		//
	}
	else {
		//
		//	Data left to retrieve?
		//
		if( _active->recv ) {
			//
			//	decrement counter and read the last byte
			//	received by the SPI hardware.
			//
			_active->recv--;
			*( _active->receiving++ ) = _dev->read();

			//
			//	If there more to be done?
			//
			if( _active->recv ) {
				//
				//	More data the collect? We need
				//	to push out a packing byte to
				//	drive the shift registers.
				//
				_dev->write( packing_byte );
			}
			else {
				//
				//	No, we have gathered the last byte
				//	of data from the remote device.
				//	The transaction is complete.
				//
				stop_trans();
			}
		}
		else {
			//
			//	Getting here with both send and recv
			//	counts empty means we are done.
			//
			stop_trans();
		}
	}
}


//
//	Define our device handler.
//
SPI_Device spi((SPI_Registers *)0x004C );

//
//	The ISR routine
//
ISR( SPI_STC_vect ) {
	COUNT_INTERRUPT;
	spi.spi_event();
}

//
//	EOF
//



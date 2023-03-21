//
//	The implementation of the AVR USART IO
//

//
//	Bring in definitions.
//
#include <Arduino.h>

#include "Environment.h"
#include "Errors.h"
#include "Library_Types.h"
#include "Critical.h"
#include "Code_Assurance.h"
#include "USART.h"

//
//	Specify the constructor to provide the details
//	of the hardware being accessed.
//
USART_Device::USART_Device( USART_Registers *dev, USART_IO **vec ) {
	 _dev = dev;
	 _vec = vec;
	 clear();
}
void USART_Device::clear( void ) {
	Critical code;
	
	_dev->clear();
}
void USART_Device::enable( bool run ) {
	if( run ) {
		_dev->enable_tx_rx();
		_dev->enable_rx_irq();
	}
	else {
		_dev->disable_tx_rx();
		_dev->disable_rx_irq();
	}
}
void USART_Device::bits( USART_char_size size ) {
	switch( size ) {
		case CS5: {
			_dev->set_charsize( 5 );
			break;
		}
		case CS6: {
			_dev->set_charsize( 6 );
			break;
		}
		case CS7: {
			_dev->set_charsize( 7 );
			break;
		}
		default: {
			_dev->set_charsize( 8 );
			break;
		}
	}
}
void USART_Device::parity( USART_data_parity parity ) {
	switch( parity ) {
		case PEven: {
			_dev->parity_even();
			break;
		}
		case POdd: {
			_dev->parity_odd();
			break;
		}
		default: {
			_dev->parity_off();
			break;
		}
	}
}
void USART_Device::stopbits( USART_stop_bits bits ) {
	if( bits == SBTwo ) {
		_dev->two_stopbits();
	}
	else {
		_dev->one_stopbit();
	}
}
bool USART_Device::baud( USART_line_speed speed ) {
	const speed_setting	*ss;
	USART_line_speed	found;
	word			ticks;
	
	ss = _configuration;
	while(( found = (USART_line_speed)progmem_read_byte( ss->speed )) != B_EOT ) {
		if( found == speed ) break;
		ss++;
	}
	//
	//	If we did not find the right record
	//	return false indicating so.
	//
	if( found == B_EOT ) return( false );
	//
	//	How many ticks do we need?
	//
	ticks = progmem_read_word( ss->ticks );
	
	//
	//	The value in ticks is the raw CPU clock speed
	//	divided by the target
	//
	if( ticks <= 0x0FFF ) {
		//
		//	The "higher speed" clock (x2 set)
		//	fits inside a 12 bit value, so we
		//	just use the value as found.
		//
		//	We remember to subtract 1 from the
		//	tick count (as per data sheet).
		//
		ticks -= 1;
		_dev->set_baud_x2();
		_dev->set_baud_h( W_TO_H( ticks ));
		_dev->set_baud_l( W_TO_L( ticks ));
	}
	else {
		//
		//	The tick count is too big to fit
		//	inside our 12 bit range, so reduce
		//	it again and do not set the clock
		//	doubling facility.
		//
		//	We round down if this is still too
		//	large.  This means we are trying to use
		//	a baud rate way too slow for the
		//	CPU speed.
		//
		//	Still need to deduct 1.
		//
		if(( ticks >>= 1 ) > 0x0FFF ) ticks = 0x0FFF;
		ticks -= 1;
		_dev->set_baud_h( W_TO_H( ticks ));
		_dev->set_baud_l( W_TO_L( ticks ));
	}
	return( true );
}
void USART_Device::dre_irq( bool enable ) {
	if( enable ) {
		_dev->enable_dre_irq();
	}
	else {
		_dev->disable_dre_irq();
	}
}
void USART_Device::attach_io( USART_IO *io ) {
	*_vec = io;
	_dev->disable_dre_irq();
	_dev->enable_rx_irq();
	_dev->enable_tx_rx();
}
void USART_Device::dettach_io( void ) {
	_dev->disable_tx_rx();
	_dev->disable_rx_irq();
	_dev->disable_dre_irq();
	*_vec = NULL;
}
void USART_Device::write( byte value ) {
	_dev->data_write( value );
}
byte USART_Device::read( void ) {
	return( _dev->data_read());
}


//////////////////////////////////////////////////
//						//
//	Arduino Uno and Nano			//
//	=====================			//
//						//
//////////////////////////////////////////////////
#if defined( ARDUINO_AVR_UNO ) || defined( ARDUINO_AVR_NANO )

//
//	Declare how many USARTs these devices have
//
static const byte usart_devices = 1;

//
//	Interrupt Vectors taken over by this module:
//
//		USART_RX_vect(_num)		Serial hardware receive data ready
//		USART_UDRE_vect(_num)		Serial hardware send buffer empty
//

static USART_IO *usart0_vector;

ISR( USART_RX_vect ) { if( usart0_vector ) usart0_vector->input_ready(); }
ISR( USART_UDRE_vect ) { if( usart0_vector ) usart0_vector->output_ready(); }

//
//	Declare the only USART the Uno/Nano has.
//
static USART_Device usart0( (USART_Registers *)0x00C0, &usart0_vector );

//
//	Define the array of pointers to drivers.
//
static USART_Device *usart[ usart_devices ] = { &usart0 };


//////////////////////////////////////////////////
//						//
//	Arduino Mega2560			//
//	================			//
//						//
//////////////////////////////////////////////////
#elif defined( ARDUINO_AVR_MEGA2560 )

//
//	Declare how many USARTs these devices have
//
static const byte usart_devices = 4;

//
//	Interrupt Vectors taken over by this module:
//
//		USART0_UDRE_vect(_num)		Serial hardware 0 receive data ready
//		USART0_RX_vect(_num)		Serial hardware 0 send buffer empty
//		USART1_UDRE_vect(_num)		Serial hardware 1 receive data ready
//		USART1_RX_vect(_num)		Serial hardware 1 send buffer empty
//		USART2_UDRE_vect(_num)		Serial hardware 2 receive data ready
//		USART2_RX_vect(_num)		Serial hardware 2 send buffer empty
//		USART3_UDRE_vect(_num)		Serial hardware 3 receive data ready
//		USART3_RX_vect(_num)		Serial hardware 3 send buffer empty
//

static USART_IO *usart0_vector;
static USART_IO *usart1_vector;
static USART_IO *usart2_vector;
static USART_IO *usart3_vector;

ISR( USART0_RX_vect ) { if( usart0_vector ) usart0_vector->input_ready(); }
ISR( USART0_UDRE_vect ) { if( usart0_vector ) usart0_vector->output_ready(); }
ISR( USART1_RX_vect ) { if( usart1_vector ) usart1_vector->input_ready(); }
ISR( USART1_UDRE_vect ) { if( usart1_vector ) usart1_vector->output_ready(); }
ISR( USART2_RX_vect ) { if( usart2_vector ) usart2_vector->input_ready(); }
ISR( USART2_UDRE_vect ) { if( usart2_vector ) usart2_vector->output_ready(); }
ISR( USART3_RX_vect ) { if( usart3_vector ) usart3_vector->input_ready(); }
ISR( USART3_UDRE_vect ) { if( usart3_vector ) usart3_vector->output_ready(); }

//
//	Declare the Mega2560 USARTs.
//
static USART_Device usart0( (USART_Registers *)0x00C0, &usart0_vector );
static USART_Device usart1( (USART_Registers *)0x00C8, &usart1_vector );
static USART_Device usart2( (USART_Registers *)0x00D0, &usart2_vector );
static USART_Device usart3( (USART_Registers *)0x0130, &usart3_vector );

//
//	Define the array of pointers to drivers.
//
static USART_Device *usart[ usart_devices ] = { &usart0, &usart1, &usart2, &usart3 };

#else
#error "Specific AVR Board not recognised (definitions required)"
#endif

//
//	The following macro calculate the initial
//	"ticks count per bit" using the CPU frequency
//	and the target baudrate.
//
//	This value has an initial divide by 8 applied
//	(associated with X2 being set to 1), but a further
//	division by 2 can be applied (is association with
//	X2 being set to 0).
//
#define BAUD_TICKS(b)	((F_CPU/(b))>>3)

//
//	Calculate the speed table we work against.
//
const USART_Device::speed_setting USART_Device::_configuration[] PROGMEM = {
	{	B300,		BAUD_TICKS( 300 )	},
	{	B600,		BAUD_TICKS( 600 )	},
	{	B2400,		BAUD_TICKS( 2400 )	},
	{	B4800,		BAUD_TICKS( 4800 )	},
	{	B9600,		BAUD_TICKS( 9600 )	},
	{	B14400,		BAUD_TICKS( 14400 )	},
	{	B19200,		BAUD_TICKS( 19200 )	},
	{	B28800,		BAUD_TICKS( 28800 )	},
	{	B38400,		BAUD_TICKS( 38400 )	},
	{	B57600,		BAUD_TICKS( 57600 )	},
	{	B115200,	BAUD_TICKS( 115200 )	},
	{	B_EOT,		0			}
};

//////////////////////////////////////////
//					//
//	USART_IO			//
//	========			//
//					//
//////////////////////////////////////////


//
//	Define the initialiser
//
USART_IO::USART_IO( void ) {
	_dev = NULL;
	_input = NULL;
	_output = NULL;
	_async = false;
}

//
//	Define the initialiser
//
bool USART_IO::initialise( byte inst, USART_line_speed speed, USART_char_size bits, USART_data_parity parity, USART_stop_bits sbits, Byte_Queue_API *in_queue, Byte_Queue_API *out_queue ) {

	Critical code;

	//
	//	Verify we have been asked to use an existent
	//	unused piece of hardware, note the address
	//	of its hardware interface.
	//
	if( inst >= usart_devices ) return( false );
	_dev = usart[ inst ];
	
	//
	//	Save IO buffer addresses
	//
	_input = in_queue;
	_output = out_queue;

	//
	//	Set up the hardware as requested:
	//
	_dev->clear();
	_dev->baud( speed );
	_dev->bits( bits );
	_dev->parity( parity );
	_dev->stopbits( sbits );

	//
	//
	//	Note that we are, currently, not asynchronously sending
	//	any data.
	//
	_async = false;

	//
	//	Attach this object to the interrupts.
	//
	_dev->attach_io( this );

	//
	//	Done.
	//
	return( true );
}

//
//	The Byte Queue API
//	==================

//
//	byte available( void )
//	----------------------
//
//	Return number of bytes queued ready to
//	be read.
//
data_size USART_IO::available( void ) {
	return( _input->available());
}

//
//	byte space( void )
//	------------------
//
//	Return the number of bytes which can be
//	added to the output queue for writing.
//
data_size USART_IO::space( void ) {
	return( _output->space());
}

//
//	byte read( void )
//	-----------------
//
//	Return next byte from the serial connection.
//
//	If there is no data available this will return
//	the nul byte, ascii 0x00, '\0'.
//
byte USART_IO::read( void ) {
	return( _input->read());
}

//
//	void write( byte data )
//	-----------------------
//
//	Write the supplied data to the output queue and
//	send it when possible.
//
//	Returns true if queued, false otherwise.
//
bool USART_IO::write( byte data ) {
	//
	//	Add to the output queue..
	//
	if( _output->write( data )) {
		//
		//	then, if the async is false we kick off
		//	the data register empty interrupts.
		//
		if( !_async ) {
			//
			//	This should immediately cause an
			//	interrupt.
			//
			_async = true;
			_dev->dre_irq( true );
		}
		return( true );
	}
	return( false );
}


//
//	Interrupts
//	==========
//
void USART_IO::input_ready( void ) { 
	if( !_input->write( _dev->read())) errors.log_error( USART_IO_ERR_DROPPED, 0 );
}

void USART_IO::output_ready( void ) {
	if( _output->available()) {
		//
		//	We have data, so send it.
		//
		_dev->write( _output->read());
	}
	else {
		//
		//	Nothing to send, so shutdown the send IRQ
		//	and mark async as false.
		//
		_dev->dre_irq( false );
		_async = false;
	}
}

//
//	EOF
//

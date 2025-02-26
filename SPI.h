//
//	SPI.h
//	=====
//
//	Provide a generic interface to the SPI driver hardware
//	proivding the "controller" functionality.  There is,
//	currently, no "peripheral" functionality.
//

#ifndef _SPI_H_
#define _SPI_H_

#include "Environment.h"
#include "Parameters.h"
#include "Configuration.h"
#include "Signal.h"
#include "Pin_IO.h"

//
//	Define a class to map onto the SPI hardware registers
//	with a set of routines for their manipulation.
//
class SPI_Registers {
private:
	//
	//	These are the registers for the AVR 8 bit CPU
	//
	//	SPCR 0x2C(0x4C) SPI Configuration Register
	//	==========================================
	//
	//	Bit	Name	Description		0	1
	//	---	----	-----------		-	-
	//	7	SPIE	SPI Interrupt enable	Disable	Enable
	//
	//	6	SPE	SPI Device Enable	Disable	Enable
	//
	//	5	DORD	Data Order		MSB	LSB
	//
	//	4	MSTR	Select Master/Slave	Slave	Master
	//			(Controller/Peripheral)
	//
	//	3	CPOL	Clock idle polarity	Low	High
	//
	//			Low: Leading edge rising, trailing edge falling
	//			High: Leading edge falling, trailing edge rising
	//
	//	2	CPHA	Clock sample phase	Leading	Trailing
	//
	//	1	SPR1	Clock rate select works with SPI2X, see
	//	0	SPR0	details listed with SPI2X.
	//
	//	SPSR 0x2D(0x4D) SPI Status Register
	//	===================================
	//
	//	Bit	Name	Description		0	1
	//	---	----	-----------		-	-
	//	7	SPIF	Transfer completed	Active	Complete
	//
	//	6	WCOL	Write collision flag	Clear	Collision
	//
	//	5-1	Reserved
	//
	//	0	SPI2X	Double SPI Speed	Normal	Doubled
	//
	//			SPI2X	SPR1	SPR0	SCK Frequency
	//			1	0	0	F(osc)/2
	//			0	0	0	F(osc)/4
	//			1	0	1	F(osc)/8
	//			0	0	1	F(osc)/16
	//			1	1	0	F(osc)/32
	//			0	1	0	F(osc)/64
	//			1	1	1	F(osc)/64 - duplicate.
	//			0	1	1	F(osc)/128
	//
	//	SPDR 0x2E(0x4E) SPI Data Register
	//	=================================
	//
	//	7-0	SPDR	SPI Data Register
	//
	volatile byte		_spcr,
				_spsr,
				_spdr;

public:
	//
	//	Constructor clears the registers safely.
	//
	SPI_Registers( void ) {
		_spcr = 0;
		_spsr = 0;
	}
	
	//
	//	Device Control routines
	//	=======================
	//
	//	The following routines provide access to the device
	//	registers in a controlled manner (loading registers
	//	in the correct order).
	//
	//	These routines are to be used against the physical
	//	device registers controlling the hardware.
	//
	//	void clear( void );
	//	-------------------
	//
	//	Reset the device to empty and inactive.
	//
	inline void clear( void ) {
		_spcr = 0;
		_spsr = 0;
	}
	
	//
	//	void load( SPI_Registers *src );
	//	-----------------------------
	//
	//	Load a device configuration from the record provided
	//	to initialise a device for use.  Do not use to clear
	//	a device; the regsiters are loaded in the wrong order.
	//
	inline void load( SPI_Registers *src ) {
		_spsr = src->_spsr;
		_spcr = src->_spcr;
	}

	//
	//	void write( byte data );
	//	------------------------
	//
	//	Send data through the SPI device.
	//
	inline void write( byte data ) {
		_spdr = data;
	}

	//
	//	byte read( void );
	//	------------------
	//
	//	Read data from the SPI device.
	//
	inline byte read( void ) {
		return( _spdr );
	}

	//
	//	Test the status of the current action.  Return
	//	true on complete, false otherwise.
	//
	inline bool spif( void ) {
		return( BOOL( _spsr & bit( SPIF )));
	}

	//
	//	Test the "Write Collision" flag.  Return
	//	true on a data write collision detected,
	//	false otherwise.
	//
	inline bool wcol( void ) {
		return( BOOL( _spsr & bit( WCOL )));
	}
	
	//
	//	Device Configuration routines
	//	=============================
	//
	//	The following routines enable a set of device registers
	//	that are NOT attached to a piece of hardware to be
	//	configured in a desired manner.
	//
	//	These routines are to be used to create a "template" set
	//	of device registers which can then be "loaded" onto the
	//	target device using the "load()" routine above.
	//

	//
	//	Routines for setting/resetting the elements of the SPI
	//	device registers.  The routines are named as per the
	//	descriptions above.
	//

	//
	//	Enable (on:true) or disable (on:false) the interrupt bit.
	//	When enabled this will cause the SPI interrupt to be requested
	//	at the completion of a transfer (or in event of an error
	//	condition).
	//	
	inline void spie( bool on ) {
		if( on ) {
			_spcr |= bit( SPIE );
		}
		else {
			_spcr &= ~bit( SPIE );
		}
	}

	//
	//	Enable (on:true) or disable (on:false) the whole SPI
	//	device.
	//
	inline void spe( bool on ) {
		if( on ) {
			_spcr |= bit( SPE );
		}
		else {
			_spcr &= ~bit( SPE );
		}
	}

	//
	//	Set the bit data order on the serial bus.  LSB (least
	//	significant bit first) true, or MSB (most significant
	//	bit first) false.
	//
	inline void dord( bool lsb ) {
		if( lsb ) {
			_spcr |= bit( DORD );
		}
		else {
			_spcr &= ~bit( DORD );
		}
	}

	//
	//	Set SPI device to operate in Master/Controller mode
	//	(master:true) or in Slave/Peripheral mode (master:false).
	//
	inline void mstr( bool master ) {
		if( master ) {
			_spcr |= bit( MSTR );
		}
		else {
			_spcr &= ~bit( MSTR );
		}
	}

	//
	//	Set Clock idle Polarity, the state the clock is in when
	//	"inactive".  Set high true (for idle high) or false (for
	//	idle low).
	//
	//	Note that this defines the where the "leading" and
	//	"trailing" edges of the clock signal can be found.  This
	//	is pertinent to the interpretation of the CHPA setting.
	//
	//	high:true	Idle high	Leading edge falling,
	//					trailing edge rising.
	//	high:false	Idle low	Leading edge rising,
	//					trailing edge falling.
	//
	inline void cpol( bool high ) {
		if( high ) {
			_spcr |= bit( CPOL );
		}
		else {
			_spcr &= ~bit( CPOL );
		}
	}

	//
	//	Set the sampling position with respect to the clock
	//	configuration.  With trailing true the data should be
	//	sampled on the trailing edge of the clock pulse.  With
	//	trailing false then the data should be sampled on the
	//	leading edge of the clock pulse.
	//
	inline void cpha( bool trailing ) {
		if( trailing ) {
			_spcr |= bit( CPHA );
		}
		else {
			_spcr &= ~bit( CPHA );
		}
	}

	//
	//	Routines used to set up the individual clock divisor
	//	bits.
	//
	inline void spi2x( bool on ) {
		if( on ) {
			_spsr |= bit( SPI2X );
		}
		else {
			_spsr &= ~bit( SPI2X );
		}
	}
	inline void spr1( bool on ) {
		if( on ) {
			_spcr |= bit( SPR1 );
		}
		else {
			_spcr &= ~bit( SPR1 );
		}
	}
	inline void spr0( bool on ) {
		if( on ) {
			_spcr |= bit( SPR0 );
		}
		else {
			_spcr &= ~bit( SPR0 );
		}
	}

};

//
//	Define here a couple of macros which will deal with the
//	transition to and from the modules "KHz" values.
//
#define HZ_TO_KHZ(h)	(word)((h)>>10)
#define KHZ_TO_HZ(h)	((unsigned long)(h)<<10)

//
//	Define the SPI manager object.  This is the object which is
//	associated with a single SPI hardware driver and controls
//	multiple IO requests through it.
//
class SPI_Device {
public:
	//
	//	Define a structure to hold the data for a specific
	//	target device.
	//
	struct spi_target {
		//
		//	This will be the template which is loaded into
		//	SPI hardware to configure it for this device.
		//
		SPI_Registers	configuration;
		//
		//	The pin used to enable the target chip and the
		//	value required to enable it (typically this are
		//	active low connections, so this would be false).
		Pin_IO		*pin;
		bool		enable;
	};

private:
	//
	//	Declare a static constant which is our base clock
	//	speed expressed in the KHz range used by this module.
	//
	static constexpr word cpu_hz = HZ_TO_KHZ( F_CPU );

	//
	//	Define and declare the clock rate table to provide a
	//	simple mechanism for checking and setting the clock
	//	divisor parameters.
	//
	struct clock_divisor {
		byte		rate, shift;
		bool		spi2x, spr1, spr0;
	};

	//
	//	The table itself.
	//
	static const clock_divisor clock_table[] PROGMEM;
	
	//
	//	Record the address of the SPI Device Registers.
	//
	SPI_Registers	*_dev;

	//
	//	Define the "packing byte" value used when empty data
	//	needs to be transmitted to facilitate the reading of
	//	a reply from the SPI attached device.
	//
	static const byte packing_byte = 0xff;

	//
	//	Define a class to hold a single activity queued or
	//	active.
	//
	struct spi_trans {
		spi_target	*target;	// The target to use.
		byte		send,		// Byte to send to the target.
				recv,		// Bytes to receive from the target.
				*sending,	// Where we read data from..
				*receiving;	// and write it to.
		Signal		*flag;		// Signal to release when completed.
		spi_trans	*next;		// Link to next request.
	};

	//
	//	Here are the pointers to the various queues.
	//
	volatile spi_trans	*_free,
				*_active,
				**_tail;

	//
	//	Select the clock divisor to be used (with respect to the
	//	base clock frequency F(osc).
	//
	//	Return true on success, false on invalid divisor.
	//
	bool set_clock( SPI_Registers *device, word clock );

	//
	//	Start transaction puts the transaction addressed by
	//	_active into action.
	//
	void start_trans( void );

	//
	//	Stop transaction end the transaction addressed by
	//	_active and initiates a following if available.
	//
	void stop_trans( void );

public:
	//
	//	This is the constructor which is passed in the
	//	address of the registers for the SPI device to
	//	be managed and the interrupt redirection register
	//	for the device.
	//
	SPI_Device( SPI_Registers *device );

	//
	//	Convert an unsigned long clock speed (in Hz) to the
	//	word value used by the new_target routine.
	//
	static word hz( unsigned long clock );

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
	spi_target *new_target( word speed, bool lsb, bool cpol, bool cpha, Pin_IO *pin, bool enable );

	//
	//	Exchange data with a registered target.  Returns true
	//	if the request is queued, false otherwise.
	//
	bool exchange( spi_target *target, byte send, byte recv, byte *buffer, Signal *flag );

	//
	//	Exchange a single byte with the attached device.
	//
	void spi_event( void );
	
};

//
//	Define our device handler.
//
extern SPI_Device spi;

#endif


//
//	EOF
//

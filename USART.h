//
//	USART
//	=====
//
//	Define a class which implements access to and from
//	any AVR MCU USART hardware.
//
//

#ifndef _USART_H_
#define _USART_H_

//
//	We will need:
//
#include "Environment.h"
#include "Parameters.h"
#include "Configuration.h"
#include "Byte_Queue.h"

//////////////////////////////////////////////////////////
//							//
//	Declare USART general enumerations.		//
//							//
//////////////////////////////////////////////////////////
typedef enum {
	B300,	B600,	B1200,	B2400,
	B4800,	B9600,	B14400,	B19200,
	B28800,	B38400,	B57600,	B115200,
	B_EOT
} USART_line_speed;
typedef enum {
	CS5	= 5,
	CS6	= 6,
	CS7	= 7,
	CS8	= 8
} USART_char_size;
typedef enum {
	PNone	= 0,
	POdd	= 1,
	PEven	= 2
} USART_data_parity;
typedef enum {
	SBOne	= 1,
	SBTwo	= 2
} USART_stop_bits;

//////////////////////////////////////////////////////////
//							//
//	Basic AVR Architecture: Uno, Mega2560 etc	//
//	=========================================	//
//							//
//////////////////////////////////////////////////////////
class USART_Registers {
private:
	//
	//	The AVR USART Registers are as follows:
	//
	//	UDRn – USART I/O Data Register n
	//	====
	//
	//			Bit	7 6 5 4 3 2 1 0
	//	RXB[7:0] UDRn (Read)	[Byte received]
	//	TXB[7:0] UDRn (Write)	[Byte to send ]
	//
	//
	//	UCSRnA – USART Control and Status Register n A
	//	======
	//
	//	Bit	7	6	5	4	3	2	1	0
	//		RXCn	TXCn	UDREn	FEn	DORn	UPEn	U2Xn	MPCMn
	//	Read/
	//	Write:	R	R/W	R	R	R	R	R/W	R/W
	//	Init
	//	Value:	0	0	1	0	0	0	0	0
	//
	//
	//	Bit 7 – RXCn: USART Receive Complete
	//
	static const byte RXCn		= 7;
	//
	//	This flag bit is set when there is unread data in the receive buffer and cleared when the receive buffer is empty (i.e. it does
	//	not contain any unread data). If the receiver is disabled, the receive buffer will be flushed and consequently the RXCn bit will
	//	become zero. The RXCn flag can be used to generate a receive complete interrupt (see description of the RXCIEn bit).
	//
	//	Bit 6 – TXCn: USART Transmit Complete
	//
	static const byte TXCn		= 6;
	//
	//	This flag bit is set when the entire frame in the transmit shift register has been shifted out and there are no new data currently
	//	present in the transmit buffer (UDRn). The TXCn flag bit is automatically cleared when a transmit complete interrupt is
	//	executed, or it can be cleared by writing a one to its bit location. The TXCn flag can generate a transmit complete interrupt
	//	(see description of the TXCIEn bit).
	//
	//	Bit 5 – UDREn: USART Data Register Empty
	//
	static const byte UDREn		= 5;
	//
	//	The UDREn flag indicates if the transmit buffer (UDRn) is ready to receive new data. If UDREn is one, the buffer is empty,
	//	and therefore ready to be written. The UDREn flag can generate a data register empty interrupt (see description of the
	//	UDRIEn bit). UDREn is set after a reset to indicate that the transmitter is ready.
	//
	//	Bit 4 – FEn: Frame Error
	//
	static const byte FEn		= 4;
	//
	//	This bit is set if the next character in the receive buffer had a frame error when received. I.e., when the first stop bit of the
	//	next character in the receive buffer is zero. This bit is valid until the receive buffer (UDRn) is read. The FEn bit is zero when
	//	the stop bit of received data is one. Always set this bit to zero when writing to UCSRnA.
	//
	//	Bit 3 – DORn: Data OverRun
	//
	static const byte DORn		= 3;
	//
	//	This bit is set if a data overrun condition is detected. A data overrun occurs when the receive buffer is full (two characters), it
	//	is a new character waiting in the receive shift register, and a new start bit is detected. This bit is valid until the receive buffer
	//	(UDRn) is read. Always set this bit to zero when writing to UCSRnA.
	//
	//	Bit 2 – UPEn: USART Parity Error
	//
	static const byte UPEn		= 2;
	//
	//	This bit is set if the next character in the receive buffer had a parity error when received and the parity checking was enabled
	//	at that point (UPMn1 = 1). This bit is valid until the receive buffer (UDRn) is read. Always set this bit to zero when writing to
	//	UCSRnA.
	//
	//	Bit 1 – U2Xn: Double the USART Transmission Speed
	//
	static const byte U2Xn		= 1;
	//
	//	This bit only has effect for the asynchronous operation. Write this bit to zero when using synchronous operation.
	//	Writing this bit to one will reduce the divisor of the baud rate divider from 16 to 8 effectively doubling the transfer rate for
	//	asynchronous communication.
	//
	//	Bit 0 – MPCMn: Multi-processor Communication Mode
	//
	static const byte MPCMn		= 0;
	//
	//	This bit enables the multi-processor communication mode. When the MPCMn bit is written to one, all the incoming frames
	//	received by the USART receiver that do not contain address information will be ignored. The transmitter is unaffected by the
	//	MPCMn setting. For more detailed information see Section 19.9 “Multi-processor Communication Mode” on page 158.
	//
	//
	//	UCSRnB – USART Control and Status Register n B
	//	======
	//
	//	Bit	7	6	5	4	3	2	1	0
	//		RXCIEn	TXCIEn	UDRIEn	RXENn	TXENn	UCSZn2	RXB8n	TXB8n
	//	Read/
	//	Write:	R/W	R/W	R/W	R/W	R/W	R/W	R	R/W
	//	Init
	//	Value:	0	0	0	0	0	0	0	0
	//
	//
	//	Bit 7 – RXCIEn: RX Complete Interrupt Enable n
	//
	static const byte RXCIEn	= 7;
	//
	//	Writing this bit to one enables interrupt on the RXCn flag. A USART receive complete interrupt will be generated only if the
	//	RXCIEn bit is written to one, the global interrupt flag in SREG is written to one and the RXCn bit in UCSRnA is set.
	//
	//	Bit 6 – TXCIEn: TX Complete Interrupt Enable n
	//
	static const byte TXCIEn	= 6;
	//
	//	Writing this bit to one enables interrupt on the TXCn flag. A USART transmit complete interrupt will be generated only if the
	//	TXCIEn bit is written to one, the global interrupt flag in SREG is written to one and the TXCn bit in UCSRnA is set.
	//
	//	Bit 5 – UDRIEn: USART Data Register Empty Interrupt Enable n
	//
	static const byte UDRIEn	= 5;
	//
	//	Writing this bit to one enables interrupt on the UDREn flag. A data register empty interrupt will be generated only if the
	//	UDRIEn bit is written to one, the global interrupt flag in SREG is written to one and the UDREn bit in UCSRnA is set.
	//
	//	Bit 4 – RXENn: Receiver Enable n
	//
	static const byte RXENn		= 4;
	//
	//	Writing this bit to one enables the USART receiver. The receiver will override normal port operation for the RxDn pin when
	//	enabled. Disabling the receiver will flush the receive buffer invalidating the FEn, DORn, and UPEn flags.
	//
	//	Bit 3 – TXENn: Transmitter Enable n
	//
	static const byte TXENn		= 3;
	//
	//	Writing this bit to one enables the USART transmitter. The transmitter will override normal port operation for the TxDn pin
	//	when enabled. The disabling of the transmitter (writing TXENn to zero) will not become effective until ongoing and pending
	//	transmissions are completed, i.e., when the transmit shift register and transmit buffer register do not contain data to be
	//	transmitted. When disabled, the transmitter will no longer override the TxDn port.
	//
	//	Bit 2 – UCSZn2: Character Size n
	//
	static const byte UCSZn2	= 2;
	//
	//	The UCSZn2 bits combined with the UCSZn1:0 bit in UCSRnC sets the number of data bits (character size) in a frame the
	//	receiver and transmitter use.
	//
	//	Bit 1 – RXB8n: Receive Data Bit 8 n
	//
	static const byte RXB8n		= 1;
	//
	//	RXB8n is the ninth data bit of the received character when operating with serial frames with nine data bits. Must be read
	//	before reading the low bits from UDRn.
	//
	//	Bit 0 – TXB8n: Transmit Data Bit 8 n
	//
	static const byte TXB8n		= 0;
	//
	//	TXB8n is the ninth data bit in the character to be transmitted when operating with serial frames with nine data bits. Must be
	//	written before writing the low bits to UDRn.
	//
	//
	//	UCSRnC – USART Control and Status Register n C
	//	======
	//
	//	Bit	7	6	5	4	3	2	1	0
	//		UMSELn1	UMSELn0	UPMn1	UPMn0	USBSn	UCSZn1	UCSZn0	UCPOLn
	//	Read/
	//	Write:	R/W	R/W	R/W	R/W	R/W	R/W	R/W	R/W
	//	Init
	//	Value:	0	0	0	0	0	1	1	0
	//
	//
	//	Bits 7:6 – UMSELn1:0 USART Mode Select
	//
	static const byte UMSELn1	= 7;
	static const byte UMSELn0	= 6;
	//
	//	These bits select the mode of operation of the USARTn.
	//
	//		UMSELn1	UMSELn0	Mode
	//		0	0	Asynchronous USART
	//		0	1	Synchronous USART
	//		1	0	(Reserved)
	//		1	1	Master SPI (MSPIM)
	//
	//	Bits 5:4 – UPMn1:0: Parity Mode
	//
	static const byte UPMn1		= 5;
	static const byte UPMn0		= 4;
	//
	//	These bits enable and set type of parity generation and check. If enabled, the transmitter will automatically generate and
	//	send the parity of the transmitted data bits within each frame. The receiver will generate a parity value for the incoming data
	//	and compare it to the UPMn setting. If a mismatch is detected, the UPEn flag in UCSRnA will be set.
	//
	//		UPMn1	UPMn0	Parity Mode
	//		0	0	Disabled
	//		0	1	Reserved
	//		1	0	Enabled, even parity
	//		1	1	Enabled, odd parity
	//
	//	Bit 3 – USBSn: Stop Bit Select
	//
	static const byte USBSn		= 3;
	//
	//	This bit selects the number of stop bits to be inserted by the transmitter. The receiver ignores this setting.
	//
	//		USBSn	Stop Bit(s)
	//		0	1-bit
	//		1	2-bit
	//
	//	Bit 2:1 – UCSZn1:0: Character Size
	//
	static const byte UCSZn1	= 2;
	static const byte UCSZn0	= 1;
	//
	//	The UCSZn1:0 bits combined with the UCSZn2 bit in UCSRnB sets the number of data bits (character size) in a frame the
	//	receiver and transmitter use.
	//
	//		UCSZn2	UCSZn1	UCSZn0	Character Size
	//		0	0	0	5-bit
	//		0	0	1	6-bit
	//		0	1	0	7-bit
	//		0	1	1	8-bit
	//		1	0	0	Reserved
	//		1	0	1	Reserved
	//		1	1	0	Reserved
	//		1	1	1	9-bit
	//
	//	Bit 0 – UCPOLn: Clock Polarity
	//
	static const byte UCPOLn	= 0;
	//
	//	This bit is used for synchronous mode only. Write this bit to zero when asynchronous mode is used. The UCPOLn bit sets
	//	the relationship between data output change and data input sample, and the synchronous clock (XCKn).
	//
	//		UCPOLn	Transmitted Data Changed (Output of TxDn Pin) Received Data Sampled (Input on RxDn Pin)
	//		0	Rising XCKn edge Falling XCKn edge
	//		1	Falling XCKn edge Rising XCKn edge
	//
	//	UBRRnL and UBRRnH – USART Baud Rate Registers
	//	=================
	//
	//	Bit	15	14	13	12	11	10	9	8
	//		7	6	5	4	3	2	1	0
	//	UBRRnH	X	x	x	x	[	Baud rate MSB	]
	//	UBRRnL	[	Baud rate LSB					]
	//
	//
	//
	//	Basic AVR systems, those compatible with Uno use the
	//	following memory mapped USART IO.
	//
	volatile byte	_UCSRnA,	// +0
			_UCSRnB,	// +1
			_UCSRnC,	// +2
			_reserved_3,	// +3
			_UBRRnL,	// +4
			_UBRRnH,	// +5
			_UDRn;		// +6

public:
	//
	//	Define the set of actions which are applied to the registers
	//	as defined above.
	//
	inline void clear( void ) {
		_UCSRnA = 0;
		_UCSRnB = 0;
		_UCSRnC = 0;
		_UBRRnL = 0;
		_UBRRnH = 0;
	}
	//
	inline void enable_tx_rx( void ) { _UCSRnB |= ( bit( TXENn ) | bit( RXENn )); }
	inline void disable_tx_rx( void ) { _UCSRnB &= ~( bit( TXENn ) | bit( RXENn )); }
	inline void enable_rx_irq( void ) { _UCSRnB |= bit( RXCIEn ); }
	inline void disable_rx_irq( void ) { _UCSRnB &= ~bit( RXCIEn ); }
	inline void enable_dre_irq( void ) { _UCSRnB |= bit( UDRIEn ); }
	inline void disable_dre_irq( void ) { _UCSRnB &= ~bit( UDRIEn ); }
	//
	inline void parity_off( void ) { _UCSRnC &= ~( bit( UPMn1 ) | bit( UPMn0 )); }
	inline void parity_odd( void ) { _UCSRnC |= ( bit( UPMn1 ) | bit( UPMn0 )); }
	inline void parity_even( void ) { _UCSRnC |= bit( UPMn1 ); }
	inline void one_stopbit( void ) { _UCSRnC &= ~bit( USBSn ); }
	inline void two_stopbits( void ) { _UCSRnC |= bit( USBSn ); }
	//
	//	A little cheaty tbh, s MUST be one of 5, 6, 7 or 8 only.
	//
	//	bits	UCSZn1	UCSZn0
	//	----	------	------
	//	 5	  0	  0
	//	 6	  0	  1
	//	 7	  1	  0
	//	 8	  1	  1
	//
	inline void set_charsize( byte s ) { _UCSRnC |= ((( s - 5 ) & 3 ) << UCSZn0 ); }
	inline void set_baud_h( byte v ) { _UBRRnH = v; }
	inline void set_baud_l( byte v ) { _UBRRnL = v; }
	inline void set_baud_x2( void ) { _UCSRnA |= bit( U2Xn ); }
	//
	//	IO access to the USART.
	//
	inline byte data_read( void ) { return( _UDRn ); }
	inline void data_write( byte v ) { _UDRn = v; }

	//
	//	End of Basic AVR definition
	//
};

//
//	Pre-declare the USART_IO class as the USART_Device class
//	needs to keep references to pointers to this address for
//	linking in interrupts.
//
class USART_IO;

//
//	To implement the access to multiple hardware instances
//	the following class is defined providing the generic API
//	to any hardware instance.
//
class USART_Device {		
	public:
		//
		//	Declare the speed table used to lookup the
		//	appropriate parameters for the USART.
		//
		//	The ticks value provided is assuming the X2 flag
		//	is set to 1.
		//
		//	If this value exceeds the 12 bit UBRR value then
		//	this can be divided by 2 and set the X2 flag to
		//	0.
		//
		//	NOTE/ the final '-1' still needs to be applied
		//	before placing into the UBRR register.
		//
		struct speed_setting {
			USART_line_speed	speed;
			word			ticks;
		};
		static const speed_setting _configuration[] PROGMEM;

		//
		//	Where are the hardware registers?
		//
		USART_Registers	*_dev;
		USART_IO	**_vec;

	public:
		//
		//	Specify the constructor to provide the details
		//	of the hardware being accessed.
		//
		USART_Device( USART_Registers *dev, USART_IO **vec );
		//
		//	Configuration methods required.
		//
		void clear( void );
		void enable( bool run );
		//
		void bits( USART_char_size size );
		void parity( USART_data_parity parity );
		void stopbits( USART_stop_bits bits );
		bool baud( USART_line_speed speed );
		//
		void dre_irq( bool enable );
		void attach_io( USART_IO *io );
		void dettach_io( void );
		//
		void write( byte value );
		byte read( void );
};

//
//	Define the Console Class.
//
class USART_IO : public Byte_Queue_API {
	private:
		//
		//	Remember which device we are assigned to
		//
		USART_Device		*_dev;
		
		//
		//	Remember the addresses of our byte queues
		//
		Byte_Queue_API		*_input,
					*_output;

		//
		//	Boolean flag used to indicate if the transmission
		//	register empty interrupt is currently enabled (ie
		//	we are currently spooling stuff out).
		//
		volatile bool		_async;

	public:
		//
		//	Define the initialiser.
		//
		USART_IO( void );

		//
		//	Define the initialiser.
		//
		bool initialise( byte inst, USART_line_speed speed, USART_char_size bits, USART_data_parity parity, USART_stop_bits sbits, Byte_Queue_API *in_queue, Byte_Queue_API *out_queue );

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
		virtual byte available( void );

		//
		//	Return the number of bytes pending in the buffer.
		//	For "looped back" or simple "internal" buffers
		//	this will be the same value as the "available()"
		//	bytes.
		//
		//	However, for devices that are presented as a buffer
		//	(bi-directional devices) this will return the number
		//	of data bytes still awaiting (pending) being sent.
		//
		virtual byte pending( void );

		//
		//	byte space( void )
		//	------------------
		//
		//	Return the number of bytes which can be
		//	added to the output queue for writing.
		//
		virtual byte space( void );

		//
		//	byte read( void )
		//	-----------------
		//
		//	Return next byte from the serial connection.
		//
		//	If there is no data available this should return
		//	the nul byte, ascii 0x00, '\0'.
		//
		virtual byte read( void );

		//
		//	bool write( byte data )
		//	-----------------------
		//
		//	Write the supplied data to the output queue and
		//	send it when possible.
		//
		//	Returns true if queued, false otherwise.
		//
		virtual bool write( byte data );

		//
		//	Perform a "reset" of the underlying system.  This
		//	is used only to recover from an unknown condition
		//	with the expectation that upon return the queue
		//	can be reliably used.
		//
		virtual void reset( void );

		//
		//	Return the address of the data ready flag.
		//
		bool *data_ready( void );

		//
		//	These are the interrupt routines used to asynchronously
		//	fill the input buffer and drain the output buffer.
		//
		void input_ready( void );
		void output_ready( void );

};


#endif


//
//	EOF
//

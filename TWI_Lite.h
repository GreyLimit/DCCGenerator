//
//	twi.h - TWI/I2C library for Wiring & Arduino
//	Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
//
//	This library is free software; you can redistribute it and/or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.
//
//	This library is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//	Lesser General Public License for more details.
//
//	You should have received a copy of the GNU Lesser General Public
//	License along with this library; if not, write to the Free Software
//	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
//	Modified 2020 by Greyson Christoforo (grey@christoforo.net) to implement timeouts
//	Modified 2020 by Jeff Penfold <jeff.penfold@googlemail.com> to add lightness
//

#ifndef _TWI_LITE_H_
#define _TWI_LITE_H_

#include <inttypes.h>


#ifndef TWI_FREQ
#define TWI_FREQ 100000L
#endif

#ifndef TWI_BUFFER_LENGTH
#define TWI_BUFFER_LENGTH 8
#endif

#define TWI_READY 0
#define TWI_MRX   1
#define TWI_MTX   2
#define TWI_SRX   3
#define TWI_STX   4

extern void twi_init( void );
extern void twi_disable( void );
extern void twi_setAddress( uint8_t address );
extern void twi_setFrequency( uint32_t frequency );
extern uint8_t twi_readFrom( uint8_t address, uint8_t* data, uint8_t length, bool sendStop );
extern uint8_t twi_writeTo( uint8_t address, uint8_t* data, uint8_t length, uint8_t wait, bool sendStop );
extern uint8_t twi_transmit( const uint8_t* data, uint8_t length );
extern void twi_attachSlaveRxEvent( void (*)( uint8_t *buffer, int len ));
extern void twi_attachSlaveTxEvent( void (*)( void ));
extern void twi_reply( bool ack );
extern void twi_stop( void );
extern void twi_releaseBus( void );
extern void twi_setTimeoutInMicros( uint32_t timeout, bool reset_with_timeout );
extern void twi_handleTimeout( bool reset );
extern bool twi_manageTimeoutFlag( bool clear_flag );

#endif

//
//	EOF
//

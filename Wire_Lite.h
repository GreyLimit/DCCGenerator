//
//	TwoWire.h - TWI/I2C library for Arduino & Wiring
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
//	Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts
//	Modified 2020 by Greyson Christoforo (grey@christoforo.net) to implement timeouts
//	Modified 2020 by Jeff Penfold (jeff.penfold@googlemail.com) to reduce footprint.
//

#ifndef _TWO_WIRE_H_
#define _TWO_WIRE_H_

#include <inttypes.h>
#include <Stream.h>

//
//	We wish to only handle small pieces of data.
//
#define BUFFER_LENGTH 8

//
//	WIRE_HAS_END means Wire has end()
//
#define WIRE_HAS_END 1

//
//	The class in not called Wire_Lite (or indeed Wire) as would normally be
//	expected) as the functional result of the library *is not* the definition
//	of the class itself but the instance of a single, global, object
//	which controls and manages all Two Wire interactions.
//
//	This is what is called "Wire" and forms the primary interface object.
//
class TwoWire {
	private:
		static uint8_t rxBuffer[ BUFFER_LENGTH ];
		static uint8_t rxBufferIndex;
		static uint8_t rxBufferLength;

		static uint8_t txAddress;
		static uint8_t txBuffer[ BUFFER_LENGTH ];
		static uint8_t txBufferIndex;
		static uint8_t txBufferLength;

		static bool transmitting;
		static void (*user_onRequest)( void );
		static void (*user_onReceive)( int numBytes );
		static void onRequestService( void );
		static void onReceiveService( uint8_t* inBytes, int numBytes );

	public:
		TwoWire();

		void begin( void );
		void begin( uint8_t address );
		void end( void );

		void setClock( uint32_t );
		void setWireTimeout( uint32_t timeout = 25000, bool reset_with_timeout = false );
		bool getWireTimeoutFlag( void );
		void clearWireTimeoutFlag( void );
		
		void beginTransmission( uint8_t address );
		uint8_t endTransmission( void );
		uint8_t endTransmission( bool );
		
		uint8_t requestFrom( uint8_t, uint8_t );
		uint8_t requestFrom( uint8_t, uint8_t, bool );
		uint8_t requestFrom( uint8_t, uint8_t, uint32_t, uint8_t, bool );
		
		size_t write( uint8_t );
		size_t write( const uint8_t *, size_t );
		
		int available( void );
		int read( void );
		int peek( void );

		void onReceive( void (*function)( int numBytes ));
		void onRequest( void (*function)( void ));

};

//
//	Finally create (well, the extern) the object which is the
//	sole purpose of this library.
//
extern TwoWire Wire;

#endif

//
//	EOF
//

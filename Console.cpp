///
///	Console - code to contain the console object.
///
/// 	Firmware for an Arduino Uno R3 and Motor shield (with other
///	components) to create a simple and free standing "Train
///	Controller" to be as simple as possible.
///
///	Copyright (C) 2020, Jeff Penfold, jeff.penfold@googlemail.com
///	
///	This program is free software: you can redistribute it and/or modify
///	it under the terms of the GNU General Public License as published by
///	the Free Software Foundation, either version 3 of the License, or
///	(at your option) any later version.
///
///	This program is distributed in the hope that it will be useful,
///	but WITHOUT ANY WARRANTY; without even the implied warranty of
///	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
///	GNU General Public License for more details.
///
///	You should have received a copy of the GNU General Public License
///	along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "Environment.h"
#include "Parameters.h"
#include "Configuration.h"
#include "Console.h"
#include "Trace.h"

//
//	The CONSOLE device
//	==================
//

Console	console;


//
//	The call to initialise it.
//
void Console::initialise( byte dev, USART_line_speed speed ) {

	//
	//	Initialise the input queue and attach the
	//	data _ready signal to it.
	//
	_in.initialise( &_ready );
	_out.initialise( NIL( Signal ));
	
	//
	//	Set up the serial connection.
	//
	USART_IO::initialise( dev, speed, CS8, PNone, SBOne, &_in, &_out );

	//
	//	Configure the synchronisation of the output.
	//
	synchronous( DEBUGGING_OPTION( true, false ));
}

//
//	Get address of the control gate.
//
Signal *Console::control_signal( void ) {
#ifdef DEBUGGING_ENABLED
	Signal *flag;

	flag = &_ready;

	TRACE_CONSOLE( console.print( F( "Console flag " )));
	TRACE_CONSOLE( console.println( flag->identity()));

	return( flag );
	
#else
	return( &_ready );
#endif
}
 


//
//	eoF
//
 

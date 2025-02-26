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
///

#ifndef _CONSOLE_H_
#define _CONSOLE_H_

//
//	Bring in USART definitions
//
#include "Parameters.h"
#include "Byte_Queue.h"
#include "USART.h"

//
//	Define the console buffer sizes, if not predefined elsewhere.
//
#ifndef CONSOLE_INPUT
#define CONSOLE_INPUT	SELECT_SML(32,64,128)
#endif
#ifndef CONSOLE_OUTPUT
#define CONSOLE_OUTPUT	SELECT_SML(32,64,128)
#endif

//
//	Declare the Console class on top of the USART_IO class.
//
class Console : public USART_IO {
private:
	//
	//	Declare the IO buffers we will be using.
	//
	Byte_Queue_Signal< CONSOLE_INPUT >	_in;
	Byte_Queue< CONSOLE_OUTPUT >		_out;
	
public:
	//
	//	Declare the initialisation routine.
	//
	void initialise( byte dev, USART_line_speed speed );

	//
	//	Provide the mechanism to access the input
	//	queue data ready flag.
	//
	Signal *control_signal( void );
};

//
//	The CONSOLE device
//	==================
//
extern Console		console;

#endif

//
//	EOF
//

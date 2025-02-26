//
//	Hardware.h
//	==========
//
//	Bring in definitions which are pertinent to the target hardware.
//

#ifndef _HARDWARE_H_
#define _HARDWARE_H_


//
//	Hardware Specific Configuration Definitions
//	===========================================
//
//	The following definitions are used to abstract the differences
//	between each of the boards.
//
//	The Macro "SELECT_SML(s,m,l)" will be used to select alternate
//	configuration values based on the apparent "size" of the target
//	micro controller.  The parameter "s" represents small MCUs with
//	2 KBytes SRAM.  "m" represents systems with between
//	2 and 4 KBytes SRAM.  All other systems will have the "l" value
//	applied.
//

//
//	Pick out hardware specific configuration elements.
//
#if defined( __AVR_ATmega328__ )| defined( __AVR_ATmega328P__ )| defined( __AVR_ATmega328PB__ )
//
//	Standard Nano or Uno R3 configuration
//	=====================================
//
#define HW_TITLE		"AVR ATmega328"

//
//	SRAM = 2 KBytes
//	Arch = AVR
//
#define SELECT_SML(s,m,l)	s
#define SELECT_ARCH(avr,arm)	avr

#define BOARD_SRAM		2048
#define BOARD_REGISTERS		256
#define BOARD_STACK		512


#elif defined( __AVR_ATmega2560__ )
//
//	Standard Mega 2560 configuration
//	================================
//
#define HW_TITLE		"AVR ATmega2560"

//
//	SRAM = 8 KBytes
//	Arch = AVR
//
#define SELECT_SML(s,m,l)	m
#define SELECT_ARCH(avr,arm)	avr

#define BOARD_SRAM		8192
#define BOARD_REGISTERS		512
#define BOARD_STACK		512


#else
//
//	Firmware has not been configured for this board.
//
#error "No configuration available for this board"

#endif




#endif

//
//	EOF
//

//
//	Errors:		A consolidated error logging system
//			to enable errors to be noted at point
//			of detection, but processed at a future
//			point of convenience.
//

#ifndef _ERRORS_H_
#define _ERRORS_H_

#include "Environment.h"
#include "Parameters.h"
#include "Configuration.h"
#include "Task_Entry.h"
#include "Signal.h"
#include "Byte_Queue.h"

//
//	Define a list of Error numbers that this code could report
//
//	Note:	Probably ought to re-group these numbers into sections
//		so adding a new error number is less disruptive.
//
#define NO_ERROR			0

//
//	Error system errors!
//
#define ERROR_QUEUE_OVERFLOW		1
#define ERROR_REPORT_FAIL		2
#define ERROR_BUFFER_OVERFLOW		3
#define ERROR_OUTPUT_FORMAT		4

//
//	DCC command construction errors.
//
#define DCC_PROTOCOL_ERROR		10
#define DCC_COMMAND_OVERFLOW		11
#define DCC_COMMAND_TRUNCATED		12
#define DCC_COMMAND_EMPTY		13

//
//	DCC command parameter errors.
//
#define INVALID_DCC_COMMAND		20
#define INVALID_COMMAND_FORMAT		21
#define INVALID_ARGUMENT_COUNT		22
#define INVALID_ADDRESS			23
#define INVALID_SPEED			24
#define INVALID_DIRECTION		25
#define INVALID_STATE			26
#define INVALID_CV_NUMBER		27
#define INVALID_FUNC_NUMBER		28
#define INVALID_BIT_NUMBER		29
#define INVALID_BIT_VALUE		30
#define INVALID_BIT_MASK		31
#define INVALID_BYTE_VALUE		32
#define INVALID_WORD_VALUE		33
#define INVALID_POWER_ZONE		34
#define INVALID_ARGUMENT_RANGE		35
#define INVALID_BITMAP_VALUE		36
#define INVALID_CONSTANT		37

//
//	Operational errors.
//
#define NO_PROGRAMMING_TRACK		40
#define POWER_NOT_OFF			41
#define POWER_OVERLOAD			42
#define POWER_SPIKE			43
#define PROGRAMMING_TRACK_ONLY		44

//
//	System processing errors.
//
#define TRANSMISSION_REPORT_FAIL	50
#define TRANSMISSION_TABLE_FULL		51
#define TRANSMISSION_PENDING_FULL	52
#define TRANSMISSION_RECORD_EMPTY	53
#define TRANSMISSION_BIT_OVERFLOW	54

//
//	Process reporting errors
//
#define COMMAND_FORMAT_FAIL		60
#define COMMAND_REPORT_FAIL		61
#define COMMAND_TRANSMISSION_FAILED	62
#define COMMAND_EXECUTION_FAILED	63

//
//	Errors relating to the (now missing)
//	structured decoder programming logic.
//
//	This is to be reinstated at a suitable
//	time in the development cycle.
//
#define PARSE_CONF_CMD_ERROR		70
#define UNRECOGNISED_CONF_CMD		71
#define READ_ONLY_VARIABLE		72
#define INVALID_CV_NAME			73
#define INVALID_CV_ACCESS		74
#define CV_CHANGE_OVERFLOWED		75
#define CV_RANGE_ERROR			76
#define CV_INDEX_ERROR			77

//
//	Unexpected TWI state change
//
#define TWI_STATE_CHANGE		80

//
//	TWI receives too much data
//
#define TWI_READ_DATA_OVERFLOW		81

//
//	Rotary button queue full
//
#define ROTARY_BUTTON_QUEUE_FULL	82

//
//	DCC Driver configuration overflow.
//
#define DCC_DRIVER_CONFIGURATION_FULL	83

//
//	Error with signal range.
//
#define SIGNAL_RANGE_ERROR		87

//
//	Unexpected ADC interrupt.
//
#define ADC_UNEXPECTED_RESULT		88

//
//	ADC queue full.
//
#define ADC_QUEUE_FULL			89

//
//	Task Manager queue is full!
//
#define TASK_MANAGER_QUEUE_FULL		90

//
//	Time of Day queue full
//
#define TIME_OF_DAY_QUEUE_FULL		91

//
//	Event time capacity exceeded.
//
#define EVENT_TIMER_QUEUE_FULL		92

//
//	Task manager errors.
//
#define TASK_DEPTH_EXCEEDED		93

//
//	LCD COMMS Errors
//
#define LCD_QUEUE_FULL			94

//
//	I2C Errors reported
//
#define I2C_COMMS_ERROR			95

//
//	Resource errors.
//
#define ERRORS_ERR_OVERFLOW		96
#define USART_IO_ERR_DROPPED		97

//
//	Heap allocation errors
//
#define HEAP_ERR_NO_ERROR		100
#define HEAP_ERR_OUT_OF_MEMORY		101
#define HEAP_ERR_INVALID_ADRS		102
#define HEAP_ERR_DUP_DEALLOCATE		103
#define HEAP_ERR_INVALID_ALLOCATION	104

//
//	SPI related errors.
//
#define SPI_TARGET_TABLE_FULL		110
#define SPI_INVALID_CLOCK_SPEED		111
#define SPI_QUEUE_FULL			112

//
//	Clock errors
//
#define CLOCK_INVALID_DIVIDER		120
#define CLOCK_ISR_DROPPED		121

//
//	DCC decoder FSM Errors and
//	Bit identification and buffer errors
//
#define BIT_BUFFER_OVERFLOW		130
#define DCCFSM_PREAMBLE_TRUNCATED	131
#define DCCFSM_PARITY_ERROR		132
#define DCCFSM_OVERFLOW_ERROR		133
//	#define BIT_BUFFER_OVERFLOW_BREAK	0
//	#define BIT_BUFFER_OVERFLOW_ONE		1
//	#define BIT_BUFFER_OVERFLOW_ZERO	-1

//
//

//
//	Programming error abort.
//
#define PROGRAMMER_ERROR_ABORT		200

//
//	Code Assurance error
//
#define CODE_ASSURANCE_ERR_ASSERT	201


//
//	Define the Error handling class
//
class Errors : public Task_Entry {
private:
	//
	//	How many errors will we try to cache?
	//
	static const byte cache_size = 4;

	//
	//	How do we keep the errors?
	//
	struct error_record {
		byte		error,		// What?
				repeats;	// How often?
		word		arg;		// Supporting data.
	};

	//
	//	Where do we keep them?
	//
	error_record	_cache[ cache_size ];
	byte		_count,
			_in,
			_out;
			
	//
	//	Where will we send them?
	//
	Byte_Queue_API	*_output;
			
	//
	//	The task conrtol signal.
	//
	Signal		_errors;

	//
	//	This is system aborted flag, and is used to stop
	//	errors generated after a system abort from messing up
	//	any data in place at the time of the abort.
	//
	bool		_aborted;
	
	//
	//	Drop the next error.
	//
	void drop_error( void );


public:
	//
	//	Constructor
	//
	Errors( void );
	
	//
	//	The initialisation routine for the errors object
	//
	void initialise( Byte_Queue_API	*to );
	
	//
	//	The task manager calls this routine to handle the
	//	output of errors.  This is controlled by the "_errors"
	//	Signal.
	//
	virtual void process( byte handle );
	
	//
	//	Log an error with the system
	//
	void log_error( byte error, word arg );
	
	//
	//	Log a terminal system error with the system.
	//
	void log_terminate( word error, const __FlashStringHelper *file_name, word line_number );
};


//
//	Externally declare the errors object.
//
extern Errors errors;

#endif
//
//	EOF
//

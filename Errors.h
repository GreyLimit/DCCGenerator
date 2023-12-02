//
//	Errors:		A consolidated error logging system
//			to enable errors to be noted at point
//			of detection, but processed at a future
//			point of convenience.
//

#ifndef _ERRORS_H_
#define _ERRORS_H_


//
//	Declare the Error handling class
//
class Errors {
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

public:
	//
	//	Constructor
	//
	Errors( void );
	
	//
	//	Log an error with the system
	//
	void log_error( word error, word arg );
	
	//
	//	Log a terminal system error with the system.
	//
	void log_terminate( word error, const char *file_name, word line_number );

	//
	//	Return count of errors pending
	//
	int pending_errors( void );

	//
	//	Peek at the top error.
	//
	//	Return true if there was one, false otherwise.
	//
	bool peek_error( word *error, word *arg, byte *repeats );

	//
	//	Drop the top error.  It is assumed that peek was
	//	used to obtain the content of the error so this
	//	allows an error which has been handled to be
	//	discarded.
	//
	void drop_error( void );
};


//
//	Define a list of Error numbers that this code could report
//
//	Note:	Probably ought to re-group these numbers into sections
//		so adding a new error number is less disruptive.
//
#define NO_ERROR			0
#define ERROR_QUEUE_OVERFLOW		1
#define ERROR_REPORT_FAIL		2
#define ERROR_BUFFER_OVERFLOW		3
#define BIT_TRANS_OVERFLOW		4
#define DCC_COMMAND_OVERFLOW		5
#define UNRECOGNISED_COMMAND		6
#define INVALID_BUFFER_NUMBER		7
#define INVALID_ARGUMENT_COUNT		8
#define INVALID_ADDRESS			9
#define INVALID_SPEED			10
#define INVALID_DIRECTION		11
#define INVALID_STATE			12
#define INVALID_CV_NUMBER		13
#define INVALID_FUNC_NUMBER		14
#define INVALID_BIT_NUMBER		15
#define INVALID_BIT_VALUE		16
#define INVALID_BIT_MASK		17
#define INVALID_BYTE_VALUE		18
#define INVALID_WORD_VALUE		19
#define COMMAND_REPORT_FAIL		20
#define TRANSMISSION_BUSY		21
#define COMMAND_QUEUE_FAILED		22
#define POWER_NOT_OFF			23
#define NO_PROGRAMMING_TRACK		24
#define POWER_OVERLOAD			25
#define POWER_SPIKE			26
//
//	Resource errors.
//
#define ERRORS_ERR_OVERFLOW		96
#define USART_IO_ERR_DROPPED		97
//
//	Code Assurance errors
//
#define CODE_ASSURANCE_ERR_ASSERT	98
#define CODE_ASSURANCE_ERR_ABORT	99


//
//	Externally declare the errors object.
//
extern Errors errors;

#endif
//
//	EOF
//

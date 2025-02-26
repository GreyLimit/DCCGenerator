//
//	Protocol.cpp
//	============
//
//	Computer IO Protocol
//

#include "Protocol.h"
#include "Signal.h"
#include "Task.h"
#include "Errors.h"
#include "Code_Assurance.h"
#include "Buffer.h"
#include "DCC.h"
#include "Districts.h"
#include "Console.h"
#include "Trace.h"

//
//	Set up ready to be initialised.
//
Protocol::Protocol( void ) {
	_inside = false;
	_valid = true;
	_len = 0;
}

//
//	Define simple "in string" number parsing routine.  This
//	effectively does the bulk of the syntax parsing for the
//	commands so it a little more fiddly than strictly
//	necessary.
//
//	Return the address of the next unparsed character, this
//	be EOS if at the end of the string.
//
char *Protocol::parse_number( char *buf, bool *found, int *value ) {

	STACK_TRACE( "char *Protocol::parse_number( char *buf, bool *found, int *value )" );

	int	v, n, c;

	//
	//	Skip any leading white space
	//
	while( isspace( *buf )) buf++;
	//
	//	Intentional assignment, remember if we are handling
	//	a negative number.
	//
	if(( n = ( *buf == '-' ))) buf++;
	//
	//	gather up a decimal number
	//
	c = 0;
	v = 0;
	while( isdigit( *buf )) {
		v = v * 10 + ( *buf++ - '0' );
		c++;
	}
	if( n ) v = -v;
	//
	//	Skip any trailing white space
	//
	while( isspace( *buf )) buf++;
	//
	//	Set up returned data
	//
	*found = ( c > 0 );
	*value = v;
	//
	//	Return address of next unprocessed character.
	//
	return( buf );
}

//
//	Break a received packet into its logical elements.
//
byte Protocol::parse_input( char *buf, char *cmd, int *arg, int max ) {

	STACK_TRACE( "byte Protocol::parse_input( char *buf, char *cmd, int *arg, int max )" );

	byte	args;
	int	value;
	bool	found;

	ASSERT( max > 0 );

	//
	//	Copy out the command character and verify
	//
	*cmd  = *buf++;
	if( !isalnum( *cmd )) return( ERROR );
	//
	//	Step through remainder of string looking for
	//	numbers.
	//
	args = 0;
	found = true;
	while( found ) {
		buf = parse_number( buf, &found, &value );
		if( found ) {
			if( args >= max ) return( MAXIMUM_BYTE );
			arg[ args++ ] = value;
		}
	}
	//
	//	Done.
	//
	return( args );
}

//
//	Parse an input buffer.
//
void Protocol::parse_buffer( char *buf ) {

	STACK_TRACE( "void Protocol::parse_buffer( char *buf )" );

	char	cmd;
	int	arg[ maximum_arguments ];
	byte	args;

	if(( args = parse_input( buf, &cmd, arg, maximum_arguments )) == MAXIMUM_BYTE ) {
		errors.log_error( INVALID_COMMAND_FORMAT, 0 );
		return;
	}
	//
	//	We have at least a command and (possibly) some arguments.
	//
	switch( cmd ) {
		//
		//	Power management command
		//	------------------------
		//
		case power: {
			Buffer<DCC::maximum_output>	reply;

			//
			//	Select zone to "power up".  Zone 0 is "no
			//	zone":  All zones powered off.
			//
			//	[P z]
			//		z=0	Power all zones off.
			//		z=1	Power Main Track.
			//		z=2	Power Programming Track
			//
			if( args != 1 ) {
				errors.log_error( INVALID_ARGUMENT_COUNT, args );
				break;
			}
			if( !valid_power_zone( arg[ 0 ])) {
				errors.log_error( INVALID_POWER_ZONE, arg[ 0 ]);
				break;
			}
			if( !reply.format( power, arg[ 0 ])) {
				errors.log_error( COMMAND_FORMAT_FAIL, power );
				break;
			}
			if( !districts.power( arg[ 0 ])) {
				errors.log_error( COMMAND_EXECUTION_FAILED, power );
				break;
			}
			if( !reply.send( &console )) {
				errors.log_error( COMMAND_REPORT_FAIL, power );
			}
			break;
		}
		
		//
		//	Cab/Mobile decoder command
		//	--------------------------
		//
		case mobile: {
			Buffer< DCC::maximum_output >	reply;
			byte				speed;
			
			//
			//	Mobile decoder set speed and direction
			//	--------------------------------------
			//
			//	[M adrs speed dir] -> [M adrs speed dir]
			//
			//		adrs:	The short (1-127) or long (128-10239) address of the engine decoder
			//		speed:	Throttle speed from 0-126, or -1 for emergency stop
			//		dir:	1=Forward, 0=Reverse
			//
			if( args != 3 ) {
				errors.log_error( INVALID_ARGUMENT_COUNT, args );
				break;
			}
			if( !valid_mobile_target( arg[ 0 ])) {
				errors.log_error( INVALID_ADDRESS, arg[ 0 ]);
				break;
			}
			if(!( valid_mobile_speed( arg[ 1 ])||( arg[ 1 ] == emergency_mobile_stop ))) {
				errors.log_error( INVALID_SPEED, arg[ 1 ]);
				break;
			}
			if( !valid_mobile_dir( arg[ 2 ])) {
				errors.log_error( INVALID_DIRECTION, arg[ 2 ]);
				break;
			}
			if( !reply.format( mobile, arg[ 0 ], arg[ 1 ], arg[ 2 ])) {
				errors.log_error( COMMAND_FORMAT_FAIL, mobile );
				break;
			}
			switch( arg[ 1 ]) {
				case emergency_mobile_stop: {
					speed = 1;
					break;
				}
				case minimum_mobile_speed: {
					speed = 0;
					break;
				}
				default: {
					speed = arg[ 1 ]+1;
					break;
				}
			}
			if( !dcc_generator.mobile_command( arg[ 0 ], speed, arg[ 2 ], &reply )) {
				errors.log_error( COMMAND_TRANSMISSION_FAILED, mobile );
			}
			break;
		}
		
		//
		//	Accessory Command
		//	-----------------
		//
		case accessory: {
			Buffer< DCC::maximum_output >	reply;

			//
			//	Accessory decoder set state
			//	---------------------------
			//
			//	[A adrs state] -> [A adrs state]
			//
			//		adrs:	The combined address of the decoder (1-2048)
			//		state:	1=on (set), 0=off (clear)
			//
			if( args != 2 ) {
				errors.log_error( INVALID_ARGUMENT_COUNT, args );
				break;
			}
			if( !valid_accessory_address( arg[ 0 ])) {
				errors.log_error( INVALID_ADDRESS, arg[ 0 ]);
				break;
			}
			if( !valid_accessory_state( arg[ 1 ])) {
				errors.log_error( INVALID_STATE, arg[ 1 ]);
				break;
			}
			if( !reply.format( accessory, arg[ 0 ], arg[ 1 ])) {
				errors.log_error( COMMAND_FORMAT_FAIL, accessory );
				break;
			}
			if( !dcc_generator.accessory_command( arg[ 0 ], arg[ 1 ], &reply )) {
				errors.log_error( COMMAND_TRANSMISSION_FAILED, accessory );
			}
			break;
		}

		//
		//	Mobile decoder functions
		//	------------------------
		//
		case function: {
			Buffer< DCC::maximum_output >	reply;

			//
			//	Mobile decoder set function state
			//	---------------------------------
			//
			//	[F adrs func state] -> [F adrs func state]
			//
			//		adrs:	The short (1-127) or long (128-10239) address of the engine decoder
			//		func:	The function number to be modified (0-28)
			//		state:	1=Enable, 0=Disable, 2=Toggle
			//
			//	Encoding a new "state": 2.  This is the act of turning
			//	on a function then almost immediately turning it off
			//	again (as a mnemonic this is, in binary, a 1 followed
			//	by a 0)
			//	
			if( args != 3 ) {
				errors.log_error( INVALID_ARGUMENT_COUNT, args );
				break;
			}
			if( !valid_mobile_target( arg[ 0 ])) {
				errors.log_error( INVALID_ADDRESS, arg[ 0 ]);
				break;
			}
			if( !valid_function_number( arg[ 1 ])) {
				errors.log_error( INVALID_FUNC_NUMBER, arg[ 1 ]);
				break;
			}
			if( !valid_function_state( arg[ 2 ])) {
				errors.log_error( INVALID_STATE, arg[ 2 ]);
				break;
			}
			if( !reply.format( function, arg[ 0 ], arg[ 1 ])) {
				errors.log_error( COMMAND_FORMAT_FAIL, function );
				break;
			}
			if( !dcc_generator.function_command( arg[ 0 ], arg[ 1 ], arg[ 2 ], &reply )) {
				errors.log_error( COMMAND_TRANSMISSION_FAILED, function );
			}
			break;
		}
		//	Write Mobile State (Operations Track)
		//	-------------------------------------
		//
		//	Overwrite the entire "state" of a specific mobile decoder
		//	with the information provided in the arguments.
		//
		//	While this is provided as a single DCC Generator command
		//	there is no single DCC command which implements this functionality
		//	so consequently the command has to be implemented as a tightly
		//	coupled sequence of commands.  This being said, the implementation
		//	of the commannd should ensure that either *all* of these commands
		//	are transmitted or *none* of them are.  While this does not
		//	guarantee that the target decoder gets all of the updates
		//	it does increase the likelihood that an incomplete update is
		//	successful.
		//
		case rewrite_state: {
			Buffer< DCC::maximum_output >	reply;
			byte				speed,
							funcs[ DCC_Constant::bit_map_array ];
			
			//
			//	[W adrs speed dir fna fnb fnc fnd] -> [W adrs speed dir]
			//
			//	0	adrs:	The short (1-127) or long (128-10239) address of the engine decoder
			//	1	speed:	Throttle speed from 0-126, or -1 for emergency stop
			//	2	dir:	1=Forward, 0=Reverse
			//	3	fna:	Bit mask (in decimal) for Functions 0 through 7
			//	4	fnb:	... Functions 8 through 15
			//	5	fnc:	... Functions 16 through 23
			//	6	fnd:	... Functions 24 through 28 (bit positions for 29 through 31 ignored)
			//
			if( args != ( 3 + DCC_Constant::bit_map_array )) {
				errors.log_error( INVALID_ARGUMENT_COUNT, args );
				break;
			}
			if( !valid_mobile_target( arg[ 0 ])) {
				errors.log_error( INVALID_ADDRESS, arg[ 0 ]);
				break;
			}
			if( !valid_mobile_speed( arg[ 1 ])) {
				errors.log_error( INVALID_SPEED, arg[ 1 ]);
				break;
			}
			if( !valid_mobile_dir( arg[ 2 ])) {
				errors.log_error( INVALID_DIRECTION, arg[ 2 ]);
				break;
			}
			for( byte i = 0; i < DCC_Constant::bit_map_array; i++ ) {
				byte j = 3 + i; // index into args.
				
				if( !valid_bitmap_value( arg[ j ])) {
					errors.log_error( INVALID_BITMAP_VALUE, arg[ j ]);
					break;
				}
				funcs[ i ] = arg[ j ];
			}
			if( !reply.format( rewrite_state, arg[ 0 ], arg[ 1 ], arg[ 2 ])) {
				errors.log_error( COMMAND_FORMAT_FAIL, rewrite_state );
				break;
			}
			switch( arg[ 1 ]) {
				case emergency_mobile_stop: {
					speed = 1;
					break;
				}
				case minimum_mobile_speed: {
					speed = 0;
					break;
				}
				default: {
					speed = arg[ 1 ]+1;
					break;
				}
			}
			if( !dcc_generator.state_command( arg[ 0 ], speed, arg[ 2 ], funcs, &reply )) {
				errors.log_error( COMMAND_TRANSMISSION_FAILED, rewrite_state );
			}
			break;
		}
		//
		//	EEPROM configurable constants
		//
		case eeprom: {
			//
			//	Accessing EEPROM configurable constants
			//
			//	[Q] -> [Q n]			Return number of tune-able constants
			//	[Q c] -> [Q c v name]		Access a specific constant C (range 0..N-1)
			//	[Q -1 -1] -> [Q -1 -1]		Reset all constants to default.
			//	[Q c v v] -> [Q c v name]	Set a specific constant C to value V,
			//					second V is to prevent accidental
			//					update.
			//
			switch( args ) {
				case 0: {
					Buffer<DCC::maximum_output>	reply;

					//
					//	[Q] -> [Q n]
					//
					//			Respond with number of constant values
					//			defined.
					//
					if( !reply.format( eeprom, CONSTANTS )) {
						errors.log_error( COMMAND_FORMAT_FAIL, eeprom );
						break;
					}
					if( !reply.send( &console )) {
						errors.log_error( COMMAND_REPORT_FAIL, eeprom );
					}
					break;
				}
				case 1: {
					Buffer<DCC::eeprom_maximum_output>	reply;

					char	*n;
					byte	*b;
					word	*w;

					//
					//	[Q n] -> [Q c v name]
					//
					//			Request details of constant referenced by
					//			index n.
					//
					if( find_constant( arg[ 0 ], &n, &b, &w ) == ERROR ) {
						errors.log_error( INVALID_CONSTANT, arg[ 0 ]);
						break;
					}
					if( b ) {
						if( !reply.format( eeprom, arg[ 0 ], (word)( *b ), n )) {
							errors.log_error( COMMAND_FORMAT_FAIL, eeprom );
							break;
						}
					}
					else {
						if( !reply.format( eeprom, arg[ 0 ], *w, n )) {
							errors.log_error( COMMAND_FORMAT_FAIL, eeprom );
							break;
						}
					}
					if( !reply.send( &console )) {
						errors.log_error( COMMAND_REPORT_FAIL, 0 );
					}
					break;
				}
				case 2: {
					Buffer<DCC::maximum_output>	reply;
	
					//
					//	[Q -1 -1] -> [Q -1 -1]
					//
					//			Reset all constants to their default
					//			setup values.
					//
					if(( arg[ 0 ] == -1 )&&( arg[ 1 ] == -1 )) {
						reset_constants();
						if( !reply.format( eeprom, -1, -1 )) {
							errors.log_error( COMMAND_FORMAT_FAIL, eeprom );
							break;
						}
						if( !reply.send( &console )) {
							errors.log_error( COMMAND_REPORT_FAIL, eeprom );
						}
					}
					else {
						errors.log_error( INVALID_COMMAND_FORMAT, eeprom );
					}
					break;
				}
				case 3: {
					Buffer<DCC::eeprom_maximum_output>	reply;
					
					char	*n;
					byte	*b;
					word	*w;
					
					//
					//	[Q c v v] -> [Q c v name]
					//
					//			Set a specific constant C to value V,
					//			second V is to prevent accidental
					//			update.
					//
					if(( arg[ 1 ] == arg[ 2 ])&&( find_constant( arg[ 0 ], &n, &b, &w ) != ERROR )) {
						if( b ) {
							if(( arg[ 1 ] < 0 )||( arg[ 1 ] > MAXIMUM_BYTE )) {
								errors.log_error( INVALID_ARGUMENT_RANGE, 1 );
								break;
							}
							*b = (byte)arg[ 1 ];
						}
						else {
							//
							//	Yes, this test is a possible source of issues as
							//	it assumes that 'word' and 'int' types are basically
							//	the same number of bits.  This is true for an AVR
							//	Arduino MCU, but not for the ARM based MCUs.
							//
							if( arg[ 1 ] < 0 ) {
								errors.log_error( INVALID_ARGUMENT_RANGE, 1 );
								break;
							}
							*w = arg[ 1 ];
						}
						record_constants();
						if( !reply.format( eeprom, arg[ 0 ], arg[ 1 ], n )) {
							errors.log_error( COMMAND_REPORT_FAIL, eeprom );
							break;
						}
						if( !reply.send( &console )) {
							errors.log_error( COMMAND_REPORT_FAIL, eeprom );
						}
					}
					break;
				}
				default: {
					errors.log_error( INVALID_ARGUMENT_COUNT, cmd );
					break;
				}
			}
			break;
		}
		default: {
			//
			//	Here we capture any unrecognised command letters.
			//
			errors.log_error( INVALID_DCC_COMMAND, cmd );
			break;
		}
	}
	//
	//	Done.
	//
}

//
//	Kick off all input processing actions
//
void Protocol::initialise( void ) {

	STACK_TRACE( "void Protocol::initialise( void )" );

	//
	//	All we do, really, is link ourselves into the console
	//	stream.  Now the 'process()' routine is called every
	//	time there is data to be processed.
	//
	if( !task_manager.add_task( this, console.control_signal())) ABORT( TASK_MANAGER_QUEUE_FULL );
}

//
//	The task entry point.
//
void Protocol::process( UNUSED( byte handle )) {

	STACK_TRACE( "void Protocol::process( byte handle )" );

	char	data;

	switch(( data = console.read())) {
		//
		//	The protocol wrappers
		//
		case lead_in: {
			//
			//	A new packet starts here.  If we were already
			//	inside one then we throw it away.
			//
			if( _inside ) errors.log_error( DCC_COMMAND_TRUNCATED, _len );
			//
			//	Set up for a new command to arrive.
			//
			_len = 0;
			_inside = true;
			_valid = true;
			break;
		}
		case lead_out: {
			//
			//	The end of a protocol packet - process it.
			//
			//	Sanity checks.
			//
			if( !_inside ) {
				errors.log_error( DCC_PROTOCOL_ERROR, 0 );
				break;
			}
			if( _len == 0 ) {
				errors.log_error( DCC_COMMAND_EMPTY, 0 );
				break;
			}

			//
			//	Parse buffer (if there was no error)  and
			//	reset for next command.
			//
			if( _valid ) {
				ASSERT( _len < buffer_size );
				_buffer[ _len ] = EOS;
				parse_buffer( _buffer );
			}
			_len = 0;
			_inside = false;
			_valid = true;
			break;
		}
		default: {
			//
			//	Some for a packet or just "other stuff" moving
			//	on past...
			//
			if( _inside ) {
				if( _len < buffer_size-1 ) {
					_buffer[ _len++ ] = data;
				}
				else {
					if( _valid ) {
						_valid = false;
						errors.log_error( DCC_COMMAND_TRUNCATED, _len );
					}
				}
			}
			break;
		}
	}
}


//
//	Here is the protocol engine.
//
Protocol protocol;


//
//	EOF
//

# Arduino Generator V1.3.4

## Summary

The continued development of the original DCC++ alternative.

This version (V1.3) is (probably) the last version before a migration to a more universal solution that incorporates support for both the "Arduino Network" and a distributed handset network.

This replacement system has been characterised as the "The Mid Controller" being half way between the "Fat" and "Thin" controller solutions.

## Version 1.3.4

This version sees the intrduction of the "State Restore" command.  This is a command specifically created to facilitate the ability to completely overwrite the "state" of a mobile decoder.  This single command allows both the speed and direction *and all* of the function states to be reset in a single (hopefully nearly atomic) action.  See the descrition of the command for details and caveates.

## Versions 1.3.x

These versions have seen the replacement of the Arduino Serial library with my own USART class and a further development of the code associated with the detection of confirmation signals when programming decoders on the programming track.

Commands operating upon the programming track have been "better" constructed as a result of a note read indicating that decoders should only act on a command if they receive TWO sequential and intact copies of it.  To this end when such a command is initiated the transmission buffer has 2 copies of the command placed into it for this purpose.


## Communication Protocol

### Native Arduino Generator Mode
The USB connection to the host computer is 8-bit serial, no parity at 38400 baud.

```
	//
	//	DCC Generator Command Summary
	//	=============================
	//
	//	For the moment the following commands are described in outline
	//	only; details to be provided.
	//
	//	Mobile decoder set speed and direction
	//	--------------------------------------
	//
	//	[M ADRS SPEED DIR] -> [M ADRS SPEED DIR]
	//
	//		ADRS:	The short (1-127) or long (128-10239) address of the engine decoder
	//		SPEED:	Throttle speed from 0-126, or -1 for emergency stop
	//		DIR:	1=Forward, 0=Reverse
	//
	//	Accessory decoder set state
	//	---------------------------
	//
	//	[A ADRS STATE] -> [A ADRS STATE]
	//
	//		ADRS:	The combined address of the decoder (1-2048)
	//		STATE:	1=on (set), 0=off (clear)
	//
	//	Mobile decoder set function state
	//	---------------------------------
	//
	//	[F ADRS FUNC VALUE] -> [F ADRS FUNC STATE]
	//
	//		ADRS:	The short (1-127) or long (128-10239) address of the engine decoder
	//		FUNC:	The function number to be modified (0-21)
	//		VALUE:	1=Enable, 0=Disable
	//		STATE:	1=Confirmed, 0=Failed
	//	
	//	Write Mobile State (Operations Track)
	//	-------------------------------------
	//
	//	Overwrite the entire "state" of a specific mobile decoder
	//	with the information provided in the arguments.
	//
	//	While this is provided as a single DCC Generator command
	//	there is no single DCC command which implements this functionality
	//	so consequently the command has to be implemented as a tightly
	//	coupled sequence of commands.  This being said, thhe implementation
	//	of the commannd should ensure that either *all* of these commands
	//	are transmitted or *none* of them are.  While this does not
	//	guarantee that the target decoder gets all of the updates
	//	it does increase the likelihood that an incomplete update is
	//	successful.
	//
	//	[W ADRS SPEED DIR FNA FNB FNC FND] -> [W ADRS SPEED DIR]
	//
	//		ADRS:	The short (1-127) or long (128-10239) address of the engine decoder
	//		SPEED:	Throttle speed from 0-126, or -1 for emergency stop
	//		DIR:	1=Forward, 0=Reverse
	//		FNA:	Bit mask (in decimal) for Functions 0 through 7
	//		FNB:	... Functions 8 through 15
	//		FNC:	... Functions 16 through 23
	//		FND:	... Functions 24 through 28 (bit positions for 29 through 31 ignored)
	//
	//	Enable/Disable Power to track
	//	-----------------------------
	//
	//	[P STATE] -> [P STATE]
	//
	//		STATE: 0=Off, 1=Operations Track ON, 2= Programming Track ON. 
	//
	//	Set CV value (Programming track)
	//	--------------------------------
	//
	//	[S CV VALUE] -> [S CV VALUE STATE]
	//
	//		CV:	Number of CV to set (1-1024)
	//		VALUE:	8 bit value to apply (0-255)
	//		STATE:	1=Confirmed, 0=Failed
	//
	//	Set CV bit value (Programming track)
	//	------------------------------------
	//
	//	Set the specified CV bit with the supplied
	//	value.
	//
	//	[U CV BIT VALUE] -> [U CV BIT VALUE STATE]
	//
	//		CV:	Number of CV to set (1-1024)
	//		BIT:	Bit number (0 LSB - 7 MSB)
	//		VALUE:	0 or 1
	//		STATE:	1=Confirmed, 0=Failed
	//
	//
	//	Verify CV value (Programming track)
	//	-----------------------------------
	//
	//	[V CV VALUE] -> [V CV VALUE STATE]
	//
	//		CV:	Number of CV to set (1-1024)
	//		VALUE:	8 bit value to apply (0-255)
	//		STATE:	1=Confirmed, 0=Failed
	//
	//	Verify CV bit value (Programming track)
	//	---------------------------------------
	//
	//	Compare the specified CV bit with the supplied
	//	value, if they are the same, return 1, otherwise
	//	(or in the case of failure) return 0.
	//
	//	[R CV BIT VALUE] -> [R CV BIT VALUE STATE]
	//
	//		CV:	Number of CV to set (1-1024)
	//		BIT:	Bit number (0 LSB - 7 MSB)
	//		VALUE:	0 or 1
	//		STATE:	1=Confirmed, 0=Failed
	//
	//	Accessing EEPROM configurable constants
	//	---------------------------------------
	//
	//		[Q] -> [Q N]				Return number of tunable constants
	//		[Q C] ->[Q C V NAME]		Access a specific constant C (range 0..N-1)
	//		[Q C V V] -> [Q C V NAME]	Set a specific constant C to value V,
	//									second V is to prevent accidental
	//									update.
	//		[Q -1 -1] -> [Q -1 -1]		Reset all constants to default.
	//
	//
	//	Asynchronous data returned from the firmware
	//	============================================
	//
	//	Change in Power state:
	//
	//		-> [P STATE]
	//
	//	Current power consumption of the system:
	//
	//		-> [L LOAD]
	//
	//		LOAD: Figure between 0 and 1023
	//
	//	Report status of individual districts
	//
	//		-> [D a b ...]
	//
	//		Reported numbers (a, b, c ...) reflect
	//		the individual districts A, B C etc (independent
	//		of the role of the district).  The values
	//		provided follow the following table:
	//
	//			0	Disabled
	//			1	Enabled
	//			2	Phase Flipped
	//			3	Overloaded
	//
	//	Error detected by the firmware
	//
	//		-> [E ERR ARG]
	//
	//		ERR:	Error number giving nature of problem
	//		ARG:	Additional information data, nature
	//			dependant on the error number.
	//
```

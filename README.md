# DCCGenerator
Firmware for an Arduino UNO and motor shield to generate an NMRA DCC signal for operating a model railway

For the moment, simply reading the source code shuld provide an insight into what this is and how it does it.

I will come back to this and extend the descritpion.



## DCC Plus Plus Command Summary

The following summary provides the sub-set of DCC commands
supported by the DCCpp Arduino Firmware which this code will
try to emulate.

Serial port configured to 115200 baud.

Descriptions of the DCC Packet format will not include the
pre-amble, the inter-byte bits nor the parity byte.

Multi-function address byte(s) are defined as one of the following
two formats (represented in binary):

- Short Address range (1-127):	0AAAAAAA
- Long Address range (1-10239):	11AAAAAA AAAAAAAA

Note:	The top 6 bits of the long address are restricted (by the
DCC standard) to be from 000000 to 100111 (39 decimal).
As a result the largest long address is 10 0111 1111 1111
translating to 10239.


### Set Engine Throttle (126-Step)

Set speed and direction of an engine fitted with a mobile decoder.

    <t BUFFER ADRS SPEED DIR>

Where:
- BUFFER:	An internal buffer number, from 1 through to MOBILE_TRANS_BUFFERS
- ADRS:	The short (1-127) or long (128-10239) address of the engine decoder
- SPEED:	Throttle speed from 0-126, or -1 for emergency stop
- DIR:	1=forward, 0=reverse
Returns:

    <T BUFFER SPEED DIRECTION>

DCC Packet format:

    [0AAAAAAA|11AAAAAA AAAAAAAA] 00111111 DSSSSSSS

Where:

- A:		Address data.
- D:		Direction; 1=forwards, 0=backwards.
- SSSSSSS:	Speed; 0=stop, 1=emergency stop, or target speed

### Modify Engine Function F0-F28
Activate/Deactivate function within a mobile decoder.    

    <f ADRS BYTE1 [BYTE2]>

- ADRS:	The short (1-127) or long (128-10239) address of the engine decoder

In the following options [Fn] represents the bit for function "n", where a 1 would
activate it and a 0 disable it (byte values shown in binary).

To set functions F0-F4:

    BYTE1:  100[F0][F4][F3][F2][F1]

To set functions F5-F8:

    BYTE1:  1011[F8][F7][F6][F5]

To set functions F9-F12:

    BYTE1:  1010[F12][F11][F10][F9]

To set functions F13-F20:

    BYTE1: 11011110 
    BYTE2: [F20][F19][F18][F17][F16][F15][F14][F13]

To set functions F21-F28:

    BYTE1: 11011111
    BYTE2: [F28][F27][F26][F25][F24][F23][F22][F21]

Returns: Nothing

DCC Packet format:

    [0AAAAAAA|11AAAAAA AAAAAAAA] {BYTE1} [{BYTE2}]

### Operate Accessory Decoder

    <a ADRS SUBADRS STATE>
    
Where:
- ADRS:	The primary address of the decoder (0-511)
- SUBADRS:The sub-address of the decoder (0-3)
- STATE:	1=on (set), 0=off (clear)
Returns: Nothing

DCC Packet format:

    10AAAAAA 1aaa1SSC

Where:

    aaaAAAAAA:	Accessory address (a's are ~'d)

- SS:		Sub address
- C:		Target state 1=on, 0=off

Returns: Nothing

### Power ON/OFF to the track

Neither of these commands have any functional scope in the DCC
protocol as these directly operate against the Arduino itself.

Power ON the track(s):

    <1>

Returns: <p1>

Power OFF the track(s):

    <0>

Returns: <p0>
    
### Note: Errors

Even in "compatibility" mode, the firmware will still send back error reports, these take the format:

        <# ERR ARG>
Where:
- ERR: Firmware specific numeric value indicating error detected
- ARG: Error specific additional information

Summary of Error numbers:

Symbolic name|Numeric Value|General Meaning
-------------|-------------|---------------
NO_ERROR|0|No error - should not be returned
ERROR_QUEUE_OVERFLOW|1|Internal queue for catching errors pending bandwidth to return them has overflowed, the argument is the number of times
ERROR_REPORT_FAIL|2|The firmware was unable to queue a command reponse as the output queue was full (partial reply might be sent)
BIT_TRANS_OVERFLOW|3|The conversion of a DCC byte level command to a bit stream format failed as the result is too big
DCC_COMMAND_OVERFLOW|4|Contruction of a DCC byte level packet failed as the result is too big
UNRECOGNISED_COMMAND|5|Command letter not recognised as a valid command
INVALID_BUFFER_NUMBER|6|Buffer number outside valid range (0..N,for N see firmware)
INVALID_ARGUMENT_COUNT|7|Incorrect number of arguments for specified command
INVALID_ADDRESS|8|Invalid DCC address
INVALID_SPEED|9|Invalid speed specified (valid values -1, 0..126)
INVALID_DIRECTION|10|Invalid direction (valid values 1, forwards, 0 backwards)
INVALID_STATE|11|Invalid state (0 off, 1 on)
INVALID_CV_NUMBER|12|Invalid CV number (valid values 1..1024)
INVALID_FUNC_NUMBER|13|Invalid function number (valid values 0 to 28)
INVALID_BIT_NUMBER|14|Invalid bit number (valid values 0 to 7)
INVALID_BIT_VALUE|15|Invalid bit value supplied (0 or 1, obviously)
INVALID_BYTE_VALUE|16|Invalid byte value (valid from 0 to 255)
INVALID_WORD_VALUE|17|Invalid word value (valid from 0 to 32767)
COMMAND_REPORT_FAIL|18|Firmware unable to send command report back to host computer
TRANSMISSION_BUSY|19|Firmware unable to locate empty transmisionn buffer for requested DCC command
COMMAND_QUEUE_FAILED|20|Firmware unable to queu pending DCC command (all pending buffers in use)
POWER_NOT_OFF|21|Power transitions only allowed between OFF and another state
POWER_OVERLOAD|22|Consistent high level power drain detected
POWER_SPIKE|23|Instant power level reading too high (short circuit?) 
ASSERT_FAILED|99|Firmware condition assertion failed (arg provides firmware source line number)

## DCC Generator Command Summary

For the moment the following commands are described in outline
only; details to be provided.

Serial port configured to 38400 baud.

### Mobile decoder set speed and direction

    [M ADRS SPEED DIR] -> [M ADRS SPEED DIR]

- ADRS:	The short (1-127) or long (128-10239) address of the engine decoder
- SPEED:	Throttle speed from 0-126, or -1 for emergency stop
- DIR:	1=Forward, 0=Reverse

### Accessory decoder set state

    [A ADRS STATE] -> [A ADRS STATE]

- ADRS:	The combined address of the decoder (1-2048)
- STATE:	1=on (set), 0=off (clear)

### Mobile decoder set function state

    [F ADRS FUNC VALUE] -> [F ADRS FUNC STATE]

- ADRS:	The short (1-127) or long (128-10239) address of the engine decoder
- FUNC:	The function number to be modified (0-21)
- VALUE:	1=Enable, 0=Disable
- STATE:	1=Confirmed, 0=Failed

### Enable/Disable Power to track

    [P STATE] -> [P STATE]

- STATE: 1=On, 0=Off

### Set CV value (Programming track)

    [S CV VALUE] -> [S CV VALUE STATE]

- CV:	Number of CV to set (1-1024)
- VALUE:	8 bit value to apply (0-255)
- STATE:	1=Confirmed, 0=Failed

#### Verify CV value (Programming track)

    [V CV VALUE] -> [V CV VALUE STATE]

- CV:	Number of CV to set (1-1024)
- VALUE:	8 bit value to apply (0-255)
- STATE:	1=Confirmed, 0=Failed

### Verify CV bit value (Programming track)

Compare the specified CV bit with the supplied
value, if they are the same, return 1, otherwise
(or in the case of failure) return 0.

    [R CV BIT VALUE] -> [R CV BIT STATE]

- CV:	Number of CV to set (1-1024)
- BIT:	Bit number (0 LSB - 7 MSB)
- VALUE:	0 or 1
- STATE:	1=Confirmed, 0=Failed

### Asynchronous data returned from the firmware

Change in Power state:

    -> [P STATE]

Current power consumption of the system:

    -> [L LOAD]

- LOAD: Figure between 0 and 1023

Error detected by the firmware

    -> [E ERR ARG]

- ERR:	Error number giving nature of problem
- ARG:	Additional information data, nature dependant on the error number.


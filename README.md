# DCCGenerator
Firmware for an Arduino UNO and motor shield to generate an NMRA DCC signal for operating a model railway

For the moment, simply reading the source code shuld provide an insight into what this is and how it does it.

I will come back to this and extend the descritpion.



## DCC Plus Plus Command Summary

The following summary provides the sub-set of DCC commands
supported by the DCCpp Arduino Firmware which this code will
try to emulate.

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


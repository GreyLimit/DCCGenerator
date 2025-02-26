//
//	DCC.cpp
//	=======
//
//	The implementation of the DCC generator
//

#include "DCC.h"
#include "Task.h"
#include "Console.h"
#include "Clock.h"
#include "Stats.h"


//
//	Private support routines.
//	-------------------------
//

//
//	Copy a DCC command to a new location and append the parity data.
//	Returns length of data in the target location.
//
byte DCC::copy_with_parity( byte *dest, byte *src, byte len ) {
	
	STACK_TRACE( "byte DCC::copy_with_parity( byte *dest, byte *src, byte len )" );
	
	byte	p, i;

	ASSERT( len > 0 );

	p = 0;
	for( i = 0; i < len; i++ ) p ^= ( *dest++ = *src++ );
	*dest = p;
	return( len+1 );
}

//
//	Define a routine to release one (one=true) or all (one=false) pending packets in a list.
//
DCC::pending_packet *DCC::release_pending_recs( DCC::pending_packet *head, bool one ) {
	
	STACK_TRACE( "DCC::pending_packet *DCC::release_pending_recs( DCC::pending_packet *head, bool one )" );
	
	pending_packet	*ptr;

	//
	//	We either release one record and return the address of the remaining
	//	records (when one is true) or we release them all and return NULL
	//	(when one is false).
	//
	while(( ptr = head ) != NIL( pending_packet )) {
		head = ptr->next;
		ptr->next = _free_packets;
		_free_packets = ptr;
		if( one ) break;
	}
	//
	//	Done.
	//
	return( head );
}

//
//	DCC Packet to bit stream conversion routine.
//	--------------------------------------------
//

//
//	Define a routine which will convert a supplied series of bytes
//	into a DCC packet defined as a series of bit transitions (as used
//	in the transmission buffer).
//
//	This routine is not responsible for the meaning (valid or otherwise)
//	of the bytes themselves, but *is* required to construct the complete
//	DCC packet formation including the preamble and inter-byte bits.
//
//	The code assumes it is being placed into a buffer of BIT_TRANSITIONS
//	bytes (as per the transmission buffers).
//
//	Returns true on success, false otherwise.
//
bool DCC::pack_command( byte *cmd, byte clen, byte preamble, byte postamble, byte *buf ) {
	
	STACK_TRACE( "bool DCC::pack_command( byte *cmd, byte clen, byte preamble, byte postamble, byte *buf )" );
	
	byte	l, b, c, v, s;

	ASSERT( preamble >= short_preamble );
	ASSERT( postamble >= 1 );

	//
	//	Start with a preamble of "1"s.
	//
	*buf++ = preamble;
	l = bit_transitions-1;

	//
	//	Prime pump with the end of header "0" bit.
	//
	b = 0;			// Looking for "0"s. Will
				// be value 0x80 when looking
				// for "1"s.
	c = 1;			// 1 zero found already.

	//
	//	Step through each of the source bytes one at
	//	a time..
	//
	while( clen-- ) {
		//
		//	Get this byte value.
		//
		v = *cmd++;
		//
		//	Count off the 8 bits in v
		//
		for( s = 0; s < 8; s++ ) {
			//
			//	take bits from from the MSB end
			//
			if(( v & 0x80 ) == b ) {
				//
				//	We simply increase the bit counter.
				//
				if( c == maximum_bit_iterations ) return( false );
				c++;
			}
			else {
				//
				//	Time to flip to the other bit.
				//
				if(!( --l )) return( false );

				*buf++ = c;
				c = 1;
				b ^= 0x80;
			}
			//
			//	Shift v over for the next bit.
			//
			v <<= 1;
		}
		//
		//	Now add inter-byte bit "0", or end of packet
		//	bit "1" (clen will be 0 on the last byte).
		//	Remember we use the top bit in b to indicate
		//	which bit we are currently counting.
		//
		if(( clen? 0: 0x80 ) == b ) {
			//
			//	On the right bit (which ever bit that
			//	is) so we simply need to add to the
			//	current bit counter.
			//
			if( c == maximum_bit_iterations ) return( false );
			c++;
		}
		else {
			//
			//	Need the other bit so save the count so
			//	far then flip to the other bit.
			//
			if(!( --l )) return( false );

			*buf++ = c;
			c = 1;
			b ^= 0x80;
		}
	}
	//
	//	Finally place the bit currently being counted, which
	//	must always be a "1" bit (end of packet marker).
	//

	ASSERT( b == 0x80 );
	
	if(!( --l )) return( false );

	//
	//	Here we have the post amble to append.  Rather than
	//	abort sending the command if adding the postamble
	//	exceeds the value of MAXIMUM_BIT_ITERATIONS, we will
	//	simply trim the value down to MAXIMUM_BIT_ITERATIONS.
	//
	if(( b = ( maximum_bit_iterations - c )) < postamble ) postamble = b;
	c += postamble;
	
	*buf++ = c;
	//
	//	Mark end of the bit data.
	//
	if(!( --l )) return( false );

	*buf = 0;
	return( true );
}

//
//	DCC composition routines
//	------------------------
//
//	The following routines are used to create individual byte oriented
//	DCC commands.
//

//
//	Create a speed and direction packet for a specified target.
//	Returns the number of bytes in the buffer.
//
byte DCC::compose_motion_packet( byte *command, word adrs, byte speed, byte dir ) {
	
	STACK_TRACE( "byte DCC::compose_motion_packet( byte *command, word adrs, byte speed, byte dir )" );
	
	byte	len;

	ASSERT( command != NIL( byte ));
	ASSERT( DCC_Constant::valid_mobile_target( adrs ));
	ASSERT( DCC_Constant::valid_mobile_speed( speed ));
	ASSERT( DCC_Constant::valid_mobile_direction( dir ));

	if( adrs > DCC_Constant::maximum_short_address ) {
		command[ 0 ] = 0b11000000 | ( adrs >> 8 );
		command[ 1 ] = adrs & 0b11111111;
		len = 2;
	}
	else {
		command[ 0 ] = adrs;
		len = 1;
	}
	//
	//	NMRA Document "S-9.2.1"
	//	Advanced Operations Instruction (001)
	//	format: 001CCCCC 0 DDDDDDDD
	//	128 set Speed/Dir: CCCCC = 0b11111
	//	D7: Direction (1=forwards, 0=backwards)
	//	D6-0: Speed, 0: stop, 1: Emergency Stop, 2-127: velocity.
	//				
	command[ len++ ] = 0b00111111;
	command[ len++ ] = ( dir << 7 )| speed;
	//
	//	Done.
	//
	return( len );
}

//
//	Create an accessory modification packet.  Return number of bytes
//	used by the command.
//
byte DCC::compose_accessory_change( byte *command, word adrs, byte subadrs, byte state ) {
	
	STACK_TRACE( "byte DCC::compose_accessory_change( byte *command, word adrs, byte subadrs, byte state )" );
	
	ASSERT( command != NIL( byte ));
	ASSERT( DCC_Constant::valid_accessory_address( adrs ));
	ASSERT( DCC_Constant::valid_accessory_sub_address( subadrs ));
	ASSERT( DCC_Constant::valid_accessory_state( state ));

	command[ 0 ] = 0b10000000 | ( adrs & 0b00111111 );
	command[ 1 ] = ((( adrs >> 2 ) & 0b01110000 ) | ( subadrs << 1 ) | state ) ^ 0b11111000;
	//
	//	Done.
	//
	return( 2 );
}

//
//	Create a Function set/reset command, return number of bytes used.
//
//	Until I can find a more focused mechanism for modifying the functions
//	this routine has to be this way.  I guess a smarter data driven approach
//	could be used to generate the same output, but at least this is clear.
//
byte DCC::compose_function_change( byte *command, word adrs, byte func, bool on ) {
	
	STACK_TRACE( "byte DCC::compose_function_change( byte *command, word adrs, byte func, bool on )" );
	
	ASSERT( command != NULL );
	ASSERT( DCC_Constant::valid_mobile_target( adrs ));
	ASSERT( DCC_Constant::valid_function_number( func ));

	if( function_cache.update( adrs, func, on )) {
		byte	len;

		//
		//	Function has changed value, update the corresponding decoder.
		//
		if( adrs > DCC_Constant::maximum_short_address ) {
			command[ 0 ] = 0b11000000 | ( adrs >> 8 );
			command[ 1 ] = adrs & 0b11111111;
			len = 2;
		}
		else {
			command[ 0 ] = adrs;
			len = 1;
		}
		//
		//	We have to build up the packets depending on the
		//	function number to modify.
		//
		if( func <= 4 ) {
			//
			//	F0-F4		100[F0][F4][F3][F2][F1]
			//
			command[ len++ ] =	0x80	| function_cache.get( adrs, 0, 0x10 )
							| function_cache.get( adrs, 1, 0x01 )
							| function_cache.get( adrs, 2, 0x02 )
							| function_cache.get( adrs, 3, 0x04 )
							| function_cache.get( adrs, 4, 0x08 );
		}
		else if( func <= 8 ) {
			//
			//	F5-F8		1011[F8][F7][F6][F5]
			//
			command[ len++ ] =	0xb0	| function_cache.get( adrs, 5, 0x01 )
							| function_cache.get( adrs, 6, 0x02 )
							| function_cache.get( adrs, 7, 0x04 )
							| function_cache.get( adrs, 8, 0x08 );
		}
		else if( func <= 12 ) {
			//
			//	F9-F12		1010[F12][F11][F10][F9]
			//
			command[ len++ ] =	0xa0	| function_cache.get( adrs, 9, 0x01 )
							| function_cache.get( adrs, 10, 0x02 )
							| function_cache.get( adrs, 11, 0x04 )
							| function_cache.get( adrs, 12, 0x08 );
		}
		else if( func <= 20 ) {
			//
			//	F13-F20		11011110, [F20][F19][F18][F17][F16][F15][F14][F13]
			//
			command[ len++ ] =	0xde;
			command[ len++ ] =		  function_cache.get( adrs, 13, 0x01 )
							| function_cache.get( adrs, 14, 0x02 )
							| function_cache.get( adrs, 15, 0x04 )
							| function_cache.get( adrs, 16, 0x08 )
							| function_cache.get( adrs, 17, 0x10 )
							| function_cache.get( adrs, 18, 0x20 )
							| function_cache.get( adrs, 19, 0x40 )
							| function_cache.get( adrs, 20, 0x80 );
		}
		else {
			//
			//	F21-F28		11011111, [F28][F27][F26][F25][F24][F23][F22][F21]
			//
			command[ len++ ] =	0xdf;
			command[ len++ ] =		  function_cache.get( adrs, 21, 0x01 )
							| function_cache.get( adrs, 22, 0x02 )
							| function_cache.get( adrs, 23, 0x04 )
							| function_cache.get( adrs, 24, 0x08 )
							| function_cache.get( adrs, 25, 0x10 )
							| function_cache.get( adrs, 26, 0x20 )
							| function_cache.get( adrs, 27, 0x40 )
							| function_cache.get( adrs, 28, 0x80 );
		}
		return( len );
	}
	//
	//	We have to "do" something..
	//	.. so we substitute in an idle packet.
	//
	command[ 0 ] = 0xff;
	command[ 1 ] = 0x00;
	return( 2 );
}

//
//	Create one part of the bulk function setting command.  Which part
//	is set by the range argument.
//
byte DCC::compose_function_block( byte *command, word adrs, byte *state, byte fn[ DCC_Constant::bit_map_array ]) {
	
	STACK_TRACE( "byte DCC::compose_function_block( byte *command, word adrs, byte *state, byte fn[ DCC_Constant::bit_map_array ])" );
	
	byte	len;

	ASSERT( DCC_Constant::valid_mobile_target( adrs ));
	ASSERT( command != NIL( byte ));
	ASSERT( state != NIL( byte ));
	
	//
	//	Function has changed value, update the corresponding decoder.
	//
	if( adrs > DCC_Constant::maximum_short_address ) {
		command[ 0 ] = 0b11000000 | ( adrs >> 8 );
		command[ 1 ] = adrs & 0b11111111;
		len = 2;
	}
	else {
		command[ 0 ] = adrs;
		len = 1;
	}
	//
	//	Now the value of the state variable tells us which DCC function
	//	setting command we need to create (which we also auto increment
	//	in preparation for the next call).
	//
	switch( (*state)++ ) {
		//
		//	I know the bit manipulation code in the following is
		//	really inefficient, but it is clear and easy to see that
		//	it's correct.  For the time being this is more important
		//	than fast code which is wrong.
		//
		case 0: {
			//
			//	F0-F4		100[F0][F4][F3][F2][F1]
			//
			command[ len++ ] =	0x80	| (( fn[0] & 0x01 )? 0x10: 0 )
							| (( fn[0] & 0x02 )? 0x01: 0 )
							| (( fn[0] & 0x04 )? 0x02: 0 )
							| (( fn[0] & 0x08 )? 0x04: 0 )
							| (( fn[0] & 0x10 )? 0x08: 0 );
			break;
		}
		case 1: {
			//
			//	F5-F8		1011[F8][F7][F6][F5]
			//
			command[ len++ ] =	0xb0	| (( fn[0] & 0x20 )? 0x01: 0 )
							| (( fn[0] & 0x40 )? 0x02: 0 )
							| (( fn[0] & 0x80 )? 0x04: 0 )
							| (( fn[1] & 0x01 )? 0x08: 0 );
			break;
		}
		case 2: {
			//
			//	F9-F12		1010[F12][F11][F10][F9]
			//
			command[ len++ ] =	0xa0	| (( fn[1] & 0x02 )? 0x01: 0 )
							| (( fn[1] & 0x04 )? 0x02: 0 )
							| (( fn[1] & 0x08 )? 0x04: 0 )
							| (( fn[1] & 0x10 )? 0x08: 0 );
			break;
		}
		case 3: {
			//
			//	F13-F20		11011110, [F20][F19][F18][F17][F16][F15][F14][F13]
			//
			command[ len++ ] =	0xde;
			command[ len++ ] =		  (( fn[1] & 0x20 )? 0x01: 0 )
							| (( fn[1] & 0x40 )? 0x02: 0 )
							| (( fn[1] & 0x80 )? 0x04: 0 )
							| (( fn[2] & 0x01 )? 0x08: 0 )
							| (( fn[2] & 0x02 )? 0x10: 0 )
							| (( fn[2] & 0x04 )? 0x20: 0 )
							| (( fn[2] & 0x08 )? 0x40: 0 )
							| (( fn[2] & 0x10 )? 0x80: 0 );
			break;
		}
		case 4: {
			//
			//	F21-F28		11011111, [F28][F27][F26][F25][F24][F23][F22][F21]
			//
			command[ len++ ] =	0xdf;
			command[ len++ ] =		  (( fn[2] & 0x20 )? 0x01: 0 )
							| (( fn[2] & 0x40 )? 0x02: 0 )
							| (( fn[2] & 0x80 )? 0x04: 0 )
							| (( fn[3] & 0x01 )? 0x08: 0 )
							| (( fn[3] & 0x02 )? 0x10: 0 )
							| (( fn[3] & 0x04 )? 0x20: 0 )
							| (( fn[3] & 0x08 )? 0x40: 0 )
							| (( fn[3] & 0x10 )? 0x80: 0 );
			break;
		}
		default: {
			return( 0 );
		}
	}
	//
	//	Done!
	//
	return( len );
}

//
//	Request an empty buffer to be set aside for building
//	a new transmission activity.
//
DCC::trans_buffer *DCC::acquire_buffer( word target, bool mobile, word action, bool overwrite ) {
	
	STACK_TRACE( "DCC::trans_buffer *DCC::acquire_buffer( word target, bool mobile, bool overwrite )" );

	TRACE_DCC( console.print( F( "DCC find " )));
	TRACE_DCC( console.print( target ));
	TRACE_DCC( console.print( F( " mobile " )));
	TRACE_DCC( console.print( mobile ));
	TRACE_DCC( console.print( F( " action " )));
	TRACE_DCC( console.print_hex( action ));
	TRACE_DCC( console.print( F( " overwrite " )));
	TRACE_DCC( console.println( overwrite ));

	//
	//	For the moment the DCC generation code cannot transmit
	//	to the broadcast address as this also implies that all
	//	the targeted transmissions are cancelled and removed.
	//
	//	This is code which is not yet present in this module.
	//
	ASSERT( target != DCC_Constant::broadcast_address );

	trans_buffer	*found;

	//
	//	When target is a valid target address we are looking
	//	for the record already sending to that address before
	//	using an empty record.
	//
	found = NIL( trans_buffer );	// Not found yet.
	
	//
	//	If we want to re-writable record we will need to scan
	//	the circular list of live buffers first.
	//
	if( overwrite ) {

		TRACE_DCC( console.println( F( "DCC Search" )));

		//
		//	Overwrite can only be used with mobile transmissions
		//	which should be obvious as only mobile speed/dir
		//	transmissions can be persistent and therefore need
		//	to be modified.
		//
		ASSERT( mobile );
	
		for( trans_buffer *look = _circle->next; look->state != state_fixed; look = look->next ) {

			TRACE_DCC( console.print( F( "DCC id " )));
			TRACE_DCC( console.print( look->target ));
			TRACE_DCC( console.print( look->mobile? F( " mob " ): F( " acc " )));
			TRACE_DCC( console.println( look->duration ));
			
			if( look->mobile &&( look->target == target )&&( look->duration == 0 )) {
				//
				//	Note the buffer located.
				//
				found = look;
				
				//
				//	Clear any pending records that may be attached.
				//	
				//
				found->pending = release_pending_recs( found->pending, false );

				break;
			}
		} 
	}
	//
	//	If we didn't want to re-writable record, or simply didn't
	//	find the required one, then try to find a free record.
	//
	if( !found ) {
		if(( found = _free_trans )) {

			TRACE_DCC( console.println( F( "DCC Reuse" )));

			_free_trans = found->next;
			
			ASSERT( found->state == state_empty );
			
			_free_buffers--;
		}
		else {
			if(( found = new trans_buffer )) {

				TRACE_DCC( console.println( F( "DCC Create" )));

				found->state = state_empty;
			}
		}
	}
	
	//
	//	If found is empty then we have failed, return
	//	an empty pointer.
	//
	if( !found ) {

		TRACE_DCC( console.println( F( "DCC Failed" )));

		return( NIL( trans_buffer ));
	}

	//
	//	Remember the target address used for this record.
	//	This is required during following record searches
	//	but has no direct impact on the actual address
	//	targeted as this is encoded in the bit patterns.
	//
	found->target = target;
	found->mobile = mobile;
	found->action = action;
	found->duration = 1;
	found->pending = NIL( pending_packet );

	//
	//	Ready to add dcc packets to the record.
	//
	//	Note that, at this point in time, the transmission record is
	//	in one of three states:
	//
	//	State		Rewritable	Reason
	//	-----		----------	------
	//	empty		false		A new "one off" series of commands
	//	empty		true		A speed command for a new target
	//	run		true		A new speed command for an existing target
	//
	return( found );
}

//
//	This is the routine which adds a pending DCC packet to the
//	pending queue on the buffer being constructed.  Returns
//	true if a new pending record has been created or false
//	if this was not possible.
//
bool DCC::extend_buffer( DCC::trans_buffer *rec, byte duration, byte preamble, byte postamble, byte *cmd, byte len ) {
	
	STACK_TRACE( "bool DCC::extend_buffer( DCC::trans_buffer *rec, byte duration, byte preamble, byte postamble, byte *cmd, byte len )" );
	
	pending_packet	*ptr;

	ASSERT( rec != NIL( trans_buffer ));
	ASSERT(( rec->state == state_empty )||( rec->state == state_run ));
	ASSERT( cmd != NIL( byte ));
	ASSERT( len > 0 );
	ASSERT( len < maximum_command );
	
	//
	//	Look for an empty pending record or create one. 
	//
	if(( ptr = _free_packets )) {
		_free_packets = ptr->next;
	}
	else {
		if(!( ptr = new pending_packet )) return( false );
	}
	
	//
	//	There is a spare record available so fill it in.
	//
	ptr->preamble = preamble;
	ptr->postamble = postamble;
	ptr->duration = duration;
	ptr->len = copy_with_parity( ptr->command, cmd, len );
	
	//
	//	We "pre-pend" this pending record to the front
	//	of the list (as this is quick) and allow the buffer
	//	complete routine to reverse the order as the buffer
	//	is activated.
	//
	ptr->next = rec->pending;
	rec->pending = ptr;

	//
	//	Done.
	//
	return( true );
}

//
//	Complete a buffer and prepare it for transmission.
//
bool DCC::complete_buffer( DCC::trans_buffer *rec ) {
	
	STACK_TRACE( "bool DCC::complete_buffer( DCC::trans_buffer *rec )" );
	
	pending_packet	*ptr;

	ASSERT( rec != NIL( trans_buffer ));
	ASSERT(( rec->state == state_empty )||( rec->state == state_run ));

	//
	//	As a point here, if we are "completing" a
	//	transmission record with an empty pending
	//	list then there is nothing to .. complete.
	//
	//	We return false because this is a error, just
	//	not a fatal one.
	//
	if(!( ptr = rec->pending )) return( false );
	
	//
	//	Reverse the pending records list to correct the
	//	order created when they are "pre-pended" onto the
	//	pending list inside the "extend_buffer()" routine.
	//
	rec->pending = NIL( pending_packet );
	while( ptr ) {
		pending_packet *t;

		//
		//	Push onto the front of the list.
		//
		t = ptr;
		ptr = t->next;
		t->next = rec->pending;
		rec->pending = t;
	}

	//
	//	Release the record into action by setting the
	//	state of the record to one which the ISR will
	//	respond to.
	//
	if( rec->state == state_run ) {
		//
		//	We need to get the ISR to pass this
		//	record back to the manager to get the
		//	bit pattern created and start transmission.
		//
		rec->state = state_reload;
	}
	else {
		//
		//	We need to get this record into the circular
		//	queue.  The best way to do this is to add it
		//	to the managers queue and signal it to do the
		//	work.
		//
		rec->state = state_load;
		{
			Critical code;

			rec->next = (trans_buffer *)_manage;
			_manage = rec;
		}
		_manager.release();
	}
	//
	//	Report success.
	//
	return( true );
}

//
//	Cancel construction of a new transmission buffer.
//
void DCC::cancel_buffer( trans_buffer *rec ) {
	
	STACK_TRACE( "void DCC::cancel_buffer( trans_buffer *rec )" );
	
	//
	//	Sanity check - we should only be cancelling
	//	a running or empty transmission record.
	//
	ASSERT( rec != NIL( trans_buffer ));
	ASSERT(( rec->state == state_run )||( rec->state == state_empty ));

	//
	//	How we cancel a buffer could depend on the
	//	state that it is currently in.
	//
	if( rec->state == state_run ) {
		//
		//	We are updating a running buffer - in which case
		//	we clear the pending list and leave it alone.
		//
		rec->pending = release_pending_recs( rec->pending, false );
	}
	else {
		//
		//	We are cancelling an empty buffer.  We clear the
		//	pending list and add it to the free trans list.
		//
		rec->pending = release_pending_recs( rec->pending, false );
		rec->next = _free_trans;
		_free_trans = rec;
		_free_buffers++;
	}
}


//
//	Constructor
//
DCC::DCC( void ) {
	//
	//	Reset everything to the start state.
	//
	_free_buffers = 0;
	_packets_sent = 0;
	_free_trans = NIL( trans_buffer );
	_free_packets = NIL( pending_packet );
	
#if defined( ENABLE_DCC_SYNCHRONISATION )
	//
	//	Start the sync time as zer as this will immediately
	//	be lifted to the first "proper" delay time.
	//
	_irq_sync = 0;
#endif

	//
	//	Set up the fixed idle packet as the constant entry
	//	into the circular buffer list.
	//
	if(!( _circle = new trans_buffer )) ABORT( HEAP_ERR_OUT_OF_MEMORY );
	//
	//	Set buffer content..
	//
	_circle->state = state_fixed;		// Note that this is the fixed record.
	_circle->target = 0;			// No target specified.
	_circle->mobile = false;		// Accessory by default (unnecessary)
	_circle->duration = 0;			// Run for ever, never delete.
						// Copy in idle packet.

	// JEFF
	//
	//	It seems reasonable for the "fixed" record to be
	//	just the transmission of a single "1" bit, not an
	//	idle record at all.  This would have the benefit of
	//	working for both the main and programming tracks and
	//	also reduce the amount of wasted bandwidth.
	//
	//	Need to check the DCC specification on this.
	//
	
	//
	//	The following array of bit transitions define the "DCC Idle Packet".
	//
	//	This packet contains the following bytes:
	//
	//		Address byte	0xff
	//		Data byte	0x00
	//		Parity byte	0xff
	//
	//	This is translated into the following bit stream:
	//
	//		1111...11110111111110000000000111111111
	//
	_circle->bits[ 0 ] = DCC::short_preamble;	// 1s
	_circle->bits[ 1 ] = 1;				// 0s
	_circle->bits[ 2 ] = 8;				// 1s
	_circle->bits[ 3 ] = 10;			// 0s
	_circle->bits[ 4 ] = 9;				// 1s
	_circle->bits[ 5 ] = 0;

	//
	//	Make the  circular list.
	//
	_circle->pending = NIL( pending_packet );
	_circle->reply_when = reply_none;
	_circle->reply[ 0 ] = EOS;
	_circle->next = _circle;
	_circle->prev = &( _circle->next );

	//
	//	Finally set up the controlling pointers.
	//
	_scan = _current = _circle;
	_run = _manage = NIL( trans_buffer );
	
	//
	//	Set up the output variables for an idle packet so the
	//	signal generator sends this out before starting on the
	//	active buffers.
	//
	_bit_string = _current->bits;
	_left = *_bit_string++;
	_side = true;
	_one = true;
}

//
//	Call this routine to get the object going.  After
//	this call is completed the dcc driver object *will*
//	start getting calls to toggle the direction pins to
//	generate the output signal.
//
//	clock out is a pin specifically used export the internal
//	DCC clock from the ISR routine to enable basic operational
//	state verification (ie, is there even a clock working,
//	and what speed is it?).
//
void DCC::initialise( void ) {
	
	STACK_TRACE( "void DCC::initialise( void )" );

	TRACE_DCC( console.print( F( "DCC manager flag " )));
	TRACE_DCC( console.println( _manager.identity()));

	//
	//	Register ourselves with the Memory Heap
	//	for memory recovery processing.
	//
	heap.recover_from( this );

	//
	//	link this object into the task manager so
	//	that the management actions can be called
	//	when required.
	//
	if( !task_manager.add_task( this, &_manager, management_process )) ABORT( TASK_MANAGER_QUEUE_FULL );

#ifdef ENABLE_DCC_SYNCHRONISATION
	//
	//	Link this object into the clock manager to
	//	generate period IRQ synchonrisation recalibrations.
	//
	if( !task_manager.add_task( this, &_recalibrate, recalibrate_process )) ABORT( TASK_MANAGER_QUEUE_FULL );
	if( !event_timer.delay_event( MSECS( dcc_recalibration_period ), &_recalibrate, true )) ABORT( EVENT_TIMER_QUEUE_FULL );
#endif

	//
	//	Now kick of the time critical code that
	//	generates the DCC signal itself.
	//
	{
		Critical code;
		
		//
		//	Set up the DCC signal timer.
		//
		//
		//		Set Timer to default empty values.
		//
		DCC_TCCRnA = 0;		// Set entire DCC_TCCRnA register to 0
		DCC_TCCRnB = 0;		// Same for DCC_TCCRnB
		DCC_TCNTn  = 0;		// Initialise counter value to 0
		//
		//		Set compare match register with an
		//		initial value just to kick off the whole
		//		process.  This will be modified every
		//		alternate interrupt to reflect outputing
		//		1's or 0's as required.
		//
		DCC_COMPARE_REGISTER = timer_digit_1_cycles;

		//
		//		Turn on CTC mode
		//
		DCC_TCCRnA |= bit( DCC_WGMn1 );

		//
		//	Assign the pre scaler value required.
		//
		switch( timer_clock_prescaler ) {
			case 1: {
				//
				//		Set DCC_CSn0 bit for no pre-scaler (factor == 1 )
				//
				DCC_TCCRnB |= bit( DCC_CSn0 );
				break;
			}
			case 8: {
				//
				//		Set DCC_CSn1 bit for pre-scaler factor 8
				//
				DCC_TCCRnB |= bit( DCC_CSn1 );
				break;
			}
			default: {
				//
				//	Not coded for!
				//
				ABORT( PROGRAMMER_ERROR_ABORT );
			}
		}
		//
		//		Enable timer compare interrupt
		//
		DCC_TIMSKn |= ( 1 << DCC_OCIEnA );
	}
}

//
//	This routine is the "manager" of the DCC packets being
//	transmitted.  This is called asynchronously from the
//	irq() routine by releasing a resource on the
//	"_manager" signal.
//
//	The task manager, seeing a resource become available, will
//	initiate the call to this routine.
//
//	The Signal object manages all aspects of of concurrency
//	where the ISR might release multiple transmission records
//	before the manager routine is scheduled.
//
void DCC::process( byte handle ) {
	
	STACK_TRACE( "void DCC::process( void )" );

	//
	//	Is this a managment request.
	//
	if( handle == management_process ) {

#if 0
		{
			console.println( F( "DCC Buffers" ));
			for( trans_buffer *p = _circle->next; p->state != state_fixed; p = p->next ) {
				console.print( '<' );
				console.print( p->target );
				console.print( p->mobile? 'M': 'A' );
				console.println( '>' );
			}
			
			console.println( F( "DCC Manage" ));
			for( trans_buffer *p = (trans_buffer *)_manage; p != NIL( trans_buffer ); p = p->next ) {
				console.print( '<' );
				console.print( p->target );
				console.print( p->mobile? 'M': 'A' );
				console.println( '>' );
			}
		}
#endif

		TRACE_DCC( console.println( F( "DCC management" )));

		ASSERT( _manage != NIL( trans_buffer ));

		trans_buffer	*tb;
		pending_packet	*pp;

		//
		//	This section of code is initiated by the ISR
		//	releasing the _manager signal once for each
		//	buffer that it has placed into the _manage
		//	queue.
		//
		//	The code simply takes the first record off
		//	this list (in a Critical section) and either
		//	loads the next pending record before sending
		//	back to the ISR or adds the record to the free
		//	list.
		//
		{
			Critical code;

			tb = (trans_buffer *)_manage;
			_manage = tb->next;
		}
		
		//
		//	Pending DCC packets to process? (assignment intentional)
		//
		if(( pp = tb->pending )) {
			//
			//	Our tasks here are to convert the pending data into live
			//	data, then get the ISR to insert the record into the
			//	circular queue.
			//
			if( pack_command( pp->command, pp->len, pp->preamble, pp->postamble, tb->bits )) {
				//
				//	Good, set up the remainder of the live parameters.
				//
				tb->duration = pp->duration;
				tb->state = state_run;
				
				//
				//	Now we dispose of the one pending record we have used.
				//
				tb->pending = release_pending_recs( tb->pending, true );
				
				//
				//	If this had a "reply at start" confirmation and
				//	the command we have just lined up is the last one in the
				//	list, then send the confirmation now.
				//
				if(( tb->reply_when == reply_at_start )&&( tb->pending == NIL( pending_packet ))) {
					//
					//	We have just loaded the last pending record, so this
					//	is the one for which the reply is appropriate.
					//
					if( !console.print( tb->reply )) errors.log_error( COMMAND_REPORT_FAIL, tb->target );

					//
					//	Clear reply flag.
					//
					tb->reply_when = reply_none;
				}
				//
				//	Hand over to the ISR via the "_run" list,
				//	another Critical code section.
				//
				{
					Critical code;

					tb->next = (trans_buffer *)_run;
					_run = tb;
				}
			}
			else {
				//
				//	The bit translation failed.
				//
				errors.log_error( TRANSMISSION_BIT_OVERFLOW, tb->target );
				
				//
				//	Scrap all pending records and add to the list of
				//	free buffer records.
				//
				tb->pending = release_pending_recs( tb->pending, false );
				tb->state = state_empty;
				tb->next = _free_trans;
				_free_trans = tb;
				
				//
				//	Note another free buffer available.
				//
				_free_buffers++;
			}
		}
		else {
			//
			//	The pending field is empty.  Before marking the buffer as empty
			//	for re-use, we should check to see if a confirmation is required.
			//
			if( tb->reply_when == reply_at_end ) {
				if( !console.print( tb->reply )) errors.log_error( COMMAND_REPORT_FAIL, tb->target );
			}
			//
			//	Add to the free list
			//
			tb->state = state_empty;
			tb->next = _free_trans;
			_free_trans = tb;

			//
			//	Note a free buffer becomes available.
			//
			_free_buffers++;
		}

		//
		//	Done.
		//
		return;
	}

#ifdef ENABLE_DCC_SYNCHRONISATION
	//
	//	Is this a recalibration request.
	//
	if( handle == recalibrate_process ) {
		if( _delay.last() < ( _irq_sync >> 1 )) {
			
			TRACE_DCC( console.println( F( "DCC recalibrated" )));
			
			_irq_sync -= 1;
		}
		return;
	}
#endif
	
	//
	//	Done.
	//
}

//
//	Define the Interrupt Service Routine, where we do the work.  This is
//	a static routine as it shuold only ever access the static variables
//	defined in the class.
//
void DCC::irq( void ) {

	STACK_TRACE( "void DCC::irq( void )" );

#if defined( ENABLE_DCC_DELAY_REPORT ) || defined( ENABLE_DCC_SYNCHRONISATION )

	byte	delayed;
	
	//
	//	Gather the DCC delay experienced, if compiled in.
	//
	//	We gather the time here, but do not do anything until
	//	after the "fixed time" output signal toggle has taken
	//	place.
	//
	delayed = DCC_COUNTER_REGISTER;
#endif

#ifdef ENABLE_DCC_SYNCHRONISATION
	//
	//	Now we wait for the target delay as calculated by the
	//	previous interrupts (if it hasn't already passed us by).
	//
	//	This does waste time on the CPU, but by doing so attempts
	//	to more accurately in time run the following line of code
	//	and create a more time accurate output signal.
	//
	//	There a risk that this will create an infinite loop
	//	if the value of _irq_sync ever equals or exceeds the
	//	value in the comparison register!  A hard ceiling
	//	needs to be set for the delay sync value.
	//
	while( DCC_COUNTER_REGISTER < _irq_sync );
#endif

	//
	//	We "flip" the output DCC signal now as this is the most
	//	time consistent position to do so.
	//
	dcc_driver.toggle();

#ifdef ENABLE_DCC_SYNCHRONISATION
	//
	//	Now that the time critical element of the ISR code has
	//	been met, we can complete the delay calculation in
	//	readyness for interrupts.
	//
	if(( delayed = _delay.add( delayed )) > _irq_sync ) {
		if( delayed < max_sync ){
			_irq_sync = delayed;
		}
		else {
			_irq_sync = max_sync;
		}
	}
#else
#ifdef ENABLE_DCC_DELAY_REPORT
	//
	//	Just gathering the DCC delay stats.
	//
	_delay.add( DCC_COUNTER_REGISTER );
#endif
#endif

	//
	//	Now undertake the logical flip and subsequent actions.
	//
	//	The variable "side" tells us which half of a transmitted
	//	bit we have *just* started (through the flip above).
	//
	//		True	This is the front half of a new bit.
	//
	//		False	This is the tail half of an old bit.
	//
	//	If it's true (start of new bit) then there *might* be
	//	stuff for us to do: Change bit value (0<->1) or even
	//	start new packet.
	//
	if(( _side = !_side )) {
		//
		//	Starting a new bit, is it more of the same?
		//
		if(!( --_left )) {
			//
			//	"left" counts down the number of similar
			//	bits (either 1s or 0s) which we are
			//	outputing in series.
			//
			//	Getting to zero means it is now time to output a series
			//	of the alternate bits.  We extract the number of
			//	bits to output from the assignment below.
			//
			if(( _left = *_bit_string++ )) {
				//
				//	If left > 0 then there are more bits to send.
				//
				//	Select the correct tick count for the next
				//	next bit (again the assignment is intentional).
				//
				DCC_COMPARE_REGISTER = ( _one = !_one )? timer_digit_1_cycles: timer_digit_0_cycles;
			}
			else {
				//
				//	_left is zero so this set of bit transitions
				//	has been completed; there are no more bits to send,
				//	we have sent a packet!
				//
				//	Will need to be aware that *if* the fixed buffer
				//	becomes a single "1" it will not be a packet.
				//
				_packets_sent++;
				
				//
				//	Before we move on to the next buffer we need to
				//	check the duration flag and act upon it.
				//
				//	If the current buffer is in RUN mode and duration
				//	is greater than 0 then we decrease duration and
				//	if becomes zero then we reset state to LOAD and
				//	move into the managers queue.
				//
				//	At the end of the next block of statements the
				//	current pointer will address the next buffer or
				//	it will be zero.
				//
				if( _current->duration && ( _current->state == state_run )) {
					if(!( --_current->duration )) {
						trans_buffer	*ptr;
						
						//
						//	Duration for this packet has dropped to zero
						//	so we have finished with it.
						//
						//	Check scan first..
						//
						if( _scan == _current ) _scan = _scan->next;
						//
						//	Unhook and move to the manager queue while
						//	moving current to the following buffer.
						//
						ptr = _current;
						//
						ptr->next->prev = ptr->prev;
						_current = *( ptr->prev ) = ptr->next;
						//
						//	Attach to the managers input list and notify
						//	the manager that there is work to do.
						//
						ptr->state = state_load;
						ptr->next = (trans_buffer *)_manage;
						_manage = ptr;
						_manager.release();
					}
					else {
						//
						//	Move onto the next buffer as this buffer still
						//	has time to run yet.
						//
						_current = _current->next;
					}
				}
				else {
					//
					//	Move onto the next buffer as this buffer is
					//	running forever.
					//
					_current = _current->next;
				}

				//
				//	At this point _current now points to the next
				//	buffer to transmit.
				//

				//
				//	The only special case we need to check for is when
				//	the manager has requested that a record is removed
				//	from the circular queue and returned to it (state_
				//	reload).  Any other state is a "run this buffer"
				//	state.
				//
				if( _current->state == state_reload ) {
					//
					//	We have been asked to send this packet
					//	back to the manager for re-work.  This is
					//	required when the manager has added new
					//	pending records but the duration is 0.
					//
					//	Check scan first..
					//
					if( _scan == _current ) _scan = _scan->next;
					//
					//	Unhook and move to manager queue.
					//
					_current->next->prev = _current->prev;
					*( _current->prev ) = _current->next;
					//
					//	Attach to the managers input list and notify
					//	the manager that there is work to do.
					//
					_current->state = state_load;
					_current->next = (trans_buffer *)_manage;
					_manage = _current;
					_manager.release();
					//
					//	put current back into the fixed buffer.
					//
					_current = _circle;
					_bit_string = _current->bits;
				}
				else {
					ASSERT(( _current->state == state_fixed )||( _current->state == state_run ));
					
					//
					//	We just transmit the packet described
					//	in the bit data.
					//
					_bit_string = _current->bits;
				}

				//
				//	Finally we see if there are buffers we need to
				//	install into the circular queue.
				//
				if( _run ) {
					trans_buffer	*ptr;

					//
					//	Unhook new buffer and set state to run.
					//
					ptr = (trans_buffer *)_run;
					_run = ptr->next;
					//
					//	Slip in before the fixed record.
					//
					ptr->state = state_run;
					*( ptr->prev = _circle->prev ) = ptr;
					ptr->next = _circle;
					_circle->prev = &( ptr->next );
				}
				
				//
				//	Initialise the remaining variables required to
				//	output the selected bit stream.
				//
				_one = true;
				DCC_COMPARE_REGISTER = timer_digit_1_cycles;
				_left = *_bit_string++;
			}
		}
	}
}

//
//	API for recovering and decoding data from the active
//	buffers.
//
void DCC::reset_scan( void ) {
	//
	//	To reset we point to the record after the fixed
	//	record at circle.  This might be the same record
	//	but that is absolutely correct, if the buffer
	//	queue/circle is empty.
	//
	_scan = _circle->next;
}
bool DCC::scan_next( word *target, bool *mobile, word *action ) {
	
	ASSERT( target != NIL( word ));
	ASSERT( mobile != NIL( bool ));
	ASSERT( action != NIL( word ));
	
	//
	//	We have to do this in a critical zoen as we
	//	are looking into the circular buffer and this
	//	is really under the control of the IRQ routine.
	//
	Critical code;
	
	//
	//	Finding the fixed record means we have traversed the
	//	whole circular buffer.
	//
	if( _scan->state == state_fixed ) return( false );
	
	//
	//	Anthing else we find we will copy the key data.
	//
	*target = _scan->target;
	*mobile = _scan->mobile;
	*action = _scan->action;

	//
	//	Then move onto the next ..
	//
	_scan = _scan->next;
	
	return( true );
}

//
//	Send a Speed and Direction command to the specified
//	mobile decoder target address.
//
bool DCC::mobile_command( word target, byte speed, byte direction, Buffer_API *reply ) {
	
	STACK_TRACE( "bool DCC::mobile_command( word target, byte speed, byte direction, Buffer_API *reply )" );

	//
	//	Where we construct the DCC packet data.
	//
	trans_buffer			*buf;
	byte				command[ maximum_command ];
	bool				stop;

	TRACE_DCC( console.print( F( "DCC mobile " )));
	TRACE_DCC( console.print( target ));
	TRACE_DCC( console.print( F( " speed " )));
	TRACE_DCC( console.print( speed ));
	TRACE_DCC( console.print( F( " dir " )));
	TRACE_DCC( console.println( direction ));

	//
	//	Sanity checks.
	//
	ASSERT( DCC_Constant::valid_mobile_target( target ));
	ASSERT( DCC_Constant::valid_mobile_speed( speed ));
	ASSERT( DCC_Constant::valid_mobile_direction( direction ));

	//
	//	Find a transmission buffer to suit.
	//
	if(!( buf = acquire_buffer( target, true, speed_and_dir( speed, direction ), true ))) {
		errors.log_error( TRANSMISSION_TABLE_FULL, Protocol::mobile );
		return( false );
	}

	//
	//	Note if we are stopping the mobile decoder
	//
	stop = DCC_Constant::stationary_speed( speed );

	//
	//	Now create and append the command to the pending list.
	//
	if( !extend_buffer( buf, ( stop? TRANSIENT_COMMAND_REPEATS: 0 ), short_preamble, 1, command, compose_motion_packet( command, target, speed, direction ))) {

		TRACE_DCC( console.println( F( "DCC extend failed" )));

		cancel_buffer( buf );
		errors.log_error( TRANSMISSION_PENDING_FULL, Protocol::mobile );
		return( false );
	}

	//
	//	Save the reply to send when we to send it.
	//
	if( reply ) {
		reply->copy( buf->reply, maximum_output );
		buf->reply_when = stop? reply_at_end: reply_at_start;
	}
	else {
		buf->reply_when = reply_none;
	}
	
	//
	//	Finalise the record and kick it off.
	//
	return( complete_buffer( buf ));
}

//
//	Accessory Command
//
bool DCC::accessory_command( word target, byte state, Buffer_API *reply ) {
	
	STACK_TRACE( "bool DCC::accessory_command( word target, byte state, Buffer_API *reply )" );
	
	//
	//	Where we construct the DCC packet data and its
	//	associated reply.
	//
	trans_buffer			*buf;
	byte				command[ maximum_command ];
	
	word				pri_adrs;
	byte				sub_adrs;

	//
	//	Sanity Checks.
	//
	ASSERT( DCC_Constant::valid_accessory_ext_address( target ));
	ASSERT( DCC_Constant::valid_accessory_state( state ));

	//
	//	Break accessory address into primary and sub address
	//
	pri_adrs = DCC_Constant::internal_acc_adrs( target );
	sub_adrs = DCC_Constant::internal_acc_subadrs( target );

	//
	//	Find an unassigned empty transmission buffer.
	//
	if(!( buf = acquire_buffer( target, false, accessory_state( state ), false ))) {
		errors.log_error( TRANSMISSION_TABLE_FULL, Protocol::accessory );
		return( false );
	}
	
	//
	//	Now create and append the command to the pending list.
	//
	if( !extend_buffer( buf, TRANSIENT_COMMAND_REPEATS, short_preamble, 1, command, compose_accessory_change( command, pri_adrs, sub_adrs, state ))) {
		cancel_buffer( buf );
		errors.log_error( TRANSMISSION_PENDING_FULL, Protocol::accessory );
		return( false );
	}

	//
	//	Save the reply to send when we to send it.
	//
	if( reply ) {
		reply->copy( buf->reply, maximum_output );
		buf->reply_when = reply_at_end;
	}
	else {
		buf->reply_when = reply_none;
	}

	//
	//	Finalise the record and kick it off.
	//
	return( complete_buffer( buf ));
}

bool DCC::function_command( word target, byte func, byte state, Buffer_API *reply ) {
	
	STACK_TRACE( "bool DCC::function_command( word target, byte func, byte state, Buffer_API *reply )" );
	
	//
	//	Where we construct the DCC packet data and its
	//	associated reply.
	//
	trans_buffer			*buf;
	byte				command[ maximum_command ];

	//
	//	Sanity Checks.
	//
	ASSERT( DCC_Constant::valid_mobile_target( target ));
	ASSERT( DCC_Constant::valid_function_number( func ));
	ASSERT( DCC_Constant::valid_function_state( state ));

	//
	//	Find an unassigned empty transmission buffer.
	//
	if(!( buf = acquire_buffer( target, true, func_and_state( func, state ), false ))) {
		errors.log_error( TRANSMISSION_TABLE_FULL, Protocol::function );
		return( false );
	}
	
	//
	//	Now create and append the command(s) to the pending list.
	//
	if( state == DCC_Constant::function_toggle ) {
		if( !extend_buffer( buf, TRANSIENT_COMMAND_REPEATS, short_preamble, 1, command, compose_function_change( command, target, func, true ))) {
			cancel_buffer( buf );
			errors.log_error( TRANSMISSION_PENDING_FULL, Protocol::function );
			return( false );
		}
		if( !extend_buffer( buf, TRANSIENT_COMMAND_REPEATS, short_preamble, 1, command, compose_function_change( command, target, func, false ))) {
			cancel_buffer( buf );
			errors.log_error( TRANSMISSION_PENDING_FULL, Protocol::function );
			return( false );
		}
	}
	else {
		if( !extend_buffer( buf, TRANSIENT_COMMAND_REPEATS, short_preamble, 1, command, compose_function_change( command, target, func, ( state == DCC_Constant::function_on )))) {
			cancel_buffer( buf );
			errors.log_error( TRANSMISSION_PENDING_FULL, Protocol::function );
			return( false );
		}
	}
	
	//
	//	Save the reply to send when we to send it.
	//
	if( reply ) {
		reply->copy( buf->reply, maximum_output );
		buf->reply_when = reply_at_end;
	}
	else {
		buf->reply_when = reply_none;
	}
	
	//
	//	Finalise the record and kick it off.
	//
	return( complete_buffer( buf ));
}

bool DCC::state_command( word target, byte speed, byte dir, byte fn[ DCC_Constant::bit_map_array ], Buffer_API *reply ) {
	
	STACK_TRACE( "bool DCC::state_command( word target, byte speed, byte dir, byte fn[ DCC_Constant::bit_map_array ], Buffer_API *reply )" );
	
	//
	//	Where we construct the DCC packet data and its
	//	associated reply.
	//
	trans_buffer			*buf;
	byte				command[ maximum_command ];

	//
	//	The "state" and "length" variables used in the
	//	function mapping code.
	//
	byte				s, l;

	//
	//	Sanity checks - the function bits cannot be checked.
	//
	ASSERT( DCC_Constant::valid_mobile_target( target ));
	ASSERT( DCC_Constant::valid_mobile_speed( speed ));
	ASSERT( DCC_Constant::valid_mobile_direction( dir ));

	//
	//	Find a transmission buffer to suit.
	//
	if(!( buf = acquire_buffer( target, true, speed_and_dir( speed, dir ), true ))) {
		errors.log_error( TRANSMISSION_TABLE_FULL, Protocol::rewrite_state );
		return( false );
	}

	//
	//	Run though the full set of function states.
	//
	s = 0;
	while(( l = compose_function_block( command, target, &s, fn )) > 0 ) {
		//
		//	Add this command to the pending list
		//
		if( !extend_buffer( buf, TRANSIENT_COMMAND_REPEATS, short_preamble, 1, command, l )) {
			cancel_buffer( buf );
			errors.log_error( TRANSMISSION_PENDING_FULL, Protocol::rewrite_state );
			return( false );
		}
	}
	
	//
	//	Now create and append the speed+direction command.
	//
	if( !extend_buffer( buf, ( DCC_Constant::stationary_speed( speed )? TRANSIENT_COMMAND_REPEATS: 0 ), short_preamble, 1, command, compose_motion_packet( command, target, speed, dir ))) {
		cancel_buffer( buf );
		errors.log_error( TRANSMISSION_PENDING_FULL, Protocol::rewrite_state );
		return( false );
	}

	//
	//	Save the reply to send when we to send it.
	//
	if( reply ) {
		reply->copy( buf->reply, maximum_output );
		buf->reply_when = reply_at_start;
	}
	else {
		buf->reply_when = reply_none;
	}
	//
	//	Finalise the record and kick it off.
	//
	return( complete_buffer( buf ));
}

//
//	The memory recovery API
//	=======================
//
bool DCC::release_memory( void ) {
	
	STACK_TRACE( "void DCC::release_memory( void )" );

	bool	r = false;

	//
	//	Clear the list of unused buffer records.
	//
	{
		trans_buffer	*ptr;

		while(( ptr = _free_trans )) {
			_free_trans = ptr->next;
			delete ptr;
			_free_buffers--;
			r = true;
		}

		ASSERT( _free_buffers == 0 );
	}
	//
	//	Clear the list of unused pending packet records.
	//
	{
		pending_packet	*ptr;

		while(( ptr = _free_packets )) {
			_free_packets = ptr->next;
			delete ptr;
			r = true;
		}
	}
	//
	//	Done.
	//
	return( r );
}

//
//	Routines used to access statistical analysis
//
byte DCC::free_buffers( void ) {
	
	STACK_TRACE( "byte DCC::free_buffers( void )" );
	
	return( _free_buffers );
}

word DCC::packets_sent( void ) {
	
	STACK_TRACE( "word DCC::packets_sent( void )" );
	
	Critical	code;
	word		sent;

	sent = _packets_sent;
	_packets_sent = 0;

	TRACE_DCC( console.print( F( "DCC pkts " )));
	TRACE_DCC( console.println( sent ));

	return( sent );
}

#if defined( ENABLE_DCC_DELAY_REPORT ) || defined( ENABLE_DCC_SYNCHRONISATION )
byte DCC::irq_delay( void ) {
	return( _delay.last());
}
#endif


#if defined( ENABLE_DCC_SYNCHRONISATION )
byte DCC::irq_sync( void ) {
	return( _irq_sync );
}
#endif


//
//	Declare the DCC Generator.
//
DCC dcc_generator;

//
//	And its interface to the timer hardware.
//
ISR( DCC_TIMERn_COMPA_vect ) {
	COUNT_INTERRUPT;
	dcc_generator.irq();
}

//
//	EOF
//

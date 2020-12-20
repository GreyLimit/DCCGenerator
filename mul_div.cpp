//
//	calculate r = ( a * b ) / c without register overflow.
//

#include "mul_div.h"

//
//	Define a basic type and some numerical
//	parameters applicable to that type.
//
#define BITS_PER_TYPE	((int)sizeof(MUL_DIV_TYPE)*8)
#define TOP_BIT_TYPE	(1<<(BITS_PER_TYPE-1))

MUL_DIV_TYPE mul_div( MUL_DIV_TYPE a, MUL_DIV_TYPE b, MUL_DIV_TYPE c ) {
	MUL_DIV_TYPE	st, sb;		// product sum top and bottom

	MUL_DIV_TYPE	d, e;		// division result

	MUL_DIV_TYPE	i,		// bit counter
			j;		// overflow check

	st = 0;
	sb = 0;

	d = 0;
	e = 0;

	for( i = 0; i < BITS_PER_TYPE; i++ ) {
		//
		//	Shift sum left to make space
		//	for next partial sum
		//
		st <<= 1;
		if( sb & TOP_BIT_TYPE ) st |= 1;
		sb <<= 1;
		//
		//	Add a to s if top bit on b
		//	is set.
		//
		if( b & TOP_BIT_TYPE ) {
			j = sb;
			sb += a;
			if( sb < j ) st++;
		}
		//
		//	Division.
		//
		d <<= 1;
		if( st >= c ) {
			d |= 1;
			st -= c;
			e++;
		}
		else {
			if( e ) e++;
		}
		//
		//	Shift b up by one bit.
		//
		b <<= 1;
	}
	//
	//	Roll in missing bits.
	//
	for( i = e; i < BITS_PER_TYPE; i++ ) {
		//
		//	Shift across product sum
		//
		st <<= 1;
		if( sb & TOP_BIT_TYPE ) st |= 1;
		sb <<= 1;
		//
		//	Division, continued.
		//
		d <<= 1;
		if( st >= c ) {
			d |= 1;
			st -= c;
		}
	}
	return( d );
}

//
//	EOF
//

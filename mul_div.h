//
//	The Mul_Div header file.
//

#ifndef _MUL_DIV_H_
#define _MUL_DIV_H_

#ifndef MUL_DIV_TYPE
#define  MUL_DIV_TYPE unsigned int
#endif

//
//	Return the result of calculating "( a * b ) / c" without
//	errors caused by the intermediate product exceeding the
//	target type capacity.
//
extern MUL_DIV_TYPE mul_div( MUL_DIV_TYPE a, MUL_DIV_TYPE b, MUL_DIV_TYPE c );

#endif

//
//	EOF
//

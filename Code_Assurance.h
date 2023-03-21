//
//	Debugging Aids
//	==============
//

#ifndef _CODE_ASSURANCE_H_
#define _CODE_ASSURANCE_H_

//
//	We must have the errors code.
//
#include "Errors.h"

//
//	Code Assurance macros.
//
#define ASSERT(v)	do{ if(!(v))errors.log_terminate( CODE_ASSURANCE_ERR_ASSERT, __FILE__, __LINE__); }while(false)
#define ABORT()		do{ errors.log_terminate( CODE_ASSURANCE_ERR_ABORT, __FILE__, __LINE__ ); }while(false)

#endif

//
//	EOF
//


//
//	Debugging Aids
//	==============
//

#ifndef _CODE_ASSURANCE_H_
#define _CODE_ASSURANCE_H_

//
//	We must have the error codes.
//
#include "Errors.h"

//
//	Code Assurance macros.
//
#if defined( DEBUGGING_ENABLED ) || !defined( DISABLE_ASSERTIONS )
#define ASSERT(v)	do{ if(!(v))errors.log_terminate( CODE_ASSURANCE_ERR_ASSERT, F( __FILE__ ), __LINE__); }while(false)
#else
#define ASSERT(v)
#endif

//
//	Abort code is always included as it
//	is not a performance issue.
//
#define ABORT(e)	do{ errors.log_terminate((e), F( __FILE__ ), __LINE__ ); }while(false)

#endif

//
//	EOF
//


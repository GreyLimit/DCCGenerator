//
//	Banner.h
//	========
//
//	Declare the banner output routines.
//

#ifndef _BANNER_H_
#define _BANNER_H_


#include "Environment.h"
#include "Parameters.h"
#include "Configuration.h"
#include "Byte_Queue.h"
#include "FrameBuffer.h"


//
//	Not really an "object" just two routines that provide
//	banner services to either the console of the frame
//	buffer.
//
extern void serial_banner( Byte_Queue_API *console );
extern void framebuffer_banner( FrameBuffer *display );


#endif

//
//	EOF
//

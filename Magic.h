///
///	Magic.h		A macro for generating a magic word value
///			containing date details.
///
///	Copyright (c) 2021 Jeff Penfold.  All right reserved.
///
///	This library is free software; you can redistribute it and/or
///	modify it under the terms of the GNU Lesser General Public
///	License as published by the Free Software Foundation; either
///	version 2.1 of the License, or (at your option) any later version.
///
///	This library is distributed in the hope that it will be useful,
///	but WITHOUT ANY WARRANTY; without even the implied warranty of
///	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
///	Lesser General Public License for more details.
///
///	You should have received a copy of the GNU Lesser General Public
///	License along with this library; if not, write to the Free Software
///	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301
///	USA
///

#ifndef _MAGIC_H_
#define _MAGIC_H_

//
//	MAGIC(y,m,d) -> 16 bit word value
//
//	y	Year (2000-2127)
//	m	Month (1-12)
//	d	Day in Month (1-31)
//
//	Word value bit meaning:
//
//	Bits	From	To	Meaning
//	----	----	--	-------
//	5	0	4	Day in Month
//	4	5	8	Month in Year
//	7	9	15	Year (offset by 2000)
//
#define MAGIC(y,m,d)		(((((y)-2000)&0177)<<9)|(((m)&017)<<5)|((d)&027))

//
//	Provide macros to unwind the magic number
//
#define MAGIC_YEAR(m)		((((m)>>9)&0177)+2000)
#define MAGIC_MONTH(m)		(((m)>>5)&017)
#define MAGIC_DAY(m)		((m)&027)

#endif

//
//	EOF
//

#ifndef XSENS_FILE_H
#define XSENS_FILE_H

#include <stdio.h>
#ifndef XSENS_MONOLITHIC
#ifndef _PSTDINT_H_INCLUDED
#	include "pstdint.h"
#endif
#endif

#ifndef _WIN32
#   include <sys/types.h>
#endif

#ifdef _WIN32
	typedef  __int64	XsensFilePos;
#else
	/* off_t is practically guaranteed not to be 64 bits on non64 bit systems.
	   We'd better explicitly use __off64_t to be sure of it's size.
	*/
	typedef  __off64_t		XsensFilePos;
#endif

// setFilePos defines
#ifdef _WIN32
#	define XSENS_FILEPOS_BEGIN				FILE_BEGIN
#	define XSENS_FILEPOS_CURRENT			FILE_CURRENT
#	define XSENS_FILEPOS_END				FILE_END
#else
#	define XSENS_FILEPOS_BEGIN				SEEK_SET
#	define XSENS_FILEPOS_CURRENT			SEEK_CUR
#	define XSENS_FILEPOS_END				SEEK_END
#endif

#endif	// XSENS_FILE_H

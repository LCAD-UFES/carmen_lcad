/*
 * File: Debug.h
 * Author: Domingo Gallardo, CMU
 * Purpose: Utility to print debug messages
 * 
 *
 * REVISION HISTORY
 *
 * $Log: Debug.h,v $
 * Revision 1.3  1996/08/05 16:08:37  rich
 * Added comments to endifs.
 *
 * Revision 1.2  1995/04/07  05:06:50  rich
 * Fixed GNUmakefiles to find the release directory.
 * Fixed problems found by sgi cc compiler.  It would not compile.
 *
 * Revision 1.1  1995/04/05  18:31:00  rich
 * Moved tview files to a subdirectory.
 *
 * Revision 1.1  1994/05/31  03:25:50  rich
 * Moved Domingo's tview tool into the main tca module.
 *
 * Revision 1.2  1993/08/13  01:42:57  domingo
 * Updated function declarations for compilation under gcc in ANSI C (still
 * a lot of warnings to fix).
 * Added automatic logging.
 *
 *
 * Dec 23 1992 - Domingo Gallardo at School of Computer Science, CMU
 * Created.
 *
 */


#ifndef _DEBUG_H
#define _DEBUG_H

#include "Standard.h"

void Debug(BOOLEAN flag, const char *format, ...);
void Error(const char *format, ...);
void Warning(const char *format, ...);

#endif /* _DEBUG_H */

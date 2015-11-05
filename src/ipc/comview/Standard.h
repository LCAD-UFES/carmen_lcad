/*
 * File: Standard.h
 * Author: Domingo Gallardo, CMU
 * Purpose: Standard definitions
 *
 * REVISION HISTORY
 *
 * $Log: Standard.h,v $
 * Revision 1.6  1996/08/05 16:08:44  rich
 * Added comments to endifs.
 *
 * Revision 1.5  1996/07/18  15:55:17  reids
 * Changes for the New Millennium environment (NMP_IPC)
 *
 * Revision 1.4  1995/06/14  03:24:12  rich
 * Added DBMALLOC_DIR.
 *
 * Revision 1.3  1995/05/31  20:58:37  rich
 * Fixed conflict with tca declarations.
 *
 * Revision 1.2  1995/04/07  05:07:04  rich
 * Fixed GNUmakefiles to find the release directory.
 * Fixed problems found by sgi cc compiler.  It would not compile.
 *
 * Revision 1.1  1995/04/05  18:31:25  rich
 * Moved tview files to a subdirectory.
 *
 * Revision 1.3  1995/03/30  15:50:45  rich
 * DBMALLOC works.  To use "gmake -k -w DBMALLOC=DBMALLOC install"
 * Do not return pointers to local variables.
 *
 * Revision 1.2  1994/11/02  21:39:04  rich
 * Now works for linux machines (i486).
 * Got afs to work on alpha (and hopefully other vendor OS's)
 * Added generic Makefile and asynchronous sender/receiver.
 * Renamed some X11 files and modified routines so we don't get library
 * conflicts.
 *
 * Revision 1.1  1994/05/31  03:26:27  rich
 * Moved Domingo's tview tool into the main tca module.
 *
 * Revision 1.3  1994/05/27  05:33:29  rich
 * Can now read from file that is being written to.
 * Fixed Menu and button names.
 * Fixed Indentation.
 * Added menu item to change orientation of the tree.
 *
 * Revision 1.2  1993/08/13  01:43:12  domingo
 * Updated function declarations for compilation under gcc in ANSI C (still
 * a lot of warnings to fix).
 * Added automatic logging.
 *
 *
 * Jan 11 1993 - Domingo Gallardo at School of Computer Science, CMU
 * Created.
 *
 */

#ifndef _STANDARD_H
#define _STANDARD_H

#include <stdio.h>
#include <malloc.h>
#include <ctype.h>
typedef int BOOLEAN;
typedef int int32;

#define True  1
#define False 0

#define possibleNULL(var,field) (var ? var->field : NULL)
#define sizeArray(var, type)    ((int)(sizeof(var) / sizeof(type)))

#undef MAX
#define MAX(x,y)  ((x) > (y) ? (x) : (y))
#undef MIN
#define MIN(x,y)  ((x) > (y) ? (y) : (x))

typedef void *Pointer;

#endif /* _STANDARD_H */

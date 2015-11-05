
/*
 * File: Memory.h
 * Author: Domingo Gallardo, CMU
 * Purpose: 
 * 
 *
 * REVISION HISTORY
 *
 * $Log: Memory.h,v $
 * Revision 1.3  1996/08/05 16:08:39  rich
 * Added comments to endifs.
 *
 * Revision 1.2  1995/07/08  18:25:40  rich
 * Change all /afs/cs to /afs/cs.cmu.edu to get ride of conflict problems.
 *
 * Revision 1.1  1995/04/05  18:31:10  rich
 * Moved tview files to a subdirectory.
 *
 * Revision 1.3  1995/01/25  00:04:22  rich
 * Release of tca 7.9.  Mostly speed improvements.
 * The cvs binaries may now be located in /usr/local.
 * Formatting changes.
 *
 * Revision 1.2  1994/11/02  21:39:02  rich
 * Now works for linux machines (i486).
 * Got afs to work on alpha (and hopefully other vendor OS's)
 * Added generic Makefile and asynchronous sender/receiver.
 * Renamed some X11 files and modified routines so we don't get library
 * conflicts.
 *
 * Revision 1.1  1994/05/31  03:26:12  rich
 * Moved Domingo's tview tool into the main tca module.
 *
 * Revision 1.3  1994/05/27  05:33:17  rich
 * Can now read from file that is being written to.
 * Fixed Menu and button names.
 * Fixed Indentation.
 * Added menu item to change orientation of the tree.
 *
 * Revision 1.2  1993/08/13  01:43:04  domingo
 * Updated function declarations for compilation under gcc in ANSI C (still
 * a lot of warnings to fix).
 * Added automatic logging.
 *
 *
 * Jan 11 1993 - Domingo Gallardo at School of Computer Science, CMU
 * Created.
 *
 */

#ifndef	_TPARSER_MEMORY_H
#define	_TPARSER_MEMORY_H

/*  Macros  */

#define	_new(t)			((t *)malloc((unsigned) sizeof(t)))
#define	_new_array(t, n)	((t *)malloc((unsigned) sizeof(t) * (n)))
#define	_resize_array(a, t, n)	((t *)realloc((char *)(a), \
					      (unsigned) sizeof(t) * (n)))
#define	_delete(object)		((void)(((object)!=NULL) ? \
					free((char*)(object)) : 0))

#endif /* _TPARSER_MEMORY_H */


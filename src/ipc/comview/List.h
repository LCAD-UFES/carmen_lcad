
/*
 * File: List.h
 * Author: Domingo Gallardo, CMU
 * Purpose: Implementation of functions to manage double linked
 *          lists
 * 
 *
 * REVISION HISTORY
 *
 * $Log: List.h,v $
 * Revision 1.6  1996/08/05 16:08:38  rich
 * Added comments to endifs.
 *
 * Revision 1.5  1996/02/07  15:32:38  reids
 * Rewrote the TCA log file parser to use lex/bison.  Fixed some small things
 *   (e.g., PMonitor is a POINT_MONITOR, not a POLLING_MONITOR).  Fixed a bug
 *   in which a task was not always correctly connected to its parent, in cases
 *   where another task with the same ID (but completed/killed) was still being
 *   displayed.  Extended TaskTree a bit to handle RAP output.
 *
 * Revision 1.4  1995/12/15  01:25:51  rich
 * Fixed the includes.
 *
 * Revision 1.3  1995/05/31  20:58:32  rich
 * Fixed conflict with tca declarations.
 *
 * Revision 1.2  1995/04/07  05:06:55  rich
 * Fixed GNUmakefiles to find the release directory.
 * Fixed problems found by sgi cc compiler.  It would not compile.
 *
 * Revision 1.1  1995/04/05  18:31:08  rich
 * Moved tview files to a subdirectory.
 *
 * Revision 1.2  1994/11/02  21:38:52  rich
 * Now works for linux machines (i486).
 * Got afs to work on alpha (and hopefully other vendor OS's)
 * Added generic Makefile and asynchronous sender/receiver.
 * Renamed some X11 files and modified routines so we don't get library
 * conflicts.
 *
 * Revision 1.1  1994/05/31  03:25:57  rich
 * Moved Domingo's tview tool into the main tca module.
 *
 * Revision 1.3  1994/05/27  05:33:12  rich
 * Can now read from file that is being written to.
 * Fixed Menu and button names.
 * Fixed Indentation.
 * Added menu item to change orientation of the tree.
 *
 * Revision 1.2  1993/08/13  01:42:59  domingo
 * Updated function declarations for compilation under gcc in ANSI C (still
 * a lot of warnings to fix).
 * Added automatic logging.
 *
 *
 * Dec 23 1992 - Domingo Gallardo at School of Computer Science, CMU
 * Created.
 *
 */

#ifndef _LIST_H
#define _LIST_H

typedef struct _ListCell{
  Pointer          item;
  struct _ListCell *next, *previous;
} _ListCell, *ListCell;


void ListCellInit(ListCell cell);
ListCell ListInsertRight(ListCell cell, ListCell item);
ListCell ListInsertLeft(ListCell cell, ListCell item);
ListCell ListForwardN(ListCell cell, int positions);
ListCell ListBackwardN(ListCell cell, int positions);
ListCell ListEnd(ListCell cell);
ListCell ListBegin(ListCell cell);
void ListCons(ListCell list1, ListCell list2);
ListCell ListFind(ListCell cell, Pointer item);

typedef BOOLEAN (*LIST_CONDITION_TYPE)(Pointer,Pointer,Pointer);
ListCell ListFindCondition(ListCell cell, LIST_CONDITION_TYPE condition,
			   Pointer parameter1, Pointer parameter2);
/* Find the condition, working backwards */
ListCell ListFindConditionBack(ListCell cell, LIST_CONDITION_TYPE condition,
			       Pointer parameter1, Pointer parameter2);
ListCell ListDelete(ListCell cell);

typedef void 
(*LIST_ITERATE_TYPE)(Pointer);
void ListIterate(ListCell cell, LIST_ITERATE_TYPE);


#endif /* _LIST_H */

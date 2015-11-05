
/*
 * File: Array.h
 * Author: Domingo Gallardo, CMU
 * Purpose: Array gives functionalities to manage arrays
 *          of pointers, extension of the array, item deletion
 *          and insertion
 * 
 *
 * REVISION HISTORY
 *
 * $Log: Array.h,v $
 * Revision 1.5  1996/08/05 16:08:35  rich
 * Added comments to endifs.
 *
 * Revision 1.4  1996/07/18  15:55:14  reids
 * Changes for the New Millennium environment (NMP_IPC)
 *
 * Revision 1.3  1995/12/15  01:25:48  rich
 * Fixed the includes.
 *
 * Revision 1.2  1995/04/07  05:06:47  rich
 * Fixed GNUmakefiles to find the release directory.
 * Fixed problems found by sgi cc compiler.  It would not compile.
 *
 * Revision 1.1  1995/04/05  18:30:54  rich
 * Moved tview files to a subdirectory.
 *
 * Revision 1.2  1995/01/25  00:04:07  rich
 * Release of tca 7.9.  Mostly speed improvements.
 * The cvs binaries may now be located in /usr/local.
 * Formatting changes.
 *
 * Revision 1.1  1994/05/31  03:25:47  rich
 * Moved Domingo's tview tool into the main tca module.
 *
 * Revision 1.3  1994/05/27  05:33:08  rich
 * Can now read from file that is being written to.
 * Fixed Menu and button names.
 * Fixed Indentation.
 * Added menu item to change orientation of the tree.
 *
 * Revision 1.2  1993/08/13  01:42:54  domingo
 * Updated function declarations for compilation under gcc in ANSI C (still
 * a lot of warnings to fix).
 * Added automatic logging.
 *
 *
 * Jan 11 1993 - Domingo Gallardo at School of Computer Science, CMU
 * Created.
 *
 */

#ifndef _ARRAY_H
#define _ARRAY_H

/* Constants */

#define REALLOC_INCREMENT 20


/* Types */


typedef struct _Array {
  BOOLEAN rewind;            /* Trick to uniform access to first item */
  int size;               /* Number of allocated ListCells */
  int n_items;            /* Number of items in array */
  ListCell current_pos;  /* Current item */
  ListCell list_deleted; /* List of empty positions */
  ListCell list_item;    /* List of busy positions */
  ListCell last_item;
  ListCell last_deleted; 
  _ListCell *array;      /* Array of items */
} _Array, *Array;


/* Macros */

#define ArrayNew()               ArrayNewWithSize(REALLOC_INCREMENT)
#define ArrayResize(array)       ArrayResizeSize(array, REALLOC_INCREMENT)
#define ArrayInsert(array, item) ArrayInsertWithSize(array, item, \
						     REALLOC_INCREMENT)
#define ArrayGet(array)          array->current_pos->item


/* Functions */

Array   ArrayNewWithSize(int n_items);
void    ArrayResizeSize(Array array, int n_items);
void    ArrayInsertWithSize(Array array, Pointer item, int n_items);
void    ArrayIterate(Array array, void (*function)(Pointer));
void    ArrayRewind(Array array);
void    ArrayDelete(Array array);
void    ArrayDeleteItem(Array array, Pointer item);
void    ArrayDeleteAll(Array array);
Pointer ArrayNext(Array array);
Pointer ArrayFind(Array array, Pointer item);
Pointer ArrayFindCondition(Array array, LIST_CONDITION_TYPE condition, 
			   Pointer parameter1, Pointer parameter2);

#endif /* _ARRAY_H */

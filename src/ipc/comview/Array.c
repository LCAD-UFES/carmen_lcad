
/*
 * File: Array.c
 * Author: Domingo Gallardo, CMU
 * Purpose: Array gives functionalities to manage arrays
 *          of pointers, extensions of the array, item deletion
 *          and insertion. Within the  array there is a 
 *          pair of double linked lists (one for deleted items and
 *          other to active items) that give a O(1) cost
 *          to insertions and deletions.
 * 
 *
 * REVISION HISTORY
 *
 * $Log: Array.c,v $
 * Revision 1.5  1996/02/10 23:36:18  rich
 * Ifdef out tests rather than comment them out.  Allows comments in the
 * test routines.
 *
 * Revision 1.4  1996/02/07  15:32:32  reids
 * Rewrote the TCA log file parser to use lex/bison.  Fixed some small things
 *   (e.g., PMonitor is a POINT_MONITOR, not a POLLING_MONITOR).  Fixed a bug
 *   in which a task was not always correctly connected to its parent, in cases
 *   where another task with the same ID (but completed/killed) was still being
 *   displayed.  Extended TaskTree a bit to handle RAP output.
 *
 * Revision 1.3  1995/12/15  01:25:47  rich
 * Fixed the includes.
 *
 * Revision 1.2  1995/04/07  05:06:45  rich
 * Fixed GNUmakefiles to find the release directory.
 * Fixed problems found by sgi cc compiler.  It would not compile.
 *
 * Revision 1.1  1995/04/05  18:30:53  rich
 * Moved tview files to a subdirectory.
 *
 * Revision 1.2  1995/01/25  00:04:01  rich
 * Release of tca 7.9.  Mostly speed improvements.
 * The cvs binaries may now be located in /usr/local.
 * Formatting changes.
 *
 * Revision 1.1  1994/05/31  03:25:45  rich
 * Moved Domingo's tview tool into the main tca module.
 *
 * Revision 1.3  1994/05/27  05:34:30  rich
 * Can now read from file that is being written to.
 * Fixed Menu and button names.
 * Fixed Indentation.
 * Added menu item to change orientation of the tree.
 *
 * Revision 1.2  1993/08/13  02:09:06  domingo
 * Updated function declarations for compilation under gcc in ANSI C (still
 * a lot of warnings to fix).
 * Added automatic logging.
 *
 * Jan 11 1993 - Domingo Gallardo at School of Computer Science, CMU
 * Created.
 *
 */

#include "Standard.h"
#include "List.h"
#include "Array.h"
#include "Memory.h"
#include "Debug.h"

#define DEBUG_ARRAY    False

static void _InitializeListDeleted(_ListCell memory[], int size);

Array ArrayNewWithSize(int size)
{
  Array new_array;
  
  new_array = _new(_Array);
  new_array->array = _new_array(_ListCell, size);
  new_array->size = size;
  new_array->n_items = 0;
  _InitializeListDeleted(new_array->array, size);
  new_array->list_deleted = new_array->array;
  new_array->list_item =  NULL;
  new_array->last_item = NULL;
  new_array->current_pos = NULL;
  new_array->last_deleted = &(new_array->array[size-1]);
  new_array->rewind = True;
  return(new_array);
}

void ArrayResizeSize(Array array, int size)
{
  
  Debug(DEBUG_ARRAY, "Resizing Array to %d\n", size); 
  array->array = _new_array(_ListCell, array->size);
  _InitializeListDeleted(array->array, size);
  if (array->list_deleted ==  NULL)
    array->list_deleted = array->array;
  else 
    ListCons(array->list_deleted, array->array);
}

void ArrayInsertWithSize(Array array, Pointer item, int size)
{
  ListCell list_cell;
  
  Debug(DEBUG_ARRAY, "Inserting %d \n", item);
  if (array->list_deleted == NULL)
    ArrayResizeSize(array, size);
  list_cell = array->list_deleted;
  if (list_cell->next == NULL)
    array->list_deleted = NULL;
  else
    array->list_deleted = ListDelete(list_cell);
  ListCellInit(list_cell);
  list_cell->item = item;
  if (array->n_items == 0)
    array->list_item = list_cell;
  else {
    if (array->current_pos == NULL)
      array->current_pos = array->list_item;
    ListInsertRight(array->current_pos, list_cell);
  }

  array->n_items++;
  array->current_pos = list_cell;
  Debug(DEBUG_ARRAY, "Item in current position %d\n", array->current_pos->item);
  Debug(DEBUG_ARRAY, "Item in first position %d\n", array->list_item->item);
}

void ArrayDelete(Array array)
{
  ListCell delete_cell;
  
  delete_cell = array->current_pos;
  if (delete_cell == NULL) {
    Debug(DEBUG_ARRAY, "Trying to delete a deleted item\n");
    return;
  }
  array->n_items--;
  if (array->n_items == 0) {
    array->list_item = NULL;
    array->current_pos = NULL;
  }
  else if (array->list_item == delete_cell) {
    array->list_item = ListDelete(delete_cell);
    array->current_pos = array->list_item;
  }
  else 
    array->current_pos = ListDelete(delete_cell);
  ListCellInit(delete_cell);
  if (array->list_deleted == NULL)
    array->list_deleted = delete_cell;
  else
    ListInsertRight(array->list_deleted, delete_cell);
}


void ArrayDeleteItem(Array array, Pointer item)
{
  if (ArrayFind(array, item))
    ArrayDelete(array);
}


void ArrayDeleteAll(Array array)
{
  ArrayRewind(array);
  while(array->n_items > 0)
    ArrayDelete(array);
}


Pointer ArrayNext(Array array)
{
  if (array->n_items == 0) {
    Debug(DEBUG_ARRAY, "Trying to Next an empty array\n");
    return(NULL);
  }
  if (array->current_pos == NULL)
    return(NULL);
  if (array->rewind)
    array->rewind = False;
  else 
    array->current_pos = array->current_pos->next;
  if (array->current_pos)
    return(array->current_pos->item);
  else 
    return(NULL);
}


void ArrayRewind(Array array)
{
  array->current_pos = array->list_item;
  array->rewind = True;
}


Pointer ArrayFind(Array array, Pointer item)
{
  ListCell found_pos;
  
  if (array->n_items == 0){
    Debug(DEBUG_ARRAY, "Trying to find in an empty array\n");
    return(NULL);
  }
  found_pos = ListFind(array->list_item, item);
  if (found_pos) {
    array->current_pos = found_pos;
    return(found_pos->item);
  }
  Debug(DEBUG_ARRAY, "Item not found\n"); 
  return(NULL);
}


Pointer ArrayFindCondition(Array array, LIST_CONDITION_TYPE condition, 
			   Pointer parameter1, Pointer parameter2)
{
  ListCell cell;
  
  if (array->n_items == 0){
    Debug(DEBUG_ARRAY, "Trying to find in an empty array\n");
    return(NULL);
  }
  
  cell = ListFindConditionBack(ListEnd(array->list_item), condition,
			       parameter1, parameter2);
  if (cell) {
    array->current_pos = cell;
    return(cell->item);
  }
  return(NULL);
}


void ArrayIterate(Array array, void (*function)(Pointer))
{
  if (array->n_items == 0)
    return;
  ListIterate(array->list_item, function);
}


/*
 * PRIVATE FUNCTIONS
 */



static void _InitializeListDeleted(_ListCell memory[], int size)
{
  int i = 0;
  
  (*memory).previous = NULL;
  while (i < size - 1) {
    (*memory).next = memory+1;
    (*memory).item = NULL;
    i++;
    memory++;
    (*memory).previous = memory-1;
  }
  (*memory).item = NULL;
  (*memory).next = NULL;
}


#ifdef TEST   
void p(int item)
{
  printf("%d ", item);
}

void main(int argc, char **argv)
{
  Array array;
  int i;
  
  array = ArrayNew();
  for (i = 1; i < 1000; i++) {
    ArrayInsert(array, (Pointer) i);
  }
  ArrayInsert(array, (Pointer) 20);
  ArrayInsert(array, (Pointer) 30);
  ArrayDelete(array);
  ArrayInsert(array, (Pointer) 40);
  ArrayFind(array, (Pointer) 10);
   ArrayDelete(array);
  ArrayRewind(array);
  while(ArrayNext(array));
  ArrayIterate(array, p);
}

#endif

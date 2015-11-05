/*
 * File: List.c
 * Author: Domingo Gallardo, CMU
 * Purpose: Functions to manage double linked list of generic
 *          pointers as items
 * 
 *
 * REVISION HISTORY
 *
 * $Log: List.c,v $
 * Revision 1.6  1997/05/29 21:27:24  reids
 * Dealing with some of the subtleties of the TCA log file (Greg & Reid)
 *
 * Revision 1.5  1996/02/10  23:36:20  rich
 * Ifdef out tests rather than comment them out.  Allows comments in the
 * test routines.
 *
 * Revision 1.4  1996/02/07  15:32:37  reids
 * Rewrote the TCA log file parser to use lex/bison.  Fixed some small things
 *   (e.g., PMonitor is a POINT_MONITOR, not a POLLING_MONITOR).  Fixed a bug
 *   in which a task was not always correctly connected to its parent, in cases
 *   where another task with the same ID (but completed/killed) was still being
 *   displayed.  Extended TaskTree a bit to handle RAP output.
 *
 * Revision 1.3  1995/12/15  01:25:49  rich
 * Fixed the includes.
 *
 * Revision 1.2  1995/04/07  05:06:53  rich
 * Fixed GNUmakefiles to find the release directory.
 * Fixed problems found by sgi cc compiler.  It would not compile.
 *
 * Revision 1.1  1995/04/05  18:31:06  rich
 * Moved tview files to a subdirectory.
 *
 * Revision 1.1  1994/05/31  03:25:56  rich
 * Moved Domingo's tview tool into the main tca module.
 *
 * Revision 1.4  1994/05/27  05:34:36  rich
 * Can now read from file that is being written to.
 * Fixed Menu and button names.
 * Fixed Indentation.
 * Added menu item to change orientation of the tree.
 *
 * Revision 1.3  1993/09/07  00:24:33  domingo
 * Fixed almost all the warnings
 *
 * Revision 1.2  1993/08/13  02:09:12  domingo
 * Updated function declarations for compilation under gcc in ANSI C (still
 * a lot of warnings to fix).
 * Added automatic logging.
 *
 * Dec 23 1992 - Domingo Gallardo at School of Computer Science, CMU
 * Created.
 *
 */

#include "Standard.h"
#include "List.h"
#include "Debug.h"

#define DEBUG_LIST    False

void ListCellInit(ListCell cell)
{
  cell->next = NULL;
  cell->previous = NULL;
  cell->item = NULL;
}


ListCell ListInsertRight(ListCell cell, ListCell item)
{
  if (!cell) {
    Debug(DEBUG_LIST, "Cell is NULL\n");
    return(cell);
  }
  item->previous = cell;
  item->next = cell->next;
  if (cell->next) 
    cell->next->previous = item;
  cell->next = item;
  return(cell);
}


ListCell ListInsertLeft(ListCell cell, ListCell item)
{
  if (!cell) {
    Debug(DEBUG_LIST, "Cell is NULL\n");
    return(cell);
  }
  item->next = cell;
  item->previous = cell->previous;
  if (cell->previous) 
    cell->previous->next = item;
  cell->previous = item;
  return(cell);
}


ListCell ListForwardN(ListCell cell, int positions)
{    
  if (!cell) {
    Debug(DEBUG_LIST, "Cell is NULL\n");
    return(cell);
  }
  for (; positions > 0; positions--) {
    cell = cell->next;
    if (cell == NULL) {
      Debug(DEBUG_LIST, "I can't move forward\n");
      return(cell);
    }
  }
  return(cell);
}


ListCell ListBackwardN(ListCell cell, int positions)
{    
  if (!cell) {
    Debug(DEBUG_LIST, "Cell is NULL\n");
    return(cell);
  }
  for (; positions > 0; positions--) {
    cell = cell->previous;
    if (cell == NULL) {
      Debug(DEBUG_LIST, "I can't move backward\n");
      return(cell);
    }
  }
  return(cell);
}


ListCell ListEnd(ListCell cell)
{
  if (!cell) {
    Debug(DEBUG_LIST, "Cell is NULL\n");
    return(cell);
  }
  while(cell->next)
    cell = cell->next;
  return(cell);
}

ListCell ListBegin(ListCell cell)
{
  if (!cell) {
    Debug(DEBUG_LIST, "Cell is NULL\n");
    return(cell);
  }
  while(cell->previous)
    cell = cell->previous;
  return(cell);
}


void ListCons(ListCell list1, ListCell list2)
{
  if (!list1 || !list2) {
    Debug(DEBUG_LIST, "Cell is NULL\n");
    return;
  }
  list1 = ListEnd(list1);
  list2 = ListBegin(list2);
  list1->next = list2;
  list2->previous = list1;
}


ListCell ListFind(ListCell cell, Pointer item)
{
  if (!cell) {
    Debug(DEBUG_LIST, "Cell is NULL\n");
    return(cell);
  }
  while(cell->item != item) {
    cell = cell->next;
    if (cell == NULL) {
      Debug(DEBUG_LIST, "Not found item %\n", item);
      return(NULL);
    }
  }
  return(cell);
}


ListCell ListFindCondition(ListCell cell, LIST_CONDITION_TYPE condition,
			   Pointer parameter1, Pointer parameter2)
{
  if (!cell) {
    Debug(DEBUG_LIST, "Cell is NULL\n");
    return(cell);
  }
  while(!(*condition)(cell->item, parameter1, parameter2)) {
    cell = cell->next;
    if (cell == NULL) {
      Debug(DEBUG_LIST, "Not found condition\n");
      return(NULL);
    }
  }
  return(cell);
}

ListCell ListFindConditionBack(ListCell cell, LIST_CONDITION_TYPE condition,
			       Pointer parameter1, Pointer parameter2)
{
  if (!cell) {
    Debug(DEBUG_LIST, "Cell is NULL\n");
    return(cell);
  }
  while(!(*condition)(cell->item, parameter1, parameter2)) {
    cell = cell->previous;
    if (cell == NULL) {
      Debug(DEBUG_LIST, "Not found condition\n");
      return(NULL);
    }
  }
  return(cell);
}


ListCell ListDelete(ListCell cell)
{
  if (!cell) {
    Debug(DEBUG_LIST, "Cell is NULL\n");
    return(cell);
  }
  if (cell->next == NULL) {
    cell->previous->next = NULL;
    return(cell->previous);
  }
  if (cell->previous == NULL) {
    cell->next->previous = NULL;
    return(cell->next);
  }
  cell->next->previous = cell->previous;
  cell->previous->next = cell->next;
  return(cell->next);
}


void ListIterate(ListCell cell, void (*function)(Pointer))
{
  if (!cell) {
    Debug(DEBUG_LIST, "Cell is NULL\n");
    return;
  }
  while (cell) {
    (*function)(cell->item);
    cell = cell->next;
  }
}




/* ------------------------------------------------ */
/*                   MODULE TEST                    */
/* ------------------------------------------------ */


#ifdef TEST   
#include "Memory.h"

void p(i)
     int i;
{
  printf("%d ", i);
}

int square(i)
     int i;
{
  return(i * i);
}

int cond(i)
     int i;
{
  return(i > 10);
}

main()
{
  ListCell l1, l2, l3;
  
  l1 = _new(_ListCell);
  l2 = _new(_ListCell);
  ListCellInit(l1);
  ListCellInit(l2);
  l1->item = (Pointer) 1;
  l2->item = (Pointer) 2;
  ListInsertRight(l1,l2);
  
  l2 = _new(_ListCell);
  ListCellInit(l2);
  l2->item = (Pointer) 10;
  ListInsertRight(l1,l2);
  
  ListIterate(l1, p);
  
  l2 = ListFind(l1, (Pointer) 2);
  ListDelete(l2);
  printf("\n");
  ListIterate(l1, p);
  
  l2 = _new(_ListCell);
  ListCellInit(l2);
  l2->item = (Pointer) 30;
  ListInsertLeft(l1,l2);
  printf("\n");
  ListIterate(l2, p);
  
  l2 = ListFindCondition(l2, cond);
  printf("\n%d\n",l2->item);
  l1 = l2;
  
  l2 = _new(_ListCell);
  ListCellInit(l2);
  l2->item = (Pointer) 100;
  l3 = _new(_ListCell);
  ListCellInit(l3);
  l3->item = (Pointer) 30;
  ListInsertRight (l2,l3);
  
  ListCons(l1,l2);
  ListIterate(l1, p);
  printf("\n");
  
}

#endif

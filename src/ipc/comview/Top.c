/*
 * File: Top.c
 * Author: Domingo Gallardo, CMU
 * Purpose: Main functions in the application
 *
 * REVISION HISTORY
 *
 * $Log: Top.c,v $
 * Revision 1.14  2000/01/28 17:47:43  reids
 * Upgraded to run under RedHat 6.x (thanks to Oleg Rodionov @ Drexel).
 * Removed lots of compiler warnings.
 *
 * Revision 1.13  1997/06/17 12:37:04  reids
 * Defined the SUNOS5 and SUNOS4 constants.
 *
 * Revision 1.12  97/05/01  20:08:45  robocomp
 * Added prototype definition needed by Linux.
 * 
 * Revision 1.11  1997/05/01 15:44:51  whelan
 * Small bug fixes (can't even remember what they are now).
 *
 * Revision 1.10  97/05/01  15:05:37  reids
 * Fixed compiler flags (SUNOS4 => sun4 || sparc, SUNOS5 => srv4)
 * 
 * Revision 1.9  97/05/01  11:41:24  reids
 * Fix to allow comview to run under Solaris
 * 
 * Revision 1.8  97/02/03  13:28:57  reids
 * Changes made at JPL being added to CMU version.
 * 
 * Revision 1.7  97/02/03  13:17:29  whelan
 * Hmmm... no recent changes, just a commit that should have been done
 * long ago.
 * 
 * Revision 1.6  1996/09/17  15:25:57  whelan
 * Added capability to select all the widgets within a region specified
 * by a rectangle.
 *
 * Revision 1.5  1996/09/06  20:47:21  whelan
 * Extended the maximum size of a buffer to be used for reading in a line
 * from the log file (from 1024 to 4096).
 *
 * Revision 1.4  1996/07/26  18:22:08  rich
 * Fixed warnings.
 *
 * Revision 1.3  1996/07/19  21:04:30  whelan
 * A file can now be opened from within Comview -- does not have to be
 * specified on the command line.
 *
 * Revision 1.2  1996/07/18  16:04:28  reids
 * Changes for the New Millennium environment (NMP_IPC) (plus log history)
 *
 * Revision 1.1  1996/06/04  18:25:02  whelan
 * First release of comview.
 *
 * Revision 1.7  1996/02/13  21:31:55  rich
 * Fixed linux header problems.
 *
 * Revision 1.6  1996/02/10  16:53:53  rich
 * Made private functions static and fixed some forward declarations.
 *
 * Revision 1.5  1996/02/07  15:39:58  reids
 * Cleaned up a bit of the code -- removing extraneous arguments, adding some
 *   "define"s for string constants.  Fixed the initialization of menu items so
 *   that their labels depend on global variables, rather than being hard-coded
 *
 * Revision 1.4  1996/01/31  22:56:01  reids
 * Added automatic updating of (micro) version control numbers.
 * Also, removed dependence in makefile on libtca.a
 *
 * Revision 1.3  1995/12/15  01:26:30  rich
 * Fixed the includes.
 *
 * Revision 1.2  1995/04/07  05:07:37  rich
 * Fixed GNUmakefiles to find the release directory.
 * Fixed problems found by sgi cc compiler.  It would not compile.
 *
 * Revision 1.1  1995/04/05  18:32:15  rich
 * Moved tview files to a subdirectory.
 *
 * Revision 1.1  1994/05/31  03:26:33  rich
 * Moved Domingo's tview tool into the main tca module.
 *
 * Revision 1.4  1994/05/27  05:34:55  rich
 * Can now read from file that is being written to.
 * Fixed Menu and button names.
 * Fixed Indentation.
 * Added menu item to change orientation of the tree.
 *
 * Revision 1.3  1993/09/07  00:24:48  domingo
 * Fixed almost all the warnings
 *
 * Revision 1.2  1993/08/13  02:09:29  domingo
 * Updated function declarations for compilation under gcc in ANSI C (still
 * a lot of warnings to fix).
 * Added automatic logging.
 *
 * Jan 11 1993 - Domingo Gallardo at School of Computer Science, CMU
 * Created.
 *
 */

#if defined(sparc)
#ifdef __svr4__
#define SUNOS5
#else
#define SUNOS4
#endif
#endif

#include <stdio.h>
#include <stdlib.h>

#ifdef linux
#include <sys/ioctl.h>
#include <unistd.h>
#endif
#ifdef SUNOS4
extern void bcopy(const void *, void *, int);
extern int read(int, void *, int);
#endif
#ifdef SUNOS5
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#endif
#include <X11/Intrinsic.h>

#include "Standard.h"
#include "MyString.h"
#include "MsgData.h"
#include "Parser.h"
#include "List.h"
#include "Array.h"
#include "TaskTree.h"
#include "Handler.h"
#include "Top.h"
#include "comview.h"

#include "tcltk.h"

FILE *global_log_file;
/*BOOLEAN _pause = True;*/
unsigned long _next_step_milliseconds = 100;

_PauseCondition _pause_condition = {
  False,          /* active */
  NULL,           /* name */
  -1,             /* id */
  STATUS_NULL     /* status */
};

static int ReadFromFile(FILE *fd, String buffer, 
			String *startPos, String *endPos);
static int ReadFromBuffer(String my_string, String buffer, 
			  String *startPos, String *endPos);
static BOOLEAN CommandInBuffer(String buffer, String startPos, String endPos);

BOOLEAN OpenFile(String file_name)
{   
  global_log_file = fopen(file_name,"r");
  return (global_log_file != NULL);
}

int load_file(ClientData clientData, Tcl_Interp *interp,
		    int argc, char *argv[])
{
  OpenFile(argv[1]);
  return TCL_OK;
}


int OneStepFromFile(FILE *file, char *msg)
{
  MsgData data;
  
  if (!file)
    return (0);
  
  *msg = '\0';

  do {
    if (ReadNlFromFile(file, msg) == EOF) {
      SetNextStep();
      return (0);
    }
    data = ParseMsg(msg);
  } while (data->type == TYPE_NULL);

  return(1);
}

#ifndef NMP_IPC
/*static void timeout_Step(XtPointer closure, XtIntervalId* id)*/
/*{*/
/*  Step((Widget) NULL, (XtPointer) NULL, (XtPointer) NULL); */
/*}*/
#endif

void SetNextStep(void)
{
/*  if (! _pause)
    XtAppAddTimeOut(app_con, _next_step_milliseconds,  timeout_Step,
    NULL); */
}

int ReadNlFromFile(FILE *fd, String my_string)
{
  static char buffer[MAX_LONG_MESSAGE+1];
  static char *startPos = buffer; 
  static char *endPos = buffer; 
  int num_read;
  
  if (!CommandInBuffer(buffer, startPos, endPos)) {
    num_read = ReadFromFile(fd, buffer, &startPos, &endPos);
    if (num_read == 0) 
      return(EOF);
  }
  return(ReadFromBuffer(my_string, buffer, &startPos, &endPos));
}


/******************************************************************************
 *
 * FUNCTION: long numChars(sd)
 *
 * DESCRIPTION:
 * Find out how many characters are available to be read.
 *
 * INPUTS: 
 * int sd;
 *
 * OUTPUTS: 
 *
 * NOTES:
 *
 *****************************************************************************/

static long numChars(int sd)
{
  long available=0;
#ifdef SUNOS5
  off_t currentLoc;
  
  currentLoc = lseek(sd, 0L, SEEK_CUR);
  if (currentLoc >= 0) {
    available = lseek(sd, 0L, SEEK_END) - currentLoc;
    lseek(sd, currentLoc, SEEK_SET);
    return available;
  } else if (errno != ESPIPE)
    return -1;
  else
#endif /* SUNOS5 */
    if (ioctl(sd, FIONREAD, &available) == 0)
      return available;
    else
      return -1;
}

static int ReadFromFile(FILE *fd, String buffer, 
			String *startPos, String *endPos)
{
  int num_read=0;
  int chars_to_read;
  long available=0;
  
  available = numChars(fileno(fd));
  chars_to_read = MIN(MAX_LONG_MESSAGE - (*endPos - *startPos),
		      available);
  if (chars_to_read > 0) {
    num_read = read(fileno(fd), *endPos, chars_to_read);
    *endPos += num_read;
  }
  return(num_read);
}


static int  ReadFromBuffer(String my_string, String buffer, 
			   String *startPos, String *endPos)
{
  String lineEnd;
  
  lineEnd = *startPos;
  while (*lineEnd != '\n') {
    lineEnd++;
    if (lineEnd == *endPos) {
      lineEnd = NULL;
      break;
    }
  }
  if (lineEnd != NULL) {
    *lineEnd = '\0';
    strcpy(my_string, *startPos);
    *startPos = lineEnd+1;
  }
  /* Fix up the buffer. Throw out any consumed lines.*/
  if (*startPos != buffer)
    { /* slide it back and wait for more characters */
#if defined(__svr4__) || defined(linux)
      //memcpy(buffer, *startPos, (*endPos - *startPos)); // @@@ Alberto: memcpy nao pode ser usada quando as regioes de memoria fonte e destino se sobrepoe...
      memmove(buffer, *startPos, (*endPos - *startPos));
#elif (defined(sun4) || defined(sparc))
      bcopy(*startPos, buffer, (*endPos - *startPos));
#else
#error "Unknown platform. You need a bcopy/memcopy call here."
#endif
      *endPos = buffer + (*endPos - *startPos);
      *startPos = buffer;
    }
  return(1);
}


static BOOLEAN CommandInBuffer(String buffer, String startPos, String endPos)
{
  String lineEnd;
  
  if (startPos == endPos)
    return(False);
  lineEnd = startPos;
  while (*lineEnd != '\n') {
    lineEnd++;
    if (lineEnd == endPos)
      return (False);
  }
  return (True);
}


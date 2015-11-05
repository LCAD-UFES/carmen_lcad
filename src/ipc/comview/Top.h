/*
 * File: Top.h
 * Author: Domingo Gallardo, CMU
 * Purpose: Main functions in the application
 *
 * REVISION HISTORY
 *
 * $Log: Top.h,v $
 * Revision 1.3  1996/08/05 16:07:29  rich
 * Added comments to endifs.
 *
 * Revision 1.2  1996/07/19  21:04:34  whelan
 * A file can now be opened from within Comview -- does not have to be
 * specified on the command line.
 *
 * Revision 1.1  1996/06/04  18:25:04  whelan
 * First release of comview.
 *
 * Revision 1.5  1996/02/07  15:39:59  reids
 * Cleaned up a bit of the code -- removing extraneous arguments, adding some
 *   "define"s for string constants.  Fixed the initialization of menu items so
 *   that their labels depend on global variables, rather than being hard-coded
 *
 * Revision 1.4  1995/12/15  01:26:31  rich
 * Fixed the includes.
 *
 * Revision 1.3  1995/04/19  14:35:00  rich
 * Added int32 for use with tca identifiers.
 *
 * Revision 1.2  1995/04/07  05:07:40  rich
 * Fixed GNUmakefiles to find the release directory.
 * Fixed problems found by sgi cc compiler.  It would not compile.
 *
 * Revision 1.1  1995/04/05  18:32:16  rich
 * Moved tview files to a subdirectory.
 *
 * Revision 1.1  1994/05/31  03:26:35  rich
 * Moved Domingo's tview tool into the main tca module.
 *
 * Revision 1.3  1994/05/27  05:33:33  rich
 * Can now read from file that is being written to.
 * Fixed Menu and button names.
 * Fixed Indentation.
 * Added menu item to change orientation of the tree.
 *
 * Revision 1.2  1993/08/13  01:43:16  domingo
 * Updated function declarations for compilation under gcc in ANSI C (still
 * a lot of warnings to fix).
 * Added automatic logging.
 *
 *
 * Jan 11 1993 - Domingo Gallardo at School of Computer Science, CMU
 * Created.
 *
 */

#ifndef _TOP_H
#define _TOP_H

#include "Standard.h"
#include "MsgData.h"
#include <tcl.h>

typedef struct _PauseCondition {
  BOOLEAN active;
  char *name;
  int32 id;
  msgStatus status;
} _PauseCondition, *PauseCondition;


extern FILE *global_log_file;
extern BOOLEAN _pause;
extern _PauseCondition _pause_condition;

BOOLEAN OpenFile(String file_name);
int  OneStepFromFile(FILE *file, char *msg);
void SetNextStep(void);
int ReadNlFromFile(FILE *fd, String my_string);
int load_file(ClientData clientData, Tcl_Interp *interp,
	      int argc, char *argv[]);

#endif /* _TOP_H */

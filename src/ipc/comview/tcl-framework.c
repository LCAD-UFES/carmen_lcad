/*****************************************************************************
 * (c) Copyright 1996 Greg Whelan and Reid Simmons.  All rights reserved.
 *
 * FILE: tcl-framework.c
 *
 * ABSTRACT: Message visualization tool
 *
 * $Source: /afs/cs.cmu.edu/project/TCA/Master/tcaV8/tools/comview/tcl-framework.c,v $ 
 * REVISION HISTORY
 * $Revision: 1.15 $
 * $Date: 1997/05/01 17:39:39 $
 * $Author: whelan $

 * $Log: tcl-framework.c,v $
 * Revision 1.15  1997/05/01 17:39:39  whelan
 * Modified tcl-interface.c to check for latest Tk lib, not Tcl lib.
 *
 * Revision 1.14  97/05/01  15:44:53  whelan
 * Small bug fixes (can't even remember what they are now).
 * 
 * Revision 1.13  97/02/03  13:37:30  reids
 * Changes made at JPL being added to CMU version.
 * 
 * Revision 1.12  97/02/03  13:17:36  whelan
 * Hmmm... no recent changes, just a commit that should have been done
 * long ago.
 * 
 * Revision 1.11  1996/09/17  15:26:10  whelan
 * Added capability to select all the widgets within a region specified
 * by a rectangle.
 *
 * Revision 1.10  1996/09/03  19:04:32  whelan
 * Fixed a few bugs in parser (removed "." as a valid name
 * character). Added support for changing start-up window size from a
 * user's settings file. An activity highlight will be updated when
 * incoming activity effects what should be highlighted.
 *
 * Revision 1.9  1996/08/27  18:56:27  whelan
 * Tcl/Tk Errors are now printed to stderr. This makes startup errors
 * easier to identify.
 *
 * Revision 1.8  1996/08/26  22:26:57  whelan
 * Support added for displaying the hostname of a module in addition to the
 * module's name.
 *
 * Revision 1.7  1996/08/26  19:04:03  whelan
 * 	o Bug fixes and code refinement (unfortunately this doesn't
 * 	  show much of a performance increase, but the code is much
 * 	  cleaner).
 *
 * 	o Additional keyboard control commands.
 *
 * 	o Menu option to toggle logging all data (ie. same as
 * 	  comview's command line "-l" option).
 *
 * 	o Ability to toggle from the "Message" menu whether a
 * 	  particular message is ignored (formerly could only be done
 * 	  from the settings file, and not changed dynamically).
 *
 * 	o A pending bar that can be clicked to highlight corresponding
 * 	  message that is actually "pending".
 *
 * 	o Feature to run through log file until a regular expression
 * 	  is found in the log file.
 *
 * 	o Improved control over colors.
 *
 * Revision 1.6  1996/07/25  18:24:53  whelan
 * Added a "colors" menu that describes what kind of message each of the
 * colors corresponds to, and allows the user to modify the colors.
 *
 * Revision 1.5  1996/07/19  21:04:43  whelan
 * A file can now be opened from within Comview -- does not have to be
 * specified on the command line.
 *
 * Revision 1.4  1996/07/19  14:26:55  whelan
 * Comview directory specified from compilation directory.
 *
 * Revision 1.3  1996/07/18  16:04:31  reids
 * Changes for the New Millennium environment (NMP_IPC) (plus log history)
 *
 ****************************************************************/

#include <stdio.h>
#include <stdlib.h>
#ifndef NMP_IPC
#include "tca/libc.h"
#endif

#include "tcltk.h"

#include <string.h>
#include <math.h>

#include "comview.h"
#include "tcl-framework.h"
#include "Top.h"

#define STARTUP_FILE "comview.tcl"

static char *tcl_RcFileName =  NULL;
char *load_filename;
char *stngs_filename;
char zoom_init_str[255];
char startup_string[255];
char set_dir_string[255];
char file_sel_string[255];

static Tcl_Interp *tcl_interp;
static Tk_Window tcl_mainwindow;

extern int COMVIEW_DEBUG;

/* -------------------- local prototypes ----------------------------- */

int Get_TCA_Data(ClientData, Tcl_Interp *, int, char *[]);
int My_Tcl_Eval(Tcl_Interp *interp, char *s);

/* ------------------------------------------------------------------- */

int Get_TCA_Data(ClientData clientData, Tcl_Interp *interp,
		 int argc, char *argv[])
{
  interp->result=uno_step(interp);

  return TCL_OK;
}

int init_grafx(int argc, char **argv, char *filename, 
	       int file_selected, char *set_filename,
	       char *zoom, int zoom_found, 
	       char *comview_directory,
	       int tcl_prompt)
{
  int c=1; /* bogus hack because we don't want Tk to see the command */
	   /* line arguments (they're not for Tk) */
  
  load_filename=filename;
  stngs_filename=set_filename;
  if (zoom_found) { sprintf(zoom_init_str,"set Scale_Factor %s", zoom); } 
  else { sprintf(zoom_init_str,"set Scale_Factor -1"); } 

  sprintf(set_dir_string, "set COMVIEW_DIRECTORY %s", comview_directory);
  sprintf(file_sel_string, "set FILE_SELECTED %d", file_selected);
  sprintf(startup_string,"source %s/%s", 
	  comview_directory, STARTUP_FILE);

  if (tcl_prompt) {
    /* For debugging purposes this will result in a wish prompt */
    Tk_Main(c, argv, (Tcl_AppInitProc*) Tcl_AppInit);
  } else {
    /* For normal usage, use the Tk main event loop (no prompt) */
    tcl_interp = Tcl_CreateInterp();
#if (TK_MAJOR_VERSION>=4)
    tcl_mainwindow = Tk_MainWindow(tcl_interp);
#else
    tcl_mainwindow = Tk_CreateMainWindow(tcl_interp,(char *)NULL,
					 argv[0],argv[0]);
#endif
    Tcl_AppInit(tcl_interp);
    Tk_MainLoop();
  }

  exit(0);

  return 0;
}

int create_new_module(Tcl_Interp *interp, int number, 
		      char *label, char *hostname)
{
  char foo[255];

  if (COMVIEW_DEBUG) {
    fprintf(stderr, "C Creating New_Module %d \"%s\" \"%s\"\n", 
	    number, label, hostname);
  }
  sprintf(foo, "New_Module %d \"%s\" \"%s\"\n", number, label, hostname);
  My_Tcl_Eval(interp, foo);

  return TCL_OK;
}

/*
 * Tcl_AppInit is called from Tcl_Main
 * after the Tcl interpreter has been created,
 * and before the script file
 * or interactive command loop is entered.
 */
int Tcl_AppInit(Tcl_Interp *interp)
{
  int result;

  /* Initialize packages * Tcl_Init sets up the Tcl library facility. */
  if (Tcl_Init(interp) == TCL_ERROR) {
    return TCL_ERROR;
  }
  if (Tk_Init(interp) == TCL_ERROR) {
    return TCL_ERROR;
  }

  /* Define application-specific commands here. */
  Tcl_CreateCommand(interp, "C_Get_TCA_Data", Get_TCA_Data,
		    (ClientData)NULL, (Tcl_CmdDeleteProc *)NULL);

  Tcl_CreateCommand(interp, "C_Update_Listbox", update_listbox,
		    (ClientData)NULL, (Tcl_CmdDeleteProc *)NULL);
  
  Tcl_CreateCommand(interp, "C_Load_File", load_file,
		    (ClientData)NULL, (Tcl_CmdDeleteProc *)NULL);

  Tcl_CreateCommand(interp, "C_Toggle_Display_All", toggle_display_all,
		    (ClientData)NULL, (Tcl_CmdDeleteProc *)NULL);
  
  Tcl_CreateCommand(interp, "C_Match_Regexp", match_regexp,
		    (ClientData)NULL, (Tcl_CmdDeleteProc *)NULL);
  
  /*
   * Define startup filename. This file is read in
   * case the program is run interactively.
   */
  tcl_RcFileName = STARTUP_FILE;

  result  = My_Tcl_Eval(interp, load_filename);
  result  = My_Tcl_Eval(interp, stngs_filename);
  result += My_Tcl_Eval(interp, file_sel_string);
  result += My_Tcl_Eval(interp, zoom_init_str);
  result += My_Tcl_Eval(interp, set_dir_string);
  result += My_Tcl_Eval(interp, startup_string);

  if (result>0) { exit(0); }

  return TCL_OK;
}

int My_Tcl_Eval(Tcl_Interp *interp, char *s)
{
  int result;

  result = Tcl_Eval(interp, s);

  if (result == TCL_ERROR) {
    fprintf(stderr, "Tcl Error: %s\n", interp->result);
  }
  
  return result;
}

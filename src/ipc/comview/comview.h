/*****************************************************************************
 * (c) Copyright 1996 Greg Whelan and Reid Simmons.  All rights reserved.
 *
 * FILE: comview.h
 *
 * ABSTRACT: Header file for the application
 *
 * AUTHOR: Greg Whelan, CMU
 *
 * $Source: /afs/cs.cmu.edu/project/TCA/Master/tcaV8/tools/comview/comview.h,v $ 
 * $Revision: 1.9 $
 * $Date: 1997/05/29 16:00:40 $
 * $Author: whelan $
 *
 * REVISION HISTORY
 * $Log: comview.h,v $
 * Revision 1.9  1997/05/29 16:00:40  whelan
 * Messages sent from and to the same module are represented as a triangle.
 *
 * Revision 1.8  1996/09/17  17:21:11  whelan
 * Added a Tcl/Tk header file (tcltk.h) so that it's possible to swap
 * using Extended Tcl (has profiling features) and plain old Tcl.
 *
 * Revision 1.7  1996/08/26  19:03:44  whelan
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
 * Revision 1.6  1996/08/05  16:07:31  rich
 * Added comments to endifs.
 *
 * Revision 1.5  1996/07/26  18:22:12  rich
 * Fixed warnings.
 *
 * Revision 1.4  1996/07/19  14:38:49  reids
 * Check if -f parameter is actually given.
 *
 *****************************************************************/

#include "tcltk.h"

#ifndef _COMVIEW_H
#define _COMVIEW_H

char *uno_step(Tcl_Interp *);

int update_listbox(ClientData clientData, Tcl_Interp *interp, 
		   int argc, char *argv[]);

int toggle_display_all(ClientData clientData, Tcl_Interp *interp, 
		   int argc, char *argv[]);

int match_regexp(ClientData clientData, Tcl_Interp *interp, 
		   int argc, char *argv[]);

void Comview_Initialize (void);

#endif /* _COMVIEW_H */

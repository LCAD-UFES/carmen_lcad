/*
 * File: Handler.h
 * Author: Reid Simmons, CMU
 * Purpose: tview X callbacks header file
 *
 * REVISION HISTORY
 *
 * $Log: Handler.h,v $
 * Revision 1.1  1996/06/04 18:24:59  whelan
 * First release of comview.
 *
 * Revision 1.2  1996/02/07  15:39:57  reids
 * Cleaned up a bit of the code -- removing extraneous arguments, adding some
 *   "define"s for string constants.  Fixed the initialization of menu items so
 *   that their labels depend on global variables, rather than being hard-coded
 *
 * Revision 1.1  1995/04/05  18:31:57  rich
 * Moved tview files to a subdirectory.
 *
 */

#ifndef _HANDLER_H
#define _HANDLER_H

#include <X11/Intrinsic.h>

#define LAYOUT_HORIZONTAL_MSG "Layout: Horizontal"
#define LAYOUT_VERTICAL_MSG   "Layout:   Vertical"

#define INFORMS_ON_MSG  "Display informs:  ON"
#define INFORMS_OFF_MSG "Display informs: OFF"

#define AUTO_REFRESH_ON_MSG  "Auto refresh:  ON"
#define AUTO_REFRESH_OFF_MSG "Auto refresh: OFF"

#define AUTO_CENTER_ON_MSG  "Auto center:  ON"
#define AUTO_CENTER_OFF_MSG "Auto center: OFF"

#define ERASE_QUERIES_ON_MSG  "Erase queries:  ON"
#define ERASE_QUERIES_OFF_MSG "Erase queries: OFF"

#define PAUSED_MSG  "....Paused"
#define RUNNING_MSG "Running..."

void Quit(Widget w, XtPointer client_data, XtPointer call_data);
void LoadFile(Widget w, XtPointer client_data, XtPointer call_data);
void layoutToggle(Widget w, XtPointer client_data, XtPointer call_data);
void AutoRefreshToggle(Widget w, XtPointer client_data, XtPointer call_data);
void EraseQueriesToggle(Widget w, XtPointer client_data, XtPointer call_data);
void DisplayInformsToggle(Widget w, XtPointer client_data, 
			  XtPointer call_data);
void AutoCenterToggle(Widget w, XtPointer client_data, XtPointer call_data);
void CenterInTask(Widget w, XtPointer client_data, XtPointer call_data);
void Restart(Widget w, XtPointer call_data, XtPointer client_data);
void PauseWhen(Widget w, XtPointer client_data, XtPointer call_data);
void PauseContinue(Widget w, XtPointer client_data, XtPointer call_data);
void PortholeCallback(Widget w, XtPointer panner_ptr, XtPointer report_ptr);
void PannerCallback(Widget  w, XtPointer client_data, XtPointer report_ptr);
void Step(Widget widget, XtPointer client_data, XtPointer call_data);
void MessageMove(Widget widget, XtPointer client_data, XtPointer call_data);
void ProcessCondition(String condition);

#endif /* _HANDLER_H */

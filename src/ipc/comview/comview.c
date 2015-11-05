/*****************************************************************************
 * (c) Copyright 1996 Greg Whelan and Reid Simmons.  All rights reserved.
 *
 * FILE: comview.c
 *
 * ABSTRACT: Message visualization tool
 *
 * $Source: /afs/cs.cmu.edu/project/TCA/Master/tcaV8/tools/comview/comview.c,v $ 
 * $Author: whelan $
 *
 * REVISION HISTORY
 * $Log: comview.c,v $
 * Revision 1.18  1997/05/29 16:00:38  whelan
 * Messages sent from and to the same module are represented as a triangle.
 *
 * Revision 1.17  97/05/01  15:44:52  whelan
 * Small bug fixes (can't even remember what they are now).
 * 
 * Revision 1.16  97/02/03  13:17:30  whelan
 * Hmmm... no recent changes, just a commit that should have been done
 * long ago.
 * 
 * Revision 1.15  1996/09/17  15:26:02  whelan
 * Added capability to select all the widgets within a region specified
 * by a rectangle.
 *
 * Revision 1.14  1996/09/06  20:47:26  whelan
 * Extended the maximum size of a buffer to be used for reading in a line
 * from the log file (from 1024 to 4096).
 *
 * Revision 1.13  1996/09/03  19:04:24  whelan
 * Fixed a few bugs in parser (removed "." as a valid name
 * character). Added support for changing start-up window size from a
 * user's settings file. An activity highlight will be updated when
 * incoming activity effects what should be highlighted.
 *
 * Revision 1.12  1996/08/26  22:26:48  whelan
 * Support added for displaying the hostname of a module in addition to the
 * module's name.
 *
 * Revision 1.11  1996/08/26  19:03:42  whelan
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
 * Revision 1.10  1996/07/26  18:22:10  rich
 * Fixed warnings.
 *
 * Revision 1.9  1996/07/25  18:24:41  whelan
 * Added a "colors" menu that describes what kind of message each of the
 * colors corresponds to, and allows the user to modify the colors.
 *
 * Revision 1.8  1996/07/19  21:04:37  whelan
 * A file can now be opened from within Comview -- does not have to be
 * specified on the command line.
 *
 * Revision 1.7  1996/07/19  14:38:51  reids
 * Check if -f parameter is actually given.
 *
 * Revision 1.6  1996/07/19  14:26:50  whelan
 * Comview directory specified from compilation directory.
 *
 * Revision 1.5  1996/07/18  16:40:01  whelan
 * COMVIEW_DIRECTORY environment variable is no longer neccessary.
 * It is passed to the C code as a define in the makefile (as the directory
 * which the program is made).
 *
 * Revision 1.4  1996/07/18  16:04:30  reids
 * Changes for the New Millennium environment (NMP_IPC) (plus log history)
 *
 ****************************************************************/

#include <stdlib.h>

#include "Top.h"
#include "Standard.h"
#include "List.h"
#include "Array.h"
#include "MsgData.h"
#include "TaskTree.h"
#include "MyString.h"
#include "MsgData.h"
#include "Parser.h"

#include "tcltk.h"

#include "comview.h"
#include "tclsetup.h"

/* -- local prototypes -------------------------------------------------*/

void initialize_modules(void);
int module_lookup(char *, char *, Tcl_Interp *);
void check_regexp(char *msg, Tcl_Interp *interp);
int CV_OneStepFromFile(FILE *, char *, MsgData *, Tcl_Interp *);
void update_listbox_vars(char *line, int msg_type);
void set_time_vars(Tcl_Interp *interp, int hours, int minutes,
		   int seconds, int m_seconds);
int logged_message_type(int msg_type);
int actual_listbox_update(Tcl_Interp *interp);
int not_processed_msg(msgType type);
int resource_msg(char *msg);
void resource_loc(char *foo, char *msg);
void log_debug(MsgData msgData, int from, int to, int dt, 
	       int prev_time, int from_resourcep);
static void *CV_AddWidgetData(void);

/*----------------------------------------------------------------------*/
 
/* the number of milliseconds that we will artificially separate messages */
/* which are recorded at the same time in the TCA log file. */
/* A value of 2 says that there's a maximum of 4 messages that can occur */
/* at the same time stamp. */
#define TIME_SEPARATION 0

/* Variables that may be exported */
char result[255];

/* Variables that are local only */
static Tcl_HashTable *tablePtr;
static int module_count;
static int msg_count=0;
static int lbox_count=0;
static char listbox_line[MAX_LONG_MESSAGE];
static int listbox_msgt;
static int g_mod;
static int regexp=0;

/* External variables */
extern int COMVIEW_DEBUG;
extern char filename[1024];
extern int file_selected;
extern char set_filename[1024];
extern char zoom[255];
extern int zoom_found;
extern int show_prompt;
extern int display_all;
extern char comview_directory[1024];

void set_time_vars(Tcl_Interp *interp, int hours, int minutes,
		  int seconds, int m_seconds)
{
  char foo[255];

  sprintf(foo, "set_hours %d", hours);
  Tcl_Eval(interp, foo);
  sprintf(foo, "set_mins %d", minutes);
  Tcl_Eval(interp, foo);
  sprintf(foo, "set_secs %d", seconds);
  Tcl_Eval(interp, foo);
  /* this is a bug waiting to happen... gotta fix */
  sprintf(foo, "set_hsecs %d", (m_seconds/10)-1);
  Tcl_Eval(interp, foo);
}

int actual_listbox_update(Tcl_Interp *interp)
{
  char foo[MAX_LONG_MESSAGE+50];

  if (logged_message_type(listbox_msgt)) {
    
    msg_count++;
    sprintf(foo, ".lbo.lbox insert end \"%d: %s\"", msg_count, listbox_line);
    Tcl_Eval(interp, foo);
      
    sprintf(foo, "add_to_lbox_array %d %d", msg_count, lbox_count);
    Tcl_Eval(interp, foo);

  } else if (listbox_msgt >= 0) {

    sprintf(foo, ".lbo.lbox insert end \"%s\"", listbox_line);
    Tcl_Eval(interp, foo);  

  }
    
  if (listbox_msgt >= 0) {
    lbox_count++;
    
    sprintf(foo, "scrollListbox");
    Tcl_Eval(interp, foo);
  }

  return TCL_OK;
}

/* update_listbox will be called from Tcl */

int update_listbox(ClientData clientData, Tcl_Interp *interp,
		   int argc, char *argv[])
{
  if (listbox_msgt) actual_listbox_update(interp);
  
  return TCL_OK;
}

/* toggle_display_all may be called from Tcl */

int toggle_display_all(ClientData clientData, Tcl_Interp *interp,
		       int argc, char *argv[])
{
  display_all = 1 - display_all;
  return TCL_OK;
}


/* logged_message_type checks if the given message type is a message */
/* which involves inter-module communication. */

int logged_message_type(int msg_type)
{
  switch (msg_type) {
  case QUERY:
  case GOAL:
  case COMMAND:
  case INFORM:
  case REPLY:
  case FAILURE:
  case SUCCESS:
  case BROADCAST:
    return 1;
    break;
  default:
    return 0;
  }
}

void update_listbox_vars(char *line, int msg_type)
{
  int n, x;
  
  /* escapify double quotes and open brackets */ 

  for (n=0, x=0; line[n]; n++) {
    if (line[n]=='"') {
      listbox_line[x++]='\\';
      listbox_line[x++]='"';
    } else if (line[n]=='[') {
      listbox_line[x++]='\\';
      listbox_line[x++]='[';
    } else {
      listbox_line[x++]=line[n];
    }
  }

  listbox_line[x]='\0';
  listbox_msgt=msg_type;
}

/* module_lookup takes a module name (string) and returns the module */
/* number. If the module name isn't already in the data structure then */
/* we have to add it into the data structure. */
int module_lookup(char *module_name, char *hostname, Tcl_Interp *interp)
{
  Tcl_HashEntry *entry;
  int exists;

  entry=Tcl_CreateHashEntry(tablePtr, module_name, &exists);

  if (exists) {
    /* not in hash table, create new module */
    Tcl_SetHashValue(entry, module_count);
    create_new_module(interp, module_count, module_name, hostname);
    module_count++;
    return (module_count-1);
  } else {
    /* already in hash table */
    return (int)Tcl_GetHashValue(entry);
  }
}

/* These messages are not used by comview (although the line from the log file
   may appear in the log window if all data is being logged). */
int not_processed_msg(msgType msg_type)
{
  switch (msg_type) {
  case TYPE_NULL:
  case TEMP_CONSTRAINT:
  case POINT_CONSTRAINT:
  case KILLED:
  case UNUSED_INFO:
    return 1;
    break;
  default:
    return 0;
  }
}

void check_regexp(char *msg, Tcl_Interp *interp)
{
  char foo[1024];
  
  sprintf(foo, "Check_Regexp \"%s\"", msg);
  Tcl_Eval(interp, foo);
}

int match_regexp(ClientData clientData, Tcl_Interp *interp,
		       int argc, char *argv[])
{
  sscanf(argv[1], "%d", &regexp);
  return TCL_OK;
}

int CV_OneStepFromFile(FILE *file, char *msg, MsgData *msgData, 
		      Tcl_Interp *interp)
{
  int count = 0;
  int invalid_line;

  *msg = '\0';
  
  /* we loop until a valid message has been read from the log file. if the end
     of the file is reached then the function returns 0. */

  do {

//	if (ReadNlFromFile(file, msg) == EOF) 
//		return (0);
// @@@ O codigo abaixo ee para tratar linhas invalidas e substitui o codigo acima
	do
	{
		if (ReadNlFromFile(file, msg) == EOF) 
			return (0);

		invalid_line = 0;

		if (strstr(msg, "Logging Task Control Server"))	// Ignore this sentence
			invalid_line = 1;

		if (strstr(msg, "Expecting 1 on port 1381"))	// Ignore this sentence
			invalid_line = 1;
			
		if (strstr(msg, "close Module"))		// Ignore this sentence
			invalid_line = 1;
			
		if (strstr(msg, "x_ipcClose"))			// Ignore this sentence
			invalid_line = 1;
			
		if (strstr(msg, "Clearing handler for message"))// Ignore this sentence
			invalid_line = 1;
			
		if (strstr(msg, "Central Abort"))		// Ignore this sentence
			invalid_line = 1;

	} while (invalid_line);
    
    *msgData = ParseMsg(msg);
    //printf("linha = %s\n", msg); // @@@ Alberto: esta lina ee util para debug

    if (COMVIEW_DEBUG) {
      fprintf(stderr, "ReadNl Got : %s\n", msg);
      fprintf(stderr, "Message type is %d\n", (*msgData)->type);
    }

    /* This next segment of code deals with the Log Window. There are a
       number of options that complicate the code; the user can select
       any of the following modes:

        1) Display all lines from log file
        2) Display only lines from the log file which are relevant 
	   to messages being sent between modules (this ignores the data
	   associated with a message, among other things).
        3) Either of the above with the ability to filter out lines
	   that have activity with an ignored module. */

    if ((count) && (listbox_msgt>=0)) { 

      /* if this is the second iteration through the while loop then
	 control never returned to Tcl, therefore the listbox variables
	 were not updated so it is done here. */
      actual_listbox_update(interp);
    }

    if (regexp) { check_regexp(msg, interp); }

    if ((display_all) || (logged_message_type((*msgData)->type))) {

      /* this does not actually update the listbox widget in Tk, it merely
	 updates some global variables that the graphical update function
	 accesses. This allows the Tcl code to look at the line and determine
	 whether or not to allow the line to be displayed. This feature is
	 used in order to ignore particular messages. */
      update_listbox_vars(msg, (*msgData)->type);

    } else {
      /* no show */
      listbox_msgt=-1;
    }

    count=1;
  } while (not_processed_msg((*msgData)->type));

  return (1);
}

/* Given a log file message module name, resource_msg determines whether the
   module is a resource. */
int resource_msg(char *msg)
{
  return (strncmp("Resource ", msg, 9)==0);
}

/* Given a log file message module name that is actually a resource,
   resource_loc returns the module associated with that resource. */
void resource_loc(char *foo, char *msg)
{
  strncpy(foo, msg+9, (strlen(msg))-8);
}

/* when debugging this function is used to display most of the elements of the
   message data structure. */
void log_debug(MsgData msgData, int from, int to, int dt, 
	       int prev_time, int from_resourcep)
{
  fprintf(stderr, "C typ:%d sts:%d nam:%s src:%s dest:%s id:%d\n",
	  msgData->type, msgData->status, msgData->name, 
	  msgData->source, msgData->dest, msgData->id);

  fprintf(stderr, "t:%d h:%d m:%d s:%d msec:%d from_resourcep:%d\n", 
	  msgData->time, msgData->hours, msgData->minutes, 
	  msgData->seconds, msgData->m_seconds, from_resourcep);
	  
  fprintf(stderr, "C from: %d  to: %d dt: %d time: %d prev: %d\n", 
	  from, to, dt, msgData->time, prev_time);
}


char *uno_step(Tcl_Interp *interp)
{
  char msg[MAX_LONG_MESSAGE];
  MsgData msgData;
  int from=0, to=0, dt=0;
  static int prev_time;
  static int first_run=1;
  char source[128];
  char dest[128];
  int from_resourcep=0;
  Task task;
  int valid_message=1;

  /* (1) Read in a line from the log file and parse it (This is all done
         by the CV_OneStepFromFile procedure.
     (2) Check what type of message the line is.  */

  if (CV_OneStepFromFile(global_log_file, msg, &msgData, interp))
      {
	if (COMVIEW_DEBUG) {
	  fprintf(stderr, "%s\n",msg);
	}

	/* For Success or Failure messages the source (from) the module
	   which is sending the message is not logged by central. This
	   information is maintained by the task-tree however, so a call to
	   FindTask will wean this information. */

	if ((msgData->type==FAILURE) || 
	    (msgData->type==SUCCESS)) {

	  task = FindTask(msgData->name, msgData->id, task_ACTIVE);
	  if (!task) {
	    fprintf(stderr, 
		    "Comview error line %d %s : could not FindTask %s\n",
		    __LINE__, __FILE__, msgData->name);
	  }
	  from=(int)task->additional;
	  from_resourcep = 0;
	  to = 0;
	  strcpy(source, "0");
	  strcpy(dest, "0");
	  if (COMVIEW_DEBUG) {
	    fprintf(stderr, "TASK: %s ", task->creator->name);
	    fprintf(stderr, "%d ", task->creation_time);
	    fprintf(stderr, "%d\n",  (int)task->additional);
	  }
	} else if (msgData->type == MODULE_CONNECT) {

	  /* The Tcl code expects the module name in the source element */
	  strcpy(source, msgData->name);
	  /* Read and Parse and additional line: the HOSTNAME */
	  CV_OneStepFromFile(global_log_file, msg, &msgData, interp);
	  strcpy(dest, msgData->name);
	  from=module_lookup(source, dest, interp);

	} else if (msgData->type == MODULE_DISCONNECT) {
	  
	  /* the destination string will hold the hostname (eventually) */
	  strcpy(dest, "0");
	  /* The Tcl code expects the module name in the source element */
	  strcpy(source, msgData->name);
	  from=module_lookup(source, dest, interp);

	} else if (logged_message_type(msgData->type)) {
	  
	  /* check if the message is coming from a resource... */
	  from_resourcep = resource_msg(msgData->source);
	  if (from_resourcep)
	      resource_loc(source, msgData->source);
	  else strcpy(source, msgData->source);
	  
	  /* check if the message is going to a resource... */
	  if (resource_msg (msgData->dest))
	      resource_loc(dest, msgData->dest);
	  else strcpy(dest, msgData->dest);
	  
	  from = module_lookup(source, "", interp);
	  to = module_lookup(dest, "", interp);
	} else {
	  fprintf(stderr, "COMVIEW WARNING: message type not handled %d\n",
		  msgData->type);
	  valid_message = 0;
	}
	
	if ((logged_message_type(msgData->type)) &&
	    (msgData->type != MODULE_CONNECT) && 
	    (msgData->type != MODULE_DISCONNECT) &&
	    (msgData->type != HOSTNAME)) {
	  
	  dt=msgData->time - prev_time;
	  
	  if (first_run) {
	    /* the first time through the prev_time is not going to be set */
	    first_run=0;
	    dt=10;
	    /* we also need to tell Tcl what time the first message */
	    /* occurred at (as a point of reference) */
	    set_time_vars(interp, msgData->hours, msgData->minutes,
			  msgData->seconds, msgData->m_seconds);
	  }

	  prev_time = msgData->time;
	} 

	if (valid_message) {
	  g_mod = to;
	  
	  if (COMVIEW_DEBUG) 
	      log_debug(msgData, from, to, dt, prev_time, from_resourcep); 
	  
	  sprintf(result, "list %d \"%s\" %d \"%s\" %d %d \"%s\" %d %d",
		  from, source, to, dest, msgData->type,
		  msgData->status, msgData->name, dt, from_resourcep);
	  
	  /* if the message was valid, then add it to the task tree */
	  if (msgData->type != TYPE_NULL)
	      TaskTreeProcessNewMessage(msgData);
	} else {
	  sprintf(result, "perror");
	}

      }
  else
      {
	/* This point is reached if there was a parse error, or if the file
	   pointer is currently at the EOF (this is more often the case). */
	sprintf(result, "perror");
	if (COMVIEW_DEBUG)
	    fprintf(stderr, "C Could not parse message: %s\n", msg);
      }
  
  /*  free(&msgData); */
  
  return result;
}

static void *CV_AddWidgetData (void)
{
  if (COMVIEW_DEBUG) fprintf(stderr, "add widget data\n");

  return (void *)(g_mod);
}

/* Comview_Initialize is the main entry point for comview.c, it is
   called from within main.c */
void Comview_Initialize (void)
{
  setAdditionalTaskDataFn(CV_AddWidgetData);

  tablePtr = (Tcl_HashTable *)malloc(sizeof(Tcl_HashTable));
  Tcl_InitHashTable(tablePtr, TCL_STRING_KEYS);

  TaskTreeInitialize(0);
  module_count=1;

  init_grafx(filename, file_selected, set_filename,
	     zoom, zoom_found, comview_directory, show_prompt);

}

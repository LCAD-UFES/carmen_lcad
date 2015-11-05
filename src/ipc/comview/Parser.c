/*
 * File: parser.c
 * Author: Domingo Gallardo, CMU
 * Purpose: Parser of log messages. Translates messages to
 *          a structure named MsgData (see file msgData.h), 
 *          which contains the message's data.
 * 
 *
 * REVISION HISTORY
 *
 * $Log: Parser.c,v $
 * Revision 1.10  1997/05/29 21:27:25  reids
 * Dealing with some of the subtleties of the TCA log file (Greg & Reid)
 *
 * Revision 1.9  1996/07/18  15:55:16  reids
 * Changes for the New Millennium environment (NMP_IPC)
 *
 * Revision 1.8  1996/02/07  15:32:40  reids
 * Rewrote the TCA log file parser to use lex/bison.  Fixed some small things
 *   (e.g., PMonitor is a POINT_MONITOR, not a POLLING_MONITOR).  Fixed a bug
 *   in which a task was not always correctly connected to its parent, in cases
 *   where another task with the same ID (but completed/killed) was still being
 *   displayed.  Extended TaskTree a bit to handle RAP output.
 *
 * Revision 1.7  1996/01/27  21:56:43  rich
 * Pre-release of 8.4.
 *
 * Revision 1.6  1995/12/15  01:25:55  rich
 * Fixed the includes.
 *
 * Revision 1.5  1995/10/07  19:09:42  rich
 * Pre-alpha release of tca-8.2.
 * Added PROJECT_DIR. Use the list of message names from centraMsg.h
 *
 * Revision 1.4  1995/05/31  20:58:35  rich
 * Fixed conflict with tca declarations.
 *
 * Revision 1.3  1995/04/19  14:32:00  rich
 * Added int32 for dealing with tca identifiers.
 *
 * Revision 1.2  1995/04/07  05:07:01  rich
 * Fixed GNUmakefiles to find the release directory.
 * Fixed problems found by sgi cc compiler.  It would not compile.
 *
 * Revision 1.1  1995/04/05  18:31:19  rich
 * Moved tview files to a subdirectory.
 *
 * Revision 1.2  1995/01/25  00:04:28  rich
 * Release of tca 7.9.  Mostly speed improvements.
 * The cvs binaries may now be located in /usr/local.
 * Formatting changes.
 *
 * Revision 1.1  1994/05/31  03:26:22  rich
 * Moved Domingo's tview tool into the main tca module.
 *
 * Revision 1.4  1994/05/27  05:34:48  rich
 * Can now read from file that is being written to.
 * Fixed Menu and button names.
 * Fixed Indentation.
 * Added menu item to change orientation of the tree.
 *
 * Revision 1.3  1993/09/07  00:24:43  domingo
 * Fixed almost all the warnings
 *
 * Revision 1.2  1993/08/13  02:09:24  domingo
 * Updated function declarations for compilation under gcc in ANSI C (still
 * a lot of warnings to fix).
 * Added automatic logging.
 *
 * Jan 1 1993 - Domingo Gallardo at School of Computer Science, CMU
 * Revised parser functions to take into account that the strings in msgData
 * are not initalized. Now ParseName & ParseModuleName return a just created
 * string.
 *
 * Dec 25 1992 - Domingo Gallardo at School of Computer Science, CMU
 * Created.
 *
 */

#include "Standard.h"
#include "MsgData.h"
#include "Parser.h"
#include "Debug.h"
#include "Memory.h"
#include "MyString.h"
#include "List.h"
#include "Array.h"
#include "TaskTree.h"
#include "../centralMsg.h"

#ifdef TEST
#define DEBUG_PARSER
#endif

BOOLEAN _process_info_msg = FALSE;
MsgData parsedMessage = NULL;

/*
 *  PRIVATE FUNCTIONS
 */

static BOOLEAN IsInfoMsg(String name)
{
  static String infoMsgNames[] = {
    X_IPC_MSG_INFO_QUERY,
    X_IPC_CLASS_INFO_QUERY,
    X_IPC_REGISTER_MSG_INFORM,
    X_IPC_TAP_MSG_INFORM,
    X_IPC_REGISTER_HND_INFORM,
    X_IPC_CONNECT_QUERY,
    X_IPC_NAMED_FORM_INFORM,
    X_IPC_HANDLER_TO_RESOURCE_INFORM,
    X_IPC_REG_MONITOR_INFORM,
    X_IPC_REGISTER_RESOURCE_INFORM,
    X_IPC_ADD_EXCEP_INFORM,
    X_IPC_CREATE_REF_QUERY,
    X_IPC_REF_STATUS_QUERY,

    X_IPC_MSG_INFO_QUERY_OLD,
    X_IPC_CLASS_INFO_QUERY_OLD,
    X_IPC_REGISTER_MSG_INFORM_OLD,
    X_IPC_TAP_MSG_INFORM_OLD,
    X_IPC_REGISTER_HND_INFORM_OLD,
    X_IPC_CONNECT_QUERY_OLD,
    X_IPC_NAMED_FORM_INFORM_OLD,
    X_IPC_HANDLER_TO_RESOURCE_INFORM_OLD,
    X_IPC_REG_MONITOR_INFORM_OLD,
    X_IPC_REGISTER_RESOURCE_INFORM_OLD,
    X_IPC_ADD_EXCEP_INFORM_OLD,
    X_IPC_CREATE_REF_QUERY_OLD,
    X_IPC_REF_STATUS_QUERY_OLD,
    
    X_IPC_ADD_EXCEPTIONS_INFORM,
    X_IPC_IGNORE_LOGGING_INFORM,
    X_IPC_LIMIT_PENDING_INFORM,
    X_IPC_REQUIRES_INFORM,
    X_IPC_HND_INFO_QUERY,
    X_IPC_WAIT_QUERY,

    "V_set_X_IPC_TAPPED_MSG_VAR",
    "V_get_X_IPC_TAPPED_MSG_VAR",
    "V_get_CENTRAL_TERMINAL_LOG_VAR",
    "V_get_CENTRAL_FILE_LOG_VAR",
    "V_set_X_IPC_BROADCAST_MSG_VAR",
    "V_get_X_IPC_BROADCAST_MSG_VAR",
    "V_watch_X_IPC_BROADCAST_MSG_VAR",

    NULL
  };
  
  int index = 0;
  
  if (name && strlen(name) > 0)
    while(infoMsgNames[index] != NULL)
      if (StringEqual(infoMsgNames[index++], name))
	return TRUE;

  return FALSE;
}  

/*
 *  Public functions
 */

MsgData CreateMsgData(void)
{
  MsgData data;
  
  data = _new(_MsgData);
  data->type = TYPE_NULL;
  data->name = "";
  data->id = -1;
  data->parent_id = -1;
  data->source = "";
  data->dest = "";
  data->status = STATUS_NULL;
  data->time = 0;
  data->hours = 0;
  data->minutes = 0;
  data->seconds = 0;
  data->m_seconds = 0;
  data->kill_when = NULL_KILL_SITUATION;
  data->msg1 = data->msg2 = "";
  data->id1 =  data->id2 = -1;
  data->temporal_constraint = 0;
  data->interval1 = data->interval2 = HANDLING_INTERVAL;
  data->point1 = data->point2 = START;
  data->point_TC = '<';
  return(data);
}

extern int yyparse(void);
extern void setInputBuffer(char *buf);

MsgData ParseMsg(String str)
{
#ifdef DEBUG_PARSER
  extern int yydebug;

  yydebug = 1;
#endif

  setInputBuffer(str);
  yyparse();

#ifdef DEBUG_PARSER
  if (parsedMessage->type == TYPE_NULL) printf("Parse error (%s)\n", str); 
#endif

  if (parsedMessage->type != TYPE_NULL && !_process_info_msg &&
      IsInfoMsg(parsedMessage->name)){
    parsedMessage->type = TYPE_NULL;
  }

  return parsedMessage;
}

#ifdef TEST

static void PrintMsgData(MsgData data)
{
  printf("Type: %d Id: %d Status: %d\n", data->type, data->id, data->status);
  printf("Name: %s\nSource: %s ParentId: %d\nDest: %s\n",
	 data->name, data->source, data->parent_id, data->dest);
  printf("Time: %d (%d:%d:%d.%d)\n", data->time,
	 data->hours, data->minutes, data->seconds, data->m_seconds);
  printf("Data: %s\n", data->msg1);
}

/*
 * fills buff with chars from fp, up until max_char arrives, 
 * or we reach a newline/endofline
 */
static char *getLine(char *buff, int max_chars, FILE *fp)
{
  int i, c;
  char *str;
  
  str = buff;
  for( i = 0; i < max_chars; i++){
    c = getc(fp);
    if (c == '\n' || c == EOF) {
      *buff++ = '\0';
      if (c == EOF)
	return NULL;
      else
	return str;
    }
    else 
      *buff++ = c;
  }
  return str;
}

/* ------------------------------------------------ */
/*                   MODULE TEST                    */
/* ------------------------------------------------ */

void  main(int argc, char **argv)
{
  extern FILE *yyin;
  extern int yyparse(void);
  extern void setInputBuffer(char *buf);
  char buffer[300];
#ifdef DEBUG_PARSER
  extern int yydebug;

  yydebug = (argc > 2);
#endif
  yyin = fopen((argc > 1 ? argv[1] : "test.a"), "r");
  while (getLine(buffer, 300, yyin)) {
    setInputBuffer(buffer);
    printf(buffer);
    yyparse();
    printf("\n");
    if (parsedMessage->type != TYPE_NULL) {
      PrintMsgData(parsedMessage);
      printf("\n");
    }
  }
}
#endif

/*****************************************************************
 * File: parser.h
 * Author: Domingo Gallardo, CMU
 * Purpose: Parser of log messages. Translates messages to
 *          a structure named MsgData, which contains all the 
 *          message's data.
 * 
 * $Revision: 1.18 $
 * $Date: 1997/07/07 15:04:06 $
 * $Author: reids $
 *
 * REVISION HISTORY
 *
 * $Log: Parser.h,v $
 * Revision 1.18  1997/07/07 15:04:06  reids
 * Fixed syntax for parsing "Will kill" statements.
 *
 * Revision 1.17  96/09/06  20:41:43  whelan
 * More locations to change MAX_BUF_LENGTH changed.
 * 
 * Revision 1.16  1996/08/05  16:08:43  rich
 * Added comments to endifs.
 *
 * Revision 1.15  1996/05/27  03:30:57  reids
 * Fixed bug where a new task might be confused with an existing "on hold"
 *   task, if the ref id was not logged.
 *
 * Revision 1.14  1996/03/08  19:58:27  reids
 * Extended parser to handle Broadcast messages.
 *
 * Revision 1.13  1996/03/07  21:18:29  reids
 * Fixed the parsing of "MODULE_DISCONNECT".
 *
 * Revision 1.12  1996/02/29  15:10:13  reids
 * Added support (needed by the comm tool) for parsing when modules
 *   connect and disconnect.
 *
 * Revision 1.11  1996/02/29  14:16:55  reids
 * Fixed the way time is parsed.
 *
 * Revision 1.10  1996/02/14  22:15:32  rich
 * Use flex rather than lex because linux does not have lex.
 *
 * Revision 1.9  1996/02/10  23:36:23  rich
 * Ifdef out tests rather than comment them out.  Allows comments in the
 * test routines.
 *
 * Revision 1.8  1996/02/10  16:53:24  rich
 * Made private functions static and fixed some forward declarations.
 * Fixed warnings for lex output.
 *
 * Revision 1.7  1996/02/07  15:32:42  reids
 * Rewrote the TCA log file parser to use lex/bison.  Fixed some small things
 *   (e.g., PMonitor is a POINT_MONITOR, not a POLLING_MONITOR).  Fixed a bug
 *   in which a task was not always correctly connected to its parent, in cases
 *   where another task with the same ID (but completed/killed) was still being
 *   displayed.  Extended TaskTree a bit to handle RAP output.
 *
 * Revision 1.6  1996/02/07  00:28:39  rich
 * Add prefix to VERSION_DATE and COMMIT_DATE.
 *
 * Revision 1.5  1996/02/01  04:04:35  rich
 * Generalized updateVersion and added recursion.
 *
 * Revision 1.4  1996/01/31  22:52:50  reids
 * Added automatic updating of (micro) version control numbers
 *
 * Revision 1.3  1995/12/15  01:25:57  rich
 * Fixed the includes.
 *
 * Revision 1.2  1995/04/07  05:07:02  rich
 * Fixed GNUmakefiles to find the release directory.
 * Fixed problems found by sgi cc compiler.  It would not compile.
 *
 * Revision 1.1  1995/04/05  18:31:21  rich
 * Moved tview files to a subdirectory.
 *
 * Revision 1.1  1994/05/31  03:26:24  rich
 * Moved Domingo's tview tool into the main tca module.
 *
 * Revision 1.3  1994/05/27  05:33:25  rich
 * Can now read from file that is being written to.
 * Fixed Menu and button names.
 * Fixed Indentation.
 * Added menu item to change orientation of the tree.
 *
 * Revision 1.2  1993/08/13  01:43:10  domingo
 * Updated function declarations for compilation under gcc in ANSI C (still
 * a lot of warnings to fix).
 * Added automatic logging.
 *
 *
 * Dec 28 1992 - Domingo Gallardo at School of Computer Science, CMU
 * Split the file into the public part (this one)
 * and the private part (parserP.h)
 *
 * Dec 25 1992 - Domingo Gallardo at School of Computer Science, CMU
 * Created.
 *
 *****************************************************************/

#ifndef _PARSER_H
#define _PARSER_H

#include <X11/Intrinsic.h>

#define TPARSER_VERSION_MAJOR  2
#define TPARSER_VERSION_MINOR  0
#define TPARSER_VERSION_MICRO  11
#define TPARSER_VERSION_DATE "July-07-97"
#define TPARSER_COMMIT_DATE "$Date: 1997/07/07 15:04:06 $"

extern BOOLEAN _process_info_msg;
extern MsgData parsedMessage;

typedef struct _msgStatusRec {
  String      string;
  msgStatus   type;
} _msgStatusRec, *MsgStatusRec;


MsgData CreateMsgData(void);
MsgData ParseMsg(String str);

#define MAX_LONG_MESSAGE 4096

#endif /* _PARSER_H */

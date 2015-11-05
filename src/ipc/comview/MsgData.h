
/*
 * File: MsgData.h
 * Author: Domingo Gallardo, CMU
 * Purpose: Public data referred to TCA messages
 * 
 *
 * REVISION HISTORY
 *
 * $Log: MsgData.h,v $
 * Revision 1.11  1996/08/27 17:05:15  whelan
 * Added term to parser to recognize the "hostname" of a module registration.
 *
 * Revision 1.10  1996/08/05  16:08:40  rich
 * Added comments to endifs.
 *
 * Revision 1.9  1996/03/08  19:58:19  reids
 * Extended parser to handle Broadcast messages.
 *
 * Revision 1.8  1996/02/29  15:10:12  reids
 * Added support (needed by the comm tool) for parsing when modules
 *   connect and disconnect.
 *
 * Revision 1.7  1996/02/07  15:32:39  reids
 * Rewrote the TCA log file parser to use lex/bison.  Fixed some small things
 *   (e.g., PMonitor is a POINT_MONITOR, not a POLLING_MONITOR).  Fixed a bug
 *   in which a task was not always correctly connected to its parent, in cases
 *   where another task with the same ID (but completed/killed) was still being
 *   displayed.  Extended TaskTree a bit to handle RAP output.
 *
 * Revision 1.6  1996/02/01  04:04:33  rich
 * Generalized updateVersion and added recursion.
 *
 * Revision 1.5  1996/01/27  21:56:42  rich
 * Pre-release of 8.4.
 *
 * Revision 1.4  1995/05/31  20:58:34  rich
 * Fixed conflict with tca declarations.
 *
 * Revision 1.3  1995/04/19  14:31:58  rich
 * Added int32 for dealing with tca identifiers.
 *
 * Revision 1.2  1995/04/07  05:06:57  rich
 * Fixed GNUmakefiles to find the release directory.
 * Fixed problems found by sgi cc compiler.  It would not compile.
 *
 * Revision 1.1  1995/04/05  18:31:13  rich
 * Moved tview files to a subdirectory.
 *
 * Revision 1.1  1994/05/31  03:26:14  rich
 * Moved Domingo's tview tool into the main tca module.
 *
 * Revision 1.3  1994/05/27  05:33:20  rich
 * Can now read from file that is being written to.
 * Fixed Menu and button names.
 * Fixed Indentation.
 * Added menu item to change orientation of the tree.
 *
 * Revision 1.2  1993/08/13  01:43:06  domingo
 * Updated function declarations for compilation under gcc in ANSI C (still
 * a lot of warnings to fix).
 * Added automatic logging.
 *
 *
 * Jan 1 1992 - Domingo Gallardo at School of Computer Science, CMU
 * Changed the type of all the strings in the data structure; they are not
 * initialized strings but void pointers.
 *
 * Dec 25 1992 - Domingo Gallardo at School of Computer Science, CMU
 * Created.
 *
 */

#ifndef _MSGDATA_H
#define _MSGDATA_H

#include <X11/Intrinsic.h>

#define MSB (0x40000000) /* Most significant bit for positive int */

typedef enum {
  STATUS_NULL=0, ACTIVE=1, RESUMPTION=2, PENDING=3, 
  INACTIVE=4, TOP_LEVEL_QUERY=5, COMPLETED=6, WAIT_FOR_REPLY=7
}  msgStatus;

#define SENT ACTIVE

typedef enum {
  TYPE_NULL=0, QUERY=1, GOAL=2, COMMAND=3, INFORM=4, EXCEPTION=5,
  KILLED=6, INTERVAL_MONITOR=7, POINT_MONITOR=8,
  FIRE_DEMON=9, REPLY=10, FAILURE=11, SUCCESS=12, BYPASS=13,
  POINT_CONSTRAINT=14, RETRY=15, WILL_KILL=16, TEMP_CONSTRAINT=17, WARNING=18,
  MODULE_CONNECT=19, MODULE_DISCONNECT=20, BROADCAST=21, HOSTNAME=22, UNUSED_INFO=23
} msgType;

typedef enum {
  NULL_KILL_SITUATION=0, WHEN_REFERENCES_RELEASED=1, WHEN_COMPLETES=2,
  WHEN_SUBMESSAGES_COMPLETES=3
} killSituation;

typedef enum {
  MSG_SEQ_ACH=1, MSG_SEQ_PLAN=2, MSG_PLAN_FIRST=3, MSG_DELAY_PLAN=4,
  MSG_INV_SEQ_ACH=5, MSG_INV_SEQ_PLAN=6
} temporalConstraint;

typedef enum {
  START=0, END=1
} intervalOp;


typedef enum {
  HANDLING_INTERVAL=0, PLANNING_INTERVAL=1, ACHIVEMENT_INTERVAL=2
} interval;


typedef struct _MsgData {
  msgType      type;
  String       name;
  int32        id;
  int32        parent_id;
  String       source;
  String       dest;
  msgStatus    status;
  int          time;
  int          hours;
  int          minutes;
  int          seconds;
  int          m_seconds;
  killSituation kill_when;
  String       msg1;
  String       msg2;
  int32        id1; 
  int32        id2;
  int32        temporal_constraint;
  interval     interval1;
  interval     interval2;
  intervalOp   point1;
  intervalOp   point2;
  char         point_TC;  /*     CONSTRAINT        CHAR
			   *
			   *         <               <
			   *         =               =
			   *         >               >
			   *         <=              /
			   *         >=              \
			   */
  
} _MsgData, *MsgData;

#endif /* _MSGDATA_H */

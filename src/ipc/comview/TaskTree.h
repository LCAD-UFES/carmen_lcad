/*
 * File: TaskTree.h
 * Author: Domingo Gallardo, CMU
 * Purpose: Header file for task tree implementation
 *
 * REVISION HISTORY
 *
 * $Log: TaskTree.h,v $
 * Revision 1.8  1996/08/05 16:08:46  rich
 * Added comments to endifs.
 *
 * Revision 1.7  1996/06/04  18:44:35  whelan
 * Changes required by comview.
 *
 * Revision 1.6  1996/02/07  15:32:44  reids
 * Rewrote the TCA log file parser to use lex/bison.  Fixed some small things
 *   (e.g., PMonitor is a POINT_MONITOR, not a POLLING_MONITOR).  Fixed a bug
 *   in which a task was not always correctly connected to its parent, in cases
 *   where another task with the same ID (but completed/killed) was still being
 *   displayed.  Extended TaskTree a bit to handle RAP output.
 *
 * Revision 1.5  1996/01/27  21:56:46  rich
 * Pre-release of 8.4.
 *
 * Revision 1.4  1995/12/15  01:26:02  rich
 * Fixed the includes.
 *
 * Revision 1.3  1995/04/19  14:32:02  rich
 * Added int32 for dealing with tca identifiers.
 *
 * Revision 1.2  1995/04/07  05:07:08  rich
 * Fixed GNUmakefiles to find the release directory.
 * Fixed problems found by sgi cc compiler.  It would not compile.
 *
 * Revision 1.1  1995/04/05  18:31:29  rich
 * Moved tview files to a subdirectory.
 *
 * Revision 1.2.2.1  1995/04/04  18:11:09  rich
 * Added support for compiling on sgi machines.  Use "gmake -k CC=cc install"
 * Fixed bugs in the declarations for some functions.
 *
 * Revision 1.3  1995/03/28  01:16:57  rich
 * Fixed problem where pointers to local variables were being returned.
 *
 * Revision 1.2  1995/01/25  00:04:34  rich
 * Release of tca 7.9.  Mostly speed improvements.
 * The cvs binaries may now be located in /usr/local.
 * Formatting changes.
 *
 * Revision 1.1  1994/05/31  03:26:31  rich
 * Moved Domingo's tview tool into the main tca module.
 *
 * Revision 1.3  1994/05/27  05:33:31  rich
 * Can now read from file that is being written to.
 * Fixed Menu and button names.
 * Fixed Indentation.
 * Added menu item to change orientation of the tree.
 *
 * Revision 1.2  1993/08/13  01:43:13  domingo
 * Updated function declarations for compilation under gcc in ANSI C (still
 * a lot of warnings to fix).
 * Added automatic logging.
 *
 *
 * Jan 11 1993 - Domingo Gallardo at School of Computer Science, CMU
 * Created.
 *
 */


#ifndef _TASKTREE_H
#define _TASKTREE_H

#define CENTRAL_MODULE_NAME "TCA Server" 
#define ON_HOLD_NAME        "ON HOLD"
#define RESOURCE_NAME       "Resource"
#define MONITOR_NAME        "*Monitor*"
#define REUSE_NAME          "REUSE MSG"
#define MAX_TASK_TYPES      7

#define ROOT_NODE_ID (-999)

typedef enum {
  task_NULL=0, task_ON_HOLD=1, task_PENDING=2, 
  task_ACTIVE=3, task_WAITING=4, task_COMPLETED=5, task_KILLED=6
} taskState;

#define FIRST_TASK_STATE task_ON_HOLD
#define LAST_TASK_STATE  task_KILLED

typedef enum {
  task_NORMAL=0, task_RESUMPTION=1, task_WILL_BE_KILLED=2
} taskState2;

typedef enum {
  task_QUERY=0, task_GOAL=1, task_COMMAND=2, task_EXCEPTION=3,
  task_INTERVAL_MONITOR=4, task_POINT_MONITOR=5, task_INFORM=6,
  task_BROADCAST=7
} taskType;



typedef struct _WatingInterval {
  int         start_time;
  int         end_time;
  struct _Task *waiting_for;
} _WaitingInterval, *WaitingInterval;


typedef struct _TemporalConstraint {
  int          temporal_constraints;
  struct _Task *to_task;
}_TemporalConstraint, *TemporalConstraint;


typedef struct _PointTC{
  interval     my_interval, task_interval;
  intervalOp   my_op, task_interval_op;
  char         point_TC;
  struct _Task *to_task;
} _PointTC, *PointTC;


typedef struct _Task {
  String             name;
  int32              id;
  int                creation_time;
  int                pending_time;
  int                start_time;
  int                end_time;
  WaitingInterval    waiting_interval;
  Array              waiting_list;
  Array              point_constraints;
  Array              temporal_constraint;
  taskState          state;
  taskState2         state2;
  taskType           type;
  String             module;
  struct _Task       *creator;         /* This task created me            */
  struct _Task       *in_goal;         /* I belong to this goal           */
  struct _Task       *first_child;     /* My first child (if I am a goal) */
  struct _Task       *sibiling;
  void		     *additional;      /* Additional information (for GUI
					  and other tools */
} _Task, *Task;

typedef struct _Resource {
  String      name;
  Array       handlers;  /* Handlers grouped by module (see next struct) */
} _Resource, *Resource;


typedef struct _Handlers {
  String      module_name;
  Array       names;
} _Handlers, *Handlers;


typedef void *(* ADDITIONAL_TASK_DATA_FN)(void); 
typedef void (* INSERT_TASK_ROOT_FN)(Task rootNode);
typedef void (* INSERT_TASK_CHILD_FN)(Task parentNode, Task childNode);
typedef void (* MODIFY_TASK_FN)(Task task, taskState newState, int time);
typedef void (* MODIFY_TASK2_FN)(Task task, taskState2 newState, int time);

extern Task mission;

/* The function to call when a new node is created; returns the "additional"
   field of the task */
void setAdditionalTaskDataFn(ADDITIONAL_TASK_DATA_FN fn);
/* The function to call when the parser adds the root node to the task tree */
void setInsertTaskRootFn(INSERT_TASK_ROOT_FN fn);
/* The function to call when the parser adds a child node to the task tree */
void setInsertTaskChildFn(INSERT_TASK_CHILD_FN fn);
/* The function to call when the taskState of a node is changed */
void setModifyTaskFn(MODIFY_TASK_FN fn);
/* The function to call when the taskState2 of a node is changed */
void setModifyTask2Fn(MODIFY_TASK2_FN fn);

void     TaskTreeInitialize(int initial_time);
Task     FindTask(String name, int id, taskState state);
void     PrintTasks(taskState state);
Task     FindActiveTaskInModule(String module_name);
Task     TaskCreate(msgType type, String name, int id, msgStatus state, 
		    int time);
void     TaskTreeProcessNewMessage(MsgData data);
void     InsertTaskAsChild(Task task_parent, Task new_child);
void     ModifyTaskState(Task task, int time, taskState new_state,
			 Task query_handler);
void     ModifyTaskState2(Task task, int time, taskState2 new_state,
			  Task query_handler);
String   TaskStateString(taskState state);
String   TaskTypeString(taskType type);
String   TaskState2String(taskState2 state2);
BOOLEAN  RefIdEqual(int32 id1, int32 id2);
Resource UserResourceCreate(String name);
void     ResourceAddHandler(Resource resource, String handler_name,
			    String handler_module);


#endif /* _TASKTREE_H */

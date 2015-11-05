/*
 * File: TaskTree.c
 * Author: Domingo Gallardo, CMU
 * Purpose: Header file for task tree implementation
 *
 * REVISION HISTORY
 *
 * $Log: TaskTree.c,v $
 * Revision 1.10  1997/05/29 21:27:26  reids
 * Dealing with some of the subtleties of the TCA log file (Greg & Reid)
 *
 * Revision 1.9  1996/06/04  18:44:34  whelan
 * Changes required by comview.
 *
 * Revision 1.8  1996/05/27  03:31:03  reids
 * Fixed bug where a new task might be confused with an existing "on hold"
 *   task, if the ref id was not logged.
 *
 * Revision 1.7  1996/02/10  23:36:25  rich
 * Ifdef out tests rather than comment them out.  Allows comments in the
 * test routines.
 *
 * Revision 1.6  1996/02/07  15:32:43  reids
 * Rewrote the TCA log file parser to use lex/bison.  Fixed some small things
 *   (e.g., PMonitor is a POINT_MONITOR, not a POLLING_MONITOR).  Fixed a bug
 *   in which a task was not always correctly connected to its parent, in cases
 *   where another task with the same ID (but completed/killed) was still being
 *   displayed.  Extended TaskTree a bit to handle RAP output.
 *
 * Revision 1.5  1996/01/27  21:56:45  rich
 * Pre-release of 8.4.
 *
 * Revision 1.4  1995/12/15  01:25:59  rich
 * Fixed the includes.
 *
 * Revision 1.3  1995/05/31  20:58:40  rich
 * Fixed conflict with tca declarations.
 *
 * Revision 1.2  1995/04/07  05:07:05  rich
 * Fixed GNUmakefiles to find the release directory.
 * Fixed problems found by sgi cc compiler.  It would not compile.
 *
 * Revision 1.1  1995/04/05  18:31:26  rich
 * Moved tview files to a subdirectory.
 *
 * Revision 1.4  1995/03/28  01:16:54  rich
 * Fixed problem where pointers to local variables were being returned.
 *
 * Revision 1.3  1995/03/16  18:06:11  rich
 * Merged in changes to the 7.9 branch.
 * Changed the VERSION_ to TCA_VERSION_
 *
 * Revision 1.2  1995/01/25  00:04:31  rich
 * Release of tca 7.9.  Mostly speed improvements.
 * The cvs binaries may now be located in /usr/local.
 * Formatting changes.
 *
 * Revision 1.1  1994/05/31  03:26:29  rich
 * Moved Domingo's tview tool into the main tca module.
 *
 * Revision 1.5  1994/05/27  05:34:50  rich
 * Can now read from file that is being written to.
 * Fixed Menu and button names.
 * Fixed Indentation.
 * Added menu item to change orientation of the tree.
 *
 * Revision 1.4  1993/09/13  04:12:29  domingo
 * Added two set of options to the tree menu:
 *
 * - Erase queries (on/off) : activate/deactivate the deletion of queries
 *      from the task tree. Erase queries by default.
 *
 * - Display constraints (on/off) : activate/deactivate the display of
 *      constraints messages. No display by default.
 *
 * Revision 1.3  1993/09/07  00:24:46  domingo
 * Fixed almost all the warnings
 *
 * Revision 1.2  1993/08/13  02:09:26  domingo
 * Updated function declarations for compilation under gcc in ANSI C (still
 * a lot of warnings to fix).
 * Added automatic logging.
 *
 * Jan 11 1993 - Domingo Gallardo at School of Computer Science, CMU
 * Created.
 *
 */

#include "Standard.h"
#include "List.h"
#include "Array.h"
#include "MsgData.h"
#include "TaskTree.h"
#include "Memory.h"
#include "Debug.h"
#include "MyString.h"

#include "../centralMsg.h"

#define DEBUG_TASKTREE False

/* Global variables */

Task mission;

static Array modules;
static Array user_resources;

static Array inactive_tasks;
static Array pending_tasks;
static Array active_tasks;
static Array waiting_tasks;
static Array completed_tasks;
static Array killed_tasks;

static INSERT_TASK_ROOT_FN insertTaskRootFn = NULL;
static INSERT_TASK_CHILD_FN insertTaskChildFn = NULL;
static MODIFY_TASK_FN modifyTaskFn = NULL;
static MODIFY_TASK2_FN modifyTask2Fn = NULL;
static ADDITIONAL_TASK_DATA_FN additionalDataFn = NULL;

static char Inverse(char point_TC);
static temporalConstraint InverseTemporalConstraint(temporalConstraint tc);
static Array ArrayStates(taskState task_state);
static Task FindTaskInArray(String name, int id, Array array);
static BOOLEAN TaskEqual(Task task, String name, int id);
static void PrintTask(Task task);
static String GetModule(String msg_module_name, String task_name);
static String FindModule(String module_name);
static Task FindTaskInModule(Array array_tasks, String module_name);
static BOOLEAN TaskInModule(Task task, String module_name);
static String FindModuleOfHandlerUserResource(Resource resource,
					      String handler_name);
static Resource FindResource(String resource_name);
static Handlers FindHandlers(String module_name);
static BOOLEAN HandlersHasName(Handlers handlers, String name, Pointer data2
			       /*unused*/);
static BOOLEAN ResourceHasName(Resource resource, String name, Pointer data2
			       /*unused*/);
static BOOLEAN IsNameResource(String name);

static void ProcessPointConstraint(MsgData data);
static void ProcessReply(MsgData data);
static void ProcessWillKill(MsgData data);
static void ProcessKilled(MsgData data);
static void ProcessFailure(MsgData data);
static void ProcessSuccess(MsgData data);
static void ProcessException(MsgData data);
static void ProcessMonitor(MsgData data);
static void ProcessGoalCommandInform(MsgData data);
static void ProcessQuery(MsgData data);
static void ProcessTemporalConstraint(MsgData data);

/* The function to call when a new node is created; returns the "additional"
   field of the task */
void setAdditionalTaskDataFn(ADDITIONAL_TASK_DATA_FN fn)
{
  additionalDataFn = fn;
}

/* The function to call when the parser adds the root node to the task tree */
void setInsertTaskRootFn(INSERT_TASK_ROOT_FN fn)
{
  insertTaskRootFn = fn;
}

/* The function to call when the parser adds a child node to the task tree */
void setInsertTaskChildFn(INSERT_TASK_CHILD_FN fn)
{
  insertTaskChildFn = fn;
}

/* The function to call when the taskState of a node is changed */
void setModifyTaskFn(MODIFY_TASK_FN fn)
{
  modifyTaskFn = fn;
}

/* The function to call when the taskState2 of a node is changed */
void setModifyTask2Fn(MODIFY_TASK2_FN fn)
{
  modifyTask2Fn = fn;
}


/*	Function Name: TaskTreeInitialize
 *	Arguments:     
 *	Description:   Creates the initial data structures
 *	Returns:       Nothing
 */

void TaskTreeInitialize(int initial_time)
{
  modules = ArrayNew();
  user_resources = ArrayNew();
  active_tasks = ArrayNew();
  pending_tasks = ArrayNew();
  killed_tasks = ArrayNew();
  inactive_tasks = ArrayNew();
  waiting_tasks = ArrayNew();
  completed_tasks = ArrayNew();
  mission = TaskCreate(GOAL, "Top", ROOT_NODE_ID, SENT, initial_time);
  mission->module = strdup("");
  if (insertTaskRootFn != NULL) (*insertTaskRootFn)(mission);
}


Task FindTask(String name, int id, taskState state)
{
  Array array;
  Task task;
  
  if (state == task_NULL) {
    for (state = FIRST_TASK_STATE; state <= LAST_TASK_STATE; state++) {
      array = ArrayStates(state);
      task = FindTaskInArray(name, id, array);
      if (task)
	return(task);
    }
    return(NULL);
  } else {
    array = ArrayStates(state);
    return (FindTaskInArray(name, id, array));
  }
}


void PrintTasks(taskState state)
{
  Array array;
  
  array = ArrayStates(state);
  ArrayIterate(array, (LIST_ITERATE_TYPE)PrintTask);
}

Task FindActiveTaskInModule(String module_name)
{
  return(FindTaskInModule(active_tasks, module_name));
}

static void AddTask (Task task, taskState state)
{
  task->state = state;
  ArrayInsert(ArrayStates(state), (Pointer)task);
}


/*	Function Name: TaskCreate 
 *	Description:   Create a new task from the msgData passed
 *                     as parameter.
 *	Arguments:    
 *	Returns:       just created task
 */
Task TaskCreate(msgType type, String name, int id, msgStatus state, int time)
{
  Task new_task;
  
  /* see if task already exists because created by
     temporal constraint */
  
  new_task = FindTask(name, id, task_ON_HOLD);
  if (new_task == NULL ||
      /* Got confused by another ON_HOLD task with the same name */
      (id == -1 && new_task->type != TYPE_NULL)) {
    new_task = _new(_Task);
    new_task->name = strdup(name);
    new_task->id = id;
    new_task->state2 = task_NORMAL;
    new_task->creation_time = time;
    new_task->pending_time = -1;
    new_task->start_time = -1;
    new_task->end_time = -1;
    new_task->waiting_interval = NULL;
    new_task->waiting_list = ArrayNew();
    new_task->point_constraints = ArrayNew();
    new_task->temporal_constraint = ArrayNew();
    new_task->module = NULL;
    new_task->creator = NULL;
    new_task->in_goal = NULL;
    new_task->first_child = NULL;
    new_task->sibiling = NULL;
    new_task->in_goal = NULL;
    new_task->additional = (additionalDataFn ? (*additionalDataFn)() : NULL);
  }
  else 
    ArrayDelete(ArrayStates(task_ON_HOLD));
  new_task->creation_time = time;
  
  switch (type) {
  case GOAL:
    new_task->type = task_GOAL;
    break;
  case QUERY:
    new_task->type = task_QUERY;
    break;
  case COMMAND:
    new_task->type = task_COMMAND;
    break;
  case INFORM:
    new_task->type = task_INFORM;
    break;
  case BROADCAST:
    new_task->type = task_BROADCAST;
    break;
  case EXCEPTION:
    new_task->type = task_EXCEPTION;
    break;
  case INTERVAL_MONITOR:
    new_task->type = task_INTERVAL_MONITOR;
    break;
  case POINT_MONITOR:
    new_task->type = task_POINT_MONITOR;
    break;
  case TYPE_NULL:
    break;
  case KILLED:
  case FIRE_DEMON:
  case FAILURE:
  case SUCCESS:
  case BYPASS:
  case POINT_CONSTRAINT:
  case RETRY:
  case TEMP_CONSTRAINT:
  case WARNING:
#ifndef TEST_CASE_COVERAGE
  default:
    Warning("Type of %s not managed\n", name);
#endif
  }
  switch (state) {
  case INACTIVE:
    AddTask(new_task, task_ON_HOLD); break;
  case PENDING:
    AddTask(new_task, task_PENDING);
    new_task->pending_time = time;
    break;
  case SENT:
    AddTask(new_task, task_ACTIVE);
    new_task->pending_time = -1;
    new_task->start_time = time;
    break;
  case COMPLETED:
    AddTask(new_task, task_COMPLETED);    
    new_task->start_time = time;
    new_task->end_time = time;
    break;
  case STATUS_NULL:
  case RESUMPTION:
  case TOP_LEVEL_QUERY:
  case WAIT_FOR_REPLY:
#ifndef TEST_CASE_COVERAGE
  default:
    break;
#endif
  }
  Debug(DEBUG_TASKTREE, "Creating task %s {%d}\n", 
	new_task->name, new_task->id);
  return (new_task);
}


void TaskTreeProcessNewMessage(MsgData data)
{
  switch (data->type) {
  case QUERY: ProcessQuery(data); break;
    
  case BROADCAST:
  case INFORM:
  case COMMAND:
  case GOAL: ProcessGoalCommandInform(data); break;
    
  case INTERVAL_MONITOR:
  case POINT_MONITOR: ProcessMonitor(data); break;
    
  case EXCEPTION: ProcessException(data); break;
    
  case BYPASS:
  case SUCCESS:
  case RETRY: ProcessSuccess(data); break;
    
  case FAILURE: ProcessFailure(data); break;
    
  case KILLED: ProcessKilled(data); break;
    
  case WILL_KILL: ProcessWillKill(data); break;
    
  case REPLY: ProcessReply(data); break;
    
  case POINT_CONSTRAINT: ProcessPointConstraint(data); break;
    
  case TEMP_CONSTRAINT: ProcessTemporalConstraint(data); break;
    
  case TYPE_NULL:
  case FIRE_DEMON:
  case WARNING:
    break;
#ifndef TEST_CASE_COVERAGE
  default:
    break;
#endif
  }
}


void InsertTaskAsChild(Task task_parent, Task new_child)
{
  Task last_child;
  
  last_child = task_parent->first_child;
  if (last_child == NULL)
    task_parent->first_child = new_child;
  else {
    while (last_child->sibiling)
      last_child = last_child->sibiling;
    /* This if is needed to prevent loops in the case where the parent ids 
     * are not present.
     */
    if (last_child != new_child)
      last_child->sibiling = new_child;
  } 
  new_child->creator = task_parent;
  Debug(DEBUG_TASKTREE, "Inserting task %s {%d} as a child of %s {%d}\n",
	new_child->name, new_child->id, task_parent->name, task_parent->id);
  if (insertTaskChildFn != NULL) (*insertTaskChildFn)(task_parent, new_child); 
}


void ModifyTaskState(Task task, int time, taskState new_state,
		     Task query_handler)
{
  Array array;
  Task task_aux;
  taskState oldState = task->state;
  
  Debug(DEBUG_TASKTREE, "Changing the state of %s {%d} from %d to %d\n", 
	task->name, task->id, task->state, new_state);
  if (modifyTaskFn != NULL) (*modifyTaskFn)(task, new_state, time);
  array = ArrayStates(task->state);
  task_aux = FindTaskInArray(task->name, task->id, array);
  if (task_aux)
    ArrayDelete(array);
  else 
    Error("Error in Modify Task State: not found task %s\n", task->name);
  AddTask(task, new_state);
  switch(new_state) {
  case task_PENDING:
    task->pending_time = time;
    break;
  case task_ACTIVE:
    if (oldState == task_WAITING) {
      task->waiting_interval->end_time = time;
      ArrayInsert(task->waiting_list,  (Pointer) task->waiting_interval);
      task->waiting_interval = NULL;
    }
    else
      task->start_time = time;
    break;
  case task_COMPLETED:
  case task_KILLED:
    
    /* Falta colocar en algun sitio un indicador del
       tipo de terminacion (failure o success) */
    
    task->end_time = time;
    break;
  case task_WAITING:
    task->waiting_interval = _new(_WaitingInterval);
    task->waiting_interval->start_time = time;
    task->waiting_interval->waiting_for = query_handler;
    break;
  case task_NULL:
  case task_ON_HOLD:
    break;
#ifndef TEST_CASE_COVERAGE
  default:
    break;
#endif
  }
}


void ModifyTaskState2(Task task, int time, taskState2 new_state,
		      Task query_handler)
{
  Debug(DEBUG_TASKTREE, "Changing the state2 of %s {%d} from %d to %d\n",
	task->name, task->id, task->state2, new_state);
  if (modifyTask2Fn != NULL) (*modifyTask2Fn)(task, new_state, time);
  task->state2 = new_state;
}


String TaskStateString(taskState state)
{
  switch (state) {
  case task_ON_HOLD:
    return(strdup("Inactive"));
    break;
  case task_ACTIVE:
    return(strdup("Active"));
    break;
  case task_PENDING:
    return(strdup("Pending"));
    break;
  case task_WAITING:
    return(strdup("Waiting"));
    break;
  case task_COMPLETED:
    return(strdup("Handled"));
    break;
  case task_KILLED:
    return(strdup("Killed"));
    break;
  case task_NULL:
    break;
#ifndef TEST_CASE_COVERAGE
  default:
    break;
#endif
  }
  return(strdup("?????"));
}

String TaskTypeString(taskType type)
{
  switch (type) {
  case task_QUERY: return(strdup("QUERY")); break;
    
  case task_GOAL: return(strdup("GOAL")); break;
    
  case task_COMMAND: return(strdup("COMMAND")); break;
    
  case task_INFORM: return(strdup("INFORM")); break;

  case task_BROADCAST: return(strdup("BROADCAST")); break;
    
  case task_EXCEPTION: return(strdup("EXCEPTION")); break;
    
  case task_INTERVAL_MONITOR: return(strdup("I_MONITOR")); break;
    
  case task_POINT_MONITOR: return(strdup("P_MONITOR")); break;
  }
  return(strdup("?????"));
}


String TaskState2String(taskState2 state2)
{
  switch (state2) {
  case task_NORMAL: return(strdup("Normal")); break;
    
  case task_RESUMPTION: return(strdup("Resumption")); break;
    
  case task_WILL_BE_KILLED: return(strdup("To be killed")); break;
  }
  return(strdup("?????"));
}


BOOLEAN RefIdEqual(int id1, int id2)
{
  return ((id1 == -1 || id2 == -1) ? 1 : (id1 == id2));
}


/*
 *  User resources stuff
 */


Resource UserResourceCreate(String name)
{
  Resource resource;
  
  resource = _new(_Resource);
  resource->name = strdup(name);
  resource->handlers = ArrayNew();
  ArrayInsert(user_resources, (Pointer) resource);
  return(resource);
}


void ResourceAddHandler(Resource resource, String handler_name,
			String handler_module)
{
  Handlers handlers;
  String  str_aux;
  
  
  handlers = FindHandlers(handler_module);
  if (handlers) {
    str_aux = strdup(handler_name);
    ArrayInsert(handlers->names, (Pointer) str_aux);
  }
  else {
    handlers = _new(_Handlers);
    handlers->names = ArrayNew();
    handlers->module_name = strdup(handler_module);
    str_aux = strdup(handler_name);
    ArrayInsert(handlers->names, (Pointer) str_aux);
    ArrayInsert(resource->handlers, (Pointer) handlers);
  }
}


/*
 *  PRIVATE FUNCTIONS 
 */

/* Create the task and insert it under its "creator" */
static Task CreateAndInsertTask (msgType type, String name, int id,
				 String source, String dest, int parent_id,
				 int time, msgStatus status)
{
  Task task, task_creator;
  
  task_creator = ((parent_id == -1) ? FindActiveTaskInModule(source)
		  : FindTask("", parent_id, task_NULL));
  task = TaskCreate(type, name, id, status, time);
  task->creator = task_creator;
  task->in_goal = possibleNULL(task_creator, in_goal);
  task->module = GetModule(dest, task->name);
  InsertTaskAsChild((task_creator ? task_creator : mission), task);
  
  return task;
}

/* Look for a message that locks/reserves a resource */
static BOOLEAN isResourceMgmtMessage(char *msgName)
{
  return (!strcmp(msgName, X_IPC_RESERVE_RESOURCE_QUERY) ||
	  !strcmp(msgName, X_IPC_RESERVE_RESOURCE_QUERY_OLD) ||
	  !strcmp(msgName, X_IPC_RESERVE_MOD_RESOURCE_QUERY) ||
	  !strcmp(msgName, X_IPC_RESERVE_MOD_RESOURCE_QUERY_OLD) ||
	  !strcmp(msgName, X_IPC_LOCK_RESOURCE_QUERY) ||
	  !strcmp(msgName, X_IPC_LOCK_RESOURCE_QUERY_OLD) ||
	  !strcmp(msgName, X_IPC_LOCK_MOD_RESOURCE_QUERY) ||
	  !strcmp(msgName, X_IPC_LOCK_MOD_RESOURCE_QUERY_OLD));
}

static void ProcessQuery(MsgData data)
{
  Task task;
  
  /* This is a screwy special case due to weirdness with TCA logging */
  if (isResourceMgmtMessage(data->name)) {
    if (data->status == PENDING) {
      task = FindTask(data->name, data->id, task_ACTIVE);
      ModifyTaskState(task, data->time, task_PENDING, NULL);
    } else {
      task = FindTask(data->name, data->id, task_PENDING);
      if (task) {
	ModifyTaskState(task, data->time, task_ACTIVE, NULL);
      } else {
	task = CreateAndInsertTask(data->type, data->name, data->id,
				   data->source, data->dest, data->parent_id,
				   data->time, data->status);
      }
    }
    if (task->creator) {
      ModifyTaskState(task->creator, data->time, task_WAITING, NULL);
    }
  } else if (IsNameResource(data->source)) {
    /* 
     * The task is PENDING, and it already exists.
     */
    task = FindTask(data->name, data->id, task_PENDING);
    if (!task) {
      Warning("Warning in Process Query: Can't find the on_hold task %s {%d}\n",
	      data->name, data->id);
      return;
    }
    ModifyTaskState(task->creator, data->time, task_WAITING, NULL);
    ModifyTaskState(task, data->time, task_ACTIVE, NULL);
  } else {
    task = CreateAndInsertTask(data->type, data->name, data->id, data->source,
			       data->dest, data->parent_id, data->time,
			       data->status);
//    if (task->creator && task->state == task_ACTIVE) {			// @@@ Alberto: Este trecho foi comentado porque Query nunca tem creator.
//     ModifyTaskState(task->creator, data->time, task_WAITING, NULL);
//    }
  }
}

static void ProcessGoalCommandInform(MsgData data)
{
  Task task;
  
  /* For the rview (RAP) program -- REUSE_NAME says it already exists;
     Update the task according to the given status */
  if (StringEqual(data->source, REUSE_NAME)) {
    taskState newState = (data->status == WAIT_FOR_REPLY ? task_WAITING :
			  data->status == ACTIVE ? task_ACTIVE :
			  data->status == COMPLETED ? task_COMPLETED : 
			  task_NULL);
    
    task = FindTask(data->name, data->id, task_NULL);
    if (!task) {
      Warning("Warning in ProcessGoalCommandInform: Can't find the task %s {%d}\n",
	      data->name, data->id);
      return;
    }
    ModifyTaskState(task, data->time, newState, NULL);
  }
  else if (StringEqual(data->source, ON_HOLD_NAME)) {
    /* 
     * The task is INACTIVE, and it already
     * exists.
     */
    task = FindTask(data->name, data->id, task_ON_HOLD);
    if (!task) {
      Warning("Warning in Process Goal/Command/Inform: Can't find the on_hold task %s {%d}\n",
	      data->name, data->id);
      return;
    }
    task->module = GetModule(data->dest, task->name);
    if (data->status == PENDING)
      ModifyTaskState(task, data->time, task_PENDING, NULL);
    else
      ModifyTaskState(task, data->time, task_ACTIVE, NULL);
  }
  else if (IsNameResource(data->source)) {
    /* 
     * The task is PENDING, and it already
     * exists.
     */
    task = FindTask(data->name, data->id, task_PENDING);
    if (!task) {
      Warning("Warning in Process Goal/Command/Inform: Can't find the pending task %s {%d}\n",
	      data->name, data->id);
      return;
    }
    ModifyTaskState(task, data->time, task_ACTIVE, NULL);
  } else {
    task = CreateAndInsertTask(data->type, data->name, data->id, data->source,
			       data->dest, data->parent_id, data->time,
			       data->status);
  }
}

static void ProcessMonitor(MsgData data)
{
  Task task;
  
  if (data->status == COMPLETED) {
    task = FindTask(data->name, data->id, task_ACTIVE);
    /* Monitor could be in the middle of handling a condition query */
    if (!task) task = FindTask(data->name, data->id, task_WAITING);
    if (task == NULL) {
      Warning(">>Warning in ProcessMonitor: %s {%d} not found as active or waiting\n",
	      data->name, data->id);
      return;
    }
    else {
      ModifyTaskState(task, data->time, task_COMPLETED, NULL);
    }
  } else if (StringEqual(data->source, ON_HOLD_NAME)) {
    /* 
     * The task is INACTIVE, and it already exists.
     */
    task = FindTask(data->name, data->id, task_ON_HOLD);
    if (!task) {
      Warning("Warning in Process Monitor: Can't find the on_hold task %s {%d}\n",
	      data->name, data->id);
      return;
    }
    task->module = GetModule(data->dest, task->name);
    if (data->status == PENDING)
      ModifyTaskState(task, data->time, task_PENDING, NULL);
    else
      ModifyTaskState(task, data->time, task_ACTIVE, NULL);
  }
  else if (IsNameResource(data->source)) {
    /* 
     * The task is PENDING, and it already
     * exists.
     */
    task = FindTask(data->name, data->id, task_PENDING);
    if (!task) {
      Warning("Warning in Process Monitor: Can't find the pending task %s {%d}\n",
	      data->name, data->id);
      return;
    }
    ModifyTaskState(task, data->time, task_ACTIVE, NULL);
  } else {
    task = CreateAndInsertTask(data->type, data->name, data->id, data->source,
			       data->dest, data->parent_id, data->time,
			       data->status);
  } 
}


static void ProcessException(MsgData data)
{
  (void)CreateAndInsertTask(data->type, data->name, data->id, data->source,
			    data->dest, data->parent_id, data->time,
			    data->status);
}


static void ProcessSuccess(MsgData data)
{
  Task task;
  
  task = FindTask(data->name, data->id, task_ACTIVE);
  if (task == NULL) {
    /*
     * Domingo, May 1. 
     * I comment this warning because currently in a waitForCommand, the process 
     * is finished when the Reply message comes, so when the success message is
     * sent, there is no active process and the warning is printed without really
     * be an error.
     * We have to fix this in some other way ... including to put the state of 
     * the caller as "waiting", but now I don't have enough info in the log file.
     
     * Warning(">>Warning in Process Success: %s {%d} not found as active\n",
     *         data->name, data->id);
     */
    return;
  } else {
    if (task->type != task_GOAL && task->type != task_COMMAND &&
	task->type != task_INFORM && task->type != task_BROADCAST &&
	task->type != task_EXCEPTION) {
      Warning("Success processed for %s (%d) (not goal/command/inform/broadcast/exception)\n", 
	      task->name, task->id);
    }
    ModifyTaskState(task, data->time, task_COMPLETED, NULL);
  }
}


static void ProcessFailure(MsgData data)
{
  Task task;
  
  task = FindTask(data->name, data->id, task_ACTIVE);
  if (task == NULL) {
    Warning(">>Warning in Process Success: %s {%d} not found as active\n",
	    data->name, data->id);
    return;
  } else {
    if (task->type != task_GOAL && task->type != task_COMMAND) {
      Warning("Failure processed for %s (%d) (not goal/command)\n", 
	      task->name, task->id);
    }
    ModifyTaskState(task, data->time, task_COMPLETED, NULL);
  }
}


static void ProcessKilled(MsgData data)
{
  Task task;
  
  task = FindTask(data->name, data->id, task_ON_HOLD);
  if (!task)
    task =  FindTask(data->name, data->id, task_PENDING);
  if (!task)
    task = FindTask(data->name, data->id, task_COMPLETED);
  if (!task)
    task = FindTask(data->name, data->id, task_ACTIVE);
  if (!task) {
    Warning(">> Warning in ProcessKilled: task %s {%d} not found\n", 
	    data->name, data->id);
    return;
  }
  else
    ModifyTaskState(task, data->time, task_KILLED, NULL);
}


static void ProcessWillKill(MsgData data)
{
  Task task;
  
  task = FindTask(data->name, data->id, task_ACTIVE);
  if (!task)
    task =  FindTask(data->name, data->id, task_WAITING);
  if (!task)
    task =  FindTask(data->name, data->id, task_ON_HOLD);
  if (!task)
    task =  FindTask(data->name, data->id, task_PENDING);
  if (!task)
    task = FindTask(data->name, data->id, task_COMPLETED);
  if (!task) {
    Warning(">> Warning in Process Will Kill: task %s {%d} not found\n",
	    data->name, data->id);
    return;
  }
  else
    ModifyTaskState2(task, data->time, task_WILL_BE_KILLED, NULL);
}


static void ProcessReply(MsgData data)
{    
  Task task;
  
  task = FindTask(data->name, data->id, task_ACTIVE);
  if (!task) {
    Warning(">> Warning in Process Reply: task %s {%d} not found as active\n",
	    data->name, data->id);
    return;
  } else if (task->type == task_QUERY) {
    ModifyTaskState(task, data->time, task_COMPLETED, NULL);
    if (task->creator->state == task_WAITING)
      ModifyTaskState(task->creator, data->time, task_ACTIVE, NULL);
  } else if (task->type == task_COMMAND) {
    Debug(True, "Ignoring reply for waitForCommand %s (%d)\n", 
	  task->name , task->id);
  } else {
    Warning("Reply processed for %s (%d) (not query/waitForCommand)\n", 
	    task->name, task->id);
  }
}


static void ProcessPointConstraint(MsgData data)
{
  PointTC pointTC1, pointTC2;
  Task task1, task2;
  
  task1 = FindTask(data->msg1, data->id1, task_NULL);
  task2 = FindTask(data->msg2, data->id2, task_NULL);
  if (!task1) 
    task1 = TaskCreate(TYPE_NULL, data->msg1, data->id1, INACTIVE, 0);
  if (!task2) 
    task2 = TaskCreate(TYPE_NULL, data->msg2, data->id2, INACTIVE, 0);
  pointTC1 = _new(_PointTC);
  pointTC1->my_interval = data->interval1;
  pointTC1->my_op = data->point1;
  pointTC1->task_interval = data->interval2;
  pointTC1->task_interval_op = data->point2;
  pointTC1->point_TC = data->point_TC;
  pointTC1->to_task = task2;
  ArrayInsert(task1->point_constraints, (Pointer) pointTC1);
  
  pointTC2 = _new(_PointTC);
  pointTC2->my_interval = data->interval2;
  pointTC2->my_op = data->point2;
  pointTC2->task_interval = data->interval1;
  pointTC2->task_interval_op = data->point1;
  pointTC2->point_TC = Inverse(data->point_TC);
  pointTC2->to_task = task1;
  ArrayInsert(task2->point_constraints, (Pointer) pointTC2);
}

static char Inverse(char point_TC)
{
  switch (point_TC) {
  case '=':
    return('=');
  case '<':
    return('>');
  case '>':
    return('<');
  case '/':      /* >= */
    return('\\');
  case '\\':
    return('/');
  }
  return(' ');
}


static void ProcessTemporalConstraint(MsgData data)
{
  Task task1, task2;
  TemporalConstraint TC1, TC2;
  
  
  task1 = FindTask(data->msg1, data->id1, task_NULL);
  if (!task1) 
    task1 = TaskCreate(TYPE_NULL, data->msg1, data->id1, INACTIVE, 0);
  TC1 = _new(_TemporalConstraint);
  TC1->temporal_constraints = data->temporal_constraint;
  ArrayInsert(task1->temporal_constraint, (Pointer) TC1);
  
  if (data->temporal_constraint != MSG_PLAN_FIRST &&
      data->temporal_constraint != MSG_DELAY_PLAN) {
    
    task2 = FindTask(data->msg2, data->id2, task_NULL);
    if (!task2) 
      task2 = TaskCreate(TYPE_NULL, data->msg2, data->id2, INACTIVE, 0);
    
    TC1->to_task = task2;
    
    TC2 = _new(_TemporalConstraint);
    TC2->temporal_constraints = 
      InverseTemporalConstraint(data->temporal_constraint);
    TC2->to_task = task1;
    ArrayInsert(task2->temporal_constraint, (Pointer) TC2);
  }
  else 
    TC1->to_task = NULL;
}

static temporalConstraint InverseTemporalConstraint(temporalConstraint tc)
{
  return (tc == MSG_SEQ_ACH ? MSG_INV_SEQ_ACH : MSG_INV_SEQ_PLAN);
}

static Array ArrayStates(taskState task_state)
{
  switch (task_state) {
  case task_ON_HOLD: return(inactive_tasks);
  case task_KILLED: return(killed_tasks);
  case task_PENDING: return(pending_tasks);
  case task_ACTIVE: return(active_tasks);
  case task_WAITING: return(waiting_tasks);
  case task_COMPLETED: return(completed_tasks);
#ifndef TEST_CASE_COVERAGE
  default:
    Debug(DEBUG_TASKTREE, "Can't find the array for that state\n");
    return(NULL);
#endif
  }
}

static Task FindTaskInArray(String name, int id, Array array)
{
  return((Task) ArrayFindCondition(array, (LIST_CONDITION_TYPE)TaskEqual, 
				   (Pointer) name, (Pointer) id));
}


static BOOLEAN TaskEqual(Task task, String name, int id)
{
  if (StringEqual(name, ""))
    return (task->id == id);
  else
    return(StringEqual(task->name, name) &&
	   RefIdEqual(task->id, id));
}


static void PrintTask(Task task)
{
  printf("%s {%d}\n", task->name, task->id);
}


static String GetModule(String msg_module_name, String task_name)
{
  Resource user_resource;
  String name;
  
  name = msg_module_name;
  Debug(DEBUG_TASKTREE, "The module name corresponding to %s", msg_module_name);
  if (StringEqual(msg_module_name, ON_HOLD_NAME))
    return(strdup(""));
  user_resource = FindResource(msg_module_name);
  if (user_resource){
    name = FindModuleOfHandlerUserResource(user_resource, task_name);
    if (name == NULL) {
      Warning("No module defined for task %s in user resources\n", task_name);
      return("");
    }
  }
  if (BeginsWith(msg_module_name, "Resource")) {
    name = SkipUntilSubString(msg_module_name, " ");
    name = SkipSpaces(name);
  }
  Debug(DEBUG_TASKTREE, " is %s\n", name);
  return(FindModule(name));
}


static String FindModule(String module_name)
{
  String static_name;
  
  static_name = (String)ArrayFindCondition(modules,
					   (LIST_CONDITION_TYPE)StringEqual,
					   (Pointer)module_name,
					   (Pointer)NULL);
  if (static_name == NULL) {
    static_name = strdup(module_name);
    ArrayInsert(modules, (Pointer) static_name);
  }
  return(static_name);
}


static Task FindTaskInModule(Array array_tasks, String module_name)
{
  return((Task) ArrayFindCondition(array_tasks, 
				   (LIST_CONDITION_TYPE)TaskInModule,
				   (Pointer) module_name, (Pointer) NULL));
}


static BOOLEAN TaskInModule(Task task, String module_name)
{
  return(StringEqual(task->module, module_name));
}

/*
 *  User resources stuff
 */


static String FindModuleOfHandlerUserResource(Resource resource,
					      String handler_name)
{
  Handlers handlers;
  
  ArrayRewind(resource->handlers);
  handlers = (Handlers) ArrayNext(resource->handlers);
  while (handlers) {
    if (ArrayFindCondition(handlers->names,
			   (LIST_CONDITION_TYPE) StringEqual, 
			   (Pointer) handler_name,  (Pointer) NULL))
      return(handlers->module_name);
    handlers = (Handlers) ArrayNext(resource->handlers);
  }
  return(NULL);
}


static Resource FindResource(String resource_name)
{
  return((Resource) ArrayFindCondition(user_resources, 
				       (LIST_CONDITION_TYPE)ResourceHasName, 
				       (Pointer) resource_name,
				       (Pointer) NULL));
}


static Handlers FindHandlers(String module_name)
{
  return((Handlers) ArrayFindCondition(user_resources, 
				       (LIST_CONDITION_TYPE)HandlersHasName, 
				       (Pointer) module_name,
				       (Pointer) NULL));
}


static BOOLEAN HandlersHasName(Handlers handlers, String name, Pointer data2
			       /*unused*/)
{
  return(StringEqual(handlers->module_name, name));
}


static BOOLEAN ResourceHasName(Resource resource, String name, Pointer data2
			       /*unused*/)
{
  return(StringEqual(resource->name, name));
}


static BOOLEAN IsNameResource(String name)
{
  if (BeginsWith(name, "Resource"))
    return(True);
  if (ArrayFindCondition(user_resources, 
			 (LIST_CONDITION_TYPE)StringEqual, (Pointer) name,
			 (Pointer) NULL))
    return(True);
  return(False);
}

#ifdef TEST   
void main(int argc, char **argv)
{
  static _MsgData data[] = {
    {GOAL, "Goal1", -1, "CTR", "PP", SENT, 0},
    {GOAL, "Goal2", -1, "PP", "TT", SENT, 1},
    {GOAL, "Goal3", -1, "PP", "UU", SENT, 1},
    {GOAL, "Goal4", -1, "PP", "VV", SENT, 1},
    {GOAL, "Goal5", -1, "TT", "QQ", SENT, 1},
    {GOAL, "Goal6", -1, "TT", "YY", SENT, 1},
    {GOAL, "Goal7", -1, "QQ", "ZZ", SENT, 1},
    {QUERY, "Is?", -1, "QQ", "Resource SS", PENDING, 1},
    {QUERY, "Is?", -1, "Resource SS", "SS", SENT, 1}
  };
  
  int i;
  
  TaskTreeInitialize();
  for (i=0; i < (sizeof(data) / sizeof(_MsgData)); i++)
    TaskTreeProcessNewMessage(&data[i]);
}

#endif

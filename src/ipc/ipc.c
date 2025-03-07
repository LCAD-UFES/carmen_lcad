/******************************************************************************
 * PROJECT: New Millennium, DS1
 *          IPC (Interprocess Communication) Package
 *
 * (c) Copyright 1996 Reid Simmons.  All rights reserved.
 *
 * FILE: ipc.c
 *
 * ABSTRACT: Implementation of IPC, using (modified) X_IPC library
 *
 * REVISION HISTORY
 *
 * $Log: ipc.c,v $
 * Revision 1.4  2008/07/23 08:40:51  kuemmerl
 * - const char* for ipc functions
 *
 * Revision 1.3  2006/02/26 03:35:25  nickr
 * Changes to address 64 bit warnings
 *
 * Revision 1.2  2005/12/12 23:51:36  nickr
 * Added support for PID access. Needed for java
 *
 * Revision 1.1.1.1  2004/10/15 14:33:15  tomkol
 * Initial Import
 *
 * Revision 1.8  2003/04/20 02:28:13  nickr
 * Upgraded to IPC 3.7.6.
 * Reversed meaning of central -s to be default silent,
 * -s turns silent off.
 *
 * Revision 2.13  2003/03/11 23:39:00  trey
 * Event handling while not connected to central:
 *   This is an experimental feature, disabled by default.  Enable it
 *   by compiling with 'gmake IPC_ALLOW_DISCONNECTED_EVENT_HANDLING=1 install'.
 *   When this feature is enabled, you can call event handling functions
 *   such as IPC_listen() and IPC_dispatch() while not connected to the
 *   central server (you must call IPC_initialize() first, though).  Of
 *   course, no IPC messages can be received, but events registered using
 *   IPC_addTimer() and IPC_subscribeFD() will be handled.
 *
 * Silently close connection to central in child process:
 *   If you fork() and exec() in a process after connecting to IPC, you
 *   should call the new function x_ipcCloseInternal(0) in the child before
 *   the exec() call.  This silently closes the child's copy of the pipe to
 *   central.  Then when the parent exits, central will correctly be
 *   notified that the module has disconnected.
 *
 *   This avoids a rare failure mode where the child outlives the parent,
 *   the parent shuts down using exit(), and the child's still-open copy of
 *   the pipe to central keeps central from finding out that the module is
 *   disconnected.
 *
 * Revision 2.12  2003/02/13 20:39:29  reids
 * Fixed bug in IPC_listenClear that could hang if central goes down.
 *
 * Revision 2.11  2002/01/03 20:52:12  reids
 * Version of IPC now supports multiple threads (Caveat: Currently only
 *   tested for Linux).
 * Also some minor changes to support Java version of IPC.
 *
 * Revision 2.10  2001/02/09 16:24:20  reids
 * Added IPC_getConnections to return list of ports that IPC is listening to.
 * Added IPC_freeDataElements to free the substructure (pointers) of a struct.
 *
 * Revision 2.9  2001/01/31 17:54:12  reids
 * Subscribe/unsubscribe to connections/disconnections of modules.
 * Subscribe/unsubscribe to changes in handler registrations for a message.
 *
 * Revision 2.8  2001/01/10 15:32:50  reids
 * Added the function IPC_subscribeData that automatically unmarshalls
 *   the data before invoking the handler.
 *
 * Revision 2.7  2000/08/14 21:28:34  reids
 * Added support for making under Windows.
 *
 * Revision 2.6  2000/07/27 16:59:10  reids
 * Added function IPC_setMsgQueueLength.
 * Made IPC_setMsgQueueLength and IPC_setMsgPriority work with point-to-point
 *   messages (used to work only with centrally-routed messages).
 * Made direct connections a bit more robust.
 *
 * Revision 2.5  2000/07/19 20:56:19  reids
 * Added IPC_listenWait
 *
 * Revision 2.4  2000/07/03 17:03:25  hersh
 * Removed all instances of "tca" in symbols and messages, plus changed
 * the names of all other symbols which conflicted with TCA.  This new
 * version of IPC should be able to interoperate TCA fully.  Client
 * programs can now link to both tca and ipc.
 *
 * Revision 2.3  2000/02/15 21:01:19  reids
 * Fixed the way IPC_defineMsg handles errors.
 *
 * Revision 2.2  2000/01/27 20:48:46  reids
 * Added more error checking for IPC_defineMsg.
 *
 * Revision 2.1.1.1  1999/11/23 19:07:34  reids
 * Putting IPC Version 2.9.0 under local (CMU) CVS control.
 *
 * Revision 1.10.2.12  1997/02/26 04:17:54  reids
 * Added IPC_isMsgDefined.
 *
 * Revision 1.10.2.11  1997/01/27 20:40:28  reids
 * Implement a function to check whether a given format string matches the
 *   one registered for a given message.
 *
 * Revision 1.10.2.10  1997/01/27 20:09:32  udo
 * ipc_2_6_006 to r3_Dev merge
 *
 * Revision 1.10.2.8  1997/01/16 22:18:00  reids
 * Check that IPC initialized before using global var.
 *
 * Revision 1.10.2.7  1997/01/11 01:21:01  udo
 * ipc 2.6.002 to r3_dev merge
 *
 * Revision 1.10.2.6.4.1  1996/12/24 14:41:38  reids
 * Merge the C and Lisp IPC libraries (both C and Lisp modules can now
 *   link with the same libipc.a).
 * Moved the Lisp-specific code to ipcLisp.c: Cleaner design and it will
 *   not be linked into C modules this way.
 *
 * Revision 1.10.2.6  1996/12/18 15:11:05  reids
 * Changed logging code to remove VxWorks dependence on varargs
 * Print name of module within IPC_perror
 *
 * Revision 1.10.2.5  1996/10/28 16:18:43  reids
 * Make IPC_Exit_On_Error the default "verbosity".
 *
 * Revision 1.10.2.4  1996/10/24 17:26:35  reids
 * Replace fprintf with x_ipcModWarning.
 *
 * Revision 1.10.2.3  1996/10/18 18:06:37  reids
 * Better error checking and reporting (IPC_errno, IPC_perror).
 * Added IPC_initialize so that you don't have to connect with central to
 *   do marshalling.
 * Added IPC_setVerbosity to set the level at which messages are reported.
 *
 * Revision 1.10.2.2  1996/10/14 03:54:40  reids
 * For NMP, added prioritized messages (i.e., prioritized pending queues).
 *
 * Revision 1.10.2.1  1996/10/02 20:58:36  reids
 * Changes to support LISPWORKS.
 *
 * Revision 1.12  1996/09/30 17:59:24  rouquett
 * Reid's suggestion.
 *
 * Revision 1.11  1996/09/06 22:30:31  pgluck
 * Removed static declarations for VxWorks
 *
 * Revision 1.10  1996/05/26 04:11:47  reids
 * Added function IPC_dataLength -- length of byte array assd with msgInstance
 *
 * Revision 1.9  1996/05/24 20:00:23  rouquett
 * swapped include order between ipc.h globalM.h for solaris compilation
 *
 * Revision 1.8  1996/05/09 01:01:31  reids
 * Moved all the X_IPC files over to the IPC directory.
 * Fixed problem with sending NULL data.
 * Added function IPC_setCapacity.
 * VxWorks m68k version released.
 *
 * Revision 1.7  1996/04/24 19:13:51  reids
 * Changes to support vxworks version.
 *
 * Revision 1.6  1996/04/03 02:45:47  reids
 * Made ipc.h C++ compatible.
 * Made IPC_listenClear wait a bit before returning, to catch pending msgs.
 * Made IPC_connect try a few times to connect with the central server.
 *
 * Revision 1.5  1996/04/01 02:36:07  reids
 * Needed to change x_ipcAddFn to x_ipcAddEventHandler.
 *
 * Revision 1.4  1996/03/31 23:49:37  reids
 * The X_IPC call x_ipcRemoveFd was changed to x_ipcRemoveEventHandler
 *
 * Revision 1.3  1996/03/06 20:20:42  reids
 * Version 2.3 adds two new functions: IPC_defineFormat and IPC_isConnected
 *
 * Revision 1.2  1996/03/05 04:29:09  reids
 * Added error checking to IPC_connect.
 *
 * Revision 1.1  1996/03/03 04:36:18  reids
 * First release of IPC files.  Corresponds to IPC Specifiction 2.2, except
 * that IPC_readData is not yet implemented.  Also contains "cover" functions
 * for the xipc interface.
 *
 ****************************************************************/

#include "ipc.h"
#include "globalM.h"
#include "ipcPriv.h"
#include "parseFmttrs.h"
#include "string.h"

IPC_ERROR_TYPE IPC_errno = IPC_No_Error;

const char *ipcErrorStrings[NUM_ERRORS] =
{ "No IPC error",
  "Not connected to IPC central",
  "IPC not initialized",
  "IPC message not defined",
  "Message is not fixed length",
  "Fixed length of message differs from length argument",
  "Argument out of range",
  "Argument is (unexpectedly) NULL",
  "Illegal format string",
  "Format string does not match that of registered message",
  "Size of byte array does not match dataSize argument",
  "Low level communication (socket) error"
};

IPC_VERBOSITY_TYPE ipcVerbosity = IPC_Exit_On_Errors;

IPC_RETURN_TYPE _IPC_initialize (BOOLEAN isLispModule)
{
  x_ipcModuleInitialize();
#ifdef LISP
  if (isLispModule) set_Is_Lisp_Module();
#else
  if (isLispModule) ; /* No-op to keep compiler happy */
#endif
  return IPC_OK;
}

IPC_RETURN_TYPE IPC_initialize (void)
{
  return _IPC_initialize(FALSE);
}

/* TNgo, 5/22/97, added another variable to this function
   so that it can be called by IPC_connectModule */
IPC_RETURN_TYPE _IPC_connect (const char *taskName,
			      const char *serverName,
			      BOOLEAN isLispModule)
{
  const char *serverHost;
  char *colon = NULL;
  int i, serverPort;
  struct timeval wait;

  /* Added by Bob Goode/Tam Ngo, 5/21/97, for WINSOCK option. */
#ifdef OS2
  sock_init();
#elif defined(_WINSOCK_)
  startWinsock();
#endif /* Winsock DLL loading */

  if (ipcVerbosity > IPC_Silent) {
    printf("NMP IPC Version %d.%d.%d (%s)\n", 
	   IPC_VERSION_MAJOR, IPC_VERSION_MINOR, IPC_VERSION_MICRO,
	   IPC_VERSION_DATE);
  }

  /* Modified by TNgo, 5/22/97 */
  serverHost = ((serverName != NULL && strlen(serverName) > 0) 
		? serverName : x_ipcServerMachine());

  for (i=0; i<MAX_RECONNECT_TRIES; i++) {
    if (i > 0 && ipcVerbosity > IPC_Silent)
      printf("  Trying again %s to connect with the central server on %s\n",
		      taskName, serverHost);
    _IPC_initialize(isLispModule);
    x_ipcConnectModule(taskName, serverHost);
    if (X_IPC_CONNECTED()) {
      x_ipcWaitUntilReady();
      if (ipcVerbosity > IPC_Silent) {
	colon = (serverHost ? (char *)index((char *)serverHost, ':') : NULL);
	serverPort = (colon == NULL ? SERVER_PORT : atoi(colon+1));
	X_IPC_MOD_WARNING1("... IPC Connected on port %d\n", serverPort);
      }
      return IPC_OK;
    } else {
      /* Need to do this, rather than sleep, because of SIGALRM's */
      wait.tv_sec = RECONNECT_WAIT; wait.tv_usec = 0;
      select(0, 0, 0, 0, &wait);
    }
  }
  /* At this point, we've been unable to connect */
  RETURN_ERROR(IPC_Not_Connected);
}

IPC_RETURN_TYPE IPC_connect (const char *taskName)
{
  return _IPC_connect(taskName, NULL, FALSE);
}

/* Added by TNgo, 5/22/97 */
IPC_RETURN_TYPE IPC_connectModule (const char *taskName,
				   const char *serverName)
{
  return _IPC_connect(taskName, serverName, FALSE);
}

IPC_RETURN_TYPE IPC_disconnect (void)
{
  x_ipcClose();
  return IPC_OK;
}

/* Returns TRUE (1) if the module is currently connected to the IPC server */
int IPC_isConnected (void)
{
  return X_IPC_CONNECTED();
}

int IPC_isModuleConnected (const char *moduleName)
{
  if (!moduleName || strlen(moduleName) == 0) {
    ipcSetError(IPC_Null_Argument); return -1;
  } else if (!X_IPC_CONNECTED()) {
    ipcSetError(IPC_Not_Connected); return -1;
  } else {
    int retVal;
    x_ipcQuery(IPC_MODULE_CONNECTED_QUERY, &moduleName, &retVal);
    return retVal;
  }
}

fd_set IPC_getConnections (void)
{
  if (!X_IPC_INITIALIZED()) {
    fd_set nullSet;
    FD_ZERO(&nullSet);
    return nullSet;
  } else {
    return *x_ipcGetConnections();
  }
}

IPC_RETURN_TYPE IPC_defineMsg (const char *msgName, unsigned int length,
			       const char *formatString)
{
  /***  INICIO DO CÓDIGO EXTRA PARA O IPC_WATCHER ***/
  if((IPC_isMsgDefined(CARMEN_IPC_WATCHER_NEW_MESSAGE_NAME) && (strcmp(msgName, CARMEN_IPC_WATCHER_NEW_MESSAGE_NAME) != 0)))
  {
    carmen_ipc_watcher_new_message msg;
    msg.msg_name = msgName;
    msg.formatString = formatString;
    msg.length = length;
    msg.timestamp = 0.0;
    msg.host = "ipc";
    IPC_RETURN_TYPE err = IPC_publishData(CARMEN_IPC_WATCHER_NEW_MESSAGE_NAME, &msg);
  }
  /***  FIM DO CÓDIGO EXTRA PARA O IPC_WATCHER ***/
  
  char msgFormat[20];

  if (!msgName || strlen(msgName) == 0) {
    RETURN_ERROR(IPC_Null_Argument);
  } else if (!X_IPC_CONNECTED()) {
    RETURN_ERROR(IPC_Not_Connected);
  } if (length != (unsigned int)IPC_VARIABLE_LENGTH && formatString != NULL &&
	length != (unsigned int)x_ipc_dataStructureSize(ParseFormatString(formatString))) {
    RETURN_ERROR(IPC_Mismatched_Formatter);
  } else {
    if (length == IPC_VARIABLE_LENGTH) {
      sprintf(msgFormat, "{int, <byte:1>}");
    } else if (length == 0) {
      msgFormat[0] = '\0';
    } else {
      sprintf(msgFormat, "[byte:%d]", length);
    }
    x_ipcRegisterMessage(msgName, BroadcastClass, msgFormat, formatString);

    return IPC_OK;
  }
}

int IPC_isMsgDefined (const char *msgName)
{
  if (!msgName || strlen(msgName) == 0) {
    ipcSetError(IPC_Null_Argument);
    return FALSE;
  } else if (!X_IPC_CONNECTED()) {
    ipcSetError(IPC_Not_Connected);
    return FALSE;
  } else {
    return x_ipcMessageRegistered(msgName);
  }
}

IPC_RETURN_TYPE IPC_publish (const char *msgName,
			     unsigned int length, BYTE_ARRAY content)
{
  MSG_PTR msg;
  CONST_FORMAT_PTR format;
  void *dataToSend;
  IPC_VARCONTENT_TYPE vc;

  if (!msgName || strlen(msgName) == 0) {
    RETURN_ERROR(IPC_Null_Argument);
  } else if (!X_IPC_CONNECTED()) {
    RETURN_ERROR(IPC_Not_Connected);
  } else {
    msg = x_ipc_msgFind(msgName);
    if (!msg) {
      RETURN_ERROR(IPC_Message_Not_Defined);
    } else {
      format = msg->msgData->msgFormat;
      if (ipcDataToSend(format, msgName, length, content, &dataToSend, &vc)
	  != IPC_OK) {
	PASS_ON_ERROR();
      } else {
	return ipcReturnValue(x_ipcBroadcast(msgName, dataToSend));
      }
    }
  }
}

IPC_RETURN_TYPE IPC_publishVC (const char *msgName,
			       IPC_VARCONTENT_PTR varcontent)
{
  if (!msgName || strlen(msgName) == 0) {
    RETURN_ERROR(IPC_Null_Argument);
  } else if (!varcontent) {
    RETURN_ERROR(IPC_Null_Argument);
  } else {
    return IPC_publish(msgName, varcontent->length, varcontent->content);
  }
}

IPC_RETURN_TYPE IPC_publishFixed (const char *msgName,
				  BYTE_ARRAY content)
{
  return IPC_publish(msgName, IPC_FIXED_LENGTH, content);
}

const char *IPC_msgInstanceName (MSG_INSTANCE msgInstance)
{
  if (!msgInstance) {
    /* RETURN_ERROR(IPC_Null_Argument); */  /* JGraham, 8/19/97 */
    ipcSetError(IPC_Null_Argument);         /* modified for g++ */
    return NULL;
  } else {
    return x_ipcReferenceName(msgInstance);
  }
}

IPC_RETURN_TYPE _IPC_subscribe (const char *msgName, const char *hndName,
				HANDLER_TYPE handler, void *clientData,
				int autoUnmarshall)
{
  HND_KEY_TYPE hndKey;
  HND_PTR hnd;

  if (!msgName || strlen(msgName) == 0) {
    RETURN_ERROR(IPC_Null_Argument);
  } else if (!X_IPC_CONNECTED()) {
    RETURN_ERROR(IPC_Not_Connected);
  } else {
    x_ipcRegisterHandler(msgName, hndName, handler);

    /* Set the client data */
    hndKey.num = 0; hndKey.str = hndName;
    LOCK_CM_MUTEX;
    hnd = GET_HANDLER(&hndKey);
    UNLOCK_CM_MUTEX;
    hnd->autoUnmarshall = autoUnmarshall;
    if (hnd->isRegistered && hnd->clientData != NO_CLIENT_DATA) {
      X_IPC_MOD_WARNING1("Resetting client data for handler %s\n", hndName);
    }
    hnd->isRegistered = TRUE;
    hnd->clientData = clientData;

    return IPC_OK;
  }
}

IPC_RETURN_TYPE IPC_subscribe (const char *msgName, HANDLER_TYPE handler,
			       void *clientData)
{
  char hndName[100];

  ipcHandlerName(msgName, handler, hndName);
  return _IPC_subscribe(msgName, hndName, handler, clientData, FALSE);
}

IPC_RETURN_TYPE IPC_subscribeData (const char *msgName, 
				   HANDLER_DATA_TYPE handler,
				   void *clientData)
{
  char hndName[100];

  ipcHandlerName(msgName, handler, hndName);
  return _IPC_subscribe(msgName, hndName, handler, clientData, TRUE);
}

IPC_RETURN_TYPE _IPC_unsubscribe (const char *msgName, const char *hndName)
{
  if (!msgName || strlen(msgName) == 0) {
    RETURN_ERROR(IPC_Null_Argument);
  } else if (!X_IPC_CONNECTED()) {
    RETURN_ERROR(IPC_Not_Connected);
  } else {
    x_ipcDeregisterHandler(msgName, hndName);
    return IPC_OK;
  }
}

IPC_RETURN_TYPE IPC_unsubscribe (const char *msgName, HANDLER_TYPE handler)
{
  char hndName[100];

  ipcHandlerName(msgName, handler, hndName);
  return _IPC_unsubscribe(msgName, hndName);
}

IPC_RETURN_TYPE IPC_subscribeFD (int fd, FD_HANDLER_TYPE handler,
			 	 void *clientData)
{
  x_ipcAddEventHandler(fd, (X_IPC_FD_HND_FN)handler, clientData);
  return IPC_OK;
}

IPC_RETURN_TYPE IPC_unsubscribeFD (int fd, FD_HANDLER_TYPE handler)
{
#ifdef UNUSED_PRAGMA
#pragma unused(handler)
#endif
  x_ipcRemoveEventHandler(fd);
  return IPC_OK;
}

IPC_RETURN_TYPE IPC_listen (unsigned int timeoutMSecs)
{
#ifndef IPC_ALLOW_DISCONNECTED_EVENT_HANDLING
  if (!X_IPC_CONNECTED()) {
    RETURN_ERROR(IPC_Not_Connected);
  }
#endif
  return ipcReturnValue(x_ipcHandleMessage(timeoutMSecs));
}

/* Listen until "timeoutMSecs", or until the queue of messages is cleared */
IPC_RETURN_TYPE IPC_listenClear (unsigned int timeoutMSecs)
{
  IPC_RETURN_TYPE retVal;

  if ((retVal = IPC_listen(timeoutMSecs)) == IPC_OK) {
    do {
      retVal = IPC_listen(LISTEN_CLEAR_WAIT);
    } while (retVal != IPC_Timeout && retVal != IPC_Error);
    return (retVal == IPC_Error ? IPC_Error : IPC_OK);
  }
  return retVal;
}

/* Listen for "timeoutMSecs"
   (unlike IPC_listen or IPC_listenClear, will not return before "timeoutMSecs
 */
IPC_RETURN_TYPE IPC_listenWait (unsigned int timeoutMSecs)
{
  IPC_RETURN_TYPE retVal;
  unsigned long endTime, now;

  endTime = x_ipc_timeInMsecs() + timeoutMSecs;

  do {
    retVal = IPC_listenClear(timeoutMSecs);
    now = x_ipc_timeInMsecs();
    timeoutMSecs = endTime - now;
  } while (retVal != IPC_Error && now <= endTime );

  return retVal;
}

/* Added by TNgo, 5/22/97 */
IPC_RETURN_TYPE IPC_handleMessage (unsigned int timeoutMSecs)
{
  X_IPC_RETURN_VALUE_TYPE retval;

  /* make sure NMP_IPC compiler option is defined to get
     the time unit in MSec in function x_ipcHandleMessage() */
  retval = x_ipcHandleMessage(timeoutMSecs);

  if      (retval == Success) { return IPC_OK; }
  else if (retval == TimeOut) { return IPC_Timeout; }
  else			      { return IPC_Error; }
}

IPC_RETURN_TYPE IPC_dispatch (void)
{
  while (IPC_listen(IPC_WAIT_FOREVER) != IPC_Error) {};
  /* If control reaches here, IPC_listen returned IPC_Error */
  PASS_ON_ERROR();
}

IPC_RETURN_TYPE IPC_setCapacity (int capacity)
{
  if (capacity < 1) {
    RETURN_ERROR(IPC_Argument_Out_Of_Range);
  } else if (!X_IPC_CONNECTED()) {
    RETURN_ERROR(IPC_Not_Connected);
  } else {
    LOCK_M_MUTEX;
    x_ipcRegisterResource(GET_M_GLOBAL(modNameGlobal), capacity);
    UNLOCK_M_MUTEX;
    return IPC_OK;
  }
}

IPC_RETURN_TYPE IPC_setMsgQueueLength (const char *msgName, int queueLength)
{
  MSG_PTR msg;

  if (queueLength < 1) {
    RETURN_ERROR(IPC_Argument_Out_Of_Range);
  } else if (!X_IPC_CONNECTED()) {
    RETURN_ERROR(IPC_Not_Connected);
  } else {
    msg = x_ipc_findOrRegisterMessage(msgName);
    msg->limit = queueLength;
    LOCK_M_MUTEX;
    x_ipcLimitPendingMessages(msgName, GET_M_GLOBAL(modNameGlobal), 
			      queueLength);
    UNLOCK_M_MUTEX;
    return IPC_OK;
  }
}

IPC_RETURN_TYPE IPC_setMsgPriority (const char *msgName, int priority)
{
  SET_MSG_PRIORITY_TYPE setMsgData;
  MSG_PTR msg;

  if (priority < 0) {
    RETURN_ERROR(IPC_Argument_Out_Of_Range);
  } else if (!X_IPC_CONNECTED()) {
    RETURN_ERROR(IPC_Not_Connected);
  } else {
    msg = x_ipc_findOrRegisterMessage(msgName);
    msg->priority = priority;

    setMsgData.msgName = (char*)msgName;
    setMsgData.priority = priority;

    return ipcReturnValue(x_ipcInform(IPC_SET_MSG_PRIORITY_INFORM,
				      &setMsgData));
  } 
}

unsigned int IPC_dataLength (MSG_INSTANCE msgInstance)
{
  if (!msgInstance) {
    RETURN_ERROR(IPC_Null_Argument);
  } else {
    return msgInstance->dataLength;
  }
}

void IPC_perror (const char *msg)
{
#ifdef VXWORKS
  LOCK_M_MUTEX;
  fprintf(stderr, "%s: ", (mGlobalp() ? GET_M_GLOBAL(modNameGlobal) : "???"));
  UNLOCK_M_MUTEX;
#endif
  if (msg && strlen(msg) > 0) {
    fprintf(stderr, "%s: ", msg);
  }
  fprintf(stderr, "%s\n", ipcErrorStrings[IPC_errno]);
}

IPC_RETURN_TYPE IPC_setVerbosity (IPC_VERBOSITY_TYPE verbosity)
{
  if (verbosity < IPC_Silent || verbosity > IPC_Exit_On_Errors) {
    RETURN_ERROR(IPC_Argument_Out_Of_Range);
  } else {
    ipcVerbosity = verbosity;
    return IPC_OK;
  }
}

static void ipcConnectHandler (MSG_INSTANCE ref, char **moduleNamePtr)
{
  CONNECT_DATA_PTR connectData;
  LIST_PTR list;

  LOCK_CM_MUTEX;
  list = GET_C_GLOBAL(connectNotifyList);
  UNLOCK_M_MUTEX;

  for (connectData = (CONNECT_DATA_PTR)x_ipc_listFirst(list); connectData;
       connectData = (CONNECT_DATA_PTR)x_ipc_listNext(list)) {
    (connectData->handler)(*moduleNamePtr, connectData->clientData);
  }

  x_ipcFreeData(IPC_msgInstanceName(ref), moduleNamePtr); /* ??? */
}

static void ipcDisconnectHandler (MSG_INSTANCE ref, char **moduleNamePtr)
{
  CONNECT_DATA_PTR disconnectData;
  LIST_PTR list;

  LOCK_CM_MUTEX;
  list = GET_C_GLOBAL(disconnectNotifyList);
  UNLOCK_CM_MUTEX;

  for (disconnectData = (CONNECT_DATA_PTR)x_ipc_listFirst(list); disconnectData;
       disconnectData = (CONNECT_DATA_PTR)x_ipc_listNext(list)) {
    (disconnectData->handler)(*moduleNamePtr, disconnectData->clientData);
  }

  x_ipcFreeData(IPC_msgInstanceName(ref), moduleNamePtr); /* ??? */
}

IPC_RETURN_TYPE IPC_subscribeConnect (CONNECT_HANDLE_TYPE handler,
				      void *clientData)
{
  if (!X_IPC_CONNECTED()) {
    RETURN_ERROR(IPC_Not_Connected);
  } else {
    LOCK_CM_MUTEX;
    insertConnect(&GET_C_GLOBAL(connectNotifyList), 
		  IPC_CONNECT_NOTIFY_MSG, (X_IPC_HND_FN)ipcConnectHandler,
		  handler, clientData);
    UNLOCK_CM_MUTEX;
    return IPC_OK;
  }
}

IPC_RETURN_TYPE IPC_subscribeDisconnect (CONNECT_HANDLE_TYPE handler,
					 void *clientData)
{
  if (!X_IPC_CONNECTED()) {
    RETURN_ERROR(IPC_Not_Connected);
  } else {
    LOCK_CM_MUTEX;
    insertConnect(&GET_C_GLOBAL(disconnectNotifyList),
		  IPC_DISCONNECT_NOTIFY_MSG, (X_IPC_HND_FN)ipcDisconnectHandler,
		  handler, clientData);
    UNLOCK_CM_MUTEX;
    return IPC_OK;
  }
}

IPC_RETURN_TYPE IPC_unsubscribeConnect (CONNECT_HANDLE_TYPE handler)
{
  if (!X_IPC_CONNECTED()) {
    RETURN_ERROR(IPC_Not_Connected);
  } else {
    LOCK_CM_MUTEX;
    removeConnect(GET_C_GLOBAL(connectNotifyList), IPC_CONNECT_NOTIFY_MSG,
		  handler);
    UNLOCK_CM_MUTEX;
    return IPC_OK;
  }
}

IPC_RETURN_TYPE IPC_unsubscribeDisconnect (CONNECT_HANDLE_TYPE handler)
{
  if (!X_IPC_CONNECTED()) {
    RETURN_ERROR(IPC_Not_Connected);
  } else {
    LOCK_CM_MUTEX;
    removeConnect(GET_C_GLOBAL(disconnectNotifyList), 
		  IPC_DISCONNECT_NOTIFY_MSG, handler);
    UNLOCK_CM_MUTEX;
    return IPC_OK;
  }
}

IPC_RETURN_TYPE IPC_subscribeHandlerChange (const char *msgName,
					    CHANGE_HANDLE_TYPE handler,
					    void *clientData)
{
  MSG_PTR msg;

  if (!msgName || strlen(msgName) == 0) {
    RETURN_ERROR(IPC_Null_Argument);
  } else if (!X_IPC_CONNECTED()) {
    RETURN_ERROR(IPC_Not_Connected);
  } else {
    msg = x_ipc_msgFind2(msgName, NULL);
    if (!msg) {
      RETURN_ERROR(IPC_Message_Not_Defined);
    } else {
      insertChange(msgName, handler, clientData);
      return IPC_OK;
    }
  }
}

IPC_RETURN_TYPE IPC_unsubscribeHandlerChange (const char *msgName,
					      CHANGE_HANDLE_TYPE handler)
{
  if (!msgName || strlen(msgName) == 0) {
    RETURN_ERROR(IPC_Null_Argument);
  } else if (!X_IPC_CONNECTED()) {
    RETURN_ERROR(IPC_Not_Connected);
  } else {
    removeChange(msgName, handler);
    return IPC_OK;
  }
}

int IPC_numHandlers (const char *msgName)
{
  DIRECT_MSG_TYPE directInfo;
  X_IPC_RETURN_VALUE_TYPE status;
  int num;

  if (!msgName || strlen(msgName) == 0) {
    ipcSetError(IPC_Null_Argument); return -1;
  } else if (!X_IPC_CONNECTED()) {
    ipcSetError(IPC_Not_Connected); return -1;
  } else {
    status = x_ipcQueryCentral(X_IPC_DIRECT_MSG_QUERY, (void *)&msgName,
			       (void *)&directInfo);
    if (status == Success) {
      num = directInfo.numHandlers;
      x_ipcFreeReply(X_IPC_DIRECT_MSG_QUERY, &directInfo);
    } else {
      num = 0;
    }
    return num;
  }
}

/****************************************************************
 *                INTERNAL FUNCTIONS 
 ****************************************************************/


/* Check that the length used in the publish matches the message format length.
 * If the message is fixed-length:
 *  Return TRUE if length is IPC_FIXED_LENGTH or the lengths agree.
 * If the message is variable-length:
 *  Return TRUE unless length is IPC_VARIABLE_LENGTH or IPC_FIXED_LENGTH.
 */
static BOOLEAN ipcCheckPublishLength (unsigned int length, 
				      CONST_FORMAT_PTR format,
				      FORMAT_CLASS_TYPE formatClass)
{
  if (!format) {
    return (length == IPC_FIXED_LENGTH || length == 0);
  } else if (formatClass == FixedArrayFMT) { 
    /* Same fixed length? */
    return (length == IPC_FIXED_LENGTH ||
	    length == (unsigned)format->formatter.a[2].i);
  } else if (formatClass == VarArrayFMT) {
    return ((length != IPC_VARIABLE_LENGTH) && (length != IPC_FIXED_LENGTH));
  } else {
    /* Something is wrong here! */
    return FALSE;
  }
}

/* Check that the format is OK for IPC, and set the "dataHandle" to
   point to the data (either a byte-array or IPC_VARCONTENT_PTR) to be sent.
   Last argument added to make this re-entrant, but without need for malloc */
IPC_RETURN_TYPE ipcDataToSend (CONST_FORMAT_PTR format, const char *msgName,
			       unsigned int length, BYTE_ARRAY content,
			       void **dataHandle, IPC_VARCONTENT_PTR varcontent)
{
  FORMAT_CLASS_TYPE formatClass;
  
  if (!format) {
    if (length == 0 || length == IPC_FIXED_LENGTH) {
      *dataHandle = NULL;
      return IPC_OK;
    } else {
      X_IPC_MOD_WARNING2("Illegal length argument %d for zero-length message\n %s",
		    length, msgName);
      RETURN_ERROR(IPC_Message_Lengths_Differ);
    }
  } else {
    formatClass = ipcFormatClassType(format);
    if (formatClass != FixedArrayFMT && formatClass != VarArrayFMT) {
      X_IPC_MOD_WARNING1("Illegal format for message %s\n", msgName);
      RETURN_ERROR(IPC_Illegal_Formatter);
    } else if (!ipcCheckPublishLength(length, format, formatClass)) {
      X_IPC_MOD_WARNING3("Illegal length argument %d for %s-length message %s\n",
		    length, (formatClass==FixedArrayFMT ? "fixed" : "variable"),
		    msgName);
      RETURN_ERROR(IPC_Message_Lengths_Differ);
    } else if (formatClass == FixedArrayFMT) {
      *dataHandle = content;
      return IPC_OK;
    } else { /* VarArrayFMT */
      varcontent->length = length;
      varcontent->content = content;
      *dataHandle = (void *)varcontent;
      return IPC_OK;
    }
  }
}

/* Transform a X_IPC_RETURN_VALUE_TYPE to an IPC_RETURN_TYPE */
IPC_RETURN_TYPE ipcReturnValue(X_IPC_RETURN_VALUE_TYPE retVal)
{
  if (retVal == Success) {
    return IPC_OK;
  } else if (retVal == TimeOut) {
    return IPC_Timeout;
  } else {
    RETURN_ERROR(IPC_Communication_Error);
  }
}

/* Create a handler name from the message name and the handler function.
   Stick the handler name into the "hndName" string (already allocated!) */
void ipcHandlerName (const char *msgName, HANDLER_TYPE handler, char *hndName)
{
#if (defined(__x86_64__))
  sprintf(hndName, "HND-%s%ld", msgName, (long)handler);
#else
  sprintf(hndName, "HND-%s%d", msgName, (int)handler);
#endif
}

void ipcSetError  (IPC_ERROR_TYPE error)
{
  IPC_errno = error;

  if (ipcVerbosity >= IPC_Print_Errors) {
    IPC_perror("ERROR");
  }
  if (ipcVerbosity >= IPC_Exit_On_Errors) {
    X_IPC_MOD_ERROR0();
  }
}

/* Added by TNgo, 5/14/98, to support multiple servers. */
IPC_RETURN_TYPE IPC_setContext (IPC_CONTEXT_PTR context)
{
  if (!context) {
    RETURN_ERROR(IPC_Null_Argument);
  } else {
    x_ipcSetContext((X_IPC_CONTEXT_PTR)context);
    return IPC_OK;
  }
}

IPC_CONTEXT_PTR IPC_getContext (void)
{
  return (IPC_CONTEXT_PTR)x_ipcGetContext();
}

long IPC_getPID (void)
{
  return (long)getpid();
}

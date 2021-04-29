/******************************************************************************
 *
 * PROJECT: IPC: Inter-Process Communication Package
 * 
 * (c) Copyright 2001 Reid Simmons.  All rights reserved.
 *
 * FILE: multiThread.h
 *
 * ABSTRACT: Enable IPC to deal with multi-threaded programs.
 *           Mutexes loosely based on David Apfelbaum's THREADS package.
 *
 * REVISION HISTORY
 *
 * $Log: multiThread.h,v $
 * Revision 1.1.1.1  2004/10/15 14:33:15  tomkol
 * Initial Import
 *
 * Revision 1.2  2003/04/20 02:28:13  nickr
 * Upgraded to IPC 3.7.6.
 * Reversed meaning of central -s to be default silent,
 * -s turns silent off.
 *
 * Revision 2.1  2002/01/03 20:52:14  reids
 * Version of IPC now supports multiple threads (Caveat: Currently only
 *   tested for Linux).
 * Also some minor changes to support Java version of IPC.
 *
 *
 * $Revision: 1.1.1.1 $
 * $Date: 2004/10/15 14:33:15 $
 * $Author: tomkol $
 *
 *****************************************************************************/

#ifndef INCmultiThread
#define INCmultiThread

#include <pthread.h>
#include <sys/time.h>

/* In between Red Hat 5.2 and Red Hat 6.2, the invocation format
 * for setting the mutex type changed.  While the pthreads library
 * maintains a backward compatibility mode, there is a newer invocation format.
 */
#if defined ( REDHAT_6 )
#define PTHREAD_MUTEX_SETKIND pthread_mutexattr_settype
#elif defined ( REDHAT_52 )
#define PTHREAD_MUTEX_SETKIND pthread_mutexattr_setkind_np
#elif defined ( PTHREAD_CREATE_JOINABLE )	/* Try to guess it... */
#define PTHREAD_MUTEX_SETKIND pthread_mutexattr_settype
/* This may produce a warning message, but the         *
 * pthreads library is, in fact, backwards compatible. */
#else
#define PTHREAD_MUTEX_SETKIND pthread_mutexattr_setkind_np
#endif

typedef struct {
  pthread_mutexattr_t  mutexAttributes;
  pthread_mutex_t      mutexData;
} MUTEX_TYPE, *MUTEX_PTR;

typedef struct {
  pthread_cond_t pingVar;
  MUTEX_TYPE     mutex;
} PING_THREAD_TYPE, *PING_THREAD_PTR;

typedef enum { Mutex_Success, Mutex_Failure, 
	       Mutex_Already_Locked } MUTEX_STATUS;

typedef enum { Ping_Success, Ping_Failure, Ping_Timeout } PING_STATUS;

MUTEX_STATUS initMutex(MUTEX_PTR mutex);
MUTEX_STATUS destroyMutex(MUTEX_PTR mutex);
MUTEX_STATUS lockMutex(MUTEX_PTR mutex);
MUTEX_STATUS unlockMutex(MUTEX_PTR mutex);
MUTEX_STATUS tryLockMutex(MUTEX_PTR mutex);

PING_STATUS initPing(PING_THREAD_PTR ping);
PING_STATUS pingThreads(PING_THREAD_PTR ping);
PING_STATUS waitForPing(PING_THREAD_PTR ping, struct timeval *timeout);

#endif /* INCmultiThread */

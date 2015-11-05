/******************************************************************************
 *
 * PROJECT: IPC: Inter-Process Communication Package
 * 
 * (c) Copyright 2001 Reid Simmons.  All rights reserved.
 *
 * FILE: multiThread.c
 *
 * ABSTRACT: Enable IPC to deal with multi-threaded programs.
 *           Mutexes loosely based on David Apfelbaum's THREADS package.
 *
 * REVISION HISTORY
 *
 * $Log: multiThread.c,v $
 * Revision 1.1.1.1  2004/10/15 14:33:15  tomkol
 * Initial Import
 *
 * Revision 1.2  2003/04/20 02:28:13  nickr
 * Upgraded to IPC 3.7.6.
 * Reversed meaning of central -s to be default silent,
 * -s turns silent off.
 *
 * Revision 2.2  2003/04/14 15:31:01  reids
 * Updated for Windows XP
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

#include <stdio.h>
#include <errno.h>

#ifdef THREADED
#include "multiThread.h"

MUTEX_STATUS initMutex(MUTEX_PTR mutex)
{
  int result;

  memset(mutex, 0, sizeof(MUTEX_TYPE));

  /* Create the Mutex Attributes */
  pthread_mutexattr_init(&mutex->mutexAttributes);

  result = PTHREAD_MUTEX_SETKIND(&mutex->mutexAttributes,
				 PTHREAD_MUTEX_RECURSIVE_NP);
  switch (result) {
  case 0:  /* It worked */
    pthread_mutex_init(&mutex->mutexData,
		       (result == 0 ? &mutex->mutexAttributes : NULL));
    return Mutex_Success;

  case EINVAL:
    fprintf(stderr, "[initMutex]  Programmer Error:  failed with EINVAL.");
    return Mutex_Failure;

  default:  /* What the hell? */
    fprintf(stderr, "[initMutex]  UNKNOWN Error: failed with: %d\n.", result);
    return Mutex_Failure;
  }
}

MUTEX_STATUS destroyMutex(MUTEX_PTR mutex)
{
  return Mutex_Success;
}

MUTEX_STATUS lockMutex(MUTEX_PTR mutex)
{
  int result;
  result = pthread_mutex_lock(&mutex->mutexData);
  switch (result) {
  case 0:  /* It worked */
    return Mutex_Success;

  case EINVAL:
    fprintf(stderr, "[lockMutex]  Error:  [EINVAL]  "
	    "Unable to lock improperly initialized mutex.\n");
    break;

  case EDEADLK:  /* Mutex::ERROR_CHECK only */
    fprintf(stderr, "[lockMutex]  Error:  [EDEADLK]  "
	    "Unable to lock mutex:  Mutex already locked by calling thread.\n");
    break;

  default:
    fprintf(stderr,
	    "[lockMutex]  Error:  pthread_mutex_lock(.) returned: %d.\n",
	    result);
    break;
  }

  return Mutex_Failure;
}

MUTEX_STATUS unlockMutex(MUTEX_PTR mutex)
{
  int result = pthread_mutex_unlock(&mutex->mutexData);
  switch (result) {
  case 0:  /* It worked */
    return Mutex_Success;

  case EINVAL:
    fprintf(stderr, "[unlockMutex]  Error:  [EINVAL]  "
	    "Unable to unlock improperly initialized mutex.\n");
    break;

  case EPERM:  /* Mutex::ERROR_CHECK only */
    fprintf(stderr, "[unlockMutex]  Error:  [EPERM]  "
	    "Unable to unlock Mutex:  Calling thread does not own the mutex.\n");
    break;

  default:
    fprintf(stderr,
	    "[unlockMutex]  Error:  pthread_mutex_unlock(.) returned: %d.\n",
	    result);
    break;
  }

  return Mutex_Failure;
}

MUTEX_STATUS tryLockMutex(MUTEX_PTR mutex)
{
  int result = pthread_mutex_trylock(&mutex->mutexData);
  switch (result) {
  case 0:  /* It worked */
    return Mutex_Success;

  case EINVAL:
    fprintf(stderr, "[tryLockMutex]  Error:  [EINVAL]  "
	    "Unable to lock improperly initialized mutex.\n");
    break;

  case EBUSY:  /* Already locked */
    return Mutex_Already_Locked;

  case EDEADLK:  /* Mutex::ERROR_CHECK only */
    fprintf(stderr, "[tryLockMutex]  Error:  [EDEADLK]  "
	    "Unable to lock mutex:  Mutex already locked by calling thread.\n");
    break;

  default:
    fprintf(stderr,
	    "[tryLockMutex]  Error:  pthread_mutex_trylock(.) returned: %d.\n",
	    result);
    break;
  }

  return Mutex_Failure;
}

PING_STATUS initPing(PING_THREAD_PTR ping)
{
  return (initMutex(&ping->mutex) == Mutex_Success &&
	  pthread_cond_init(&ping->pingVar, NULL) == 0
	  ? Ping_Success : Ping_Failure);
}

PING_STATUS pingThreads(PING_THREAD_PTR ping)
{
  lockMutex(&ping->mutex);
  //fprintf(stderr, "PINGING\n");
  pthread_cond_broadcast(&ping->pingVar);
  unlockMutex(&ping->mutex);

  return Ping_Success;
}

PING_STATUS waitForPing(PING_THREAD_PTR ping, struct timeval *timeout)
{
  int retcode;
  struct timeval now;
  struct timespec ptimeout;

  lockMutex(&ping->mutex);
  if (timeout == NULL) {
    retcode = pthread_cond_wait(&ping->pingVar, &ping->mutex.mutexData);
    //fprintf(stderr, "PINGED (%ld)!\n", pthread_self());
  } else {
    gettimeofday(&now, NULL);
    ptimeout.tv_nsec = (now.tv_usec + timeout->tv_usec) * 1000;
    if (ptimeout.tv_nsec >= 1000000000) {
      now.tv_sec++;
      ptimeout.tv_nsec -= 1000000000;
    }
    ptimeout.tv_sec = now.tv_sec + timeout->tv_sec;
    retcode = pthread_cond_timedwait(&ping->pingVar, &ping->mutex.mutexData,
				     &ptimeout);
    fprintf(stderr, "PINGED WAIT (%ld)!\n", pthread_self());
  }
  unlockMutex(&ping->mutex);
  return (retcode == ETIMEDOUT ? Ping_Timeout : Ping_Success);
}

#endif /* THREADED */

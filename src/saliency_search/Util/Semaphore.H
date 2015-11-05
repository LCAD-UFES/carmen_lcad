/*!@file Util/Semaphore.H */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/Semaphore.H $
// $Id: Semaphore.H 13883 2010-09-04 05:13:13Z dberg $
//

#ifndef UTIL_SEMAPHORE_H_DEFINED
#define UTIL_SEMAPHORE_H_DEFINED

#include "Util/log.H"

#include <cerrno>
#include <pthread.h>
#include <semaphore.h> // for sem_t

class PosixSemaphore;
class CondVarSemaphore;

#ifdef HAVE_WORKING_POSIX_SEMAPHORES
typedef PosixSemaphore Semaphore;
#else
typedef CondVarSemaphore Semaphore;
#endif

/// Wrapper around posix sem_t semaphore
class PosixSemaphore
{
public:
  PosixSemaphore(unsigned int initvalue = 0)
  {
    if (0 != sem_init(&itsSem, 0, initvalue))
      PLFATAL("sem_init() failed");
  }

  ~PosixSemaphore()
  {
    if (0 != sem_destroy(&itsSem))
      PLERROR("sem_destroy() failed");
  }

  /// Returns true on success
  bool post()
  {
    if (0 != sem_post(&itsSem))
      {
        // Don't PLFATAL() here because we may call post from
        // destructor chains:
        PLERROR("sem_post() failed");
        return false;
      }
    return true;
  }

  //return true if a lock was performed
  bool trywait()
  {
    errno = 0;
    if(0 == sem_trywait(&itsSem))
      return true;
    else
      {
        if(errno == EAGAIN)
          {
            errno = 0;
          }
        else
          {
            PLERROR("sem_trywait() failed");
          }
        return false;
      }
  }

  /// Returns true on success
  bool wait()
  {
    errno = 0;
    while (0 != sem_wait(&itsSem))
      {
        if (errno == EINTR)
          {
            PLERROR("sem_wait() failed; retrying... ");
            continue;
          }
        else
          {
            PLERROR("sem_wait() failed");
            return false;
          }
      }
    return true;
  }

private:
  PosixSemaphore(const PosixSemaphore&); // not implemented
  PosixSemaphore& operator=(const PosixSemaphore&); // not implemented

  sem_t itsSem;
};

class CondVarSemaphore
{
public:
  CondVarSemaphore(unsigned int initvalue = 0)
    :
    itsCounter(initvalue)
  {
    if (0 != pthread_mutex_init(&itsMutex, NULL))
      PLFATAL("pthread_mutex_init() failed");

    if (0 != pthread_cond_init(&itsCond, NULL))
      PLFATAL("pthread_cond_init() failed");
  }

  ~CondVarSemaphore()
  {
    if (0 != pthread_cond_destroy(&itsCond))
      PLERROR("pthread_cond_destroy() failed");

    if (0 != pthread_mutex_destroy(&itsMutex))
      PLERROR("pthread_mutex_destroy() failed");
  }

  bool post()
  {
    bool retval = true;

    if (0 != pthread_mutex_lock(&itsMutex)) retval = false;
    ++itsCounter;
    if (0 != pthread_cond_signal(&itsCond)) retval = false;
    if (0 != pthread_mutex_unlock(&itsMutex)) retval = false;

    return retval;
  }

  bool trywait()
  {
    bool retval = true;
    if (0 != pthread_mutex_lock(&itsMutex)) retval = false;
    
    if (itsCounter > 0)
      --itsCounter;
    
    if (0 != pthread_mutex_unlock(&itsMutex)) retval = false;
    return retval;
  }
  
  bool wait()
  {
    bool retval = true;

    if (0 != pthread_mutex_lock(&itsMutex)) retval = false;

    while (itsCounter == 0)
      if (0 != pthread_cond_wait(&itsCond, &itsMutex)) retval = false;

    --itsCounter;

    if (0 != pthread_mutex_unlock(&itsMutex)) retval = false;

    return retval;
  }

private:
  CondVarSemaphore(const CondVarSemaphore&); // not implemented
  CondVarSemaphore& operator=(const CondVarSemaphore&); // not implemented

  volatile unsigned int itsCounter;
  pthread_cond_t itsCond;
  pthread_mutex_t itsMutex;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_SEMAPHORE_H_DEFINED

/**
   \file  Robots/LoBot/misc/LoCondition.H
   \brief An object-oriented wrapper around pthreads condition variables.
*/

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
// Primary maintainer for this file: mviswana usc edu
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/thread/LoCondition.H $
// $Id: LoCondition.H 13523 2010-06-07 00:22:03Z mviswana $
//

#ifndef LOBOT_CONDITION_VARIABLE_DOT_H
#define LOBOT_CONDITION_VARIABLE_DOT_H

//------------------------------ HEADERS --------------------------------

// POSIX threads
#ifdef INVT_HAVE_LIBPTHREAD

#include <pthread.h>

#else // fake pthreads API to allow builds to succeed

typedef int pthread_mutex_t ;
typedef int pthread_cond_t  ;
typedef struct {} timeval   ;
typedef struct {} timespec  ;

static void pthread_mutex_lock  (pthread_mutex_t*) {}
static void pthread_mutex_unlock(pthread_mutex_t*) {}

static void pthread_cond_wait     (pthread_cond_t*, pthread_mutex_t*) {}
static void pthread_cond_timedwait(pthread_cond_t*, pthread_mutex_t*,
                                   struct timespec*)                  {}

static void pthread_cond_signal   (pthread_cond_t*) {}
static void pthread_cond_broadcast(pthread_cond_t*) {}

#endif

// Standard Unix/C headers
#include <sys/time.h>
#include <errno.h>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Condition
   \brief A simple encapsulation of pthread condition variables.
*/
class Condition {
   // Prevent copy and assignment
   Condition(const Condition&) ;
   Condition& operator=(const Condition&) ;

   /// A condition variable is always associated with a mutex.
   pthread_mutex_t m_mutex ;

   /// The underlying pthread condition variable.
   pthread_cond_t m_cond ;

   /// A helper class to lock and unlock the mutex automatically so that
   /// exceptions in client code don't result in the mutex remaining
   /// locked. The constructor locks the mutex and the destructor unlocks
   /// it. This class is meant to be instantiated on the stack to ensure
   /// that the desctructor gets called when the variable goes out of
   /// scope. Thus, no special processing will be needed in case of
   /// exceptions in client code; that is, the Condition class's
   /// functions don't need to explicitly catch and rethrow exceptions
   /// and take care of unlocking the mutex in the exception handlers.
   ///
   /// DEVNOTE: We could reuse lobot::AutoMutex here instead of
   /// reimplementing its functionality. However, it seemed like a good
   /// idea to keep the Condition class entirely self-contained so that
   /// it could be reused without requiring clients to forcibly also rely
   /// on the lobot Mutex encapsulation. Instead, this class relies only
   /// and solely on pthreads.
   class AutoMutex {
      pthread_mutex_t& mutex ;
   public:
      AutoMutex(pthread_mutex_t&) ;
      ~AutoMutex() ;
   } ;

public:
   /// Default constructor: sets up the pthread condition variable.
   Condition() ;

   /// This function can be used by clients to wait on this condition
   /// variable. The wait is forever, i.e., the calling thread will wait
   /// until the condition is signaled by some other thread.
   ///
   /// This function expects to be passed a predicate function or
   /// function object that should return true if the shared data being
   /// waited on is in the desired state, i.e., the state in which the
   /// condition variable is flagged as being satisfied. If the predicate
   /// returns false, the calling thread will continue to wait.
   ///
   /// Thus, this function will block the calling thread until the
   /// predicate returns true.
   ///
   /// Basically, this function implements the usual wait pattern to be
   /// applied for pthread condition variables so that clients need only
   /// implement the actual test required to decide whether the condition
   /// variable is satisfied or not.
   template<typename F> void wait(F pred) ;

   /// This function can be used by clients to wait on this condition
   /// variable. The wait is timed, i.e., the calling thread will wait
   /// until either the condition is signaled by some other thread or
   /// until the wait times out.
   ///
   /// This function expects to be passed a predicate function or
   /// function object that should return true if the shared data being
   /// waited on is in the desired state, i.e., the state in which the
   /// condition variable is flagged as being satisfied. If the predicate
   /// returns false, the calling thread will continue to wait.
   ///
   /// Additionally, the function should be passed a timeout value in
   /// milliseconds.
   ///
   /// Thus, this function will block the calling thread until the
   /// client-supplied test condition becomes true or until the timeout
   /// is exceeded.
   ///
   /// This function will return true to indicate that the wait completed
   /// successfully, i.e., the condition being waited on is signaled. A
   /// false return value indicates that the wait timed out.
   ///
   /// Basically, this function implements the usual timed-wait pattern
   /// to be applied for pthread condition variables so that clients
   /// need only implement the actual test required to decide whether the
   /// condition variable is satisfied or not.
   template<typename F> bool wait(F pred, int timeout) ;

   /// This function signals any one thread waiting on this condition
   /// variable that the condition has been satisfied.
   ///
   /// The function expects to be passed a predicate function or function
   /// object that should return true to indicate that the shared data
   /// being waited on is in the desired state, i.e., the state in which
   /// the condition variable is flagged as being satisfied. If the
   /// predicate returns false, the calling thread will not signal the
   /// condition variable and waiting threads will continue to wait.
   ///
   /// Additionally, the predicate function/object should also perform
   /// the necessary operations to actually bring the shared data into
   /// the state in which the condition variable can be signaled.
   ///
   /// Basically, this function implements the usual pthread signaling
   /// pattern for condition variables so that clients need only
   /// implement the actual test required to decide whether the condition
   /// variable is satisfied or not.
   template<typename F> void signal(F pred) ;

   /// This function signals all threads waiting on this condition
   /// variable that the condition has been satisfied.
   ///
   /// The function expects to be passed a predicate function or function
   /// object that should return true to indicate that the shared data
   /// being waited on is in the desired state, i.e., the state in which
   /// the condition variable is flagged as being satisfied. If the
   /// predicate returns false, the calling thread will not signal the
   /// condition variable and waiting threads will continue to wait.
   ///
   /// Additionally, the predicate function/object should also perform
   /// the necessary operations to actually bring the shared data into
   /// the state in which the condition variable can be signaled.
   ///
   /// Basically, this function implements the usual pthread signaling
   /// pattern for condition variables so that clients need only
   /// implement the actual test required to decide whether the condition
   /// variable is satisfied or not.
   template<typename F> void broadcast(F pred) ;

   /// This function allows clients to execute some function under the
   /// protection of the mutex associated with this condition variable.
   /// The condition variable itself is not involved; only its associated
   /// mutex comes into play. The mutex is locked prior to calling the
   /// client-supplied function or funcion object and unlocked after it.
   template<typename F> void protect(F func) ;

   /// Clean-up: release the pthread condition variable and its
   /// associated mutex.
   ~Condition() ;
} ;

//--------------------- TEMPLATE MEMBER FUNCTIONS -----------------------

#ifdef INVT_HAVE_LIBPTHREAD

//------------------------------ WAITING --------------------------------

// Wait on condition variable until client-supplied test becomes true
template<typename F>
void Condition::wait(F pred)
{
   AutoMutex M(m_mutex) ;
   while (! pred())
      pthread_cond_wait(&m_cond, &m_mutex) ;
}

// Timed-wait on condition variable until client-supplied test becomes true
template<typename F>
bool Condition::wait(F pred, int timeout_ms)
{
   AutoMutex M(m_mutex) ;

   // pthread_cond_timedwait() needs an absolute time to wait until. So
   // we need to express the supplied timeout delay as the current time
   // plus delay.
   struct timeval now ;
   if (gettimeofday(&now, 0) == -1) // error getting current time
      return false ;

   // Convert current time and timeout delay to nanoseconds and add them
   // all to yield desired wait time for pthread_cond_timedwait().
   long long to = static_cast<long long>(now.tv_sec)  * 1000000000L
                + static_cast<long long>(now.tv_usec) * 1000L
                + static_cast<long long>(timeout_ms)  * 1000000L  ;

   // Express absolute timeout time in units required by
   // pthread_cond_timedwait(): seconds + nanoseconds.
   struct timespec timeout ;
   timeout.tv_sec  = static_cast<time_t>(to/1000000000L) ;
   timeout.tv_nsec = static_cast<long>(to % 1000000000L) ;

   // Now, wait until client-supplied test condition becomes true or
   // until timeout expires...
   int timed_out = 0 ;
   while (! pred() && timed_out != ETIMEDOUT)
      timed_out = pthread_cond_timedwait(&m_cond, &m_mutex, &timeout) ;
   return (timed_out != ETIMEDOUT);
}

//----------------------------- SIGNALING -------------------------------

// Wake up any one thread waiting on condition
template<typename F>
void Condition::signal(F pred)
{
   AutoMutex M(m_mutex) ;
   if (pred())
      pthread_cond_signal(&m_cond) ;
}

// Wake up all threads waiting on condition
template<typename F>
void Condition::broadcast(F pred)
{
   AutoMutex M(m_mutex) ;
   if (pred())
      pthread_cond_broadcast(&m_cond) ;
}

//------------------------ PROTECTED EXECUTION --------------------------

// Execute client-supplied function under the protection of the condition
// variable's mutex.
template<typename F>
void Condition::protect(F func)
{
   AutoMutex M(m_mutex) ;
   func() ;
}

//----------------------------- EMPTY API -------------------------------

#else // pthreads missing

template<typename F> void Condition::wait(F){}
template<typename F> bool Condition::wait(F, int){return false ;}
template<typename F> void Condition::signal(F){}
template<typename F> void Condition::broadcast(F){}
template<typename F> void Condition::protect(F){}

#endif

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

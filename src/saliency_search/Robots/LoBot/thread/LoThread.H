/**
   \file  Robots/LoBot/misc/LoThread.H
   \brief A thin wrapper around some parts of the pthreads creation and
   management API.

   This file defines a base class, viz., lobot::Thread, for creating
   threads to handle different sensorimotor operations within the
   Robolocust system. Robolocust modules that need to create new threads
   should use this class rather than the raw pthreads API.

   However, it should be noted that lobot::Thread is only for creating
   threads and some basic thread management related tasks. Thus, users
   have to fall back on raw pthreads for mutexes, condition variables and
   other such paraphernalia.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/thread/LoThread.H $
// $Id: LoThread.H 13521 2010-06-06 14:23:03Z mviswana $
//

#ifndef LOBOT_THREAD_DOT_H
#define LOBOT_THREAD_DOT_H

//------------------------------ HEADERS --------------------------------

// POSIX threads
#ifdef INVT_HAVE_LIBPTHREAD

#include <pthread.h>

#else // fake pthreads API to allow builds to succeed

typedef int pthread_t ;
typedef int pthread_mutex_t ;
typedef int pthread_cond_t ;

#endif

// Standard C++ headers
#include <string>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Thread
   \brief An ABC for POSIX threads.

   lobot::Thread is an abstract base class for creating threads to handle
   different tasks within the Robolocust system. It wraps around some of
   the basic creation and management APIs of the POSIX threads library
   and provides some additional custom support on top of that.

   In terms of the additional custom support alluded to above,
   specifically, this class keeps track of the number of threads
   currently running and allows the main thread to wait for all the other
   threads to die with one convenient API.

   Robolocust modules that need to create new threads should use this
   class rather than the raw pthreads API. However, only some basic
   thread creation and management functions are implemented. This means
   that clients have to either fall back on the raw POSIX API or rely on
   some other wrapper classes for mutexes, condition variables, etc.

   To use this class, simply derive from it and define the run() method.
   Then, instantiating the derived class will take care of the mechanics
   of creating the thread, keeping track of it, etc.

   An important thing to keep in mind when using this class is that the
   threads it creates are detached, i.e., it is not possible for another
   thread to execute pthread_join() on the thread. This ensures that all
   thread resources are immediately cleaned up by the OS when the thread
   exits. Naturally, it also prevents other threads from joining with it.

   The rationale for this design choice is as follows: the Robolocust
   system maintains a central shutdown object that signals all the
   threads that it's time to wind-up. Each Robolocust thread is required
   to monitor the state of this shutdown object in its main loop and take
   appropriate action to gracefully kill itself when the shutdown object
   is in the signaled state. Additionally, when shutdown is signaled, the
   main thread waits for all the other threads to die and then exits the
   application. Thus, there is no need for any thread to wait for another
   by joining with it.

   NOTE: This class does not prevent clients from creating instances on
   the stack (i.e., as local variables). However, doing that would
   generally be a Bad Idea because when the local object goes out of
   scope, its run() method will no longer be available, which, most
   likely, will cause all hell to break loose.

   DEVNOTE: We could use the named constructor idiom to require clients
   to always create Thread objects with the new operator. However, that
   precludes using this class in conjunction with lobot::singleton, which
   is why we don't enforce the do-not-create-on-stack policy.
*/
class Thread {
   // Prevent copy and assignment
   Thread(const Thread&) ;
   Thread& operator=(const Thread&) ;

private:
   /// Every thread has to have an ID and a name.
   //@{
   pthread_t   m_id ;
   std::string m_name ;
   //@}

public:
   /// Return this thread's ID.
   pthread_t id() const {return m_id ;}

   /// Return this thread's name.
   const std::string& name() const {return m_name ;}

   /// Check if the thread is actually running.
   bool running() const {return m_id != 0 ;}

protected:
   /// A protected constructor because clients should not be able to
   /// directly instantiate this class.
   Thread() ;

   /// Creating a new thread object doesn't automatically start the
   /// thread running. Derived classes *must* call this method in the
   /// body of their constructors (usually, towards the end). The reason
   /// for this two-step procedure is due to the fact that the thread
   /// function implemented in this base class calls the run() method in
   /// the derived class. For this to work properly, the derived class
   /// object must be fully constructed with its v-table setup for
   /// action. If we were to start the new thread running in this class's
   /// constructor, it can happen that Thread::thread_func() will crash
   /// when it attempts to invoke the pure virtual run() method.
   ///
   /// Therefore, *all* derived class constructors *must* include a call
   /// to Thread::start(). By the time we get into a constructor's body,
   /// we can be assured that the object is fully constructed and its
   /// v-table properly setup. This ensures that Thread::thread_func()
   /// will be able to use the virtual function mechanism to call into
   /// the derived class's run() method.
   ///
   /// Note, however, that client modules do not see this two-step thread
   /// creation procedure. A client module simply creates an instance of
   /// a Thread subclass without having to fuss around with anything
   /// else. Once the instance is created, the thread will automatically
   /// start running. The onus here is on subclasses to ensure that this
   /// automatic thread creation and starting business actually works
   /// from the point of view of client modules.
   ///
   /// Usually, a derived class's run() method will implement some sort
   /// of "main" loop and when that loop exits, the run() method will
   /// return. However, in some situations, the main loop may be
   /// implemented externally (e.g., glutMainLoop()). Threads that
   /// implement run() methods that don't return normally should pass
   /// false as the second parameter to Thread::start().
   void start(const std::string& name,
              bool thread_func_returns_normally = true) ;

private:
   /// This is the thread function for each thread. In this base class,
   /// it simply serves as the mechanics for getting the pthreads API to
   /// work properly without burdening derived/client classes with the
   /// gory details. Instead, derived classes need only derive from this
   /// one and then define the run() method.
   static void* thread_func(void* arg) ;

   /// Normally, a thread's run() method will return normally, i.e., back
   /// to the thread_func. This allows thread_func() to keep track of the
   /// number of threads currently running.
   ///
   /// However, in some cases, the run() method may not return to the
   /// thread_func. For example, a thread that calls glutMainLoop() will
   /// have to terminate itself using pthread_exit() rather than by
   /// falling out a of a loop because GLUT controls the thread's main
   /// loop and provides no direct means of exiting that loop.
   ///
   /// In such cases, thread_func_no_return() is used as the thread_func.
   /// The difference between this thread_func and the usual one is that
   /// it does not increment/decrement the thread count prior to and
   /// after the run() method call (because the run() method will never
   /// return control to it).
   ///
   /// What this means is that there may be more or less threads actually
   /// running than the count maintained in the Thread class. This is an
   /// unfortunate consequence of the run-method-not-returning-normally
   /// problem. However, it is necessary to have this mechanism in place
   /// to ensure that the Robolocust controller exits without any hassles
   /// when such threads are running. If we did not have this workaround
   /// in place, the main thread would wait for all threads to die; but
   /// thread count would never go down to zero because the run method of
   /// a thread whose main loop is implemented "externally" would never
   /// return, thus, causing the main thread to wait forever.
   ///
   /// Moral of the story: limit the number of threads that don't return
   /// normally from their run() methods. And, in such threads, be sure
   /// to pass false to the second parameter of the start() method.
   static void* thread_func_no_return(void* arg) ;

public:
   /// This method implements the thread's custom processing. It must be
   /// defined by every subclass.
   virtual void run() = 0 ;

private:
   /// This class keeps track of the number of extant threads with a
   /// simple shared counter.
   static int m_count ;

   /// Since the above counter can be accessed by multiple threads, we
   /// need a mutex to ensure serialized updates to the count.
   static pthread_mutex_t m_count_mutex ;

public:
   /// Return the current number of extant threads.
   static int count() ;

private:
   /// This class provides an API that allows the main thread to wait for
   /// all the other extant threads to die. This condition is signaled
   /// using a condition variable that is flagged when the count of
   /// extant threads reaches zero.
   static pthread_cond_t m_count_zero_condition ;

public:
   /// Wait for all the extant threads to die.
   ///
   /// NOTE: This method is meant to be called *only* by the main thread.
   static void wait_all() ;

   /// Clean-up.
   virtual ~Thread() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

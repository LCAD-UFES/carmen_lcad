/**
   \file  Robots/LoBot/misc/LoShutdown.H
   \brief An object to signal and coordinate an orderly application
   shutdown.

   This file defines a class that implements a thread that simply waits
   for a termination signal (SIGINT, SIGHUP, etc.). It provides an API
   for the other Robolocust modules/threads to check if application
   shutdown is in progress and take appropriate clean-up action.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/thread/LoShutdown.H $
// $Id: LoShutdown.H 13521 2010-06-06 14:23:03Z mviswana $
//

#ifndef LOBOT_SHUTDOWN_DOT_H
#define LOBOT_SHUTDOWN_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/thread/LoThread.H"
#include "Robots/LoBot/misc/singleton.hh"

// POSIX threads
#ifdef INVT_HAVE_LIBPTHREAD

#include <pthread.h>

#else // fake pthreads API to allow builds to succeed

typedef int pthread_rwlock_t ;

#endif

// Standard C headers
#include <signal.h>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Shutdown
   \brief A thread that waits for a shutdown signal and allows other
   modules/threads to query whether shutdown is in progress or not.

   This class implements a signal handling thread that simply waits for
   one of the SIGINT, SIGHUP, etc. signals to be sent to the Robolocust
   application. Once such a signal is delivered, it updates its internal
   state to reflect that the application is to be shutdown. All other
   Robolocust threads are required to monitor this Shutdown object in
   their respective main loops and break out of these loops when shutdown
   has been signaled.
*/
class Shutdown : public singleton<Shutdown>, private Thread {
   // Prevent copy and assignment
   Shutdown(const Shutdown&) ;
   Shutdown& operator=(const Shutdown&) ;

   // Boilerplate code to make the generic singleton design pattern work
   friend class singleton<Shutdown> ;

   /// The shutdown object listens for the signals specified by this
   /// mask.
   static sigset_t m_signals_mask ;

   /// This flag indicates whether or not the shutdown thread has begun
   /// listening for app termination signals. It is used in conjunction
   /// with the current thread count to try an enforce the restriction
   /// that the start_listening() method be called only once and that too
   /// only from the application's main thread.
   static bool m_listening ;

   /// The shutdown object uses this flag to indicate whether or not it
   /// has received a shutdown signal.
   bool m_shutdown ;

   /// Since the above flag can be accessed by multiple threads, we need
   /// to synchronize accesses to it. Since the Shutdown object's signal
   /// handling thread is the only one that will ever write to the above
   /// variable and since all the other threads in the Robolocust system
   /// will only read it, we can use a reader-writer lock rather than a
   /// mutex.
   pthread_rwlock_t m_shutdown_lock ;

   /// A private constructor because this class is a singleton.
   Shutdown() ;

   /// This method implements the shutdown thread's custom processing,
   /// which takes care of the necessary signal handling logic.
   void run() ;

public:
   /// A convenience method for setting up the signals mask and starting
   /// the shutdown signals monitoring thread.
   ///
   /// NOTE: This method should be called sometime early on in the
   /// application's lifetime. Moreover, it *must* be called *only* by
   /// the application's main thread.
   static void start_listening() ;

   /// Let clients know whether a shutdown signal has been received or
   /// not.
   static bool signaled() ;

   /// Allow a client to explicitly post a SIGTERM to the shutdown
   /// thread.
   static void signal() ;

   /// Clean-up.
   ~Shutdown() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

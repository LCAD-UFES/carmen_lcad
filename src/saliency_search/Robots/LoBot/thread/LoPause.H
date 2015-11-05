/**
   \file  Robots/LoBot/misc/LoPause.H
   \brief An object to signal application pauses.

   This file defines a class that implements a pause signal for the
   Robolocust application. When this signal is set, all threads should
   cease their normal processing, i.e., stop reading sensors, issuing
   motor commands, updating the UI, etc. until the signal is cleared.

   Basically, this class just holds a Boolean flag protected with a
   read/write lock.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/thread/LoPause.H $
// $Id: LoPause.H 13567 2010-06-13 15:58:59Z mviswana $
//

#ifndef LOBOT_PAUSE_DOT_H
#define LOBOT_PAUSE_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/misc/singleton.hh"

// POSIX threads
#ifdef INVT_HAVE_LIBPTHREAD

#include <pthread.h>

#else // fake pthreads API to allow builds to succeed

typedef int pthread_rwlock_t ;

#endif

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Pause
   \brief A Boolean flag to indicate whether threads should work as usual
   or wait until the application is unpaused.

   This class holds a Boolean flag indicating whether the different
   Robolocust threads should suspend their normal operations or proceed
   normally. When this flag is set, all threads are required to stop
   doing their normal tasks such as reading sensors, issuing motor
   commands, etc. and idle until the pause flag is cleared.

   Basically, this class serves to encapsulate an application-wide bool
   variable that is to be used in conjunction with a read/write lock.
*/
class Pause : public singleton<Pause> {
   // Prevent copy and assignment
   Pause(const Pause&) ;
   Pause& operator=(const Pause&) ;

   // Boilerplate code to make the generic singleton design pattern work
   friend class singleton<Pause> ;

   /// The pause object's raison d'etre: indicate to all Robolocust
   /// threads whether they should quietly idle or carry on with their
   /// usual tasks.
   bool m_flag ;

   /// Since the above flag can be accessed by multiple threads, we need
   /// to synchronize accesses to it. More threads will read this
   /// variable than write to it. Therefore, a reader-writer lock (rather
   /// than a mutex) is an appropriate synchronization mechanism.
   pthread_rwlock_t m_lock ;

   /// A private constructor because this class is a singleton.
   Pause() ;

public:
   /// Allow a thread to signal all other threads that they should pause,
   /// i.e., suspend normal operations and idle.
   static void set() ;

   /// Allow a thread to signal all other threads that they should stop
   /// idling and resume normal operations.
   static void clear() ;

   /// Allow a thread to toggle the application's current pause state.
   static void toggle() ;

   /// This function returns true if the application is currently paused.
   static bool is_set() ;

   /// This function returns true if the application is currently not
   /// paused.
   static bool is_clear() {return ! is_set() ;}

   /// Clean-up.
   ~Pause() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/**
   \file  Robots/LoBot/misc/LoMutex.H
   \brief An object-oriented wrapper around pthreads mutexes.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/thread/LoMutex.H $
// $Id: LoMutex.H 13521 2010-06-06 14:23:03Z mviswana $
//

#ifndef LOBOT_MUTEX_DOT_H
#define LOBOT_MUTEX_DOT_H

//------------------------------ HEADERS --------------------------------

// POSIX threads
#ifdef INVT_HAVE_LIBPTHREAD

#include <pthread.h>

#else // fake pthreads API to allow builds to succeed

typedef int pthread_mutex_t ;

#endif

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Mutex
   \brief A simple encapsulation of pthread mutexes.
*/
class Mutex {
   // Prevent copy and assignment
   Mutex(const Mutex&) ;
   Mutex& operator=(const Mutex&) ;

   /// The underlying pthread mutex this class wraps around.
   pthread_mutex_t m_mutex ;

public:
   /// Default constructor: sets up the pthread mutex object.
   Mutex() ;

   /// Mutex acquisition and release.
   //@{
   void acquire() ;
   void release() ;

   void lock()   {acquire() ;}
   void unlock() {release() ;}
   //@}

   /// Clean-up.
   ~Mutex() ;
} ;

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::AutoMutex
   \brief Mutexes created on the stack that acquire an "underlying" mutex
   on initialization and release it when the object goes out of scope.
*/
class AutoMutex {
   // Prevent copy and assignment
   AutoMutex(const AutoMutex&) ;
   AutoMutex& operator=(const AutoMutex&) ;

   /// The underlying mutex this object is "tied" to.
   Mutex& m_mutex ;

public:
   /// Constructor: will acquire the underlying mutex object.
   AutoMutex(Mutex&) ;

   /// Constructor for const Mutex objects (e.g., when called from const
   /// member functions in client classes).
   AutoMutex(const Mutex&) ;

   /// Clean-up: will release the underlying mutex object.
   ~AutoMutex() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

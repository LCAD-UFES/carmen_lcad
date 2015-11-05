/**
   \file  Robots/LoBot/misc/LoRWLock.H
   \brief An object-oriented wrapper around pthreads read/write locks.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/thread/LoRWLock.H $
// $Id: LoRWLock.H 13613 2010-06-23 23:29:23Z mviswana $
//

#ifndef LOBOT_RWLOCK_DOT_H
#define LOBOT_RWLOCK_DOT_H

//------------------------------ HEADERS --------------------------------

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
   \class lobot::RWLock
   \brief A simple encapsulation of pthread read/write locks.
*/
class RWLock {
   // Prevent copy and assignment
   RWLock(const RWLock&) ;
   RWLock& operator=(const RWLock&) ;

   /// The underlying pthread lock this class wraps around.
   pthread_rwlock_t m_lock ;

public:
   /// Default constructor: sets up the pthread read/write lock object.
   RWLock() ;

   /// RWLock acquisition and release.
   //@{
   void begin_read() ;
   void end_read() ;
   void begin_write() ;
   void end_write() ;
   //@}

   /// Clean-up.
   ~RWLock() ;
} ;

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::AutoReadLock
   \brief Read locks created on the stack that acquire an "underlying"
   read/write lock for reading on initialization and release it when the
   object goes out of scope.
*/
class AutoReadLock {
   // Prevent copy and assignment
   AutoReadLock(const AutoReadLock&) ;
   AutoReadLock& operator=(const AutoReadLock&) ;

   /// The underlying lock this object is "tied" to.
   RWLock& m_lock ;

public:
   /// Constructor: will acquire the underlying lock object for reading.
   AutoReadLock(RWLock&) ;

   /// Constructor for const RWLock objects (e.g., when called from const
   /// member functions in client classes).
   AutoReadLock(const RWLock&) ;

   /// Clean-up: will release the underlying lock object.
   ~AutoReadLock() ;
} ;

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::AutoWriteLock
   \brief Write locks created on the stack that acquire an "underlying"
   read/write lock for writing on initialization and release it when the
   object goes out of scope.
*/
class AutoWriteLock {
   // Prevent copy and assignment
   AutoWriteLock(const AutoWriteLock&) ;
   AutoWriteLock& operator=(const AutoWriteLock&) ;

   /// The underlying lock this object is "tied" to.
   RWLock& m_lock ;

public:
   /// Constructor: will acquire the underlying lock object for writing.
   AutoWriteLock(RWLock&) ;

   /// Constructor for const RWLock objects (e.g., when called from const
   /// member functions in client classes).
   AutoWriteLock(const RWLock&) ;

   /// Clean-up: will release the underlying lock object.
   ~AutoWriteLock() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

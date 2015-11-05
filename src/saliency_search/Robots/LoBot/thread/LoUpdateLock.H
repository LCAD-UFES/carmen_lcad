/**
   \file  Robots/LoBot/misc/LoUpdateLock.H
   \brief An object to help coordinate read-write accesses to various
   sensor and other objects within the Robolocust system.

   This file defines a class that implements a reader-writer lock meant
   to be shared by the main thread and the different behaviours that make
   up the Robolocust controller. The main thread updates the video I/O
   objects, the laser range finder, the locust LGMD models, integration
   algorithm, etc. The behaviours simply retrieve the latest values from
   these objects.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/thread/LoUpdateLock.H $
// $Id: LoUpdateLock.H 13521 2010-06-06 14:23:03Z mviswana $
//

#ifndef LOBOT_UPDATE_LOCK_DOT_H
#define LOBOT_UPDATE_LOCK_DOT_H

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
   \class lobot::UpdateLock
   \brief An object to help coordinate read-write accesses to various
   shared objects within the Robolocust controller.

   This class implements a reader-writer lock meant to be shared by the
   Robolocust controller's main thread and each of its behaviours. The
   main thread is responsible for updating the different sensors (laser
   range finder, cameras, etc.) and other objects (LGMD models,
   integration algorithm, etc.). The individual behaviours simply
   retrieve the latest values from these objects.

   Each thread must take care of using this lock appropriately when
   required. That is, the objects themselves do not perform any locking
   (in other words, they are not thread-safe).
*/
class UpdateLock : public singleton<UpdateLock> {
   // Prevent copy and assignment
   UpdateLock(const UpdateLock&) ;
   UpdateLock& operator=(const UpdateLock&) ;

   // Boilerplate code to make the generic singleton design pattern work
   friend class singleton<UpdateLock> ;

   /// The reader-writer lock used to synchronize access.
   pthread_rwlock_t m_update_lock ;

   /// A private constructor because this class is a singleton.
   UpdateLock() ;

public:
   /// The locking functions.
   //@{
   void read_lock() ;
   void write_lock() ;
   void unlock() ;
   //@}

   /// Convenience functions.
   //@{
   static void begin_read()  {instance().read_lock() ;}
   static void end_read()    {instance().unlock() ;}
   static void begin_write() {instance().write_lock() ;}
   static void end_write()   {instance().unlock() ;}
   //@}

   /// Clean-up.
   ~UpdateLock() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

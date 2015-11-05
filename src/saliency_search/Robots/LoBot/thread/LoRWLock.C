/**
   \file  Robots/LoBot/misc/LoRWLock.C
   \brief This file defines the non-inline member functions of the
   lobot::RWLock, lobot::AutoReadLock and lobot::AutoWriteLock classes.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/thread/LoRWLock.C $
// $Id: LoRWLock.C 13613 2010-06-23 23:29:23Z mviswana $
//

//---------------------- ALTERNATIVE DEFINITION -------------------------

// In case pthreads is missing
#ifndef INVT_HAVE_LIBPTHREAD

#include "Robots/LoBot/thread/LoRWLock.H"
#include "Robots/LoBot/misc/LoExcept.H"

namespace lobot {

// Constructors
RWLock::RWLock()
   : m_lock(0)
{
   throw missing_libs(MISSING_PTHREAD) ;
}

AutoReadLock::AutoReadLock(RWLock& L)
   : m_lock(L)
{
   throw missing_libs(MISSING_PTHREAD) ;
}

AutoReadLock::AutoReadLock(const RWLock& L)
   : m_lock(const_cast<RWLock&>(L))
{
   throw missing_libs(MISSING_PTHREAD) ;
}

AutoWriteLock::AutoWriteLock(RWLock& L)
   : m_lock(L)
{
   throw missing_libs(MISSING_PTHREAD) ;
}

AutoWriteLock::AutoWriteLock(const RWLock& L)
   : m_lock(const_cast<RWLock&>(L))
{
   throw missing_libs(MISSING_PTHREAD) ;
}

// Empty API
void RWLock::begin_read() {}
void RWLock::end_read()   {}
void RWLock::begin_write(){}
void RWLock::end_write()  {}

// Destructors
RWLock::~RWLock(){}
AutoReadLock::~AutoReadLock(){}
AutoWriteLock::~AutoWriteLock(){}

} // end of namespace encapsulating above empty definition

#else // pthreads available ==> the real McCoy

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/thread/LoRWLock.H"
#include "Robots/LoBot/misc/LoExcept.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

RWLock::RWLock()
{
   if (pthread_rwlock_init(& m_lock, 0) != 0)
      throw thread_error(RWLOCK_INIT_ERROR) ;
}

AutoReadLock::AutoReadLock(RWLock& L)
   : m_lock(L)
{
   m_lock.begin_read() ;
}

AutoReadLock::AutoReadLock(const RWLock& L)
   : m_lock(const_cast<RWLock&>(L))
{
   m_lock.begin_read() ;
}

AutoWriteLock::AutoWriteLock(RWLock& L)
   : m_lock(L)
{
   m_lock.begin_write() ;
}

AutoWriteLock::AutoWriteLock(const RWLock& L)
   : m_lock(const_cast<RWLock&>(L))
{
   m_lock.begin_write() ;
}

//-------------------- READ/WRITE LOCK OPERATIONS -----------------------

void RWLock::begin_read()
{
   if (pthread_rwlock_rdlock(& m_lock) != 0)
      throw thread_error(RWLOCK_RDLOCK_FAILED) ;
}

void RWLock::end_read()
{
   pthread_rwlock_unlock(& m_lock) ;
}

void RWLock::begin_write()
{
   if (pthread_rwlock_wrlock(& m_lock) != 0)
      throw thread_error(RWLOCK_WRLOCK_FAILED) ;
}

void RWLock::end_write()
{
   pthread_rwlock_unlock(& m_lock) ;
}

//----------------------------- CLEAN-UP --------------------------------

RWLock::~RWLock()
{
   pthread_rwlock_destroy(& m_lock) ;
}

AutoReadLock::~AutoReadLock()
{
   m_lock.end_read() ;
}

AutoWriteLock::~AutoWriteLock()
{
   m_lock.end_write() ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif // INVT_HAVE_PTHREAD

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

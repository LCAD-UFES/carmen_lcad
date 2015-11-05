/**
   \file  Robots/LoBot/misc/LoPause.C
   \brief This file defines the non-inline member functions of the
   lobot::Pause class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/thread/LoPause.C $
// $Id: LoPause.C 13567 2010-06-13 15:58:59Z mviswana $
//

//---------------------- ALTERNATIVE DEFINITION -------------------------

// In case pthreads is missing
#ifndef INVT_HAVE_LIBPTHREAD

#include "Robots/LoBot/thread/LoPause.H"
#include "Robots/LoBot/misc/LoExcept.H"

namespace lobot {

// Constructor
Pause::Pause()
   : m_flag(false), m_lock(0)
{
   throw missing_libs(MISSING_PTHREAD) ;
}

// Empty API
void Pause::set(){}
void Pause::clear(){}
void Pause::toggle(){}
bool Pause::is_set(){return false ;}

// Destructor
Pause::~Pause(){}

} // end of namespace encapsulating above empty definition

#else // pthreads available ==> the real McCoy

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/thread/LoPause.H"
#include "Robots/LoBot/misc/LoExcept.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

Pause::Pause()
   : m_flag(false)
{
   if (pthread_rwlock_init(& m_lock, 0) != 0)
      throw thread_error(RWLOCK_INIT_ERROR) ;
}

//------------------------ SETTING PAUSE STATE --------------------------

void Pause::set()
{
   Pause& P = instance() ;
   if (pthread_rwlock_rdlock(& P.m_lock) == 0) {
      P.m_flag = true ;
      pthread_rwlock_unlock(& P.m_lock) ;
   }
}

void Pause::clear()
{
   Pause& P = instance() ;
   if (pthread_rwlock_rdlock(& P.m_lock) == 0) {
      P.m_flag = false ;
      pthread_rwlock_unlock(& P.m_lock) ;
   }
}

void Pause::toggle()
{
   Pause& P = instance() ;
   if (pthread_rwlock_rdlock(& P.m_lock) == 0) {
      P.m_flag = ! P.m_flag ;
      pthread_rwlock_unlock(& P.m_lock) ;
   }
}

//------------------------ PAUSE SIGNAL STATUS --------------------------

bool Pause::is_set()
{
   bool flag = false ;
   Pause& P = instance() ;
   if (pthread_rwlock_rdlock(& P.m_lock) == 0) {
      flag = P.m_flag ;
      pthread_rwlock_unlock(& P.m_lock) ;
   }
   return flag ;
}

//----------------------------- CLEAN-UP --------------------------------

Pause::~Pause()
{
   pthread_rwlock_destroy(& m_lock) ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif // INVT_HAVE_PTHREAD

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

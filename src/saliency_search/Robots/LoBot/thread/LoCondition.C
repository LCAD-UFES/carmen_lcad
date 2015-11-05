/**
   \file  Robots/LoBot/misc/LoCondition.C
   \brief This file defines the non-inline member functions of the
   lobot::Condition class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/thread/LoCondition.C $
// $Id: LoCondition.C 13523 2010-06-07 00:22:03Z mviswana $
//

//---------------------- ALTERNATIVE DEFINITION -------------------------

// In case pthreads is missing
#ifndef INVT_HAVE_LIBPTHREAD

#include "Robots/LoBot/thread/LoCondition.H"
#include "Robots/LoBot/misc/LoExcept.H"

namespace lobot {

// Constructors
Condition::Condition()
   : m_mutex(0), m_cond(0)
{
   throw missing_libs(MISSING_PTHREAD) ;
}

Condition::AutoMutex::AutoMutex(pthread_mutex_t&)
   : mutex(0)
{}

// Destructors
Condition::~Condition(){}
Condition::AutoMutex::~AutoMutex(){}

} // end of namespace encapsulating above empty definition

#else // pthreads available ==> the real McCoy

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/thread/LoCondition.H"
#include "Robots/LoBot/misc/LoExcept.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

Condition::Condition()
{
   if (pthread_mutex_init(&m_mutex, 0) != 0)
      throw thread_error(MUTEX_INIT_ERROR) ;
   if (pthread_cond_init(&m_cond, 0) != 0) {
      pthread_mutex_destroy(&m_mutex) ;
      throw thread_error(COND_INIT_ERROR) ;
   }
}

Condition::AutoMutex::AutoMutex(pthread_mutex_t& m)
   : mutex(m)
{
   pthread_mutex_lock(&mutex) ;
}

//----------------------------- CLEAN-UP --------------------------------

Condition::~Condition()
{
   pthread_cond_destroy (&m_cond) ;
   pthread_mutex_destroy(&m_mutex);
}

Condition::AutoMutex::~AutoMutex()
{
   pthread_mutex_unlock(&mutex) ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif // INVT_HAVE_PTHREAD

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

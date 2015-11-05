/**
   \file  Robots/LoBot/misc/LoThread.C
   \brief This file defines the static data members and non-inline member
   functions of the lobot::Thread class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/thread/LoThread.C $
// $Id: LoThread.C 13521 2010-06-06 14:23:03Z mviswana $
//

//---------------------- ALTERNATIVE DEFINITION -------------------------

// In case pthreads is missing
#ifndef INVT_HAVE_LIBPTHREAD

#include "Robots/LoBot/thread/LoThread.H"
#include "Robots/LoBot/misc/LoExcept.H"

namespace lobot {

// Static data members
int             Thread::m_count ;
pthread_mutex_t Thread::m_count_mutex ;
pthread_cond_t  Thread::m_count_zero_condition ;

// Constructor
Thread::Thread()
   : m_id(0)
{
   throw missing_libs(MISSING_PTHREAD) ;
}

// Empty API
void  Thread::start(const std::string&, bool){}
void* Thread::thread_func(void*)           {return 0 ;}
void* Thread::thread_func_no_return(void*) {return 0 ;}
int   Thread::count() {return 0 ;}
void  Thread::wait_all(){}

// Destructor
Thread::~Thread(){}

} // end of namespace encapsulating above empty definition

#else // pthreads available ==> the real McCoy

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/thread/LoThread.H"
#include "Robots/LoBot/misc/LoExcept.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------ STATIC DATA MEMBERS --------------------------

// A counter to keep track of the number of extant threads
int Thread::m_count ;

// A mutex to ensure proper access to above counter
pthread_mutex_t Thread::m_count_mutex = PTHREAD_MUTEX_INITIALIZER ;

// Condition variable for signaling main thread when all others are done
pthread_cond_t Thread::m_count_zero_condition = PTHREAD_COND_INITIALIZER ;

//-------------------------- INITIALIZATION -----------------------------

Thread::Thread()
   : m_id(0)
{}

void Thread::start(const std::string& name, bool thread_func_returns_normally)
{
   m_name = name ;

   pthread_attr_t attr ;
   pthread_attr_init(& attr) ;
   pthread_attr_setdetachstate(& attr, PTHREAD_CREATE_DETACHED) ;

   int create_ok = 0 ;
   if (thread_func_returns_normally)
      create_ok = pthread_create(& m_id, & attr,
                                 thread_func,
                                 reinterpret_cast<void*>(this)) ;
   else
      create_ok = pthread_create(& m_id, & attr,
                                 thread_func_no_return,
                                 reinterpret_cast<void*>(this)) ;
   if (create_ok != 0)
      throw thread_error(THREAD_CREATION_FAILURE) ;
}

//------------------------ THE THREAD FUNCTION --------------------------

void* Thread::thread_func(void* arg)
{
   pthread_mutex_lock(& m_count_mutex) ;
      ++m_count ;
   pthread_mutex_unlock(& m_count_mutex) ;

   Thread* derived_class_instance = reinterpret_cast<Thread*>(arg) ;
   derived_class_instance->run() ;

   pthread_mutex_lock(& m_count_mutex) ;
      --m_count ;
      if (m_count <= 0)
         pthread_cond_broadcast(& m_count_zero_condition) ;
   pthread_mutex_unlock(& m_count_mutex) ;

   derived_class_instance->m_id = 0 ;
   return 0 ;
}

// Threads that don't return normally from their run() methods should not
// be counted. Otherwise, the wait_all() method won't work properly
// because the thread count will never go down to zero.
void* Thread::thread_func_no_return(void* arg)
{
   Thread* derived_class_instance = reinterpret_cast<Thread*>(arg) ;
   derived_class_instance->run() ;
   //derived_class_instance->m_id = 0 ;
   return 0 ;
}

//------------------------- THREAD MANAGEMENT ---------------------------

int Thread::count()
{
   int n = -1 ;
   pthread_mutex_lock(& m_count_mutex) ;
      n = m_count ;
   pthread_mutex_unlock(& m_count_mutex) ;
   return n ;
}

void Thread::wait_all()
{
   pthread_mutex_lock(& m_count_mutex) ;
   while (m_count > 0)
      pthread_cond_wait(& m_count_zero_condition, & m_count_mutex) ;
   pthread_mutex_unlock(& m_count_mutex) ;
}

//----------------------------- CLEAN-UP --------------------------------

Thread::~Thread(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif // INVT_HAVE_PTHREAD

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

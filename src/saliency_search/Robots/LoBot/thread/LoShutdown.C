/**
   \file  Robots/LoBot/misc/LoShutdown.C
   \brief This file defines the static data members and non-inline member
   functions of the lobot::Shutdown class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/thread/LoShutdown.C $
// $Id: LoShutdown.C 13521 2010-06-06 14:23:03Z mviswana $
//

//---------------------- ALTERNATIVE DEFINITION -------------------------

// In case pthreads is missing
#ifndef INVT_HAVE_LIBPTHREAD

#include "Robots/LoBot/thread/LoShutdown.H"
#include "Robots/LoBot/misc/LoExcept.H"

namespace lobot {

// Static data members
sigset_t Shutdown::m_signals_mask ;
bool     Shutdown::m_listening ;

// Constructor
Shutdown::Shutdown()
   : m_shutdown(false), m_shutdown_lock(0)
{
   throw missing_libs(MISSING_PTHREAD) ;
}

// Empty API
void Shutdown::run(){}
void Shutdown::start_listening(){}
bool Shutdown::signaled() {return false ;}
void Shutdown::signal(){}

// Destructor
Shutdown::~Shutdown(){}

} // end of namespace encapsulating above empty definition

#else // pthreads available ==> the real McCoy

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/thread/LoShutdown.H"
#include "Robots/LoBot/misc/LoExcept.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------ STATIC DATA MEMBERS --------------------------

// The signals to listen for (SIGINT, SIGHUP, SIGQUIT, SIGTERM, etc.)
sigset_t Shutdown::m_signals_mask ;

// Flag indicating whether listening for various termination signals has
// begun or not.
bool Shutdown::m_listening ;

//-------------------------- INITIALIZATION -----------------------------

void Shutdown::start_listening()
{
   if (Thread::count() == 0 && ! m_listening) {
      sigemptyset(& m_signals_mask) ;
      sigaddset(& m_signals_mask, SIGINT) ;
      sigaddset(& m_signals_mask, SIGHUP) ;
      sigaddset(& m_signals_mask, SIGQUIT) ;
      sigaddset(& m_signals_mask, SIGTERM) ;
      if (pthread_sigmask(SIG_BLOCK, & m_signals_mask, 0) != 0)
         throw misc_error(SIGNALS_MASK_SETUP_FAILURE) ;
      instance() ; // force creation of shutdown thread
      m_listening = true ;
   }
}

Shutdown::Shutdown()
   : m_shutdown(false)
{
   if (pthread_rwlock_init(& m_shutdown_lock, 0) != 0)
      throw thread_error(RWLOCK_INIT_ERROR) ;
   start("shutdown") ;
}

//------------------------ THE THREAD FUNCTION --------------------------

void Shutdown::run()
{
   for(;;)
   {
      int signum = 0 ;
      if (sigwait(& m_signals_mask, & signum) != 0)
         return ;
      switch (signum)
      {
         case SIGINT:
         case SIGHUP:
         case SIGQUIT:
         case SIGTERM:
            pthread_rwlock_wrlock(& m_shutdown_lock) ;
               m_shutdown = true ;
            pthread_rwlock_unlock(& m_shutdown_lock) ;
            return ;
         default:
            break ;
      }
   }
}

//---------------------- SHUTDOWN SIGNAL STATUS -------------------------

bool Shutdown::signaled()
{
   if (! instance().running())
      return true ; // dangerous if a thread calls before this one is up!

   bool shutdown_flag = false ;
   if (pthread_rwlock_rdlock(& instance().m_shutdown_lock) == 0) {
      shutdown_flag = instance().m_shutdown ;
      pthread_rwlock_unlock(& instance().m_shutdown_lock) ;
   }
   return shutdown_flag ;
}

void Shutdown::signal()
{
   if (instance().running())
      pthread_kill(instance().id(), SIGTERM) ;
}

//----------------------------- CLEAN-UP --------------------------------

Shutdown::~Shutdown()
{
   pthread_rwlock_destroy(& m_shutdown_lock) ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif // INVT_HAVE_PTHREAD

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

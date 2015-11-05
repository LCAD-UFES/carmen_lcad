/**
   \file  Robots/LoBot/control/LoArbiter.C
   \brief This file defines the non-inline member functions of the
   lobot::Arbiter class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoArbiter.C $
// $Id: LoArbiter.C 13567 2010-06-13 15:58:59Z mviswana $
//

//---------------------- ALTERNATIVE DEFINITION -------------------------

// In case pthreads is missing
#ifndef INVT_HAVE_LIBPTHREAD

#include "Robots/LoBot/control/LoArbiter.H"
#include "Robots/LoBot/misc/LoExcept.H"

namespace lobot {

// Constructor
Arbiter::Arbiter(int)
   : m_update_delay(0),
     m_freeze_priority(-1), m_freeze_mutex(0),
     m_votes_mutex(0), m_viz_mutex(0)
{
   throw missing_libs(MISSING_PTHREAD) ;
}

// Empty API
void Arbiter::run(){}
void Arbiter::pre_run(){}
void Arbiter::post_run(){}

void  Arbiter::init_priorities(){}
float Arbiter::priority(const std::string&) {return 0 ;}

void Arbiter::render_cb(unsigned long){}
void Arbiter::render(){}

Arbiter::vote_data::vote_data(const std::string&, long long, VoteBase*){}
Arbiter::vote_data::~vote_data(){}
void Arbiter::vote(const std::string&, void*){}

void Arbiter::  freeze (const std::string&){}
void Arbiter::unfreeze (const std::string&){}
bool Arbiter::is_frozen(const std::string&) const {return false ;}

// Destructor
Arbiter::~Arbiter(){}

} // end of namespace encapsulating above empty definition

#else // pthreads available ==> the real McCoy

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoArbiter.H"
#include "Robots/LoBot/control/LoBehavior.H"

#include "Robots/LoBot/LoApp.H"
#include "Robots/LoBot/ui/LoLaserViz.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"

#include "Robots/LoBot/thread/LoShutdown.H"
#include "Robots/LoBot/thread/LoPause.H"

#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/util/LoSTL.H"
#include "Robots/LoBot/util/LoTime.H"

// INVT utilities
#include "Util/log.H"

// Unix headers
#include <unistd.h>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

Arbiter::
Arbiter(int update_delay, const std::string& name, const Drawable::Geometry& g)
   : Drawable(name, g),
     m_update_delay(clamp(update_delay, 1, 900000) * 1000),
     m_freeze_priority(-1)
{
   if (pthread_mutex_init(& m_freeze_mutex, 0) != 0)
      throw thread_error(MUTEX_INIT_ERROR) ;
   if (pthread_mutex_init(& m_votes_mutex, 0) != 0) {
      pthread_mutex_destroy(& m_freeze_mutex) ;
      throw thread_error(MUTEX_INIT_ERROR) ;
   }
}

//------------------------ THE THREAD FUNCTION --------------------------

void Arbiter::pre_run(){}

void Arbiter::run()
{
   try
   {
      // Before initializing behaviour priorities and checking for
      // existence of motor subsystem, ensure that the application object
      // has been fully loaded.
      App::wait_for_init() ;

      init_priorities() ;
      if (! App::robot()) {
         LERROR("arbiter error: robot sensorimotor subsystem unavailable") ;
         return ;
      }

      // Main loop
      pre_run() ;
      while (! Shutdown::signaled())
      {
         if (Pause::is_clear())
         {
            pthread_mutex_lock(& m_votes_mutex) ;
            if (! m_votes.empty()) {
               motor_cmd(m_votes, App::robot()) ;
               purge_container(m_votes) ;
               m_votes.clear() ;
            }
            pthread_mutex_unlock(& m_votes_mutex) ;
         }
         usleep(m_update_delay) ;
      }
      post_run() ;
   }
   catch (uhoh& e)
   {
      LERROR("arbiter error: %s", e.what()) ;
      purge_container(m_votes) ;
      m_votes.clear() ;
      pthread_mutex_unlock(& m_votes_mutex) ;
   }
}

void Arbiter::post_run(){}

//---------------------- BEHAVIOUR PRIORITY MAP -------------------------

void Arbiter::init_priorities()
{
   float sum = 0 ;
   App::Behaviours::const_iterator it = App::behaviours().begin() ;
   for (; it != App::behaviours().end(); ++it)
   {
      const std::string& i = (*it)->name ;
      float p = get_configured_priority(i) ;
      m_priorities[i] = p ;
      sum += p ;
   }

   // Normalize the user-assigned priorities
   if (sum > 0)
      for (it = App::behaviours().begin(); it != App::behaviours().end(); ++it)
         m_priorities[(*it)->name] /= sum ;
}

float Arbiter::priority(const std::string& behaviour) const
{
   PriorityMap::const_iterator it = m_priorities.find(behaviour) ;
   if (it == m_priorities.end())
      return 0 ;
   return it->second ;
}

//--------------------- ARBITER FREEZING SUPPORT ------------------------

void Arbiter::freeze(const std::string& name)
{
   float p = priority(name) ;
   pthread_mutex_lock(& m_freeze_mutex) ;
      if (p > m_freeze_priority) {
         m_freezer = name ;
         m_freeze_priority = p ;
      }
   pthread_mutex_unlock(& m_freeze_mutex) ;
}

void Arbiter::unfreeze(const std::string& name)
{
   pthread_mutex_lock(& m_freeze_mutex) ;
      if (m_freezer == name) {
         m_freezer.clear() ;
         m_freeze_priority = -1 ;
      }
   pthread_mutex_unlock(& m_freeze_mutex) ;
}

bool Arbiter::is_frozen(const std::string& name) const
{
   bool frozen = false ;
   pthread_mutex_lock(& m_freeze_mutex) ;
      if (name == m_freezer)
         frozen = true ;
   pthread_mutex_unlock(& m_freeze_mutex) ;
   return frozen ;
}

//---------------------- ARBITER VOTING SUPPORT -------------------------

Arbiter::VoteBase::~VoteBase(){}

Arbiter::vote_data::
vote_data(const std::string& n, long long t, VoteBase* v)
   : behavior_name(n), vote_time(t), vote(v)
{}

Arbiter::vote_data::~vote_data()
{
   delete vote ;
}

void Arbiter::vote(const std::string& name, VoteBase* vote)
{
   if (! running())
      throw arbiter_error(ARBITER_NOT_RUNNING) ;

   pthread_mutex_lock(& m_freeze_mutex) ;
      float freeze_priority = m_freeze_priority ;
   pthread_mutex_unlock(& m_freeze_mutex) ;
   if (priority(name) < freeze_priority)
      return ;

   pthread_mutex_lock(& m_votes_mutex) ;
      m_votes.push_back(new vote_data(name, current_time(), vote)) ;
   pthread_mutex_unlock(& m_votes_mutex) ;
}

//----------------------------- CLEAN-UP --------------------------------

Arbiter::~Arbiter()
{
   pthread_mutex_destroy(& m_freeze_mutex) ;
   pthread_mutex_destroy(& m_votes_mutex) ;
   purge_container(m_votes) ; // should be okay to do without mutex
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif // INVT_HAVE_PTHREAD

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

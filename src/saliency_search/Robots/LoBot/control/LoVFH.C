/**
   \file  Robots/LoBot/control/LoVFH.C
   \brief This file defines the non-inline member functions of the
   lobot::VFH class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoVFH.C $
// $Id: LoVFH.C 13037 2010-03-23 01:00:53Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoVFH.H"
//#include "Robots/LoBot/control/LoSpeedArbiter.H"

//#include "Robots/LoBot/LoApp.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"

//#include "Robots/LoBot/misc/LoUpdateLock.H"
//#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/LoRegistry.H"
#include "Robots/LoBot/util/LoGL.H"

// OpenGL headers
#ifdef INVT_HAVE_LIBGL
#include <GL/gl.h>
#endif

// Standard C++ headers
#include <algorithm>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//--------------------------- LOCAL HELPERS -----------------------------

// Retrieve settings from vfh section of config file
template<typename T>
static inline T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>(LOBE_VFH, key, default_value) ;
}

//-------------------------- INITIALIZATION -----------------------------

VFH::VFH()
   : base(clamp(conf("update_delay", 150), 1, 1500))
{
   start(LOBE_VFH) ;
}

void VFH::pre_run()
{
}

//---------------------- THE BEHAVIOUR'S ACTION -------------------------

void VFH::action()
{
   // Vote to drive straight ahead
   //TurnArbiter::Vote* V = new TurnArbiter::Vote() ;
   //V->dump("VFH::action") ;
   //TurnArbiter::instance().vote(base::name, V) ;

   // Vote to drive at user-specified cruising speed (or PWM)
   //SpeedArbiter::instance().vote(base::name,
      //new SpeedArbiter::Vote(Params::cruising_speed(),
                             //Params::cruising_pwm())) ;

   //viz_lock() ;
      //m_vote = *V ; // record most recent vote for visualization
      //m_vote.dump("VFH::action") ;
   //viz_unlock() ;
}

//--------------------------- VISUALIZATION -----------------------------

#ifdef INVT_HAVE_LIBGL

void VFH::render()
{
   // Render the votes visualization
   //glBegin(GL_LINES) ;
   //viz_lock() ;
      /*
      for (TurnArbiter::Vote::iterator it = m_vote.begin(); it; ++it)
      {
         float vote = it.value() ;
         float direction = it.direction() ;
         if (vote < 0) // voted against this direction
         {
            glColor3f(1, 0, 0) ;
            glVertex2i(0, 0) ;
            glVertex2f(-vote * cos(direction), -vote * sin(direction)) ;
         }
         else if (vote > 0) // voted for this direction
         {
            glColor3f(0, 1, 0) ;
            glVertex2i(0, 0) ;
            glVertex2f(vote * cos(direction), vote * sin(direction)) ;
         }
      }
      */
   //viz_unlock() ;
   //glEnd() ;

   // Label the visualization so that it is easy to tell which behaviour
   // is being visualized.
   glColor3f(0, 1, 1) ;
   glPushMatrix() ;
      glLoadIdentity() ;
      draw_label(-0.9f, -0.9f, "VFH") ;
   glPopMatrix() ;
}

#endif

//----------------------------- CLEAN-UP --------------------------------

VFH::~VFH(){}

//-------------------------- KNOB TWIDDLING -----------------------------

// Parameters initialization
VFH::Params::Params()
{}

// Parameters clean-up
VFH::Params::~Params(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

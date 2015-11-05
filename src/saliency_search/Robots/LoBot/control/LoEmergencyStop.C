/**
   \file  Robots/LoBot/control/LoEmergencyStop.C
   \brief This file defines the non-inline member functions of the
   lobot::EmergencyStop class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoEmergencyStop.C $
// $Id: LoEmergencyStop.C 13620 2010-06-25 05:12:03Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoEmergencyStop.H"
#include "Robots/LoBot/control/LoMetrics.H"
#include "Robots/LoBot/control/LoSpeedArbiter.H"

#include "Robots/LoBot/LoApp.H"
#include "Robots/LoBot/slam/LoMap.H"

#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/thread/LoUpdateLock.H"

#include "Robots/LoBot/misc/LoRegistry.H"
#include "Robots/LoBot/util/LoGL.H"
#include "Robots/LoBot/util/LoMath.H"

// OpenGL headers
#ifdef INVT_HAVE_LIBGL
#include <GL/gl.h>
#endif

// Standard C++ headers
#include <iomanip>
#include <algorithm>
#include <iterator>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//--------------------------- LOCAL HELPERS -----------------------------

// Retrieve settings from emergency_stop section of config file
template<typename T>
static inline T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>(LOBE_EMERGENCY_STOP, key, default_value) ;
}

//-------------------------- INITIALIZATION -----------------------------

// DEVNOTE: It ought to be okay to use the DangerZone object without the
// UpdateLock because, at this point, only the main thread should be
// active. The other threads, even if they have been created, should be
// waiting for the main thread to signal that initialization is complete.
// See the lobot::App, lobot::Behavior and lobot::Arbiter class
// implementations for the details.
EmergencyStop::EmergencyStop()
   : base(clamp(conf("update_delay", 75), 1, 1000),
          LOBE_EMERGENCY_STOP,
          conf<std::string>("geometry", "480 140 140 140"))
{
   m_blocks.reserve(DangerZone::num_blocks()) ;
   std::copy(DangerZone::begin(), DangerZone::end(),
             std::back_inserter(m_blocks)) ;
   start(LOBE_EMERGENCY_STOP) ;
}

//---------------------- THE BEHAVIOUR'S ACTION -------------------------

// The emergency stop behaviour monitors the robot's danger zone and
// issues a stop command when things get too close.
void EmergencyStop::action()
{
   UpdateLock::begin_read() ;
      bool danger_zone_penetrated = DangerZone::penetrated() ;

      viz_lock() ;
         std::copy(DangerZone::begin(), DangerZone::end(), m_blocks.begin()) ;
      viz_unlock() ;
   UpdateLock::end_read() ;

   if (danger_zone_penetrated) { // stop the robot
      SpeedArbiter::instance().
         vote(LOBE_EMERGENCY_STOP, new SpeedArbiter::Vote(0, 0)) ;

      Metrics::Log log ;
      log << std::setw(Metrics::opw()) << std::left << "emergency stop" ;
      Map* M = App::map() ;
      if (M)
         log << M->current_pose() ;
   }
}

//--------------------------- VISUALIZATION -----------------------------

#ifdef INVT_HAVE_LIBGL

static void render_reading(const LRFData::Reading& R)
{
   float d = R.distance()/DangerZone::max() ;
   glVertex2i(0, 0) ;
   glVertex2f(d * cos(R.angle()), d * sin(R.angle())) ;
}

static void render_block(const DangerZone::Block& block)
{
   std::for_each(block.danger_begin(), block.danger_end(), render_reading) ;
}

void EmergencyStop::render_me()
{
   // Make local copy so that emergency stop thread isn't held up waiting
   // for visualization thread to complete.
   viz_lock() ;
      Blocks blocks = m_blocks ;
   viz_unlock() ;

   // Render the stop state visualization
   unit_view_volume() ;
   glBegin(GL_LINES) ;
      glColor3f(1, 0, 0) ;
      std::for_each(blocks.begin(), blocks.end(), render_block) ;
   glEnd() ;

   // Label the visualization so that it is easy to tell which behaviour
   // is being visualized.
   restore_view_volume() ;
   text_view_volume() ;
   glColor3f(0, 1, 1) ;
   draw_label(3, 12, "Eme. Stop") ;

   restore_view_volume() ;
}

#endif

//----------------------------- CLEAN-UP --------------------------------

EmergencyStop::~EmergencyStop(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

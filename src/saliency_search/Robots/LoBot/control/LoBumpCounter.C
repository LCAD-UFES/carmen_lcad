/**
   \file  Robots/LoBot/control/LoBumpCounter.C
   \brief This file defines the non-inline member functions of the
   lobot::BumpCounter class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoBumpCounter.C $
// $Id: LoBumpCounter.C 13786 2010-08-13 00:26:38Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoBumpCounter.H"
#include "Robots/LoBot/control/LoMetrics.H"

#include "Robots/LoBot/LoApp.H"
#include "Robots/LoBot/slam/LoMap.H"

#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/LoRegistry.H"
#include "Robots/LoBot/util/LoMath.H"

// Standard C++ headers
#include <iomanip>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

BumpCounter::BumpCounter()
   : base(clamp(get_conf(LOBE_BUMP_COUNTER, "update_delay", 1500),
                1000, 5000),
          LOBE_BUMP_COUNTER)
{
   start(LOBE_BUMP_COUNTER) ;
}

void BumpCounter::pre_run()
{
   Robot* robot = App::robot() ;
   if (! robot)
      throw behavior_error(MOTOR_SYSTEM_MISSING) ;
   robot->add_hook(Robot::SensorHook(
      sensor_hook, reinterpret_cast<unsigned long>(this))) ;
}

//---------------------- THE BEHAVIOUR'S ACTION -------------------------

void
BumpCounter::
sensor_hook(const Robot::Sensors& sensors, unsigned long client_data)
{
   if (sensors.bump_front()) {
      Metrics::Log log ;
      log << std::setw(Metrics::opw()) << std::left << "bump" ;
      Map* M = App::map() ;
      if (M)
         log << M->current_pose() ;
   }
}

// All the action is in the sensor hook
void BumpCounter::action(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

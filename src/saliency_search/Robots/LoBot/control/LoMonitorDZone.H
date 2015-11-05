/**
   \file  Robots/LoBot/control/LoMonitorDZone.H
   \brief A behaviour for monitoring the robot's danger zone and taking
   one of several possible actions as per the user's configuration.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoMonitorDZone.H $
// $Id: LoMonitorDZone.H 14040 2010-09-24 17:55:23Z mviswana $
//

#ifndef LOBOT_MONITOR_DANGER_ZONE_BEHAVIOUR_DOT_H
#define LOBOT_MONITOR_DANGER_ZONE_BEHAVIOUR_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoBehavior.H"
#include "Robots/LoBot/misc/factory.hh"

// Standard C++ headers
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::MonitorDZone
   \brief A behaviour for monitoring the robot's danger zone and taking
   one of several possible actions depending on the user's config
   settings.

   This class implements a behaviour that monitors the danger zone and
   takes some user-specified action after it has been penetrated for at
   least some predetermined period. The supported actions are:

      - pause the robot and let user figure out how to proceed
      - spin by a random amount
      - quit the Robolocust controller application
*/
class MonitorDZone : public Behavior {
   // Prevent copy and assignment
   MonitorDZone(const MonitorDZone&) ;
   MonitorDZone& operator=(const MonitorDZone&) ;

   // Handy type to have around in a derived class
   typedef Behavior base ;

   // Boilerplate code to make the generic factory design pattern work
   friend  class subfactory<MonitorDZone, base> ;
   typedef register_factory<MonitorDZone, base> my_factory ;
   static  my_factory register_me ;

   /// This variable keeps track of when the danger zone was penetrated.
   long long m_time ;

   /// A private constructor because behaviours are instantiated with an
   /// object factory and not directly by clients.
   MonitorDZone() ;

   /// This method provides the body of the behaviour's main loop.
   void action() ;

   /// This method resets the danger zone penetration time stamp.
   void reset() ;

   /// Clean-up.
   ~MonitorDZone() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

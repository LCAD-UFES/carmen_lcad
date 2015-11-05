/**
   \file  Robots/LoBot/control/LoEmergencyStop.H
   \brief A behaviour for stopping the robot if stuff gets too close for
   comfort.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoEmergencyStop.H $
// $Id: LoEmergencyStop.H 12858 2010-02-17 20:35:10Z mviswana $
//

#ifndef LOBOT_EMERGENCY_STOP_BEHAVIOUR_DOT_H
#define LOBOT_EMERGENCY_STOP_BEHAVIOUR_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoBehavior.H"
#include "Robots/LoBot/io/LoDangerZone.H"
#include "Robots/LoBot/misc/factory.hh"

// Standard C++ headers
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::EmergencyStop
   \brief A behaviour for stopping the robot dead in its tracks when it
   senses things getting too close for comfort.

   This class implements a behaviour that uses the laser range finder to
   sense when the robot's "danger zone" has been penetrated and shuts off
   the motors when that happens.
*/
class EmergencyStop : public Behavior {
   // Prevent copy and assignment
   EmergencyStop(const EmergencyStop&) ;
   EmergencyStop& operator=(const EmergencyStop&) ;

   // Handy type to have around in a derived class
   typedef Behavior base ;

   // Boilerplate code to make the generic factory design pattern work
   friend  class subfactory<EmergencyStop, base> ;
   typedef register_factory<EmergencyStop, base> my_factory ;
   static  my_factory register_me ;

   /// This behaviour maintains a copy of the danger zone's blocks for
   /// visualization purposes.
   //@{
   typedef std::vector<DangerZone::Block> Blocks ;
   Blocks m_blocks ;
   //@}

   /// A private constructor because behaviours are instantiated with an
   /// object factory and not directly by clients.
   EmergencyStop() ;

   /// This method provides the body of the behaviour's main loop.
   void action() ;

   /// Visualization routines to aid with development and debugging.
   void render_me() ;

   /// Clean-up.
   ~EmergencyStop() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/**
   \file  Robots/LoBot/control/LoExtricate.H
   \brief A behaviour for getting the robot unstuck when the emergency
   stop behaviour commands a full stop because stuff has gotten too close
   for comfort.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoExtricate.H $
// $Id: LoExtricate.H 13813 2010-08-21 05:36:12Z mviswana $
//

#ifndef LOBOT_EXTRICATE_BEHAVIOUR_DOT_H
#define LOBOT_EXTRICATE_BEHAVIOUR_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoBehavior.H"
#include "Robots/LoBot/misc/LoVector.H"
#include "Robots/LoBot/misc/factory.hh"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Extricate

   \brief A behaviour for unsticking the robot after the emergency stop
   behaviour stops it dead in its tracks because things have gotten too
   close for comfort.

   This class implements a behaviour that monitors the robot's danger
   zone and commands the robot to back-up, turn and/or drive forwards as
   required to get it away from the wall, corner or other obstacle
   currently obstructing its path so that it can start moving normally
   again.

   The extrication algorithm is based on the potential field idea: each
   danger reading exerts a virtual repulsive force on the robot while
   distance measurements outside of the danger zone exert an attractive
   force. The sum of all these force vectors is used to drive and steer
   the robot away from obstacles.
*/
class Extricate : public Behavior {
   // Prevent copy and assignment
   Extricate(const Extricate&) ;
   Extricate& operator=(const Extricate&) ;

   // Handy type to have around in a derived class
   typedef Behavior base ;

   // Boilerplate code to make the generic factory design pattern work
   friend  class subfactory<Extricate, base> ;
   typedef register_factory<Extricate, base> my_factory ;
   static  my_factory register_me ;

   /// The extricate behaviour works by computing a virtual repulsive
   /// force for each danger zone reading and an attractive force for
   /// each distance reading that lies outside the danger zone. The
   /// resulting total force exerted on the robot is then used to drive
   /// and steer the robot away from obstacles.
   ///
   /// These data members are used to keep track of the attractive,
   /// repulsive and total force vectors.
   ///
   /// DEVNOTE: Actually, we only need to remember these forces for
   /// visualization purposes.
   Vector m_attractive, m_repulsive, m_total_force ;

   /// In each iteration, the extricate behaviour issues both a drive and
   /// a turn command. This structure is a convenient way to hold both
   /// these commands together in one place.
   struct Command {
      int drive ;
      int turn  ;

      Command() ;
   } ;

   /// This data member holds the extricate behaviour's most recent
   /// drive and turn commands. Useful for visualization.
   Command m_cmd ;

   /// A private constructor because behaviours are instantiated with an
   /// object factory and not directly by clients.
   Extricate() ;

   /// Things to do/check before regular action processing kicks in.
   void pre_run() ;

   /// This method implements the behaviour's extrication strategy. As
   /// mentioned earlier, it works by computing attractive and repulsive
   /// forces based on the laser range finder's distance readings.
   void action() ;

   /// Visualization routine to aid with development and debugging.
   void render_me() ;

   /// Clean-up.
   ~Extricate() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

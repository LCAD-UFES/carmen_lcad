/**
   \file  Robots/LoBot/control/LoLGMDExtricateSimple.H
   \brief A behaviour for getting the robot unstuck using the raw LGMD
   spikes and a simple, reactive approach in conjunction with in-place
   turns as steering commands instead of "vector-style" turn commands.

   This file defines a class that implements a reactive obstacle
   avoidance behaviour that uses raw LGMD spikes as its input. The
   behaviour's output consists of steering commands that stop and spin
   the robot away from approaching obstacles rather than veer it away
   while it moves.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoLGMDExtricateSimple.H $
// $Id: LoLGMDExtricateSimple.H 13813 2010-08-21 05:36:12Z mviswana $
//

#ifndef LOBOT_LGMD_EXTRICATE_SIMPLE_DOT_H
#define LOBOT_LGMD_EXTRICATE_SIMPLE_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoBehavior.H"
#include "Robots/LoBot/misc/factory.hh"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::LGMDExtricateSimple

   \brief A behaviour for moving the robot away from obstacles by
   stopping and spinning it in the direction of least LGMD spiking.

   This class implements a behaviour that uses raw LGMD spikes to move
   lobot away from obstacles obstructing its path. What we mean by "raw"
   here is that this behaviour does not process the LGMD spikes in any
   way (e.g., time-to-impact state estimation or signal processing of any
   kind).

   Instead, the behaviour simply scans the LGMD array and spins the robot
   to make it face the direction corresponding to minimal spiking
   activity.
*/
class LGMDExtricateSimple : public Behavior {
   // Prevent copy and assignment
   LGMDExtricateSimple(const LGMDExtricateSimple&) ;
   LGMDExtricateSimple& operator=(const LGMDExtricateSimple&) ;

   // Handy type to have around in a derived class
   typedef Behavior base ;

   // Boilerplate code to make the generic factory design pattern work
   friend  class subfactory<LGMDExtricateSimple, base> ;
   typedef register_factory<LGMDExtricateSimple, base> my_factory ;
   static  my_factory register_me ;

   /// In each iteration, the lgmd_extricate_simple behaviour issues both
   /// a drive and a turn command. This structure is a convenient way to
   /// hold both these commands together in one place.
   struct Command {
      int drive ;
      int turn  ;

      Command() ;
   } ;

   /// This data member holds the behaviour's most recent drive and turn
   /// commands. Useful for visualization.
   Command m_cmd ;

   /// A private constructor because behaviours are instantiated with an
   /// object factory and not directly by clients.
   LGMDExtricateSimple() ;

   /// Things to do/check before regular action processing kicks in.
   void pre_run() ;

   /// This method implements the behaviour's extrication strategy. As
   /// mentioned earlier, it works by spinning the robot to face the
   /// direction corresponding to minimal LGMD spiking activity.
   void action() ;

   /// Visualization routine to aid with development and debugging.
   void render_me() ;

   /// Clean-up.
   ~LGMDExtricateSimple() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

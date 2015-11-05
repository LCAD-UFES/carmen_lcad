/**
   \file  Robots/LoBot/control/LoLGMDExtricateVFF.H
   \brief A behaviour for getting the robot unstuck using the raw LGMD
   spikes and a virtual force field.

   This file defines a class that implements the virtual force field
   concept for moving lobot away from obstacles using raw LGMD spikes.
   What we mean by "raw" here is that this behaviour does not process the
   LGMD spikes in any way (e.g., time-to-impact state estimation or
   signal processing of any kind). Instead, the behaviour considers a
   spike rate threshold. When the spike rate for a locust exceeds this
   threshold, it is converted to a repulsive force along the direction in
   which that locust is looking. Similarly, spike rates below the
   threshold become attractive forces along their respective directions.
   The attractive and repulsive forces are then combined to produce a
   vector that is used to decide on an appropriate steering command.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoLGMDExtricateVFF.H $
// $Id: LoLGMDExtricateVFF.H 13813 2010-08-21 05:36:12Z mviswana $
//

#ifndef LOBOT_LGMD_EXTRICATE_VFF_DOT_H
#define LOBOT_LGMD_EXTRICATE_VFF_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoBehavior.H"
#include "Robots/LoBot/misc/LoVector.H"
#include "Robots/LoBot/misc/factory.hh"

// Standard C++ headers
#include <vector>
#include <utility>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::LGMDExtricateVFF

   \brief A behaviour for moving the robot away from obstacles by
   applying a virtual force field comprised of repulsive and attractive
   forces based on the raw LGMD inputs.

   This class implements a behaviour designed to move lobot away from
   obstacles obstructing its path by applying the virtual force field
   concept using raw LGMD spikes. What we mean by "raw" here is that this
   behaviour does not process the LGMD spikes in any way (e.g.,
   time-to-impact state estimation or signal processing of any kind).

   Instead, the behaviour considers a user-specified spike rate
   threshold. When the spike rate for a locust exceeds this threshold, it
   is converted to a repulsive force along the direction in which that
   locust is looking. Similarly, spike rates below the threshold become
   attractive forces along their respective directions. The attractive
   and repulsive forces are then combined to produce a vector that is
   used to decide on an appropriate steering command.
*/
class LGMDExtricateVFF : public Behavior {
   // Prevent copy and assignment
   LGMDExtricateVFF(const LGMDExtricateVFF&) ;
   LGMDExtricateVFF& operator=(const LGMDExtricateVFF&) ;

   // Handy type to have around in a derived class
   typedef Behavior base ;

   // Boilerplate code to make the generic factory design pattern work
   friend  class subfactory<LGMDExtricateVFF, base> ;
   typedef register_factory<LGMDExtricateVFF, base> my_factory ;
   static  my_factory register_me ;

   /// This behaviour works by computing a virtual repulsive force for
   /// each locust whose LGMD spike rate exceeds some preconfigured
   /// threshold and an attractive force for each locust whose LGMD spike
   /// rate falls below the same threshold. The resulting total force
   /// exerted on the robot is then used to drive and steer the robot
   /// away from obstacles.
   ///
   /// These data members are used to keep track of the attractive,
   /// repulsive and total force vectors.
   ///
   /// DEVNOTE: Actually, we only need to remember these forces for
   /// visualization purposes.
   Vector m_attractive, m_repulsive, m_total_force ;

   /// In each iteration, this behaviour issues both a drive and a turn
   /// command. This structure is a convenient way to hold both these
   /// commands together in one place.
   struct Command {
      int drive ;
      int turn  ;

      Command() ;
   } ;

   /// This data member holds the most recent commands issued by this
   /// behaviour. Useful for visualization.
   Command m_cmd ;

   /// A private constructor because behaviours are instantiated with an
   /// object factory and not directly by clients.
   LGMDExtricateVFF() ;

   /// Things to do/check before regular action processing kicks in.
   void pre_run() ;

   /// This method implements the behaviour's extrication strategy. As
   /// mentioned earlier, it works by determining the sum of attractive
   /// and repulsive forces that are computed using LGMD spike rates and
   /// a threshold.
   void action() ;

   /// Visualization routine to aid with development and debugging.
   void render_me() ;

   /// Clean-up.
   ~LGMDExtricateVFF() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

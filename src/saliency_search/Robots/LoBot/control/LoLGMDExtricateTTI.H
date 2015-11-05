/**
   \file  Robots/LoBot/control/LoLGMDExtricateTTI.H
   \brief A behaviour for getting the robot unstuck using LGMD spikes and
   a virtual force field based on a Bayesian time-to-impact state
   estimation loop.

   This file defines a class that implements the virtual force field
   concept for moving lobot away from obstacles using time-to-impact
   estimates made using LGMD spikes.

   As per the research by Gabbiani, et al., we know that LGMD spikes are
   related to the time-to-impact of approaching objects. Unfortunately,
   given a spike rate, we cannot easily determine the corresponding
   time-to-impact (TTI) because the spike rate function is
   non-invertible.

   Thus, for each (actual or virtual) locust connected to the robot, this
   behaviour performs Bayesian state estimation in an attempt to
   determine the corresponding TTI given spike rate. Once we have TTI, we
   can use the velocity information returned by the robot's motor system
   to calculate a distance estimate for each locust. This would, in
   effect, allow us to use the locust array as a kind of range sensor.

   These distance "readings" can then be used to construct a virtual
   force field comprised of repulsive and attractive forces just as we
   would do for a real range sensor.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoLGMDExtricateTTI.H $
// $Id: LoLGMDExtricateTTI.H 13879 2010-09-03 19:52:57Z mviswana $
//

#ifndef LOBOT_LGMD_EXTRICATE_TTI_DOT_H
#define LOBOT_LGMD_EXTRICATE_TTI_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoBehavior.H"

#include "Robots/LoBot/tti/LoTTIEstimator.H"
#include "Robots/LoBot/lgmd/LocustModel.H"

#include "Robots/LoBot/misc/LoVector.H"
#include "Robots/LoBot/misc/factory.hh"

// Standard C++ headers
#include <string>
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::LGMDExtricateTTI

   \brief A behaviour for moving the robot away from obstacles by
   applying a virtual force field comprised of repulsive and attractive
   forces based on time-to-impact estimates derived from LGMD inputs.

   This class implements a behaviour designed to move lobot away from
   obstacles obstructing its path by applying the virtual force field
   concept on distance "readings" derived from time-to-impact (TTI)
   estimates for each (real or virtual) locust connected to the robot.

   Since LGMD spikes are related to the TTI of approaching objects, we
   can use Bayesian state estimation to determine the TTI given the
   current spike rate. Combining this TTI with the motor system's
   velocity info allows us to compute the distance to obstacles for each
   locust direction, in effect, converting our array of visual collision
   detectors into a range sensor.

   Once we have distance estimates, we can build a virtual force field
   wherein distances above a threshold represent attractive forces and
   distances below the same threshold exert repulsive forces. The
   attractive and repulsive forces are then combined to produce a vector
   that is used to decide on an appropriate steering command.
*/
class LGMDExtricateTTI : public Behavior {
   // Prevent copy and assignment
   LGMDExtricateTTI(const LGMDExtricateTTI&) ;
   LGMDExtricateTTI& operator=(const LGMDExtricateTTI&) ;

   // Handy type to have around in a derived class
   typedef Behavior base ;

   // Boilerplate code to make the generic factory design pattern work
   friend  class subfactory<LGMDExtricateTTI, base> ;
   typedef register_factory<LGMDExtricateTTI, base> my_factory ;
   static  my_factory register_me ;

   /// In effect, this behaviour attempts to convert the array of locusts
   /// into a range sensor by estimating the time-to-impact from each
   /// locust's LGMD spike rate. This data structure holds the TTI
   /// estimators for all the locusts.
   std::vector<TTIEstimator*> m_tti ;

   /// Once we have TTI estimates for each locust, we compute distances
   /// to obstacles by using the velocity information returned by the
   /// motor system. Then, we construct a virtual force field based on
   /// these distance "readings." Each distance estimate below a certain
   /// threshold exerts a repulsive force on the robot; a distance
   /// estimate above the same threhold results in an attractive force.
   /// The resulting total force is used to drive and steer the robot
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
   LGMDExtricateTTI() ;

   /// This method creates the TTI estimators before the behaviour's
   /// regular action processing loop begins.
   void pre_run() ;

   /// This method implements the behaviour's extrication strategy. As
   /// mentioned earlier, it works by determining the sum of attractive
   /// and repulsive forces that are computed using distance
   /// "measurements" derived from TTI estimates based on LGMD spike
   /// rates.
   void action() ;

   /// Helper method to record visualization data.
   void record_viz(const Vector& att = Vector(),
                   const Vector& rep = Vector(),
                   const Vector& tot = Vector(),
                   const Command& = Command()) ;

   /// Visualization routines to aid with development and debugging.
   void render_me() ;

   /// Clean-up.
   ~LGMDExtricateTTI() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

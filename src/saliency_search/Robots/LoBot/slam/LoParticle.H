/**
   \file  Robots/LoBot/misc/LoParticle.H
   \brief A particle for the FastSLAM algorithm's particle filter.

   This file defines a class that encapsulates a particle as required by
   the Robolocust implementation of the FastSLAM algorithm, a
   Rao-Blackwellized particle filter for building an occupancy grid map
   and recording lobot's trajectory within this map.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/slam/LoParticle.H $
// $Id: LoParticle.H 13575 2010-06-17 01:42:18Z mviswana $
//

#ifndef LOBOT_PARTICLE_DOT_H
#define LOBOT_PARTICLE_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/slam/LoOccGrid.H"
#include "Robots/LoBot/slam/LoScanMatch.H"
#include "Robots/LoBot/slam/LoPose.H"
#include "Robots/LoBot/slam/LoOdometry.H"

// Boost headers
#include <boost/shared_ptr.hpp>

// Standard C++ headers
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Particle
   \brief Encapsulation of particle used in FastSLAM algorithm.

   Robolocust uses FastSLAM to build an occupancy grid map of its
   surroundings and record its trajectory (which is used to compare the
   performance of different LGMD-based avoidance algorithms). This class
   encapsulates the notion of a particle as required by the FastSLAM
   algorithm.
*/
class Particle {
   /// In a particle filter, each particle hypothesizes the current state
   /// of the system. In a robot mapping application, the state is
   /// basically the robot's pose.
   Pose m_pose ;

   /// In addition to the pose, in FastSLAM, a Rao-Blackwellized particle
   /// filter, each particle must also carry with it its own copy of the
   /// occupancy grid being built.
   ///
   /// DEVNOTE: We use a shared pointer here rather than an OccGrid
   /// object because this FastSLAM implementation can be configured to
   /// perform localization only. In this mode of operation, the user
   /// supplies a known obstacle map and we use only the Monte Carlo
   /// Localization component of the FastSLAM algorithm.
   ///
   /// Since the map will be common to all particles in localization
   /// mode, we use a shared pointer. In normal SLAM mode, each shared
   /// pointer will, in fact, point to its own unique OccGrid instance.
   /// But in localization mode, all the shared pointers will point to a
   /// single OccGrid instance.
   typedef boost::shared_ptr<OccGrid> MapPtr ;
   MapPtr m_map ;

   /// Since grid-based FastSLAM requires each particle to carry its own
   /// copy of the occupancy map, the algorithm's CPU and memory
   /// requirements can become quite daunting. Therefore, it is critical
   /// that we somehow reduce the total number of particles required by
   /// the filter.
   ///
   /// One way to do that is to eschew raw odometry in favour something
   /// more precise (e.g., laser range finder based scan matching). For
   /// that to work, we need to keep track of the most recent LRF scan.
   /// The latest scan will then be registered to this reference scan and
   /// the resulting scan matching transformation will be used as the
   /// odometric input to the particle filter's motion model.
   Scan m_ref_scan ;

   /// Each particle is weighted according to the sensor model. This data
   /// member holds the particle's current weight.
   float m_weight ;

   /// Debugging support: since there are (typically) hundreds of
   /// particles, distinguishing the state of one particle's debug output
   /// from another's can be challenging. To make this a little easier,
   /// we prefix each of a particle's debug output with that particle's
   /// address. However, having to use reinterpret_cast in each debug
   /// statement is a hassle. So, we use the following data member to
   /// store the particle's address as an unsigned int when the particle
   /// is created. Thereafter, we can simply use this value without
   /// worrying about casting the same pointer over and over...
   unsigned long m_this ;

public:
   /// When a particle is created, the client should supply an initial
   /// weight as well as an initial LRF scan (for the scan matching). The
   /// particle will start off assuming a pose of (0,0,0) and an
   /// occupancy map which has equal probabilities for all locations.
   Particle(float initial_weight, const std::vector<int>& initial_scan) ;

   /// When FastSLAM is configured to perform localization only, i.e., no
   /// mapping, the user will specify a map to use and FastSLAM will use
   /// only its Monte Carlo Localization component. This constructor
   /// takes the same parameters as the normal SLAM mode constructor. In
   /// addition, it takes an OccGrid to serve as the common map to be
   /// shared by all the particles. The map has to be loaded by this
   /// class's client.
   Particle(float initial_weight, const std::vector<int>& initial_scan,
            const boost::shared_ptr<OccGrid>& known_map) ;

   /// Copy.
   Particle(const Particle&) ;

   /// Assignment.
   Particle& operator=(const Particle&) ;

   /// This method "perturbs" the particle so as to produce a new state
   /// hypothesis based on the current control input. The current sensor
   /// data is also used because the Robolocust implementation of
   /// FastSLAM uses laser range scan matching to correct the raw
   /// odometry obtained from the robot's lower layers.
   ///
   /// In Bayesian/particle filter parlance, this method implements the
   /// prediction step, producing a new state according to the likelihood
   /// function P(xt|xt-1,ut).
   void apply_motion_model(const Odometry& control_input,
                           const std::vector<int>& sensor_data) ;

   /// This method reweights the particle once it has been "moved" to a
   /// new state. The weighting is based on the current sensor
   /// observation.
   ///
   /// In Bayesian/particle filter parlance, this method implements the
   /// correction step, taking the latest observation into account to
   /// correct the state predicted by the motion model according to the
   /// likelihood function P(xt|zt).
   void apply_sensor_model(const std::vector<int>& range_readings) ;

   /// This method updates the occupancy map based on the latest state
   /// hypothesis and laser range finder data. That is, it "evolves" the
   /// latest occupancy probabilities according to the likelihood
   /// function P(mt|xt,mt-1).
   void update_map(const std::vector<int>& range_readings) ;

   /// This method normalizes the particle weight.
   void normalize(float normalizer) {m_weight *= normalizer ;}

   /// Accessors.
   //@{
   float  weight() const {return m_weight    ;}
   Pose   pose()   const {return m_pose      ;}
   const OccGrid& map() const {return *m_map ;}
   //@}

   /// This function can be used to randomize this particle's pose
   /// estimate. It is useful when we want to introduce random particles
   /// to help deal with mislocalizations.
   void randomize() ;

   /// Visualization support. This helper packages a particle's pose and
   /// weight together in one structure.
   struct Viz {
      Pose  pose ;
      float weight ;
      Viz(const Pose&, float) ;
   } ;

   /// Returns the particle's current pose and weight for visualization
   /// purposes.
   Viz viz() const {return Viz(m_pose, m_weight) ;}
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

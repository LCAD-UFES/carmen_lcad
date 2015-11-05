/**
   \file  Robots/LoBot/misc/LoFastSLAM.H
   \brief An implementation of the grid-based FastSLAM algorithm.

   This file defines a class that implements the FastSLAM algorithm for
   building an occupancy grid map while simultaneously localizing the
   robot within that map.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/slam/LoFastSLAM.H $
// $Id: LoFastSLAM.H 13575 2010-06-17 01:42:18Z mviswana $
//

#ifndef LOBOT_FASTSLAM_DOT_H
#define LOBOT_FASTSLAM_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/slam/LoParticle.H"
#include "Robots/LoBot/slam/LoOccGrid.H"
#include "Robots/LoBot/slam/LoPose.H"
#include "Robots/LoBot/slam/LoOdometry.H"
#include "Robots/LoBot/io/LoLRFData.H"

// Boost headers
#include <boost/shared_ptr.hpp>

// Standard C++ headers
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::FastSLAM
   \brief An implementation of the grid-based FastSLAM algorithm.

   This class implements the FastSLAM algorithm for building an occupancy
   grid map while simultaneously localizing the robot within this map.
*/
class FastSLAM {
   // Prevent copy and assignment
   FastSLAM(const FastSLAM&) ;
   FastSLAM& operator=(const FastSLAM&) ;

   /// The FastSLAM algorithm is a Rao-Blackwellized particle filter.
   /// Thus, it needs a list of particles to get its work done.
   //@{
   typedef std::vector<Particle> Particles ;
   std::vector<Particle> m_particles ;
   //@}

   /// When the particle filter arrives at some incorrect state estimate,
   /// it is important that it be able to extricate itself from that
   /// erroneous state and move towards the correct state. To help with
   /// this goal, we introduce random particles spread all over the state
   /// space in the hope that, eventually, one of these will be near the
   /// correct state and get a high weight, thus, "guiding" the filter to
   /// the correct state.
   ///
   /// Obviously, it would be detrimental to the filter's performance to
   /// introduce a lot of random particles or to introduce them when the
   /// filter is in the correct state. To determine when random particles
   /// are necessary, we keep track of the short and long-term averages
   /// of the particle weights. When the short-term average performs
   /// poorly w.r.t. the long-term average, we inject random particles
   /// with a frequency proportional to the ratio of the
   /// short-to-long-term particle weight average.
   ///
   /// This technique is described in Chapter 8, section 8.3.5, pages 256
   /// through 260 of "Probabilistic Robotics" by Thrun, Burgard and Fox.
   float m_w_slow, m_w_fast ;

public:
   /// In the grid-based version of FastSLAM, each particle needs to
   /// carry its own copy of the occupancy map being built. As this can
   /// result in a rather large computational and memory burden, it is
   /// crucial to reduce the total number of particles required to
   /// effectively perform the SLAM operation.
   ///
   /// One way to do that is to eschew raw odometry in favour of
   /// something more precise such as 2D scan matching based on laser
   /// range finder measurements. The Robolocust implementation of
   /// FastSLAM does in fact use scan matching.
   ///
   /// Therefore, during initialization, the client must supply an
   /// initial LRF scan.
   FastSLAM(const LRFData&) ;

   /// This implementation of FastSLAM can be configured to forgo the
   /// mapping operation and perform localization only. In that case, the
   /// user must supply a known occupancy grid map, which will be used to
   /// exercise the Monte Carlo Localization component of FastSLAM.
   ///
   /// This constructor is identical to the previous one except that it
   /// expects to be given a known map for localization only mode.
   FastSLAM(const LRFData&, const boost::shared_ptr<OccGrid>&) ;

   /// This method updates the occupancy map and current pose given the
   /// latest (raw) odometry (i.e., control input) and LRF readings
   /// (i.e., sensor data).
   void update(const Odometry&, const LRFData&) ;

private:
   /// This method updates the short and long-term averages of the
   /// particle weights. The ratio of the short-to-long-term averages is
   /// used to determine when to introduce random particles to help deal
   /// with mislocalizations and also the overall number of random
   /// particles to be injected during resampling.
   void update_averages() ;

public:
   /// As mentioned earlier, in grid-based FastSLAM, each particle
   /// carries its own copy of both the occupancy map being built as well
   /// as the robot's current pose. These methods return the current best
   /// hypothesis of the actual map or pose by applying a robust mean
   /// procedure that works like so:
   ///
   /// First, we find the particle with maximum weight. Then, we find the
   /// K particles whose states most closely match that of the max
   /// particle. The final state is computed as the average of these K
   /// states.
   ///
   /// NOTE: K is specified in the Robolocust config file as a percentage
   /// of the total number of particles.
   //@{
   OccGrid current_map()  const ;
   Pose    current_pose() const ;
   //@}

   /// Visualization support: return a list of Particle::Viz structures
   /// so that the client knows the current pose hypotheses and their
   /// corresponding weights.
   std::vector<Particle::Viz> viz() const ;

   /// Clean-up.
   ~FastSLAM() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

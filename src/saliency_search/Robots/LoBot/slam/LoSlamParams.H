/**
   \file  Robots/LoBot/slam/LoSlamParams.H
   \brief A helper class to hold the different SLAM parameters.

   This file defines a helper class that holds the different SLAM
   parameters together in one place that can be used by the different
   bits and pieces that implement Robolocust's SLAM subsystem.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/slam/LoSlamParams.H $
// $Id: LoSlamParams.H 13575 2010-06-17 01:42:18Z mviswana $
//

#ifndef LOBOT_SLAM_PARAMS_DOT_H
#define LOBOT_SLAM_PARAMS_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoDrawable.H"
#include "Robots/LoBot/misc/singleton.hh"
#include "Robots/LoBot/util/range.hh"
#include "Robots/LoBot/util/triple.hh"

// Standard C++ headers
#include <string>
#include <utility>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::SlamParams
   \brief A helper class for holding the various SLAM related parameters.

   The settings for Robolocust's SLAM subsystem are spread over a couple
   of different sections of the config file. This class helps put them
   all in one place so that all the SLAM related modules can access these
   parameters from one place instead of having to duplicate code or
   introduce unweildy module dependencies.
*/
class SlamParams : public singleton<SlamParams> {
   /// Robolocust uses an occupancy grid for its map. Each cell in this
   /// grid holds a probability indicating the confidence level regarding
   /// the presence of an obstacle at that particular position in the
   /// map.
   ///
   /// To make the map cover a reasonably substantial physical area while
   /// simultaneously minimizing the amount of memory required to hold
   /// the grid, we need to discretize the physical space of the robot's
   /// environment. Thus, each cell in the grid will represent a small
   /// square area of the environment.
   ///
   /// This setting specifies the size of the above-mentioned square (in
   /// mm).
   float m_cell_size ;

   /// The map's extents are specified using [left, right, bottom, top]
   /// values. These extents are in the physical environment's coordinate
   /// system (i.e., mm).
   float m_extents[4] ;

   /// The above parameters dictate the size of the map's grid. These two
   /// data members are used to store the grid's dimensions.
   int m_width, m_height ;

   /// This member holds the length of the map's diagonal. It is useful
   /// for the probabilistic beam model applied by the FastSLAM algorithm
   /// to the laser range finder measurements.
   float m_max_distance ;

   /// This member variable holds the x and y scale factors for
   /// converting real/physical coordinates to grid coordinates and vice
   /// versa.
   std::pair<float, float> m_scale ;

   /// The robot's pose w.r.t. the map is specified using an (x, y)
   /// position and an angle indicating the direction in which it is
   /// "looking." This setting specifies the robot's initial pose.
   triple<float, float, float> m_initial_pose ;

   /// When the robot moves, it will experience some amount of
   /// translational and rotational slip. This parameter captures these
   /// errors and they are used in the prediction step of the FastSLAM
   /// algorithm.
   ///
   /// The value of this setting is expected to be a triple of floating
   /// point numbers. The first two numbers specify the amount of
   /// translational error (in mm) to expect in the x and y directions
   /// respectively. The third number specifies the amount of rotational
   /// error in degrees.
   ///
   /// NOTE: Ideally, these parameters will be empirically determined via
   /// either manual experiments designed to characterize and quantify
   /// systematic odometric errors or via some automated learning
   /// approach based on sample data.
   triple<float, float, float> m_errors ;

   /// The FastSLAM algorithm is a Rao-Blackwellized particle filter.
   /// This setting allows us to configure the number of particles to
   /// use.
   int m_num_particles ;

   /// In order to cut down on the amount of processing required by the
   /// FastSLAM algorithm and also to preserve particle diversity,
   /// Robolocust will only resample the particle population when the
   /// variance of the particle weights becomes high (thus indicating a
   /// lot of spread and high uncertainty in the state estimate).
   ///
   /// To make the actual determination as to when to resample,
   /// Robolocust computes the effective sample size by taking the
   /// reciprocal of the sum of the squares of all the weights. When the
   /// effective sample size (ESS) drops below some threshold, the
   /// particle filter's resampling step will kick in.
   ///
   /// The following setting specifies the effective sample size
   /// threshold. Usually, a good value to use for it would be half the
   /// number of particles. However, to disable the ESS check, you may
   /// set this to zero or to a value greater than the number of
   /// particles. In that case, resampling will take place after each and
   /// every iteration of the SLAM update.
   int m_ess_threshold ;

   /// During resampling, it might be a good idea to introduce some
   /// random particles just in case the filter has arrived at an
   /// incorrect state estimate. However, we don't want to insert random
   /// particles all the time.
   ///
   /// Therefore, Robolocust tracks the averages of the particle weights
   /// and inserts random particles only when the short-term average
   /// performs poorly w.r.t. the long-term average. An exponential
   /// filter is used to smooth the short and long-term particle weight
   /// averages to ensure that momentary glitches don't get misconstrued
   /// as incorrect state estimates.
   ///
   /// The following setting specifies the values of the exponential
   /// decays to use for filtering the short and long-term averages.
   /// These values are plugged into a running average formula for
   /// computing the averages. The long-term average's decay parameter
   /// must be much less than that of the short-term average.
   ///
   /// This setting expects to be given two numbers. The first is the
   /// decay parameter for the short-term average and the second is the
   /// decay parameter for the long-term average. Both numbers must lie
   /// between zero and one.
   std::pair<float, float> m_alpha ;

   /// As we know, in a particle filter, each particle makes a guess
   /// about the current state. These guesses are, of course, not
   /// entirely random; rather they are the result of probabilistic
   /// motion and sensor models. Good guesses, i.e., ones that match the
   /// sensor data, get more weight while poor ones are assigned lower
   /// weights and get filtered out. The overall state of the robot can
   /// then be gleaned by analyzing the particle population's density.
   ///
   /// There are many ways to perform particle density analysis. The one
   /// used by Robolocust's FastSLAM implementation works in the
   /// following manner:
   ///
   ///    1. First, find the particle with maximum weight
   ///    2. Then, find K particles whose states most closely match that
   ///       of the above particle
   ///    3. Compute the current state as the average of these K states
   ///
   /// The following setting specifies the value to use for K. It is
   /// specified as a percentage of the total number of particles. To use
   /// just the particle with max weight and not bother with computing
   /// the average of the top K particles, set this value to a negative
   /// number.
   int m_k ;

   /// Like all Bayesian filters, the FastSLAM algorithm has a prediction
   /// step, wherein the latest control command is used to predict a new
   /// state, and a correction step, wherein the latest sensor data is
   /// used to correct the state prediction. Since a laser range finder
   /// serves as lobot's primary sensing modality, the Robolocust
   /// implementation of FastSLAM employs a probabilistic beam model in
   /// its correction step. This beam model involves ray casting, a
   /// computationally intense operation. To cut down on the number of
   /// rays to be cast, we can configure Robolocust to consider only some
   /// of the distance measurements returned by the laser range finder.
   ///
   /// The following setting allows us to do that. It is expected to be a
   /// triple of integers. The first two numbers specify the starting and
   /// ending angles to consider (e.g., from -90 to +90 degrees). The
   /// third number is a step size. Thus, for example, if the beam_specs
   /// setting is "-90 90 15," then the FastSLAM algorithm will only
   /// consider the laser range finder readings corresponding to -90,
   /// -75, -60, -45, -30, -15, 0, 15, 30, 45, 60, 75 and 90 degrees,
   /// which makes for a total of 13 ray casting operations instead of
   /// (potentially) > 180.
   ///
   /// NOTE: Limiting the field of view of the laser range finder speeds
   /// up the FastSLAM algorithm but at the expense of map accuracy.
   triple<int, int, int> m_beam_specs ;

   /// The probabilistic beam model mentioned above works by computing
   /// the expected range reading from the current state estimate (i.e.,
   /// robot pose and occupancy map) and then seeing how well the actual
   /// distance measurements match the expected ones. We expect particles
   /// with good state estimates to match the actual sensor data better
   /// than those with poor estimates. Thus, good particles will get
   /// higher weights and the particle filter will pick those particles
   /// while culling the poor ones from the particle population.
   ///
   /// To rate the state estimates made by different particles, we run
   /// the actual range reading through the formula for a normal
   /// distribution centered at the expected range. This will produce a
   /// number that we can use for the particle weight. The following
   /// setting specifies the standard deviation for the above-mentioned
   /// application of the normal distribution formula. It is a distance
   /// (in mm).
   ///
   /// The value of this setting should reflect the amount of error we
   /// are willing to tolerate between the estimated range readings and
   /// the actual ones. It should also take into account the
   /// discretization parameters of the occupancy map.
   float m_beam_model_sigma ;

   /// The beam model described above computes the likelihood of a single
   /// distance measurement. Since the laser range finder returns
   /// multiple distance readings, we have to multiply the individual
   /// beam probabilities to obtain the final/total measurement
   /// likelihood.
   ///
   /// Unfortunately, as each beam's probability is a number between zero
   /// and one, multiplying the individual probabilities together will
   /// very rapidly result in extremely small numbers: of the order of
   /// ten raised to -50. This causes floating point instabilities.
   ///
   /// To work around this problem, we multiply individual beam
   /// probabilities by a large constant and multiply the resulting
   /// numbers instead of the original [0,1] likelihoods. The following
   /// setting specifies the value of the above-mentioned large constant.
   float m_beam_model_fudge ;

   /// For the beam model described above to work, we need to specify the
   /// minimum and maximum range readings the laser range finder can
   /// produce. Although the robot can determine this directly from the
   /// device itself, depending on the environment and on other factors,
   /// we may want the FastSLAM algorithm to consider a range different
   /// from the device's actual range. This setting allows us to do that.
   /// Its value should be two floating point numbers. The first one
   /// specifies the beam's minimum range (in mm) and the second one its
   /// maximum range (again in mm).
   range<float> m_beam_range ;

   /// In addition to the expected measurement computation, the
   /// probabilistic beam model should also consider various other
   /// factors that come into play and corrupt sensor data. Instead of
   /// explicitly modeling these other things, we simply lump all of them
   /// into a uniform distribution based on the range of distance
   /// readings returned by the LRF. Since this value is a constant, we
   /// can compute and store it with the parameters structure.
   float m_prob_rnd ;

   /// During the map update step, the SLAM algorithm uses each distance
   /// measurement to determine which cells in the occupancy grid are
   /// occupied. Since the grid is a discretized version of the world,
   /// there will, naturally, be discrepancies between the actual
   /// distance reading and the same range in grid coordinates.
   ///
   /// This setting specifies the amount of discrepancy that should be
   /// tolerated. As an example of how this works, let's say that the
   /// value of this setting is set to 100mm and that we get a reading
   /// (in some direction) of 622mm. Then, depending on the map's
   /// discretization parameters, as the algorithm traces the distance
   /// measurement within the occupancy grid in order to find the cell at
   /// which the measurement ends, it will accept a cell that is within
   /// 100mm of 622, i.e., from 522 to 722mm.
   float m_occ_range_error ;

   /// This setting specifies the minimum probability value a cell in the
   /// occupancy grid should have for it to be considered as being
   /// occupied. Cells with probabilities lower than this threshold will
   /// be considered as free space.
   ///
   /// This parameter's value should be a floating point number between
   /// zero and one.
   ///
   /// NOTE: Internally, this number gets converted to its equivalent
   /// log-odds form and the threshold check is then performed in
   /// "log-odds space."
   float m_occ_threshold ;

   /// As the robot moves around and senses its environment, its internal
   /// grid update and query algorithms will not always reliably hit the
   /// same cell over and over (due, for example, to sensor and actuator
   /// noise). Therefore, when we update a cell (x,y) in the occupancy
   /// grid, we should also update its immediate neighbours to ensure
   /// that subsequent queries near (x,y) are not entirely off the mark.
   ///
   /// The following setting specifies a weighting factor to use when
   /// updating a cell. This factor will applied to the cell in question
   /// and the remaining weight will be applied to its immediate
   /// neighbours. To illustrate how this works, let us say the update
   /// weight is set to 0.75. Then, when the robot updates a cell (x,y),
   /// it will apply 75% of the update delta (see the "occupancy_deltas"
   /// setting defined above) to (x,y) and divide the remaining 25% of
   /// the delta value equally between the remaining 8 cells in the
   /// immediate neighbourhood of (x,y). If (x,y) is in one of the
   /// corners of the map, the remaining 25% will be divided equally
   /// between the 3 neighbouring cells; if (x,y) is on an edge boundary
   /// other than a corner, the remaining 25% will be divided equally
   /// between the 5 immediate neighbours.
   ///
   /// As can be gleaned from the above discussion, the value of this
   /// setting should be a floating point number in the range [0,1].
   ///
   /// NOTE: Although it is referred to as the "update" weight, this
   /// parameter is also used when querying the map for occupancy
   /// likelihoods. Thus, when the robot wants to check the occupancy
   /// likelihood of some location (x,y), this weighting factor (e.g.,
   /// 75%) will be applied to the actual value stored in that cell and
   /// the remaining weight (25%) will be divided equally between the
   /// immediate neighbours of (x,y) to produce a weighted sum for the
   /// final occupancy value retrieved from the grid.
   float m_update_weight ;

   /// Since odometry tends to be more error-prone than a laser range
   /// finder, Robolocust can be configured to use a scan matching
   /// algorithm to correct the raw odometry and use the corrected values
   /// to compute pose updates in the filter's prediction step. The
   /// following flag can be used to turn scan matching on or off
   /// (default is off).
   bool m_match_scans ;

   /// Although SLAM is designed to learn a map and localize the robot at
   /// the same time, we can configure the Robolocust system to only
   /// perform localization given a known map. The following setting
   /// specifies a file name for a map. The robot will read this map
   /// during initialization and then localize itself w.r.t. this map
   /// using the Monte Carlo Localization component of the SLAM
   /// algorithm.
   ///
   /// The map file is expected to be a plain text file containing
   /// coordinates of obstacles. Each obstacle is specified using four
   /// coordinates. The first two coordinates specify the location of the
   /// lower left corner of a rectangle where the obstacle "begins" and
   /// the next two are for the upper right corner where the obstacle
   /// "ends."
   ///
   /// The coordinates are all expressed in the map's real/physical
   /// coordinate system.
   ///
   /// NOTE: This setting is actually part of the survey behaviour.
   std::string m_map_file ;

   /// This member holds the geometry specs for the map's visualization.
   Drawable::Geometry m_geometry ;

   /// Private constructor because this class is a singleton.
   SlamParams() ;
   friend class singleton<SlamParams> ; // boilerplate

public:
   /// This function returns the size of each cell of the occupancy grid,
   /// i.e., the amount of actual physical area each cell in the grid
   /// takes up.
   ///
   /// NOTE: Each cell in the occupancy grid is a square. This function
   /// returns the length of a side of the square.
   static float map_cell_size() {return instance().m_cell_size ;}

   /// This function returns extents of the map in physical coordinates.
   static void map_extents(float* L, float* R, float* B, float* T) ;

   /// This function returns the map extents via an array of four
   /// numbers ordered like so: left, right, bottom, top. The extents are
   /// in physical coordnates.
   static void map_extents(float ext[4]) ;

   /// This function returns a pointer to the internal array holding the
   /// map extents. This is a convenience function. Clients should not
   /// misuse the returned pointer.
   static const float* map_extents() {return instance().m_extents ;}

   /// These function return the size of the occupancy grid, i.e., the
   /// number of cells in the horizontal and vertical directions. This
   /// number is derived from the map discretization as specified in the
   /// config file.
   //@{
   static int map_width()  {return instance().m_width  ;}
   static int map_height() {return instance().m_height ;}
   //@}

   /// This function returns the max distance that can be traversed
   /// within the map.
   static float max_map_distance() {return instance().m_max_distance ;}

   /// These functions return the scale factors for converting
   /// real/physical coordinates to grid coordinates and vice versa.
   //@{
   static float Sx() {return instance().m_scale.first  ;}
   static float Sy() {return instance().m_scale.second ;}
   //@}

   /// This function returns the robot's initial pose as configured by
   /// the user.
   static const triple<float, float, float>& initial_pose() {
      return instance().m_initial_pose ;
   }

   /// These functions return the translational and rotational errors for
   /// the robot's motion as specified by the user.
   //@{
   static float x_error() {return instance().m_errors.first  ;}
   static float y_error() {return instance().m_errors.second ;}
   static float t_error() {return instance().m_errors.third  ;}
   //@}

   /// This function returns the number of particles to be used by the
   /// FastSLAM algorithm.
   static int num_particles() {return instance().m_num_particles ;}

   /// This function returns the threshold value for the effective sample
   /// size test, which determines when the particle filter will resample
   /// its particle population.
   static int ess_threshold() {return instance().m_ess_threshold ;}

   /// These functions return the decay parameters for the short and
   /// long-term averages used to determine the appropriate time and
   /// number of random particles to be inserted.
   //@{
   static float alpha_fast() {return instance().m_alpha.first  ;}
   static float alpha_slow() {return instance().m_alpha.second ;}
   //@}

   /// This function returns the number of particles that should most
   /// closely match the particle with max weight when deciding the
   /// current best hypothesis regarding the robot's pose and environment
   /// map.
   static int num_matches() {return instance().m_k ;}

   /// These functions return various parameters used by the
   /// probabilistic beam model.
   //@{
   static int   beam_start()           {return instance().m_beam_specs.first ;}
   static int   beam_end()             {return instance().m_beam_specs.second;}
   static int   beam_step()            {return instance().m_beam_specs.third ;}
   static float beam_sigma()           {return instance().m_beam_model_sigma ;}
   static float beam_fudge()           {return instance().m_beam_model_fudge ;}
   static range<float> beam_range()    {return instance().m_beam_range       ;}
   static float beam_prob_rnd()        {return instance().m_prob_rnd         ;}
   static float beam_occ_range_error() {return instance().m_occ_range_error  ;}
   //@}

   /// This function returns the minimum occupancy likelihood
   /// corresponding to the "occupied" state.
   static float occ_threshold() {return instance().m_occ_threshold ;}

   /// This function returns the amount of "spread" associated with
   /// checking or setting a cell's likelihood across its immediate
   /// neighbours.
   static float update_weight() {return instance().m_update_weight ;}

   /// This function returns true if scan matching is enabled; false
   /// otherwise.
   static bool match_scans() {return instance().m_match_scans ;}

   /// If SLAM is configured to perform localization only, i.e., no
   /// mapping, then the user will be expected to specify a file name
   /// containing the known map to be used for localization. This
   /// function returns the map file name.
   static std::string map_file() {return instance().m_map_file ;}

   /// This function returns true if SLAM is configured to perform
   /// localization only. In this mode, the user must supply a known map
   /// and only the Monte Carlo Localization component of the SLAM
   /// algorithm will be used. This mode is useful for testing the
   /// localization part of SLAM in isolation from the mapping part.
   static bool localization_mode() {return ! instance().m_map_file.empty() ;}

   /// This function returns true if SLAM is configured to perform both
   /// localization and mapping. This is the usual operational mode for a
   /// SLAM algorithm.
   static bool slam_mode() {return ! localization_mode() ;}

   /// This function returns the geometry of the map's UI drawable. It is
   /// useful when figuring out transformations for map related rendering
   /// operations.
   static Drawable::Geometry map_geometry() {return instance().m_geometry ;}
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

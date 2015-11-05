/**
   \file  Robots/LoBot/misc/LoParticle.C
   \brief This file defines the non-inline member functions of the
   lobot::Particle class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/slam/LoParticle.C $
// $Id: LoParticle.C 13628 2010-06-28 23:48:02Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/slam/LoParticle.H"
#include "Robots/LoBot/slam/LoMap.H"
#include "Robots/LoBot/slam/LoCoords.H"
#include "Robots/LoBot/slam/LoSlamParams.H"

#include "Robots/LoBot/config/LoConfigHelpers.H"

#include "Robots/LoBot/misc/LoClipper.H"
#include "Robots/LoBot/misc/LoLineRasterizer.H"
#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/singleton.hh"

#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/util/triple.hh"

// INVT utilities
#include "Util/log.H"

// Standard C++ headers
#include <numeric>
#include <algorithm>
#include <utility>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

Particle::
Particle(float initial_weight, const std::vector<int>& range_readings)
   : m_pose(SlamParams::initial_pose()),
     m_map(new OccGrid()),
     m_ref_scan(m_pose, range_readings),
     m_weight(initial_weight)
{
   m_this = reinterpret_cast<unsigned long>(this) ;
}

Particle::
Particle(float initial_weight, const std::vector<int>& range_readings,
         const boost::shared_ptr<OccGrid>& known_map)
   : m_pose(SlamParams::initial_pose()),
     m_map(known_map),
     m_ref_scan(m_pose, range_readings),
     m_weight(initial_weight)
{
   m_this = reinterpret_cast<unsigned long>(this) ;
}

Particle::Particle(const Particle& that)
   : m_pose(that.m_pose),
     m_map(SlamParams::localization_mode() ? that.m_map
                                           : MapPtr(new OccGrid(*that.m_map))),
     m_ref_scan(that.m_ref_scan),
     m_weight(that.m_weight)
{
   m_this = reinterpret_cast<unsigned long>(this) ; // remember: this != that
}

Particle& Particle::operator=(const Particle& that)
{
   if (this != &that)
   {
      m_pose = that.m_pose ;
      m_map  = that.m_map ;
      m_ref_scan = that.m_ref_scan ;
      m_weight   = that.m_weight ;
      m_this = reinterpret_cast<unsigned long>(this) ;//remember: this != that
   }
   return *this ;
}

// Randomize this particle's pose estimate; useful to help deal with
// filter mislocalizations.
void Particle::randomize()
{
   /*
   LERROR("old pose:   [%08lX] %10.2f %10.2f %6.1f",
          m_this, m_pose.x(), m_pose.y(), m_pose.theta()) ;
   // */

   float L, R, B, T ;
   SlamParams::map_extents(&L, &R, &B, &T) ;

   m_pose.x(randomf(L, R)) ;
   m_pose.y(randomf(B, T)) ;
   m_pose.t(clamp_angle(randomf(0, 360))) ;

   /*
   LERROR("rnd pose:   [%08lX] %10.2f %10.2f %6.1f",
          m_this, m_pose.x(), m_pose.y(), m_pose.theta()) ;
   // */
}

//--------------------------- MOTION MODEL ------------------------------

namespace {

// This function computes a new pose from the given pose and odometry
// data.
//
// NOTE: Raw odometry will simply inform us about the net displacement
// and rotation. In actuality, the robot might well have followed a
// bizarrely tortuous/circuitous route to get to its new location and
// bearing. However, there is no (easy/straightforward) way to know or
// model such an occurence.
//
// Nonetheless, if the robot's odometry is checked with reasonable
// frequency, we can safely assume that the path followed by the robot as
// indicated by raw odometry ought to be along a short circular arc,
// which is much easier to model.
//
// But something even easier to work with is simple linear motion. That
// is, if we use raw odometry in short, frequent bursts, we can just
// approximate the motion as an in-place rotation at the current location
// so as to face the destination and then a straight-line translation to
// the new position.
//
// A slight improvement to the above "point-and-shoot" approach is to
// break up the overall circular motion into several piecewise linear
// steps. Thus, we start off in the robot's original heading, move a
// short distance, turn a little, move a little, etc. till we get to the
// final position and heading.
//
// Here, we choose to perform the above piecewise linear motion in three
// steps. First, we move by a third of the total displacement reported by
// odometry. Then, we rotate by half the amount reported by odometry and
// travel straight for another third of the total displacement. Finally,
// we rotate the remaining half angle and travel the final third of the
// way to get to the final position.
//
// Although this approach is crude, the probabilistic framework of the
// FastSLAM particle filter should still be robust enough to work without
// too much degradation in the face of this approximation.
Pose operator+(const Pose& p, const Odometry& od)
{
   float d = od.displacement()/3.0f ;
   float r = od.rotation()/2.0f ;

   // STEP 1: Travel 1/3 of total displacement along current heading
   float t = p.theta() ;
   float x = p.x() + d * cos(t) ;
   float y = p.y() + d * sin(t) ;

   // STEP 2: Rotate 1/2 total rotation and travel another third of the way
   t += r ;
   x += d * cos(t) ;
   y += d * sin(t) ;

   // STEP 3: Rotate and travel remaining amount
   t += r ;
   x += d * cos(t) ;
   y += d * sin(t) ;

   return Pose(x, y, t) ;
}

} // end of local anonymous namespace encapsulating above helpers

// The motion model uses the current control input and produces a new
// state from the current state according to P(xt|xt-1,ut).
//
// NOTE: The raw odometry returned by the robot's lower layers does not
// serve directly as the current control input. Instead, we use the
// latest laser range finder measurements to correct the raw odometry by
// applying the Lu-Milios scan matching algorithm and then use its
// resulting transformation as the control input (viz., ut).
//
// This approach allows us to drastically reduce the number of particles
// required by the grid-based FastSLAM algorithm, which is imperative
// because it requires each particle to carry its own copy of the
// occupancy map. See "An Efficient FastSLAM Algorithm for Generating
// Maps of Large-Scale Cyclic Environments from Raw Laser Range
// Measurements" by Hahnel, Burgard, Fox and Thrun, IROS 2003.
//
// NOTE 2: The Robolocust implementation of the Lu-Milios IDC algorithm
// does not converge (circa May 2010)! As a result, it produces bizarre
// "corrections," which, in fact, are far worse than using raw odometry.
// Therefore, for now, as a workaround, we use a config flag to turn scan
// matching off.
void
Particle::
apply_motion_model(const Odometry& odometry, const std::vector<int>& lrf)
{
   // First, use raw odometry to produce an estimate of the new pose
   Pose new_pose = m_pose + odometry ;
   /*
   LERROR("old pose:   [%08lX] %10.2f %10.2f %6.1f",
          m_this, m_pose.x(), m_pose.y(), m_pose.theta()) ;
   LERROR("odometry:   [%08lX] %7d %18d",
          m_this, odometry.displacement(), odometry.rotation()) ;
   LERROR("new pose:   [%08lX] %10.2f %10.2f %6.1f",
          m_this, new_pose.x(), new_pose.y(), new_pose.theta()) ;
   // */

   // Then, if scan matching is enabled, correct the raw odometry and use
   // the corrected values and suitable noise to perturb the particle.
   float L, R, B, T ;
   SlamParams::map_extents(&L, &R, &B, &T) ;
   const float x_noise = SlamParams::x_error() ;
   const float y_noise = SlamParams::y_error() ;
   const float t_noise = SlamParams::t_error() ;
   if (SlamParams::match_scans())
   {
      throw misc_error(BROKEN_FEATURE) ;

      Scan new_scan(new_pose, lrf) ;
      Transformation ut = match_scans(new_scan, m_ref_scan) ;
      m_ref_scan = new_scan ;
      /*
      LERROR("scan match: [%08lX] %10.2f %10.2f %6.1f",
             m_this, ut.Tx(), ut.Ty(), ut.w()) ;
      // */

      m_pose.dx(-ut.Tx() + sample_tri(x_noise)) ;
      m_pose.dy(-ut.Ty() + sample_tri(y_noise)) ;
      m_pose.dt(-ut.w()  + sample_tri(t_noise)) ;
   }
   else // scan matching off; therefore, produce new
   {    // state estimate using raw odometry and noise
      Pose p(new_pose) ;
      for (int i = 0; i < 10; ++i)
      {
         p.x(clamp(new_pose.x() + sample_tri(x_noise), L, R)) ;
         p.y(clamp(new_pose.y() + sample_tri(y_noise), B, T)) ;
         p.t(new_pose.t() + sample_tri(t_noise)) ;

         // Ensure that new state obtained by applying odometry + noise
         // ends up somewhere reasonable, i.e., in a map cell that is not
         // occupied by some obstacle. This ought to improve the proposal
         // distribution, especially near obstacles.
         //
         // NOTE: We could end up testing the newly sampled poses against
         // the occupancy map forever. So we test this a maximum of ten
         // times (or some other reasonable, small number).
         int x, y ;
         Coords::to_grid(p.x(), p.y(), &x, &y) ;
         if (m_map->is_vacant(x, y))
            break ;
      }
      m_pose = p ;
   }
   /*
   LERROR("final pose: [%08lX] %10.2f %10.2f %6.1f",
          m_this, m_pose.x(), m_pose.y(), m_pose.theta()) ;
   // */
}

//--------------------------- SENSOR MODEL ------------------------------

namespace {

// This function object implements the beam model described in Chapter 6
// (section 6.3) of Thrun, Burgard and Fox's "Probabilistic Robotics."
// Specifically, it implements the loop body of the algorithm presented
// in table 6.1 on page 158.
class apply_beam_model {
   const   OccGrid& m_map   ;
   const   Pose&    m_pose  ;
   const   unsigned long m_this ; // for debugging
   const   Clipper  m_clipper   ;
   const   int      m_step  ;
   int m_angle ;
   int m_obx, m_oby ;

public:
   apply_beam_model(const OccGrid&, const Pose&, unsigned long particle_addr) ;
   float operator()(float q, int z) ;

private:
   float cast_ray(float angle) ;
   float cast_ray(const float P[4]) ;
   float prob_exp(float z_expected, float z_actual) const ;

   // This is a helper function object for checking the occupancy grid's
   // cells to see if they are occupied or not. It is used in conjunction
   // with the ray casting operation to find the nearest obstacle
   // in the direction of a range measurement's beam.
   class check_occupancy {
      apply_beam_model& outer ;
   public:
      check_occupancy(apply_beam_model&) ;
      bool operator()(int x, int y) ;
   } ;

   // The above inner class will need access to the apply_beam_model
   // object's innards (so that it can set the m_obx and m_oby fields
   // when it finds an obstacle).
   friend class check_occupancy ;
} ;

apply_beam_model::
apply_beam_model(const OccGrid& m, const Pose& p, unsigned long particle_addr)
   : m_map(m), m_pose(p), m_this(particle_addr),
     m_clipper(SlamParams::map_extents()),
     m_step(SlamParams::beam_step()), m_angle(SlamParams::beam_start()),
     m_obx(-1), m_oby(-1)
{}

apply_beam_model::check_occupancy::check_occupancy(apply_beam_model& m)
   : outer(m)
{}

// This is the actual function that implements the loop body of algorithm
// beam_range_finder_model on page 158 of "Probabilistic Robotics" by
// Thrun, Burgard and Fox. The basic idea is to compute the likelihood of
// seeing the current measurement, zt, given the current state, xt, and
// the current map, mt; in other words: P(zt|xt,mt).
//
// Since each scan of a laser range finder contains several distance
// measurements, we must evaluate P(zt|xt,mt) for each of these readings.
// The final value of P(zt|xt,mt), viz., q, is obtained by multiplying
// the probabilities for the individual readings.
//
// This function computes the p term shown in table 6.1, page 158 of the
// Thrun book and then returns a "running total" of q by taking the
// product of p and the value of q so far. Thus, this function object is
// meant to be used in conjunction with the STL accumulate algorithm.
//
// NOTE: The beam model described in the Thrun book makes use of four
// components, viz., p_hit, p_short, p_max and p_rnd. Here, we only use
// p_hit, which we refer to as p_exp, and p_rnd. The basic idea here is
// to compute a reasonable number that can be used to weight a particle's
// estimate of the robot's pose and occupancy map. Therefore, we don't
// need a very detailed model; only something that is good enough to
// produce good relative weighting factors.
//
// When a particle has high weight, it means that its state estimate
// matches the current sensor data and it should be retained by the
// filter's resampling step. A particle with low weight, on the other
// hand, implies a poor state estimate and would be a candidate for
// culling from the particle population.
//
// To fulfill this weighting requirement, we simply check the current
// measurement against the particle's map and pose by computing an
// expected range reading and seeing how closely the actual measurement
// matches the expected one. The p_exp component of the probability
// computation takes care of rating the particle's state estimate based
// on the actual measurement.
//
// The p_rnd component takes care of accounting for all sorts of effects
// that cannot or need not be explicitly modeled. Combining these two
// will yield the necessary beam probability, which more than suffices
// for weighting particles. There is no need for additional complexities
// such as p_short and weighting the individual components, etc.; they
// simply don't add enough value to warrant their use.
float apply_beam_model::operator()(float q, int z)
{
   if (z < 0)    // bad distance reading from LRF
      return q ; // discard it from beam model's probability computations

   // If we have a good distance reading, then compute expected range by
   // ray casting in the map along z's direction and seeing where the
   // nearest obstacle lies.
   float z_exp = cast_ray(m_pose.theta() + m_angle) ;
   /*
   if (z_exp > 0) {
      LERROR("range:  [%08lX] {%4d} z_exp = %6.1f, z_act = %4d",
             m_this, m_angle, z_exp, z) ;
   }
   // */

   // Now, figure out how far off the mark the actual measurement is by
   // applying a Gaussian centered at the expected range. If a good
   // expected range could not be computed (because perhaps the map has
   // not yet congealed), then we set this component of the beam
   // probability to zero.
   float p_exp = (z_exp < 0) ? 0
                             : gaussian(z, z_exp, SlamParams::beam_sigma()) ;

   // The actual measurement may also be off the mark due to various
   // random effects.
   //
   // NOTE: Random effects are modeled as a uniform distribution spanning
   // the LRF measurement range. Therefore, this likelihood is a constant
   // and can be computed by and stored in the parameters structure.
   float p_rnd = SlamParams::beam_prob_rnd() ;

   // Overall likelihood of seeing current measurement given newly
   // computed state (from motion model) and most recent occupancy map
   // will be a combination of the above two probabilities.
   float p = p_exp + p_rnd ;
   /*
   if (p_exp > 0) {
      LERROR("P_exp:  [%08lX] {%4d} %10.8f", m_this, m_angle, p_exp) ;
      LERROR("P_rnd:  [%08lX] {%4d} %10.8f", m_this, m_angle, p_rnd) ;
      LERROR("P_tot:  [%08lX] {%4d} %10.8f", m_this, m_angle, p) ;
   }
   // */

   // Setup for next distance reading's ray casting...
   m_angle += m_step ;

   // Finally, combine this beam's likelihood value with that of the
   // other beams...
   //
   // NOTE: This step corresponds to the q = q.p line in table 6.1, page
   // 158 of the Thrun book. This function returns the RHS of the above
   // expression. The LHS is put together by the STL accumulate
   // algorithm, which this helper function object is passed to in
   // Particle::apply_sensor_model() defined below.
   //
   // NOTE: To work around floating point instabilities associated with
   // the tiny numbers that result from multiplying several [0,1]
   // probabilities, we apply a fudge factor to scale the individual beam
   // probabilities to > 1. The particle filter's normalization step
   // should bring the particle weights back down to [0,1].
   return q * p * SlamParams::beam_fudge() ;
}

// This function casts a ray from the robot's current position within the
// map along the specified angular direction. When it encounters an
// obstacle along the ray, i.e., an occupancy grid cell with high
// likelihood of containing some object, it returns the distance between
// that cell and the robot's current position as the expected range
// reading. The discrepancy between the expected and actual range
// readings is then used in the P(zt|xt,mt) computation.
//
// Since ray casting uses a line rasterization algorithm, we need to
// determine the ray's start and end points. As mentioned above, the
// start point is the robot's current position. To find the end point, we
// simply multiply the ray's direction vector by a large distance and
// then we clip the resulting line to the map's boundaries to ensure that
// the ray doesn't end up exploring an area outside of the map.
float apply_beam_model::cast_ray(float angle)
{
   const float D = SlamParams::max_map_distance() ;
   float ray_end_points[] = {
      m_pose.x(), m_pose.y(),
      m_pose.x() + D * cos(angle), m_pose.y() + D * sin(angle),
   } ;
   float new_end_points[4] = {0} ;
   switch (m_clipper.clip(ray_end_points, new_end_points))
   {
      case Clipper::COMPLETELY_INSIDE:
      case Clipper::SECOND_POINT_CLIPPED:
         /*
         LERROR("ray re: [%08lX] {%6.1f} (%.2f, %.2f) (%.2f, %.2f)",
                m_this, angle,
                new_end_points[0], new_end_points[1],
                new_end_points[2], new_end_points[3]) ;
         // */
         return cast_ray(new_end_points) ;

      case Clipper::COMPLETELY_OUTSIDE:   // robot outside map!?!
      case Clipper::FIRST_POINT_CLIPPED:
      case Clipper::BOTH_POINTS_CLIPPED:
      default: // Clipper has returned total crap!
         return -1 ; // discard this ray
   }
}

// The previous function's main task is to ensure that the ray casting
// operation stays within the map's bounds. The following function is the
// one that actually performs the ray casting and expected range
// computation given the ray's end points.
//
// As we walk along the ray, if we find no obstacle at all, then we
// return -1 to indicate that no good estimate could be made of the
// expected range in the given ray's direction. But if we do find an
// obstacle, then the expected range is calculated as the Euclidean
// distance between the obstacle's location and the robot's position.
float apply_beam_model::cast_ray(const float P[4])
{
   // First, convert the ray's end points from "real space" into "grid
   // space." This allows us to perform the line rasterization part of
   // ray casting entirely with integer coordinates. And, in any case, we
   // need to perform ray casting in grid coordinates because we want to
   // find the grid cells that intersect with the ray.
   int x0, y0, x1, y1 ;
   Coords::to_grid(P[0], P[1], &x0, &y0) ;
   Coords::to_grid(P[2], P[3], &x1, &y1) ;
   //LERROR("ray gr: [%08lX] (%4d, %4d) (%4d, %4d)", m_this, x0, y0, x1, y1) ;

   // Once we have the ray's end points in grid coordinates, we rasterize
   // the line joining these two points to find the grid cells
   // intersected by the range reading. The rasterizer will invoke
   // check_occupancy's operator()(int, int) method (defined below),
   // which will mark the nearest obstacle's location when it finds that
   // spot.
   //
   // Since grid coordinates go from zero to W-1 or H-1 (where W and H
   // are the width and height of the occupancy grid), we use (-1,-1) to
   // indicate the lack of an obstacle along the ray. When the rasterizer
   // is done, if the obstacle location is still (-1,-1), then there is
   // no obstacle in the ray's direction.
   m_obx = m_oby = -1 ;
   rasterize_line(x0, y0, x1, y1, check_occupancy(*this)) ;
   //LERROR("ob.loc: [%08lX] (%4d,%4d) [grid coords]", m_this, m_obx, m_oby) ;
   if (m_obx < 0 || m_oby < 0) // no obstacle along this ray
      return -1 ;

   // If the above ray casting operation did find an obstacle, we convert
   // that cell's coordinates to real/physical coordinates and then
   // return the expected range as the Euclidean distance between the
   // obstacle's location and the robot's current position.
   float ox, oy ;
   Coords::to_real(m_obx, m_oby, &ox, &oy) ;
   //LERROR("ob.loc: [%08lX] (%10.2f,%10.2f) [real coords]", m_this, ox, oy) ;
   return std::min(sqrtf(sqr(ox - P[0]) + sqr(oy - P[1])),
                   SlamParams::beam_range().max());
}

// To find the expected range reading in some direction, we cast a ray
// along that direction from the robot's current position in the map and
// look at the cells in the occupancy grid intersected by that ray. When
// the probability in a cell is above some predefined threshold, we mark
// that spot as the location of the nearest obstacle in the ray's
// direction.
//
// The ray casting operation is implemented using Bresenham's line
// rasterization algorithm, which enumerates the pixel (i.e., occupancy
// grid) positions starting at the robot's current location and ending at
// the cell where the range reading terminates. The Robolocust
// implementation of Bresenham's algorithm uses a callback function
// object for each pixel on the line joining two points.
//
// apply_beam_model::check_occupancy serves as the function object for
// the line rasterizer. In particular, the following member function is
// the one called by the rasterizer. As mentioned above, each time it is
// called, it will be passed the coordinates of the current pixel (i.e.,
// occupancy grid cell), under consideration. To do its thing, it checks
// the probability value of that cell against a preconfigured threshold
// and then marks that spot as the location of the nearest obstacle if
// the threshold condition is satisfied.
//
// Furthermore, the Robolocust implementation of Bresenham's line
// rasterization algorithm features an early exit test. Once we've found
// the location of the nearest obstacle along the ray being cast, there
// is no need to check the remaining occupancy grid cells in that ray's
// direction. Therefore, after marking the spot, we can tell the
// rasterizer to stop the ray casting.
//
// DEVNOTE: check_occupancy is an inner, friend class of
// apply_beam_model. check_occupancy::outer is a reference to the
// apply_beam_model instance that creates and passes this function object
// to the line rasterizer. We need this indirect approach because
// lobot::rasterize_line() is a template function that takes a visitor
// function for triggering its set_pixel callbacks.
//
// This callback can be a regular function pointer or a function object.
// In case of the latter, that object will be passed by value.
// Consequently, the state changes recorded by lobot::rasterize_line()'s
// local copy of the function object will not be reflected in the line
// rasterization function's caller.
//
// In this particular case, that means the obstacle location will not get
// passed back here to apply_beam_model. Therefore, we need the
// check_occupancy function object to hold a reference to the relevant
// data members of apply_beam_model. But instead of holding a reference
// to just those data members we need here, it is just simpler to
// reference the entire apply_beam_model object and use whatever of its
// members we require.
bool apply_beam_model::check_occupancy::operator()(int x, int y)
{
   /*
   float occ = outer.m_map.get(x, y) ;
   LERROR("oc.chk: [%08lX] (%4d,%4d) = %11.8f [%10.8f]",
          outer.m_this, x, y, occ, log_odds_to_prob(occ)) ;
   // */
   if (outer.m_map.is_occupied(x, y)) {
      outer.m_obx = x ;
      outer.m_oby = y ;
      return false ; // found nearest obstacle ==> stop ray casting
   }
   return true ;
}

} // end of local anonymous namespace encapsulating above helpers

// The sensor model uses the latest observation to correct the state
// prediction made by the motion model according to P(xt|zt). Since we're
// doing SLAM, the state is also contingent upon a map, i.e., the
// filter's correction step has to gauge P(xt|zt,mt).
//
// In accordance with Bayes' Rule, we evaluate the diagnostic probability
// P(xt|zt,mt) by using the more easily available causal information,
// viz., P(zt|xt,mt). To figure out P(zt|xt,mt), we apply a beam model to
// the laser range finder that computes the required probability by
// seeing how closely the actual sensor data matches the expected range
// measurements as per the particle's pose and map.
//
// The above model is described in Chapter 6, section 3 of "Probabilistic
// Robotics" by Thrun, Burgard and Fox. This function implements the
// algorithm beam_range_finder_model shown in table 6.1, page 158.
// Instead of an explicit loop as shown in the book, we use the STL
// accumulate algorithm to compute q, which is the desired causal
// probability, viz., P(zt|xt,mt). This probability is then used to
// weight each particle in the correction step of the FastSLAM particle
// filter.
//
// NOTE: When we multiply the probabilities for individual beams to get
// the final P(z|x,m) value, the number will be extremely small (e.g., in
// the range 10^-50 to 10^-80). Such numbers result in numerical
// instabilities (regardless of floating point precision). To work around
// this problem, we multiply the individual beam probabilities by a large
// constant. Consequently, the particle weight will not generally lie in
// the range [0,1]. The filter's normalization step should take care of
// bringing the particle weights back to the usual [0,1] ranage.
void Particle::apply_sensor_model(const std::vector<int>& zt)
{
   //LERROR("wt.be4: [%08lX] %14.8e (%10.8e)", m_this, m_weight, m_weight) ;
   m_weight = std::accumulate(zt.begin(), zt.end(), 1.0f,
                              apply_beam_model(*m_map, m_pose, m_this)) ;
   //LERROR("wt.aft: [%08lX] %14.8e", m_this, m_weight) ;
}

//---------------------------- MAP UPDATES ------------------------------

namespace {

// A helper function object for applying the inverse measurement model
// described in Chapter 9, section 2 of "Probabilistic Robotics" by
// Thrun, Burgard and Fox.
class apply_inverse_range_sensor_model {
   OccGrid&      m_map  ;
   const Pose&   m_pose ;
   unsigned long m_this ; // for debugging
   const    int  m_step ;
   const   float m_ray_delta, m_ray_diagonal_delta ;
   int   m_angle ;
   int   m_prev_x, m_prev_y ;
   float m_range_reading, m_ray_range ;

public:
   apply_inverse_range_sensor_model(OccGrid&, const Pose&, unsigned long) ;
   void operator()(int z) ;

private:
   // This is a helper function object for updating the occupancy grid's
   // cells. It is used in conjunction with the ray casting operation to
   // mark the cells at the end of a range measurement's beam as occupied
   // and the intermediate ones as vacant.
   class update_occupancy {
      apply_inverse_range_sensor_model& outer ;
   public:
      update_occupancy(apply_inverse_range_sensor_model&) ;
      bool operator()(int x, int y);
   } ;

   // The above inner class will need access to the
   // apply_inverse_range_sensor_model object's innards (so that it can
   // update the occupancy grid and track/set other relevant data members
   // during the ray casting).
   friend class update_occupancy ;
} ;

apply_inverse_range_sensor_model::
apply_inverse_range_sensor_model(OccGrid& M, const Pose& P, unsigned long addr)
   : m_map(M), m_pose(P), m_this(addr),
     m_step(SlamParams::beam_step()),
     m_ray_delta(SlamParams::map_cell_size()),
     m_ray_diagonal_delta(1.41421356f /*sqrt(2)*/ * m_ray_delta),
     m_angle(SlamParams::beam_start()),
     m_prev_x(-1), m_prev_y(-1),
     m_range_reading(-1), m_ray_range(-1)
{}

apply_inverse_range_sensor_model::
update_occupancy::update_occupancy(apply_inverse_range_sensor_model& m)
   : outer(m)
{}

// This function implements a variation of algorithm
// occupancy_grid_mapping shown in table 9.1, page 286 of the Thrun book.
// Instead of looping over all the cells in the occupancy grid and
// checking if they are in the perceptual field of the laser range
// finder's range readings, we take each reading one at a time and cast a
// ray along its direction. All the cells that intersect this ray will be
// in the perceptual field of that particular reading; the others will
// not.
void apply_inverse_range_sensor_model::operator()(int z)
{
   if (z < 0) // bad LRF reading
      return ;

   // Setup ray casting by converting from physical to grid coordinates
   // the robot's current position, which is the ray's starting point,
   // and the end point of the range measurement, the ray's end point.
   int x0, y0, x1, y1 ;
   Coords::to_grid(m_pose.x(), m_pose.y(), &x0, &y0) ;
   Coords::to_grid(m_pose.x() + z * cos(m_pose.theta() + m_angle),
                   m_pose.y() + z * sin(m_pose.theta() + m_angle),
                   &x1, &y1) ;
   /*
   LERROR("range:    [%08lX] {%4d} %4d", m_this, m_angle, z) ;
   LERROR("ray grid: [%08lX] {%4d} (%4d,%4d) (%4d,%4d)",
          m_this, m_angle, x0, y0, x1, y1) ;
   // */

   // Ray casting will be performed by the line rasterizer, which will
   // invoke the function call operator for the update_occupancy object.
   // That function will be passed the coordinates of each intersecting
   // cell as the ray progresses from its start point to its end point.
   // However, in order to properly update a cell, the line rasterizer
   // callback function will need to know the current range reading, the
   // distance between the current cell and the robot position, etc.
   //
   // Therefore, before triggering ray casting, we setup the parameters
   // the ray casting callback will need for its map update operation.
   m_range_reading = z ;
   m_ray_range     = 0 ;
   m_prev_x = x0 - sign0(x1 - x0) ;
   m_prev_y = y0 - sign0(y1 - y0) ;

   // Now, we're ready to cast the ray along the current reading's
   // direction starting at the robot's current position...
   rasterize_line(x0, y0, x1, y1, update_occupancy(*this)) ;

   // Setup for the next range reading...
   m_angle += m_step ;
}

// This is the callback function for the line rasterizer. It implements a
// variation of algorithm inverse_range_sensor_model shown in table 9.2,
// page 288 of the Thrun book. The basic idea here is to mark the cells
// near the end of the ray as occupied and all the cells in between as
// free space.
bool
apply_inverse_range_sensor_model::update_occupancy::
operator()(int x, int y)
{
   const int W = SlamParams::map_width() ;
   const int H = SlamParams::map_height();
   if (x < 0 || x >= W || y < 0 || y >= H)
      return false ; // crossed map boundaries ==> quit ray casting
   /*
   float occ = outer.m_map.get(x, y) ;
   LERROR("current:  [%08lX] (%4d,%4d) p[%11.8f %10.8f] r = %10.2f",
          outer.m_this, x, y, occ, log_odds_to_prob(occ),
          outer.m_ray_range) ;
   // */

   // m_ray_range holds the distance, in the real/world coordinate
   // system, between the current cell (x,y) and the robot's position. If
   // the difference between this cell's distance and the range reading
   // is less than 10% of the range reading's value, then cell (x,y) must
   // be near the end of the ray. Therefore, we mark it as occupied.
   // Otherwise, the cell must be one of the intermediate ones; so we
   // mark it as free space.
   const float threshold = SlamParams::beam_occ_range_error() ;
   if (abs(outer.m_range_reading - outer.m_ray_range) < threshold) {
      outer.m_map.occupied(x, y) ;
      /*
      float occ = outer.m_map.get(x, y) ;
      LERROR("occupied: [%08lX] (%4d,%4d) p[%11.8f %10.8f] r = %10.2f",
             outer.m_this, x, y, occ, log_odds_to_prob(occ),
             outer.m_ray_range) ;
      // */
      //return false ; // reached an obstacle ==> mark it and quit ray casting
   }
   else if (outer.m_ray_range < outer.m_range_reading) { // double check
      outer.m_map.vacant(x, y) ;
      /*
      float occ = outer.m_map.get(x, y) ;
      LERROR("vacant:   [%08lX] (%4d,%4d) p[%11.8f %10.8f] r = %10.2f",
             outer.m_this, x, y, occ, log_odds_to_prob(occ),
             outer.m_ray_range) ;
      // */
   }

   // In the Thrun book, they compute r in the algorithm on page 288 for
   // all the cells that lie in the measurement's perceptual field. In
   // this implementation, we simply use m_ray_range as r and keep track
   // of it as we move along the ray.
   //
   // In each iteration, the line rasterizer will move one pixel from the
   // previous/current pixel to the next one. Since we know the size of a
   // cell in the occupancy grid, we can keep track of r by incrementing
   // m_ray_range by the cell size. The only catch is that for diagonal
   // moves, the increment amount will be sqrt(2) times the cell size
   // (Pythagoras).
   switch (abs(x - outer.m_prev_x) + abs(y - outer.m_prev_y))
   {
      case 0:  // ray has not moved (will happen in the first iteration)
         break ;
      case 1:  // ray has moved vertically or horizontally
         outer.m_ray_range += outer.m_ray_delta ;
         break ;
      case 2:  // ray has moved diagonally
         outer.m_ray_range += outer.m_ray_diagonal_delta ;
         break ;
      default: // line rasterizer moved more than one pixel in one step?
         throw misc_error(LOGIC_ERROR) ;
         break ;
   }

   // Setup for next cell in ray's path and continue ray casting...
   outer.m_prev_x = x ;
   outer.m_prev_y = y ;
   return true ;
}

} // end of local anonymous namespace encapsulating above helpers

// Bayesian state estimation consists of a prediction step, wherein we
// apply the control input to figure out a new state, and a correction
// step, wherein we use the latest sensor data to correct the state
// prediction made in the previous step.
//
// In the SLAM problem, the state we're trying to estimate is the robot's
// pose as well as the map of its surroundings. Thus, the correction
// step requires us to correct the latest pose predicted by the action
// model and also update the map using the corrected pose.
//
// This method implements the map correction step of the FastSLAM
// algorithm. It is a variation of the occupancy_grid_mapping procedure
// presented in table 9.1, page 286 of "Probabilistic Robotics" by Thrun,
// Burgard and Fox. Rather than looping over all the cells in the
// occupancy grid and applying the inverse sensor model to those cells
// that lie in the perceptual field of the measurement, we loop over the
// individual range readings making up the latest LRF scan and use ray
// casting to find the cells that lie in the perceptual fields of each of
// the readings and apply the inverse sensor model to those cells.
void Particle::update_map(const std::vector<int>& zt)
{
   std::for_each(zt.begin(), zt.end(),
                 apply_inverse_range_sensor_model(*m_map, m_pose, m_this)) ;
}

//----------------------- VISUALIZATION SUPPORT -------------------------

Particle::Viz::Viz(const Pose& p, float w)
   : pose(p), weight(w)
{}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

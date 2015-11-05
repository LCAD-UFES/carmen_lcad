/**
   \file  Robots/LoBot/misc/LoFastSLAM.C
   \brief This file implements the non-inline member functions of the
   lobot::FastSLAM class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/slam/LoFastSLAM.C $
// $Id: LoFastSLAM.C 13575 2010-06-17 01:42:18Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/slam/LoFastSLAM.H"
#include "Robots/LoBot/slam/LoMap.H"
#include "Robots/LoBot/slam/LoSlamParams.H"

#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/util/LoStats.H"
#include "Robots/LoBot/util/LoSTL.H"
#include "Robots/LoBot/util/LoString.H"
#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/util/range.hh"
#include "Robots/LoBot/util/triple.hh"
#include "Robots/LoBot/misc/singleton.hh"

// INVT utilities
#include "Util/log.H"

// Boost headers
#include <boost/bind.hpp>

// Standard C++ headers
#include <string>
#include <numeric>
#include <algorithm>
#include <functional>
#include <iterator>
#include <utility>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

// Generator function object to create a new particle
namespace {

class new_particle {
   float initial_weight ;
   std::vector<int> initial_scan ;
   boost::shared_ptr<OccGrid> known_map ;
public:
   new_particle(float initial_weight, const LRFData& initial_scan) ;
   new_particle(float, const LRFData&, const boost::shared_ptr<OccGrid>&) ;
   Particle operator()() const ;
} ;

new_particle::new_particle(float w, const LRFData& lrf)
   : initial_weight(w), initial_scan(lrf.distances())
{}

new_particle::
new_particle(float w, const LRFData& lrf, const boost::shared_ptr<OccGrid>& m)
   : initial_weight(w), initial_scan(lrf.distances()), known_map(m)
{}

Particle new_particle::operator()() const
{
   if (known_map)
      return Particle(initial_weight, initial_scan, known_map) ;
   else
      return Particle(initial_weight, initial_scan) ;
}

} // end of local anonymous namespace encapsulating above helper

// Since this implementation of FastSLAM uses laser range scan matching
// to reduce the number of particles required by the algorithm, it needs
// to be passed an initial scan of range readings during initialization.
FastSLAM::FastSLAM(const LRFData& lrf)
   : m_w_slow(0), m_w_fast(0)
{
   const int N = SlamParams::num_particles() ;
   m_particles.reserve(N) ;
   std::generate_n(std::back_inserter(m_particles), N,
                   new_particle(1.0f/N, lrf)) ;
}

FastSLAM::
FastSLAM(const LRFData& lrf, const boost::shared_ptr<OccGrid>& known_map)
   : m_w_slow(0), m_w_fast(0)
{
   const int N = SlamParams::num_particles() ;
   m_particles.reserve(N) ;
   std::generate_n(std::back_inserter(m_particles), N,
                   new_particle(1.0f/N, lrf, known_map)) ;
}

//------------------------ FASTSLAM ALGORITHM ---------------------------

namespace {

// Helper function object to update a particle's state using the current
// control and data inputs.
class update_particle {
   std::vector<int> lrf;
   const Odometry&  ut ; // control input at (current) time step t
   std::vector<int> zt ; // sensor  data  at (current) time step t
   mutable int n ;       // particle number (for debugging)
   const bool slam_mode ;// are we doing MCL only or full SLAM?
public:
   update_particle(const Odometry&, const LRFData&) ;
   void operator()(Particle&) const ;
} ;

update_particle::update_particle(const Odometry& odometry, const LRFData& L)
   : lrf(L.distances()), ut(odometry), n(0), slam_mode(SlamParams::slam_mode())
{
   const int begin = SlamParams::beam_start() ;
   const int end   = SlamParams::beam_end()   ;
   const int step  = SlamParams::beam_step()  ;

   const int N = (end - begin)/step + 1 ;
   zt.reserve(N) ;
   for (int angle = begin; angle <= end; angle += step)
      zt.push_back(L[angle]) ;
}

void update_particle::operator()(Particle& P) const
{
   //LERROR("--------------- Particle #%-4d ---------------", n++) ;
   P.apply_motion_model(ut, lrf) ; // full LRF scan for scan matching
   P.apply_sensor_model(zt) ;      // limited scan for filter's
   if (slam_mode)                  // sensor-based correction steps
      P.update_map(zt) ;
}

// Divide each particle's weight by the sum of all their weights
void normalize_weights(std::vector<Particle>& particles)
{
   accumulator<float> sum =
      std::transform(particles.begin(), particles.end(), accumulator<float>(0),
                     std::mem_fun_ref(&Particle::weight)) ;
   std::for_each(particles.begin(), particles.end(),
                 std::bind2nd(std::mem_fun_ref(&Particle::normalize),
                              1/sum.value())) ;

   //LERROR("w_sum = %12.8f (%14.8e)", sum.value(), sum.value()) ;
   /*
   const int N = particles.size() ;
   for  (int i = 0; i < N; ++i) {
       float w = particles[i].weight() ;
       LERROR("particle[%4d].weight = %12.8f (%14.8e)", i, w, w) ;
   }
   // */
}

// This function computes the effective sample size of the particle
// population by taking the reciprocal of the sum of the squares of the
// normalized individual particle weights.
int effective_sample_size(const std::vector<Particle>& particles)
{
   float sum =
      std::accumulate(particles.begin(), particles.end(), 0.0f,
                      boost::bind(sum_of_squares<float>(), _1,
                                  boost::bind(&Particle::weight, _2))) ;
   int N = round(1/sum) ;
   //LERROR("w_sum_sqr = %12.8f, Neff = %d", sum, N) ;
   return N ;
}

} // end of local anonymous namespace encapsulating above helpers

// This function implements the FastSLAM algorithm's particle update and
// resampling steps. Resampling is carried out only when the effective
// number of particles falls below a certain threshold. This strategy is
// described in many different papers on particle filters. See, for
// example, "Improving Grid-based SLAM with Rao-Blackwellized Particle
// Filters by Adaptive Proposals and Selective Resampling" by Grisetti,
// Stachniss and Burgard, ICRA 2005.
//
// NOTE: The effective sample size check can be turned off by configuring
// the ESS threshold to be a negative number or to a number greater than
// the number of particles. If the user has switched off the ESS check,
// then resampling will occur after each and every FastSLAM update.
void FastSLAM::update(const Odometry& ut, const LRFData& zt)
{
   // First, update all particles using latest control and sensor inputs
   std::for_each(m_particles.begin(), m_particles.end(),
                 update_particle(ut, zt)) ;

   // Then, normalize the particle weights and resample particle
   // population based on their weights.
   //LERROR("=============== Resampling...  ===============") ;
   normalize_weights(m_particles) ;
   update_averages() ;

   const int N = m_particles.size() ;
   const int T = SlamParams::ess_threshold() ;
   if (T <= 0 || T > N || effective_sample_size(m_particles) < T)
   {
      // Low-variance resampling; see "Probabilistic Robotics" by Thrun,
      // Burgard and Fox (pg. 110).
      Particles particles ;
      particles.reserve(N) ;

      float n = 1.0f/N ;
      float r = randomf(0, n) ;
      float c = m_particles[0].weight() ;
      float w = std::max(0.0f, 1 - m_w_fast/m_w_slow);
      int   i = 0 ;
      //LERROR("n = %12.8f, r = %12.8f, w = %12.8f", n, r, w) ;

      float U = r ;
      for (int j = 0; j < N; ++j, U += n)
      {
         if (randomf(0, 1) < w)
         {
            //LERROR("randomizing particle #%-4d", i) ;
            Particle tmp = m_particles[i] ;
            tmp.randomize() ;
            particles.push_back(tmp) ;
         }
         else
         {
            //LERROR("U = %12.8f, c = %12.8f", U, c) ;
            while (U > c) {
               ++i ;
               c += m_particles[i].weight() ;
               //LERROR("c = %12.8f, i = %4d", c, i) ;
            }
            //LERROR("U = %12.8f, c = %12.8f, i = %4d", U, c, i) ;
            /*
            LERROR("particle[%4d] =  %-4d (w: %12.8f)",
                   j, i, m_particles[i].weight()) ;
            // */
            particles.push_back(m_particles[i]) ;
         }
      }

      m_particles = particles ;
   }
}

// This method updates the short and long-term particle weight averages,
// which are used in the resampling step to determine when to insert
// random particles to help deal with mislocalizations and also the
// overall number of random particles that will be inserted.
void FastSLAM::update_averages()
{
   float w_avg = mean<float>(m_particles.begin(), m_particles.end(),
                             std::mem_fun_ref(& Particle::weight)) ;
   m_w_slow += SlamParams::alpha_slow() * (w_avg - m_w_slow) ;
   m_w_fast += SlamParams::alpha_fast() * (w_avg - m_w_fast) ;
}

//-------------------- PARTICLE DENSITY EXTRACTION ----------------------

namespace {

// This helper class implements a function object that can be used to
// compute the overall error in the relevant state variables between the
// particle with maximum weight and any other particle.
//
// The type parameters for this template are:
//
//    1. T -- the state variable for which we are interested in the error
//    2. C -- a "converter" that returns a T given a lobot::Particle
//    3. E -- a callable that makes the actual error determination
//
// Since this class is used in conjunction with Robolocust's
// implementation of FastSLAM, the relevant state variables we're
// interested in are either lobot::OccGrid or lobot::Pose. Thus, T would
// be one of these two classes.
//
// This function object is intended to be used in conjunction with an STL
// algorithm that iterates over a container of Particles. For each
// particle, it will return an estimate of the error in the state of that
// particle against the state of the particle with maximum weight. When
// this function object is created, it will be passed the state
// corresponding to the Particle with maximum weight. In order to
// retrieve the state for each Particle in the sequence being iterated
// over, the object will use the Particle-to-T converter (type C).
//
// Since the lobot::Particle class provides member functions to retrieve
// that Particle's Pose and OccGrid map, type C will usually be an
// adapter function object that calls the relevant member function of the
// Particle class to "convert" the Particle to the state type T. An
// example of such an adapter would be a function object returned by
// std::mem_fun_ref.
//
// The E function will be passed two state variables: the first
// corresponding to the Particle with maximum weight and the second
// corresponding to the "current" Particle in the sequence being iterated
// over. This function is expected to return a floating point number that
// somehow measures the error between the current Particle's state
// variable and the state of the Particle with maximum weight.
//
// As mentioned above, this function object is intended to be used in
// conjunction with an STL algorithm that iterates over a container of
// Particles. For each Particle in the sequence, it will return an
// estimate of the error in the state of that Particle against the state
// of the Particle with maximum weight. In addition to this error value,
// it will also return the index of the Particle for which it just
// performed the error evaluation. These two items are returned via an
// STL pair, error value first and particle index second.
template<typename T, typename C, typename E = float (*)(const T&, const T&)>
class compute_error {
   const T& max ; // state corresponding to Particle with max weight
   const C& p2t ; // Particle-to-T "converter"
   const E& err ; // function to evaluate error between max & other Particles
   mutable int i; // index of current Particle in the Particle list
public:
   compute_error(const T&, const C&, const E&) ;
   std::pair<float, int> operator()(const Particle&) const ;
} ;

// When the compute_error function object is created, it should be passed
// the state of the particle with maximum weight, a function or function
// object that retrieves/converts a Particle to a state variable (type T)
// and a callable that can be used to estimate the error for this state
// variable.
template<typename T, typename C, typename E>
compute_error<T,C,E>::compute_error(const T& t, const C& c, const E& e)
   : max(t), p2t(c), err(e), i(0)
{}

// As mentioned earlier, the compute_error function object is meant to be
// used with a container of Particles. Each time this function object is
// called, it will evaluate the error in the state of the "current"
// Particle against that of the Particle with maximum weight using the
// error function it was given when it was created.
template<typename T, typename C, typename E>
std::pair<float, int>
compute_error<T,C,E>::
operator()(const Particle& P) const
{
   return std::make_pair(err(max, p2t(P)), i++) ;
}

// This function evaluates the error in the occupancy grids of two
// particles. To measure the amount of error between two occupancy grids,
// we perform a pixel/cell-wise comparison and return the overall sum of
// the squares of the differences between the individual cells.
float map_error(const OccGrid& m1, const OccGrid& m2)
{
   if (&m1 == &m2) // same map ==> error will be zero
      return 0 ;   // so don't waste time computing the obvious

   using boost::bind ;
   accumulator<float> acc =
      std::transform(m1.begin(), m1.end(), m2.begin(), accumulator<float>(0),
                     bind(sqr<float>, bind(std::minus<float>(), _1, _2))) ;
   return acc.value() ; // sum of square errors
}

// This function evaluates the error in the poses of two Particles. The
// total error in the pose of two particles is evaluated as simply the
// sum of the squares of the differences between the individual pose
// variables (x, y, theta).
float pose_error(const Pose& p1, const Pose& p2)
{
   // NOTE: Cannot apply (&p1 == &p2) check here to bypass useless zero
   // computation (as in map_error above) because Particle::pose()
   // returns a copy of its Pose object whereas Particle::map() returns a
   // reference to its OccGrid object. Therefore, the addresses of this
   // function's two parameters will always be different.
   //
   // DEVNOTE: We could change Particle::pose() so that it too returns a
   // reference to its Pose object. However, the error computation for
   // Pose objects is much less strenuous than the error computation for
   // OccGrid objects. So the object address check for bypassing the
   // error computation here is not as imperative as for the map case.
   return sqr(p2.x() - p1.x())
        + sqr(p2.y() - p1.y())
        + sqr(p2.t() - p1.t()) ;
}

// This helper function returns a function object for computing the map
// error between the given map and the map associated with any other
// Particle. It is meant to be used for the computation of the particle
// filter's current best hypothesis regarding the occupancy grid map of
// the robot's surroundings. Therefore, the map object passed to this
// function should be the map of the Particle with maximum weight.
//
// Additionally, this function should be passed a "converter" (type C)
// that can return a lobot::OccGrid given a lobot::Particle. Usually,
// this would be a member function adapter such as the thing returned by
// std::mem_fun_ref or boost::bind.
//
// With these two parameters in hand, this function will instantiate the
// compute_error function object with the appropriate types and arguments
// and return this object properly setup to perform the map error
// computations.
template<typename C>
compute_error<OccGrid, C>
error_function(const OccGrid& max_map, const C& particle_to_occgrid)
{
   return compute_error<OccGrid, C>(max_map, particle_to_occgrid, map_error) ;
}

// This helper function returns a function object for computing the pose
// error between the given Pose and the Pose associated with any other
// Particle. It is meant to be used for the computation of the particle
// filter's current best hypothesis regarding the robot's pose.
// Therefore, the Pose object passed to this function should be the Pose
// of the Particle with maximum weight.
//
// Additionally, this function should be passed a "converter" (type C)
// that can return a lobot::Pose given a lobot::Particle. Usually, this
// would be a member function adapter such as the thing returned by
// std::mem_fun_ref or boost::bind.
//
// With these two parameters in hand, this function will instantiate the
// compute_error function object with the appropriate types and arguments
// and return this object properly setup to perform the pose error
// computations.
template<typename C>
compute_error<Pose, C>
error_function(const Pose& max_pose, const C& particle_to_pose)
{
   return compute_error<Pose, C>(max_pose, particle_to_pose, pose_error) ;
}

// To construct the particle filter's current best hypothesis regarding
// the robot's state (either the occupancy map or the robot pose), we use
// a robust mean computed thusly:
//
//    1. Find particle with max weight
//    2. Pick K particles whose maps/poses most closely match above particle
//    3. Return average of above K maps/poses
//
// Since the procedure for both occupancy maps and poses is the same, we
// use a template function to implement it. This function is parametrized
// on the following types:
//
//    1. T -- the type of state extracted from the particle population
//    2. C -- a "converter" function for returning a T from a Particle
//
// Since this function is used in conjunction with FastSLAM, the state
// we're concerned with is either the occupancy grid map of the robot's
// surroundings or the robot's pose. Thus, T will be either
// lobot::OccGrid or lobot::Pose.
//
// The type C will be a function or function object that takes a Particle
// and returns a T (OccGrid or Pose) from that Particle. Since the
// lobot::Particle class provides member functions for returning that
// particle's hypothesis regarding the map and pose, type C will usually
// be a member function adapter (e.g., returned by std::mem_fun_ref).
//
// The parameters to this function are:
//
//    1. particles -- the particle filter's current list of particles
//    2. p2t -- a function/object that takes a Particle and returns a T
//
// DEVNOTE: Since this function's argument list does not contain a T,
// which only figures in its return type, automatic type deduction will
// not work (as return types don't count when determining a function's
// signature).
template<typename T, typename C>
T current_best_hypothesis(const std::vector<Particle>& particles, C p2t)
{
   // Find particle with max weight
   std::vector<Particle>::const_iterator max =
      std::max_element(particles.begin(), particles.end(),
                       boost::bind(std::less<float>(),
                                   boost::bind(&Particle::weight, _1),
                                   boost::bind(&Particle::weight, _2))) ;

   // Special case: don't bother with finding top K matches to above
   // particle when K is one. If K is one, then the particle with max
   // weight is the thing we're interested in.
   const int K = SlamParams::num_matches() ;
   if (K == 1)
      return p2t(*max) ;

   // Compute overall errors between the relevant state variable (T) of
   // all other particles and the corresponding state variable of the
   // particle with max weight. These errors are stored along with their
   // corresponding particle indices.
   typedef std::pair<float, int> ErrIndPair ;
   std::vector<ErrIndPair> E ;
   E.reserve(particles.size()) ;
   std::transform(particles.begin(), particles.end(),
                  std::back_inserter(E), error_function(p2t(*max), p2t)) ;

   // Partially sort the errors so that the top K particles whose states
   // are closest to the state of the particle with max weight are at the
   // front of the error-index pair vector.
   std::partial_sort(E.begin(), E.begin() + K, E.end(),
                     boost::bind(std::less<float>(),
                                 boost::bind(get_first<ErrIndPair>, _1),
                                 boost::bind(get_first<ErrIndPair>, _2))) ;

   // Compute final state as average of states of top K particles closest
   // to particle with max weight.
   T t ;
   for (int i = 0; i < K; ++i)
      t += p2t(particles[E[i].second]) ;
   return t * (1.0f/K) ;
}

} // end of local anonymous namespace encapsulating above helpers

// This method computes the particle filter's current best hypothesis
// regarding the occupancy grid map of the robot's surroundings using a
// robust mean, i.e., a mean of the maps of the K particles whose maps
// most closesly resemble the map built by the particle with maximum
// weight.
OccGrid FastSLAM::current_map() const
{
   return current_best_hypothesis<OccGrid>(m_particles,
                                           std::mem_fun_ref(&Particle::map)) ;
}

// This method computes the particle filter's current best hypothesis
// regarding the robot's pose using a robust mean, i.e., a mean of the
// poses of the K particles whose poses most closesly match the pose
// estimated by the particle with maximum weight.
Pose FastSLAM::current_pose() const
{
   return current_best_hypothesis<Pose>(m_particles,
                                        std::mem_fun_ref(&Particle::pose)) ;
}

//----------------------- VISUALIZATION SUPPORT -------------------------

std::vector<Particle::Viz> FastSLAM::viz() const
{
   std::vector<Particle::Viz> v ;
   v.reserve(m_particles.size()) ;
   std::transform(m_particles.begin(), m_particles.end(),
                  std::back_inserter(v), std::mem_fun_ref(&Particle::viz)) ;
   return v ;
}

//----------------------------- CLEAN-UP --------------------------------

FastSLAM::~FastSLAM(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

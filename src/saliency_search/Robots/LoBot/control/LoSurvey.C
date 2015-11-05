/**
   \file  Robots/LoBot/control/LoSurvey.C
   \brief This file defines the non-inline member functions of the
   lobot::Survey class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoSurvey.C $
// $Id: LoSurvey.C 13782 2010-08-12 18:21:14Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoSurvey.H"

#include "Robots/LoBot/LoApp.H"

#include "Robots/LoBot/slam/LoMap.H"
#include "Robots/LoBot/slam/LoSlamParams.H"

#include "Robots/LoBot/io/LoLRFData.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"

#include "Robots/LoBot/thread/LoUpdateLock.H"
#include "Robots/LoBot/thread/LoShutdown.H"

#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/LoRegistry.H"
#include "Robots/LoBot/util/LoMath.H"

// INVT utilities
#include "Util/log.H"

// OpenGL headers
#ifdef INVT_HAVE_LIBGLU
#include <GL/glu.h>
#endif

#ifdef INVT_HAVE_LIBGL
#include <GL/gl.h>
#endif

// Boost headers
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

// Standard C++ headers
#include <string>
#include <algorithm>
#include <vector>
#include <utility>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- KNOB TWIDDLING -----------------------------

namespace {

// Retrieve settings from survey section of config file
template<typename T>
static inline T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>(LOBE_SURVEY, key, default_value) ;
}

// Overload for retrieving STL pairs
template<typename T>
static inline std::pair<T, T>
conf(const std::string& key, const std::pair<T, T>& default_value)
{
   return get_conf<T>(LOBE_SURVEY, key, default_value) ;
}

/// This local class encapsulates various parameters that can be used to
/// tweak different aspects of the survey behaviour.
class Params : public singleton<Params> {
   /// Since SLAM updates can be quite intense, the survey behaviour
   /// waits for the robot to move or turn by at least some minimum
   /// amount before triggering the SLAM algorithm. This setting
   /// specifies the above-mentioned thresholds. The distance threshold
   /// is in mm and the angle threshold in degrees.
   ///
   /// NOTE: This setting expects its value to be two integers. The first
   /// number is used for the distance threshold and the second for the
   /// angle threshold. Set these values to -1 to turn the thresholds
   /// off.
   std::pair<int, int> m_thresholds ;

   /// Sometimes the low-level controller goes berserk and sends
   /// nonsensical odometry packets. This setting specifies the maximum
   /// acceptable values for the distance and rotation sent by the
   /// low-level. Values greater than these thresholds will be reported
   /// on stderr but ignored for SLAM updates.
   std::pair<int, int> m_odometry_max ;

   /// The number of milliseconds between successive iterations of this
   /// behaviour.
   ///
   /// WARNING: The ability to change a behaviour's update frequency is a
   /// very powerful feature whose misuse or abuse can wreak havoc! Be
   /// sure to use reasonable values for this setting.
   int m_update_delay ;

   /// Does the user want to show particle bearings as well as the
   /// particle positions? By default, the visualization only shows the
   /// particle positions.
   bool m_show_bearings ;

   /// Bearing scale factors: these "parameters" are scale factors used
   /// to convert the computer screen's pixel coordinates into the map's
   /// real/physical coordinates. These scale factors are used to show
   /// particle bearings in a fixed screen pixel size but in the map's
   /// coordinate system.
   std::pair<float, float> m_bearing_scale ;

   /// Private constructor because this is a singleton.
   Params() ;
   friend class singleton<Params> ;

public:
   /// Accessing the various parameters
   //@{
   static const std::pair<int, int>& thresholds() {
      return instance().m_thresholds ;
   }
   static int   max_distance()  {return instance().m_odometry_max.first  ;}
   static int   max_rotation()  {return instance().m_odometry_max.second ;}
   static int   update_delay()  {return instance().m_update_delay  ;}
   static bool  show_bearings() {return instance().m_show_bearings ;}
   static float Bx() {return instance().m_bearing_scale.first  ;}
   static float By() {return instance().m_bearing_scale.second ;}
   //@}

   /// Clean-up
   ~Params() ;
} ;

// Parameter initialization
Params::Params()
   : m_thresholds(conf("thresholds", std::make_pair(100, 5))),
     m_odometry_max(conf("odometry_max", std::make_pair(200, 30))),
     m_update_delay(clamp(conf("update_delay", 500), 250, 2500)),
     m_show_bearings(conf("show_bearings", false))
{
   m_thresholds.first  =
      (m_thresholds.first  < 0) ? -1 : clamp(m_thresholds.first,  10, 250) ;
   m_thresholds.second =
      (m_thresholds.second < 0) ? -1 : clamp(m_thresholds.second,  1,  30) ;

   m_odometry_max.first  = clamp(m_odometry_max.first,  100, 250) ;
   m_odometry_max.second = clamp(m_odometry_max.second,   5,  45) ;

   // Compute scale factors to show particle bearings as lines exactly 10
   // pixels long.
   float L, R, B, T ;
   SlamParams::map_extents(&L, &R, &B, &T) ;
   Drawable::Geometry G(SlamParams::map_geometry());
   m_bearing_scale.first  = 15 * (R - L)/G.width   ;
   m_bearing_scale.second = 15 * (T - B)/G.height  ;
}

// Parameter clean-up
Params::~Params(){}

} // end of local anonymous namespace encapsulating above helpers

//-------------------------- INITIALIZATION -----------------------------

Survey::Survey()
   : base(Params::update_delay(), LOBE_SURVEY, Drawable::Geometry()),
     m_slam(0), m_slam_busy(false)
{
   m_odometry.set_thresholds(Params::thresholds()) ;
   start(LOBE_SURVEY) ;
}

// The odometry_helper function object is used to signal the survey
// behaviour's thread that it should trigger the SLAM algorithm with the
// latest odometry.
Survey::odometry_helper::odometry_helper(int dist, int ang)
   : distance(dist), angle(ang)
{}

// Before we begin regular action processing for the survey behaviour, we
// should first ensure that all the necessary objects have been properly
// setup. We also need to setup the low-level hook for getting odometry
// updates, take care of initializing the SLAM algorithm and perform any
// other required initialization that cannot/should not be done in the
// constructor.
void Survey::pre_run()
{
   const LaserRangeFinder* lrf = App::lrf() ;
   if (! lrf)
      throw behavior_error(LASER_RANGE_FINDER_MISSING) ;

   Robot* robot = App::robot() ;
   if (! robot)
      throw behavior_error(MOTOR_SYSTEM_MISSING) ;

   Map* map = App::map() ;
   if (! map)
      throw behavior_error(MAPPING_DISABLED) ;

   robot->add_hook(Robot::SensorHook(
      sensor_hook, reinterpret_cast<unsigned long>(this))) ;

   UpdateLock::begin_read() ;
      LRFData scan(lrf) ;
   UpdateLock::end_read() ;

   const std::string map_file = SlamParams::map_file() ;
   if (map_file.empty()) // SLAM should perform both localization and mapping
      m_slam.reset(new FastSLAM(scan)) ;
   else { // SLAM will be given a known map and need only perform localization
      OccGrid* known_map = new OccGrid(map_file) ;
      map->update(*known_map) ;
      m_slam.reset(new FastSLAM(scan, boost::shared_ptr<OccGrid>(known_map))) ;
   }

   if (visualize(LOBE_SURVEY))
      map->add_hook(RenderHook(
         render_particles, reinterpret_cast<unsigned long>(this))) ;
}

//---------------------- THE BEHAVIOUR'S ACTION -------------------------

// This function implements a custom "main loop" for the survey
// behaviour. Instead of relying on a sleep to decide when to go in for
// the behaviour's next iteration (as is the case with the default main
// loop for behaviours), it uses a condition variable and the signal/wait
// mechanism to go in for its next iteration.
void Survey::run()
{
   try
   {
      App::wait_for_init() ;

      // Main loop
      pre_run() ;
      while (! Shutdown::signaled())
      {
         if (m_odometry_cond.wait(threshold_check(), Params::update_delay())) {
            action() ;
            m_odometry_cond.protect(reset_slam_busy_flag()) ;
         }
      }
      post_run() ;
   }
   catch (uhoh& e)
   {
      LERROR("behaviour %s encountered an error: %s", LOBE_SURVEY, e.what()) ;
      return ;
   }
}

// When the main thread updates the low-level sensor state, this hook
// function will be triggered. The survey behaviour uses the latest
// odometry data to track the cumulative pose change since the previous
// invocation of this hook. When the cumulative pose change reaches some
// predefined limit, the behaviour will trigger the next update of its
// SLAM algorithm as long as the SLAM module is not busy working on the
// previous update.
//
// DEVNOTE: If the low-level odometry packet smells bad, this function
// will throw an exception. Since this hook is executing in the context
// of the main thread, having been called via Robot::update(), the
// exception will result in the application quitting. This is not an
// inappropriate action. If the low-level is truly berserk and sending
// bogus odometry packets, then the high-level cannot function properly
// and the user should fix the low-level problem (e.g., reboot robot) and
// restart the application. If the low-level is okay, then the configured
// odometry maximums are probably wrong. In that case too, the best thing
// to do is to quit the application and have the user fix the config
// before retrying.
void
Survey::
sensor_hook(const Robot::Sensors& sensors, unsigned long client_data)
{
   int distance = sensors.distance() ;
   int rotation = sensors.angle() ;
   /*
   LERROR("raw odometry= [%4d %4d] @ %lld",
          distance, rotation, sensors.time_stamp()) ;
   // */
   const int D = Params::max_distance() ;
   const int R = Params::max_rotation() ;
   if (abs(distance) > D || abs(rotation) > R) // bad low-level odometry?
   {
      // When the low-level controller responds to the Roomba's bump or
      // cliff sensors and also when it responds to the SPIN command sent
      // by the high level, it can result in the robot experiencing quite
      // a bit of translation and/or rotation. In these situations, we
      // should not expect the configured odometry maximums to apply.
      if (sensors.bump() || sensors.cliff() || sensors.spin())
         ; // odometry maximums do not apply
      else // normal situation: low-level controller did not move robot
         throw behavior_error(BOGUS_LOW_LEVEL_ODOMETRY) ; // thresholds apply
   }

   (reinterpret_cast<Survey*>(client_data))->accumulate(distance, rotation) ;
}

// Since low-level odometry updates come in from the main thread, we need
// to "switch thread contexts" by signaling the survey behaviour's thread
// that new odometry is available.
void Survey::accumulate(int distance, int angle)
{
   //LERROR("raw odometry = [%4d %4d]", distance, angle) ;
   m_odometry_cond.signal(odometry_update(distance, angle)) ;
}

// This predicate is used in conjunction with the above function's
// signal. It adds the latest low-level odometry packet (distance and
// angle) to the survey behaviour's odometry tracker and checks if the
// SLAM module is busy or not. If not, it returns true to signal the
// survey behaviour that new odometry (viz., control input) is available
// for SLAM.
bool Survey::odometry_helper::operator()(Survey& survey)
{
   survey.m_odometry.add(distance, angle) ;
   /*
   LERROR("acc odometry = [%4d %4d]",
          survey.m_odometry.displacement(), survey.m_odometry.rotation()) ;
   // */
   if (! survey.m_slam_busy && survey.m_odometry.thresholds_crossed()) {
      survey.ut = survey.m_odometry ;
      survey.m_odometry.reset() ;
      return true ;
   }
   return false ;
}

// The survey behaviour waits until the low-level odometry has
// accumulated up to some predefined minimum limit before it triggers the
// next SLAM update. This predicate is used in conjunction with
// Survey::m_odometry_cond to check the odometry thresholds. If the
// thresholds have been crossed, it will set the SLAM module's state to
// busy so that odometric updates from the main thread accumulate while
// the SLAM update takes place and then return true to end the survey
// behaviour's thread's waiting and go ahead with the SLAM update.
bool Survey::threshold_helper::operator()(Survey& survey)
{
   if (survey.ut.thresholds_crossed()) {
      survey.m_slam_busy = true ;
      return true ;
   }
   return false ;
}

// Once the SLAM update is done, we need to mark the SLAM module as "not
// busy" so that we can proceed with the next update. As long as the SLAM
// module is busy, the main thread's odometric updates will be
// accumulated rather than immediately acted upon. To ensure proper
// synchronization with the main thread, this function is used in
// conjunction with Survey::m_odometry_cond's (internal) mutex.
void Survey::reset_helper::operator()(Survey& survey)
{
   survey.m_slam_busy = false ;
   survey.ut.reset() ;
}

// The survey behaviour uses a SLAM algorithm to build a map and record
// the robot's trajectory. This function implements the next SLAM update
// using the latest sensor and control inputs.
void Survey::action()
{
   UpdateLock::begin_read() ;
      LRFData zt(App::lrf()) ; // measurement at current time step t
   UpdateLock::end_read() ;

   viz_lock() ;
   try
   {
      /*
      LERROR("    ctl odometry = [%4d %4d]",
             ut.displacement(), ut.rotation()) ;
      // */
      m_slam->update(ut, zt) ; // DEVNOTE: ut is a member variable
   }
   catch (misc_error&) // ignore LOGIC_ERROR: line rasterizer moved more
   {                   // than one pixel in one step; ought not to happen,
   }                   // but if it does, visualization will freeze because
   viz_unlock() ;      // viz lock will still be held by dead SURVEY thread

   Map* M = App::map() ;
   M->update(m_slam->current_pose()) ;
   if (SlamParams::slam_mode()) // update map only when doing full SLAM
      M->update(m_slam->current_map()) ;
}

//--------------------------- VISUALIZATION -----------------------------

// Quick helper function to compare two particle visualization structures
// using the particle weights. Avoids the hassle of boost::bind for use
// with the std::min_element and std::max_element algorithms.
//
// DEVNOTE: Because Particle::Viz is defined in the lobot namespace, this
// comparison function also needs to be in the lobot namespace for the
// compiler to be able to find it. That is, putting it in an anonymous
// namespace here won't work. That's why we use the static keyword to
// make this function invisible outside of this translation unit.
static bool operator<(const Particle::Viz& a, const Particle::Viz& b)
{
   return a.weight < b.weight ;
}

// Given the min and max weights for all the particles, this function
// linearly intepolates a particle's weight to lie in [0,1] and returns a
// new particle with the same pose but the rescaled weight.
//
// NOTE: Since the denominator for the linear scaling formula, (max-min),
// is constant, we require the caller to compute that value and pass it
// in so that it doesn't need to be recomputed over and over in this
// function (which will be called from a loop in an STL algorithm).
static Particle::Viz
rescale_weight(const Particle::Viz& p, float min, float max_minus_min)
{
   return Particle::Viz(p.pose, (p.weight - min)/max_minus_min) ;
}

// This function shows the given particle's estimate of the robot's
// current position within the map as a simple dot. The dot's color is
// set so as to depict the particle's relative importance. Each particle
// is painted in a blue-green combination. Particles with higher weights
// have more green in them whereas those with lower weights are bluer.
static void render_particle(const Particle::Viz& p)
{
   glColor3f(0, p.weight, 1 - p.weight) ;
   glVertex2f(p.pose.y(), p.pose.x()) ;
}

// This function shows the particle's estimate of the robot's current
// bearing w.r.t. the map as a fixed-size line. The line's size is fixed
// in screen pixels; appropriate scale factors for converting this fixed
// size to the map's coordinate system are determined in the Params
// initialization. These scale factors should be passed in to this
// function.
static void render_bearing(const Particle::Viz& p, float Sx, float Sy)
{
   const float x = p.pose.x() ;
   const float y = p.pose.y() ;
   const float t = p.pose.t() ;

   glColor3f(0, p.weight, 1 - p.weight) ;
   glVertex2f(y, x) ;
   glVertex2f(y + Sy * sin(t), x + Sx * cos(t)) ;
}

// Callback function for drawing the particles on the map
void Survey::render_particles(unsigned long client_data)
{
   (reinterpret_cast<Survey*>(client_data))->render_particles() ;
}

void Survey::render_particles()
{
   // First, retrieve particle poses and weights from SLAM algorithm
   viz_lock() ;
      std::vector<Particle::Viz> particles = m_slam->viz() ;
   viz_unlock() ;

   // Then, rescale particle weights to lie within [0,1]
   float min = (std::min_element(particles.begin(), particles.end()))->weight ;
   float max = (std::max_element(particles.begin(), particles.end()))->weight ;
   std::transform(particles.begin(), particles.end(), particles.begin(),
                  boost::bind(rescale_weight, _1, min, max - min)) ;

   // Setup 2D "view volume" to match real/physical coordinate system
   // except that the whole thing is rotated 90 degrees ccw so that our
   // notion of "up" matches that of the robot's.
   float L, R, B, T ; // map extents
   SlamParams::map_extents(&L, &R, &B, &T) ;
   glMatrixMode(GL_PROJECTION) ;
   glPushMatrix() ;
   glLoadIdentity() ;
   gluOrtho2D(T, B, L, R) ;

   glMatrixMode(GL_MODELVIEW) ;
   glPushMatrix() ;
   glLoadIdentity() ;

   // Now we're ready to draw the particles...
   bool show_bearings = Params::show_bearings() ;
   glPushAttrib(GL_POINT_BIT) ;
   glPointSize(show_bearings ? 5 : 1) ;
   glBegin(GL_POINTS) ;
      std::for_each(particles.begin(), particles.end(), render_particle) ;
   glEnd() ;
   if (show_bearings) {
      glBegin(GL_LINES) ;
         std::for_each(particles.begin(), particles.end(),
                       boost::bind(render_bearing, _1,
                                   Params::Bx(), Params::By())) ;
      glEnd() ;
   }
   glPopAttrib() ;

   // Reset GL transformations so next drawable won't get screwed
   restore_view_volume() ;
}

//----------------------------- CLEAN-UP --------------------------------

Survey::~Survey(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

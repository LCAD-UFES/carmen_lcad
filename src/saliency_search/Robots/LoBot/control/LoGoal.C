/**
   \file  Robots/LoBot/control/LoGoal.C
   \brief This file defines the non-inline member functions of the
   lobot::Goal class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoGoal.C $
// $Id: LoGoal.C 13812 2010-08-21 04:04:10Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoGoal.H"
#include "Robots/LoBot/control/LoMetrics.H"
#include "Robots/LoBot/control/LoTurnArbiter.H"
#include "Robots/LoBot/control/LoSpinArbiter.H"

#include "Robots/LoBot/LoApp.H"
#include "Robots/LoBot/slam/LoMap.H"
#include "Robots/LoBot/slam/LoSlamParams.H"

#include "Robots/LoBot/config/LoConfigHelpers.H"

#include "Robots/LoBot/thread/LoShutdown.H"
#include "Robots/LoBot/thread/LoUpdateLock.H"
#include "Robots/LoBot/thread/LoPause.H"

#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/LoRegistry.H"
#include "Robots/LoBot/misc/singleton.hh"

#include "Robots/LoBot/util/LoGL.H"
#include "Robots/LoBot/util/LoMath.H"

// OpenGL headers
#ifdef INVT_HAVE_LIBGLU
#include <GL/glu.h>
#endif

#ifdef INVT_HAVE_LIBGL
#include <GL/gl.h>
#endif

// Standard C++ headers
#include <iomanip>
#include <sstream>
#include <numeric>
#include <algorithm>
#include <functional>
#include <iterator>
#include <utility>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- KNOB TWIDDLING -----------------------------

namespace {

// Retrieve settings from goal section of config file
template<typename T>
inline T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>(LOBE_GOAL, key, default_value) ;
}

// Overload for retrieving pairs
template<typename T>
inline std::pair<T, T>
conf(const std::string& key, const std::pair<T, T>& default_value)
{
   return get_conf<T>(LOBE_GOAL, key, default_value) ;
}

/// This local class encapsulates various parameters that can be used to
/// tweak different aspects of the goal behaviour.
class Params : public singleton<Params> {
   /// This behaviour can be configured to actively steer the robot to
   /// each goal or to simply monitor the robot's current location to
   /// check if the robot is at a goal or not. When set, this flag will
   /// make the goal behaviour actively seek each goal in its list. By
   /// default, this flag is off, i.e., the behaviour only monitors the
   /// robot's location without affecting its steering in any way.
   bool m_seek_mode ;

   /// What should we do when we reach a goal? The options are:
   ///
   ///    - continue on to the next goal
   ///    - pause robot and wait for user to start it moving again
   ///
   /// This setting specifies which of the above two actions to use. By
   /// default, the goal behaviour implements the "continue" action,
   /// i.e., when the robot reaches a goal, it simply carries on to the
   /// next one. However, by setting this flag, the behaviour will pause
   /// the robot and wait for the user to explicitly signal resumption of
   /// normal operations.
   bool m_pause ;

   /// By default, when the final goal in the goal list is reached, the
   /// goal behaviour will maintain that goal. Thus, if another behaviour
   /// causes the robot to move away from the final goal in the goal
   /// list, the goal behaviour will redirect the robot back to that
   /// goal.
   ///
   /// However, by setting this flag, we can have the goal behaviour loop
   /// back to the first goal in the list and start over.
   bool m_loop ;

   /// In loop mode, instead of starting over at the first goal, we can
   /// have the goal behaviour backtrack over the goal list, i.e., seek
   /// each of the goals in reverse. This flag enables backtracking. It
   /// is off by default.
   bool m_backtrack ;

   /// When configured to actively steer the robot towards goals, the
   /// goal behaviour directs the robot towards the current goal by
   /// implementing the VFF method described by Borenstein and Koren (see
   /// "Real-time Obstacle Avoidance for Fast Mobile Robots," IEEE
   /// Transactions on Systems, Man, and Cybernetics 19(5):1179--1187,
   /// Sep/Oct 1989).
   ///
   /// The following setting specifies the size of the active window
   /// within the certainty grid (i.e., occupancy map). This size is
   /// specified in terms of the number of cells occupied by the active
   /// window.
   ///
   /// NOTE: Actually, this setting specifies half of the active window's
   /// size. That is, internally, this number is doubled.
   int m_vff_window_size ;

   /// The VFF method mentioned above computes a repulsive force vector
   /// for each cell within the active window and an attractive force
   /// vector for the goal. The sum of all the repulsive forces and the
   /// single attractive force result in a final steering vector.
   ///
   /// This setting specifies the force constants to use for the
   /// individual force vectors. It should be a pair of numbers. The
   /// first number is the repulsive force constant and the second one is
   /// the attractive force constant.
   std::pair<float, float> m_vff_forces ;

   /// Usually, steering control is effected using the turn arbiter,
   /// which veers the robot in different directions while it moves,
   /// i.e., smooth car-like turns. However, the lgmd_extricate_tti
   /// behaviour also supports spin-style steering, i.e., momentarily
   /// stopping the robot and then turning it cw/ccw in-place. This flag
   /// turns on spin-style steering. By default, the behaviour uses the
   /// normal car-like steering mode.
   bool m_spin_style_steering ;

   /// The number of milliseconds between successive iterations of this
   /// behaviour.
   ///
   /// WARNING: The ability to change a behaviour's update frequency is a
   /// very powerful feature whose misuse or abuse can wreak havoc! Be
   /// sure to use reasonable values for this setting.
   int m_update_delay ;

   /// The location and size (within the Robolocust main window) of the
   /// goal behaviour's visualization.
   typedef Drawable::Geometry Geom ; // just for properly aligning accessors
   Drawable::Geometry m_geometry ;

   /// In addition to rendering its steering votes, the goal behaviour
   /// can also render the goal list on the Robolocust map. This flag
   /// turns on goal list rendering. It is off by default.
   bool m_render_goals ;

   /// Private constructor because this is a singleton.
   Params() ;
   friend class singleton<Params> ;

public:
   /// Accessing the various parameters
   //@{
   static bool  seek_mode()    {return instance().m_seek_mode    ;}
   static bool  pause()        {return instance().m_pause        ;}
   static bool  loop()         {return instance().m_loop         ;}
   static bool  backtrack()    {return instance().m_backtrack    ;}
   static bool  render_goals() {return instance().m_render_goals ;}
   static bool  spin_style_steering(){return instance().m_spin_style_steering;}
   static int   update_delay()       {return instance().m_update_delay       ;}
   static Geom  geometry()           {return instance().m_geometry           ;}
   static int   vff_window_size()    {return instance().m_vff_window_size    ;}
   static float vff_rep_force()      {return instance().m_vff_forces.first   ;}
   static float vff_att_force()      {return instance().m_vff_forces.second  ;}
   //@}
} ;

// Parameter initialization
Params::Params()
   : m_seek_mode(conf("actively_seek_goals", false)),
     m_pause(conf("pause_at_goal", false)),
     m_loop(conf("loop", false)),
     m_backtrack(conf("backtrack", false)),
     m_vff_forces(conf("vff_forces", std::make_pair(100.0f, 1000.0f))),
     m_spin_style_steering(conf("spin_style_steering", false)),
     m_update_delay(clamp(conf("update_delay", 1750), 500, 10000)),
     m_geometry(conf<std::string>("geometry", "0 0 10 10")),
     m_render_goals(conf("render_goals", false))
{
   const float R = clamp(robot_conf("size", 400), 150, 1000) ;
   const float s = SlamParams::map_cell_size() ;
   const int   n = round(R/s)/2 ; // num cells occupied by half robot
   m_vff_window_size = clamp(conf("vff_window_size", 3*n/2), n + 1, 4 * n) ;

   m_vff_forces.first  = clamp(m_vff_forces.first,  1.0f, 1e6f) ;
   m_vff_forces.second = clamp(m_vff_forces.second, 1.0f, 1e6f) ;
}

} // end of local anonymous namespace encapsulating above helpers

//-------------------------- INITIALIZATION -----------------------------

Goal::Goal()
   : base(Params::update_delay(), LOBE_GOAL, Params::geometry()),
     m_goal(-1), m_turn_dir(0)
{
   start(LOBE_GOAL) ;
}

Goal::Target::Target(float L, float R, float B, float T)
   : m_left(L), m_right(R), m_bottom(B), m_top(T),
     m_center_x((L + R)/2), m_center_y((B + T)/2)
{}

void Goal::pre_run()
{
   if (! App::robot())
      throw behavior_error(MOTOR_SYSTEM_MISSING) ;

   Map* map = App::map() ;
   if (! map)
      throw behavior_error(MAPPING_DISABLED) ;

   // Read in goal list
   float L, R, B, T ;
   SlamParams::map_extents(&L, &R, &B, &T) ;
   std::istringstream goals(conf<std::string>("goals", "")) ;
   for(;;)
   {
      float l, r, b, t ;
      goals >> l >> r >> b >> t ;
      if (! goals)
         break ;
      m_goals.push_back(Target(clamp(l, L, R), clamp(r, l, R),
                               clamp(b, B, T), clamp(t, b, T))) ;
   }
   if (m_goals.empty())
      throw behavior_error(NO_GOALS) ;

   // Start off seeking the first goal
   m_goal = 0 ;

   // Easy way to implement backtracking: simply append the goals in
   // reverse to the goal list.
   if (Params::loop() && Params::backtrack())
      std::copy(m_goals.rbegin() + 1, m_goals.rend() - 1,
                std::back_inserter(m_goals)) ;

   // Init VFF mask
   const int   W = Params::vff_window_size() ;
   const float S = SlamParams::map_cell_size() ;
   const float F = -Params::vff_rep_force() ;

   int x, y ;
   float dx, dy ;
   m_vff_mask.reserve(sqr(W * 2 + 1)) ;
   for (y =  W, dy = y * S; y >= -W; --y, dy -= S)
   for (x = -W, dx = x * S; x <=  W; ++x, dx += S)
   {
      float d2 = sqr(dx) + sqr(dy) ;
      float d  = sqrtf(d2) ;
      m_vff_mask.push_back(Vector((x == 0) ? 0 : F/d2 * dx/d,
                                  (y == 0) ? 0 : F/d2 * dy/d)) ;
   }

   // Add goal rendering callback to map object
   if (Params::render_goals())
      map->add_hook(RenderHook(
         render_goals, reinterpret_cast<unsigned long>(this))) ;
}

//---------------------- THE BEHAVIOUR'S ACTION -------------------------

// Override from base class because we want to monitor the Pause state.
// Every time the application resumes from a paused state, we want to
// send a message to the metrics log stating that goal seeking has begun.
// When a goal is reached, the action method will log that event.
//
// DEVNOTE: In the while loop below, we use the App::map() function's
// returned pointer without checking it. This is okay because pre_run()
// would have thrown an exception if it were not.
void Goal::run()
{
   try
   {
      App::wait_for_init() ;
      pre_run() ;

      const Map* M = App::map() ;
      bool paused_previously = Pause::is_set() ;
      while (! Shutdown::signaled())
      {
         bool paused_now = Pause::is_set() ;
         if (! paused_now) {
            if (paused_previously)
               log("seeking goal", m_goal, m_goals[m_goal], M->current_pose());
            action() ;
         }
         paused_previously = paused_now ;
         usleep(m_update_delay) ;
      }

      post_run() ;
   }
   catch (uhoh& e)
   {
      LERROR("behaviour %s encountered an error: %s", name.c_str(), e.what()) ;
      return ;
   }
}

// To drive towards the current goal, we turn based on the difference
// between the current bearing and the steering vector computed by the
// VFF technique. This steering vector takes into account the local
// obstacle configuration as well as the goal location.
void Goal::action()
{
   const Map* map = App::map() ;
   const Pose P = map->current_pose() ;

   int turn_dir = 0 ;
   Vector Ft, Fr, R ;

   const Target goal = m_goals[m_goal] ;
   if (goal.at(P.x(), P.y()))
   {
      log("reached goal", m_goal, goal, P) ;

      if (Params::pause())
         Pause::set() ;

      ++m_goal ;
      if (m_goal == static_cast<int>(m_goals.size())) {
         if (Params::loop())
            m_goal = 0 ;
         else // maintain final goal in list
            --m_goal ;
      }
   }
   else if (Params::seek_mode()) // not yet at goal and configured to seek it
   {
      // We use Borenstein and Koren's VFF method to steer the robot
      // towards the goal. The first step is to obtain the VFF active
      // window, which is a subportion of the occupancy map centered at
      // the robot's current position.
      std::vector<unsigned char> vff_window =
         map->submap(Params::vff_window_size()) ;

      // The lobot::Map class uses zero to indicate the presence of an
      // obstacle and 255 for free space. We need to flip that so that
      // the repulsive force computations work correctly.
      std::transform(vff_window.begin(), vff_window.end(), vff_window.begin(),
                     std::bind1st(std::minus<unsigned char>(), 255)) ;

      // Now, compute the repulsive force vector by convolving the VFF
      // vector mask with the certainty values in the active window.
      Fr = std::inner_product(m_vff_mask.begin(), m_vff_mask.end(),
                              vff_window.begin(), Vector(0, 0)) ;

      // Compute the target/goal's attractive force vector...
      Ft = normalized(Vector(goal.x() - P.x(), goal.y() - P.y()))
         * Params::vff_att_force() ;

      // Compute the steering vector...
      R = Ft + Fr ;

      // Follow the VFF steering vector by determining the amount of
      // error between it and the robot's current heading.
      const int T = TurnArbiter::turn_max() ;
      const float current_heading =
         (P.bearing() < 180) ? P.bearing() : P.bearing() - 360 ;
      const int heading_error = round(direction(R) - current_heading) ;
      if (Params::spin_style_steering())
      {
         turn_dir = heading_error ;
         SpinArbiter::instance().vote(base::name,
                                      new SpinArbiter::Vote(turn_dir)) ;
      }
      else
      {
         turn_dir = clamp(heading_error, -T, T) ;
         TurnArbiter::instance().vote(base::name,
            new TurnArbiter::Vote(turn_vote_centered_at(turn_dir))) ;
      }
   }

   // Record VFF vectors for visualization
   viz_lock() ;
      m_attractive = Ft ;
      m_repulsive  = Fr ;
      m_steering   = R  ;
      m_turn_dir   = turn_dir ;
   viz_unlock() ;
}

// Helper to send appropriate messages to metrics log
void
Goal::
log(const char* msg, int index, const Goal::Target& goal, const Pose& p) const
{
   int n = index + 1 ;
   const int N = m_goals.size()/2 + 1 ;
   if (Params::loop() && Params::backtrack() && n > N)
      n = N - (n - N) ;

   Metrics::Log() << std::setw(Metrics::opw()) << std::left
                  << (std::string(msg) + " #" + to_string(n))
                  << goal << " from/at" << Metrics::newl()
                  << dup_string(" ", 6 + Metrics::opw()) << p ;
}

//--------------------------- VISUALIZATION -----------------------------

#ifdef INVT_HAVE_LIBGL

// Helper function to convert the magnitude of a vector into a string,
// prefixing the supplied single character label to the mag string.
static std::string mag(char label, const Vector& v)
{
   using namespace std ;

   std::ostringstream v_str ;
   v_str << label   << ": "
         << setw(8) << fixed << setprecision(1) << magnitude(v) ;
   return v_str.str() ;
}

void Goal::render_me()
{
   // Make local copy so that goal behaviour's thread isn't held up
   // waiting for visualization thread to complete.
   viz_lock() ;
      Vector A = m_attractive ;
      Vector R = m_repulsive  ;
      Vector T = m_steering   ;
      int turn = m_turn_dir   ;
   viz_unlock() ;

   // Draw principal axes (to make it easier to understand what's going
   // on with the force vectors and turn command).
   unit_view_volume() ;
   glColor3f(1, 1, 1) ;
   glBegin(GL_LINES) ;
      glVertex2i(0, -1) ;
      glVertex2i(0,  1) ;
      glVertex2i(-1, 0) ;
      glVertex2i( 1, 0) ;
   glEnd() ;

   // Render the VFF vectors
   if (is_zero(T.i) && is_zero(T.j))
      ; // no steering took place
   else
   {
      glColor3f(0, 1, 0) ;
      draw_vector(normalized(A)) ;

      glColor3f(1, 0, 0) ;
      draw_vector(normalized(R)) ;

      glColor3f(0.15f, 0.75f, 0.85f) ;
      draw_vector(normalized(T)) ;

      glColor3f(1, 1, 1) ;
      draw_vector(Vector(0.75f * cos(turn), 0.75f * sin(turn))) ;
   }

   // Label the visualization so that it is easy to tell which behaviour
   // is being visualized. Also label the speed value and, if applicable,
   // the distance reading that resulted in the chosen speed.
   restore_view_volume() ;
   text_view_volume() ;
      glColor3f(0, 1, 1) ;
      draw_label(3, 12, "Goal") ;

      glColor3f(0, 1, 0) ;
      draw_label(3, m_geometry.height - 28, mag('A', A).c_str()) ;

      glColor3f(1, 0, 0) ;
      draw_label(3, m_geometry.height - 16, mag('R', R).c_str()) ;

      glColor3f(0.15f, 0.75f, 0.85f) ;
      draw_label(3, m_geometry.height -  4, mag('T', T).c_str()) ;
   restore_view_volume() ;
}

void Goal::render_goals(unsigned long client_data)
{
   (reinterpret_cast<Goal*>(client_data))->render_goals() ;
}

void Goal::render_goals()
{
   // Setup 2D "view volume" to match real/physical coordinate system
   // except that the whole thing is rotated 90 degrees ccw so that our
   // notion of "up" matches that of the robot's.
   float L, R, B, T ; // map extents
   SlamParams::map_extents(&L, &R, &B, &T) ;
   setup_view_volume(T, B, L, R) ;

   // Now we're ready to draw the goal locations...
   glPushAttrib(GL_POINT_BIT | GL_LINE_BIT) ;
   glPointSize(5) ;
   glColor3f(0, 0, 1) ;
   glBegin(GL_POINTS) ;
      std::for_each(m_goals.begin(), m_goals.end(), render_goal) ;
   glEnd() ;

   // For current goal being sought, we also draw a dashed box around it
   glEnable(GL_LINE_STIPPLE) ;
   glLineStipple(1, 0xCCCC) ;
   glColor3f(1, 0, 0) ;
   glBegin(GL_LINE_LOOP) ;
      const Target& current = m_goals[m_goal] ; // WARNING: m_goal: no viz lock
      glVertex2f(current.T(), current.L()) ;
      glVertex2f(current.B(), current.L()) ;
      glVertex2f(current.B(), current.R()) ;
      glVertex2f(current.T(), current.R()) ;
   glEnd() ;
   glPopAttrib() ;

   // Reset GL transformations so next drawable won't get screwed
   restore_view_volume() ;
}

void Goal::render_goal(const Goal::Target& t)
{
   glVertex2f(t.y(), t.x()) ;//coord sys 90 degree ccw rotation ==> swap x & y
}

#endif

//----------------------------- CLEAN-UP --------------------------------

Goal::~Goal(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/**
   \file  Robots/LoBot/control/LoOpenPath.C
   \brief This file defines the non-inline member functions of the
   lobot::OpenPath class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoOpenPath.C $
// $Id: LoOpenPath.C 14306 2010-12-09 00:11:01Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoOpenPath.H"
#include "Robots/LoBot/control/LoSpinArbiter.H"

#include "Robots/LoBot/LoApp.H"
#include "Robots/LoBot/ui/LoLaserViz.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/thread/LoUpdateLock.H"

#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/LoRegistry.H"
#include "Robots/LoBot/misc/singleton.hh"

#include "Robots/LoBot/util/LoGL.H"
#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/util/range.hh"

// OpenGL headers
#ifdef INVT_HAVE_LIBGLU
#include <GL/glu.h>
#endif

#ifdef INVT_HAVE_LIBGL
#include <GL/gl.h>
#endif

// Standard C++ headers
#include <algorithm>
#include <map>
#include <functional>
#include <utility>

// Standard C headers
#include <math.h>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- KNOB TWIDDLING -----------------------------

namespace {

// Retrieve settings from open_path section of config file
template<typename T>
inline T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>(LOBE_OPEN_PATH, key, default_value) ;
}

// Overload of above function for retrieving ranges
template<typename T>
inline range<T> conf(const std::string& key, const range<T>& default_value)
{
   return get_conf<T>(LOBE_OPEN_PATH, key, default_value) ;
}

/// This inner class encapsulates various parameters that can be used
/// to tweak different aspects of the open path behaviour.
class Params : public singleton<Params> {
   /// The width of the robot. This setting specifies the minimum
   /// width (in mm) that each open path must be.
   float m_path_width ;

   /// We will only consider open paths that are at least the length
   /// (in mm) specified by this setting.
   float m_min_path_length ;

   /// A derived parameter that keeps track of the angular range about
   /// a given direction that must be searched for candidate open
   /// paths.
   int m_alpha ;

   /// Instead of looking for open paths across the laser range
   /// finder's entire FOV, we can restrict this behaviour to search
   /// only some subportion of it. This setting specifies the angular
   /// range to search for open paths. It is a single number
   /// specifying the angular range for each side of zero degrees.
   int m_fov ;

   /// Instead of looking for open paths at each angle, we can use
   /// this parameter to skip some angles.
   int m_step ;

   /// To ensure that the robot doesn't keep turning unnecessarily, we
   /// mark some angular range in front of the robot as a dead zone.
   /// When the most open path lies in this angular range, the open
   /// path behaviour will simply keep driving straight and not affect
   /// the steering.
   ///
   /// To disable this policy, simply specify a range that lies
   /// outside the laser range finder's FOV (e.g., 260 to 280). That
   /// way, all open paths will lie outside of the dead zone and the
   /// robot will always steer towards the most open path  in every
   /// iteration of this behaviour.
   ///
   /// To have the robot never turn towards the most open path, simply
   /// use the entire FOV of the laser range finder. Then, since all
   /// open paths will lie in the dead zone, the robot will never turn
   /// towards the mostopen path. (Note, however, that an easier way
   /// to do this is to simply disable the open path behaviour.)
   ///
   /// This setting expects two integers. The first specifies the
   /// minimum of the dead zone's angular range (usually a negative
   /// number indicating an angle on the robot's right). The second
   /// number is the dead zone's maximum (usually a positive number
   /// for a direction on the left of the robot).
   range<int> m_dead_zone ;

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
   Geom m_geometry ;

   /// Private constructor because this is a singleton.
   Params() ;

   // Boilerplate code to make generic singleton design pattern work
   friend class singleton<Params> ;

public:
   /// Accessing the various parameters.
   //@{
   static float path_width()      {return instance().m_path_width ;}
   static float min_path_length() {return instance().m_min_path_length ;}
   static int   alpha()           {return instance().m_alpha ;}
   static int   fov()             {return instance().m_fov   ;}
   static int   step()            {return instance().m_step  ;}
   static range<int> dead_zone()  {return instance().m_dead_zone ;}
   static bool  spin_style_steering(){return instance().m_spin_style_steering;}
   static int   update_delay()       {return instance().m_update_delay       ;}
   static Geom  geometry()           {return instance().m_geometry           ;}
   //@}
} ;

// Parameters initialization
Params::Params()
   : m_path_width(clamp(conf("path_width", 175.0f), 150.0f, 500.0f)),
     m_min_path_length(clamp(conf("min_path_length", 250.0f),
                             100.0f, 2500.0f)),
     m_alpha(round(atan(m_path_width/2/m_min_path_length))),
     m_fov(clamp(conf("fov", 90), 0, 180)),
     m_step(clamp(conf("step", 10), 1, 45)),
     m_dead_zone(conf("dead_zone", range<int>(-20, 20))),
     m_spin_style_steering(conf("spin_style_steering", false)),
     m_update_delay(clamp(conf("update_delay", 600), 250, 10000)),
     m_geometry(conf<std::string>("geometry", "0 0 10 10"))
{}

} // end of local anonymous namespace encapsulating above helpers

//-------------------------- INITIALIZATION -----------------------------

OpenPath::OpenPath()
   : base(Params::update_delay(), LOBE_OPEN_PATH, Params::geometry())
{
   start(LOBE_OPEN_PATH) ;
}

void OpenPath::pre_run()
{
   if (! App::lrf())
      throw behavior_error(LASER_RANGE_FINDER_MISSING) ;
   if (! App::robot())
      throw behavior_error(MOTOR_SYSTEM_MISSING) ;

   Drawable* v = App::laser_viz() ;
   if (v)
      v->add_hook(RenderHook(render_paths,
                             reinterpret_cast<unsigned long>(this))) ;
}

OpenPath::PathInfo::PathInfo(float length, float width)
   : m_info(length, width, length * width)
{
   if (length < 0 || width < 0)
      m_info.third = -1 ;
}

//---------------------- THE BEHAVIOUR'S ACTION -------------------------

void OpenPath::action()
{
   // Make local copy of LRF data to avoid holding update lock for too
   // long...
   UpdateLock::begin_read() ;
      LRFData lrf(App::lrf()) ;
   UpdateLock::end_read() ;

   // The list of candidate open paths is stored as a mapping between LRF
   // measurement angles and the corresponding path lengths along those
   // directions.
   Paths paths ;

   // Find the candidate open paths in each direction supported by LRF
   int fov = std::max(lrf.min_angle() + Params::alpha(), -Params::fov()) ;
   for (int angle = 0; angle >= fov; angle -= Params::step())
   {
      PathInfo p = open_path(angle, lrf) ;
      if (p.area() > 0)
         paths.insert(std::make_pair(angle, p)) ;
   }

   fov = std::min(lrf.max_angle() - Params::alpha(), Params::fov()) ;
   for (int angle = Params::step(); angle <= fov; angle += Params::step())
   {
      PathInfo p = open_path(angle, lrf) ;
      if (p.area() > 0)
         paths.insert(std::make_pair(angle, p)) ;
   }
   viz_lock() ;
      m_paths = paths ; // record raw paths for visualization
   viz_unlock() ;
   //dump(paths, "OpenPath::action", "paths(raw)") ;

   // Find most open path (i.e., path with max length)
   Paths::const_iterator max =
      std::max_element(paths.begin(), paths.end(), map_value_compare(paths)) ;
   //LERROR("max reading = [%4d %8.1f]", max->first, max->second) ;

   // Steer towards most open path only if it is significantly to the
   // left or right of the robot. Otherwise, just keep going straight.
   if (! Params::dead_zone().in(max->first))
   {
      if (Params::spin_style_steering())
         SpinArbiter::instance().vote(base::name,
                                      new SpinArbiter::Vote(max->first)) ;
      else
      {
         const int T = TurnArbiter::turn_max() ;
         TurnArbiter::Vote* V = new TurnArbiter::Vote(
            turn_vote_centered_at(clamp(max->first, -T, T))) ;
         //V->dump("OpenPath::action") ;

         // Record the above vote for visualization before turning it over
         // to the turn arbiter. Otherwise, it is possible the vote might
         // get deleted by the turn arbiter, which would cause a segfault
         // here when this thread attempts to dereference the pointer.
         viz_lock() ;
            m_vote = *V ;
         viz_unlock() ;

         TurnArbiter::instance().vote(base::name, V) ;
      }
   }
}

// This function returns an open path (if available) at the specified
// direction.
OpenPath::PathInfo OpenPath::open_path(int theta, const LRFData& lrf) const
{
   const float w = Params::path_width()/2 ;

   float L = lrf.max_distance() * 2 ;
   for (int x = theta - Params::alpha(); x <= theta + Params::alpha(); ++x)
   {
      int D = lrf[x] ;
      if (D < 0)
         continue ;
      if (D * sin(abs(theta - x)) <= w) {
         float d = D * cos(abs(theta - x)) ;
         if (d < L)
            L = d ;
      }
   }

   if (L > lrf.max_distance() || L < Params::min_path_length())
      return PathInfo(-1, -1) ;
   return PathInfo(L, Params::path_width()) ;
}
//--------------------------- VISUALIZATION -----------------------------

#ifdef INVT_HAVE_LIBGL

void OpenPath::render_me()
{
   // Make local copy so that open path behaviour's thread isn't held up
   // waiting for visualization thread to complete.
   viz_lock() ;
      TurnArbiter::Vote V = m_vote ;
   viz_unlock() ;

   // Render the votes visualization
   unit_view_volume() ;
   glBegin(GL_LINES) ;
      for (TurnArbiter::Vote::iterator it = V.begin(); it; ++it)
      {
         float vote = it.value() ;
         float direction = it.direction() ;
         if (vote < 0) // voted against this direction
         {
            glColor3f(1, 0, 0) ;
            glVertex2i(0, 0) ;
            glVertex2f(-vote * cos(direction), -vote * sin(direction)) ;
         }
         else if (vote > 0) // voted for this direction
         {
            glColor3f(0, 1, 0) ;
            glVertex2i(0, 0) ;
            glVertex2f(vote * cos(direction), vote * sin(direction)) ;
         }
      }
   glEnd() ;

   // Label the visualization so that it is easy to tell which behaviour
   // is being visualized.
   restore_view_volume() ;
   text_view_volume() ;
      glColor3f(0, 1, 1) ;
      draw_label(3, 12, "Open Path") ;
   restore_view_volume() ;
}

void OpenPath::render_paths(unsigned long client_data)
{
   (reinterpret_cast<OpenPath*>(client_data))->render_paths() ;
}

void OpenPath::render_paths()
{
   // Make local copy so that open path behaviour's thread isn't held up
   // waiting for visualization thread to complete.
   viz_lock() ;
      Paths P = m_paths ;
   viz_unlock() ;

   // Render the open paths, overlaying them on the LRF visualization
   glPushMatrix() ;
   glRotatef(get_conf("laser_viz", "lrf_direction", 90.0f), 0, 0, 1) ;
   glBegin(GL_LINES) ;
      glColor3f(1, 1, 1) ;
      for (Paths::const_iterator it = P.begin(); it != P.end(); ++it)
      {
         glVertex2i(0, 0) ;
         glVertex2f(it->second.length() * cos(it->first),
                    it->second.length() * sin(it->first));
      }
   glEnd() ;
   glPopMatrix() ;
}

#endif

//----------------------------- CLEAN-UP --------------------------------

OpenPath::~OpenPath(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/**
   \file  Robots/LoBot/control/LoForward.C
   \brief This file defines the non-inline member functions of the
   lobot::Forward class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoForward.C $
// $Id: LoForward.C 13521 2010-06-06 14:23:03Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoForward.H"

#include "Robots/LoBot/LoApp.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/thread/LoUpdateLock.H"

#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/LoRegistry.H"
#include "Robots/LoBot/util/LoGL.H"
#include "Robots/LoBot/util/LoMath.H"

// OpenGL headers
#ifdef INVT_HAVE_LIBGL
#include <GL/gl.h>
#endif

// Standard C++ headers
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <limits>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//--------------------------- LOCAL HELPERS -----------------------------

// Retrieve settings from forward section of config file
template<typename T>
static inline T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>(LOBE_FORWARD, key, default_value) ;
}

// Overload of above function for retrieving ranges
template<typename T>
static inline range<T>
conf(const std::string& key, const range<T>& default_value)
{
   return get_conf<T>(LOBE_FORWARD, key, default_value) ;
}

//-------------------------- INITIALIZATION -----------------------------

Forward::Forward()
   : base(clamp(conf("update_delay", 1000), 100, 2500),
          LOBE_FORWARD, conf<std::string>("geometry", "480 0 140 140")),
     m_min_distance(0, -1)
{
   start(LOBE_FORWARD) ;
}

void Forward::pre_run()
{
   if (Params::adaptive_mode())
   {
      if (! App::lrf())
         throw behavior_error(LASER_RANGE_FINDER_MISSING) ;
   }
   //else: in fixed mode, don't need the LRF
}

//---------------------- THE BEHAVIOUR'S ACTION -------------------------

// Depending on how it's configured, the forward driving behaviour will
// either vote to drive at a fixed speed or, in adaptive mode, at a speed
// based on distance readings from the LRF.
void Forward::action()
{
   // Steering decision is always to drive straight ahead
   // Speed, however, is decided based on operational mode
   TurnArbiter::Vote* T = new TurnArbiter::Vote(turn_vote_centered_at(0)) ;
   SpeedInfo S = Params::adaptive_mode() ? adaptive_speed() : fixed_speed() ;

   // First, record votes and other info for visualization before
   // turning them over to the arbiters. Otherwise, it is possible the
   // votes might get deleted by the arbiter threads, which would cause
   // a segfault here when this thread attempts to dereference those
   // pointers.
   viz_lock() ;
      m_turn_vote    = *T ;
      m_speed_vote   = *S.second ;
      m_min_distance = S.first ;
   viz_unlock() ;

   // Cast the turn and speed votes
    TurnArbiter::instance().vote(base::name, T) ;
   SpeedArbiter::instance().vote(base::name, S.second) ;
}

// In fixed mode, simply return a speed vote using the cruising params
Forward::SpeedInfo Forward::fixed_speed() const
{
   return SpeedInfo(LRFData::Reading(0, -1),
                    new SpeedArbiter::Vote(Params::cruising_speed(),
                                           Params::cruising_pwm())) ;
}

// In adaptive mode, regulate speed based on distance to closest obstacle
Forward::SpeedInfo Forward::adaptive_speed() const
{
   // Make local copy of LRF data to avoid holding update lock for too long
   UpdateLock::begin_read() ;
      LRFData lrf(App::lrf()) ;
   UpdateLock::end_read() ;

   // Find minimum average distance in configured FOV.
   //
   // NOTE: We multiply the average distance for each angle we consider
   // by the cosine of that angle in order to project that "distance
   // vector" onto the vector corresponding to straight ahead (i.e., zero
   // degree heading).
   const range<int> fov = Params::fov() ;
   const int step = Params::averaging_block() ;

   LRFData::Reading min(std::numeric_limits<int>::min(),
                        std::numeric_limits<int>::max()) ;
   for (int angle = fov.min(); angle <= fov.max(); angle += step)
   {
      const int m = clamp(angle - step/2, lrf.min_angle(), angle - 1) ;
      const int M = clamp(angle + step/2, angle + 1, lrf.max_angle()) ;
      const int d = round(lrf.average_distance(m, M)) ;
      if (d <= 0) // all bad readings in [angle - step/2, angle + step/2]?!?
         continue ;
      if (d < min.distance())
         min = LRFData::Reading(angle, d) ;
   }

   // Compute speed and PWM corresponding to min distance using linear
   // interpolation.
   const range<float> S = Params::speed_range() ;
   const range<int>   P = Params::pwm_range() ;
   const range<int>   D = Params::distance_range() ;

   if (min.distance() == std::numeric_limits<int>::max()) // all bad readings?
      return SpeedInfo(LRFData::Reading(180, -1),
                       new SpeedArbiter::Vote(S.min(), P.min())) ;

   const float d =
      static_cast<float>(min.distance() - D.min())/(D.max() - D.min()) ;
   const float s = clamp(S.min() + d * (S.max() - S.min()),
                         S.min(), S.max()) ;
   const int   p = clamp(P.min() + round(d * (P.max() - P.min())),
                         P.min(), P.max()) ;
   //LERROR("min distance = [%4d @ %4d], speed vote = %6.3f",
          //min.distance(), min.angle(), s) ;
   return SpeedInfo(min, new SpeedArbiter::Vote(s, p)) ;
}

//--------------------------- VISUALIZATION -----------------------------

#ifdef INVT_HAVE_LIBGL

// Helper to convert the minimum distance reading into a string label
static std::string dist(int d)
{
   std::ostringstream label ;
   label << d << " mm" ;
   return label.str() ;
}

// Helper to convert the chosen speed value a string label
static std::string speed(float s)
{
   using namespace std ;

   std::ostringstream label ;
   label << setw(6) << fixed << setprecision(3) << s << " m/s" ;
   return label.str() ;
}

void Forward::render_me()
{
   // Make local copy so that forward behaviour's thread isn't held up
   // waiting for visualization thread to complete.
   viz_lock() ;
       TurnArbiter::Vote V = m_turn_vote ;
      SpeedArbiter::Vote S = m_speed_vote ;
      LRFData::Reading   D = m_min_distance ;
   viz_unlock() ;

   unit_view_volume() ;
   glBegin(GL_LINES) ;
      // Render the turn vote's visualization
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

      // Render distance reading's direction corresponding to speed vote
      if (Params::adaptive_mode()) {
         glColor3f(1, 1, 1) ;
         glVertex2i(0, 0) ;
         glVertex2f(0.75f * cos(D.angle()), 0.75f * sin(D.angle())) ;
      }
   glEnd() ;

   // Label the visualization so that it is easy to tell which behaviour
   // is being visualized. Also label the speed value and, if applicable,
   // the distance reading that resulted in the chosen speed.
   restore_view_volume() ;
   text_view_volume() ;
   glColor3f(0, 1, 1) ;
   draw_label(3, 12, "Forward") ;
   if (Params::adaptive_mode())
      draw_label(3, m_geometry.height - 16, dist(D.distance()).c_str()) ;
   draw_label(3,  m_geometry.height - 4, speed(S.speed()).c_str()) ;

   restore_view_volume() ;
}

#endif

//----------------------------- CLEAN-UP --------------------------------

Forward::~Forward(){}

//-------------------------- KNOB TWIDDLING -----------------------------

// Parameters initialization
Forward::Params::Params()
   : m_cruising_speed(clamp(conf("cruising_speed", 0.5f), 0.1f, 10.0f)),
     m_cruising_pwm(clamp(conf("cruising_pwm", 30), 15, 100)),
     m_adaptive_mode(conf("adaptive_mode", false)),
     m_fov(clamp(conf("fov", range<int>(-60, 60)), range<int>(-90, 90))),
     m_averaging_block(clamp(conf("averaging_block", 10), 1, 20)),
     m_speed_range(clamp(conf("speed_range", range<float>(0.1f, 0.5f)),
                         range<float>(0.05f, 0.5f))),
     m_pwm_range(clamp(conf("pwm_range", range<int>(25, 100)),
                       range<int>(10, 100))),
     m_distance_range(clamp(conf("distance_range", range<int>(250, 2500)),
                            range<int>(100, 5000)))
{}

// Parameters clean-up
Forward::Params::~Params(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

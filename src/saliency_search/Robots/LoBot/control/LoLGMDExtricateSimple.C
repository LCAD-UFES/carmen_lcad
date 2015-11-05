/**
   \file  Robots/LoBot/control/LoLGMDExtricateSimple.C
   \brief This file defines the non-inline member functions of the
   lobot::LGMDExtricateSimple class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoLGMDExtricateSimple.C $
// $Id: LoLGMDExtricateSimple.C 13840 2010-08-27 22:28:12Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoLGMDExtricateSimple.H"
#include "Robots/LoBot/control/LoMetrics.H"
#include "Robots/LoBot/control/LoTurnArbiter.H"
#include "Robots/LoBot/control/LoSpinArbiter.H"
#include "Robots/LoBot/control/LoSpeedArbiter.H"

#include "Robots/LoBot/LoApp.H"
#include "Robots/LoBot/slam/LoMap.H"
#include "Robots/LoBot/lgmd/LocustModel.H"

#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/thread/LoUpdateLock.H"

#include "Robots/LoBot/misc/LoRegistry.H"
#include "Robots/LoBot/util/LoGL.H"
#include "Robots/LoBot/misc/LoVector.H"
#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/util/LoSTL.H"
#include "Robots/LoBot/misc/singleton.hh"
#include "Robots/LoBot/util/range.hh"

// OpenGL headers
#ifdef INVT_HAVE_LIBGL
#include <GL/gl.h>
#endif

// Boost headers
#include <boost/bind.hpp>

// Standard C++ headers
#include <iomanip>
#include <algorithm>
#include <functional>
#include <vector>
#include <iterator>
#include <utility>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- KNOB TWIDDLING -----------------------------

namespace {

// Retrieve settings from lgmd_extricate_simple section of config file
template<typename T>
inline T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>(LOBE_LGMD_EXTRICATE_SIMPLE, key, default_value) ;
}

/// This local class encapsulates various parameters that can be used
/// to tweak different aspects of this behaviour.
class Params : public singleton<Params> {
   /// If this extricate behaviour were to spin the robot in each and
   /// every iteration, it might make the robot zigzag about excessively.
   /// Therefore, we have the behaviour check the total range of LGMD
   /// spike rates, i.e., max - min, and act only when this range crosses
   /// a threshold, i.e., when the range of spiking activity has a
   /// significant differential in it. If the range is lower than the
   /// above-mentioned threshold, it means all the locusts are
   /// experiencing a similar level of spiking and so extrication would
   /// simply result in erratic, noisy behaviour.
   ///
   /// This setting specifies the minimum range across all the LGMD
   /// readings that we expect to see before the simple, reactive
   /// LGMD-based extrication will kick in. It should be a positive
   /// floating point number. Its units are Hertz.
   float m_threshold ;

   /// Once the above range threshold has been crossed, the behaviour can
   /// simply spin the robot towards the direction corresponding to
   /// minimal activity. However, there might be several readings that
   /// are the same or reasonably close to the minimum. In this
   /// situation, we would like to spin the robot by the least amount.
   ///
   /// The following setting specifies the maximum spike rate starting at
   /// the minimum that we consider "reasonably low." This extricate
   /// behaviour then looks at all the "reasonably low" LGMD readings and
   /// chooses the one that will result in the minimum amount of spin.
   ///
   /// The value of this setting should be a floating point number
   /// expressed in Hertz.
   float m_window ;

   /// Users may specify what speed they would like the extrication to
   /// occur.
   ///
   /// CAUTION: It would be unwise to make this speed too high.
   float m_extricate_speed ;

   /// In case the RPM sensor is configured to be off, we will need to
   /// specify the extricate "speed" in terms of PWM values as well.
   ///
   /// NOTE: All speed related behaviours should specify both a speed
   /// in m/s and a PWM value.
   ///
   /// CAUTION: For the extricate behaviour, it would be unwise to
   /// make the extrication PWM too high.
   int m_extricate_pwm ;

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
   static float threshold()    {return instance().m_threshold    ;}
   static float window()       {return instance().m_window       ;}
   static float extricate_speed()    {return instance().m_extricate_speed    ;}
   static int   extricate_pwm()      {return instance().m_extricate_pwm      ;}
   static bool  spin_style_steering(){return instance().m_spin_style_steering;}
   static int   update_delay()       {return instance().m_update_delay       ;}
   static Geom  geom()               {return instance().m_geometry           ;}
   //@}
} ;

// Parameters initialization
Params::Params()
   : m_threshold(clamp(conf("threshold", 50.0f), 0.1f, 1000.0f)),
     m_window(clamp(conf("window", 25.0f), 0.1f, 100.0f)),
     m_extricate_speed(clamp(conf("extricate_speed", 0.15f), 0.1f, 0.75f)),
     m_extricate_pwm(clamp(conf("extricate_pwm", 25), 10, 50)),
     m_spin_style_steering(conf("spin_style_steering", false)),
     m_update_delay(clamp(conf("update_delay", 1000), 1, 5000)),
     m_geometry(conf<std::string>("geometry", "0 0 10 10"))
{}

} // end of local anonymous namespace encapsulating above helpers

//-------------------------- INITIALIZATION -----------------------------

LGMDExtricateSimple::LGMDExtricateSimple()
   : base(Params::update_delay(), LOBE_LGMD_EXTRICATE_SIMPLE, Params::geom())
{
   start(LOBE_LGMD_EXTRICATE_SIMPLE) ;
}

LGMDExtricateSimple::Command::Command()
   : drive(0), turn(0)
{}

void LGMDExtricateSimple::pre_run()
{
   if (! App::robot())
      throw behavior_error(MOTOR_SYSTEM_MISSING) ;
}

//---------------------- THE BEHAVIOUR'S ACTION -------------------------

// Stuff a locust's direction and current LGMD spike rate into an STL pair
static std::pair<float, float> get_locust_data(const LocustModel* L)
{
   return std::make_pair(L->direction(), L->get_lgmd()) ;
}

// Compare locust data according to their directions
static bool
dir_cmp(const std::pair<float, float>& a, const std::pair<float, float>& b)
{
   return abs(a.first) < abs(b.first) ;
}

// Compare locust data according to their LGMD spike rates
static bool
lgmd_cmp(const std::pair<float, float>& a, const std::pair<float, float>& b)
{
   return a.second < b.second ;
}

// This version of the extricate behaviour monitors the LGMD spike rates
// of all the locusts and spins the robot to face the direction
// corresponding to minimal spiking activity.
void LGMDExtricateSimple::action()
{
   typedef std::pair<float, float> DLP ; // locust direction-LGMD pair
   typedef std::vector<DLP> DLV ;
   DLV dlv ;

   // Copy each locust's direction and current LGMD spike rate
   UpdateLock::begin_read() ;
      const App::LocustModels& L = App::locusts() ;
      dlv.reserve(L.size()) ;
      std::transform(L.begin(), L.end(), std::back_inserter(dlv),
                     get_locust_data) ;
   UpdateLock::end_read() ;

   // Sort the locusts according to their spike rates
   std::sort(dlv.begin(), dlv.end(), lgmd_cmp) ;

   // Issue extrication commands only if spike rate range is significant,
   // i.e., there is enough variation in spiking across all the locusts.
   Command C ;
   if (dlv.back().second - dlv.front().second >= Params::threshold())
   {
      // Starting at locust with least spiking activity, find locust
      // whose spike rate exceeds acceptable "window" of low readings.
      DLV::iterator end = std::find_if(dlv.begin(), dlv.end(),
         boost::bind(std::greater<float>(),
                     boost::bind(get_second<DLP>, _1),
                     dlv.front().second + Params::window())) ;

      // Within the window of spikes acceptable as low, find the one that
      // involves least amount of turn from current heading and vote for
      // that direction.
      DLV::const_iterator min = std::min_element(dlv.begin(), end, dir_cmp) ;
      if (Params::spin_style_steering())
      {
         C.turn = round(min->first) ;
         SpinArbiter::instance().
            vote(base::name, new SpinArbiter::Vote(C.turn)) ;
      }
      else
      {
         const int T = random(TurnArbiter::turn_step(),
                              TurnArbiter::turn_max()) ;
         switch (quadrant(min->first))
         {
            case 1:
               C.drive =  1 ; C.turn =  T ;
               break ;
            case 2:
               C.drive = -1 ; C.turn = -T ;
               break ;
            case 3:
               C.drive = -1 ; C.turn =  T ;
               break ;
            case 4:
               C.drive =  1 ; C.turn = -T ;
               break ;
            default: // quadrant() shouldn't return anything outside [1,4]
               throw misc_error(LOGIC_ERROR) ;
         }
         SpeedArbiter::instance().vote(base::name,
            new SpeedArbiter::Vote(C.drive * Params::extricate_speed(),
                                   C.drive * Params::extricate_pwm())) ;
         TurnArbiter::instance().vote(base::name,
            new TurnArbiter::Vote(turn_vote_centered_at(C.turn))) ;
      }

      // Metrics logging
      Metrics::Log log ;
      log << std::setw(Metrics::opw()) << std::left << base::name ;
      Map* M = App::map() ;
      if (M)
         log << M->current_pose() ;
   }

   // Record stuff for visualization
   viz_lock() ;
      m_cmd = C ;
   viz_unlock() ;
}

//--------------------------- VISUALIZATION -----------------------------

#ifdef INVT_HAVE_LIBGL

// The visualization callback
void LGMDExtricateSimple::render_me()
{
   // Make local copies so that extricate thread isn't held up waiting
   // for visualization thread to complete.
   viz_lock() ;
      Command C = m_cmd ;
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

   // Render the extrication decision's visualization
   if (C.drive == 0 && C.turn == 0)
      ; // no extrication took place
   else
   {
      Vector v(0.75f * cos(C.turn), 0.75f * sin(C.turn)) ;
      if (C.drive == 0) // differential steering
         glColor3f(1, 0, 0) ;
      else // normal, i.e., car-like, steering
      {
         v.i *= C.drive ;
         glColor3f(1, 1, 1) ;
      }
      draw_vector(v) ;
   }

   // Label the visualization so that it is easy to tell which behaviour
   // is being visualized. Also show the magnitudes of the forces.
   restore_view_volume() ;
   text_view_volume() ;
      glColor3f(0, 1, 1) ;
      draw_label(3, 12, "LGMD Ext. Sim.") ;
   restore_view_volume() ;
}

#endif

//----------------------------- CLEAN-UP --------------------------------

LGMDExtricateSimple::~LGMDExtricateSimple(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

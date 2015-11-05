/**
   \file  Robots/LoBot/control/LoLGMDExtricateVFF.C
   \brief This file defines the non-inline member functions of the
   lobot::LGMDExtricateVFF class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoLGMDExtricateVFF.C $
// $Id: LoLGMDExtricateVFF.C 13840 2010-08-27 22:28:12Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoLGMDExtricateVFF.H"
#include "Robots/LoBot/control/LoMetrics.H"
#include "Robots/LoBot/control/LoTurnArbiter.H"
#include "Robots/LoBot/control/LoSpinArbiter.H"
#include "Robots/LoBot/control/LoSpeedArbiter.H"

#include "Robots/LoBot/LoApp.H"
#include "Robots/LoBot/slam/LoMap.H"
#include "Robots/LoBot/lgmd/LocustModel.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/thread/LoUpdateLock.H"

#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/LoRegistry.H"
#include "Robots/LoBot/misc/singleton.hh"

#include "Robots/LoBot/util/LoGL.H"
#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/util/range.hh"

// OpenGL headers
#ifdef INVT_HAVE_LIBGL
#include <GL/gl.h>
#endif

// Standard C++ headers
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <memory>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- KNOB TWIDDLING -----------------------------

namespace {

// Retrieve settings from lgmd_extricate_vff section of config file
template<typename T>
static inline T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>(LOBE_LGMD_EXTRICATE_VFF, key, default_value) ;
}

/// This inner class encapsulates various parameters that can be used to
/// tweak different aspects of the lgmd_extricate_vff behaviour.
class Params : public singleton<Params> {
   /// A locust whose LGMD spike rate exceeds a certain threshold will
   /// result in a repulsive force along the direction in which that
   /// locust is looking. Conversely, when a locust's LGMD spike rate
   /// falls below this threshold, it will cause an attractive force.
   /// When all the forces are put together, we will get a final force
   /// vector that can be used to drive and steer the robot.
   ///
   /// This setting specifies the value of the above-mentioned
   /// threshold in spikes per second (i.e., Hz). It should be a
   /// reasonable value that lies within the [min, max] spike range
   /// specified in the chosen LGMD model's settings section. In case
   /// the robot is connected to actual locusts, then this setting's
   /// value should be some reasonable empirically determined value.
   float m_threshold ;

   /// To prevent this behaviour from becoming overly sensitive, we
   /// require the LGMD spike rates of a minimum number of locusts to
   /// exceed the above threshold before the behaviour engages its
   /// extrication algorithm.
   ///
   /// This setting specifies the above-mentioned minimum locust
   /// count. It should be a reasonable number <= the number of
   /// locusts actually present.
   int m_count ;

   /// In some situations, it can be useful to amplify the magnitudes
   /// of the attractive and repulsive force vectors. These two
   /// settings specify the values for the amplification factors.
   /// Numbers greater then one will amplify the vectors; numbers
   /// between 0 and 1 will reduce the magnitudes of the force
   /// vectors; negative numbers will negate the directions of the
   /// force vectors.
   float m_att_amp, m_rep_amp ;

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
   /// i.e., smooth car-like turns. However, the lgmd_extricate_vff
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
   /// lgmd_extricate_vff behaviour's visualization.
   Drawable::Geometry m_geometry ;

   /// Private constructor because this is a singleton.
   Params() ;

   // Boilerplate code to make generic singleton design pattern work
   friend class singleton<Params> ;

public:
   /// Accessing the various parameters.
   //@{
   static float threshold()       {return instance().m_threshold ;}
   static int   count()           {return instance().m_count     ;}
   static float att_amp()         {return instance().m_att_amp   ;}
   static float rep_amp()         {return instance().m_rep_amp   ;}
   static float extricate_speed() {return instance().m_extricate_speed ;}
   static int   extricate_pwm()   {return instance().m_extricate_pwm   ;}
   static bool  spin_style_steering(){return instance().m_spin_style_steering;}
   static int   update_delay()       {return instance().m_update_delay       ;}
   static Drawable::Geometry geometry() {return instance().m_geometry        ;}
   //@}
} ;

// Parameters initialization
Params::Params()
   : m_threshold(conf("threshold", 250)),
     m_count(conf("count", 1)),
     m_att_amp(conf("attractive_amplifier", 1.0f)),
     m_rep_amp(conf("repulsive_amplifier",  1.0f)),
     m_extricate_speed(clamp(conf("extricate_speed", 0.15f), 0.1f, 0.75f)),
     m_extricate_pwm(clamp(conf("extricate_pwm", 25), 10, 50)),
     m_spin_style_steering(conf("spin_style_steering", false)),
     m_update_delay(clamp(conf("update_delay", 150), 1, 1000)),
     m_geometry(conf<std::string>("geometry", "0 0 10 10"))
{}

} // end of local anonymous namespace encapsulating above helpers

//-------------------------- INITIALIZATION -----------------------------

LGMDExtricateVFF::LGMDExtricateVFF()
   : base(Params::update_delay(), LOBE_LGMD_EXTRICATE_VFF, Params::geometry())
{
   start(LOBE_LGMD_EXTRICATE_VFF) ;
}

LGMDExtricateVFF::Command::Command()
   : drive(0), turn(0)
{}

void LGMDExtricateVFF::pre_run()
{
   if (! App::robot())
      throw behavior_error(MOTOR_SYSTEM_MISSING) ;
}

//---------------------- THE BEHAVIOUR'S ACTION -------------------------

// Helper function object to compute the attractive and repulsive forces
// for a block and maintain a running total.
namespace {

// DEVNOTE: This function object is meant to be used with the STL
// for_each algorithm. The goal is to compute the total attractive and
// repulsive forces using the current spike rates of all the availbale
// LGMD's. Also, we want to count how many locusts have exceeded the
// preconfigured spiking threshold.
//
// Since std::for_each() passes its function object parameter by value,
// the call site will not get back the force vectors and threshold count
// computed by this function object unless special measures are taken to
// facilitate the return of these quantities.
//
// One possibility is to use object references. That is, the att, rep and
// num_rep data members defined below can be of type Vector& and int&
// respectively. In LGMDExtricateVFF::action(), we can create local
// variables for the result vectors and threshold count, passing
// references to them to this function object's constructor. Thus, when
// std::for_each() is done, LGMDExtricateVFF::action() will have the
// results.
//
// Alternatively, recalling that the STL for_each algorithm returns the
// function object passed to it at the end of the for_each() function, we
// can have the caller not discard for_each's return value (as is
// typically done) and, instead, use the function's return value to
// retrieve the force vectors and threshold count computed.
//
// This second approach is the one adopted here.
class compute_forces {
   mutable Vector att  ; // attractive force
   mutable Vector rep  ; // repulsive  force
   mutable int num_rep ; // number of repulsive vectors
public:
   compute_forces() ;
   void operator()(const LocustModel*) const ;

   const Vector& attractive()  const {return att ;}
   const Vector& repulsive()   const {return rep ;}
   const int repulsive_count() const {return num_rep ;}
} ;

compute_forces::compute_forces()
   : num_rep(0)
{}

// When the LGMD spike rate for the locust under consideration exceeds
// the spike threshold set in the config file, it means that particular
// locust has sensed an object on a collisional trajectory towards it.
// When the spike rate distance measurement is below the threshold, there
// ought not be anything nearby.
//
// If we think of the spike rate threshold as a kind of membrane, then
// the difference between an actual LGMD spike rate and the threshold is
// a measure of the amount by which the object being sensed has
// penetrated the membrane (if the difference is positive; if negative,
// then it tells us how "far away" the object is from the membrane).
//
// This difference, which we can think of as an error, can be used to
// size the force vectors. The greater the error, the greater the
// magnitude of the force.
void compute_forces::operator()(const LocustModel* L) const
{
   float lgmd = L->get_lgmd() ;
   float e = lgmd - Params::threshold() ; // threshold "error"
   Vector f = Vector(cos(L->direction()), sin(L->direction())) ;
   if (e < 0) // LGMD is below threshold
      att += -Params::att_amp() * e * f ;
   else { // LGMD spike rate is above threshold
      rep += -Params::rep_amp() * e * f ;
      ++num_rep ;
   }
}

} // end of local namespace encapsulating above helpers

// This version of the extricate behaviour monitors the LGMD spike rates
// of all the locusts and issues appropriate turn and drive commands when
// things get too close by computing virtual attractive and repulsive
// forces on the basis of the spikes.
void LGMDExtricateVFF::action()
{
   // Compute the attractive and repulsive forces and also the number of
   // LGMD's that have exceeded the spiking threshold.
   UpdateLock::begin_read() ;
      compute_forces force_field =
         std::for_each(App::locusts().begin(), App::locusts().end(),
                       compute_forces()) ;
   UpdateLock::end_read() ;

   // If the danger zone has been penetrated, use the attractive and
   // repulsive forces to determine correct motor commands.
   Command C ;
   Vector A, R, F ;
   if (force_field.repulsive_count() >= Params::count())
   {
      A = force_field.attractive() ;
      R = force_field.repulsive()  ;
      F = A + R ;

      const int T = random(TurnArbiter::turn_step(), TurnArbiter::turn_max());
      switch (quadrant(direction(F)))
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
         default: // hunh?!? quadrant() shouldn't return anything outside [1,4]
            throw misc_error(LOGIC_ERROR) ;
      }

      if (Params::spin_style_steering())
         SpinArbiter::instance().vote(base::name,
                                      new SpinArbiter::Vote(C.turn)) ;
      else
      {
         SpeedArbiter::instance().vote(base::name,
            new SpeedArbiter::Vote(C.drive * Params::extricate_speed(),
                                   C.drive * Params::extricate_pwm())) ;
         TurnArbiter::instance().vote(base::name,
            new TurnArbiter::Vote(turn_vote_centered_at(C.turn))) ;
      }

      Metrics::Log log ;
      log << std::setw(Metrics::opw()) << std::left << base::name ;
      Map* M = App::map() ;
      if (M)
         log << M->current_pose() ;
   }

   // Record stuff for visualization
   viz_lock() ;
      m_attractive  = A ;
      m_repulsive   = R ;
      m_total_force = F ;

      m_cmd = C ;
   viz_unlock() ;
}

//--------------------------- VISUALIZATION -----------------------------

#ifdef INVT_HAVE_LIBGL

// Helper function to convert the magnitude of a vector into a string,
// prefixing the supplied single character label to the mag string.
static std::string mag(char label, const Vector& v)
{
   using namespace std ;

   std::ostringstream v_str ;
   v_str << label   << ": " << fixed << setprecision(1) << magnitude(v) ;
   return v_str.str() ;
}

// The visualization callback
void LGMDExtricateVFF::render_me()
{
   // Make local copies so that extricate thread isn't held up waiting
   // for visualization thread to complete.
   viz_lock() ;
      Command C = m_cmd ;
      Vector  A = m_attractive  ;
      Vector  R = m_repulsive   ;
      Vector  T = m_total_force ;
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
      glColor3f(0, 1, 0) ;
      draw_vector(normalized(A)) ;

      glColor3f(1, 0, 0) ;
      draw_vector(normalized(R)) ;

      glColor3f(0.15f, 0.75f, 0.85f) ;
      draw_vector(normalized(T)) ;

      glColor3f(1, 1, 1) ;
      draw_vector(Vector(0.75f * C.drive * cos(C.turn), 0.75f * sin(C.turn))) ;
   }

   // Label the visualization so that it is easy to tell which behaviour
   // is being visualized. Also show the magnitudes of the forces.
   restore_view_volume() ;
   text_view_volume() ;
      glColor3f(0, 1, 1) ;
      draw_label(3, 12, "LGMD Ext. VFF") ;

      glColor3f(0, 1, 0) ;
      draw_label(3, m_geometry.height - 28, mag('A', A).c_str()) ;

      glColor3f(1, 0, 0) ;
      draw_label(3, m_geometry.height - 16, mag('R', R).c_str()) ;

      glColor3f(0.15f, 0.75f, 0.85f) ;
      draw_label(3, m_geometry.height -  4, mag('T', T).c_str()) ;
   restore_view_volume() ;
}

#endif

//----------------------------- CLEAN-UP --------------------------------

LGMDExtricateVFF::~LGMDExtricateVFF(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

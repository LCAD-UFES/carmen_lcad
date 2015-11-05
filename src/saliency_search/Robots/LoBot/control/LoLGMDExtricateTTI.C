/**
   \file  Robots/LoBot/control/LoLGMDExtricateTTI.C
   \brief This file defines the non-inline member functions of the
   lobot::LGMDExtricateTTI class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoLGMDExtricateTTI.C $
// $Id: LoLGMDExtricateTTI.C 14022 2010-09-23 18:49:14Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoLGMDExtricateTTI.H"
#include "Robots/LoBot/control/LoMetrics.H"
#include "Robots/LoBot/control/LoTurnArbiter.H"
#include "Robots/LoBot/control/LoSpinArbiter.H"
#include "Robots/LoBot/control/LoSpeedArbiter.H"

#include "Robots/LoBot/LoApp.H"
#include "Robots/LoBot/slam/LoMap.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/thread/LoUpdateLock.H"

#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/misc/LoRegistry.H"
#include "Robots/LoBot/misc/singleton.hh"

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

//-------------------------- KNOB TWIDDLING -----------------------------

namespace {

// Retrieve settings from lgmd_extricate_tti section of config file
template<typename T>
inline T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>(LOBE_LGMD_EXTRICATE_TTI, key, default_value) ;
}

/// This inner class encapsulates various parameters that can be used
/// to tweak different aspects of the lgmd_extricate_tti behaviour.
class Params : public singleton<Params> {
   /// In order to build a virtual force field made up of repulsive as
   /// well as attractive forces, we use the distance estimates plus a
   /// threshold. When a distance estimate is below this threshold it
   /// will result in a repulsive force along the direction in which
   /// that locust is looking. Conversely, when a distance estimate
   /// exceeds the same threshold, it will cause an attractive force
   /// in that locust's direction. The sum of all these force vectors
   /// will result in a vector that can be used to drive and steer the
   /// robot away from obstacles.
   ///
   /// This setting specifies the value of the above-mentioned
   /// threshold in millimeters.
   float m_threshold ;

   /// To prevent this behaviour from becoming overly sensitive, we
   /// require the distance estimates corresponding to some minimum
   /// number of locusts to fall below the above threshold before the
   /// behaviour engages its extrication algorithm.
   ///
   /// This setting specifies the minimum number of distance estimates
   /// that must fall below the distance threshold before extrication
   /// will occur. It should be a reasonable number <= the number of
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

   /// When the robot is backing up due to an extrication command, the
   /// LGMD's fire a small amount. Often, the lgmd_extricate_tti
   /// behaviour construes this as further actionable input and issues
   /// yet more back-up commands, which can result in extended periods
   /// of reverse driving. These are entirely spurious extrications and
   /// should not occur. This setting helps with the above-mentioned
   /// problem by specifying a threshold speed (in m/s) that must be met
   /// before lgmd_extricate_tti will kick in (or interfere).
   ///
   /// By setting this to a small positive quantity such as 0.05 or 0.1,
   /// we can ensure that lgmd_extricate_tti remains passive when the
   /// robot is backing up (wherein it will report a negative speed). To
   /// disable this threshold check, simply use a negative number with a
   /// magnitude larger than the robot's top speed (e.g., -10).
   ///
   /// NOTE: Using a large positive number for this setting (e.g., 10)
   /// will result in effectively disabling the lgmd_extricate_tti
   /// behaviour. In fact, setting this configuration value to something
   /// greater than the robot's average cruising speed will pretty much
   /// disable the behaviour. An easier way to do that is to simply not
   /// include it in the config file's list of active behaviours. That
   /// way, it won't uselessly consume CPU and RAM. Therefore, it would
   /// be best to stick to a value such as 0.05 or 0.1m/s.
   float m_interference_threshold ;

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

   /// Normally, this behaviour will only log an entry stating that it
   /// made a steering decision. However, with this flag, we can make it
   /// also log the states of each of its time-to-impact estimators.
   bool m_log_tti_predictions ;

   /// The number of milliseconds between successive iterations of this
   /// behaviour.
   ///
   /// WARNING: The ability to change a behaviour's update frequency is a
   /// very powerful feature whose misuse or abuse can wreak havoc! Be
   /// sure to use reasonable values for this setting.
   int m_update_delay ;

   /// The location and size (within the Robolocust main window) of the
   /// lgmd_extricate_tti behaviour's visualization.
   typedef Drawable::Geometry Geom ; // just so accessor definitions line-up
   Drawable::Geometry m_geometry ;

   /// Private constructor because this is a singleton.
   Params() ;

   // Boilerplate code to make generic singleton design pattern work
   friend class singleton<Params> ;

public:
   /// Accessing the various parameters.
   //@{
   static float threshold()          {return instance().m_threshold ;}
   static int   count()              {return instance().m_count     ;}
   static float att_amp()            {return instance().m_att_amp   ;}
   static float rep_amp()            {return instance().m_rep_amp   ;}
   static float extricate_speed()    {return instance().m_extricate_speed    ;}
   static int   extricate_pwm()      {return instance().m_extricate_pwm      ;}
   static bool  spin_style_steering(){return instance().m_spin_style_steering;}
   static bool  log_tti_predictions(){return instance().m_log_tti_predictions;}
   static int   update_delay()       {return instance().m_update_delay       ;}
   static Geom  geometry()           {return instance().m_geometry           ;}
   static float interference_threshold() {
      return instance().m_interference_threshold ;
   }
   //@}
} ;

// Parameters initialization
Params::Params()
   : m_threshold(clamp(conf("threshold", 300.0f), 50.0f, 1000.0f)),
     m_count(clamp(conf("count", 3), 1, 180)),
     m_att_amp(conf("attractive_amplifier", 1.0f)),
     m_rep_amp(conf("repulsive_amplifier", 1.0f)),
     m_interference_threshold(conf("interference_threshold", 0.1f)),
     m_extricate_speed(clamp(conf("extricate_speed", 0.15f), 0.1f, 0.75f)),
     m_extricate_pwm(clamp(conf("extricate_pwm", 25), 10, 50)),
     m_spin_style_steering(conf("spin_style_steering", false)),
     m_log_tti_predictions(conf("log_tti_predictions", false)),
     m_update_delay(clamp(conf("update_delay", 150), 1, 1000)),
     m_geometry(conf<std::string>("geometry", "0 0 -1 -1"))
{}

} // end of local anonymous namespace encapsulating above helpers

//-------------------------- INITIALIZATION -----------------------------

LGMDExtricateTTI::LGMDExtricateTTI()
   : base(Params::update_delay(), LOBE_LGMD_EXTRICATE_TTI, Params::geometry())
{
   start(LOBE_LGMD_EXTRICATE_TTI) ;
}

LGMDExtricateTTI::Command::Command()
   : drive(0), turn(0)
{}

// Before the lgmd_extricate_tti behaviour's thread begins its regular
// action processing loop, we must ensure that the sensor model for the
// Bayesian time-to-impact estimation is properly setup and that the TTI
// estimators for each locust have been created and are ready for use.
void LGMDExtricateTTI::pre_run()
{
   if (! App::robot())
      throw behavior_error(MOTOR_SYSTEM_MISSING) ;

   const App::LocustModels& L = App::locusts() ;
   const int N = L.size() ;
   m_tti.reserve(N) ;
   for (int i = 0; i < N; ++i)
      m_tti.push_back(new TTIEstimator(L[i])) ;
}

//---------------------- THE BEHAVIOUR'S ACTION -------------------------

static void log_tti_predictions(float speed, const TTIEstimator* E)
{
   using std::setw  ; using std::left ; using std::right ;
   using std::fixed ; using std::setprecision ;

   Metrics::Log log ;
   log << setw(Metrics::opw())  << left << LOBE_LGMD_EXTRICATE_TTI
       << "TTI estimator info:" << Metrics::newl() ;

   log << "     current robot speed = " << fixed << setprecision(3)
       << speed << " m/s" << Metrics::newl();

   log << "        locust direction = " << fixed << setprecision(0)
       << E->locust_direction() << " degrees" << Metrics::newl() ;
   log << "         LGMD spike rate = " << fixed << setprecision(0)
       << E->lgmd()             << " Hz" << Metrics::newl() ;

   log << "   actual time-to-impact = "
       << fixed << setprecision(1) << setw(4) << right
       << E->actual_tti()  << " seconds"<< Metrics::newl() ;
   log << "predicted time-to-impact = "
       << fixed << setprecision(1) << setw(4) << right
       << E->predicted_tti()<<" seconds"<< Metrics::newl() ;

   log << "         actual distance = "
       << fixed << setprecision(0) << setw(4) << right
       << E->actual_distance()   << " mm" << Metrics::newl() ;
   log << "      predicted distance = "
       << fixed << setprecision(0) << setw(4) << right
       << E->predicted_distance()<< " mm" << Metrics::newl() ;

   log << "   prediction confidence = " << fixed << setprecision(1)
       << E->confidence() * 100 << "%" ;
}

// This version of the extricate behaviour feeds the LGMD spikes of the
// individual locusts into time-to-impact estimators and then computes
// distances to obstacles based on these TTI estimates, which, in turn,
// are used to build a vector force field composed of attractive and
// repulsive force vectors. The sum of all the force vectors result in
// the final steering vector.
void LGMDExtricateTTI::action()
{
   // To avoid holding update lock for an extended period, copy latest
   // LGMD spike rates to TTI estimators before running Bayesian state
   // estimation update equations.
   const int N = m_tti.size() ;
   UpdateLock::begin_read() ;
      for (int i = 0; i < N; ++i)
         m_tti[i]->copy_lgmd() ;

      float speed   = App::robot()->current_speed() ;
      float heading = App::robot()->current_heading() ;
   UpdateLock::end_read() ;

   // Don't interfere with a potentially ongoing extrication...
   if (speed < Params::interference_threshold()) {
      record_viz() ; // no extrication, therefore, nothing to record
      return ;
   }

   // Update TTI estimate for each locust and use the TTI to compute
   // distance to obstacle in that locust's direction.
   Vector velocity(speed * cos(heading), speed * sin(heading)) ;
   int  count = 0 ; // number of "distance readings" below configured threshold
   for (int i = 0; i < N; ++i)
   {
      TTIEstimator* estimator = m_tti[i] ;
      estimator->update() ;
      estimator->compute_distance(velocity) ;
      float D = estimator->distance() ;
      if (D > 0 && D <= Params::threshold())
         ++count ;
      if (Params::log_tti_predictions())
         log_tti_predictions(speed, estimator) ;
   }

   // Issue motor commands if we have the requisite number of distance
   // readings below the "danger zone" threshold.
   Command C ;
   Vector A, R, F ;
   if (count >= Params::count())
   {
      // Compute attractive, repulsive and total force vectors
      for (int i = 0; i < N; ++i)
      {
         const TTIEstimator* estimator = m_tti[i] ;
         float distance = estimator->distance() ;
         if (distance < 0) // no distance estimate from this locust
            continue ;
         float e = distance - Params::threshold() ; // distance "error"
         if (e < 0) // distance reading is inside danger zone
            R += (Params::rep_amp() * e) * estimator->direction() ;
         else // distance reading is outside danger zone
            A += (Params::att_amp() * e) * estimator->direction() ;
      }
      F = A + R ;

      // Decide drive and turn commands based on above force field
      const int T = random(TurnArbiter::turn_step(), TurnArbiter::turn_max()) ;
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
   record_viz(A, R, F, C) ;
}

// Record stuff for visualization
void
LGMDExtricateTTI::
record_viz(const Vector& att, const Vector& rep, const Vector& tot,
           const LGMDExtricateTTI::Command& cmd)
{
   viz_lock() ;
      m_attractive  = att ;
      m_repulsive   = rep ;
      m_total_force = tot ;
      m_cmd = cmd ;
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

// This function renders the extrication decision as well as the faux
// distance "readings" returned by the LGMD "range" sensor.
void LGMDExtricateTTI::render_me()
{
   // Make local copies of required visualization info so that extricate
   // thread isn't held up waiting for visualization to complete.
   const int N = m_tti.size() ;

   typedef std::pair<Vector, float> DDP ; // direction-distance pair
   std::vector<DDP> ddp ;
   ddp.reserve(N) ;
   float normalizer = std::numeric_limits<float>::min() ;

   viz_lock() ;
      Command C  = m_cmd ;
      Vector  A  = m_attractive  ;
      Vector  R  = m_repulsive   ;
      Vector  T  = m_total_force ;

      for (int i = 0; i < N; ++i) {
         ddp.push_back(DDP(m_tti[i]->direction(), m_tti[i]->distance())) ;
         if (normalizer < m_tti[i]->distance())
            normalizer = m_tti[i]->distance() ;
      }
   viz_unlock() ;
   normalizer = 1/normalizer ;

   // Draw principal axes to make it easier to understand what's going on
   // with the force vectors and turn command.
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
      // Render the TTI-based distance estimates
      glBegin(GL_LINES) ;
         for (int i = 0; i < N; ++i)
         {
            if (ddp[i].second < 0) // no distance estimate in this direction
               continue ;
            Vector d = (normalizer * ddp[i].second) * ddp[i].first ;
            if (ddp[i].second > Params::threshold())
               glColor3f(0, 0.8f, 0.2f) ; // greenish
            else
               glColor3f(0.8f, 0, 0.2f) ; // reddish
            glVertex2i(0, 0) ;
            glVertex2f(d.i, d.j) ;
         }
      glEnd() ;

      // Render force field vectors
      glColor3f(0, 1, 0) ;
      draw_vector(normalized(A)) ;

      glColor3f(1, 0, 0) ;
      draw_vector(normalized(R)) ;

      glColor3f(0.15f, 0.75f, 0.85f) ;
      draw_vector(normalized(T)) ;

      // Render drive and turn commands
      glColor3f(1, 1, 1) ;
      draw_vector(Vector(0.75f * C.drive * cos(C.turn), 0.75f * sin(C.turn))) ;
   }

   // Label the visualization so that it is easy to tell which behaviour
   // is being visualized. Also show the magnitudes of the forces.
   restore_view_volume() ;
   text_view_volume() ;
      glColor3f(0, 1, 1) ;
      draw_label(3, 12, "LGMD Ext. TTI") ;

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

LGMDExtricateTTI::~LGMDExtricateTTI()
{
   purge_container(m_tti) ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

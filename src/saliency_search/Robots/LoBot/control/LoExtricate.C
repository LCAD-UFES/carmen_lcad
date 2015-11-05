/**
   \file  Robots/LoBot/control/LoExtricate.C
   \brief This file defines the non-inline member functions of the
   lobot::Extricate class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoExtricate.C $
// $Id: LoExtricate.C 13840 2010-08-27 22:28:12Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoExtricate.H"
#include "Robots/LoBot/control/LoMetrics.H"
#include "Robots/LoBot/control/LoTurnArbiter.H"
#include "Robots/LoBot/control/LoSpinArbiter.H"
#include "Robots/LoBot/control/LoSpeedArbiter.H"

#include "Robots/LoBot/LoApp.H"
#include "Robots/LoBot/slam/LoMap.H"
#include "Robots/LoBot/io/LoDangerZone.H"
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
#include <memory>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- KNOB TWIDDLING -----------------------------

namespace {

// Retrieve settings from extricate section of config file
template<typename T>
static inline T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>(LOBE_EXTRICATE, key, default_value) ;
}

/// This local class encapsulates various parameters that can be used to
/// tweak different aspects of the extricate behaviour.
class Params : public singleton<Params> {
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

   /// When lobot is running the LGMD-based avoidance behaviours, it can
   /// come to a complete halt due to the emergency stop behaviour and
   /// then not restart because the lack of motion will result in the
   /// LGMD spikes becoming dormant. To work around this problem, we can
   /// run the LRF-based extricate behaviour in addition to the
   /// LGMD-based extricate behaviours to restart the robot so as to
   /// start the spiking activity again.
   ///
   /// However, in this capacity, it would be best to reduce the
   /// frequency at which the LRF-based extricate behaviour runs.
   /// Furthermore, we would also want this behaviour to only perform its
   /// extrication actions if the robot is actually stopped.
   ///
   /// This setting is a flag that indicates whether the extricate
   /// behaviour should operate in "restart mode" as described above. By
   /// default, it is off, i.e., we assume this behaviour is the only
   /// active extricate behaviour and that it should perform extrication
   /// whenever the danger zone is penetrated, regardless of the robot's
   /// current motion state.
   bool m_restart_mode ;

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
   friend class singleton<Params> ;

public:
   /// Accessing the various parameters
   //@{
   static float extricate_speed()    {return instance().m_extricate_speed    ;}
   static int   extricate_pwm()      {return instance().m_extricate_pwm      ;}
   static bool  restart_mode()       {return instance().m_restart_mode       ;}
   static bool  spin_style_steering(){return instance().m_spin_style_steering;}
   static int   update_delay()       {return instance().m_update_delay       ;}
   static Geom  geometry()           {return instance().m_geometry           ;}
   //@}
} ;

// Parameter initialization
Params::Params()
   : m_extricate_speed(clamp(conf("extricate_speed", 0.15f), 0.1f, 0.75f)),
     m_extricate_pwm(clamp(conf("extricate_pwm", 25), 10, 50)),
     m_restart_mode(conf("restart_mode", false)),
     m_spin_style_steering(conf("spin_style_steering", false)),
     m_update_delay(clamp(conf("update_delay", 1750), 500, 10000)),
     m_geometry(conf<std::string>("geometry", "0 0 10 10"))
{}

} // end of local anonymous namespace encapsulating above helpers

//-------------------------- INITIALIZATION -----------------------------

Extricate::Extricate()
   : base(Params::update_delay(), LOBE_EXTRICATE, Params::geometry())
{
   start(LOBE_EXTRICATE) ;
}

Extricate::Command::Command()
   : drive(0), turn(0)
{}

void Extricate::pre_run()
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
// repulsive forces for all of the DangerZone's blocks one block at a
// time. Since std::for_each() passes its function object parameter by
// value, the call site will not get back the force vectors computed by
// this function object unless special measures are taken to facilitate
// the return of these vectors.
//
// One possibility is to use object references. That is, the att and rep
// data members defined below can be of type Vector& and, in
// Extricate::action(), we can create the result vectors and pass them to
// this function object as references (via the constructor). Thus, when
// std::for_each() is done, Extricate::action() will have the results.
//
// Alternatively, recalling that the STL for_each algorithm returns the
// function object passed to it at the end of the for_each() function, we
// can have the caller not discard for_each's return value (as is
// typically done) and, instead, use the function's return value to
// retrieve the force vectors computed for all the DangerZone blocks.
//
// This second approach is the one adopted here.
class compute_forces {
   const LRFData& lrf ;
   mutable Vector att ; // attractive force
   mutable Vector rep ; // repulsive  force
public:
   compute_forces(const LRFData&) ;
   void operator()(const DangerZone::Block&) const ;

   const Vector& attractive() const {return att ;}
   const Vector& repulsive()  const {return rep ;}
} ;

compute_forces::compute_forces(const LRFData& L)
   : lrf(L)
{}

// Each DangerZone::Block contains a distance threshold. When the LRF
// returns a distance measurement less than this threshold, it means that
// particular reading indicates something inside the danger zone. When
// the distance measurement is greater than the block's distance
// threshold, the object being sensed lies outside the danger zone.
//
// If we think of a block's distance threshold as a kind of membrane,
// then the difference between a distance reading and the distance
// threshold for the block to which that reading belongs is a measure of
// the amount by which the object being sensed has penetrated the
// membrane (if the difference is negative; if positive, then it tells us
// how far away the object is from the membrane).
//
// This difference, or distance "error", can be used to size the force
// vectors. The greater the error, the greater the magnitude of the
// force.
//
// Since the distance thresholds for a block will be fairly small
// relative to the total distance range that can be sensed by the laser
// range finder, the magnitudes of negative distance errors (i.e., when
// an object has penetrated the danger zone) will be much smaller than
// the magnitudes of positive errors (especially for objects far away on
// one side of the robot). For example, the Hokuyo URG-LX-01 can sense
// distances in the range 60mm to 5600mm; and distance thresholds for the
// robot are usually in the range of 250-450mm. So, if the robot is near
// a wall on its right but has a clear path to its left and in front, the
// total magnitude of the attractive force will be quite high while that
// of the repulsive force will be rather low.
//
// To even out this lopsidedness, we square the errors for repulsive
// forces but not for attractive forces. The force vector is then
// computed simply by multiplying this scalar value and the unit vector
// for each LRF reading's angle.
void compute_forces::operator()(const DangerZone::Block& B) const
{
   for (int angle = B.start(); angle <= B.end(); ++angle)
   {
      int distance = lrf[angle] ;
      if (distance < 0) // bad reading along this direction
         continue ;
      float  e = distance - B.danger_zone() ; // distance "error"
      Vector f = Vector(cos(angle), sin(angle)) ;
      if (e < 0) // distance reading is inside danger zone
         rep += sqr(e) * f ;
      else // distance reading is outside danger zone
         att += e * f ;
   }
}

} // end of local namespace encapsulating above helpers

// The extricate behaviour monitors the robot's danger zone and issues
// appropriate turn and drive commands to get the robot unstuck, which
// happens when things have gotten too close (thereby triggering the
// robot's emergency stop response).
//
// This behaviour works by assigning a repulsive force to each distance
// reading that lies within the danger zone and an attractive force to
// each reading outside it. The total force vector is then used to
// determine suitable motor commands.
void Extricate::action()
{
   DangerZone::Blocks blocks ;
   std::auto_ptr<LRFData> lrf ;

   // Copy the danger zone blocks and corresponding LRF data so as to not
   // hold the update lock for too long.
   UpdateLock::begin_read() ;
      bool danger_zone_penetrated = DangerZone::penetrated() ;
      if (danger_zone_penetrated) {
         blocks.reserve(DangerZone::num_blocks()) ;
         std::copy(DangerZone::begin(), DangerZone::end(),
                   std::back_inserter(blocks)) ;
         lrf.reset(new LRFData(DangerZone::lrf_data())) ;
      }
      bool stopped = App::robot()->stopped() ;
   UpdateLock::end_read() ;

   // If the danger zone has been penetrated, determine the attractive
   // and repulsive forces and the corresponding motor commands. In
   // restart mode, also check if the robot is actually stopped before
   // commencing extrication.
   //
   // DEVNOTE: To understand the logic of the if statement below, let:
   //      D = Boolean variable indicating that danger zone is penetrated
   //      M = Boolean variable indicating beh. is configured for restart mode
   //      S = Boolean variable indicating robot is stopped
   // and  E = Boolean variable indicating extrication should be performed
   //
   // In normal mode, i.e., when R is false, we only need check D. In
   // restart mode, we have to check both D and S. Thus, utilizing the
   // symbols of Boolean algebra, we have:
   //      E = ~MD + MSD
   //
   // Simplifying the above, gives us:
   //      E = D(~M + MS) = D(~M + S)
   Command C ;
   Vector A, R, F ;
   if (danger_zone_penetrated && (! Params::restart_mode() || stopped))
   {
      compute_forces force_field =
         std::for_each(blocks.begin(), blocks.end(), compute_forces(*lrf)) ;
      A = force_field.attractive() ;
      R = force_field.repulsive()  ;
      F = A - R ;

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
   v_str << label   << ": "
         << setw(8) << fixed << setprecision(1) << magnitude(v) ;
   return v_str.str() ;
}

// The visualization callback
void Extricate::render_me()
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
      draw_label(3, 12, "Extricate") ;

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

Extricate::~Extricate(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

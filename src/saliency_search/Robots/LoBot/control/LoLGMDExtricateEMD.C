/**
   \file  Robots/LoBot/control/LoLGMDExtricateEMD.C
   \brief This file defines the non-inline member functions of the
   lobot::LGMDExtricateEMD class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoLGMDExtricateEMD.C $
// $Id: LoLGMDExtricateEMD.C 13840 2010-08-27 22:28:12Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoLGMDExtricateEMD.H"
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
#include <memory>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- KNOB TWIDDLING -----------------------------

namespace {

// Retrieve settings from extricate section of config file
template<typename T>
inline T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>(LOBE_LGMD_EXTRICATE_EMD, key, default_value) ;
}

/// This local class encapsulates various parameters that can be used
/// to tweak different aspects of the lgmd_extricate_emd behaviour.
class Params : public singleton<Params> {
   /// This behaviour's extrication commands will kick in only when the
   /// magnitude of the spiking vector computed by the EMD array exceeds
   /// the threshold specified by this setting.
   float m_threshold ;

   /// Users may specify what speed they would like the extrication to
   /// occur.
   ///
   /// CAUTION: It would be unwise to make this speed too high.
   float m_extricate_speed ;

   /// In case the RPM sensor is configured to be off, we will need to
   /// specify the extricate "speed" in terms of PWM values as well.
   ///
   /// NOTE: All speed related behaviours should specify both a speed in
   /// m/s and a PWM value.
   ///
   /// CAUTION: For the extricate behaviour, it would be unwise to make
   /// the extrication PWM too high.
   int m_extricate_pwm ;

   /// Usually, steering control is effected using the turn arbiter,
   /// which veers the robot towards different direction while it moves.
   /// However, the lgmd_extricate behaviour also supports spin-style
   /// steering, i.e., momentarily stopping the robot and then turning it
   /// cw/ccw in-place. This flag turns on spin-style steering. By
   /// default, the behaviour uses the normal steering mode.
   bool m_spin_style_steering ;

   /// The number of milliseconds between successive iterations of this
   /// behaviour.
   ///
   /// WARNING: The ability to change a behaviour's update frequency is a
   /// very powerful feature whose misuse or abuse can wreak havoc! Be
   /// sure to use reasonable values for this setting.
   int m_update_delay ;

   /// The location and size (within the Robolocust main window) of the
   /// lgmd_extricate_emd behaviour's visualization.
   Drawable::Geometry m_geometry ;

   /// Private constructor because this is a singleton.
   Params() ;

   // Boilerplate code to make generic singleton design pattern work
   friend class singleton<Params> ;

public:
   /// Accessing the various parameters.
   //@{
   static float threshold()          {return instance().m_threshold ;}
   static float extricate_speed()    {return instance().m_extricate_speed    ;}
   static int   extricate_pwm()      {return instance().m_extricate_pwm      ;}
   static bool  spin_style_steering(){return instance().m_spin_style_steering;}
   static int   update_delay()       {return instance().m_update_delay       ;}
   static Drawable::Geometry geometry() {return instance().m_geometry        ;}
   //@}
} ;

// Parameters initialization
Params::Params()
   : m_threshold(conf("threshold", 1000.0f)),
     m_extricate_speed(clamp(conf("extricate_speed", 0.15f), 0.1f, 0.75f)),
     m_extricate_pwm(clamp(conf("extricate_pwm", 25), 10, 50)),
     m_spin_style_steering(conf("spin_style_steering", false)),
     m_update_delay(clamp(conf("update_delay", 150), 1, 1000)),
     m_geometry(conf<std::string>("geometry", "0 0 10 10"))
{}

} // end of local anonymous namespace encapsulating above helpers

//-------------------------- INITIALIZATION -----------------------------

LGMDExtricateEMD::LGMDExtricateEMD()
   : base(Params::update_delay(), LOBE_LGMD_EXTRICATE_EMD, Params::geometry())
{
   start(LOBE_LGMD_EXTRICATE_EMD) ;
}

void LGMDExtricateEMD::pre_run()
{
   if (! App::robot())
      throw behavior_error(MOTOR_SYSTEM_MISSING) ;

   UpdateLock::begin_read() ;
      const App::LocustModels& L = App::locusts() ;
      const int N = L.size() - 1 ;
      if (N < 1) {
         UpdateLock::end_read() ;
         throw behavior_error(NOT_ENOUGH_LOCUSTS) ;
      }

      m_emds.reserve(N) ;
      for (int i = 0; i < N; ++i) {
         float direction = (L[i]->direction() + L[i + 1]->direction())/2.0f ;
         m_emds.push_back(new LMD(L[i], L[i + 1], direction)) ;
      }
   UpdateLock::end_read() ;
}

LGMDExtricateEMD::Command::Command()
   : drive(0), turn(0)
{}

//---------------------- THE BEHAVIOUR'S ACTION -------------------------

// This version of the extricate behaviour feeds the LGMD spikes of all
// the locusts into an array of elementary motion detectors and issues
// appropriate turn and drive commands based on the output of the EMD
// array.
void LGMDExtricateEMD::action()
{
   // Use EMD array to determine the direction in which maximal spiking
   // activity is currently ocurring.
   UpdateLock::begin_read() ;
      const int N = m_emds.size() ;
      Vector emd ;
      for (int i = 0; i < N; ++i)
         emd += m_emds[i]->update() ;
   UpdateLock::end_read() ;

   // Issue motor commands based on EMD vector's direction and magnitude
   Command C ;
   if (magnitude(emd) >= Params::threshold())
   {
      const int T = random(TurnArbiter::turn_step(), TurnArbiter::turn_max());
      int s = 0 ; // in-place turn amount when spin-style steering is in effect
      switch (octant(direction(emd)))
      {
         case 1: // obstacle in front and on the left
            C.drive = -1 ; C.turn =  T ; s = -T ;
            break ;
         case 2: // obstacle on the left
         case 3: // obstacle on the left
         case 4: // obstacle behind and on the left
            C.drive =  1 ; C.turn = -T ; s = -T ;
            break ;
         case 5: // obstacle behind and on the right
         case 6: // obstacle on the right
         case 7: // obstacle on the right
            C.drive =  1 ; C.turn =  T ; s = T ;
            break ;
         case 8: // obstacle in front and on the right
            C.drive = -1 ; C.turn = -T ; s = T ;
            break ;
         default: // hunh?!? octant() shouldn't return anything outside [1,8]
            throw misc_error(LOGIC_ERROR) ;
      }

      if (Params::spin_style_steering())
         SpinArbiter::instance().vote(base::name, new SpinArbiter::Vote(s)) ;
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
      m_emd_vector = emd ;
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
void LGMDExtricateEMD::render_me()
{
   // Make local copies so that extricate thread isn't held up waiting
   // for visualization thread to complete.
   viz_lock() ;
      Command C = m_cmd ;
      Vector  E = m_emd_vector ;
   viz_unlock() ;

   // Draw principal axes and diagonals marking the octants (to make it
   // easier to understand what's going on with the EMD vector and turn
   // command).
   unit_view_volume() ;
   glColor3f(1, 1, 1) ;
   glBegin(GL_LINES) ;
      glVertex2i(0, -1) ;
      glVertex2i(0,  1) ;
      glVertex2i(-1, 0) ;
      glVertex2i( 1, 0) ;
   glEnd() ;

   glPushAttrib(GL_LINE_BIT) ;
      glEnable(GL_LINE_STIPPLE) ;
      glLineStipple(1, 0x0F0F) ;
      glBegin(GL_LINES) ;
         glVertex2i(-1, +1) ;
         glVertex2i(+1, -1) ;

         glVertex2i(+1, +1) ;
         glVertex2i(-1, -1) ;
      glEnd() ;
   glPopAttrib() ;

   // Render the extrication decision's visualization
   if (C.drive == 0 && C.turn == 0)
      ; // no extrication took place
   else
   {
      glColor3f(1, 0, 0) ;
      draw_vector(normalized(E)) ;

      glColor3f(0, 1, 0) ;
      draw_vector(Vector(0.75f * C.drive * cos(C.turn), 0.75f * sin(C.turn))) ;
   }

   // Label the visualization so that it is easy to tell which behaviour
   // is being visualized. Also show the magnitudes of the forces.
   restore_view_volume() ;
   text_view_volume() ;
      glColor3f(0, 1, 1) ;
      draw_label(3, 12, "LGMD Ext. EMD") ;

      glColor3f(0.15f, 0.75f, 0.85f) ;
      draw_label(3, m_geometry.height -  4, mag('E', E).c_str()) ;
   restore_view_volume() ;
}

#endif

//----------------------------- CLEAN-UP --------------------------------

LGMDExtricateEMD::~LGMDExtricateEMD()
{
   purge_container(m_emds) ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

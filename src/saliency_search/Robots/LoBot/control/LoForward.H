/**
   \file  Robots/LoBot/control/LoForward.H
   \brief A behaviour for driving the robot forward.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoForward.H $
// $Id: LoForward.H 13037 2010-03-23 01:00:53Z mviswana $
//

#ifndef LOBOT_FORWARD_BEHAVIOUR_DOT_H
#define LOBOT_FORWARD_BEHAVIOUR_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoBehavior.H"
#include "Robots/LoBot/control/LoTurnArbiter.H"
#include "Robots/LoBot/control/LoSpeedArbiter.H"

#include "Robots/LoBot/io/LoLRFData.H"

#include "Robots/LoBot/misc/factory.hh"
#include "Robots/LoBot/misc/singleton.hh"
#include "Robots/LoBot/util/range.hh"

// Standard C++ headers
#include <utility>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Forward
   \brief A behaviour for driving the robot forward.

   This class implements a simple behaviour for driving the robot forward
   at the configured cruising speed.
*/
class Forward : public Behavior {
   // Prevent copy and assignment
   Forward(const Forward&) ;
   Forward& operator=(const Forward&) ;

   // Handy type to have around in a derived class
   typedef Behavior base ;

   // Boilerplate code to make the generic factory design pattern work
   friend  class subfactory<Forward, base> ;
   typedef register_factory<Forward, base> my_factory ;
   static  my_factory register_me ;

   /// This behaviour features an adaptive mode, wherein it regulates the
   /// robot's speed based on the distance to the closest obstacle. To be
   /// able to properly visualize this mode of operation, it is useful to
   /// keep track of the particular reading that resulted in the adaptive
   /// speed value. This structure holds the distance reading plus speed
   /// vote together in one place.
   typedef std::pair<LRFData::Reading, SpeedArbiter::Vote*> SpeedInfo ;

   /// To aid with development and debugging, this behaviour supports a
   /// visualization callback, which needs the most recent votes so that
   /// it can perform the proper visualization.
   //@{
    TurnArbiter::Vote m_turn_vote  ;
   SpeedArbiter::Vote m_speed_vote ;
   LRFData::Reading m_min_distance ;
   //@}

   /// A private constructor because behaviours are instantiated with an
   /// object factory and not directly by clients.
   Forward() ;

   /// Stuff to do before regular action processing begins.
   void pre_run() ;

   /// These methods implement this behaviour's action.
   //@{
   void action() ;
   SpeedInfo fixed_speed() const ;
   SpeedInfo adaptive_speed() const ;
   //@}

   /// Visualization routines to aid with development and debugging.
   void render_me() ;

   /// Clean-up.
   ~Forward() ;

   /// This inner class encapsulates various parameters that can be used
   /// to tweak different aspects of the forward driving behaviour.
   class Params : public singleton<Params> {
      /// The forward behaviour always attempts to drive the robot
      /// forward at the specified cruising speed (specified in meters
      /// per second).
      float m_cruising_speed ;

      /// If lobot is configured to ignore its RPM sensor, then speed
      /// related behaviours must specify their drive commands in terms
      /// of motor PWM values. Thus, in addition to a cruising speed, the
      /// forward behaviour must also be configured with a cruising PWM.
      int m_cruising_pwm ;

      /// Instead of driving at a fixed (cruising) speed, the forward
      /// driving behaviour can be configured to regulate the robot's
      /// speed based on the distance to the closest obstacle. This flag
      /// turns on adaptive driving mode.
      bool m_adaptive_mode ;

      /// In adaptive mode, the behaviour will consider the distance
      /// readings in the angular range specified by this setting.
      range<int> m_fov ;

      /// Adaptive mode works by adjusting the robot's speed according to
      /// the minimum distance reading in the FOV specified by the above
      /// setting. However, the behaviour does not consider the "raw"
      /// distance readings; rather it averages the distance readings in
      /// a small block about each angle it considers, starting at the
      /// FOV's minimum and going up to the FOV maximum in steps
      /// specified by this setting.
      ///
      /// For example, if the FOV is [-30, 30] and the averaging block is
      /// 10, the forward behaviour will look at the average distances in
      /// the following blocks: [-35, -25], [-25, -15], [-5, -15], [-5, 5],
      /// [5, 15], [15, 25] and [25, 35].
      int m_averaging_block ;

      /// Once the minimum distance reading has been found, adaptive mode
      /// will compute a corresponding speed using a simple linear
      /// interpolation using the minimum distance reading, the distance
      /// range setting (see below) and the speed range (this setting's
      /// value).
      range<float> m_speed_range ;

      /// Since all speed votes are required to work in terms of both PWM
      /// values and m/s speeds, we also need a PWM range so that
      /// adaptive mode can compute an appropriate PWM value using the
      /// same linear interpolation described above.
      range<int> m_pwm_range ;

      /// To "convert" LRF distance readings into corresponding speed
      /// values, we use a linear interpolation that involves the
      /// minimum distance reading, the distance range (this setting)
      /// and the speed range (defined above).
      range<int> m_distance_range ;

      /// Private constructor because this is a singleton.
      Params() ;

      // Boilerplate code to make generic singleton design pattern work
      friend class singleton<Params> ;

   public:
      /// Accessing the various parameters.
      //@{
      static float cruising_speed() {return instance().m_cruising_speed ;}
      static int   cruising_pwm()   {return instance().m_cruising_pwm   ;}
      static bool  adaptive_mode()  {return instance().m_adaptive_mode  ;}
      static range<int> fov()       {return instance().m_fov ;}
      static int   averaging_block(){return instance().m_averaging_block;}
      static range<float> speed_range() {return instance().m_speed_range;}
      static range<int>   pwm_range()   {return instance().m_pwm_range  ;}
      static range<int> distance_range(){return instance().m_distance_range ;}
      //@}

      /// Clean-up.
      ~Params() ;
   } ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

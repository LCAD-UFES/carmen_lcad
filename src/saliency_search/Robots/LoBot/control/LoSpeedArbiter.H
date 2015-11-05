/**
   \file  Robots/LoBot/control/LoSpeedArbiter.H
   \brief An arbiter for issuing speed commands to the robot.

   This file defines a class that implements a DAMN speed arbiter for
   Robolocust.
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
// Primary maintainer for this file: Manu Viswanathan mviswana usc edu
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoSpeedArbiter.H $
// $Id: LoSpeedArbiter.H 12827 2010-02-12 00:17:25Z mviswana $
//

#ifndef LOBOT_SPEED_ARBITER_DOT_H
#define LOBOT_SPEED_ARBITER_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoArbiter.H"
#include "Robots/LoBot/misc/singleton.hh"

// Standard C++ headers
#include <string>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::SpeedArbiter
   \brief A DAMN speed arbiter for controlling Robolocust's speed.

   This class implements a DAMN speed arbiter that acts as the interface
   between the Robolocust behaviours and the robot's speed controls. Each
   behaviour that wants to influence the robot's driving speed will have
   to vote by specifying an acceptable maximum speed value. The arbiter
   will issue the motor control command for the highest priority
   behaviour.
*/
class SpeedArbiter : public Arbiter, public singleton<SpeedArbiter> {
   // Prevent copy and assignment
   SpeedArbiter(const SpeedArbiter&) ;
   SpeedArbiter& operator=(const SpeedArbiter&) ;

   // Boilerplate code to make the generic singleton design pattern work
   friend class singleton<SpeedArbiter> ;

   /// A private constructor because this class is a singleton.
   SpeedArbiter() ;

   /// Retrieve the user-assigned priority for the given behaviour.
   float get_configured_priority(const std::string& behaviour) const ;

   /// Tally votes and issue appropriate motor command.
   void motor_cmd(const Votes&, Robot*) ;

public:
   /// To control the robot's speed, each speed related behaviour must
   /// vote for the maximum acceptable speed. These votes are represented
   /// by this inner class. In order to vote, a behaviour must
   /// instantiate this class with the new operator, fill out the voting
   /// structure properly and then pass it to the arbiter's vote()
   /// method.
   class Vote : public VoteBase {
      /// A vote is simply a number indicating the maximum acceptable
      /// speed (in m/s).
      float m_speed ;

      /// Depending on how lobot is configured, its drive commands may
      /// require behaviours to specify motor PWM values directly rather
      /// than have the motor system compute PWM values corresponding to
      /// commanded velocities on the basis of the RPM sensor. Therefore,
      /// all speed related behaviours must also supply appropriate PWM
      /// values in addition to a m/s speed value.
      int m_pwm ;

   public:
      /// Filling out a speed arbiter vote object simply involves passing
      /// the desired speed to the constructor.
      Vote(float speed = 0, int pwm = 0) ;

      /// Retrieving the specified speed.
      float speed() const {return m_speed ;}

      /// Retrieving the specified PWM.
      int pwm() const {return m_pwm ;}
   } ;

private:
   /// The Arbiter class maintains the votes in a list of vote_data*. To
   /// find the vote corresponding to the behaviour with the maximum
   /// priority, we need a custom function object for use with the
   /// std::max_element() algorithm.
   ///
   /// DEVNOTE: Since vote_data is protected in the lobot::Arbiter base
   /// class, this comparator need to be a member function object rather
   /// than a local helper in LoSpeedArbiter.C.
   class compare_priorities {
      const SpeedArbiter* arbiter ;
   public:
      compare_priorities(const SpeedArbiter*) ;
      bool operator()(const vote_data* a, const vote_data* b) const ;
   } ;

   /// Speed arbiter clean-up.
   ~SpeedArbiter() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

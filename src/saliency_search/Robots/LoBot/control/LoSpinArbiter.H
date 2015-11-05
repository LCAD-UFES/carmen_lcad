/**
   \file  Robots/LoBot/control/LoSpinArbiter.H
   \brief An arbiter for issuing spin commands to the robot.

   This file defines a class that implements a DAMN spin arbiter for
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
// Primary maintainer for this file: mviswana usc edu
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoSpinArbiter.H $
// $Id: LoSpinArbiter.H 13770 2010-08-08 06:49:50Z mviswana $
//

#ifndef LOBOT_SPIN_ARBITER_DOT_H
#define LOBOT_SPIN_ARBITER_DOT_H

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
   \class lobot::SpinArbiter
   \brief A DAMN spin arbiter for steering Robolocust using in-place
   turns.

   This class implements a DAMN arbiter that acts as the interface
   between the Robolocust behaviours and the robot's in-place turn
   commands. Each behaviour that wants to steer the robot using in-place
   turns will have to vote by specifying a desired rotation amount. The
   arbiter will issue the appropriate motor control command by combining
   the votes of all the different behaviours.
*/
class SpinArbiter : public Arbiter, public singleton<SpinArbiter> {
   // Prevent copy and assignment
   SpinArbiter(const SpinArbiter&) ;
   SpinArbiter& operator=(const SpinArbiter&) ;

   // Boilerplate code to make the generic singleton design pattern work
   friend class singleton<SpinArbiter> ;

   /// A private constructor because this class is a singleton.
   SpinArbiter() ;

   /// Retrieve the user-assigned priority for the given behaviour.
   float get_configured_priority(const std::string& behaviour) const ;

   /// Tally votes and issue appropriate motor command.
   void motor_cmd(const Votes&, Robot*) ;

   /// Spin arbiter clean-up.
   ~SpinArbiter() ;

public:
   /// To spin the robot, each behaviour must vote for the desired amount
   /// of rotation. These votes are represented by this inner class. In
   /// order to vote, a behaviour must instantiate this class with the
   /// new operator, fill out the voting structure properly and then pass
   /// it to the arbiter's vote() method.
   class Vote : public VoteBase {
      /// A vote is simply a number indicating the desired amount of
      /// rotation (in degrees). This value should be a number in the
      /// range [-360, 360]. Negative values result in clockwise spins
      /// while positive ones are for counterclockwise spins.
      float m_spin ;

   public:
      /// Filling out a speed arbiter vote object simply involves passing
      /// the desired spin amount to the constructor.
      Vote(float spin = 0) ;

      /// Retrieving the specified spin.
      float spin() const {return m_spin ;}
   } ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

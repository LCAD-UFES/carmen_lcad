/**
   \file  Robots/LoBot/control/LoRemoteControl.H

   \brief A behaviour for controlling getting the iRobot Roomba/Create
   using its remote.
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
// Primary maintainer for this file: Manu Viswanathan <mviswana at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoRemoteControl.H $
// $Id: LoRemoteControl.H 12708 2010-01-28 18:55:12Z mviswana $
//

#ifndef LOBOT_REMOTE_CONTROL_BEHAVIOUR_DOT_H
#define LOBOT_REMOTE_CONTROL_BEHAVIOUR_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoBehavior.H"

#include "Robots/LoBot/misc/factory.hh"
#include "Robots/LoBot/misc/singleton.hh"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::RemoteControl

   \brief A behaviour for controlling the iRobot Roomba/Create using its
   remote.

   This class implements a behaviour that uses the Roomba Remote to drive
   lobot. This can be useful, for instance, if the robot is about to do
   something dangerous or if it is doing something stupid (e.g.,
   repetetive failures to extricate from a corner).

   The behaviour works by freezing the arbiters and then issuing drive
   and turn commands based on the sensor data sent by the robot. If the
   user does not supply any remote control commands for more than some
   (configured) timeout, the behaviour will release the arbiters so that
   the other behaviours can start working normally again.
*/
class RemoteControl : public Behavior {
   // Prevent copy and assignment
   RemoteControl(const RemoteControl&) ;
   RemoteControl& operator=(const RemoteControl&) ;

   // Handy type to have around in a derived class
   typedef Behavior base ;

   // Boilerplate code to make the generic factory design pattern work
   friend  class subfactory<RemoteControl, base> ;
   typedef register_factory<RemoteControl, base> my_factory ;
   static  my_factory register_me ;

   /// The remote control behaviour operates in one of two states: either
   /// the robot is working autonomously or it is being remote
   /// controlled. The behaviour starts off in the autonomous state. When
   /// a remote control command is received, it freezes the arbiters and
   /// transitions to the remote control state. If no more remote control
   /// commands come in within the configured timeout, the behaviour will
   /// release the arbiters and return to the autonomous state.
   //@{
   enum State {
      AUTONOMOUS,
      REMOTE_CONTROL,
   } ;
   State m_state ;
   long long m_time ;
   //@}

   /// A private constructor because behaviours are instantiated with an
   /// object factory and not directly by clients.
   RemoteControl() ;

   /// Some things to do before commencing regular action processing.
   void pre_run() ;

   /// This method implements the behaviour's extrication strategy. As
   /// mentioned earlier, it works by determining the kind of structure
   /// from which the robot needs to escape and then issues appropriate
   /// motor command votes to get the robot unstuck from that structure.
   void action() ;

   /// This is a helper method that executes the command specified by the
   /// remote control.
   void execute(int infrared_byte) ;

   /// Clean-up.
   ~RemoteControl() ;

   /// This inner class encapsulates various parameters that can be used
   /// to tweak different aspects of the remote control behaviour.
   class Params : public singleton<Params> {
      /// The remote control behaviour works by listening for remote
      /// control commands. These commands must be sent continuously for
      /// the behaviour to retain control, i.e., the user must keep
      /// pressing the buttons on the Roomba Remote in order to maintain
      /// manual control of the robot. When these commands stop coming
      /// in, the behaviour relinquishes control and lets the other
      /// behaviours function normally.
      ///
      /// This setting specifies the timeout between successive remote
      /// control commands. Thus, if the remote control behaviour does
      /// not sense any remote control commands for this much time, it
      /// will relinquish control of the robot.
      ///
      /// This setting should be a time expressed in milliseconds.
      int m_timeout ;

      /// Users may specify the driving speed for remote control
      /// commands. The speed should be a positive floating point number
      /// not to exceed 0.5 (the Roomba's top speed). Its unit is m/s.
      float m_drive_speed ;

      /// Remote control turn commands can use a different speed setting
      /// than the drive commands. This setting specifies the turn speed.
      /// It is expeted to be a positive floating point number whose unit
      /// is m/s and whose value should not exceed 0.5 (the Roomba's top
      /// speed).
      float m_turn_speed ;

      /// Private constructor because this is a singleton.
      Params() ;

      // Boilerplate code to make generic singleton design pattern work
      friend class singleton<Params> ;

   public:
      /// Accessing the various parameters.
      //@{
      static int   timeout()     {return instance().m_timeout ;}
      static float drive_speed() {return instance().m_drive_speed ;}
      static float turn_speed()  {return instance().m_turn_speed  ;}
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

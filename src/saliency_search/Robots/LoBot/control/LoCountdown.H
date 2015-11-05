/**
   \file  Robots/LoBot/control/LoCountdown.H
   \brief A stop watch behaviour that measures the time elapsed and quits
   the high-level controller's application when the count hits zero.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoCountdown.H $
// $Id: LoCountdown.H 14305 2010-12-08 21:17:33Z mviswana $
//

#ifndef LOBOT_COUNTDOWN_BEHAVIOUR_DOT_H
#define LOBOT_COUNTDOWN_BEHAVIOUR_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoBehavior.H"
#include "Robots/LoBot/misc/factory.hh"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Countdown
   \brief A "stop watch" behaviour that either pauses the robot or quits
   the controller application once the counter hits zero.

   This class implements a behaviour that keeps track of how much time
   has elapsed since the robot started running. When this time exceeds
   some preconfigured duration, the behaviour will quit the high-level
   controller application.

   NOTE: The stop watch continues to count down even if the robot is
   paused!
*/
class Countdown : public Behavior {
   // Prevent copy and assignment
   Countdown(const Countdown&) ;
   Countdown& operator=(const Countdown&) ;

   // Handy type to have around in a derived class
   typedef Behavior base ;

   // Boilerplate code to make the generic factory design pattern work
   friend  class subfactory<Countdown, base> ;
   typedef register_factory<Countdown, base> my_factory ;
   static  my_factory register_me ;

   /// This variable keeps track of when the behaviour was started.
   long long m_time ;

   /// A private constructor because behaviours are instantiated with an
   /// object factory and not directly by clients.
   Countdown() ;

   /// Record the time when the behaviour starts running.
   void pre_run() ;

   /// This method provides the body of the behaviour's main loop.
   void action() ;

   /// Clean-up.
   ~Countdown() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/**
   \file  Robots/LoBot/control/LoBumpCounter.H
   \brief A simple behaviour to log each low-level bump sensor event.

   This file defines a class that implements a very simple behaviour
   whose sole job is to log each low-level bump sensor event to the
   Robolocust metrics log. The intent here is to be able to keep track of
   all "catastrophic" failures of the LGMD-based extrication algorithms,
   viz., when they do not detect obstacles in time and allow the robot to
   physically bump into things.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoBumpCounter.H $
// $Id: LoBumpCounter.H 13786 2010-08-13 00:26:38Z mviswana $
//

#ifndef LOBOT_BUMP_COUNTER_BEHAVIOUR_DOT_H
#define LOBOT_BUMP_COUNTER_BEHAVIOUR_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoBehavior.H"
#include "Robots/LoBot/io/LoRobot.H"
#include "Robots/LoBot/misc/factory.hh"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::BumpCounter
   \brief A behaviour for "counting" bump events.

   This class implements a behaviour that logs all low-level bump sensor
   events to the Robolocust metrics log so that, at the end of a
   run/experiment, we can tell how many times and where all the
   LGMD-based (and, potentially, other) behaviours failed to get the
   robot to avoid obstacles.

   NOTE: This behaviour does not vote for any motor actions or perform
   any visualization.
*/
class BumpCounter : public Behavior {
   // Prevent copy and assignment
   BumpCounter(const BumpCounter&) ;
   BumpCounter& operator=(const BumpCounter&) ;

   // Handy type to have around in a derived class
   typedef Behavior base ;

   // Boilerplate code to make the generic factory design pattern work
   friend  class subfactory<BumpCounter, base> ;
   typedef register_factory<BumpCounter, base> my_factory ;
   static  my_factory register_me ;

   /// A private constructor because behaviours are instantiated with an
   /// object factory and not directly by clients.
   BumpCounter() ;

   /// Some things to do before commencing regular action processing.
   void pre_run() ;

   /// This is the callback function for the low-level updates of the
   /// robot's bump sensors.
   static void sensor_hook(const Robot::Sensors&, unsigned long client_data) ;

   /// This method triggers the SLAM update using the latest control and
   /// sensor inputs. It is called by the run function, which takes care
   /// of the necessary signaling/synchronization with the main thread.
   void action() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

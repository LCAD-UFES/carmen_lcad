/**
   \file  Robots/LoBot/control/LoSurvey.H
   \brief A cartographic behaviour, i.e., a behaviour that builds the
   obstacle map used by Robolocust.

   This file defines a class that implements a map-making behaviour. This
   behaviour uses a SLAM algorithm to build a map and record the robot's
   trajectory within this map.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoSurvey.H $
// $Id: LoSurvey.H 13523 2010-06-07 00:22:03Z mviswana $
//

#ifndef LOBOT_SURVEY_BEHAVIOUR_DOT_H
#define LOBOT_SURVEY_BEHAVIOUR_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoBehavior.H"

#include "Robots/LoBot/slam/LoFastSLAM.H"
#include "Robots/LoBot/slam/LoOdometry.H"

#include "Robots/LoBot/io/LoRobot.H"
#include "Robots/LoBot/thread/LoCondition.H"
#include "Robots/LoBot/misc/factory.hh"

// Standard C++ headers
#include <memory>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Survey

   \brief A behaviour for updating the Robolocust occupancy grid.

   This class implements a behaviour that uses a SLAM algorithm to build
   an occupancy map of lobot's environment and record its trajectory
   within this map.

   NOTE: This behaviour does not vote for any motor actions.
*/
class Survey : public Behavior {
   // Prevent copy and assignment
   Survey(const Survey&) ;
   Survey& operator=(const Survey&) ;

   // Handy type to have around in a derived class
   typedef Behavior base ;

   // Boilerplate code to make the generic factory design pattern work
   friend  class subfactory<Survey, base> ;
   typedef register_factory<Survey, base> my_factory ;
   static  my_factory register_me ;

   /// The survey behaviour uses a SLAM algorithm to build a map of the
   /// robot's surroundings and keep track of its trajectory.
   std::auto_ptr<FastSLAM> m_slam ;

   /// Since SLAM updates can be fairly intense and take a while to
   /// complete, this class uses a flag to keep track of when SLAM is in
   /// progress and when it is not. This is necessitated by the fact that
   /// odometric updates take place in the main thread. If SLAM is
   /// currently in progress, then the low-level odometric updates should
   /// be accumulated till the SLAM update is done. After the SLAM update
   /// is complete, it can be retriggered with the accumulated odometry.
   bool m_slam_busy ;

   /// This data member keeps track of the low-level odometry. It
   /// accumulates the low-level odometry packets. When the SLAM
   /// algorithm is ready for its next iteration, the Survey behaviour
   /// will pass the accumulated odometry to the SLAM module and reset
   /// this variable so that it can start accumulating odometry again.
   Odometry m_odometry ;

   /// As mentioned above, since SLAM updates can take a while, we must
   /// accumulate low-level odometry until the SLAM algorithm is ready to
   /// run its next iteration. The m_odometry data member is used to
   /// accumulate odometry. When the SLAM module is ready, the contents
   /// of m_odometry will be copied to this data member, viz., ut, and
   /// m_odometry will be reset so it can start accumulating odometry
   /// afresh.
   ///
   /// NOTE: This data member is named ut to denote the latest control
   /// input u at time t to the SLAM algorithm.
   Odometry ut ;

   /// Low-level odometric updates occur in the main thread whereas SLAM
   /// takes place in the survey behaviour's thread. To ensure that the
   /// SLAM algorithm is invoked with minimal amounts of odometric
   /// accumulation, we use a condition variable to signal the survey
   /// behaviour's thread whenever the main thread receives a low-level
   /// odometry packet.
   Condition m_odometry_cond ;

   /// The lobot::Condition class requires a function or function object
   /// to be passed to its waiting and signaling methods. This inner
   /// helper class is used to implement a function object that can be
   /// used in conjunction with lobot::Condition.
   ///
   /// This class takes two type parameters:
   ///
   ///     F -- function or function object implementing the predicate
   ///          required by lobot::Condition
   ///     R -- the return type for F; usually bool
   ///
   /// On instantiation, the cond_helper should be given a reference to
   /// the Survey object and a function or function object that will
   /// implement the desired condition variable test. When its function
   /// call operator is invoked, cond_helper will in turn call F, passing
   /// it the Survey object, and will return to lobot::Condition whatever
   /// F returns.
   ///
   /// Thus, this helper class acts as an intermediary and a binder. In
   /// its role as an intermediary, it transmits/forwards
   /// lobot::Condition's predicate call to F and F's return value to
   /// lobot::Condition. In its role as a binder, it binds the Survey
   /// object to F, since any meaninful synchronization actions between
   /// the main and survey behaviour threads will require a Survey object
   /// whereas the generic lobot::Condition interface calls functions
   /// without any arguments.
   template<typename F, typename R = bool>
   class cond_helper {
      Survey& survey ;
      F pred ;
   public:
      cond_helper(Survey&, F) ;
      R operator()() {return pred(survey) ;}
   } ;

   // The cond_helper needs access to the Survey class's innards to
   // properly keep track of odometry, the SLAM algorithm's state, etc.
   template<typename F, typename R> friend class cond_helper ;

   /// This inner class acts as a predicate for signaling the Survey
   /// behaviour's thread when new odometry is available. If the SLAM
   /// algorithm is currently busy, it will take care of accumulating the
   /// odometry. Otherwise, it will check if the odometric thresholds set
   /// by the user have been crossed and, if so, will return true to
   /// trigger the SLAM update.
   class odometry_helper {
      int distance, angle ;
   public:
      odometry_helper(int dist, int ang) ;
      bool operator()(Survey&) ;
   } ;

   /// This inner class acts as a predicate for blocking the Survey
   /// behaviour's thread until the configured odometric thresholds have
   /// been crossed. Once this condition is met, the Survey behaviour
   /// will trigger the next SLAM update. Before the update is triggered,
   /// the SLAM algorithm will be marked busy.
   struct threshold_helper {
      bool operator()(Survey&) ;
   } ;

   /// This inner class acts as a function object for resetting the state
   /// of the SLAM module. When this flag is cleared, the main thread's
   /// odometric accumulation will cease and the Survey behaviour will be
   /// signaled to trigger the next SLAM update.
   struct reset_helper {
      void operator()(Survey&) ;
   } ;

   /// Since it can be cumbersome to directly instantiate the cond_helper
   /// template, we use these helper functions to return the desired
   /// object with all the correct types and parameters.
   //@{
   cond_helper<odometry_helper> odometry_update(int dist, int ang) {
      return cond_helper<odometry_helper>(*this, odometry_helper(dist, ang)) ;
   }

   cond_helper<threshold_helper> threshold_check() {
      return cond_helper<threshold_helper>(*this, threshold_helper()) ;
   }

   cond_helper<reset_helper, void> reset_slam_busy_flag() {
      return cond_helper<reset_helper, void>(*this, reset_helper()) ;
   }
   //@}

   /// A private constructor because behaviours are instantiated with an
   /// object factory and not directly by clients.
   Survey() ;

   /// Most behaviours rely on the base class to implement the action
   /// processing loop in lobot::Behavior::run(). By default, that
   /// function checks the lobot::Shutdown signal and calls the virtual
   /// action() method in a loop. Successive iterations are separated by
   /// a short sleep.
   ///
   /// However, for the survey behaviour, we need to override the base
   /// class's main loop because a sleep-based loop is inappropriate. If
   /// we were to rely on the default run function, then we could
   /// potentially accumulate quite a bit of odometry before performing a
   /// SLAM update. In general, we would like to invoke the SLAM
   /// algorithm with minimal odometric accumulation so that the map and
   /// pose get updated with small, incremental motions rather than in
   /// relatively large jumps.
   ///
   /// Therefore, this class needs to provide its own implementation of
   /// the run function.
   void run() ;

   /// Some things to do before commencing regular action processing.
   void pre_run() ;

   /// This is the callback function for the low-level updates of the
   /// robot's odometry.
   static void sensor_hook(const Robot::Sensors&, unsigned long client_data) ;

   /// This function keeps track of the robot's low-level odometry. It
   /// executes in the context of the main thread. Thus, to let the
   /// survey behaviour know that new odometry is available, it uses the
   /// condition variable m_odometry_cond to signal the survey
   /// behaviour's thread.
   void accumulate(int distance, int angle) ;

   /// This method triggers the SLAM update using the latest control and
   /// sensor inputs. It is called by the run function, which takes care
   /// of the necessary signaling/synchronization with the main thread.
   void action() ;

   /// Visualization support: functions for rendering the FastSLAM
   /// algorithm's particles on the application-wide map so we can see
   /// how FastSLAM is working...
   //@{
   static void render_particles(unsigned long client_data) ;
   void render_particles() ;
   //@}

   /// Clean-up.
   ~Survey() ;
} ;

//------------------- TEMPLATE FUNCTION DEFINITIONS ---------------------

// Constructor for the Survey behaviour's odometry condition variable's
// helper function object.
template<typename F, typename R>
Survey::cond_helper<F,R>::cond_helper(Survey& s, F f)
   : survey(s), pred(f)
{}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

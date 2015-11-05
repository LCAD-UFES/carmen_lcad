/**
   \file  Robots/LoBot/control/LoBehavior.H
   \brief An ABC for Robolocust behaviours.

   This file defines an abstract base class that provides a common
   interface for the different behaviours that constitute lobot's
   behaviour-based robot controller.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoBehavior.H $
// $Id: LoBehavior.H 13608 2010-06-23 04:27:01Z mviswana $
//

#ifndef LOBOT_BEHAVIOR_DOT_H
#define LOBOT_BEHAVIOR_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoDrawable.H"
#include "Robots/LoBot/thread/LoThread.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Behavior
   \brief An ABC defining the common interface for all of lobot's
   behaviours.

   This class implements an abstract base class for all the behaviours
   that make up the multithreaded Robolocust behaviour-based control
   system. Each behaviour is a goal-directed action that ties together
   a sensorimotor loop of some sort and runs in a separate thread.
   Behaviours are instantiated by the main thread during Robolocust's
   start-up sequence via an object factory. Which behaviours the robot
   controller should run are specified in the config file.

   Each behaviour's main loop should take care to coordinate accesses to
   various shared sensors and other resources using the Robolocust
   UpdateLock.

   Also, since a behaviour runs in its own thread, all subclasses *must*
   call Thread::start() in their constructors.
*/
class Behavior : public Drawable, private Thread {
   // Prevent copy and assignment
   Behavior(const Behavior&) ;
   Behavior& operator=(const Behavior&) ;

public:
   /// Each behaviour has a name that must be set by the client module.
   ///
   /// NOTE: Being a public data member, this field is rife with
   /// possibilities for abuse. But, hopefully, clients will refrain from
   /// doing so (after all, what's the point in making life harder for
   /// yourself?). The intent here is to set this field when the
   /// behaviour is instantiated and leave it the hell alone thereafter.
   std::string name ;

protected:
   /// Every behaviour must wait a bit after its action method is
   /// executed before the next iteration. This update delay is a user
   /// setting specified in the config file. The delay is expected to be
   /// in milliseconds.
   ///
   /// CAUTION: Since the update delay for each behaviour is
   /// customizable, it is possible for users to completely ruin the
   /// lobot controller by providing bizarre values. Therefore, each
   /// behaviour is expected to guard itself against such weirdness.
   int m_update_delay ;

   /// A protected constructor because behaviours are instantiated with
   /// an object factory and not directly by clients.
   ///
   /// Derived classes must specify an appropriate update delay to use
   /// when they invoke this constructor. It is up to each individual
   /// behaviour to guard against possibly catastrophic update delay
   /// settings in the config file and provide reasonable defaults and
   /// boundaries.
   ///
   /// Optionally, behaviours interested in visualization may provide the
   /// names and geometries for their respective drawables.
   Behavior(int update_delay,
            const std::string& drawable_name = "",
            const Drawable::Geometry& = Drawable::Geometry()) ;

   // Since we're using private inheritance, Thread::start() won't be
   // visible to subclasses without this explicit say-so.
   using Thread::start ;

   /// This method implements the behaviour's main loop, taking care of
   /// checking with the lobot::Shutdown object whether or not it's time
   /// to quit.
   ///
   /// NOTE: Derived classes may but generally should not provide their
   /// own implementations of the Thread::run() method. If, for some
   /// reason, a subclass of lobot::Behavior needs to define its own
   /// version of run(), it should be sure to check the status of the
   /// lobot::Shutdown object.
   void run() ;

   /// This method implements the behaviour's action. Each behaviour must
   /// provide its own custom implementation of this function.
   ///
   /// NOTE: This function is called by run() and is already ensconced
   /// within a loop. Derived classes should *not* run their own infinite
   /// or main-loop type loops in this method. Instead, they have to
   /// provide only the *body* of the main loop. That is, the
   /// Behavior::run() method implements the actual main loop of the
   /// behaviour while the derived class merely provides the individual
   /// steps that go into that main loop.
   ///
   /// Also: Behavior subclasses should use UpdateLock as required in
   /// their definitions of the action() method to ensure proper access
   /// to shared sensor objects and other resources.
   virtual void action() = 0 ;

   /// These methods provides derived classes hooks for implementing any
   /// pre- and post-run operations. pre_run() is called right before the
   /// main loop is entered and post_run() right after it is exited.
   ///
   /// Derived classes are not required to define these functions. The
   /// default versions implemented in the Behavior base class do
   /// nothing.
   //@{
   virtual void pre_run() ;
   virtual void post_run() ;
   //@}

public:
   /// Clean-up.
   virtual ~Behavior() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

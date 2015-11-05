/**
   \file  Robots/LoBot/control/LoArbiter.H
   \brief An ABC for Robolocust motor control arbiters.

   This file defines an abstract base class that provides a common
   interface for the different kinds of arbiters that are used to control
   lobot's motors.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoArbiter.H $
// $Id: LoArbiter.H 13521 2010-06-06 14:23:03Z mviswana $
//

#ifndef LOBOT_ARBITER_DOT_H
#define LOBOT_ARBITER_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/ui/LoDrawable.H"
#include "Robots/LoBot/io/LoRobot.H"
#include "Robots/LoBot/thread/LoThread.H"

// POSIX threads
#ifdef INVT_HAVE_LIBPTHREAD

#include <pthread.h>

#else // fake pthreads API to allow builds to succeed

typedef int pthread_mutex_t ;

#endif

// Standard C++ headers
#include <string>
#include <map>
#include <list>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Arbiter
   \brief An ABC defining the common interface for lobot's motor control
   arbiters.

   This class implements an abstract base class that provides a common
   interface for a behaviour arbitration mechanism. The Robolocust
   arbiter follows Rosenblatt's ideas as described in his Ph.D. thesis on
   the Distributed Architecture for Mobile Navigation (DAMN).

   Thus, Robolocust behaviours do not directly control the motors or
   engage in such nefarious activities as suppressing or inhibiting each
   other (a la subsumption). Instead, they vote for each of the different
   possible motor control commands. A central arbiter tallies votes using
   a weighted sum and smoothing procedure that results in command fusion
   (as opposed to sensor fusion) and then issues the command with the
   highest vote.
*/
class Arbiter : public Drawable, private Thread {
   // Prevent copy and assignment
   Arbiter(const Arbiter&) ;
   Arbiter& operator=(const Arbiter&) ;

   /// After each iteration of the main loop implemented in the run()
   /// method, the arbiter will pause a while. This update delay is a
   /// user setting specified in the config file. The delay is expected
   /// to be in milliseconds.
   ///
   /// CAUTION: Since the update delay for each arbiter is customizable,
   /// it is possible for users to completely ruin the lobot controller
   /// by providing bizarre values. Therefore, each arbiter is expected
   /// to guard itself against such weirdness.
   int m_update_delay ;

protected:
   /// A protected constructor because only subclasses should be able to
   /// invoke it. Clients cannot directly create arbiters.
   ///
   /// Derived classes must specify an appropriate update delay to use
   /// when they invoke this constructor. It is up to each individual
   /// arbiter to guard against possibly catastrophic update delay
   /// settings in the config file and provide reasonable defaults and
   /// boundaries.
   ///
   /// Optionally, arbiters interested in visualization may provide the
   /// names and geometries for their respective drawables.
   Arbiter(int update_delay,
           const std::string& drawable_name = "",
           const Drawable::Geometry& = Drawable::Geometry()) ;

   /// This method implements the arbiter's main loop, taking care of
   /// checking with the lobot::Shutdown object whether or not it's time
   /// to quit.
   ///
   /// NOTE: Derived classes may but generally should not provide their
   /// own implementations of the Thread::run() method. If, for some
   /// reason, a subclass of lobot::Arbiter needs to define its own
   /// version of run(), it should be sure to check the status of the
   /// lobot::Shutdown object.
   void run() ;

   /// Since an arbiter runs in its own thread, all subclasses *must*
   /// call Thread::start() in their constructors. However, because of
   /// private inheritance, Thread::start() won't be visible to
   /// subclasses without this explicit say-so.
   using Thread::start ;

   /// These methods provide derived classes hooks for implementing any
   /// pre- and post-run operations. pre_run() is called right before the
   /// main loop is entered and post_run() right after it is exited.
   ///
   /// Derived classes are not required to define these functions. The
   /// default versions implemented in the Arbiter base class do nothing.
   //@{
   virtual void pre_run() ;
   virtual void post_run() ;
   //@}

private:
   /// In order to perform command fusion properly, the arbiter needs to
   /// know each behaviour's priority. Behaviour priorities are assigned
   /// by users. User-specified values are usually not normalized, which
   /// is why we need to maintain this map.
   //@{
   typedef std::map<std::string, float> PriorityMap ;
   PriorityMap m_priorities ;
   //@}

   /// This method populates the behaviours priority map when the arbiter
   /// thread starts up.
   void init_priorities() ;

   /// Retrieve the priority associated with the given behaviour. Each
   /// Arbiter subclass must implement this method. Usually, it would
   /// involve a lookup in the Robolocust configuration database.
   virtual float get_configured_priority(const std::string& beh) const = 0 ;

protected:
   /// Returns the normalized priority value stored in the behaviours
   /// priority map.
   float priority(const std::string& behaviour_name) const ;

public:
   /// Sometimes, a behaviour might want/need exclusive control over the
   /// robot's actuators. To facilitate this, the freeze() method freezes
   /// the arbiter's priority to that of the behaviour identified by the
   /// given name. Once frozen, the arbiter will ignore votes cast by
   /// behaviours whose priorities are lower than the priority at which
   /// the arbiter is frozen.
   ///
   /// After its need is fulfilled, the behaviour that has frozen the
   /// arbiter must unfreeze it to resume normal operation.
   ///
   /// NOTE: Freezing the arbiter goes against the grain of the DAMN
   /// paradigm. This feature is meant for occasional use by high
   /// priority behaviours that might have a need for implementing a
   /// sequence of actions without "interference" from other behaviours.
   /// Use this feature only if absolutely necessary.
   //@{
   void freeze  (const std::string& name) ;
   void unfreeze(const std::string& name) ;
   //@}

   /// Check if the named behaviour has frozen the arbiter.
   bool is_frozen(const std::string& name) const ;

private:
   /// These data members keep track of the arbiter's freeze state.
   //@{
   std::string m_freezer ;         ///< behaviour that has frozen arbiter
   float       m_freeze_priority ; ///< priority at which arbiter is frozen
   //@}

   /// Because the freeze state can be accessed by multiple threads, we
   /// need to synchronize accesses, which we do with this mutex.
   mutable pthread_mutex_t m_freeze_mutex ;

protected:
   /// In the DAMN paradigm to robot control, the behaviours do not
   /// directly issue motor commands. Rather they vote for or against the
   /// available motor commands. Then, the DAMN arbiter issues the motor
   /// command by tallying votes and performing appropriate command
   /// fusions.
   ///
   /// Different types of arbiters will have different voting semantics
   /// and provide their own vote structures. However, all these vote
   /// structures should be derived from this one, i.e.,
   /// Arbiter::VoteBase.
   ///
   /// NOTE: This inner class, which serves as a common base for all
   /// arbiter vote types, exists only to provide a virtual destructor so
   /// that the Arbiter base class can properly clean up its vote list.
   struct VoteBase {
      virtual ~VoteBase() ;
   } ;

   /// This inner class is used to hold some vote metadata plus the vote
   /// itself (in terms of a VoteBase pointer).
   ///
   /// NOTE: This class's data members are all public. However, Arbiter
   /// subclasses should treat it as a read-only structure.
   struct vote_data {
      std::string behavior_name ;
      long long   vote_time ;
      VoteBase*   vote ;

      vote_data(const std::string&, long long, VoteBase*) ;
      ~vote_data() ;
   } ;

   /// The arbiter maintains all the votes in a list of this type. The
   /// list is held privately by the base (i.e., this) class and passed
   /// to subclasses as part of the motor_cmd() method.
   typedef std::list<vote_data*> Votes ;

private:
   /// All the votes are stored in this list and tallied in the arbiter's
   /// main loop.
   Votes m_votes ;

   /// When a behaviour casts its vote for some set of motor commands,
   /// the vote gets added to the arbiter's list of votes. After the
   /// arbiter is done tallying votes, the list is purged prior to
   /// starting the next cycle. Since behaviours and arbiters run in
   /// separate threads, it is imperative to protect against simultaneous
   /// accesses to the votes list.
   pthread_mutex_t m_votes_mutex ;

public:
   /// Behaviours use this method to cast their votes.
   void vote(const std::string& name, VoteBase* vote) ;

protected:
   /// The DAMN arbiter's main loop (implemented in Arbiter::run()) is
   /// responsible for tallying the available votes and issuing the
   /// appropriae motor command. However, since each type of arbiter can
   /// have different voting semantics, the base class cannot tally votes
   /// and issue the motor commands. That must be done by subclasses.
   /// This method performs the vote tallying and issuance of motor
   /// commands. It must be implemented by each subclass.
   ///
   /// NOTE: Subclasses must treat the votes list as read-only. Although
   /// this list is passed to the subclass as const object, its nodes are
   /// actually up for grabs and can be modified. However, subclasses
   /// must refrain from editing the individual vote_data* nodes held by
   /// the votes list. Otherwise, Bad Things will happen.
   virtual void motor_cmd(const Votes&, Robot*) = 0 ;

public:
   /// Clean-up.
   virtual ~Arbiter() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

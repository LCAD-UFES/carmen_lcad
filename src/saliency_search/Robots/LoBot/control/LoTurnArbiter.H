/**
   \file  Robots/LoBot/control/LoTurnArbiter.H
   \brief An arbiter for issuing turn commands to steer the robot.

   This file defines a class that implements a DAMN turn arbiter for
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoTurnArbiter.H $
// $Id: LoTurnArbiter.H 12858 2010-02-17 20:35:10Z mviswana $
//

#ifndef LOBOT_TURN_ARBITER_DOT_H
#define LOBOT_TURN_ARBITER_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoArbiter.H"
#include "Robots/LoBot/misc/singleton.hh"

// Standard C++ headers
#include <string>
#include <map>
#include <vector>
#include <iterator>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::TurnArbiter
   \brief A DAMN turn arbiter for controlling Robolocust's steering.

   This class implements a DAMN turn arbiter that acts as the interface
   between the Robolocust behaviours and the robot's steering controls.
   The arbiter supports a certain set of turns (hard right, medium right,
   soft right, straight ahead, etc.). Each behaviour that wants to
   influence the steering direction will have to vote for or against each
   of these possible steering commands. The turn arbiter will then tally
   all the votes using a weighted sum and smoothing procedure and issue
   the motor control command that ends up with the maximum votes.
*/
class TurnArbiter : public Arbiter, public singleton<TurnArbiter> {
   // Prevent copy and assignment
   TurnArbiter(const TurnArbiter&) ;
   TurnArbiter& operator=(const TurnArbiter&) ;

   // Boilerplate code to make the generic singleton design pattern work
   friend class singleton<TurnArbiter> ;

   /// A private constructor because this class is a singleton.
   TurnArbiter() ;

   /// Retrieve the user-assigned priority for the given behaviour.
   float get_configured_priority(const std::string& behaviour) const ;

   /// Tally votes and issue appropriate motor command.
   void motor_cmd(const Votes&, Robot*) ;

public:
   /// To control the robot's steering, each turn related behaviour must
   /// vote for or against each possible turn direction. These votes are
   /// represented by this inner class. In order to vote, a behaviour
   /// must instantiate this class with the new operator, fill out the
   /// voting structure properly and then pass it to the arbiter's vote()
   /// method.
   ///
   /// A vote is a number between -1 and +1. If a behaviour votes -1 for
   /// some direction, it means that the behaviour is dead-set against
   /// turning in that direction; a vote of +1 indicates a strong
   /// preference for going in that direction; and a vote of zero means
   /// the behaviour is neutral with regards to that direction.
   /// Fractional numbers indicate varying degrees between the three
   /// states described above. For example, an obstacle avoidance
   /// behaviour might scale vote values based on the distance to
   /// obstacles.
   class Vote : public VoteBase {
      /// We store the votes for each turn direction supported by the
      /// controller in a map that maps directions to their corresponding
      /// vote values.
      //@{
      typedef std::map<int, float> VoteMap ;
      VoteMap m_votes ;
      //@}

      // Allow the turn arbiter to access VoteMap
      friend class TurnArbiter ;

   public:
      /// When a new turn arbiter vote object is created, we initialize
      /// the internal turn-direction-to-vote-value map by using the
      /// appropriate config file parameters.
      Vote() ;

      /// Retrieve the supported turn directions in a vector.
      std::vector<int> get_directions() const ;

      /// Operator to access the vote value corresponding to the supplied
      /// direction. If the turn direction is not supported by the
      /// arbiter, an exception will be thrown.
      VoteMap::mapped_type& operator[](int direction) ;

      /// After creating a new turn arbiter vote object, behaviours can
      /// use this method to specify their votes for a given direction.
      void vote(int direction, float vote_value) {
         operator[](direction) = vote_value ;
      }

      // Forward declarations
      class iterator ;
      friend class iterator ; // because it needs to muck around with vote map

      /// An iterator interface for filling out votes for all the
      /// directions.
      class iterator {
         /// Each Vote iterator has to be associated with a Vote object.
         Vote& m_vote ;

         /// A Vote iterator keeps track of itself simply by reusing an
         /// iterator from the Vote's vote map.
         mutable Vote::VoteMap::iterator m_iterator ;

         /// Private constructors to ensure that only the Vote class can
         /// create Vote iterators.
         //@{
         iterator(const Vote&) ;
         iterator(const Vote&, bool) ;
         friend class Vote ;
         //@}

      public:
         /// Copy, assignment and clean-up for turn arbiter vote object
         /// iterators.
         //@{
         iterator(const iterator&) ;
         iterator& operator=(const iterator&) ;
         ~iterator() ;
         //@}

         /// Typedefs for STL compatibility.
         //@{
         typedef std::bidirectional_iterator_tag iterator_category ;
         typedef Vote::VoteMap::mapped_type value_type ;
         typedef int difference_type ;
         typedef value_type* pointer ;
         typedef value_type& reference ;
         //@}

         /// Item access
         //@{
               reference operator*()        {return   m_iterator->second ;}
         const reference operator*()  const {return   m_iterator->second ;}
               pointer   operator->()       {return & m_iterator->second ;}
         const pointer   operator->() const {return & m_iterator->second ;}
         //@}

         /// Prefix increment
         //@{
               iterator& operator++()       {++m_iterator ; return *this ;}
         const iterator& operator++() const {++m_iterator ; return *this ;}
         //@}

         /// Postfix increment
         //@{
         iterator operator++(int) {
            iterator tmp(*this) ;
            ++*this ;
            return tmp ;
         }
         const iterator operator++(int) const {
            iterator tmp(*this) ;
            ++*this ;
            return tmp ;
         }
         //@}

         /// Prefix decrement
         //@{
               iterator& operator--()       {--m_iterator ; return *this ;}
         const iterator& operator--() const {--m_iterator ; return *this ;}
         //@}

         /// Postfix decrement
         //@{
         iterator operator--(int) {
            iterator tmp(*this) ;
            --*this ;
            return tmp ;
         }
         const iterator operator--(int) const {
            iterator tmp(*this) ;
            --*this ;
            return tmp ;
         }
         //@}

         /// Relational operators
         //@{
         operator bool() const {
            return m_iterator != m_vote.m_votes.end() ;
         }
         bool operator==(const iterator& it) const {
            return m_iterator == it.m_iterator ;
         }
         bool operator!=(const iterator& it) const {
            return ! operator==(it) ;
         }
         //@}

         /// Additional functions for Vote object iterators.
         //@{
         const Vote::VoteMap::key_type& direction() const {
            return m_iterator->first ;
         }
         const Vote::VoteMap::mapped_type& value()  const {
            return operator*() ;
         }
         //@}
      } ;

      /// Obtaining iterators for the vote object.
      //@{
      iterator begin() {return iterator(*this) ;}
      iterator end()   {return iterator(*this, true) ;}
      const iterator begin() const {return iterator(*this) ;}
      const iterator end()   const {return iterator(*this, true) ;}
      //@}

      /// An operator to add one vote to another.
      Vote& operator+=(const Vote& v) ;

      /// When many votes are added together, the result can go out of
      /// the [-1, +1] range. This method normalizes such votes so that
      /// all directions get a vote in the proper range.
      void normalize() ;

      /// Normalization requires finding the current min and max votes.
      /// However, sometimes, clients might obligingly have already done
      /// this. This method can be used when the current min and max vote
      /// values are known beforehand.
      void normalize(float min, float max) ;

      /// Helpers to return the turn direction parameters.
      //@{
      int num_directions() const {return m_votes.size() ;}
      //@}

      /// Turn arbiter vote clean-up.
      ~Vote() ;

      /// Debug support
      void dump(const std::string& caller = "Vote::dump") const ;
   } ;

private:
   /// To aid with development and debugging, this arbiter supports a
   /// visualization callback, which needs the most recent vote so that
   /// it can perform the proper visualization.
   Vote m_vote ;

   /// Visualization routines to aid with development and debugging.
   void render_me() ;

public:
   /// Turn arbiter clean-up.
   ~TurnArbiter() ;

private:
   /// This inner class encapsulates various parameters that can be used
   /// to tweak different aspects of the turn arbiter.
   class Params : public singleton<Params> {
      /// The turn arbiter maintains a set of turn directions in which it
      /// can command the motors to go. These directions are specified
      /// with a max value and a step value. For example, max and step
      /// values of 30 and 10 would indicate that the robot can be
      /// commanded to turn in the directions corresponding to -30, -20,
      /// -10, 0, 10, 20 and 30 degrees.
      int m_turn_max, m_turn_step ;

      /// The turn arbiter tallies all the votes by applying a weighted
      /// sum procedure (where the weights are the behaviour priorities).
      /// It then smooths the resulting weighted sum by applying a
      /// Gaussian to adjacent vote values. The following parameter
      /// specifies the size of the smoothing window, i.e., it specifies
      /// how many neighbouring vote values should be considered while
      /// smoothing each one.
      int m_smoothing_width ;

      /// This parameter specifies the standard deviation to use for the
      /// Gaussian smoothing mentioned above. Since the Gaussian operates
      /// in turn command space, this standard deviation is in degrees.
      float m_sigma ;

      /// Private constructor because this is a singleton.
      Params() ;

      // Boilerplate code to make generic singleton design pattern work
      friend class singleton<Params> ;

   public:
      /// Accessing the various parameters.
      //@{
      static int   turn_max()        {return instance().m_turn_max  ;}
      static int   turn_step()       {return instance().m_turn_step ;}
      static int   smoothing_width() {return instance().m_smoothing_width ;}
      static float sigma()           {return instance().m_sigma ;}
      //@}

      /// Clean-up.
      ~Params() ;
   } ;

public:
   /// Helpers to return some turn arbiter parameters used by other
   /// modules.
   //@{
   static int turn_max()  {return Params::turn_max()  ;}
   static int turn_step() {return Params::turn_step() ;}
   //@}
} ;

//------------------------- HELPER FUNCTIONS ----------------------------

/// A convenience function to return a turn vote centered around a given
/// angle, i.e., the returned vote is +1 for the given direction and
/// falls linearly away from +1 as we fan outwards from the input angle.
///
/// To illustrate how this function works, let us say that the supported
/// steering directions go from -6 degrees (on the right) to +6 degrees
/// (on the left) in steps of 3 degrees. That is, turn_max is 6 and
/// turn_step is 3 and the supported steering directions are 6, 3, 0, -3,
/// -6.
///
/// If we would like to make a medium left turn, i.e., turn direction is
/// 3, then the votes returned by this function will be +1 for 3 and less
/// than that for the other directions. The amount by which the other
/// directions' votes will be less depends on the turn_max and turn_step
/// parameters. In this example, the vote step is 3/6 (step/max) or 0.5.
/// Thus, the steering direction 3 will get a vote of +1; 6 and 0 will
/// get 1 - 0.5 = 0.5; and -3 will get 1 - 2*0.5 = 0; and -6 will be
/// assigned 1 - 3*.5 = -0.5. That is, the votes will look like so:
///
///                         6   3   0   -3   -6
///                        0.5  1  0.5   0  -0.5
TurnArbiter::Vote turn_vote_centered_at(float direction) ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

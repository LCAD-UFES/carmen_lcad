/**
   \file  Robots/LoBot/control/LoGoal.H
   \brief A behaviour for driving the robot toward a list of goals.

   This file implements a class designed to use lobot's SLAM/localization
   module to drive towards each goal location specified in a list of
   goals. The behaviour will drive the robot towards the first goal in
   the list, then the second, then the third, and so on. It can be
   configured to continually loop through the goal list and/or to
   backtrack over the goals once it reaches the end of the list.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoGoal.H $
// $Id: LoGoal.H 13703 2010-07-28 00:15:12Z mviswana $
//

#ifndef LOBOT_GOAL_BEHAVIOUR_DOT_H
#define LOBOT_GOAL_BEHAVIOUR_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoBehavior.H"
#include "Robots/LoBot/misc/LoVector.H"
#include "Robots/LoBot/misc/factory.hh"

// Standard C++ headers
#include <ostream>
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

// Forward declaration
class Pose ;

/**
   \class lobot::Goal
   \brief A behaviour for driving the robot toward a list of goals.

   This class implements a behaviour for driving the robot toward each
   goal in a list of goals. When the robot reaches a goal, the behaviour
   can be configured to continue on to the next goal or pause and wait
   for the user to resume the robot. When the robot is at the last goal
   in the list, the behaviour can either maintain the final goal or loop
   to the first goal and start over. Additionally, in looping mode, the
   behaviour can either start from the first goal or backtrack over the
   goal list in reverse.

   By default, the behaviour continues on to the next goal once it
   reaches one goal on the list and it maintains the final goal when it
   reaches the end of the goal list.

   A goal is a rectangle specified in map coordinates. The behaviour will
   direct the robot towards the center of each goal rectangle. However,
   the goal is considered satisfied when the robot is located anywhere
   within the target rectangle.
*/
class Goal : public Behavior {
   // Prevent copy and assignment
   Goal(const Goal&) ;
   Goal& operator=(const Goal&) ;

   // Handy type to have around in a derived class
   typedef Behavior base ;

   // Boilerplate code to make the generic factory design pattern work
   friend  class subfactory<Goal, base> ;
   typedef register_factory<Goal, base> my_factory ;
   static  my_factory register_me ;

   /// A helper class for encapsulating a goal.
   class Target {
      /// A goal is specified as a rectangle in map coordinates.
      float m_left, m_right, m_bottom, m_top ;

      /// Since the goal seeking behaviour will often want to know a goal
      /// rectangle's center, it makes sense to compute these coordinates
      /// during initialization and keep them around for later use.
      float m_center_x, m_center_y ;

   public:
      /// Goal init.
      Target(float left, float right, float bottom, float top) ;

      /// Return the goal's bounds.
      //@{
      float left()   const {return m_left   ;}
      float right()  const {return m_right  ;}
      float bottom() const {return m_bottom ;}
      float top()    const {return m_top    ;}
      //@}

      /// Convenient aliases.
      //@{
      float L() const {return left()   ;}
      float R() const {return right()  ;}
      float B() const {return bottom() ;}
      float T() const {return top()    ;}
      //@}

      /// Return the goal rectangle's center coordinates.
      //@{
      float x() const {return m_center_x ;}
      float y() const {return m_center_y ;}
      //@}

      /// Check if a location is within the goal's bounds.
      bool at(float x, float y) const {
         return x >= m_left && x <= m_right && y >= m_bottom && y <= m_top ;
      }

      /// Helper to dump goals to an output stream.
      friend std::ostream& operator<<(std::ostream& os, const Target& t) {
         os << '(' << t.x() << ", " << t.y() << ") "
            << '[' << t.L() << ' '  << t.R()
            << ' ' << t.B() << ' '  << t.T() << ']' ;
         return os ;
      }
   } ;

   /// The goal behaviour maintains a list of goals and drives the robot
   /// to each one in turn.
   std::vector<Target> m_goals ;

   /// This data member is used to keep track of the current goal.
   int m_goal ;

   /// To direct the robot towards the current goal, this behaviour
   /// implements the VFF method described by Borenstein and Koren in
   /// "Real-time Obstacle Avoidance for Fast Mobile Robots," IEEE
   /// Transactions on Systems, Man, and Cybernetics 19(5):1179--1187,
   /// Sep/Oct 1989.
   ///
   /// To compute the total repulsive force exerted by obstacles within
   /// the active window of the certainty grid, we use a vector mask.
   /// Each cell of this mask contains a force vector scaled by Fcr/d2,
   /// where Fcr is the constant repulsive force and d2 is the square of
   /// the distance between that cell and the mask's center. See equation
   /// (1) in the above-mentioned paper.
   ///
   /// When we want to perform the repulsive force computation, we
   /// extract a subportion of the map centered at the robot's current
   /// position and sized the same as the mask. We then convolve the mask
   /// and certainty values in the map's subportion to obtain the final
   /// repulsive force vector.
   std::vector<Vector> m_vff_mask ;

   /// We need the most recent VFF vectors for visualization.
   Vector m_attractive, m_repulsive, m_steering ;

   /// The turn direction corresponding to the steering vector is also a
   /// useful thing to visualize.
   int m_turn_dir ;

   /// A private constructor because behaviours are instantiated with an
   /// object factory and not directly by clients.
   Goal() ;

   /// Stuff to do before regular action processing begins.
   void pre_run() ;

   /// To properly log when this behaviour starts seeking the next goal
   /// on its list and when it reaches a goal, we need to override the
   /// default run() method provided by the Behavior class.
   void run() ;

   /// This method implements this behaviour's goal-seeking action.
   void action() ;

   /// A helper function for sending goal stats to the metrics log.
   void log(const char* msg, int goal_index, const Target&, const Pose&) const;

   /// Visualizing the goal behaviour's votes.
   void render_me() ;

   /// Visualizing the goals on lobot's map.
   //@{
   static void render_goals(unsigned long client_data) ;
   void render_goals() ;
   static void render_goal(const Target&) ;
   //@}

   /// Clean-up.
   ~Goal() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

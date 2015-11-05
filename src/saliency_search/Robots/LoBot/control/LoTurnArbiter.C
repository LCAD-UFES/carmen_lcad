/**
   \file  Robots/LoBot/control/LoTurnArbiter.C
   \brief This file defines the non-inline member functions of the
   lobot::TurnArbiter class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/control/LoTurnArbiter.C $
// $Id: LoTurnArbiter.C 13521 2010-06-06 14:23:03Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/control/LoTurnArbiter.H"

#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/thread/LoUpdateLock.H"

#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/util/LoGL.H"
#include "Robots/LoBot/util/LoDebug.H"
#include "Robots/LoBot/util/LoSTL.H"

// OpenGL headers
#ifdef INVT_HAVE_LIBGL
#include <GL/gl.h>
#endif

// Standard C++ headers
#include <algorithm>
#include <functional>
#include <iterator>
#include <utility>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//--------------------------- LOCAL HELPERS -----------------------------

// Retrieve settings from turn_arbiter section of config file
template<typename T>
static inline T conf(const std::string& key, const T& default_value)
{
   return get_conf<T>("turn_arbiter", key, default_value) ;
}

//-------------------------- INITIALIZATION -----------------------------

TurnArbiter::TurnArbiter()
   : Arbiter(clamp(conf("update_delay", 500), 1, 1000),
             "turn_arbiter", conf<std::string>("geometry", "480 420 140 140"))
{
   start("turn_arbiter") ;
}

float TurnArbiter::get_configured_priority(const std::string& behaviour) const
{
   return abs(get_conf(behaviour, "turn_priority", 0.0f)) ;
}

//-------------------------- MOTOR COMMANDS -----------------------------

// Helper to scale all the vote values in a Vote by the given weight
static TurnArbiter::Vote operator*(TurnArbiter::Vote V, float weight)
{
   std::transform(V.begin(), V.end(), V.begin(),
                  std::bind2nd(std::multiplies<float>(), weight)) ;
   return V ;
}

// To issue an appropriate motor command, the turn arbiter must first
// combine the votes from all the behaviours, weighting each vote by its
// corresponding behaviour's priority. It then smooths the resulting vote
// using a Gaussian and picks the motor command with the maximum vote
// value.
void TurnArbiter::motor_cmd(const Arbiter::Votes& votes, Robot* robot)
{
   Vote result ;

   // First, compute weighted sum of all votes
   Arbiter::Votes::const_iterator it = votes.begin() ;
   for (; it != votes.end(); ++it)
      result +=
         (*dynamic_cast<Vote*>((*it)->vote)) * priority((*it)->behavior_name) ;
   //result.dump("TurnArbiter::motor_cmd before smooting") ;

   // Then, Gaussian smooth the weighted sum
   typedef Vote::VoteMap::value_type DVP ; // direction-vote pair
   std::vector<DVP> V(result.m_votes.begin(), result.m_votes.end()) ;

   const int   N = V.size() ;
   const int   W = clamp(Params::smoothing_width(), 1, N) ;
   const float S = 2 * Params::sigma() * Params::sigma() ;
   const float s = 1/(2.506628f /* sqrt(2*pi) */ * Params::sigma()) ;

   for  (int i = 0; i < N; ++i)
   {
      float v = 0 ;
      for (int j = i - W; j <= i + W; ++j) {
         if (j < 0 || j >= N)
            continue ;
         float d = V[j].first - V[i].first ;
         v += V[j].second * exp(-(d*d)/S) * s ;
      }
      result[V[i].first] = v ;
   }
   //result.dump("TurnArbiter::motor_cmd after smoothing") ;

   result.normalize() ;
   //result.dump("TurnArbiter::motor_cmd after normalization") ;

   viz_lock() ;
      m_vote = result ; // record most recent vote for visualization
      //m_vote.dump("TurnArbiter::motor_cmd") ;
   viz_unlock() ;

   // Finally, pick the command with the maximum votes
   Vote::iterator max = std::max_element(result.begin(), result.end()) ;
   //LERROR("max vote %g for direction: %d", max.value(), max.direction()) ;

   UpdateLock::begin_write() ;
      robot->turn(max.direction()) ;
   UpdateLock::end_write() ;
}

//------------------------------- VOTES ---------------------------------

// Vote initialization: neutral for all directions
TurnArbiter::Vote::Vote()
{
   const int M = turn_max() ;
   const int S = turn_step() ;
   m_votes[0]  = 0 ;
   for (int direction = +S; direction <= +M; direction += S)
      m_votes[direction] = 0 ;
   for (int direction = -S; direction >= -M; direction -= S)
      m_votes[direction] = 0 ;
}

// Return the supported turn directions a behaviour can vote on
std::vector<int> TurnArbiter::Vote::get_directions() const
{
   std::vector<int> V ;
   V.reserve(m_votes.size()) ;
   std::transform(m_votes.begin(), m_votes.end(), std::back_inserter(V),
                  get_first<VoteMap::value_type>) ;
   return V ;
}

class abs_diff {
   int direction ;
public:
   abs_diff(int d) : direction(d) {}
   bool operator()(const std::pair<int, float>& a,
                   const std::pair<int, float>& b) const {
      return abs(a.first - direction) < abs(b.first - direction) ;
   }
} ;

// Operator to access vote value corresponding to specified turn
// direction.
TurnArbiter::Vote::VoteMap::mapped_type&
TurnArbiter::Vote::
operator[](int direction)
{
   VoteMap::iterator it = m_votes.find(direction) ;
   if (it == m_votes.end())
      //throw arbiter_error(UNSUPPORTED_TURN_DIRECTION) ;
      it = std::min_element(m_votes.begin(), m_votes.end(),
                            abs_diff(direction)) ;
   return it->second ;
}

// Vote clean-up
TurnArbiter::Vote::~Vote(){}

// Vote iterator start constructor
TurnArbiter::Vote::iterator::iterator(const TurnArbiter::Vote& V)
   : m_vote(const_cast<TurnArbiter::Vote&>(V)),
     m_iterator(m_vote.m_votes.begin())
{}

// Vote iterator end constructor
TurnArbiter::Vote::iterator::iterator(const TurnArbiter::Vote& V, bool)
   : m_vote(const_cast<TurnArbiter::Vote&>(V)),
     m_iterator(m_vote.m_votes.end())
{}

// Vote iterator copy constructor
TurnArbiter::Vote::iterator::iterator(const TurnArbiter::Vote::iterator& it)
   : m_vote(it.m_vote),
     m_iterator(it.m_iterator)
{}

// Vote iterator assignment operator
TurnArbiter::Vote::iterator&
TurnArbiter::Vote::iterator::operator=(const TurnArbiter::Vote::iterator& it)
{
   if (& it != this) {
      m_vote     = it.m_vote ;
      m_iterator = it.m_iterator ;
   }
   return *this ;
}

// Vote iterator destructor
TurnArbiter::Vote::iterator::~iterator(){}

// Adding votes
TurnArbiter::Vote& TurnArbiter::Vote::operator+=(const TurnArbiter::Vote& V)
{
   std::transform(V.begin(), V.end(), begin(), begin(), std::plus<float>()) ;
   return *this ;
}

/*
  The following function object is meant to be used with the STL
  transform algorithm and scales votes to lie in the range [-1, +1]. It
  needs to know the min and max vote values that occur in a vote and then
  simply performs a linear interpolation to compute the scaled vote value
  like so:

                            v - m     s - (-1)
                            -----  =  --------
                            M - m     1 - (-1)

                            s + 1     v - m
                     ===>   -----  =  -----
                              2       M - m

                     ===>       s  =  2(v - m)/(M - m) - 1

   where m = min vote value
         M = max vote value
         v = unscaled vote value
         s = scaled vote value
*/
class scale_vote {
   float min, max ;
public:
   scale_vote(float m, float M) : min(m), max(M) {}
   float operator()(const float& v) const {
      return 2 * (v - min)/(max - min) - 1 ;
   }
} ;

// Normalizing votes so that all directions' votes are in the [-1, +1]
// range.
void TurnArbiter::Vote::normalize()
{
   float min = *std::min_element(begin(), end()) ;
   float max = *std::max_element(begin(), end()) ;
   normalize(min, max) ;
}

// Normalizing votes so that all directions' votes are in the [-1, +1]
// range.
void TurnArbiter::Vote::normalize(float min, float max)
{
   std::transform(begin(), end(), begin(), scale_vote(min, max)) ;
}

// Debug support: dump a vote's direction-value pairs
void TurnArbiter::Vote::dump(const std::string& caller) const
{
   lobot::dump(m_votes, caller, "m_votes") ;
}

/*
   The following helper function returns a vote for driving in the
   specified direction with votes for the other directions falling
   linearly away from +1.

   To illustrate how this function works, let us say that the supported
   steering directions go from -6 degrees (on the right) to +6 degrees
   (on the left) in steps of 3 degrees. That is, turn_max is 6 and
   turn_step is 3 and, therefore, the supported steering directions are
   6, 3, 0, -3, -6.

   If we would like to make a medium left turn, i.e., turn direction is
   3, then the votes returned by this function will be +1 for 3 and less
   than that for the other directions. The amount by which the other
   directions' votes will be less depends on the turn_max and turn_step
   parameters. In this example, the vote step is 3/6 (step/max) or 0.5.
   Thus, the steering direction 3 will get a vote of +1; 6 and 0 will get
   1 - 0.5 = 0.5; and -3 will get 1 - 2*0.5 = 0; and -6 will be assigned
   1 - 3*.5 = -0.5. That is, the votes will look like so:

                           6   3   0   -3   -6
                          0.5  1  0.5   0  -0.5

   As we can see, the votes for the directions other than the one at
   which they are to be "centered", viz., C, fall away according to the
   angular distance between C and that particular direction. The vote for
   the direction C is +1 and for the others is something less than 1.
   That is, the vote for some direction d is 1 - something. This
   something is a function of the angular distance between d and C, i.e.,
   d - C.

   Now, the vote step for consecutive directions is simply s/T, where s
   is the turn step and T the turn max. In the above example, the vote
   step is 3/6 = 0.5. What this means is that if the vote for some
   direction is pegged at v, then the vote for its two surrounding
   directions will be v +/- 0.5.

   Therefore, if we are d - C degrees away from some turn angle C, we can
   divide d - C by the turn step s to get the number of angular steps
   between d and C. We then multiply this number by the vote step to get
   the total number of vote steps between the directions d and C.
   Finally, we subtract that from 1 (the vote for C) to get the vote for
   the direction d.

   This gives us the following expression for the vote for a steering
   angle d:

                              d - C    s
              vote[d] =  1 -  ----- x ---  = 1 - (d - C)/T
                                s      T

   But notice that if we were to use the above expression as-is, the
   votes for directions whose magnitudes are less than C will be greater
   than 1. For example, the vote for direction 0 will be 1 - (0 - 3)/6 =
   1 - (-3)/6 = 1 + 0.5 = 1.5!

   The problem is that we are really interested in the magnitude of the
   difference between d and C and not its sign. In other words, if we
   think of (d - C) as an error, then we only want the amount of error
   and not the direction in which the error goes. This gives us:

                       vote[d] = 1 - abs(d - C)/T
*/
TurnArbiter::Vote turn_vote_centered_at(float C)
{
   const int T = TurnArbiter::turn_max() ;

   TurnArbiter::Vote V ;
   TurnArbiter::Vote::iterator it = V.begin() ;
   for (; it != V.end(); ++it)
      *it = clamp(1 - abs(it.direction() - C)/T, -1.0f, +1.0f) ;
   return V ;
}

//--------------------------- VISUALIZATION -----------------------------

#ifdef INVT_HAVE_LIBGL

void TurnArbiter::render_me()
{
   // Copy most recent vote so that turn arbiter thread is not
   // unnecessarily held up while the rendering takes place.
   viz_lock() ;
      Vote V = m_vote ;
   viz_unlock() ;

   // Render the votes visualization
   unit_view_volume() ;
   glBegin(GL_LINES) ;
      for (Vote::iterator it = V.begin(); it; ++it)
      {
         float vote = it.value() ;
         float direction = it.direction() ;
         if (vote < 0) // voted against this direction
         {
            glColor3f(1, 0, 0) ;
            glVertex2i(0, 0) ;
            glVertex2f(-vote * cos(direction), -vote * sin(direction)) ;
         }
         else if (vote > 0) // voted for this direction
         {
            glColor3f(0, 1, 0) ;
            glVertex2i(0, 0) ;
            glVertex2f(vote * cos(direction), vote * sin(direction)) ;
         }
      }

      Vote::iterator max = std::max_element(V.begin(), V.end()) ;
      if (max.value() > 0) // this is the direction that won
      {
         glColor3f(1, 1, 1) ;
         glLineWidth(2) ;
         glVertex2i(0, 0) ;
         glVertex2f(cos(max.direction()), sin(max.direction())) ;
      }
   glEnd() ;

   // Label the visualization so that it is easy to tell what is being
   // visualized.
   restore_view_volume() ;
   text_view_volume() ;
   glColor3f(0, 1, 1) ;
   draw_label(3, 12, "Turn Arbiter") ;

   restore_view_volume() ;
}

#endif

//----------------------- TURN ARBITER CLEAN-UP -------------------------

TurnArbiter::~TurnArbiter(){}

//-------------------------- KNOB TWIDDLING -----------------------------

// Parameters initialization
TurnArbiter::Params::Params()
   : m_turn_max (clamp(conf("turn_max", 20), 5, 60)),
     m_turn_step(clamp(conf("turn_step", 1), 1, m_turn_max/2)),
     m_smoothing_width(conf("smoothing_window_width", 7)),
     m_sigma(clamp(conf("smoothing_sigma", 1.0f), 0.5f, m_turn_max/2.0f))
{}

// Parameters clean-up
TurnArbiter::Params::~Params(){}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

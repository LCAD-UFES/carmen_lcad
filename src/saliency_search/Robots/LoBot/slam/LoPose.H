/**
   \file  Robots/LoBot/misc/LoPose.H
   \brief A simple structure for holding a robot's pose.

   This file defines a class that holds a robot's pose, viz., its
   location and bearing.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/slam/LoPose.H $
// $Id: LoPose.H 13619 2010-06-25 01:59:32Z mviswana $
//

#ifndef LOBOT_POSE_DOT_H
#define LOBOT_POSE_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/util/triple.hh"

// Standard C++ headers
#include <ostream>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Pose
   \brief A simple structure for representing a robot's pose.

   A robot's pose consists of its location specified using an (x, y)
   coordinate pair and its heading/bearing (i.e., theta). This class
   packages these three quantities in one neat little bundle.
*/
class Pose {
   /// The robot's pose consists of an (x, y) position within the map
   /// and an angle to indicate the direction in which the robot is
   /// pointed.
   triple<float, float, float> m_pose ;

public:
   /// Pose initialization.
   //@{
   Pose(float x = 0, float y = 0, float theta = 0) ;
   Pose(const triple<float, float, float>& pose) ;
   //@}

   /// Retrieving pose values.
   //@{
   float x() const {return m_pose.first  ;}
   float y() const {return m_pose.second ;}
   float t() const {return m_pose.third  ;}
   float theta()   const {return t() ;}
   float heading() const {return t() ;}
   float bearing() const {return t() ;}
   //@}

   /// Updating pose values.
   //@{
   void x(float xx) {m_pose.first  = xx ;}
   void y(float yy) {m_pose.second = yy ;}
   void t(float th) ;
   void theta(float th)   {t(th) ;}
   void heading(float th) {t(th) ;}
   void bearing(float th) {t(th) ;}

   void dx(float d) {m_pose.first  += d ;}
   void dy(float d) {m_pose.second += d ;}
   void dt(float t) ;
   void dtheta(float t) {dt(t) ;}
   //@}

   /// This operator adds this pose to another and retains the result in
   /// this Pose object itself.
   Pose& operator+=(const Pose&) ;

   /// This operator adds this pose to another and returns the result via
   /// a new Pose object.
   Pose operator+(const Pose&) const ;

   /// This operator multiplies this pose's values by the supplied
   /// weighting factor and returns the scaled result in a new Pose
   /// object.
   Pose operator*(float weight) const ;
} ;

//----------------------- NON-MEMBER FUNCTIONS --------------------------

/// Dump a pose to an output stream.
std::ostream& operator<<(std::ostream&, const Pose&) ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

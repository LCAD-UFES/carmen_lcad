/**
   \file  Robots/LoBot/misc/LoPose.C
   \brief This file defines the non-inline member functions of the
   lobot::Pose class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/slam/LoPose.C $
// $Id: LoPose.C 13620 2010-06-25 05:12:03Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/slam/LoPose.H"
#include "Robots/LoBot/util/LoMath.H"

// Standard C++ headers
#include <iomanip>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

Pose::Pose(float x, float y, float theta)
   : m_pose(x, y, clamp_angle(theta))
{}

Pose::Pose(const triple<float, float, float>& p)
   : m_pose(p.first, p.second, clamp_angle(p.third))
{}

//--------------------------- POSE UPDATES ------------------------------

void Pose::t(float th)
{
   m_pose.third = clamp_angle(th) ;
}

void Pose::dt(float th)
{
   m_pose.third = clamp_angle(m_pose.third + th) ;
}

//-------------------------- POSE OPERATIONS ----------------------------

// Add two poses
Pose& Pose::operator+=(const Pose& p)
{
   m_pose.first  += p.x() ;
   m_pose.second += p.y() ;
   m_pose.third  += p.t() ;
   return *this ;
}

// Add two poses
Pose Pose::operator+(const Pose& p) const
{
   return Pose(x() + p.x(), y() + p.y(), t() + p.t()) ;
}

// Return a new Pose weighted by the supplied factor
Pose Pose::operator*(float weight) const
{
   return Pose(x() * weight, y() * weight, t() * weight) ;
}

//----------------------------- POSE I/O --------------------------------

std::ostream& operator<<(std::ostream& os, const Pose& p)
{
   using namespace std ;
   os << '('
      << setw(8) << left << round(p.x())
      << setw(8) << left << round(p.y()) ;

   int th = round(p.theta()) ;
   if (th > 180)
      th -= 360 ;
   if (th == 0)
      os << th ;
   else
      os << showpos << th << noshowpos ;
   os << ')' ;

   return os ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

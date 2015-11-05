/**
   \file  Robots/LoBot/misc/LoPointList.C
   \brief This file defines the non-inline member functions of the
   lobot::PointList class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/metlog/LoPointList.C $
// $Id: LoPointList.C 13936 2010-09-15 08:01:24Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/metlog/LoPointList.H"
#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/misc/LoExcept.H"

// Boost headers
#include <boost/bind.hpp>

// Standard C++ headers
#include <algorithm>
#include <iterator>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

PointList::PointList(int n)
{
   if (n > 0)
      m_list.reserve(n) ;
}

// Add new points to the point list without any duplications, i.e., make
// the point list behave like a point set.
void PointList::add(int x, int y)
{
   mPoint p(x, y) ;
   if (std::find(m_list.begin(), m_list.end(), p) == m_list.end())
      m_list.push_back(p) ;
}

// Append given point list to this one
PointList& PointList::operator+=(const PointList& L)
{
   std::copy(L.m_list.begin(), L.m_list.end(), std::back_inserter(m_list)) ;
   return *this ;
}

//----------------------- POINT CORRESPONDENCES -------------------------

// Return true if the Euclidean distance between point a and reference
// point p is less than the Euclidean distance between point b and
// reference point p, i.e., return true if a is closer to p than b.
static bool dist_cmp(const mPoint& a, const mPoint& b, const mPoint& p)
{
   return sqrt(sqr(a.first - p.first) + sqr(a.second - p.second))
        < sqrt(sqr(b.first - p.first) + sqr(b.second - p.second)) ;
}

// In the point list L, find the point closest to reference point p
static mPoint nearest_point(const mPoint& p, const PointList& L)
{
   return *std::min_element(L.begin(), L.end(),
                            boost::bind(dist_cmp, _1, _2, p)) ;
}

// For each point in this point list, find the nearest point in the given
// point list L and return the resulting point list.
PointList PointList::find_correspondences(const PointList& L) const
{
   if (m_list.empty() || L.m_list.empty())
      throw misc_error(LOGIC_ERROR) ;

   PointList result(m_list.size()) ;
   std::transform(m_list.begin(), m_list.end(),
                  std::back_inserter(result.m_list),
                  boost::bind(nearest_point, _1, L)) ;
   return result ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

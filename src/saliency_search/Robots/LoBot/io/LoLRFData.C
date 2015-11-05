/**
   \file  Robots/LoBot/misc/LoLRFData.C
   \brief This file defines the non-inline member functions of the
   lobot::LRFData class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoLRFData.C $
// $Id: LoLRFData.C 13434 2010-05-19 04:36:08Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/io/LoLRFData.H"
#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/util/LoMath.H"

// INVT headers
#include "Image/Image.H"

// Standard C++ headers
#include <algorithm>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

LRFData::LRFData(const LaserRangeFinder* lrf)
   : m_angle_range(lrf->get_angular_range()),
     m_distance_range(lrf->get_distance_range()),
     m_distances(new int[m_angle_range.size()])
{
   Image<int> D = lrf->get_distances() ;
   std::copy(D.begin(), D.end(), m_distances) ;
}

LRFData::LRFData(const LRFData& L)
   : m_angle_range(L.m_angle_range),
     m_distance_range(L.m_distance_range),
     m_distances(new int[m_angle_range.size()])
{
   std::copy(L.m_distances, L.m_distances + m_angle_range.size(), m_distances);
}

LRFData& LRFData::operator=(const LRFData& L)
{
   if (&L != this) {
      m_angle_range    = L.m_angle_range ;
      m_distance_range = L.m_distance_range ;
      std::copy(L.m_distances, L.m_distances + m_angle_range.size(),
                m_distances) ;
   }
   return *this ;
}

LRFData::Reading::Reading(int angle, int distance)
   : m_angle(angle), m_distance(distance)
{}

//---------------------- DISTANCE DATA RETRIEVAL ------------------------

// Return measurement corresponding to given angle
int LRFData::distance(int angle) const
{
   if (angle < m_angle_range.min() || angle > m_angle_range.max())
      return -1 ;
   return m_distances[angle - m_angle_range.min()] ;
}

// Return all distance readings in an STL vector
std::vector<int> LRFData::distances() const
{
   return std::vector<int>(m_distances, m_distances + m_angle_range.size()) ;
}

// Return the average distance in some angular range
float LRFData::average_distance(int min_angle, int max_angle) const
{
   // Calculate indices corresponding to min and max angle
   const int a = std::max(min_angle - m_angle_range.min(), 0) ;
   const int b = std::min(max_angle - m_angle_range.min(),
                          m_angle_range.size() - 1) ;

   // Calculate and return average distance between above indices
   if (a == b) // special case for single angle
      return m_distances[a] ;

   int n = 0 ; float sum = 0 ;
   for (int i = a; i <= b; ++i)
      if (m_distances[i] > 0) {
         sum += m_distances[i] ;
         ++n ;
      }
   return (n > 0) ? sum/n : -1 ;
}

// Return the minimum reading in some angular range
LRFData::Reading LRFData::min_distance(int min_angle, int max_angle) const
{
   min_angle = clamp(min_angle, m_angle_range.min(), m_angle_range.max()) ;
   max_angle = clamp(max_angle, m_angle_range.min(), m_angle_range.max()) ;
   if (min_angle > max_angle)
      std::swap(min_angle, max_angle) ;

   const int m = min_angle - m_angle_range.min() ;
   const int M = max_angle - m_angle_range.min() ;

   int* min = m_distances + m ;
   while (*min == -1 && min < m_distances + M + 1) // skip initial bad readings
      ++min ;
   if (min >= m_distances + M) // all bad readings!?!
      min = m_distances + m ;

   for (int* p = min + 1; p < m_distances + M + 1; ++p)
   {
      if (*p == -1) // bad reading
         continue ;
      if (*p < *min)
         min = p ;
   }

   return Reading((min - m_distances) + m_angle_range.min(), *min) ;
}

//----------------------------- CLEAN-UP --------------------------------

LRFData::~LRFData()
{
   delete[] m_distances ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

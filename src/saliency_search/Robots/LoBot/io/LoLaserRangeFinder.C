/**
   \file  Robots/LoBot/io/LoLaserRangeFinder.C
   \brief This file defines the non-inline member functions of the
   lobot::LaserRangeFinder class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoLaserRangeFinder.C $
// $Id: LoLaserRangeFinder.C 12863 2010-02-19 20:43:51Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/io/LoLaserRangeFinder.H"
#include "Robots/LoBot/misc/LoExcept.H"

// Standard C++ headers
#include <algorithm>

#include <ctime>
#include <cstdlib>

//---------------------- ALTERNATIVE DEFINITION -------------------------

// Un/comment following line when laser range finder is un/available
//#define LOBOT_LRF_DEVMODE 1

// A convenient dummy during development when the laser range finder may
// not always be at hand for testing purposes.
#ifdef LOBOT_LRF_DEVMODE

namespace lobot {

// Constructor
LaserRangeFinder::LaserRangeFinder(const std::string&, int)
   : m_bufsiz(0), m_retsiz(0), m_buffer(0),
     m_angle_range(-119, 135),
     m_distance_range(60, 5600),
     m_distances(new int[m_angle_range.size()])
{}

// A quick function object to generate random integers in the given range
class rand_int {
   int min, max ;
public:
   rand_int(int i = 0, int x = 100) : min(i - 1), max(x + 1) {
      std::srand(std::time(0)) ;
   }
   int operator()() const {
      const float n = static_cast<float>(std::rand())/RAND_MAX ;
      return static_cast<int>(min + n * (max - min)) ;
   }
} ;

// Dummy API
void LaserRangeFinder::update()
{
   std::generate_n(m_distances, m_angle_range.size(),
                   rand_int(m_distance_range.min(), m_distance_range.max())) ;
}

int LaserRangeFinder::get_distance(int angle) const
{
   if (angle < m_angle_range.min() || angle > m_angle_range.max())
      return -1 ;
   return m_distances[angle - m_angle_range.min()] ;
}

Image<int> LaserRangeFinder::get_distances() const
{
   Image<int> D(m_angle_range.size(), 1, ZEROS) ;
   std::copy(m_distances, m_distances + m_angle_range.size(), D.beginw()) ;
   return D ;
}

float LaserRangeFinder::average_distance(int min_angle, int max_angle) const
{
   const int a = std::max(min_angle - m_angle_range.min(), 0) ;
   const int b = std::min(max_angle - m_angle_range.min(),
                          m_angle_range.size() - 1) ;
   if (a == b) // special case for single angle
      return m_distances[a] ;

   float sum = 0 ; int n = 0 ;
   for (int i = a; i <= b; ++i)
      if (m_distances[i] > 0) {
         sum += m_distances[i] ;
         ++n ;
      }
   return (n > 0) ? sum/n : -1 ;
}

int LaserRangeFinder::max_reading(int min_angle, int max_angle) const
{
   const int a = std::max(min_angle - m_angle_range.min(), 0) ;
   const int b = std::min(max_angle - m_angle_range.min(),
                          m_angle_range.size() - 1) ;
   if (a == b) // special case for single angle
      return m_distances[a] ;
   return *(std::max_element(m_distances + a, m_distances + b + 1)) ;
}

// Destructor
LaserRangeFinder::~LaserRangeFinder()
{
   delete[] m_distances ;
}

}

#else // not in development mode ==> LRF available ==> use the real thing

//---------------------- ALTERNATIVE DEFINITION -------------------------

// In case liburg is missing
#ifndef INVT_HAVE_LIBURG

namespace lobot {

// Constructor
LaserRangeFinder::LaserRangeFinder(const std::string&, int)
   : m_handle(0), m_bufsiz(0), m_retsiz(0), m_buffer(0),
     m_angle_range(0, 0), m_distance_range(-1, -1), m_distances(0)
{
   throw missing_libs(MISSING_LIBURG) ;
}

// Empty API
void       LaserRangeFinder::update(){}
int        LaserRangeFinder::get_distance(int) const {return -1 ;}
Image<int> LaserRangeFinder::get_distances()   const {return Image<int>() ;}
float      LaserRangeFinder::average_distance(int, int) const {return -1  ;}
int        LaserRangeFinder::max_reading(int, int)      const {return -1  ;}

// Destructor
LaserRangeFinder::~LaserRangeFinder(){}

} // end of namespace encapsulating above empty definition

#else // liburg available ==> the real McCoy

//-------------------------- FULL DEFINITION ----------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

LaserRangeFinder::LaserRangeFinder(const std::string& device, int baud_rate)
   : m_bufsiz(0), m_retsiz(0), m_buffer(0),
     m_angle_range(0, 0), m_distance_range(-1, -1), m_distances(0)
{
   if (urg_connect(& m_handle, device.c_str(), baud_rate) < 0)
      throw lrf_error(LRF_CONNECTION_FAILURE) ;

   m_bufsiz = urg_getDataMax(& m_handle) ;
   m_buffer = new long[m_bufsiz] ;
   std::fill_n(m_buffer, m_bufsiz, 0) ;

   m_angle_range.reset(urg_index2deg(& m_handle, 0),
                       urg_index2deg(& m_handle, m_bufsiz - 1)) ;
   m_distances = new int[m_angle_range.size()] ;
   std::fill_n(m_distances, m_angle_range.size(), 0) ;

   m_distance_range.reset(static_cast<int>(urg_getDistanceMin(& m_handle)),
                          static_cast<int>(urg_getDistanceMax(& m_handle))) ;
}

//---------------------- DISTANCE DATA RETRIEVAL ------------------------

// Buffer latest measurements from device
void LaserRangeFinder::update()
{
   if (urg_requestData(& m_handle, URG_GD, URG_FIRST, URG_LAST) < 0)
      throw lrf_error(LRF_DATA_RETRIEVAL_FAILURE) ;

   m_retsiz = urg_receiveData(& m_handle, m_buffer, m_bufsiz) ;
   if (m_retsiz < 0)
      throw lrf_error(LRF_DATA_RETRIEVAL_FAILURE) ;

   const int m = m_angle_range.min() ;
   const int M = m_angle_range.max() ;
   for  (int angle = m, i = 0; angle <= M; ++angle, ++i)
   {
      long d = m_buffer[urg_deg2index(const_cast<urg_t*>(& m_handle), angle)] ;
      if (d < m_distance_range.min() || d > m_distance_range.max())
         d = -1 ;
      m_distances[i] = static_cast<int>(d) ;
   }
}

// Return measurement corresponding to given angle
int LaserRangeFinder::get_distance(int angle) const
{
   if (angle < m_angle_range.min() || angle > m_angle_range.max())
      return -1 ;
   return m_distances[angle - m_angle_range.min()] ;
}

// Return all measurements packed together in an Image<int>
Image<int> LaserRangeFinder::get_distances() const
{
   Image<int> D(m_angle_range.size(), 1, NO_INIT) ;
   std::copy(m_distances, m_distances + m_angle_range.size(), D.beginw()) ;
   return D ;
}

// Return average distance over the given range of angles
float LaserRangeFinder::average_distance(int min_angle, int max_angle) const
{
   const int a = std::max(min_angle - m_angle_range.min(), 0) ;
   const int b = std::min(max_angle - m_angle_range.min(),
                          m_angle_range.size() - 1) ;
   if (a == b) // special case for single angle
      return m_distances[a] ;

   float sum = 0 ; int n = 0 ;
   for (int i = a; i <= b; ++i)
      if (m_distances[i] > 0) {
         sum += m_distances[i] ;
         ++n ;
      }
   return (n > 0) ? sum/n : -1 ;
}

// Return maximum distance reading in given angular range
int LaserRangeFinder::max_reading(int min_angle, int max_angle) const
{
   const int a = std::max(min_angle - m_angle_range.min(), 0) ;
   const int b = std::min(max_angle - m_angle_range.min(),
                          m_angle_range.size() - 1) ;
   if (a == b) // special case for single angle
      return m_distances[a] ;
   return *(std::max_element(m_distances + a, m_distances + b + 1)) ;
}

//----------------------------- CLEAN-UP --------------------------------

LaserRangeFinder::~LaserRangeFinder()
{
   urg_disconnect(& m_handle) ;
   delete[] m_distances ;
   delete[] m_buffer ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif // #ifndef INVT_HAVE_LIBURG
#endif // #ifdef  LOBOT_LRF_DEVMODE

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

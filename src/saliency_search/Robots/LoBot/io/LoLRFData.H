/**
   \file Robots/LoBot/misc/LoLRFData.H

   \brief A helper for the lobot::LaserRangeFinder class that copies all
   the current measurements and provides a convenient API to use them.

   This file defines a class that provides a convenient API for copying
   and then using the current measurements from lobot::LaserRangeFinder.
   It was written so as to keep lobot::LaserRangeFinder unchanged and
   uncluttered and to provide a quick way to make local copies of the LRF
   measurements so that behaviours don't hold the update locks for very
   long.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoLRFData.H $
// $Id: LoLRFData.H 13434 2010-05-19 04:36:08Z mviswana $
//

#ifndef LOBOT_LRF_DATA_DOT_H
#define LOBOT_LRF_DATA_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/io/LoLaserRangeFinder.H"
#include "Robots/LoBot/util/range.hh"

// Standard C++ headers
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::LRFData
   \brief Convenient API to locally copy and use LRF measurements so as
   to minimize update lock usage by different behaviours.

   This class provides a convenient API for behaviours to make local
   copies of the current LRF measurements so that they don't have to hold
   on to the Robolocust update lock for very long.
*/
class LRFData {
   /// What is the device's range?
   range<int> m_angle_range, m_distance_range ;

   /// The current distance measurements.
   int* m_distances ;

public:
   /// Initialization
   LRFData(const LaserRangeFinder*) ;

   /// Copy
   LRFData(const LRFData&) ;

   /// Assignment
   LRFData& operator=(const LRFData&) ;

   /// What is the distance measurement (in mm) along the specified
   /// direction (in degrees)? A negative value is returned if the angle
   /// is out of range. Zero degrees corresponds to the front of the
   /// device. Directions to the left are positive angles and those to
   /// the right are negative.
   int distance(int angle) const ;

   /// Convenience function to return distance corresponding to some
   /// angle. A negative value is returned if the angle is out of range.
   /// Zero degrees corresponds to the front of the device. Directions to
   /// the left are positive angles and those to the right are negative.
   int operator[](int angle) const {return distance(angle) ;}

   /// Return all the distance readings via an STL vector.
   std::vector<int> distances() const ;

   /// What is the device's distance measurement range?
   //@{
   range<int> distance_range() const {return m_distance_range ;}
   int min_distance() const {return m_distance_range.min() ;}
   int max_distance() const {return m_distance_range.max() ;}
   //@}

   /// Retrieve the device's angular range.
   //@{
   range<int> angular_range() const {return m_angle_range ;}
   int min_angle() const {return m_angle_range.min() ;}
   int max_angle() const {return m_angle_range.max() ;}
   //@}

   /// Return the average distance in the given angular range.
   //@{
   float average_distance(int min_angle, int max_angle) const ;
   float average_distance(const range<int>& R) const {
      return average_distance(R.min(), R.max()) ;
   }
   //@}

   /// Clean-up
   ~LRFData() ;

   /// This inner class encapsulates an LRF reading, viz., a distance
   /// measurement plus the angle at which that measurement was made.
   class Reading {
      /// This class holds a distance measurement plus its corresponding
      /// angle.
      int m_angle, m_distance ;

   public:
      /// Constructor
      Reading(int angle, int distance) ;

      /// Return the distance held by the reading.
      int distance() const {return m_distance ;}

      /// Return the angle held by the reading.
      int angle() const {return m_angle ;}

      /// Operator to compare readings based on their distances.
      bool operator<(const Reading& R) const {
         return m_distance < R.m_distance ;
      }
   } ;

   /// This function returns the reading corresponding to the minimum
   /// distance in the given angular range.
   //@{
   Reading min_distance(int min_angle, int max_angle) const ;
   Reading min_distance(const range<int>& R) const {
      return min_distance(R.min(), R.max()) ;
   }
   //@}
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

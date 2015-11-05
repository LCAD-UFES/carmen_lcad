/**
   \file Robots/LoBot/io/LoLaserRangeFinder.H

   \brief A wrapper around the URG API for interfacing with the Hokuyo
   line of laser range finders.

   This file defines a class that provides a higher-level API than the
   plain C API implemented by the URG library for retrieving distance
   measurements from a Hokuyo laser range finder.

   NOTE: The URG library in question is a heavily hacked version of
   Satofumi Kamimura's original URG library. It was produced by Manu
   Viswanathan and does away with the C++ and sample code parts of the
   original. It also collapses the original's libsystem and libconnection
   into a single liburg so that client programs need only link with this
   one library.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/io/LoLaserRangeFinder.H $
// $Id: LoLaserRangeFinder.H 13037 2010-03-23 01:00:53Z mviswana $
//

#ifndef LOBOT_LASER_RANGE_FINDER_DOT_H
#define LOBOT_LASER_RANGE_FINDER_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/util/range.hh"

// INVT image support
#include "Image/Image.H"

// URG headers
#ifdef INVT_HAVE_LIBURG

#include <urg/urg.h>

#else // fake URG API just to let this class compile

typedef int urg_t ;

#endif

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::LaserRangeFinder
   \brief Encapsulation of liburg API for interfacing with Hokuyo laser
   range finders.

   This class provides a higher-level abstraction over liburg's plain C
   API for retrieving distance measurements from lobot's Hokuyo laser
   range finder. It takes care of the low-level details of correctly
   invoking the liburg API and allows clients to query for distance along
   a particular direction.
*/
class LaserRangeFinder {
   // The URG "handle"
   urg_t m_handle ;

   // The buffer that will receive distance measurements from the device
   int   m_bufsiz ; // total size of receive buffer
   int   m_retsiz ; // actual number of measurements received per update
   long* m_buffer ;

   // What is the device's range?
   range<int> m_angle_range, m_distance_range ;

   // We don't need all the values returned in the receive buffer. On
   // update, we extract just those distances we actually need and store
   // them separately. This speeds up subsequent accesses because we
   // don't have to invoke any URG functions to make sense of the buffer.
   int* m_distances ;

public:
   /// Initialization
   LaserRangeFinder(const std::string& device = "/dev/ttyACM0",
                    int baud_rate = 115200) ;

   /// Retrieve distance data from the laser range finder.
   void update() ;

   /// What is the distance measurement (in mm) along the specified
   /// direction (in degrees)? A negative value is returned if the angle
   /// is out of range. Zero degrees corresponds to the front of the
   /// device. Directions to the left are positive angles and those to
   /// the right are negative.
   int get_distance(int angle) const ;

   /// Retrieve the entire set of distances.
   Image<int> get_distances() const ;

   /// A convenience routine for returning the average distance for a
   /// range of angles.
   //@{
   float average_distance(int min, int max) const ;
   float average_distance(const range<int>& angles) const {
      return average_distance(angles.min(), angles.max()) ;
   }
   //@}

   /// A convenience routine for returning the maximum distance reading
   /// for a given range of angles.
   //@{
   int max_reading(int min, int max) const ;
   int max_reading(const range<int>& angles) const {
      return max_reading(angles.min(), angles.max()) ;
   }
   int max_reading() const {return max_reading(m_angle_range) ;}
   //@}

   /// What is the device's distance measurement range?
   //@{
   range<int> get_distance_range() const {return m_distance_range ;}
   int min_distance() const {return m_distance_range.min() ;}
   int max_distance() const {return m_distance_range.max() ;}
   //@}

   /// Retrieve the device's angular range.
   //@{
   range<int> get_angular_range() const {return m_angle_range ;}
   int min_angle() const {return m_angle_range.min() ;}
   int max_angle() const {return m_angle_range.max() ;}
   //@}

   /// Clean-up
   ~LaserRangeFinder() ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

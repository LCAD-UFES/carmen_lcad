/**
   \file  Robots/LoBot/misc/LoOdometry.H
   \brief A simple structure for holding lobot's odometry data.

   This file defines a class that provides a convenient odometry-related
   API for the SLAM related parts of Robolocust.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/slam/LoOdometry.H $
// $Id: LoOdometry.H 13478 2010-05-25 03:54:43Z mviswana $
//

#ifndef LOBOT_ODOMETRY_DOT_H
#define LOBOT_ODOMETRY_DOT_H

//------------------------------ HEADERS --------------------------------

// Standard C++ headers
#include <utility>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Odometry
   \brief A simple API for various odometry-related operations.

   This class stores the distance and angle returned by lobot's low-level
   motor system. It also provides an API for accumulating these odometric
   parameters and for maintaining and testing against certain thresholds.
*/
class Odometry {
   /// lobot's odometry consists of distance traveled and angle turned.
   /// We simply stuff both these quantities into an STL pair.
   typedef std::pair<int, int> OdometryData ;

   /// As the robot moves, the low-level layers will report odometric
   /// updates in terms of the amount of distance and turn since the
   /// previous update (i.e., deltas). Additionally, this class can also
   /// keep track of the total displacement and turn.
   OdometryData m_delta, m_total ;

   /// This class can be used to check when the low-level odometry
   /// crosses some predefined thresholds. This data member stores the
   /// afore-mentioned thresholds.
   OdometryData m_thresholds ;

public:
   /// Initialization.
   Odometry() ;

   /// This method updates the current odometric values using the
   /// supplied ones.
   void add(int distance, int angle) ;

   /// This method allows clients to set some threshold values. Later,
   /// clients can check if these thresholds have been crossed or not.
   void set_thresholds(int distance, int angle) {
      m_thresholds.first  = distance ;
      m_thresholds.second = angle ;
   }

   /// Convenience function for setting thresholds specified in an STL
   /// pair.
   void set_thresholds(const std::pair<int, int>& T) {
      set_thresholds(T.first, T.second) ;
   }

   /// This method checks if the total distance or angle have crossed the
   /// thresholds set by the client.
   bool thresholds_crossed() const ;

   /// Often, after the robot has traveled a certain distance and/or
   /// turned by some predefined amount, the higher layers of the robot's
   /// controller will want to perform some action (e.g., SLAM update,
   /// scan matching) and then reset the odometry so that the tracking
   /// can begin anew.
   ///
   /// This method resets the odometry as described above.
   void reset() {
      m_delta.first = m_delta.second = m_total.first = m_total.second = 0 ;
   }

   /// This method returns the net displacement recorded by this odometry
   /// packet.
   int displacement() const {return m_delta.first ;}

   /// This method returns the net rotation recorded by this odometry
   /// packet.
   int rotation() const {return m_delta.second ;}
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/**
   \file  Robots/LoBot/misc/LoScanMatch.H
   \brief Besl and McKay's Iterative Closest Point algorithm for laser
   range finder scan matching.

   This file defines data structures and functions that implement the
   Iterative Closest Point algorithm described in:

      Besl, P. J., McKay, N. D.
      A Method for Registration of 3-D Shapes
      IEEE Transactions on Pattern Analysis and Machine Intelligence,
      14(2):239--256, Feb 1992

   For Robolocust, the ICP algorithm is used to perform scan matching
   between the current laser range finder scan and the previous one. The
   results of the scan matching procedure are used to correct odometry
   and then fed into the motion model for a grid-based FastSLAM algorithm
   so as to reduce the number of particles required by the
   Rao-Blackwellized particle filter (and, thereby, the total number of
   instances of the maps required).
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/slam/LoScanMatch.H $
// $Id: LoScanMatch.H 13570 2010-06-16 15:56:00Z mviswana $
//

#ifndef LOBOT_SCAN_MATCH_DOT_H
#define LOBOT_SCAN_MATCH_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/slam/LoPose.H"
#include "Robots/LoBot/util/triple.hh"

// Standard C++ headers
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Scan
   \brief An encapsulation of a laser range finder scan.

   This class holds the range readings made by lobot's laser range finder
   in both polar and Cartesian form. It also records the robot's pose
   corresponding to those readings.

   The scan matching algorithm takes two instances of this class and
   returns the transformation required to "convert" one scan to the other.
*/
class Scan {
   /// Each scan must be associated with a corresponding robot pose.
   Pose m_pose ;

   /// A scan consists of the range readings returned by the laser range
   /// finder in polar form as well as Cartesian coordinates. This inner
   /// class holds both these readings forms together in one structure.
public:
   class RangeReading {
      /// Each range reading is held in both polar and Cartesian forms.
      float m_r, m_x, m_y ;

   public:
      /// During initialization, the client must specify both polar and
      /// Cartesisan coordinates for a range reading.
      RangeReading(float r, float x, float y) ;

      /// Accessors.
      //@{
      float r() const {return m_r ;}
      float x() const {return m_x ;}
      float y() const {return m_y ;}
      //@}
   } ;

private:
   /// This data structure holds the range readings associated with a
   /// scan.
   std::vector<RangeReading> m_range_readings ;

public:
   /// When a scan object is created, it should be given the pose and a
   /// vector containing the range readings in polar form. The
   /// constructor will take care of deriving the Cartesian coordinates
   /// for each distance reading from the given pose and the polar
   /// ranges.
   //@{
   Scan(const Pose&, const std::vector<int>&) ;
   Scan(const Pose&, const std::vector<float>&) ;
   //@}

   /// Various accessors.
   //@{
   float x() const {return m_pose.x() ;}
   float y() const {return m_pose.y() ;}
   float theta() const {return m_pose.theta() ;}
   RangeReading& operator[](int i) {return m_range_readings[i] ;}
   const RangeReading& operator[](int i) const {return m_range_readings[i] ;}
   //@}

   /// Clean-up.
   ~Scan() ;
} ;

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Transformation
   \brief An encapsulation of a transformation for "converting" one scan
   to another.

   This class holds the rotation and translation required to transform a
   new scan into a reference scan.

   Given two scans, this implementation of the IDC algorithm returns a
   Transformation.
*/
class Transformation {
   /// The rotation required to convert one scan to another.
   float m_w ; // omega (as used in the Lu-Milios paper)

   /// The translation involved in converting one scan to another.
   float m_Tx, m_Ty ;

public:
   /// Initialization.
   //@{
   Transformation(float w, float Tx, float Ty) ;
   Transformation(const triple<float, float, float>&) ;
   //@}

   /// Accessors.
   //@{
   float w()  const {return m_w  ;}
   float Tx() const {return m_Tx ;}
   float Ty() const {return m_Ty ;}
   //@}
} ;

//----------------------------- FUNCTIONS -------------------------------

/// This function implements the Iterative Dual Correspondence algorithm
/// described by Lu and Milios to match two laser range finder scans and
/// return the transformation required to properly register the current
/// scan with reference to the previous one.
Transformation match_scans(const Scan& curr, const Scan& prev) ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

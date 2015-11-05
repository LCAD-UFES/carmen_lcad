/**
   \file  Robots/LoBot/misc/LoPointList.H
   \brief A class for storing a list of points.

   When we conduct an experiment to gauge lobot's performance, we collect
   several different lists of points: one to record the robot's
   trajectory from start to finish, another to record the locations where
   the robot's emergency stop behaviour was activated, another for the
   locations where the LGMD avoidance algorithm averted the robot away
   from an approaching obstacle; so on and so forth.

   This file defines a class that is used to store the above-mentioned
   point lists.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/metlog/LoPointList.H $
// $Id: LoPointList.H 13967 2010-09-18 08:00:07Z mviswana $
//

#ifndef LOBOT_POINT_LIST_DOT_H
#define LOBOT_POINT_LIST_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/metlog/LoPointTypes.H"

// Standard C++ headers
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::PointList
   \brief A container for storing lists of points.

   This class provides a convenient API for storing a list of points and
   performing some useful operations such as finding correspondences
   between two point lists. Additionally, though it is named PointList,
   it, in fact, behaves like a set, i.e., each point appears only once in
   the list. Thus, attempts to add duplicate points will be silently
   ignored.
*/
class PointList {
   /// This class's raison d'etre: to store a bunch of points.
   //@{
   typedef std::vector<mPoint> List ;
   List m_list ;
   //@}

public:
   /// Initalization: creates a point list with enough memory to
   /// initially store the specified number of points. If this memory
   /// fills up, not to worry, extra memory will be automagically
   /// acquired.
   PointList(int n = 0) ;

   /// This method adds a point to the point list. If the point already
   /// appears in the list, it will not be added a second time. Thus,
   /// this point list actually is a point set.
   void add(int x, int y) ;

   /// This operator appends the given point list to this one.
   PointList& operator+=(const PointList&) ;

   /// This method returns the number of points stored in this point
   /// list.
   int size() const {return m_list.size() ;}

   /// This method checks if the point list has anything in it or not.
   bool empty() const {return m_list.empty() ;}

   /// These types are defined for STL compatibility and to allow this
   /// class to be used directly in STL algorithms.
   //@{
   typedef List::const_reference const_reference ;
   typedef List::const_iterator iterator ;
   //@}

   /// These functions allow clients to walk through the point list using
   /// STL algorithms.
   //@{
   iterator begin() const {return m_list.begin() ;}
   iterator end()   const {return m_list.end()   ;}
   //@}

   /// This function deletes all the entries in the point list.
   void clear() {m_list.clear() ;}

   /// For each point in this point list, find the closest point in the
   /// given point list P and return the resulting point list.
   ///
   /// NOTE: If either this point list or the other point list is empty,
   /// this function will throw a lobot::misc_error with LOGIC_ERROR as
   /// the code.
   PointList find_correspondences(const PointList& P) const ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

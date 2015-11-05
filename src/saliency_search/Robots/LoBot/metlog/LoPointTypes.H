/**
   \file  Robots/LoBot/metlog/LoPointTypes.H
   \brief Various useful types used in several different modules of lomet
   data analysis program.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/metlog/LoPointTypes.H $
// $Id: LoPointTypes.H 13934 2010-09-14 23:17:01Z mviswana $
//

#ifndef LOBOT_POINT_TYPES_DOT_H
#define LOBOT_POINT_TYPES_DOT_H

//------------------------------ HEADERS --------------------------------

// Standard C++ headers
#include <utility>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//--------------------------- ENUMERATIONS ------------------------------

/// To gauge lobot's performance, we built an enclosure containing a
/// slalom course. The robot had to start at an initial position and work
/// its way to a goal location. As the robot moved through this obstacle
/// course, it logged a variety of information.
///
/// When we are analyzing these logs, we sift the information into
/// different lists. For example, one list holds the locations at which
/// the robot's obstacle avoidance algorithms failed and caused it to
/// bump into obstacles; another holds the locations where the robot's
/// emergency stop behaviour was activated; so on an so forth.
///
/// This enumeration lists the names of the different point lists.
enum PointListName {
   TRAJECTORY,
   EMERGENCY_STOP,
   EXTRICATE,
   LGMD_EXTRICATE,
   BUMP,
} ;

/// As its name suggests, a point list is container for several points.
/// For the lomet Robolocust metrics log data analysis program, points
/// are simply pairs of integers. Because lomet is a quick-and-dirty,
/// throwaway sort of program, we are not particularly interested
/// building robust and generally applicable abstractions. Thus, we
/// simply use an STL pair to represent lomet-related points.
///
/// DEVNOTE: misc/LoTypes.H also defines a type Point. To prevent
/// conflicts, we name this point type mPoint, where the 'm' prefix
/// stands for "lomet".
typedef std::pair<int, int> mPoint ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

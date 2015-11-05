/**
   \file  Robots/LoBot/slam/LoCoords.H
   \brief API for coordinate SLAM related system transformations.

   This file provides an API for converting real/physical coordinates to
   occupancy grid coordinates and vice versa.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/slam/LoCoords.H $
// $Id: LoCoords.H 13560 2010-06-11 12:58:57Z mviswana $
//

#ifndef LOBOT_SLAM_COORDINATE_SYSTEM_API_DOT_H
#define LOBOT_SLAM_COORDINATE_SYSTEM_API_DOT_H

//------------------------------ HEADERS --------------------------------

//----------------------------- NAMESPACE -------------------------------

namespace lobot  {
namespace Coords {

//----------------- COORDINATE SYSTEM CONVERSION API --------------------

/// This function converts real/physical coordinates to grid coordinates.
void to_grid(float rx, float ry, int* gx, int* gy) ;

/// This function converts grid coordinates to real/physical coordinates.
void to_real(int gx, int gy, float* rx, float* ry) ;

//-----------------------------------------------------------------------

} // end of Coords namespace
} // end of lobot  namespace

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

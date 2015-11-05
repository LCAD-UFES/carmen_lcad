/*!@file Image/OpticalFlowTypes.H Definition of the pyramid types */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Beobot2/Navigation/FOE_Navigation/OpticalFlowTypes.H $
// $Id: $
//

#ifndef OPTICALFLOWTYPES_H_DEFINED
#define OPTICALFLOWTYPES_H_DEFINED

#include <string> // for string conversions

// ######################################################################
//! Possible types of optical flow algorithms
enum OpticalFlowType {
  LucasKanade  = 0, //!< Lucas Kanade optical flow
  HornSchunck  = 1, //!< Horn & Schunck optical flow
  combo        = 2
  // if you add a new type here, also update the names in the function below!
};

//! number of known OpticalFlowTypes:
#define NBOPTICALFLOWTYPES 3

inline const char* opticalFlowTypeName(const OpticalFlowType p)
{
  static const char n[NBOPTICALFLOWTYPES][14] = {
    "LucasKanade", "HornSchunck", "combo"};
  return n[int(p)];
}


//! OpticalFlowType overload
/*! Format is "name" as defined by pyramidTypeName() in OpticalFlowTypes.H */
std::string convertToString(const OpticalFlowType val);

//! OpticalFlowType overload
/*! Format is "name" as defined by pyramidTypeName() in OpticalFlowTypes.H */
void convertFromString(const std::string& str, OpticalFlowType& val);

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

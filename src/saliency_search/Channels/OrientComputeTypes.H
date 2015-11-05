/*!@file Channels/OrientComputeTypes.H Definition of the pyramid types */

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
// Primary maintainer for this file: Dirk Walther <walther@caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/OrientComputeTypes.H $
// $Id: OrientComputeTypes.H 8827 2007-10-12 17:45:29Z rjpeters $
//

#ifndef ORIENTCOMPUTETYPES_H_DEFINED
#define ORIENTCOMPUTETYPES_H_DEFINED

#include <string>

// ######################################################################
//! Possible types computing orientations
enum OrientComputeType {
  ORISteerable       = 0, //!< use steerable filters
  ORIGabor           = 1, //!< use a Gabor filter
  ORIGaborEnergyNorm = 2  //!< energy-normalize the image, then use a Gabor filter
  // if you add a new type here, also update the names in the function below!
};

//! number of known OrientComputeTypes:
#define NBORIENTCOMPUTETYPES 3

inline const char* orientComputeTypeName(const OrientComputeType p)
{
  static const char n[NBORIENTCOMPUTETYPES][17] = {
    "Steerable", "Gabor", "GaborEnergyNorm"};
  return n[int(p)];
}

//! OrientComputeType overload
/*! Format is "name" as defined in OrientComputeTypes.H */
std::string convertToString(const OrientComputeType val);
//! OrientComputeType overload
/*! Format is "name" as defined in OrientComputeTypes.H */
void convertFromString(const std::string& str, OrientComputeType& val);


#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/*!@file Image/PyramidTypes.H Definition of the pyramid types */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/PyramidTypes.H $
// $Id: PyramidTypes.H 6674 2006-05-31 20:55:11Z lior $
//

#ifndef PYRAMIDTYPES_H_DEFINED
#define PYRAMIDTYPES_H_DEFINED

#include <string> // for string conversions

// ######################################################################
//! Possible types of pyramids
enum PyramidType {
  Gaussian3  = 0, //!< Corresponds to GaussianPyrBuilder with filter width 3.
  Gaussian5  = 1, //!< Corresponds to GaussianPyrBuilder with filter width 5.
  Gaussian9  = 2, //!< Corresponds to GaussianPyrBuilder with filter width 9.
  Laplacian5 = 3, //!< Corresponds to LaplacianPyrBuilder with filter width 5.
  Laplacian9 = 4, //!< Corresponds to LaplacianPyrBuilder with filter width 9.
  Oriented5  = 5, //!< Corresponds to OrientedPyrBuilder with filter width 5.
  Oriented9  = 6, //!< Corresponds to OrientedPyrBuilder with filter width 9.
  QuickLocalAvg = 7,
  QuickLocalMax = 8,
  BlockScale = 9,
  Gabor9 = 10
  // if you add a new type here, also update the names in the function below!
};

//! number of known PyramidTypes:
#define NBPYRAMIDTYPES 11

inline const char* pyramidTypeName(const PyramidType p)
{
  static const char n[NBPYRAMIDTYPES][14] = {
    "Gaussian3", "Gaussian5", "Gaussian9", "Laplacian5", "Laplacian9",
    "Oriented5", "Oriented9", "QuickLocalAvg", "QuickLocalMax",
    "BlockScale", "Gabor9"};
  return n[int(p)];
}


//! PyramidType overload
/*! Format is "name" as defined by pyramidTypeName() in PyramidTypes.H */
std::string convertToString(const PyramidType val);

//! PyramidType overload
/*! Format is "name" as defined by pyramidTypeName() in PyramidTypes.H */
void convertFromString(const std::string& str, PyramidType& val);

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

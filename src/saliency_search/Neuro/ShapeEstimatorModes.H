/*!@file Neuro/ShapeEstimatorModes.H definition of the shape estimator modes */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/ShapeEstimatorModes.H $
// $Id: ShapeEstimatorModes.H 14857 2011-07-26 01:14:21Z siagian $
//

#ifndef SHAPEESTIMATORMODES_H_DEFINED
#define SHAPEESTIMATORMODES_H_DEFINED

#include <string>

//! Possible ShapeEstimator modes
enum ShapeEstimatorMode {
  SEMnone               = 0, //! No shape estimation
  SEMfeatureMap         = 1, //! Based on winning feature map
  SEMconspicuityMap     = 2, //! Based on winning conspicuity map
  SEMsaliencyMap        = 3, //! Based on saliency map
  SEMMTfeatureMap       = 4, //! Based on MT feature map
  SEMContourBoundaryMap = 5, //! Based on contour boundary map
  SEMCSHistogram        = 6  //! Based on center surround histogram difference
  // if you add a new mode here, also update the names in the function below!
};

//! number of known ShapeEstimator modes:
#define NBSHAPEESTIMATORMODES 7

//! Returns name of given mode
inline const char* shapeEstimatorModeName(const ShapeEstimatorMode p)
{
  static const char n[NBSHAPEESTIMATORMODES][19] = {
    "None", 
    "FeatureMap", 
    "ConspicuityMap", 
    "SaliencyMap", 
    "MTfeatureMap",
    "ContourBoundaryMap",
    "CSHistogram"};
  return n[int(p)];
}


//! Possible ShapeEstimator smooth methods
enum ShapeEstimatorSmoothMethod {
  SESMnone =     0, //! Don't smooth at all
  SESMgaussian = 1, //! smooth by convolving with a Gaussian
  SESMchamfer =  2  //! smooth using opening and chamfering
  // if you add a new mode here, also update the names in the function below!
};

//! number of known ShapeEstimator smooth methods
#define NBSHAPEESTIMATORSMOOTHMETHODS 3

//! Returns name of given smooth method
inline const char* shapeEstimatorSmoothMethodName(const ShapeEstimatorSmoothMethod p)
{
  static const char n[NBSHAPEESTIMATORSMOOTHMETHODS][15] = {
    "None", "Gaussian", "Chamfer" };
  return n[int(p)];
}


//! ShapeEstimatorMode overload
/*! Format is "name" as defined in ShapeEstimatorModes.H */
std::string convertToString(const ShapeEstimatorMode val);
//! ShapeEstimatorMode overload
/*! Format is "name" as defined in ShapeEstimatorModes.H */
void convertFromString(const std::string& str, ShapeEstimatorMode& val);

//! ShapeEstimatorSmoothType overload
/*! Format is "name" as defined in ShapeEstimatorModes.H */
std::string convertToString(const ShapeEstimatorSmoothMethod val);

//! ShapeEstimatorSmoothType overload
/*! Format is "name" as defined in ShapeEstimatorModes.H */
void convertFromString(const std::string& str,
                       ShapeEstimatorSmoothMethod& val);


#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

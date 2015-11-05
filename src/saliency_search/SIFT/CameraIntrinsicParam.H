/*!@file SIFT/CameraIntrinsicParam.H Stores a camera intrinsic parameter*/

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/CameraIntrinsicParam.H $
// $Id: CameraIntrinsicParam.H 6182 2006-01-31 18:41:41Z rjpeters $
//

#ifndef CAMERAINTRINSICPARAM_H_DEFINED
#define CAMERAINTRINSICPARAM_H_DEFINED

#include "Image/Image.H"

//! A class that stores a camera's intrinsic parameter
class CameraIntrinsicParam {
public:

  //! Constructor
  inline CameraIntrinsicParam
  (double fcU, double fcV, double ccU, double ccV, double skew);

  //! get the camera param in a form of Camera Matrix
  inline Image<double> getCameraMatrix();

private:

  double itsFcU;      //!< Horizontal(U) component of Focal Length
  double itsFcV;      //!< Vertical(V) component of Focal Length
  double itsCcU;      //!< Horizontal principal point position in camera coor
  double itsCcV;      //!< Vertical principal point position in camera coor
  double itsSkew;     //!< skew angle in form of cotangent(theta)
};

// ######################################################################
// CameraIntrinsicParam I/O:
// ######################################################################


// ######################################################################
// ########## Implementation of inline functions
// ######################################################################

// ######################################################################
inline CameraIntrinsicParam::CameraIntrinsicParam
(double fcU, double fcV, double ccU, double ccV, double skew):
  itsFcU(fcU), itsFcV(fcV), itsCcU(ccU), itsCcV(ccV), itsSkew(skew)
{ }

// ######################################################################
inline Image<double> CameraIntrinsicParam::getCameraMatrix()
{
  Image<double> res(3,3,ZEROS);

  res.setVal(0,0, itsFcU);
  res.setVal(1,0, itsSkew*itsFcU);
  res.setVal(1,1, itsFcV);
  res.setVal(2,0, itsCcU);
  res.setVal(2,1, itsCcV);
  res.setVal(2,2, 1.0);

  return res;
}

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

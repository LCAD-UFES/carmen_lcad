/*!@file SIFT/SIFThough.H Data structure for SIFT Hough transform */

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
// Primary maintainer for this file: James Bonaiuto <bonaiuto@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/SIFThough.H $
// $Id: SIFThough.H 4938 2005-07-14 20:44:30Z rjpeters $
//

#ifndef SIFTHOUGH_H_DEFINED
#define SIFTHOUGH_H_DEFINED

#include <vector>

//! SIFThough accumalates data for the Hough transform used in SIFT
/*! This class is used to accumulate evidence for various possible
  geometric transforms which are compatible with pairs of matching
  keypoints. Here we do not solve the general affine transform
  problem, rather we rely on simple differences between the data
  available in Keypoint, namely: x, y, scale, and orientation. We tile
  each dimension into eight bins, yielding a total of 4096 bins. Each
  pair of matching keypoints votes for 16 bins with linear
  interpolation (i.e., for two adjacent bins in each dimension. The
  philosophy and implementation here is very similar to that of
  FeatureVector. */
class SIFThough
{
public :
  //! Constructor
  SIFThough();

  //! Destructor
  ~SIFThough();

  //! add value
  /*! Linearly add a value. Indices are as follows:
    0.0 <= dx <= 8.0 : [0 .. 7]   dx=2.0 falls equally between bins 1 and 2;
    0.0 <= dy <= 8.0 : [0 .. 7]   dy=2.0 falls equally between bins 1 and 2;
    0.0 <= do <= 8.0 : [0 .. 7]   do=4.0 falls equally between bins 3 and 4;
    0.0 <= ds <= 8.0 : [0 .. 7]   ds=4.0 falls equally between bins 3 and 4;
  Note that the orientation dimension is different as it has Dirichlet
  boundary conditions. */
  void addValue(const float dx, const float dy, const float doo,
                const float ds, const float value);

  //! Get the parameter values at the peak
  void getPeak(float& dx, float& dy, float& doo, float& ds) const;

private:
  std::vector<float> itsData;        // vector of data
};


// ######################################################################
// Inlined member functions
// ######################################################################



#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

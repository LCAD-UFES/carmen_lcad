/*!@file Image/MorphOps.H functions for binary morphology (dilate, erode, open, close) */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/MorphOps.H $
// $Id: MorphOps.H 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef IMAGE_MORPHOPS_H_DEFINED
#define IMAGE_MORPHOPS_H_DEFINED

class Dims;
template <class T> class Point2D;
template <class T> class Image;

//! Dilate the img with structuring element se with a given origin
/*! @param origin is the origin of the structuring element. This
  parameter is optional - the default is the center of the SE*/
template <class T>
Image<T> dilateImg(const Image<T>& img, const Image<T>& se,
                Point2D<int> origin = Point2D<int>(-1,-1));

//! Erode the img with structuring element se with a given origin
/*! @param origin is the origin of the structuring element. This
  parameter is optional - the default is the center of the SE*/
template <class T>
Image<T> erodeImg(const Image<T>& img, const Image<T>& se,
               Point2D<int> origin = Point2D<int>(-1,-1));

//! Open (erode and the close) img with struct element se with a given origin
/*! @param origin is the origin of the structuring element. This
  parameter is optional - the default is the center of the SE*/
template <class T>
Image<T> openImg(const Image<T>& img, const Image<T>& se,
              Point2D<int> origin = Point2D<int>(-1,-1));

//! Close (dilate and erode) img with struct element se with a given origin
/*! @param origin is the origin of the structuring element. This
  parameter is optional - the default is the center of the SE*/
template <class T>
Image<T> closeImg(const Image<T>& img, const Image<T>& se,
               Point2D<int> origin = Point2D<int>(-1,-1));

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_MORPHOPS_H_DEFINED

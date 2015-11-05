/*!@file Image/LowPass.H low-pass filtering and smoothing functions */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/LowPass.H $
// $Id: LowPass.H 8633 2007-07-26 00:00:28Z rjpeters $
//

#ifndef IMAGE_LOWPASS_H_DEFINED
#define IMAGE_LOWPASS_H_DEFINED

#include "Util/Promotions.H"

class Dims;
template <class T> class Image;

//! Low-pass filter, coeff * [0.25 0.5 0.25], applied separably in X and Y
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPass3(const Image<T_or_RGB>& src,
         const bool go_x = true, const bool go_y = true);

//! Low-pass filter, coeff * [0.25 0.5 0.25], applied in X
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPass3x(const Image<T_or_RGB>& src);

//! Low-pass filter, coeff * [0.25 0.5 0.25], applied in Y
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPass3y(const Image<T_or_RGB>& src);

//! Low-pass filter, Anderson's 5x5 separable
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPass5(const Image<T_or_RGB>& src,
         const bool go_x = true, const bool go_y = true);

//! Low-pass filter, Anderson's 5x5 separable, applied in X
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPass5x(const Image<T_or_RGB>& src);

//! Low-pass filter, Anderson's 5x5 separable, applied in Y
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPass5y(const Image<T_or_RGB>& src);

//! Low-pass filter, Anderson's 5x5 separable, applied in X, and X decimated by factor
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPass5xDecX(const Image<T_or_RGB>& src, const int factor = 2);

//! Low-pass filter, Anderson's 5x5 separable, applied in Y, and Y decimated by factor
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPass5yDecY(const Image<T_or_RGB>& src, const int factor = 2);

//! Low-pass filter, 9x9 separable
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPass9(const Image<T_or_RGB>& src,
         const bool go_x = true, const bool go_y = true);

//! Low-pass filter, 9x9 separable, applied in X
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPass9x(const Image<T_or_RGB>& src);

//! Low-pass filter, 9x9 separable, applied in Y
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPass9y(const Image<T_or_RGB>& src);

//! Low-pass filter, NxN separable
/*! Just forwards to lowPass3/lowPass5/lowPass9, but allows the filter
    width to be selected at runtime. */
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPass(const int N, const Image<T_or_RGB>& src,
        const bool go_x = true, const bool go_y = true);

//! Low-pass filter, NxN separable, applied in X
/*! Just forwards to lowPass3x/lowPass5x/lowPass9x, but allows the filter
    width to be selected at runtime. */
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPassX(const int N, const Image<T_or_RGB>& src);

//! Low-pass filter, NxN separable, applied in Y
/*! Just forwards to lowPass3y/lowPass5y/lowPass9y, but allows the filter
    width to be selected at runtime. */
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPassY(const int N, const Image<T_or_RGB>& src);

//! Apply a 3-pt median filter in the x direction
template <class T_or_RGB>
Image<T_or_RGB> median3x(const Image<T_or_RGB>& in);

//! Apply a 3-pt median filter in the y direction
template <class T_or_RGB>
Image<T_or_RGB> median3y(const Image<T_or_RGB>& in);

//! Apply a 3-pt median filter in the x and/or y directions;
template <class T_or_RGB>
Image<T_or_RGB> median3(const Image<T_or_RGB>& in,
                        bool go_x = true, bool go_y = true);

//! convolve with a separable 2D gaussian
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
convGauss(const Image<T_or_RGB>& src, const float sigmaX, const float sigmaY,
          const float threshperc);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_LOWPASS_H_DEFINED

/*!@file Image/Convolutions.H basic 1-D and 2-D filtering operations */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Convolutions.H $
// $Id: Convolutions.H 8643 2007-07-27 18:15:43Z rjpeters $
//

#ifndef IMAGE_CONVOLUTIONS_H_DEFINED
#define IMAGE_CONVOLUTIONS_H_DEFINED

#include "Image/Image.H"
#include "Util/Promotions.H"

class Dims;
template <class T> class Image;

enum ConvolutionBoundaryStrategy
  {
    CONV_BOUNDARY_ZERO,      ///< zero-pad, a.k.a. truncated
    CONV_BOUNDARY_CLEAN,     ///< normalize by the sum of the used filter coefficients
    CONV_BOUNDARY_REPLICATE  ///< replicate the nearest image pixel value
  };

//! Brute force, super inefficient 2D convolution, with C filter of Nx*Ny
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
convolve(const Image<T_or_RGB>& src, const float* filter,
         const int Nx, const int Ny,
         ConvolutionBoundaryStrategy boundary);

//! Brute force, super inefficient 2D convolution
template <class T_or_RGB> inline
Image<typename promote_trait<T_or_RGB, float>::TP>
convolve(const Image<T_or_RGB>& src, const Image<float>& filter,
         ConvolutionBoundaryStrategy boundary)
{
  return convolve(src, filter.getArrayPtr(),
                  filter.getWidth(), filter.getHeight(), boundary);
}

//! This is a somewhat-optimized 2-D convolution.
/*! It is optimized in the sense that there are special-case inner loops
  for (1) cases where the filter doesn't completely overlap with the
  local image patch because we're at the image boundary (in this case
  we need a specialized inner loop to handle these boundary
  conditions), and (2) cases where the filter and local image patch
  overlap completely (in which case we can use a more efficient inner
  loop). */
template <class T>
Image<typename promote_trait<T, float>::TP>
optConvolve(const Image<T>& src, const Image<float>& filter);

//! Brute force, super inefficient 2D convolution (truncated filter boundary)
/*! With normalization by the local image energy, a la HMAX */
template <class T>
Image<typename promote_trait<T, float>::TP>
convolveHmax(const Image<T>& src, const Image<float>& filter);

//! Separable filter, any size (size=0 for no filter)
/*! If you want to do horizontal-only or vertical-only filtering, then
    simply pass an empty filter (i.e., Image<float>()) for the other filter.

    @param boundary controls the boundary-handling strategy (i.e.,
    zero-fill, clean, replicate)

    HISTORICAL NOTE: This function takes over for what used to be many
    functions, including sepFilter(), sepFiltClean(), xFilter(),
    yFilter(), xFilterClean(), and yFilterClean(). First, the x+y
    variants are now collapsed into a single function; if you want
    x-only or y-only filtering, simply pass an empty image for the
    other filter. Second, the functions are collapsed across boundary
    handling strategy, where you can now select the boundary strategy
    at run-time with the ConvolutionBoundaryStrategy parameter.
*/
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
sepFilter(const Image<T_or_RGB>& src, const Image<float>& hFilter,
          const Image<float>& vFilter,
          ConvolutionBoundaryStrategy boundary);

template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
sepFilter(const Image<T_or_RGB>& src, const float* hFilter,
          const float* vFilter, const int hfsize, const int vfsize,
          ConvolutionBoundaryStrategy boundary);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_CONVOLUTIONS_H_DEFINED

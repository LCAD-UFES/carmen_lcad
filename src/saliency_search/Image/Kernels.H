/*!@file Image/Kernels.H Functions to construct various kinds of
  filter kernels
 */

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
// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Kernels.H $
// $Id: Kernels.H 12821 2010-02-11 07:15:07Z itti $
//

#ifndef IMAGE_KERNELS_H_DEFINED
#define IMAGE_KERNELS_H_DEFINED

#include "Util/Types.H" // for byte
#include "Image/Pixels.H"          //forPixRGB
#include <complex>


class Dims;
template <class T> class Point2D;
template <class T> class Image;

//! Create a DoG kernel for HMax
template <class T>
Image<T> dogFilterHmax(const float theta, const float gamma, const int halfsize, const float div);

//! Create a DoG kernel; if halfsize is 0, compute it automatically
template <class T>
Image<T> dogFilter(const float stddev, const float theta,
                   const int halfsize = 0);

//! Create a non oriented DoG kernel; if halfsize is 0, compute it automatically
template <class T>
Image<T> dogFilter(const float stddev, const int halfsize = 0);

//! Create a dog kernel for HMAX
template <class T>
Image<T> dogFilterHmax(const float stddev, const float theta,
                       const int cBegin, const int cEnd);


//Create a Gabor Kernel including color
Image<PixRGB<byte> > gaborFilterRGB(const float stddev, const float freq,
                                    const float theta,const float hueShift);

//! Create a Gabor kernel
template <class T>
Image<T> gaborFilter(const float stddev, const float period,
                     const float phase, const float theta,
                     const float = 0.0f, const float ampl = 1.0f);

//! Create a Gabor kernel with a a given scale and angle (in radians)
template <class T>
Image<T> gaborFilter(const float scale, const float theta);

//! Create a Gabor kernel with default 1*sigma^2 instead of 2*sigma^2
/*! this is essentailly used in some psycho metric experiments (for instance
, Polat and Sagi, 1994) instead
of 2*sigma^2 so it is included here, if your weird you may make sigMod
something other than 1.0 (or 2.0). As always feel free to be loopy */
template <class T>
Image<T> gaborFilter2(const float stddev, const float period,
                      const float phase, const float theta,
                      const float sigMod = 1.0F,
                      const float amplitude = 1.0F);

//! Produces a Gabor kernel with optionally unequal major+minor axis lengths.
/*!@param size determines the filter size (in pixels) - in the default case
  (-1), the size is computed from the stddev*/
Image<float> gaborFilter3(const float major_stddev, const float minor_stddev,
                          const float period, const float phase,
                          const float theta, int size = -1);

//! Create a 2D gaussian, compatible with gaborFilter2 for mask purposes
template <class T>
Image<T> gaussian2D(const float stddev, const float sigMod = 1.0F,
                    const float amplitude = 255.0F);

//! Create a 2D Gaussian blob centered at given coords
template <class T>
Image<T> gaussianBlob(const Dims& dims, const Point2D<int>& center,
                      const float sigmaX, const float sigmaY);

//! Create a 2D Gaussian blob centered at given coords, with peak value = 1.0
template <class T>
Image<T> gaussianBlobUnnormalized(const Dims& dims, const Point2D<int>& center,
                                  const float sigmaX, const float sigmaY);

//! Create a 1D binomial distribution kernel (shape of sz-by-1).
/*! These kernels are normalized to sum to 1, and are used (implicitly) in
    the various lowPass*() functions. */
Image<float> binomialKernel(const int sz);

//! Create a sinusoidal grating
template <class T>
Image<T> grating(const int width, const int height,
                 const float period, const float phase,
                 const float theta);

//! Generate 1D gaussian
/*! @param coeff a coefficient so that the peak of the Gaussian has the value
  coeff. If coeff is given as 0.0, then it will be computed internally as
  1/(sigma*sqrt(2*pi)), i.e., the normal Gaussian coefficient.
  @param sigma the Gaussian standard deviation, in pixels
  @param maxhw maximum allower filter half-width. If maxhw is 0 there
  is no limit.
  @param threshperc cut the spatial extent of the kernel so that only
  values greater than threshperc times the max remain in the
  kernel. This allows us to automatically compute the appropriate
  kernel size to use for a given sigma and approximation tolerance
  given by threshpec. */
template <class T>
Image<T> gaussian(const float coeff, const float sigma, const int maxhw,
                  const float threshperc = 1.0);

//! Build a long-range excitatory filter
template <class T>
Image<T> longRangeExcFilter(const float factor, const float orient);

//! Generate a fixation mask, i.e., a Gaussian blob
template <class T>
Image<T> fixationMask(const Dims& dims, const Point2D<int>& fixation,
                      const float pixperdeg, const T maxval,
                      const float sigma = 0.5f);

//! Generate a fixation mask, adding it to existing mask
template <class T>
Image<T> fixationMask(const Image<T>& mask, const Point2D<int>& fixation,
                      const float pixperdeg, const T maxval,
                      const float sigma = 0.5f);

//! Return an image of Dimensions d filled with 255s
Image<byte> twofiftyfives(Dims d);

//! Return an image of width w and height h filled with 255s
Image<byte> twofiftyfives(int w, int h);

//! Return a square image of width w filled with 255s
Image<byte> twofiftyfives(int w);

#endif // !IMAGE_KERNELS_H_DEFINED

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

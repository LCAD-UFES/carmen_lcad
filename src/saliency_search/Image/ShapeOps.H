/*!@file Image/ShapeOps.H Shape operations on Image
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/ShapeOps.H $
// $Id: ShapeOps.H 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef IMAGE_SHAPEOPS_H_DEFINED
#define IMAGE_SHAPEOPS_H_DEFINED

#include "Util/Assert.H"
#include "Util/Types.H"

#include <string>

class Dims;
template <class T> class Point2D;
class Rectangle;

template <class T> class Image;
template <class T> class PixRGB;

//! Compute quick and dirty scaling down of array.
template <class T_or_RGB>
Image<T_or_RGB> quickLocalAvg(const Image<T_or_RGB>& array, const int scale);

//! Compute local average of each 2x2 block and decimate by 2x2
template <class T_or_RGB>
Image<T_or_RGB> quickLocalAvg2x2(const Image<T_or_RGB>& array);

//! Compute quick and dirty scaling down of array.
template <class T_or_RGB>
Image<T_or_RGB> quickLocalMax(const Image<T_or_RGB>& array, const int scale);

//! Compute quick and dirty scaling down of array.
template <class T_or_RGB>
Image<T_or_RGB> quickLocalMin(const Image<T_or_RGB>& array, const int scale);

//! Dumb interpolation.
template <class T_or_RGB>
Image<T_or_RGB> quickInterpolate(const Image<T_or_RGB>& src,
                                 const int sfactor);

//! Double image size, using linear interpolation
/*! This function doubles the size of an image in the x and y
  directions. It differs from intXY() in that some interpolation will
  be made to create the new pixels, instead of duplication or
  insersion of zeros. It also differs slightly from using rescale() in
  the weights used for the interpolation. Specifically, we obtain:

  <PRE>
  ORIGa  NEW1  ORIGb  ...
  NEW2   NEW3  NEW4   ...
  ORIGc  NEW5  ORIGd  ...
  ...    ...   ...    ...

  where ORIGx are the pixels from the original image, and

  NEW1 = (ORIGa + ORIGb) / 2
  NEW2 = (ORIGa + ORIGc) / 2
  NEW3 = (ORIGa + ORIGb + ORIGc + ORIGd) / 4
  NEW4 = (ORIGb + ORIGd) / 2
  NEW5 = (ORIGc + ORIGd) / 2
  </PRE>

  Note that for the last row and column we assume something that is
  half-way between a duplication and a black border. */
template <class T_or_RGB>
Image<T_or_RGB> interpolate(const Image<T_or_RGB>& src);

//! Scale to new width & height using bilinear interpolation.
template <class T_or_RGB>
Image<T_or_RGB> rescaleBilinear(const Image<T_or_RGB>& src, const Dims& dims);

//! Scale to new width & height using bilinear interpolation.
template <class T_or_RGB>
Image<T_or_RGB> rescaleBilinear(const Image<T_or_RGB>& src, int width, int height);

//! Scale to new width & height with no interpolation.
template <class T_or_RGB>
Image<T_or_RGB> rescaleNI(const Image<T_or_RGB>& src, const Dims& dims);

//! Scale to new width & height with no interpolation.
template <class T_or_RGB>
Image<T_or_RGB> rescaleNI(const Image<T_or_RGB>& src, int width, int height);

/// Different types of image rescaling functions
enum RescaleType
{
  RESCALE_SIMPLE_NOINTERP,
  RESCALE_SIMPLE_BILINEAR,
  RESCALE_FILTER_BOX,
  RESCALE_FILTER_TRIANGLE,
  RESCALE_FILTER_BELL,
  RESCALE_FILTER_BSPLINE,
  RESCALE_FILTER_HERMITE,
  RESCALE_FILTER_LANCZOS3,
  RESCALE_FILTER_MITCHELL
};

/// Convert a character mnemonic into a RescaleType;
RescaleType getRescaleTypeFromChar(char ftype);

/// Convert RescaleType -> string
std::string convertToString(RescaleType ftype);

/// Convert string -> RescaleType
void convertFromString(const std::string& str1, RescaleType& ftype);

/// Generic image rescaling function with runtime-selectable rescaling algorithm
template <class T_or_RGB>
Image<T_or_RGB> rescale(const Image<T_or_RGB>& src, const Dims& newdims,
                        RescaleType ftype = RESCALE_SIMPLE_BILINEAR);

/// Generic image rescaling function with runtime-selectable rescaling algorithm
template <class T_or_RGB>
Image<T_or_RGB> rescale(const Image<T_or_RGB>& src,
                        const int width, const int height,
                        RescaleType ftype = RESCALE_SIMPLE_BILINEAR);


//! Scale to new width & height
/*! Calls rescale() or rescaleNI() depending on value of interp */
template <class T_or_RGB>
Image<T_or_RGB> rescaleOpt(const Image<T_or_RGB>& src,
                        const Dims& dims, const bool interp);

//! Scale to new width & height
/*! Calls rescale() or rescaleNI() depending on value of interp */
template <class T_or_RGB>
Image<T_or_RGB> rescaleOpt(const Image<T_or_RGB>& src, int width, int height,
                        const bool interp);

//! downscale an image using fancy widgets like anti-aliasing, resampling
/*! Adapted from PhotoPNMtools by Boris Van Schooten boris@13thmonkey.org
 */
template <class T>
Image<PixRGB<T> > downscaleFancy(const Image<PixRGB<T> >& src,
                                 int width, int height, int weighting_slope,
                                 bool no_weight_black);

//! Downsize using alternating lowpass/decimate operations.
/*! The width used internally in the lowpass filters is given by
    filterWidth (default = 9). */
template <class T>
Image<T> downSize(const Image<T>& src, const Dims& dims,
                  const int filterWidth = 9);

//! Downsize using alternating lowpass/decimate operations.
/*! The width used internally in the lowpass filters is given by
    filterWidth (default = 9). */
template <class T>
Image<T> downSize(const Image<T>& src, const int width, const int height,
                  const int filterWidth = 9);

//! Like downSize(), except image proportions don't have to match exactly.
/*! This operation applies alternating lowpass/decimate as long as the
    result is larger than the desired size, and then finally does a
    bilinear interpolation to the final size. If the desired size is
    actually larger than the input size, then the image is simply
    upscaled with a bilinear interpolation. The advantage of using
    this operation over just using rescale() directly is that
    rescale() may introduce aliasing if an image is being downsized by
    several ocatves; successive lowpass/decimate operations ensure
    that total image energy is preserved. */
Image<float> downSizeClean(const Image<float>& src, const Dims& new_dims,
                           const int filterWidth = 9);

//! Concatenate all image arrays, Nx arrays per line.
/*! Also reshape each array to (destX, destY) if these are != -1. */
template <class T_or_RGB>
Image<T_or_RGB> concatArray(const Image<T_or_RGB> arr[],
                            const int nbarr, const int Nx,
                            const int destX = -1, const int destY = -1);

//! Decimate in X and Y (take one every 'factor' pixels).
/*! @param xfactor factor by which to decimate in the x direction

    @param yfactor factor by which to decimate in the y direction; if
    this value is less than zero, then just take yfactor=xfactor
*/
template <class T_or_RGB>
Image<T_or_RGB> decXY(const Image<T_or_RGB>& src,
                      const int xfactor = 2, const int yfactor = -1);

//! Decimate in X (take one every 'factor' pixels).
template <class T_or_RGB>
Image<T_or_RGB> decX(const Image<T_or_RGB>& src, const int factor = 2);

//! Decimate in Y (take one every 'factor' pixels).
template <class T_or_RGB>
Image<T_or_RGB> decY(const Image<T_or_RGB>& src, const int factor = 2);

//! Blur and then decimate in Y. Equivalent to decY(sepFilter(src)).
/*! This is logically equivalent to constructing a boxcar filter of
    size factor, then doing decY(sepFilter(src, [], filter)). However,
    this implementation is optimized to be much faster, especially if
    factor is large, since we avoid computing the filter results that
    would just be thrown away by the subsequent decimation. */
template <class T>
Image<T> blurAndDecY(const Image<T>& src, const int factor);

//! Interpolate in X and Y (zero-pad if dupli == false, or duplicate).
template <class T_or_RGB>
Image<T_or_RGB> intXY(const Image<T_or_RGB>& src, const bool dupli);

//! Interpolate in X (zero-pad if dupli == false or duplicate).
template <class T_or_RGB>
Image<T_or_RGB> intX(const Image<T_or_RGB>& src, const bool dupli);

//! Interpolate in Y (zero-pad if dupli == false or duplicate).
template <class T_or_RGB>
Image<T_or_RGB> intY(const Image<T_or_RGB>& src, const bool dupli);

//! Zoom up an image by duplicating pixels.
/*! Separate scaling factors can be given for the x and y dimensions.

    @param src Input image.
    @param xzoom Zoom factor for x dimension (default 2).
    @param yzoom Zoom factor for y dimension. If negative (the default),
    then yzoom is taken to be the same as xzoom.
*/
template <class T_or_RGB>
Image<T_or_RGB> zoomXY(const Image<T_or_RGB>& src,
                       int xzoom = 2, int yzoom = -1);

//! Rotate an image about (x,y) by ang(in Radians), without interpolation
template <class T_or_RGB>
Image<T_or_RGB> rotate(const Image<T_or_RGB>& srcImg,
                       const int x, const int y,
                       const float ang);

//! Zoom by a factor numer/denom if numer>denom, else decimate by a factor of denom/numer
template <class T>
inline Image<T> zoomRational(const Image<T>& in,
                             const uint numer, const uint denom)
{
  ASSERT(numer > 0);
  ASSERT(denom > 0);

  return (numer > denom)
    ? zoomXY(in, numer / denom)
    : decXY(in, denom / numer);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !IMAGE_SHAPEOPS_H_DEFINED

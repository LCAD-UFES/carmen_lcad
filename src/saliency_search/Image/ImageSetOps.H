/*!@file Image/ImageSetOps.H Free functions operating on sets of images
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/ImageSetOps.H $
// $Id: ImageSetOps.H 14633 2011-03-23 22:55:54Z dberg $
//

#ifndef IMAGESET_OPS_H_DEFINED
#define IMAGESET_OPS_H_DEFINED

#include "Util/Types.H" // for uint

class Dims;
template <class T> class Image;
template <class T> class ImageSet;
template <class T> class Range;

// ############################################################
// ############################################################
/*! @name ImageSet processing functions

    Not all of these functions are templates, simply because alternate
    versions aren't currently needed.
*/
//@{

//! Returns true if all images in the set are the same size.
/*! Also returns true for the empty set. */
template <class T_or_RGB>
bool isHomogeneous(const ImageSet<T_or_RGB>& x);

//! Check whether the pyramid is dyadic.
/*! A dyadic pyramid is one in which each level is one half the width
    and one half the height of the preceding level. NOTE that this
    function is defined here, in ImageSetOps.H, rather than in
    Pyramid_Ops.H, in order to avoid a cyclic dependency between those
    two modules. ImageSetOps needs isDyadic() in order to implement
    makeImageArray(). */
template <class T_or_RGB>
bool isDyadic(const ImageSet<T_or_RGB>& pyr);

//! Returns the sum of all the images in the set. Requires isHomogeneous().
template <class T>
Image<T> sum(const ImageSet<T>& x);

//! Returns the sum of all the images in the set. Requires isHomogeneous().
template <class T>
Image<T> mean(const ImageSet<T>& x);

//! Returns the overall range of all images in the set.
template <class T>
Range<T> rangeOf(const ImageSet<T>& x);

//! Make an ImageSet from the s'th level from each of the N input sets.
/*! This is useful, for example, in transforming an array of ImageSets
    from multiple-scales-per-set/one-orientation-per-set into the
    reverse, multiple-orientations-per-set/one-scale-per-set.
*/
template <class T>
ImageSet<T> takeSlice(const ImageSet<T>* sets, uint N, uint s);

//! Make a single image-array from the set (see concatArray() in Image_ShapeOps).
/*! @param Nx Number of columns; if negative, the result will be roughly square.
    @param grid_width Pixel width of grid separators to be overlaid on image.
    @param grid_color Color in which to draw grid.
    @param destX Width to which each image is reshaped;
                          if negative, reshape to size of first image.
    @param destY Height to which each image is reshaped;
                          if negative, reshape to size of first image.
*/
template <class T>
Image<T> makeImageArray(const ImageSet<T>& x,
                        int Nx = -1, int grid_width = 1, T grid_color = T(),
                        int destX = -1, int destY = -1);

//! Make a new ImageSet by downscaling the input by the given number of octaves.
template <class T>
ImageSet<T> reduce(const ImageSet<T>& x, int octaves);

//! Make a new ImageSet by rescaling each input image to the given dimensions.
template <class T>
ImageSet<T> rescale(const ImageSet<T>& x, const Dims& dims);

//! Generate a set of oriented filter outputs for a given set of orientations.
/*! Caller is responsible for passing input in the form of
    (image - lowPass(image)) */
ImageSet<float> orientedFilterSet(const Image<float>& lowPassedInput,
                                  float period,
                                  const float* angles, const uint numAngles);

//!split the positive and negative values for each image in x
void splitPosNeg(const ImageSet<float>& x, ImageSet<float>& pos, ImageSet<float>& neg);

//@}


// ############################################################
// ############################################################
/*! @name ImageSet mathematical operators

    This set of operators is obviously incomplete; more operators can
    be added as they are needed. There are three basic kinds of
    operators:

    (1) ImageSet-scalar ops. These place no restrictions on the
        ImageSet.

    (2) ImageSet-Image ops. These require the ImageSet to be
        homogeneous (all images the same size), and require the Image
        argument to be the same size as well.

    (3) ImageSet-ImageSet ops. These require the two sets to have the
        same number of images, and for each pair of corresponding
        images to be the same size.
*/

//! x -= y, for each image in x; requires isHomogeneous(x).
template <class T>
ImageSet<T>& operator-=(ImageSet<T>& x, const Image<T>& y);

//! x += y, for each image in x; requires isHomogeneous(x).
template <class T>
ImageSet<T>& operator+=(ImageSet<T>& x, const Image<T>& y);

//! x *= y, for each image in x; requires isHomogeneous(x).
template <class T>
ImageSet<T>& operator*=(ImageSet<T>& x, const Image<T>& y);

//! x /= y, for each image in x; requires isHomogeneous(x).
template <class T>
ImageSet<T>& operator/=(ImageSet<T>& x, const Image<T>& y);

//! x -= v, for each image in x
template <class T>
ImageSet<T>& operator-=(ImageSet<T>& x, const T& v);

//! x += v, for each image in x
template <class T>
ImageSet<T>& operator+=(ImageSet<T>& x, const T& v);

//! x *= v, for each image in x
template <class T>
ImageSet<T>& operator*=(ImageSet<T>& x, const T& v);

//! x /= v, for each image in x
template <class T>
ImageSet<T>& operator/=(ImageSet<T>& x, const T& v);

//! x - v, for each image in x
template <class T>
ImageSet<T> operator-(ImageSet<T>& x, const T& v);

//! x + v, for each image in x
template <class T>
ImageSet<T> operator+(ImageSet<T>& x, const T& v);

//! x * v, for each image in x
template <class T>
ImageSet<T> operator*(ImageSet<T>& x, const T& v);

//! x / v, for each image in x
template <class T>
ImageSet<T> operator/(ImageSet<T>& x, const T& v);

//! x -= y, for each image in x with the respective image in y
template <class T>
ImageSet<T>& operator-=(ImageSet<T>& x, const ImageSet<T>& y);

//! x += y, for each image in x with the respective image in y
template <class T>
ImageSet<T>& operator+=(ImageSet<T>& x, const ImageSet<T>& y);

//! x *= y, for each image in x with the respective image in y
template <class T>
ImageSet<T>& operator*=(ImageSet<T>& x, const ImageSet<T>& y);

//! x /= y, for each image in x with the respective image in y
template <class T>
ImageSet<T>& operator/=(ImageSet<T>& x, const ImageSet<T>& y);

//! clampedDiff(b,c) for each image
template <class T>
ImageSet<T> clampedDiff(const ImageSet<T>& b, const ImageSet<T>& c);

//@}

// ############################################################
// ############################################################
/*! @name In-place ImageSet modification functions

    Any functions which modify an ImageSet in-place should adhere to
    the following conventions:

    (1) The to-be-modified ImageSet should be passed by non-const
        reference as the first argument to the function.

    (2) The function should be named with a "do" prefix, to
        distinguish it from functions that work by returning a new
        result instead of modifying an argument in place.

    Not all of these functions are templates, simply because alternate
    versions aren't currently needed.
*/
//@{

//! x = rectify(x), for each image in x
template <class T>
void doRectify(ImageSet<T>& x);

//! For each image in x, replace values < threshold with newval
template <class T>
void doLowThresh(ImageSet<T>& x, const T threshold, const T newval = T());

//! For each image in x, replace values whose abs is < threshold with newval
template <class T>
void doLowThreshAbs(ImageSet<T>& x, const T threshold, const T newval = T());

//! For each image in x, take the square root at every pixel
template <class T>
void doSqrt(ImageSet<T>& x);

//! For each image in x, take the square at every pixel
template <class T>
void doSquared(ImageSet<T>& x);

//! Subtract the overall mean from each image
void doMeanNormalize(ImageSet<float>& x);

//! Normalize so overall max value is 1.0
void doOneNormalize(ImageSet<float>& x);

//! Divide image by the local image energy, then subtract overall mean.
void doEnergyNorm(ImageSet<float>& x);

//! Multiply each image by the corresponding bias
void doApplyBiases(ImageSet<float>& x, const float* biases);

//! x += v * y, for each image in x and y, respectively
void doAddWeighted(ImageSet<float>& x, const ImageSet<float>& y, float v);

//! x = v, for each image in x
void doClear(ImageSet<float>& x, float v);

//@}

#endif // !IMAGESET_OPS_H_DEFINED

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

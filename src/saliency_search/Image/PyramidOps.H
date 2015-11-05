/*!@file Image/PyramidOps.H Free functions operating on pyramid data structures */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/PyramidOps.H $
// $Id: PyramidOps.H 14473 2011-02-03 20:36:39Z dberg $
//

#ifndef IMAGE_PYRAMIDOPS_H_DEFINED
#define IMAGE_PYRAMIDOPS_H_DEFINED

#include "Image/Convolutions.H"
#include "Image/PyramidTypes.H"
#include "Util/Promotions.H"
#include "Util/Types.H"

template <class T> class Point2D;
template <class T> class Image;
template <class T> class ImageSet;

// ############################################################
// ##### Dyadic pyramid pixel operations:
// ############################################################

//! Get value of a pixel from a dyadic pyramid at a given level and coordinates
/*! @param pix The pixel value returned
  @param p Coordinates given at scale 0 (base pyramid level);
  bilinear interpolation will be used to compute pixel value at
  given level.
  @param lev The pyramid level to get the pixel value from. */
template <class T_or_RGB>
T_or_RGB getPyrPixel(const ImageSet<T_or_RGB>& pyr,
                     const Point2D<int>& p, const uint lev);

//! Get value of a pixel from a dyadic pyramid at a given level and coordinates, no interpolation
/*! @param pix The pixel value returned
  @param p Coordinates given at scale 0 (base pyramid level);
  This version does not use interpolation.
  @param lev The pyramid level to get the pixel value from. */
template <class T_or_RGB>
T_or_RGB getPyrPixelNI(const ImageSet<T_or_RGB>& pyr,
                       const Point2D<int>& p, const uint lev);

//! Get value of a pixel from a dyadic pyramid with trilinear interpolation
template <class T_or_RGB>
T_or_RGB getPyrPixel(const ImageSet<T_or_RGB>& pyr,
                     const float x, const float y, const float z);

// ############################################################
// ##### Pyramid builder functions:
// ############################################################

//! Uses a Gaussian filter of size \a filterSize.
template <class T_or_RGB>
ImageSet<T_or_RGB> buildPyrGaussian(const Image<T_or_RGB>& image,
                                    int firstlevel, int depth,
                                    int filterSize);

//! Uses a Gaussian filter of size 5 designed for radially transformed images
template <class T_or_RGB>
ImageSet<T_or_RGB> buildRadialPyrGaussian(const Image<T_or_RGB>& image,
                                          int firstlevel, int depth);

//! Uses an arbitrary filter given as argument
template <class T_or_RGB>
ImageSet<T_or_RGB> buildPyrConvolve(const Image<T_or_RGB>& image,
                                    int firstlevel, int depth,
                                    const Image<float>& filter,
                                    ConvolutionBoundaryStrategy boundary);

//! Uses a Laplacian filter of size \a filterSize.
template <class T_or_RGB>
ImageSet<T_or_RGB> buildPyrLaplacian(const Image<T_or_RGB>& image,
                                     int firstlevel, int depth,
                                     int filterSize);

//! Uses an oriented Laplacian filter with a given size/orientation/intensity.
/*! This is a fast approximation to gabor convolution.

    @param usetab Whether to use trig tables to speed up the
    computations at the cost of some loss of precision.
*/
template <class T_or_RGB>
ImageSet<T_or_RGB> buildPyrOrientedFromLaplacian(const ImageSet<T_or_RGB>& laplacian,
                                                 int filterSize,
                                                 float theta, float intens = 10.0f,
                                                 const bool usetab = false);

//! Uses an oriented Laplacian filter with a given size/orientation/intensity.
/*! This is a fast approximation to gabor convolution.

    @param usetab Whether to use trig tables to speed up the
    computations at the cost of some loss of precision.
*/
template <class T_or_RGB>
ImageSet<T_or_RGB> buildPyrOriented(const Image<T_or_RGB>& image,
                                    int firstlevel, int depth,
                                    int filterSize,
                                    float theta, float intens = 10.0f,
                                    const bool usetab = false);

//! Just fills the deepest level with the local image average at that scale.
template <class T_or_RGB>
ImageSet<T_or_RGB> buildPyrLocalAvg(const Image<T_or_RGB>& image, int depth);

//! Each level contains local 2x2 averages of the previous level
template <class T_or_RGB>
ImageSet<T_or_RGB> buildPyrLocalAvg2x2(const Image<T_or_RGB>& image, int depth);

//! Just fills the deepest level with the local image maximum at that scale.
template <class T_or_RGB>
ImageSet<T_or_RGB> buildPyrLocalMax(const Image<T_or_RGB>& image, int depth);

//! Build a pyramid of type given by the PyramidType.
template <class T_or_RGB>
ImageSet<T_or_RGB> buildPyrGeneric(const Image<T_or_RGB>& image,
                                   int firstlevel, int depth,
                                   const PyramidType typ,
                                   const float gabor_theta = 0.0f,
                                   const float gabor_intens = 10.0f);

enum GaborFlags
  {
    DO_ENERGY_NORM = 1,
    DO_LAPLACIAN = 2,
    DO_CLAMPED_DIFF = 4,
    NO_ABS = 8
  };

//! Build a Gabor Pyramid with a given angle, filter period and elongation
/*!@param size determines the filter size (in pixels) - in the default case
  (-1), the size is computed from the filter_period and elongation*/
ImageSet<float> buildPyrGabor(const ImageSet<float>& gaussianPyr,
                              float angle, float filter_period,
                              float elongation = 1.0,
                              int size = -1, int flags = 0);

//! Build a Gabor Pyramid with a given angle, filter period and elongation
/*!@param size determines the filter size (in pixels) - in the default case
  (-1), the size is computed from the filter_preiod and elongation*/
ImageSet<float> buildPyrGabor(const Image<float>& img,
                              int firstlevel, int depth, float angle,
                              float filter_period, float elongation = 1.0,
                              int size = -1, int flags = 0);

// ############################################################
// ##### Processing Functions:
// ############################################################

//! Compute center-surround between images at lev1 and lev2
template <class T>
Image<T> centerSurround(const ImageSet<T>& pyr,
                        const int lev1, const int lev2,
                        const bool absol = false,
                        const ImageSet<float>* clipPyr = 0);

//! Compute center-surround between images, splitting positive and negative
template <class T>
void centerSurround(const ImageSet<T>& pyr,
                    const int lev1, const int lev2,
                    Image<T>& pos, Image<T>& neg,
                    const ImageSet<float>* clipPyr = 0);

//! Compute single-opponent center-surround between images at lev1 and lev2
/*! This is essentially cpyr[lev1] - spyr[lev2] */
template <class T>
Image<T> centerSurroundSingleOpponent(const ImageSet<T>& cpyr,
                                      const ImageSet<T>& spyr,
                                      const int lev1, const int lev2,
                                      const bool absol = false,
                                      const ImageSet<float>* clipPyr = 0);

//! Compute single-opp center-surr between imgs, splitting pos and neg
template <class T>
void centerSurroundSingleOpponent(const ImageSet<T>& cpyr,
                                  const ImageSet<T>& spyr,
                                  const int lev1, const int lev2,
                                  Image<T>& pos, Image<T>& neg,
                                  const ImageSet<float>* clipPyr = 0);

//! Compute center-surround between difference images at lev1 and lev2
/*! Difference images are obtained at each level by taking the difference
  between the image from *this minus the image from other_pyr */
template <class T>
Image<T> centerSurroundDiff(const ImageSet<T>& pyr1,
                            const ImageSet<T>& pyr2,
                            const int lev1, const int lev2,
                            const bool absol = false,
                            const ImageSet<float>* clipPyr = 0);

//! Compute center-surround differnces, splitting pos & neg values
template <class T>
void centerSurroundDiff(const ImageSet<T>& pyr1,
                        const ImageSet<T>& pyr2,
                        const int lev1, const int lev2,
                        Image<T>& pos, Image<T>& neg,
                        const ImageSet<float>* clipPyr = 0);

//! Compute single-opp center-surround btw difference images at lev1 and lev2
/*! Difference images are obtained at each level by taking the difference
  between the image from *this minus the image from other_pyr */
template <class T>
Image<T> centerSurroundDiffSingleOpponent(const ImageSet<T>& cpyr1,
                                          const ImageSet<T>& cpyr2,
                                          const ImageSet<T>& spyr1,
                                          const ImageSet<T>& spyr2,
                                          const int lev1, const int lev2,
                                          const bool absol = false,
                                          const ImageSet<float>* clipPyr = 0);

//! Compute single-opp center-surround differnces, splitting pos & neg values
template <class T>
void centerSurroundDiffSingleOpponent(const ImageSet<T>& cpyr1,
                                      const ImageSet<T>& cpyr2,
                                      const ImageSet<T>& spyr1,
                                      const ImageSet<T>& spyr2,
                                      const int lev1, const int lev2,
                                      Image<T>& pos, Image<T>& neg,
                                      const ImageSet<float>* clipPyr = 0);

//! generate a weighted blur
/*! The higher the modulator value (0..255), the more blurring at that
  location */
template <class T_or_RGB>
Image<T_or_RGB> weightedBlur(const Image<byte>& modulator,
                             const ImageSet<T_or_RGB>& pyr);

//! Generate a foveation blur
/*! Mask should be 255 inside the object and 0 outside; the pyramid
  should be a Gaussian pyramid and will be used to create the
  eccentricity-dependent blur. The deeper the pyramid, the more severe
  the blur */
template <class T_or_RGB>
Image<T_or_RGB> foveate(const Image<byte>& mask,
                        const ImageSet<T_or_RGB>& pyr);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !IMAGE_PYRAMIDOPS_H_DEFINED

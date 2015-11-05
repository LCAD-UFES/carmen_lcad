/*!@file Image/FilterOps.H Filtering operations on Image
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/FilterOps.H $
// $Id: FilterOps.H 13801 2010-08-19 19:54:49Z lior $
//


#ifndef IMAGE_FILTEROPS_H_DEFINED
#define IMAGE_FILTEROPS_H_DEFINED

#include "Util/Promotions.H"

// NOTE: Many of the functions that were previously in this file have
// been split into separate source files, for better logical
// organization and to reduce recompile times. The basic filtering
// functions, including all of the "convolve" variants as well as the
// separable-filter functions are now found in Image/Convolutions.H.
// The lowpass filtering functions, both the optimized 3-, 5-, and
// 9-point versions as well as the generic versions, are now found in
// Image/LowPass.H. Binary morphology operations
// (dilate/erode/open/close) are now found in Image/MorphOps.H.

// Include these for backward compatibility so that Image/FilterOps.H
// still brings in all the same declarations that it did previously:
#include "Image/Convolutions.H"
#include "Image/LowPass.H"

class Dims;
template <class T> class Point2D;
template <class T> class Image;

//! template matching correlation
template <class T>
  Image<typename promote_trait<T, float>::TP>
correlation(const Image<T>& src, const Image<float>& filter);

//! template matching useing opencv defaults to correlation if no openCV installed
//! defaults to CV_TM_SQDIFF
template <class T>
  Image<typename promote_trait<T, float>::TP>
templMatch(const Image<T>& src, const Image<float>& filter, int method = 0);

//! Spatial pooling taking the max value over a neighborhood
/*! Compute max over a rectangle of size (si, sj) starting at positions
  that increment in steps (sti, stj) */
template <class T>
Image<typename promote_trait<T, float>::TP>
spatialPoolMax(const Image<T>& src, const int si, const int sj,
               const int sti, const int stj);

//! Feature/spatial pooling for the S2 layer of Hmax
template <class T>
float featurePoolHmax(const Image<T>& img1, const Image<T>& img2,
                      const Image<T>& img3, const Image<T>& img4,
                      const int si, const int sj, const float s2t);


//! Oriented filter along theta (deg), spatial freq k
/*! This filter works on the difference between the image and itself
    low-passed. It modulates this difference by a complex sinusoid,
    and then low-passes the real and imaginary parts, and finally
    takes the modulus. This method is by Greenspan et al. (1994).
    CAUTION: *you* must provide as input (i.e., in "this") the
    difference between an image and itself low-pass filtered.
    CAUTION: results are possibly bad with the integer versions. Use
    float.

    @param usetab Whether to use trig lookup tables internally to get
    sin and cos values. This may give up to a 2x speedup, at the cost
    of some minor loss of precision,
*/
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
orientedFilter(const Image<T_or_RGB>& src, const float k, const float theta,
               const float intensity = 1.0, const bool usetab = false);

//! Compute center-surround difference, taking the abs if absol==true
/*! To the (hires) center will be substracted a (lowres)
  surround. Image sizes must be int multiples.  CAUTION: values will
  be clamped to the range of your template type.
  @param absol if true, take the absolute value of the difference (otherwise,
  negative values are clamped to zero). */
template <class T>
Image<T> centerSurround(const Image<T>& center,
                        const Image<T>& surround,
                        const bool absol = false);

//! Compute center-surround difference, without clamping or rectifying
/*! To the (hires) center will be substracted a (lowres)
  surround. Image sizes must be int multiples. This is implemented in
  a different function from centerSurround() as it returns two images,
  for the positive and negative values of the difference */
template <class T>
void centerSurround(const Image<T>& center,
                    const Image<T>& surround,
                    Image<T>& pos, Image<T>& neg);

//! compute double-opponent response
/*! (cplus - cminus) [-] (splus - sminus) where [-] is subtraction of two
  images of possibly different sizes, followed by absolute value. The result
  will have the size of the larger (center) image. */
template <class T>
Image<typename promote_trait<T, float>::TP>
doubleOpp(const Image<T>& cplus, const Image<T>& cminus,
          const Image<T>& splus, const Image<T>& sminus);

//! Compute average orientation and strength using steerable filters
template <class T>
void avgOrient(const Image<T>& src,
               Image<float>& orient, Image<float>& strength);

//! Divide image by the local image energy, then subtract overall mean.
template <class T>
Image<T> energyNorm(const Image<T>& img);

//! Compute response of a junction detector filter
/*! In the full implementation here, the junction filter responds only
if the relevant features are present and the irrelevant features are
absent.
  @param i0 image filtered by a 0deg (horisontal) Gabor filter
  @param i45 image filtered by a 45deg Gabor filter
  @param i90 image filtered by a 90deg (vertical) Gabor filter
  @param i135 image filtered by a 135deg Gabor filter
  @param r boolean array of which features are considered relevant to
  the junction. The first element is for the horizontal (i0) feature
  at (x+dx, y), the second for the 45deg (i45) feature at (x+dx,
  y-dy), and so on, going counterclockwise:

  :    3   2   1
  :      \ | /
  :   4 -- o -- 0
  :      / | \
  :    5   6   7

  @param dx horizontal distance from current pixel at which the
  presence or absence of a given feature should be checked for.
  @param dy vertical distance from current pixel at which the
  presence or absence of a given feature should be checked for. */
template <class T>
Image<T> junctionFilterFull(const Image<T>& i0,  const Image<T>& i45,
                            const Image<T>& i90, const Image<T>& i135,
                            const bool r[8],     const int dx = 6,
                            const int dy = 6,
                            const bool useEuclidDiag = true);

//! Compute response of a junction detector filter, partial implementation
/*! In the partial implementation here, the junction filter responds
  when the relevant features are present, without consideration of
  what the values of the irrelevant features might be.
  @param i0 image filtered by a 0deg (horisontal) Gabor filter
  @param i45 image filtered by a 45deg Gabor filter
  @param i90 image filtered by a 90deg (vertical) Gabor filter
  @param i135 image filtered by a 135deg Gabor filter
  @param r boolean array of which features are considered relevant to
  the junction. The first element is for the horizontal (i0) feature
  at (x+dx, y), the second for the 45deg (i45) feature at (x+dx,
  y-dy), and so on, going counterclockwise:

  :    3   2   1
  :      \ | /
  :   4 -- o -- 0
  :      / | \
  :    5   6   7

  @param dx horizontal distance from current pixel at which the
  presence or absence of a given feature should be checked for.
  @param dy vertical distance from current pixel at which the
  presence or absence of a given feature should be checked for. */
template <class T>
Image<T> junctionFilterPartial(const Image<T>& i0, const Image<T>& i45,
                               const Image<T>& i90, const Image<T>& i135,
                               const bool r[8], const int dx = 6,
                               const int dy = 6,
                               const bool useEuclidDiag = false);


//! Compute response of a MST detector filter
/*! In the full implementation here, the MST filter responds only
if the relevant features are present and the irrelevant features are
absent.
  @param i0 image filtered by a 0deg (horisontal) Gabor filter
  @param i45 image filtered by a 45deg Gabor filter
  @param i90 image filtered by a 90deg (vertical) Gabor filter
  @param i135 image filtered by a 135deg Gabor filter
  @param r boolean array of which features are considered relevant to
  the MST. The first element is for the horizontal (i0) feature
  at (x+dx, y), the second for the 45deg (i45) feature at (x+dx,
  y-dy), and so on, going counterclockwise:

  :    3   2   1
  :      \ | /
  :   4 -- o -- 0
  :      / | \
  :    5   6   7

  @param dx horizontal distance from current pixel at which the
  presence or absence of a given feature should be checked for.
  @param dy vertical distance from current pixel at which the
  presence or absence of a given feature should be checked for. */
template <class T>
Image<T> MSTFilterFull(const Image<T>& i0,  const Image<T>& i45,
                            const Image<T>& i90, const Image<T>& i135,
                            const bool r[8],     const int dx = 6,
                            const int dy = 6,
                            const bool useEuclidDiag = true);

//! Compute response of a MST detector filter, partial implementation
/*! In the partial implementation here, the MST filter responds
  when the relevant features are present, without consideration of
  what the values of the irrelevant features might be.
  @param i0 image filtered by a 0deg (horisontal) Gabor filter
  @param i45 image filtered by a 45deg Gabor filter
  @param i90 image filtered by a 90deg (vertical) Gabor filter
  @param i135 image filtered by a 135deg Gabor filter
  @param r boolean array of which features are considered relevant to
  the MST. The first element is for the horizontal (i0) feature
  at (x+dx, y), the second for the 45deg (i45) feature at (x+dx,
  y-dy), and so on, going counterclockwise:

  :    3   2   1
  :      \ | /
  :   4 -- o -- 0
  :      / | \
  :    5   6   7

  @param dx horizontal distance from current pixel at which the
  presence or absence of a given feature should be checked for.
  @param dy vertical distance from current pixel at which the
  presence or absence of a given feature should be checked for. */
template <class T>
Image<T> MSTFilterPartial(const Image<T>& i0, const Image<T>& i45,
                               const Image<T>& i90, const Image<T>& i135,
                               const bool r[8], const int dx = 6,
                               const int dy = 6,
                               const bool useEuclidDiag = false);



//! Compute the magnitude of the gradient of an image
/*! This is an approximation to the gradient magnitude as used in the
  SIFT code. output(x, y) = sqrt([input(x+1, y) - input(x-1, y)]^2 +
  [input(x, y+1) - input(x, y-1)]^2)*/
template <class T>
Image<typename promote_trait<T, float>::TP> gradientmag(const Image<T>& input);

//! Compute the orientation of the gradient of an image
/*! This is an approximation to the gradient orientation as used in
  the SIFT code. output(x, y) = atan2(input(x, y+1) - input(x, y-1),
  input(x+1, y) - input(x-1, y)). Result is in radians. A value of 0
  corresponds to a purely horizontal rightward gradient, other values
  relating to that in a clockwise manner. */
template <class T>
Image<typename promote_trait<T, float>::TP> gradientori(const Image<T>& input);

//! Compute the magnitude and orientation of the gradient
/*! This is just an efficient combination of gradientmag() and
  gradientori() */
template <class T>
void gradient(const Image<T>& input,
              Image<typename promote_trait<T, float>::TP>& mag,
              Image<typename promote_trait<T, float>::TP>& ori);

//! Compute the magnitude and orientation of the gradient using the sobel op
template <class T>
void gradientSobel(const Image<T>& input,
              Image<typename promote_trait<T, float>::TP>& mag,
              Image<typename promote_trait<T, float>::TP>& ori,
                                  int kernelSize = 3);


//! Compute the non maximal suppression (edge thinning)
Image<float> nonMaxSuppr(const Image<float>& mag, const Image<float>& ori);


//! shuffle the contents of an image using Fisher-Yates shuffle
//Randomly permute N elements by exchanging each element ei with a
//random element from i to N. It consumes ?(N log N) bits and runs
//in linear time. Obtained from
//<http://www.nist.gov/dads/HTML/fisherYatesShuffle.html>

template <class T_or_RGB>
Image<T_or_RGB> shuffleImage(const Image<T_or_RGB> &img);


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !IMAGE_FILTEROPS_H_DEFINED

/*!@file Image/PyramidOps.C Free functions operating on pyramid data structures */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/PyramidOps.C $
// $Id: PyramidOps.C 14474 2011-02-04 01:21:21Z dberg $
//

#include "Image/PyramidOps.H"

#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/ImageSetOps.H"
#include "Image/FilterOps.H"
#include "Image/LowPassLpt.H"
#include "Image/Kernels.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Image/Transforms.H" // for chamfer34()
#include "Image/Pixels.H"
#include "Util/Assert.H"
#include "rutz/trace.h"

// ############################################################
// ##### Dyadic pyramid pixel operations:
// ############################################################

// ######################################################################
template <class T>
T getPyrPixel(const ImageSet<T>& pyr,
              const Point2D<int>& p, const uint lev)
{
  ASSERT(lev < pyr.size());
  ASSERT(pyr[0].coordsOk(p));
  ASSERT(isDyadic(pyr));

  float fac = float(1 << lev);
  float ii = float(p.i) / fac, jj = float(p.j) / fac;
  int ww = pyr[lev].getWidth() - 1, hh = pyr[lev].getHeight() - 1;
  if (ii > ww) ii = ww; if (jj > hh) jj = hh;

  return pyr[lev].getValInterp(ii, jj);
}

// ######################################################################
template <class T>
T getPyrPixelNI(const ImageSet<T>& pyr,
                const Point2D<int>& p, const uint lev)
{
  ASSERT(lev < pyr.size());
  ASSERT(pyr[0].coordsOk(p));
  ASSERT(isDyadic(pyr));

  int ii = p.i >> lev, jj = p.j >> lev;
  int ww = pyr[lev].getWidth() - 1, hh = pyr[lev].getHeight() - 1;
  if (ii > ww) ii = ww; if (jj > hh) jj = hh;

  return pyr[lev].getVal(ii, jj);
}

// ######################################################################
template <class T>
T getPyrPixel(const ImageSet<T>& pyr,
              const float x, const float y, const float z)
{
  ASSERT(isDyadic(pyr));

  const uint nlev = pyr.size();

  // let's do a getValInterp on the two adjacent levels, then a linear
  // interpolation between these two values:
  const uint lev1 = int(floor(z)); ASSERT(lev1 < nlev);

  // we will interpolate between lev and lev+1. Let's compute the
  // scaled down (x, y) coordinates for lev:
  const float fac1 = float(1 << lev1);
  float ii1 = x / fac1, jj1 = y / fac1;
  const float ww1 = float(pyr[lev1].getWidth() - 1);
  const float hh1 = float(pyr[lev1].getHeight() - 1);
  if (ii1 > ww1) ii1 = ww1;
  if (jj1 > hh1) jj1 = hh1;

  const T val1 = pyr[lev1].getValInterp(ii1, jj1);

  // if we are at the top level, then we cannot interpolate in the
  // scale dimension, so we just use that single level:
  if (lev1 == nlev - 1) return val1;

  // otherwise, repeat for level lev+1:
  const uint lev2 = lev1 + 1;
  const float fac2 = float(2 << lev2);
  float ii2 = x / fac2, jj2 = y / fac2;
  float ww2 = float(pyr[lev2].getWidth() - 1);
  float hh2 = float(pyr[lev2].getHeight() - 1);
  if (ii2 > ww2) ii2 = ww2;
  if (jj2 > hh2) jj2 = hh2;

  const T val2 = pyr[lev2].getValInterp(ii2, jj2);

  // now a last linear interpolation between these two values:
  const float fz = z - float(lev1);   // fractional z value

  return T(val1 + (val2 - val1) * fz); // no need to clamp
}

// ######################################################################
// ##### Pyramid builder functions:
// ######################################################################

// ######################################################################
template <class T>
ImageSet<T> buildPyrGaussian(const Image<T>& image,
                             int firstlevel, int depth, int filterSize)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(image.initialized());

  ImageSet<T> result(depth);

  if (0 >= firstlevel)
    result[0] = image;

  Image<T> a = image;
  for (int lev = 1; lev < depth; ++lev)
    {
      if (filterSize == 5)
        {
          a = lowPass5yDecY(lowPass5xDecX(a,2),2);
        }
      else
        {
          a = decX(lowPassX(filterSize, a));
          a = decY(lowPassY(filterSize, a));
        }

      // NOTE: when lev < firstlevel, we leave result[lev] empty even
      // though it isn't any extra work to fill it in; that way, other
      // functions that see the result ImageSet might be able to gain
      // additional speed-ups by seeing that some of the images are
      // empty and avoid processing them further
      if (lev >= firstlevel)
        result[lev] = a;
    }

  return result;
}

// ######################################################################
template <class T>
ImageSet<T> buildRadialPyrGaussian(const Image<T>& image, int firstlevel, int depth)
{
  GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(image.initialized());

  ImageSet<T> result(depth);
  
  if (0 >= firstlevel)
    result[0] = image;
  
  Image<T> a = image;
  for (int lev = 1; lev < depth; ++lev)
    {
      a = decXY(lowPassLpt5(a, CROSS_HEMI));
      
      // NOTE: when lev < firstlevel, we leave result[lev] empty even
      // though it isn't any extra work to fill it in; that way, other
      // functions that see the result ImageSet might be able to gain
      // additional speed-ups by seeing that some of the images are
      // empty and avoid processing them further
      if (lev >= firstlevel)
        result[lev] = a;
    }
  
  return result;
}

// ######################################################################
template <class T>
ImageSet<T> buildPyrConvolve(const Image<T>& image,
                             int firstlevel, int depth,
                             const Image<float>& filter,
                             ConvolutionBoundaryStrategy boundary)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(image.initialized());

  ImageSet<T> result(depth);

  Image<T> base;
  for (int lev = 0; lev < depth; ++lev)
    {
      if (lev == 0) base = image; else base = decXY(lowPass5(base));

      if (lev >= firstlevel)
        result[lev] = convolve(base, filter, boundary);
    }

  return result;
}

// ######################################################################
template <class T>
ImageSet<T> buildPyrLaplacian(const Image<T>& image,
                              int firstlevel, int depth, int filterSize)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(image.initialized());

  ImageSet<T> result(depth);

  Image<T> lpf = lowPass(filterSize, image);

  if (0 >= firstlevel)
    result[0] = image - lpf;

  for (int lev = 1; lev < depth; ++lev)
    {
      const Image<T> dec = decXY(lpf);
      lpf = lowPass(filterSize, dec);

      if (lev >= firstlevel)
        result[lev] = dec - lpf;
    }

  return result;
}

// ######################################################################
template <class T>
ImageSet<T> buildPyrOrientedFromLaplacian(const ImageSet<T>& laplacian,
                                          int filterSize,
                                          float theta, float intens,
                                          const bool usetab)
{
  int attenuation_width = -1;
  float spatial_freq = -1.0;

  switch (filterSize)
    {
    case 5: attenuation_width = 3; spatial_freq = M_PI_2; break;
    case 9: attenuation_width = 5; spatial_freq = 2.6; break;
    default:
      LFATAL("Filter size %d is not supported", filterSize);
    }

  // compute oriented filter from laplacian:
  ImageSet<T> result(laplacian.size());

  for (size_t lev = 0; lev < result.size(); ++lev)
    {
      if (laplacian[lev].initialized())
        {
          result[lev] = orientedFilter(laplacian[lev], spatial_freq,
                                       theta, intens, usetab);
          // attenuate borders that are overestimated due to filter trunctation:
          inplaceAttenuateBorders(result[lev], attenuation_width);
        }
    }

  return result;
}

// ######################################################################
template <class T>
ImageSet<T> buildPyrOriented(const Image<T>& image,
                             int firstlevel, int depth, int filterSize,
                             float theta, float intens,
                             const bool usetab)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(image.initialized());

  // let's promote the intermediary results to float
  typedef typename promote_trait<T, float>::TP TF;

  const ImageSet<TF> laplacian =
    buildPyrLaplacian(Image<TF>(image), firstlevel, depth, filterSize);

  const ImageSet<TF> orifilt =
    buildPyrOrientedFromLaplacian(laplacian, filterSize, theta,
                                  intens, usetab);

  ImageSet<T> result(orifilt.size());
  for (size_t i = 0; i < result.size(); ++i)
    result[i] = Image<T>(orifilt[i]);

  return result;
}

// ######################################################################
template <class T>
ImageSet<T> buildPyrLocalAvg(const Image<T>& image, int depth)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(image.initialized());

  ASSERT(depth >= 1);

  ImageSet<T> result(depth);

  // only deepest level of pyr is filled with local avg:
  const int scale = int(pow(2.0, depth - 1));
  result[depth - 1] = quickLocalAvg(image, scale);

  return result;
}

// ######################################################################
template <class T>
ImageSet<T> buildPyrLocalAvg2x2(const Image<T>& image, int depth)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(depth >= 0);

  if (depth == 0)
    return ImageSet<T>();

  ImageSet<T> result(depth);
  result[0] = image;

  for (int i = 1; i < depth; ++i)
    result[i] = quickLocalAvg2x2(result[i-1]);

  return result;
}

// ######################################################################
template <class T>
ImageSet<T> buildPyrLocalMax(const Image<T>& image, int depth)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(image.initialized());

  ASSERT(depth >= 1);

  ImageSet<T> result(depth);

  // only deepest level of pyr is filled with local max:
  const int scale = int(pow(2.0, depth - 1));
  result[depth - 1] = quickLocalMax(image, scale);

  return result;
}

// ######################################################################
template <class T>
ImageSet<T> buildPyrGeneric(const Image<T>& image,
                            int firstlevel, int depth,
                            const PyramidType typ, const float gabor_theta,
                            const float intens)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  switch(typ)
    {
    case Gaussian3: return buildPyrGaussian(image, firstlevel, depth, 3);
    case Gaussian5: return buildPyrGaussian(image, firstlevel, depth, 5);
    case Gaussian9: return buildPyrGaussian(image, firstlevel, depth, 9);

    case Laplacian5: return buildPyrLaplacian(image, firstlevel, depth, 5);
    case Laplacian9: return buildPyrLaplacian(image, firstlevel, depth, 9);

    case Oriented5: return buildPyrOriented(image, firstlevel, depth, 5, gabor_theta, intens);
    case Oriented9: return buildPyrOriented(image, firstlevel, depth, 9, gabor_theta, intens);

    case QuickLocalAvg: return buildPyrLocalAvg(image, depth);
    case QuickLocalMax: return buildPyrLocalMax(image, depth);

    default:
      LFATAL("Attempt to create Pyramid of unknown type %d", typ);
    }

  ASSERT(false);
  return ImageSet<T>(); // "can't happen", but placate compiler
}

// ######################################################################
ImageSet<float> buildPyrGabor(const ImageSet<float>& gaussianPyr,
                              float angle, float filter_period,
                              float elongation, int size, int flags)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const double major_stddev = filter_period / 3.0;
  const double minor_stddev = major_stddev * elongation;

  // We have to add 90 to the angle here when constructing the gabor
  // filter. That's because the angle used to build the gabor filter
  // actually specifies the direction along which the grating
  // varies. This direction is orthogonal to the the direction of the
  // contours that the grating will detect.
  const double theta = angle + 90.0f;

  // In concept, we want to filter with four phases of the filter: odd
  // on+off, and even on+off (that would be xox, oxo, xo, and ox). But
  // since the on version just produces the negation of the off version,
  // we can get the summed output of both by just taking the absolute
  // value of the outputs of one of them. So we only need to convolve
  // with the filter in two phases, then take the absolute value (i.e.,
  // we're doing |xox| and |xo|).

  Image<float> g0 = gaborFilter3(major_stddev, minor_stddev,
                                 filter_period, 0.0f, theta, size);
  Image<float> g90 = gaborFilter3(major_stddev, minor_stddev,
                                  filter_period, 90.0f, theta, size);

  LDEBUG("angle = %.2f, period = %.2f pix, size = %dx%d pix",
         angle, filter_period, g0.getWidth(), g0.getHeight());

  Image<float> f0, f90;
  ImageSet<float> result(gaussianPyr.size());

  for (uint i = 0; i < gaussianPyr.size(); ++i)
    {
      // if the i'th level in our input is empty, then leave it empty
      // in the output as well:
      if (!gaussianPyr[i].initialized())
        continue;

      if (flags & DO_ENERGY_NORM)
        {
          Image<float> temp = energyNorm(gaussianPyr[i]);
          f0 = optConvolve(temp, g0);
          f90 = optConvolve(temp, g90);
        }
      else
        {
          f0 = optConvolve(gaussianPyr[i], g0);
          f90 = optConvolve(gaussianPyr[i], g90);
        }

      if (!(flags & NO_ABS))
        {
          f0 = abs(f0);
          f90 = abs(f90);
        }

      // this really doesn't do much - take it out for now
      // DW 04/25/03
      //if (flags & DO_CLAMPED_DIFF)
      //{
      //  result[i] =
      //    clampedDiff(f0, gaussianPyr[i]) + clampedDiff(f90, gaussianPyr[i]);
      //}
      //else
      //{
      result[i] = f0 + f90;
      //}
    }

  return result;
}

// ######################################################################
ImageSet<float> buildPyrGabor(const Image<float>& img,
                              int firstlevel, int depth, float angle,
                              float filter_period, float elongation,
                              int size, int flags)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ImageSet<float> pyr;

  //LINFO("flags: %d", flags);
  if (flags & DO_LAPLACIAN)
    {
      pyr = buildPyrLaplacian(img, firstlevel, depth, 9);
    }
  else
    {
      pyr = buildPyrGaussian(img, firstlevel, depth, 9);
    }

  return buildPyrGabor(pyr, angle, filter_period, elongation, size, flags);
}

// ######################################################################
// ##### Processing Functions:
// ######################################################################

// ######################################################################
template <class T>
Image<T> centerSurround(const ImageSet<T>& pyr,
                        const int lev1, const int lev2,
                        const bool absol,
                        const ImageSet<float>* clipPyr)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(lev1 >= 0 && lev2 >= 0);
  ASSERT(uint(lev1) < pyr.size() && uint(lev2) < pyr.size());

  const int largeLev = std::min(lev1, lev2);
  const int smallLev = std::max(lev1, lev2);

  if (clipPyr != 0 && clipPyr->isNonEmpty())
    {
      ASSERT((*clipPyr)[largeLev].getDims() == pyr[largeLev].getDims());
      ASSERT((*clipPyr)[smallLev].getDims() == pyr[smallLev].getDims());

      return
        ::centerSurround(Image<T>(pyr[largeLev] * (*clipPyr)[largeLev]),
                         Image<T>(pyr[smallLev] * (*clipPyr)[smallLev]),
                         absol);
    }
  else
    return ::centerSurround(pyr[largeLev], pyr[smallLev], absol);
}

// ######################################################################
template <class T>
void centerSurround(const ImageSet<T>& pyr,
                    const int lev1, const int lev2,
                    Image<T>& pos, Image<T>& neg,
                    const ImageSet<float>* clipPyr)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(lev1 >= 0 && lev2 >= 0);
  ASSERT(uint(lev1) < pyr.size() && uint(lev2) < pyr.size());

  const int largeLev = std::min(lev1, lev2);
  const int smallLev = std::max(lev1, lev2);

  if (clipPyr != 0 && clipPyr->isNonEmpty())
    {
      ASSERT((*clipPyr)[largeLev].getDims() == pyr[largeLev].getDims());
      ASSERT((*clipPyr)[smallLev].getDims() == pyr[smallLev].getDims());

      ::centerSurround(Image<T>(pyr[largeLev] * (*clipPyr)[largeLev]),
                       Image<T>(pyr[smallLev] * (*clipPyr)[smallLev]),
                       pos, neg);
    }
  else
    ::centerSurround(pyr[largeLev], pyr[smallLev], pos, neg);
}

// ######################################################################
template <class T>
Image<T> centerSurroundSingleOpponent(const ImageSet<T>& cpyr,
                                      const ImageSet<T>& spyr,
                                      const int lev1, const int lev2,
                                      const bool absol,
                                      const ImageSet<float>* clipPyr)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(lev1 >= 0 && lev2 >= 0);
  ASSERT(uint(lev1) < cpyr.size() && uint(lev2) < cpyr.size());
  ASSERT(uint(lev1) < spyr.size() && uint(lev2) < spyr.size());

  const int largeLev = std::min(lev1, lev2);
  const int smallLev = std::max(lev1, lev2);

  if (clipPyr != 0 && clipPyr->isNonEmpty())
    {
      ASSERT((*clipPyr)[largeLev].getDims() == cpyr[largeLev].getDims());
      ASSERT((*clipPyr)[smallLev].getDims() == cpyr[smallLev].getDims());
      ASSERT((*clipPyr)[largeLev].getDims() == spyr[largeLev].getDims());
      ASSERT((*clipPyr)[smallLev].getDims() == spyr[smallLev].getDims());

      return
        ::centerSurround(Image<T>(cpyr[largeLev] * (*clipPyr)[largeLev]),
                         Image<T>(spyr[smallLev] * (*clipPyr)[smallLev]),
                         absol);
    }
  else
    return ::centerSurround(cpyr[largeLev], spyr[smallLev], absol);
}

// ######################################################################
template <class T>
void centerSurroundSingleOpponent(const ImageSet<T>& cpyr,
                                  const ImageSet<T>& spyr,
                                  const int lev1, const int lev2,
                                  Image<T>& pos, Image<T>& neg,
                                  const ImageSet<float>* clipPyr)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(lev1 >= 0 && lev2 >= 0);
  ASSERT(uint(lev1) < cpyr.size() && uint(lev2) < cpyr.size());
  ASSERT(uint(lev1) < spyr.size() && uint(lev2) < spyr.size());

  const int largeLev = std::min(lev1, lev2);
  const int smallLev = std::max(lev1, lev2);

  if (clipPyr != 0 && clipPyr->isNonEmpty())
    {
      ASSERT((*clipPyr)[largeLev].getDims() == cpyr[largeLev].getDims());
      ASSERT((*clipPyr)[smallLev].getDims() == cpyr[smallLev].getDims());
      ASSERT((*clipPyr)[largeLev].getDims() == spyr[largeLev].getDims());
      ASSERT((*clipPyr)[smallLev].getDims() == spyr[smallLev].getDims());

      ::centerSurround(Image<T>(cpyr[largeLev] * (*clipPyr)[largeLev]),
                       Image<T>(spyr[smallLev] * (*clipPyr)[smallLev]),
                       pos, neg);
    }
  else
    ::centerSurround(cpyr[largeLev], spyr[smallLev], pos, neg);
}

// ######################################################################
template <class T>
Image<T> centerSurroundDiff(const ImageSet<T>& pyr1,
                            const ImageSet<T>& pyr2,
                            const int lev1, const int lev2,
                            const bool absol,
                            const ImageSet<float>* clipPyr)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(lev1 >= 0 && lev2 >= 0);
  ASSERT(uint(lev1) < pyr1.size() && uint(lev2) < pyr1.size());

  const int largeLev = std::min(lev1, lev2);
  const int smallLev = std::max(lev1, lev2);

  ASSERT(pyr1[largeLev].getDims() == pyr2[largeLev].getDims());
  ASSERT(pyr1[smallLev].getDims() == pyr2[smallLev].getDims());

  // compute differences between the two pyramids:
  const Image<T> limg = pyr1[largeLev] - pyr2[largeLev];
  const Image<T> simg = pyr1[smallLev] - pyr2[smallLev];

  if (clipPyr != 0 && clipPyr->isNonEmpty())
    {
      ASSERT((*clipPyr)[largeLev].getDims() == limg.getDims());
      ASSERT((*clipPyr)[smallLev].getDims() == simg.getDims());

      return ::centerSurround(Image<T>(limg * (*clipPyr)[largeLev]),
                              Image<T>(simg * (*clipPyr)[smallLev]),
                              absol);
    }
  else
    return ::centerSurround(limg, simg, absol);
}

// ######################################################################
template <class T>
void centerSurroundDiff(const ImageSet<T>& pyr1,
                        const ImageSet<T>& pyr2,
                        const int lev1, const int lev2,
                        Image<T>& pos, Image<T>& neg,
                        const ImageSet<float>* clipPyr)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(lev1 >= 0 && lev2 >= 0);
  ASSERT(uint(lev1) < pyr1.size() && uint(lev2) < pyr1.size());

  const int largeLev = std::min(lev1, lev2);
  const int smallLev = std::max(lev1, lev2);

  ASSERT(pyr1[largeLev].getDims() == pyr2[largeLev].getDims());
  ASSERT(pyr1[smallLev].getDims() == pyr2[smallLev].getDims());

  // compute differences between the two pyramids:
  const Image<T> limg = pyr1[largeLev] - pyr2[largeLev];
  const Image<T> simg = pyr1[smallLev] - pyr2[smallLev];

  if (clipPyr != 0 && clipPyr->isNonEmpty())
    {
      ASSERT((*clipPyr)[largeLev].getDims() == limg.getDims());
      ASSERT((*clipPyr)[smallLev].getDims() == simg.getDims());

      ::centerSurround(Image<T>(limg * (*clipPyr)[largeLev]),
                       Image<T>(simg * (*clipPyr)[smallLev]),
                       pos, neg);
    }
  else
    ::centerSurround(limg, simg, pos, neg);
}

// ######################################################################
template <class T>
Image<T> centerSurroundDiffSingleOpponent(const ImageSet<T>& cpyr1,
                                          const ImageSet<T>& cpyr2,
                                          const ImageSet<T>& spyr1,
                                          const ImageSet<T>& spyr2,
                                          const int lev1, const int lev2,
                                          const bool absol,
                                          const ImageSet<float>* clipPyr)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(lev1 >= 0 && lev2 >= 0);
  ASSERT(uint(lev1) < cpyr1.size() && uint(lev2) < cpyr1.size());

  const int largeLev = std::min(lev1, lev2);
  const int smallLev = std::max(lev1, lev2);

  ASSERT(cpyr1[largeLev].getDims() == cpyr2[largeLev].getDims());
  ASSERT(cpyr1[smallLev].getDims() == cpyr2[smallLev].getDims());
  ASSERT(spyr1[largeLev].getDims() == spyr2[largeLev].getDims());
  ASSERT(spyr1[smallLev].getDims() == spyr2[smallLev].getDims());

  // compute differences between the two pyramids:
  const Image<T> limg = cpyr1[largeLev] - cpyr2[largeLev];
  const Image<T> simg = spyr1[smallLev] - spyr2[smallLev];

  if (clipPyr != 0 && clipPyr->isNonEmpty())
    {
      ASSERT((*clipPyr)[largeLev].getDims() == limg.getDims());
      ASSERT((*clipPyr)[smallLev].getDims() == simg.getDims());

      return ::centerSurround(Image<T>(limg * (*clipPyr)[largeLev]),
                              Image<T>(simg * (*clipPyr)[smallLev]),
                              absol);
    }
  else
    return ::centerSurround(limg, simg, absol);
}

// ######################################################################
template <class T>
void centerSurroundDiffSingleOpponent(const ImageSet<T>& cpyr1,
                                      const ImageSet<T>& cpyr2,
                                      const ImageSet<T>& spyr1,
                                      const ImageSet<T>& spyr2,
                                      const int lev1, const int lev2,
                                      Image<T>& pos, Image<T>& neg,
                                      const ImageSet<float>* clipPyr)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(lev1 >= 0 && lev2 >= 0);
  ASSERT(uint(lev1) < cpyr1.size() && uint(lev2) < cpyr1.size());

  const int largeLev = std::min(lev1, lev2);
  const int smallLev = std::max(lev1, lev2);

  ASSERT(cpyr1[largeLev].getDims() == cpyr2[largeLev].getDims());
  ASSERT(cpyr1[smallLev].getDims() == cpyr2[smallLev].getDims());
  ASSERT(spyr1[largeLev].getDims() == spyr2[largeLev].getDims());
  ASSERT(spyr1[smallLev].getDims() == spyr2[smallLev].getDims());

  // compute differences between the two pyramids:
  const Image<T> limg = cpyr1[largeLev] - cpyr2[largeLev];
  const Image<T> simg = spyr1[smallLev] - spyr2[smallLev];

  if (clipPyr != 0 && clipPyr->isNonEmpty())
    {
      ASSERT((*clipPyr)[largeLev].getDims() == limg.getDims());
      ASSERT((*clipPyr)[smallLev].getDims() == simg.getDims());

      ::centerSurround(Image<T>(limg * (*clipPyr)[largeLev]),
                       Image<T>(simg * (*clipPyr)[smallLev]),
                       pos, neg);
    }
  else
    ::centerSurround(limg, simg, pos, neg);
}

// ######################################################################
template <class T>
Image<T> weightedBlur(const Image<byte>& modulator, const ImageSet<T>& pyr)
{
  // make sure params are correct:
  ASSERT(pyr.size() > 0);
  ASSERT(modulator.getDims() == pyr[0].getDims());

  // the modulator is 0 inside the objects and up to 255
  // outside.  We are now going to use the pyramid
  // to spatially modulate image resolution by picking a
  // pyramid level (with interpolation) according to the
  // modulator value. The code here is similar in spirit to
  // that in retina.C
  const int w = modulator.getWidth(), h = modulator.getHeight();
  Image<T> image(modulator.getDims(), NO_INIT);
  Image<byte>::const_iterator mod = modulator.begin();
  typename Image<T>::iterator ima = image.beginw();
  const int maxdepth = pyr.size() - 1;
  const float rstep = (float)(256 / maxdepth);

  for (int j = 0; j < h; j ++)
    for (int i = 0; i < w; i ++)
      {
        // determine resolution from which this pixel will be taken:
        const float d = (*mod++) / rstep;

        // get the pixel value from depth d, d+1:
        const int di = (int)(floor(d));
        const float dd = d - (float)di;

        const Point2D<int> loc(i, j);
        const T pix0 = getPyrPixel(pyr, loc, di);
        const T pix1 = getPyrPixel(pyr, loc, std::min(di+1, maxdepth));
        // This is fast but may cause clamping problems:
        //*ima++ = pix0 + (pix1 - pix0) * dd;
        // slower but more robust:
        *ima++ = clamped_convert<T>(pix0 * (1.0F - dd) + pix1 * dd);
      }
  return image;
}

// ######################################################################
template <class T>
Image<T> foveate(const Image<byte>& mask, const ImageSet<T>& pyr)
{
  // make sure params are correct:
  ASSERT(mask.getDims() == pyr[0].getDims());

  // normalize the mask:
  Image<float> remask = mask;
  inplaceNormalize(remask, 0.0f, 255.0f);
  inplaceLowThresh(remask, 254.0f, 0.0f);

  // let's now tranform the mask into a distance map:
  const int w = remask.getWidth(), h = remask.getHeight();
  int maxdist = std::max(w, h) * 3;
  float scalefac = float(maxdist) / 255.0f;
  remask = chamfer34(remask, float(maxdist)) / scalefac;
  Image<byte> modulator = remask;  // will clamp as necessary

  // if modulator does not contain any point at zero (inside
  // object), that means that the mask was empty, which is
  // the case at the beginning of a simulation. Set it to some
  // intermediary value to provide a uniform medium blur:
  byte mi, ma; getMinMax(modulator, mi, ma);
  if (mi > 0) modulator /= 3;

  return weightedBlur(modulator, pyr);
}

// Include the explicit instantiations
#include "inst/Image/PyramidOps.I"

template ImageSet<double> buildPyrLocalAvg2x2(const Image<double>&, int);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

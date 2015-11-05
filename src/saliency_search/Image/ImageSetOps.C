/*!@file Image/ImageSetOps.C Free functions operating on sets of images */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/ImageSetOps.C $
// $Id: ImageSetOps.C 14633 2011-03-23 22:55:54Z dberg $
//

#include "Image/ImageSetOps.H"

#include "Image/DrawOps.H"
#include "Image/FilterOps.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/MathOps.H"
#include "Image/Range.H"
#include "Image/ShapeOps.H"
#include "Util/Assert.H"
#include "rutz/compat_cmath.h"

// ######################################################################
// ######################################################################
// ##### ImageSet processing functions
// ######################################################################
// ######################################################################

// ######################################################################
template <class T>
bool isHomogeneous(const ImageSet<T>& x)
{
  if (x.size() == 0) return true;

  const Dims d = x[0].getDims();

  for (uint i = 1; i < x.size(); ++i)
    if (d != x[i].getDims())
      return false;

  return true;
}

// ######################################################################
template <class T>
bool isDyadic(const ImageSet<T>& pyr)
{
  if (pyr.size() == 0) return false;

  for (uint i = 1; i < pyr.size(); ++i)
    {
      const Dims prevdims = pyr[i-1].getDims();
      const Dims curdims = pyr[i].getDims();

      // make sure we don't go below 1
      const int pw2 = std::max(prevdims.w()/2,1);
      const int ph2 = std::max(prevdims.h()/2,1);

      if (curdims.w() != pw2) return false;
      if (curdims.h() != ph2) return false;
    }

  return true;
}

// ######################################################################
template <class T>
Image<T> sum(const ImageSet<T>& x)
{
  ASSERT(isHomogeneous(x));

  Image<T> result(x[0].getDims(), ZEROS);

  for (uint a = 0; a < x.size(); ++a)
    {
      result += x[a];
    }

  return result;
}

// ######################################################################
template <class T>
Image<T> mean(const ImageSet<T>& x)
{
  Image<T> result = sum(x);
  result /= x.size();
  return result;
}

// ######################################################################
template <class T>
Range<T> rangeOf(const ImageSet<T>& x)
{
  Range<T> result;

  for (uint i = 0; i < x.size(); ++i)
    {
      result.merge(rangeOf(x[i]));
    }

  return result;
}

// ######################################################################
template <class T>
ImageSet<T> takeSlice(const ImageSet<T>* sets, uint nsets, uint level)
{
  ImageSet<T> result(nsets);

  for (uint i = 0; i < nsets; ++i)
    {
      result[i] = sets[i][level];
    }

  return result;
}

// ######################################################################
template <class T>
Image<T> makeImageArray(const ImageSet<T>& x,
                        int Nx, int grid_width, T grid_color,
                        int destX, int destY)
{
  if (Nx < 0)
    {
      Nx = int(ceil(sqrt(x.size())));
    }

  Image<T> result;

  // One way or the other, all images must be reshaped to the same size
  // for concatArray(). So, if the user didn't request any specific
  // shaping (by setting destX or destY to -1), then we specify to
  // reshape each image to the size of the largest level.
  if (destX <= 0 || destY <= 0)
    {
      destX = x[0].getWidth();
      destY = x[0].getHeight();
    }

  if (isDyadic(x) && destX == x[0].getWidth() && destY == x[0].getHeight())
    {
      // With a dyadic pyramid, we get more appropriate upscaling by just
      // duplicating pixels.
      ImageSet<T> y = x;
      for (unsigned int i = 1; i < y.size(); ++i)
        y[i] = zoomXY(y[i], 1<<i, 1<<i);

      result = concatArray(&y[0], y.size(), Nx);
    }
  else
    {
      result = concatArray(&x[0], x.size(), Nx, destX, destY);
    }

  if (grid_width > 0)
    drawGrid(result, destX, destY, grid_width, grid_width, grid_color);

  return result;
}

// ######################################################################
template <class T>
ImageSet<T> reduce(const ImageSet<T>& x, int octaves)
{
  ImageSet<T> result(x);

  for (int n = 0; n < octaves; ++n)
    {
      for (uint i = 0; i < x.size(); ++i)
        {
          result[i] = lowPass3(result[i]);
          result[i] = decXY(result[i]);
        }
    }

  return result;
}

// ######################################################################
template <class T>
ImageSet<T> rescale(const ImageSet<T>& x, const Dims& dims)
{
  ImageSet<T> result(x.size());

  for (uint i = 0; i < x.size(); ++i)
    {
      result[i] = rescale(x[i], dims);
    }

  return result;
}

// ######################################################################
ImageSet<float> orientedFilterSet(const Image<float>& lowPassedInput,
                                  float period,
                                  const float* angles, const uint numAngles)
{
  ImageSet<float> result(numAngles);

  LINFO("oriented laplacian period is %f", period);

  const float k = (2.0f * M_PI) / period;

  for (uint i = 0; i < numAngles; ++i)
    result[i] = orientedFilter(lowPassedInput, k, angles[i]);

  return result;
}

// ######################################################################
void splitPosNeg(const ImageSet<float>& x, ImageSet<float>& pos, ImageSet<float>& neg)
{
  pos = ImageSet<float>(x.size());
  neg = ImageSet<float>(x.size());
  for (uint i = 0; i < x.size(); ++i)
    splitPosNeg(x[i], pos[i], neg[i]);
}

// ######################################################################
// ######################################################################
// ##### ImageSet mathematical operators
// ######################################################################
// ######################################################################

// ######################################################################
template <class T>
ImageSet<T>& operator-=(ImageSet<T>& x, const Image<T>& y)
{
  ASSERT(isHomogeneous(x));

  for (uint i = 0; i < x.size(); ++i)
    x[i] -= y;

  return x;
}

// ######################################################################
template <class T>
ImageSet<T>& operator+=(ImageSet<T>& x, const Image<T>& y)
{
  ASSERT(isHomogeneous(x));

  for (uint i = 0; i < x.size(); ++i)
    x[i] += y;

  return x;
}

// ######################################################################
template <class T>
ImageSet<T>& operator*=(ImageSet<T>& x, const Image<T>& y)
{
  ASSERT(isHomogeneous(x));

  for (uint i = 0; i < x.size(); ++i)
    x[i] *= y;

  return x;
}

// ######################################################################
template <class T>
ImageSet<T>& operator/=(ImageSet<T>& x, const Image<T>& y)
{
  ASSERT(isHomogeneous(x));

  for (uint i = 0; i < x.size(); ++i)
    x[i] /= y;

  return x;
}

// ######################################################################
template <class T>
ImageSet<T>& operator-=(ImageSet<T>& x, const T& v)
{
  for (uint i = 0; i < x.size(); ++i)
    x[i] -= v;

  return x;
}

// ######################################################################
template <class T>
ImageSet<T>& operator+=(ImageSet<T>& x, const T& v)
{
  for (uint i = 0; i < x.size(); ++i)
    x[i] += v;

  return x;
}

// ######################################################################
template <class T>
ImageSet<T>& operator*=(ImageSet<T>& x, const T& v)
{
  for (uint i = 0; i < x.size(); ++i)
    x[i] *= v;

  return x;
}

// ######################################################################
template <class T>
ImageSet<T>& operator/=(ImageSet<T>& x, const T& v)
{
  for (uint i = 0; i < x.size(); ++i)
    x[i] /= v;

  return x;
}

// ######################################################################
template <class T>
ImageSet<T> operator-(ImageSet<T>& x, const T& v)
{
  ImageSet<T> res = x;
  return (res -= v);
}

// ######################################################################
template <class T>
ImageSet<T> operator+(ImageSet<T>& x, const T& v)
{
  ImageSet<T> res = x;
  return (res += v);
}

// ######################################################################
template <class T>
ImageSet<T> operator*(ImageSet<T>& x, const T& v)
{
  ImageSet<T> res = x;
  return (res *= v);
}

// ######################################################################
template <class T>
ImageSet<T> operator/(ImageSet<T>& x, const T& v)
{
  ImageSet<T> res = x;
  return (res /= v);
}

// ######################################################################
template <class T>
ImageSet<T>& operator-=(ImageSet<T>& x, const ImageSet<T>& y)
{
  ASSERT(x.size() == y.size());

  for (uint i = 0; i < x.size(); ++i)
    x[i] -= y[i];

  return x;
}

// ######################################################################
template <class T>
ImageSet<T>& operator+=(ImageSet<T>& x, const ImageSet<T>& y)
{
  ASSERT(x.size() == y.size());

  for (uint i = 0; i < x.size(); ++i)
    x[i] += y[i];

  return x;
}

// ######################################################################
template <class T>
ImageSet<T>& operator*=(ImageSet<T>& x, const ImageSet<T>& y)
{
  ASSERT(x.size() == y.size());

  for (uint i = 0; i < x.size(); ++i)
    x[i] *= y[i];

  return x;
}

// ######################################################################
template <class T>
ImageSet<T>& operator/=(ImageSet<T>& x, const ImageSet<T>& y)
{
  ASSERT(x.size() == y.size());

  for (uint i = 0; i < x.size(); ++i)
    x[i] /= y[i];

  return x;
}

// ######################################################################
template <class T>
ImageSet<T> clampedDiff(const ImageSet<T>& b, const ImageSet<T>& c)
{
  ASSERT(b.size() == c.size());

  ImageSet<T> res(b.size());
  for (uint i = 0; i < b.size(); ++i)
    res[i] = clampedDiff(b[i],c[i]);

  return res;
}

// ######################################################################
// ######################################################################
// ##### In-place ImageSet modification functions
// ######################################################################
// ######################################################################

// ######################################################################
template <class T> inline
void doRectify(ImageSet<T>& x)
{
  for (uint i = 0; i < x.size(); ++i)
    inplaceRectify(x[i]);
}

// ######################################################################
template <class T> inline
void doLowThresh(ImageSet<T>& x, const T threshold, const T newval)
{
  for (uint i = 0; i < x.size(); ++i)
    inplaceLowThresh(x[i], threshold, newval);
}

// ######################################################################
template <class T> inline
void doLowThreshAbs(ImageSet<T>& x, const T threshold, const T newval)
{
  for (uint i = 0; i < x.size(); ++i)
    inplaceLowThreshAbs(x[i], threshold, newval);
}

// ######################################################################
template <class T> inline
void doSqrt(ImageSet<T>& x)
{
  for (uint i = 0; i < x.size(); ++i)
    x[i] = sqrt(x[i]);
}

// ######################################################################
template <class T> inline
void doSquared(ImageSet<T>& x)
{
  for (uint i = 0; i < x.size(); ++i)
    x[i] = squared(x[i]);
}

// ######################################################################
void doMeanNormalize(ImageSet<float>& x)
{
  x -= mean(x);
}

// ######################################################################
void doOneNormalize(ImageSet<float>& x)
{
  const Range<float> r = rangeOf(x);

  for (uint i = 0; i < x.size(); ++i)
    {
      x[i] /= r.max();
    }
}

// ######################################################################
void doEnergyNorm(ImageSet<float>& x)
{
  for (uint i = 0; i < x.size(); ++i)
    {
      x[i] = energyNorm(x[i]);
    }
}

// ######################################################################
void doApplyBiases(ImageSet<float>& x, const float* biases)
{
  for (uint i = 0; i < x.size(); ++i)
    {
      x[i] *= biases[i];
    }
}

// ######################################################################
void doAddWeighted(ImageSet<float>& x, const ImageSet<float>& y, float multiple)
{
  ASSERT(x.size() == y.size());
  for (uint a = 0; a < x.size(); ++a)
    x[a] += y[a] * multiple;
}

// ######################################################################
void doClear(ImageSet<float>& x, float v)
{
  for (uint a = 0; a < x.size(); ++a)
    {
      x[a].clear(v);
    }
}


// Include the explicit instantiations
#include "inst/Image/ImageSetOps.I"

template bool isDyadic(const ImageSet<int>& pyr);
template ImageSet<double>& operator/=(ImageSet<double>& x, const double& v);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

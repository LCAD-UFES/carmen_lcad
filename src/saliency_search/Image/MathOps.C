/*!@file Image/MathOps.C Mathematical operations on Image               */
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/MathOps.C $
// $Id: MathOps.C 14665 2011-03-31 23:34:38Z dberg $

#ifndef IMAGE_MATHOPS_C_DEFINED
#define IMAGE_MATHOPS_C_DEFINED

#include "Image/MathOps.H"

// WARNING: Try not include any other "Image*Ops.H" headers here -- if
// you find that you are needing such a header, then the function
// you're writing probably belongs outside Image_MathOps.C.
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/Range.H"
#include "Image/MatrixOps.H"
#include "Image/CutPaste.H"
#include "Image/FourierEngine.H"
#include "Util/Assert.H"
#include "Util/MathFunctions.H"
#include "Util/safecopy.H"
#include "Util/FileUtil.H"
#include "rutz/trace.h"

#include <algorithm>
#include <cmath>
#include <climits>
#include <cfloat>
#include <numeric> // for std::accumulate(), etc.

#if defined(INVT_USE_MMX) || defined(INVT_USE_SSE) || defined(INVT_USE_SSE2)
#include "Util/mmx-sse.H"
#endif

// ######################################################################
template <class T>
double sum(const Image<T>& a)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return std::accumulate(a.begin(), a.end(), 0.0);
}

// ######################################################################
// Optimized implementations using SSE2/SSE for some canonical datatypes:
#ifdef INVT_USE_SSE
double sum(const Image<double> &a)
{
  double d;
  sse_sum((const double *)(a.begin()), &d, a.getSize());
  return d;
}
#endif
#ifdef INVT_USE_SSE2
double sum(const Image<float> &a)
{
  double d;
  sse2_sum((const float *)(a.begin()), &d, a.getSize());
  return d;
}

double sum(const Image<int> &a)
{
  double d;
  sse2_sum((const int *)(a.begin()), &d, a.getSize());
  return d;
}

double sum(const Image<byte> &a)
{
  double d;
  sse2_sum((const byte *)(a.begin()), &d, a.getSize());
  return d;
}
#endif


// ######################################################################
template <class T>
double mean(const Image<T>& a)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(a.getSize() > 0);
  return sum(a) / double(a.getSize());
}

// ######################################################################
template<class T>
double stdev(const Image<T>& a)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(a.getSize() > 1);

  typename Image<T>::const_iterator aptr = a.begin(), stop = a.end();

  double sum = 0.0;
  double avg = mean(a);

  while (aptr != stop)
    {
      double val = ((double)(*aptr++)) - avg;
      sum += val * val;
    }
  return sqrt(sum / ((double)(a.getSize() - 1)));
}

// ######################################################################
template <class T>
Range<T> rangeOf(const Image<T>& img)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return std::for_each(img.begin(), img.end(), Range<T>());
}

// ######################################################################
template <class T>
Image<T> remapRange(const Image<T>& img,
                    const Range<T>& from, const Range<T>& to)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<T> result(img.getDims(), NO_INIT);

  typename Image<T>::const_iterator sptr = img.begin();
  typename Image<T>::iterator dptr = result.beginw(), stop = result.endw();

  double scale = double(to.range()) / double(from.range());

  while (dptr != stop)
    {
      *dptr = to.min() + T((*sptr - from.min()) * scale);
      ++sptr; ++dptr;
    }

  return result;
}

// ######################################################################
template <class T>
Image<T> squared(const Image<T>& a)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<T> result(a.getDims(), NO_INIT);

  /* This is more succinct but unfortunately does not seem to get
     inlined into efficient code:

     std::transform(a.begin(), a.end(), result.beginw(), squareOf<T>);

     so instead we use an explicit loop
  */

  typename Image<T>::const_iterator sptr = a.begin();
  typename Image<T>::iterator dptr = result.beginw();
  typename Image<T>::iterator stop = result.endw();

  while (dptr != stop)
    {
      *dptr++ = (*sptr) * (*sptr);
      ++sptr;
    }
  return result;
}

namespace
{
  template <class T>
  struct ToPow
  {
    double pp;
    ToPow(double p) : pp(p) {}
    T operator()(T x) { return clamped_convert<T>(pow(x, pp)); }
  };

  template <class T>
  struct EpsInv
  {
    double ee;
    EpsInv(T e) : ee(e) {}
    T operator()(T x) {
      if (fabs(x) > ee) return clamped_convert<T>(1.0F / x);
      else return T();
    }
  };
}

// ######################################################################
template <class T>
Image<typename promote_trait<T,float>::TP>
toPower(const Image<T>& a, double pow)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<typename promote_trait<T,float>::TP> result(a.getDims(), NO_INIT);
  std::transform(a.begin(), a.end(), result.beginw(), ToPow<T>(pow));
  return result;
}

// ######################################################################
template <class T>
Image<typename promote_trait<T,float>::TP>
toPowerRegion(const Image<T>& a, double pwr, std::vector<Point2D<int> > region)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  Image<typename promote_trait<T,float>::TP> result(a.getDims(), NO_INIT);
  result = a;
  Point2D<int> temp;
  typename std::vector<Point2D<int> >::iterator vptr = region.begin();
  while(vptr!=region.end())
  {
      temp = *vptr++;
      result.setVal(temp,pow(result.getVal(temp),pwr));
  }

  return result;
}

// ######################################################################
namespace
{
  template <class T> inline T abs_helper(const T& x)
  { return (x > 0) ? x : -x; }
}

template <class T>
Image<T> abs(const Image<T>& a)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<T> result(a.getDims(), NO_INIT);
  std::transform(a.begin(), a.end(), result.beginw(), abs_helper<T>);
  return result;
}

// ######################################################################
template <class T>
Image<typename promote_trait<T,float>::TP>
hmaxActivation(const Image<T>& a, const float sigma)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  typedef typename promote_trait<T, float>::TP TF;
  Image<TF> result(a.getDims(), NO_INIT);
  float sq = 0.5F / (sigma * sigma);
  int siz = a.getSize();
  typename Image<T>::const_iterator sptr = a.begin();
  typename Image<TF>::iterator dptr = result.beginw();

  for (int i = 0; i < siz; i ++)
    *dptr++ = clamped_convert<TF>(exp((*sptr++) * sq));

  return result;
}

// ######################################################################
namespace
{
  template <class T>
  inline T scalarAbsDiff(const T& t1,
                         const T& t2)
  {
    if (t1 < t2) return T(t2 - t1);
    else return T(t1 - t2);
  }

  template <class T>
  inline PixRGB<T> scalarAbsDiff(const PixRGB<T>& t1,
                                 const PixRGB<T>& t2)
  {
    return PixRGB<T>(scalarAbsDiff(t1.p[0], t2.p[0]),
                     scalarAbsDiff(t1.p[1], t2.p[1]),
                     scalarAbsDiff(t1.p[2], t2.p[2]));
  }
}

template <class T>
Image<T> absDiff(const Image<T>& b, const Image<T>& c)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(c.isSameSize(b));

  Image<T> result(b.getDims(), NO_INIT);

  typename Image<T>::const_iterator bptr = b.begin();
  typename Image<T>::const_iterator cptr = c.begin();
  typename Image<T>::iterator dptr = result.beginw();
  typename Image<T>::iterator stop = result.endw();

  while (dptr != stop)
    *dptr++ = scalarAbsDiff(*bptr++, *cptr++);

  return result;
}

// ######################################################################
// Optimized absDiff implementations using SSE2 for some canonical datatypes:
#ifdef INVT_USE_SSE2
Image<byte> absDiff(const Image<byte>& b, const Image<byte> &c)
{
  Image<byte> result(b.getDims(), NO_INIT);
  sse2_absDiff((const byte *)(b.begin()),(const byte *)(c.begin()),
              (byte *)(result.beginw()), b.getSize());
  return result;
}

Image<float> absDiff(const Image<float>& b, const Image<float> &c)
{
  Image<float> result(b.getDims(), NO_INIT);
  sse2_absDiff((const float *)(b.begin()),(float *)(c.begin()),(float *)
              (result.beginw()), b.getSize());
  return result;
}

Image<int> absDiff(const Image<int>& b, const Image<int> &c)
{
  Image<int> result(b.getDims(), NO_INIT);
  sse2_absDiff((const int *)(b.begin()),(const int *)(c.begin()),
              (int *)(result.beginw()), b.getSize());
  return result;
}
#endif
#ifdef INVT_USE_SSE
Image<double> absDiff(const Image<double>& b, const Image<double> &c)
{
  Image<double> result(b.getDims(), NO_INIT);
  sse_absDiff((const double *)(b.begin()),(double *)(c.begin()),
              (double *)(result.beginw()), b.getSize());
  return result;
}
#endif


// ######################################################################
namespace
{
  template <class T>
  inline T scalarClampedDiff(const T& t1,
                             const T& t2)
  {
    if (t1 < t2) return T(0);
    else return T(t1 - t2);
  }

  template <class T>
  inline PixRGB<T> scalarClampedDiff(const PixRGB<T>& t1,
                                     const PixRGB<T>& t2)
  {
    return PixRGB<T>(scalarClampedDiff(t1.p[0], t2.p[0]),
                     scalarClampedDiff(t1.p[1], t2.p[1]),
                     scalarClampedDiff(t1.p[2], t2.p[2]));
  }
}

template <class T>
Image<T> clampedDiff(const Image<T>& b, const Image<T>& c)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(c.isSameSize(b));

  Image<T> result(b.getDims(), NO_INIT);

  typename Image<T>::const_iterator bptr = b.begin();
  typename Image<T>::const_iterator cptr = c.begin();
  typename Image<T>::iterator dptr = result.beginw();
  typename Image<T>::iterator stop = result.endw();

  while (dptr != stop)
    *dptr++ = scalarClampedDiff(*bptr++, *cptr++);

  return result;
}

#ifdef INVT_USE_SSE

// ######################################################################

Image<float> clampedDiff(const Image<float>& b, const Image<float>& c)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(c.isSameSize(b));

  Image<float> result(b.getDims(), NO_INIT);

  const float *bptr= b.begin();
  const float *cptr= c.begin();

  sse_clampedDiff(bptr, cptr,
                  (float *) (result.beginw()), b.getSize());

  return result;
}


// ######################################################################

Image<byte> clampedDiff(const Image<byte>& b, const Image<byte>& c)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(c.isSameSize(b));

  Image<byte> result(b.getDims(), NO_INIT);

  const byte *aptr = (const_cast <byte *> (b.begin()));
  const byte *bptr = (const_cast <byte *> (c.begin()));
  byte *dptr= (const_cast <byte *> (result.beginw()));

  sse_clampedDiff( aptr, bptr, dptr, b.getSize());

  return result;
}

// ######################################################################

Image<int> clampedDiff(const Image<int>& b, const Image<int>& c)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(c.isSameSize(b));

  Image<int> result(b.getDims(), NO_INIT);

  const int *bptr = (const_cast <int *> (b.begin()));
  const int *cptr = (const_cast <int *> (c.begin()));
  int *dptr = (const_cast <int *> (result.beginw()));

  sse_clampedDiff(bptr, cptr, dptr, b.getSize());

  return result;
}


#endif

// ######################################################################
template <class T>
Image<T> binaryReverse(const Image<T>& a, const T val)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<T> result(a.getDims(), NO_INIT);

  typename Image<T>::const_iterator aptr = a.begin();
  typename Image<T>::iterator dptr = result.beginw();
  typename Image<T>::iterator stop = result.endw();
  while (dptr != stop)
    {
      *dptr++ = val - *aptr++;
    }

  return result;
}

// ######################################################################
template <class T>
Image<T> average(const Image<T>& b, const Image<T>& c)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(b.isSameSize(c));
  Image<T> result(b.getDims(), NO_INIT);
  typename Image<T>::iterator aptr = result.beginw();
  typename Image<T>::iterator stop = result.endw();
  typename Image<T>::const_iterator bptr = b.begin();
  typename Image<T>::const_iterator cptr = c.begin();

  while (aptr != stop)
    *aptr++ = (T)( (*bptr++ * 0.5F) + (*cptr++ * 0.5F) );

  return result;
}

// ######################################################################

template <class T>
Image<T> averageWeighted(Image<T>& a, const Image<T>& b, T *aWeight)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(a.isSameSize(b));
  typename Image<T>::iterator aptr;
  typename Image<T>::const_iterator bptr = b.begin();
  for(aptr = a.beginw();  aptr != a.endw(); ++aptr, ++bptr)
    *aptr = (((*aptr)*(*aWeight))+(*bptr))/(1+(*aWeight));
  return a;
}

namespace
{
  template <class T>
  T element_wise_max(const T& t1, const T& t2)
  {
    return std::max(t1, t2);
  }

  template <class T>
  PixRGB<T> element_wise_max(const PixRGB<T>& t1,
                             const PixRGB<T>& t2)
  {
    return PixRGB<T>(std::max(t1.red(), t2.red()),
                     std::max(t1.green(), t2.green()),
                     std::max(t1.blue(), t2.blue()));
  }
}

// ######################################################################
template <class T>
Image<T> takeMax(const Image<T>& a, const Image<T>& b)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(a.isSameSize(b));

  Image<T> result(a.getDims(), NO_INIT);

  typename Image<T>::const_iterator aptr = a.begin();
  typename Image<T>::const_iterator bptr = b.begin();
  typename Image<T>::iterator dptr = result.beginw();
  typename Image<T>::iterator stop = result.endw();

  while (dptr != stop)
    {
      *dptr++ = element_wise_max(*aptr++, *bptr++);
    }

  return result;
}


// ######################################################################
template <class T> 
void takeLinkedMax(const Image<T>& a1, const Image<T>& a2, const Image<T>& b1, const Image<T>& b2, Image<T>& aout, Image<T>& bout)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(a1.isSameSize(a2));
  ASSERT(b1.isSameSize(b2));
  ASSERT(a1.isSameSize(aout));
  ASSERT(aout.isSameSize(bout));
  ASSERT(a1.isSameSize(b1));

  typename Image<T>::const_iterator a1ptr = a1.begin();
  typename Image<T>::const_iterator a2ptr = a2.begin();
  typename Image<T>::const_iterator b1ptr = b1.begin();
  typename Image<T>::const_iterator b2ptr = b2.begin();
  typename Image<T>::iterator aoutptr = aout.beginw(), stop = aout.endw();
  typename Image<T>::iterator boutptr = bout.beginw();

  while (aoutptr != stop)
    {
      if(*a1ptr > *a2ptr)
        {
          *aoutptr=*a1ptr;
          *boutptr=*b1ptr;
        }
      else
        {
          *aoutptr=*a2ptr;
          *boutptr=*b2ptr;
        }
      a1ptr++; a2ptr++;
      b1ptr++; b2ptr++;
      aoutptr++; boutptr++;
    }

}



// ######################################################################
template <class T>
Image<T> takeMin(const Image<T>& a, const Image<T>& b)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(a.isSameSize(b));

  Image<T> result(a.getDims(), NO_INIT);

  typename Image<T>::const_iterator aptr = a.begin();
  typename Image<T>::const_iterator bptr = b.begin();
  typename Image<T>::iterator dptr = result.beginw();
  typename Image<T>::iterator stop = result.endw();

  while (dptr != stop)
    {
      *dptr++ = std::min(*aptr++, *bptr++);
    }

  return result;
}

// ######################################################################
template <class T>
Image<T> quadEnergy(const Image<T>& img1, const Image<T>& img2)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(img1.isSameSize(img2));

  Image<T> result(img1.getDims(), NO_INIT);

  typename Image<T>::const_iterator s1ptr = img1.begin();
  typename Image<T>::const_iterator s2ptr = img2.begin();
  typename Image<T>::iterator dptr = result.beginw();
  typename Image<T>::iterator stop = result.endw();
  typedef typename promote_trait<T, float>::TP TF;
  while (dptr != stop)
    {
      const TF s1( *s1ptr++ );
      const TF s2( *s2ptr++ );
      *dptr++ = T(sqrt(s1 * s1 + s2 * s2));
    }

  return result;
}

// ######################################################################
template <class T>
double RMSerr(const Image<T>& arr1, const Image<T>& arr2)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(arr1.initialized() && arr2.initialized());
  ASSERT(arr1.isSameSize(arr2));

  typename Image<T>::const_iterator ap1 = arr1.begin();
  typename Image<T>::const_iterator ap2 = arr2.begin();
  typename Image<T>::const_iterator stop2 = arr2.end();

  double err = 0.0f;
  while (ap2 != stop2)
    { double v = *ap1++ - *ap2++; err += v * v;  }
  return sqrt(err / double(arr1.getSize()));
}

// ######################################################################
template <class T>
Image<T> overlay(const Image<T>& top, const Image<T>& bottom,
                 const float trans)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // 0 to 100 percent only:
  ASSERT((trans >= 0.0F) && (trans <= 100.0F));
  // images must be same size:
  ASSERT(top.isSameSize(bottom));

  // result is the same size as the input images:
  Image<T> result(top.getDims(), NO_INIT);

  typename Image<T>::const_iterator tptr = top.begin();
  typename Image<T>::const_iterator bptr = bottom.begin();
  typename Image<T>::iterator dptr = result.beginw();
  typename Image<T>::iterator stop = result.endw();
  float tfac = trans * 0.01F;
  while (dptr != stop)
    {
      *dptr = T((*tptr) - ( (*tptr) - (*bptr) ) * tfac);
      ++dptr; ++bptr; ++tptr;
    }

  return result;
}

// ######################################################################
template <class T>
float sumXY(const Image<T>& img, std::vector<float>& sumX,
            std::vector<float>& sumY)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  int w = img.getWidth(), h = img.getHeight();
  sumX.resize(w,0.0F);
  sumY.resize(h,0.0F);
  std::vector<float>::iterator sX, sY = sumY.begin();
  float sum = 0.0F;
  typename Image<T>::const_iterator ptr = img.begin();
  for (int y = 0; y < h; y++)
    {
      sX = sumX.begin();
      for (int x = 0; x < w; x++)
        {
          //sumX[x] += (float)*ptr;
          //sumY[y] += (float)*ptr;
          *sX += (float)*ptr;
          *sY += (float)*ptr;
          sum += (float)*ptr++;
          ++sX;
        }
      ++sY;
    }

  return sum;
}

// ######################################################################
template <class T>
double distance(const Image<T> &img1, const Image<T> &img2)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(img1.isSameSize(img2));
  double d = 0.0;
  typename Image<T>::const_iterator ptr1 = img1.begin(),
    ptr2 = img2.begin(), e = img1.end();
  while(ptr1 != e) {
    d += (*ptr1 - *ptr2) * (*ptr1 - *ptr2);
    ptr1 ++; ptr2 ++;
  }
  return sqrt(d);
}

// ######################################################################
template <class T>
double distance(const Image<T> &img1, const Image<T> &img2,
                const Image<float> &weight)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(img1.isSameSize(img2) && img1.isSameSize(weight));

  double d = 0.0;
  typename Image<T>::const_iterator ptr1 = img1.begin(),
    ptr2 = img2.begin(), e = img1.end();
  Image<float>::const_iterator w = weight.begin();

  while(ptr1 != e) {
    d += *w * (*ptr1 - *ptr2) * (*ptr1 - *ptr2);
    ptr1 ++; ptr2 ++; w ++;
  }
  return sqrt(d);
}

// ######################################################################
double corrcoef(const Image<float>& img1, const Image<float>& img2)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(img1.initialized());
  ASSERT(img2.initialized());
  ASSERT(img1.isSameSize(img2));

  Image<float>::const_iterator p1 = img1.begin();
  Image<float>::const_iterator p2 = img2.begin();
  const Image<float>::const_iterator stop1 = img1.end();

  double sum1 = 0.0;
  double sum2 = 0.0;

  double sumsq1 = 0.0;
  double sumsq2 = 0.0;

  double sumprod = 0.0;

  while (p1 != stop1)
    {
      sum1    += (*p1);
      sum2    += (*p2);
      sumsq1  += double(*p1) * double(*p1);
      sumsq2  += double(*p2) * double(*p2);
      sumprod += double(*p1) * double(*p2);

      ++p1;
      ++p2;
    }

  const int n = img1.getSize();

  const double avg1 = sum1 / n;
  const double avg2 = sum2 / n;

  const double rsq =
    ((sumsq1 - n*avg1*avg1) != 0.0 && (sumsq2 - n*avg2*avg2) != 0.0)
    ? (squareOf(sumprod - n*avg1*avg2)
       /
       ( (sumsq1 - n*avg1*avg1) * (sumsq2 - n*avg2*avg2) ))
    : 0.0;

  ASSERT(rsq >= 0.0);

  // can't just do ASSERT(rsq<=1.0) because that's susceptible to
  // floating-point errors where rsq ends up slightly greater than 1.0
  ASSERT(rsq-1.0 < 1.0e-5);

  return rsq;
}

// ######################################################################
template <class T>
double corrpatch(const Image<T>& img1, const Point2D<int>& topleft1,
                 const Dims& patchdims, const Image<T>& img2,
                 const Point2D<int>& topleft2, const double eps)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (patchdims.sz() == 0) return 0.0;
  ASSERT(img1.rectangleOk(Rectangle(topleft1, patchdims)));
  ASSERT(img2.rectangleOk(Rectangle(topleft2, patchdims)));

  const int pw = patchdims.w(), ph = patchdims.h();
  const int i1w = img1.getWidth(), i2w = img2.getWidth();

  typename Image<T>::const_iterator p1 = img1.begin() + topleft1.j * i1w + topleft1.i;
  typename Image<T>::const_iterator p2 = img2.begin() + topleft2.j * i2w + topleft2.i;
  const int stride1 = i1w - pw, stride2 = i2w - pw;

  double sum1 = 0.0, sum2 = 0.0, sumsq1 = 0.0, sumsq2 = 0.0, sumprod = 0.0;

  for (int j = 0; j < ph; j ++)
    {
      for (int i = 0; i < pw; i ++)
        {
          const double val1 = double(*p1++), val2 = (*p2++);

          sum1    += val1;
          sum2    += val2;
          sumsq1  += val1 * val1;
          sumsq2  += val2 * val2;
          sumprod += val1 * val2;
        }

      // switch to next row in inputs:
      p1 += stride1; p2 += stride2;
    }

  const double n = double(pw * ph);
  const double avg1 = sum1 / n, avg2 = sum2 / n;

  const double denom1 = sumsq1 - n * avg1 * avg1;
  const double denom2 = sumsq2 - n * avg2 * avg2;

  if (fabs(denom1) < eps || fabs(denom2) < eps)
    {
      if (fabs(denom1) < eps && fabs(denom2) < eps) return 1.0;
      return 0.0;
    }

  return (sumprod - n * avg1 * avg2) / sqrt(denom1 * denom2);
}

// ######################################################################
template <class T>
void corrEigenMatrix(const std::vector<std::vector<Image<T> > > &baseImages,
                     Image<T> &baseCorr, Image<T> &baseMean,
                     Image<T> &baseSTD,  Image<T> &baseSS,
                     uint &baseN, bool returnR = false)
{


  const ushort baseDim = baseImages[0].size();

  baseMean.resize(baseDim,1);
  baseSS.resize(baseDim,1);
  baseSTD.resize(baseDim,1);
  baseCorr.resize(baseDim,baseDim);

  for(ushort i = 0; i < baseDim; i++)
  {
    baseMean.setVal(i,0,0.0);
    baseSS.setVal(i,0,0.0);
    baseSTD.setVal(i,0,0.0);
    for(ushort j = 0; j < baseDim; j++)
    {
      baseCorr.setVal(i,j,0.0);
    }
  }

  baseN = 0;

  typename std::vector<std::vector<Image<T> > >::const_iterator ibaseImages
    = baseImages.begin();

  // compute mean and standard deviation over each feature type

  LINFO("Getting base stats");

  // Run over each FRAME
  while(ibaseImages != baseImages.end())
  {
    typename std::vector<Image<T> >::const_iterator iibaseImages
      = ibaseImages->begin();
    typename Image<T>::iterator ibaseMean = baseMean.beginw();
    typename Image<T>::iterator ibaseSS   = baseSS.beginw();
    typename Image<T>::iterator ibaseSTD  = baseSTD.beginw();

    baseN += iibaseImages->getHeight() * iibaseImages->getWidth();

    // Run over each FEATURE
    while(iibaseImages != ibaseImages->end())
    {
      typename Image<T>::const_iterator iiibaseImages = iibaseImages->begin();

      // Run over each PIXEL
      while(iiibaseImages != iibaseImages->end())
      {
        *ibaseSS   += static_cast<T>(pow(*iiibaseImages,2));
        *ibaseMean += *iiibaseImages;
        ++iiibaseImages;
      }
      ++iibaseImages;
      ++ibaseMean; ++ibaseSS; ++ibaseSTD;
    }
    ++ibaseImages;
  }

  LINFO("Getting final base stats");

  typename Image<T>::iterator ibaseMean = baseMean.beginw();
  typename Image<T>::iterator ibaseSS   = baseSS.beginw();
  typename Image<T>::iterator ibaseSTD  = baseSTD.beginw();
  // Run over each FEATURE
  while(ibaseMean != baseMean.endw())
  {
    if((baseN > 0) && (*ibaseSS > 0))
    {
      // mean
      *ibaseMean = *ibaseMean/baseN;
      // stddev
      *ibaseSTD  = static_cast<T>(sqrt((*ibaseSS/baseN) - pow(*ibaseMean,2)));
    }
    else
    {
      *ibaseMean = 0;
      *ibaseSTD  = 0;
    }
    ++ibaseMean; ++baseN; ++ibaseSS; ++ibaseSTD;
  }

  ibaseImages = baseImages.begin();



  LINFO("computing product covariance");
  // compute product covariance
  // Run over each FRAME
  while(ibaseImages != baseImages.end())
  {
    typename std::vector<Image<T> >::const_iterator iibaseImages
      = ibaseImages->begin();
    // Run over each FEATURE
    for(ushort i = 0; i < baseDim; i++, ++iibaseImages)
    {
      // Run over each PIXEL
      for(int x = 0; x < iibaseImages->getWidth(); x++)
      {
        for(int y = 0; y < iibaseImages->getHeight(); y++)
        {
          const T base = iibaseImages->getVal(x,y);
          typename std::vector<Image<T> >::const_iterator iicbaseImages
            = ibaseImages->begin();
          // Run over each OTHER FEATURE
          for(ushort j = 0; j < baseDim; j++, ++iicbaseImages)
          {
            // compute product sum over each X*Y
            T corrVal = baseCorr.getVal(i,j);
            corrVal += base * iicbaseImages->getVal(x,y);
            baseCorr.setVal(i,j,corrVal);
          }
        }
      }
    }
    ++ibaseImages;
  }

  LINFO("Computing final covariance");

  // compute final covaraince
  if(returnR)
  {
    // Normalized true pearson R correlation
    for(uint i = 0; i < baseDim; i++)
    {
      for(uint j = 0; j < baseDim; j++)
      {
        // compute (sum(X*Y)/N - Xbar*Ybar) / Xstd*Ystd
        T val = baseCorr.getVal(i,j);
        val = ((val/baseN) - (baseMean[i] * baseMean[j])) /
               (baseSTD[i] * baseSTD[j]);
        baseCorr.setVal(i,j,val);
      }
    }
  }
  else
  {
    // Un-normalized eigen matrix for Likelyhood est
    // Used in Mahalanobis distance computation as well
    for(uint i = 0; i < baseDim; i++)
    {
      for(uint j = 0; j < baseDim; j++)
      {
        T val = baseCorr.getVal(i,j);
        // compute sum(X*Y)/N - Xbar*Ybar
        val = (val/baseN) - (baseMean[i] * baseMean[j]);
        baseCorr.setVal(i,j,val);
      }
    }
  }
}

// ######################################################################
template <class T>
void getLikelyhoodImage(const std::vector<Image<T> > &baseImages,
                        const Image<T> &baseCorr, const Image<T> &baseMean,
                        const bool returnLogLikelyhood,
                        Image<T> &likelyhoodImage,
                        Image<T> &nonNormalizedLImage)
{
  // matrix must be square
  if(baseCorr.getWidth() != baseCorr.getHeight())
    LFATAL("ERROR matrix is not square %d x %d",
           baseCorr.getWidth(),baseCorr.getHeight());

  // hold the matrix dims
  // const T dims           = baseCorr.getWidth();
  // first compute the inverse eigen matrix
  const Image<T> invCorr = matrixInv(baseCorr);
  // compute the determinant of the eigen matrix
  // In this case, it is just the product of the eigen values
  // WE take the log sum rather than product to keep the number sane

  //const T detCorr        = static_cast<T>(sqrt(matrixDet(baseCorr)));
  //T logDetCorr = static_cast<T>(log(sqrt((2*M_PI)/baseCorr.getVal(0,0))));
  //T detCorr = static_cast<T>(1/(pow((2*M_PI),(dims/2))));
  T logDetCorr = 0;
  for(ushort i = 0; i < (ushort)baseCorr.getWidth(); i++)
  {
    //LINFO("** baseCorr %f",baseCorr.getVal(i,i));
    logDetCorr += static_cast<T>(log(1.0/sqrt((2.0*M_PI)*baseCorr.getVal(i,i))));
    //LINFO("** logDetCorr %f",logDetCorr);
  }

  //LINFO("logDetCorr %f",logDetCorr);
  // compute the easy left hand side of the guassian
  const T LHS = static_cast<T>(logDetCorr);
  //LINFO("LHS computed as %f",LHS);


  likelyhoodImage.resize(baseImages[0].getWidth(),
                         baseImages[0].getHeight());

  nonNormalizedLImage.resize(baseImages[0].getWidth(),
                             baseImages[0].getHeight());

  // create column difference x - mu matrix
  Image<T> diffMatrix;
  diffMatrix.resize(1,baseCorr.getWidth());

  for(uint i = 0; i < (uint)baseImages[0].getWidth(); i++)
  {
    for(uint j = 0; j < (uint)baseImages[0].getHeight(); j++)
    {

      typename Image<T>::iterator                     idiffMatrix =
        diffMatrix.beginw();
      typename Image<T>::const_iterator               ibaseMean   =
        baseMean.begin();
      typename std::vector<Image<T> >::const_iterator ibaseImages =
        baseImages.begin();

      while(ibaseImages != baseImages.end())
      {
        // for each feature, compute x - mu
        *idiffMatrix = ibaseImages->getVal(i,j) - *ibaseMean;
        ++ibaseImages; ++ibaseMean; ++idiffMatrix;
      }
      // compute Mahalanobis distance (x - mu)*iSIG*(x - mu)
      const T RHS = static_cast<T>(sum(matrixMult(
                    clamped_convert<Image<T> >
                    (vmMult(transpose(diffMatrix),invCorr))
                                   ,diffMatrix)));
      //LINFO("RHS computed as %f",RHS);
      const T newRHS = static_cast<T>((-1.0/2.0) * RHS);
      //LINFO("new RHS %f",newRHS);
      //LINFO("LHS %f",LHS);

      // compute final p(x)
      if(returnLogLikelyhood)
      {
        nonNormalizedLImage.setVal(i,j,newRHS);
        const T Px  = static_cast<T>(LHS + newRHS);
        likelyhoodImage.setVal(i,j,Px);
      }
      else
      {
        nonNormalizedLImage.setVal(i,j,exp(newRHS));
        const T Px  = static_cast<T>(exp(LHS + newRHS));
        likelyhoodImage.setVal(i,j,Px);
      }
    }
  }
}

// ######################################################################
template <class T>
Image<T> getNormalizedBayesImage(const Image<T> classImage1,
                                 const Image<T> classImage2,
                                 const bool usingLogLikelyhood,
                                 const T beta,
                                 const T classPrior1,
                                 const T classPrior2,
                                 const T bias)
{
  if(classImage1.getHeight() != classImage2.getHeight())
    LFATAL("classImage1 height %d != classImage2 height %d",
           classImage1.getHeight(),classImage2.getHeight());
  if(classImage1.getWidth() != classImage2.getWidth())
    LFATAL("classImage1 width %d != classImage2 width %d",
           classImage1.getWidth(),classImage2.getWidth());
  Image<T> returnImage;
  returnImage.resize(classImage1.getWidth(),
                     classImage1.getHeight());

  typename Image<T>::iterator       ireturnImage = returnImage.beginw();
  typename Image<T>::const_iterator iclassImage1 = classImage1.begin();
  typename Image<T>::const_iterator iclassImage2 = classImage2.begin();

  if(usingLogLikelyhood)
  {
    while(ireturnImage != returnImage.end())
    {
      const T a     = static_cast<T>(*iclassImage1 - *iclassImage2 +
                                     log(classPrior1/classPrior2));
      *ireturnImage = static_cast<T>((1.0/(1.0 + exp(-1.0*a*beta))) * bias);
      ++ireturnImage; ++iclassImage1; ++iclassImage2;
    }
  }
  else
  {
    while(ireturnImage != returnImage.end())
    {
      const T a     = static_cast<T>(log(((*iclassImage1)*classPrior1)/
                                         (*iclassImage2)*classPrior2));
      *ireturnImage = static_cast<T>((1.0/(1.0 + exp(-1.0*a*beta))) * bias);
      ++ireturnImage; ++iclassImage1; ++iclassImage2;
    }
  }

  return returnImage;
}

// ######################################################################
template <class T>
Image<T> getPearsonRMatrix(const Image<T> &eigenMatrix,
                           const Image<T> &STDMatrix)
{
  if(eigenMatrix.getWidth() != eigenMatrix.getHeight())
    LFATAL("eigenMatrix not square width %d != height %d",
           eigenMatrix.getWidth(),eigenMatrix.getHeight());

  uint baseDim = eigenMatrix.getWidth();

  Image<T> returnImage;
  returnImage.resize(eigenMatrix.getWidth(),eigenMatrix.getHeight());

  // Normalized true pearson R correlation
  for(uint i = 0; i < baseDim; i++)
  {
    for(uint j = 0; j < baseDim; j++)
    {
      // compute (sum(X*Y)/N - Xbar*Ybar) / Xstd*Ystd from
      // the eigen matrix values
      T val = eigenMatrix.getVal(i,j);
      val = val / (STDMatrix[i] * STDMatrix[j]);
      returnImage.setVal(i,j,val);
    }
  }
  return returnImage;
}

// ######################################################################
template <class T>
void getAugmentedBeliefBayesImage(const Image<T> &bayesImage,
                                  const Image<T> &beliefImage1,
                                  const Image<T> &beliefImage2,
                                  const T medianPoint,
                                  Image<T> &beliefImage,
                                  Image<T> &beliefValue)
{
  beliefImage.resize(bayesImage.getWidth(), bayesImage.getHeight());
  beliefValue.resize(bayesImage.getWidth(), bayesImage.getHeight());

  typename Image<T>::const_iterator ibayesImage   = bayesImage.begin();
  typename Image<T>::const_iterator ibeliefImage1 = beliefImage1.begin();
  typename Image<T>::const_iterator ibeliefImage2 = beliefImage2.begin();
  typename Image<T>::iterator       ibeliefImage  = beliefImage.beginw();
  typename Image<T>::iterator       ibeliefValue  = beliefValue.beginw();

  float min = FLT_MAX;
  float max = FLT_MIN;


  while(ibeliefValue != beliefValue.endw())
  {
    *ibeliefValue = static_cast<T>(*ibeliefImage1 + *ibeliefImage2);
    if(*ibeliefValue > max)
      max = static_cast<float>(*ibeliefValue);
    if(*ibeliefValue < min)
      min = static_cast<float>(*ibeliefValue);
    ++ibeliefImage1; ++ibeliefImage2; ++ibeliefValue;
  }

  ibeliefValue  = beliefValue.beginw();

  const T interval = static_cast<T>(max - min);
  const T minT     = static_cast<T>(min);

  while(ibeliefValue != beliefValue.endw())
  {
    *ibeliefValue = static_cast<T>(exp(((*ibeliefValue - minT)/interval)-1));
    ++ibeliefValue;
  }
  ibeliefImage1 = beliefImage1.begin();
  ibeliefImage2 = beliefImage2.begin();
  ibeliefValue  = beliefValue.beginw();

  while(ibayesImage != bayesImage.end())
  {
    const T diff = static_cast<T>(
                   fabs(static_cast<float>(*ibayesImage - medianPoint)));
    const T aug  = diff * *ibeliefValue;

    if(*ibayesImage >= medianPoint)
      *ibeliefImage = medianPoint + aug;
    else
      *ibeliefImage = medianPoint - aug;

    ++ibayesImage; ++ibeliefImage1; ++ibeliefImage2; ++ibeliefImage;
    ++ibeliefValue;
  }
}

// ######################################################################
template <class T>
double pSNR(const Image<T> &img1, const Image<T> &img2)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  double sigma2 = distance(img1, img2);
  sigma2 = sigma2 * sigma2 / double(img1.getSize());
  return 10.0 * log10(255.0*255.0 / sigma2);
}

// ######################################################################
template <class T>
double pSNR(const Image<T> &img1, const Image<T> &img2,
            const Image<float>& weight)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  double sigma2 = distance(img1, img2, weight);
  sigma2 = sigma2 * sigma2 / double(img1.getSize());
  return 10.0 * log10(255.0*255.0 / sigma2);
}

// ######################################################################
template <class T>
Image<typename promote_trait<T,float>::TP> sqrt(const Image<T>& a)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<typename promote_trait<T,float>::TP> result(a.getDims(), NO_INIT);
  typename Image<T>::const_iterator src = a.begin(), stop = a.end();
  typename Image<typename promote_trait<T,float>::TP>::iterator
    dest = result.beginw();
  while(src != stop) *dest++ = sqrt(*src++);
  return result;
}

// ######################################################################
template <class T>
Image<typename promote_trait<T,float>::TP> inverse(const Image<T>& a,
                                                   const T eps)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<typename promote_trait<T,float>::TP> result(a.getDims(), NO_INIT);
  std::transform(a.begin(), a.end(), result.beginw(), EpsInv<T>(eps));
  return result;
}

// ######################################################################
template <class T>
Image<typename promote_trait<T,float>::TP> exp(const Image<T>& a)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<typename promote_trait<T,float>::TP> result(a.getDims(), NO_INIT);
  typename Image<T>::const_iterator src = a.begin(), stop = a.end();
  typename Image<typename promote_trait<T,float>::TP>::iterator
    dest = result.beginw();
  while(src != stop) *dest++ = exp(*src++);
  return result;
}

// ######################################################################
template <class T>
Image<typename promote_trait<T,float>::TP> negexp(const Image<T>& a)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<typename promote_trait<T,float>::TP> result(a.getDims(), NO_INIT);
  typename Image<T>::const_iterator src = a.begin(), stop = a.end();
  typename Image<typename promote_trait<T,float>::TP>::iterator
    dest = result.beginw();
  while(src != stop) *dest++ = exp( - (*src++));
  return result;
}

// ######################################################################
template <class T>
Image<typename promote_trait<T,float>::TP> log(const Image<T>& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  typedef typename promote_trait<T,float>::TP TF;
  Image<TF> result(src.getDims(), NO_INIT);
  typename Image<T>::const_iterator sptr = src.begin();
  typename Image<TF>::iterator dptr = result.beginw();
  typename Image<TF>::iterator stop = result.endw();

  while (dptr != stop)
    *dptr++ = log(*sptr++);

  return result;
}

// ######################################################################
template <class T>
Image<typename promote_trait<T,float>::TP> log10(const Image<T>& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  typedef typename promote_trait<T,float>::TP TF;
  Image<TF> result(src.getDims(), NO_INIT);
  typename Image<T>::const_iterator sptr = src.begin();
  typename Image<TF>::iterator dptr = result.beginw();
  typename Image<TF>::iterator stop = result.endw();

  while (dptr != stop)
    *dptr++ = log10(*sptr++);

  return result;
}

// ######################################################################
template <class T>
bool getCentroidFirstLast(std::vector<T> vect, float& centroid,
                          int& first, int& last)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  double sum = 0.0;
  double mi = 0.0;
  bool before = true;
  const T zero = T();

  typename std::vector<T>::iterator vit = vect.begin();

  for (uint i = 0; i < vect.size(); ++i, ++vit)
    {
      //LINFO("vect[%d] = %g",i,vect[i]);
      mi  += (*vit * i);
      sum += *vit;
      if (before) first = i;
      if (*vit != zero)
        {
          before = false;
          last = i;
        }
    }

  // there was nobody at home?
  if (before)
    {
      centroid = -1.0F; first = -1; last = -1;
      return false;
    }

  if (sum == 0.0)
    LFATAL("The sum of non-zero numbers is zero - don't know how "
           "to compute the center of mass in this case.");

  centroid = mi / sum;
  return true;
}

// ######################################################################
template <class T>
Point2D<int> centroid(const Image<T>& img, Rectangle& boundingBox,
                 float& cenX, float& cenY)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  std::vector<float> sumx, sumy;
  sumXY(img, sumx, sumy);
  int firstX, lastX, firstY, lastY;
  cenX = 0.0F; cenY = 0.0F;

  bool success = getCentroidFirstLast(sumx, cenX, firstX, lastX);
  success |= getCentroidFirstLast(sumy, cenY, firstY, lastY);

  if (!success)
    {
      boundingBox = Rectangle();
      return Point2D<int>(-1,-1);
    }

  boundingBox = Rectangle::tlbrI(firstY, firstX, lastY, lastX);
  return Point2D<int>(int(cenX + 0.5F), int(cenY + 0.5F));
}

// ######################################################################
template <class T>
Point2D<int> centroid(const Image<T>& img)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Rectangle boundingBox;
  float cenX, cenY;
  return centroid(img, boundingBox, cenX, cenY);
}

// ######################################################################
template<class T>
Image<T> squash(const Image<T>& ima,
                const T oldmin, const T newmin,
                const T oldmid, const T newmid,
                const T oldmax, const T newmax)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // if the input is blank, let's return blank:
  if (oldmin == oldmax) return ima;

  // let's compute the remapping coeffs and do some checking that we
  // actually can remap:
  float a = float(oldmin), b = float(oldmid), c = float(oldmax);
  float d = float(newmin), e = float(newmid), f = float(newmax);

  // here are the polynomial coefficients, straight out of
  // Mathematica. They are not optimized for computation speed, but we
  // don't really care here as we only need to compute them once. They
  // all need to be divided by the denominator below:
  float q0 = c*c*(b*(b-c)*(b-c)*(-4.0F*a*a + 3.0F*a*b + 2.0F*a*c - b*c)*d +
                  a*a*(a-c)*(a-c)*(a-c)*e) + a*a*(a-b)*(a-b)*b*
    (a*(b-2.0F*c) + c*(-3.0F*b + 4.0F*c))*f;
  float q1 = 2.0F*a*c*(c*c*c*c*(e-d) - 3.0F*b*b*b*b*(d-f) +
                       4.0F*b*b*b*c*(d-f) + 2.0F*a*a*a*c*(e-f) + a*a*a*a*(f-e)+
                       2.0F*a*(c*c*c*(d-e) + 2.0F*b*b*b*(d-f) +
                               3.0F*b*b*c*(f-d)));
  float q2 = a*a*a*a*a*(e-f) + a*a*a*a*c*(e-f) + 8.0F*a*a*a*c*c*(f-e) +
    (a+c)*(c*c*c*c*(d-e) + 3.0F*b*b*b*b*(d-f) + 4.0F*b*b*b*c*(f-d)) -
    4.0F*a*a*(2.0F*c*c*c*(d-e) + b*b*b*(d-f) + 3.0F*b*c*c*(f-d));
  float q3 = 2.0F*(c*c*c*c*(e-d) + 2.0F*a*a*(b-c)*(b-c)*(d-f) +
                   2.0F*b*b*c*c*(d-f) + 2.0F*a*a*a*c*(e-f) +
                   b*b*b*b*(f-d) + a*a*a*a*(f-e) +
                   2.0F*a*c*(c*c*(d-e) + b*b*(d-f) + 2.0F*b*c*(f-d)));
  float q4 = -3.0F*a*b*b*d + 2.0F*b*b*b*d + 6.0F*a*b*c*d - 3.0F*b*b*c*d -
    3.0F*a*c*c*d + c*c*c*d + a*a*a*e - 3.0F*a*a*c*e + 3.0F*a*c*c*e -
    c*c*c*e - (a-b)*(a-b)*(a + 2.0F*b - 3.0F*c)*f;

  float denom = (a-b)*(a-b)*(a-c)*(a-c)*(a-c)*(b-c)*(b-c);
  if (fabs(denom) < 1.0e-5F)
    { LERROR("Zero denominator - cannot remap - RETURNING INPUT"); return ima;}
  q0 /= denom; q1 /= denom; q2 /= denom; q3 /= denom; q4 /= denom;

  Image<T> result(ima.getDims(), NO_INIT);
  typename Image<T>::const_iterator src = ima.begin(), stop = ima.end();
  typename Image<T>::iterator dest = result.beginw();
  while(src != stop) {
    float val = float(*src++);
    // here we optimize the computation for speed:
    val = q0 + val*(q1 + val*(q2 + val*(q3 + val*q4)));
    *dest++ = clamped_convert<T>(val);
  }
  return result;
}

// ######################################################################
template<class T>
Image<T> squash(const Image<T>& src, const T newmin,
                const T oldmid, const T newmid, const T newmax)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // get oldmin and oldmax from the source image:
  T oldmin, oldmax; getMinMax(src, oldmin, oldmax);

  // call the general squash:
  return squash(src, oldmin, newmin, oldmid, newmid, oldmax, newmax);
}

// ######################################################################
template<class T>
Image<T> squash(const Image<T>& src, const T oldmid, const T newmid)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // get oldmin and oldmax from the source image:
  T oldmin, oldmax; getMinMax(src, oldmin, oldmax);

  // call the general squash:
  return squash(src, oldmin, oldmin, oldmid, newmid, oldmax, oldmax);
}

// ######################################################################
template <class T, class TT>
Image<TT> thresholdedMix(const Image<T>& mask, const T& thresh,
                         const Image<TT>& lower, const Image<TT>& higher)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(mask.isSameSize(lower) && mask.isSameSize(higher));

  Image<TT> result(mask.getDims(), NO_INIT);
  typename Image<T>::const_iterator msrc = mask.begin(), mstop = mask.end();
  typename Image<TT>::const_iterator
    lsrc = lower.begin(), hsrc = higher.begin();
  typename Image<TT>::iterator dest = result.beginw();

  while(msrc != mstop) {
    if (*msrc >= thresh) *dest = *hsrc; else *dest = *lsrc;
    msrc ++; lsrc++; hsrc++; dest ++;
  }

  return result;
}

// ######################################################################
Image<float> logSig(const Image<float>& ima, float o, float b)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<float> result(ima.getDims(), NO_INIT);
  Image<float>::const_iterator ptr = ima.begin();
  Image<float>::iterator res = result.beginw();
  Image<float>::iterator stop = result.endw();
  for ( ; res != stop; ++ptr, ++res)
    *res = 1.0f / (1.0f + expf(b*(o-*ptr)));
  return result;
}

// ######################################################################
template <class T>
Image<T> scramble(const Image<T>& ima)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const int siz = ima.getSize(); ASSERT(siz > 0);
  Image<T> result = ima; typename Image<T>::iterator a = result.beginw();

  // this is very similar to randShuffle() in saliency.C:
  for (int i = 0; i < siz; i ++)
    {
      T tmp = a[i];
      int idx = i + randomUpToNotIncluding(siz - i);
      a[i] = a[idx]; a[idx] = tmp;
    }

  return result;
}

// ######################################################################
int32 findMonteMap(Image<float>& ima,
                   std::vector<Point2D<int> >* coords,
                   int decimation, float bias = 1.0F)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const unsigned int siz = (unsigned)ima.getSize(); ASSERT(siz > 0);
  std::vector<Point2D<int> >::iterator itr;
  ASSERT(siz <= coords->size());
  Image<float>::iterator a = ima.beginw();

  float minVal     = *a;
  float maxVal     = 0;

  //First find upper and lower bounds on image values
  //apply an exponent bias if needed
  if(bias != 1.0F)
    for(a = ima.beginw(); a != ima.endw(); ++a)
    {
      *a = *a * *a;
      //*a = pow(*a,bias);
    }

  for(a = ima.beginw(); a != ima.endw(); ++a)
  {
    if(*a > maxVal)
      maxVal = *a;
    if(*a < minVal)
      minVal = *a;
  }

  a                 = ima.beginw();
  itr               = coords->begin();
  int32 counter     = 0;
  float salVal      = 0.0F;

  // determin which pixels are selected based upon there
  // relation to min/max values
  for (unsigned int i = 0; i < siz; i ++)
  {
    // higher order bits (better)
    salVal = ((minVal)+((maxVal)*rand()/(RAND_MAX+1.0)));
    // lower order bits (faster)
    //salVal = (*minVal)+(rand() % (int)*maxVal);
    if(*a > salVal)
    {
      *itr = Point2D<int>(i % ima.getWidth(), i/ima.getWidth());
      ++itr;
      counter++;
    }
    ++a;
  }

  return counter;
}

// ######################################################################
int32 makeSparceMap(std::vector<Point2D<int> >* coords, std::vector<Point2D<int>*>* cmap,
                    std::vector<Point2D<int> >* Oldcoords,
                    std::vector<bool>* keep, int inPoints, int outPoints)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(coords->size() >= cmap->size());
  std::vector<Point2D<int>*>::iterator icmap;
  std::vector<Point2D<int> >::iterator icoords;
  std::vector<Point2D<int> >::iterator iOldcoords;
  std::vector<bool>::iterator ikeep;
  int interval = inPoints/(outPoints+1);
  int mod = 0;
  int count = 0;
  icoords = coords->begin();
  iOldcoords = Oldcoords->begin();
  icmap = cmap->begin();
  ikeep = keep->begin();
  for(int i = 0; i < inPoints; i++)
  {
    if(mod == 0)
    {
      if(*ikeep == false)
        *icmap = &*icoords;
      else
        *icmap = &*iOldcoords;
      ++icmap;
      ++ikeep;
      ++iOldcoords;
      count++;
    }
    else
    {
      if(mod == interval)
        mod = -1;
    }
    mod++;
    ++icoords;
  }
  return count;
}

// ######################################################################
int32 makeCoordArray(std::vector<Point2D<int> >* coords, unsigned int initX, unsigned int initY, unsigned int sizeX, unsigned int sizeY)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  coords->resize(initX*initY);
  std::vector<Point2D<int> >::iterator icoords = coords->begin();
  float scaleX = sizeX / initX;
  float scaleY = sizeY / initY;
  for(unsigned int i = 0; i < initX; i++)
  {
    for(unsigned int j = 0; j < initY; j++)
    {
      icoords->i = (int)(i*scaleX);
      icoords->j = (int)(j*scaleY);
    }
  }
  return initX*initY;
}

// ######################################################################
template <class T>
void inplaceAddWeighted(Image<T>& a,
                        const Image<T>& b, const float coeff)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(b.initialized());
  if (a.initialized())   // a += b * coeff
    {
      ASSERT(a.isSameSize(b));
      typename Image<T>::iterator aptr = a.beginw();
      typename Image<T>::const_iterator bptr = b.begin();
      typename Image<T>::const_iterator stop = b.end();

      while (bptr != stop)
        *aptr++ += (T)(*bptr++ * coeff);
    }
  else   // a = b * coeff
    {
      a = Image<T>(b.getDims(), NO_INIT);
      typename Image<T>::iterator aptr = a.beginw();
      typename Image<T>::const_iterator bptr = b.begin();
      typename Image<T>::const_iterator stop = b.end();

      while (bptr != stop)
        *aptr++ = (T)(*bptr++ * coeff);
    }
}

// ######################################################################
template <class T>
void inplaceSquare(Image<T>& a)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  typename Image<T>::iterator aptr = a.beginw();
  typename Image<T>::iterator stop = a.endw();
  while (aptr != stop)
    {
      (*aptr) *= (*aptr);

      // need to keep the increment in a separate statement; if we do
      // "*aptr *= *aptr++", then it's undefined whether the "*="
      // happens before or after the "++" (thanks to g++ 4.0 for
      // catching this with a warning!)
      ++aptr;
    }
}

// ######################################################################
void inplaceReplaceVal(Image<byte>& dest,
                       const byte& from, const byte& to)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<byte>::iterator
    aptr = dest.beginw(),
    endptr = dest.endw();

  while (aptr != endptr)
    {
      if (*aptr == from) *aptr = to;
      ++aptr;
    }
}

// ######################################################################
template <class T>
void inplaceAttenuateBorders(Image<T>& a, int size)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(a.initialized());

  Dims dims = a.getDims();

  if (size * 2 > dims.w()) size = dims.w() / 2;
  if (size * 2 > dims.h()) size = dims.h() / 2;
  if (size < 1) return;  // forget it

  float increment = 1.0 / (float)(size + 1);
  // top lines:
  float coeff = increment;
  typename Image<T>::iterator aptr = a.beginw();
  for (int y = 0; y < size; y ++)
    {
      for (int x = 0; x < dims.w(); x ++)
        {
          *aptr = (T)( (*aptr) * coeff );
          ++aptr;
        }
      coeff += increment;
    }
  // normal lines: start again from beginning to attenuate corners twice:
  aptr = a.beginw();
  for (int y = 0; y < dims.h(); y ++)
    {
      coeff = increment;
      for (int x = 0; x < size; x ++)
        {
          *(aptr + dims.w() - 1 - x * 2) =
            (T)(*(aptr + dims.w() - 1 - x * 2) * coeff);

          *aptr = (T)( (*aptr) * coeff );
          ++aptr;
          coeff += increment;
        }
      aptr += dims.w() - size;
    }
  // bottom lines
  aptr = a.beginw() + (dims.h() - size) * dims.w();
  coeff = increment * (float)size;
  for (int y = dims.h() - size; y < dims.h(); y ++)
    {
      for (int x = 0; x < dims.w(); ++x)
        {
          *aptr = (T)( (*aptr) * coeff );
          ++aptr;
        }
      coeff -= increment;
    }
}

// ######################################################################
template <class T>
void inplaceSetBorders(Image<T>& a,
                       const int size, const T value)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(a.initialized());
  int siz = size;
  if (size * 2 > a.getWidth()) siz = a.getWidth() / 2;
  if (size * 2 > a.getHeight()) siz = a.getHeight() / 2;
  if (siz < 1) return;  // forget it

  typename Image<T>::iterator const data = a.beginw();

  // do the first line explicitly:
  for (int i = 0; i < a.getWidth(); ++i)
    data[i] = value;

  // then copy to next lines:
  for (int y = 1; y < siz; ++y)
    safecopy(data + (y * a.getWidth()),
             data,
             a.getWidth());

  // then do the vertical borders explicitly:
  for (int y = siz; y < a.getHeight() - siz; ++y)
    {
      typename Image<T>::iterator aptr = data + (y * a.getWidth());
      for (int x = 0; x < siz; ++x)
        {
          aptr[x] = value;
          aptr[a.getWidth() - 1 - x] = value;
        }
    }

  // finally do the bottom lines as a copy of the top lines
  safecopy(data + (a.getHeight() - siz) * a.getWidth(),
           data,
           siz * a.getWidth());
}

// ######################################################################
void inplaceSpeckleNoise(Image<byte>& dest,
                         const float thresh, const int size,
                         const byte noise_color, bool random_color)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(dest.initialized()); ASSERT(size >= 1);

  if (random_color)
    {
      for (int j = 0; j <= dest.getHeight() - size; j += size)
        for (int i = 0; i <= dest.getWidth() - size; i += size)
          if (randomDouble() < thresh) {
            byte rcol = clamped_convert<byte>(noise_color * randomDouble());
            for (int k = j; k < j + size; k ++)
              for (int l = i; l < i + size; l ++)
                dest.setVal(l, k, rcol);
          }
    }
  else
    {
      for (int j = 0; j <= dest.getHeight() - size; j += size)
        for (int i = 0; i <= dest.getWidth() - size; i += size)
          if (randomDouble() < thresh)
            for (int k = j; k < j + size; k ++)
              for (int l = i; l < i + size; l ++)
                dest.setVal(l, k, noise_color);
    }
}

// ######################################################################
template <class T>
Image<typename promote_trait<T,float>::TP>
addPowerNoise(const Image<T>&  src, double beta)
{
  //Adds power noise on an input image
  //beta =  0 for white noise (flat)
  //beta = -1 for pink  noise (1/f)
  //beta = -2 fo  brown noise (1/f^2)
  
  typedef std::complex<float> complexf;

  int inputW = src.getWidth(), inputH = src.getHeight();
  const int N = inputW > inputH ? inputW : inputH; //max dimension
  
  // ifft takes a half plane in the positive x direction
  // so we need to pass it coordinates for 
  // 0 < x <= N, -N/2 <= y <= N/2 (excluding 0)
  std::vector<float> xaxis(N), yaxis(N), u(N*N), v(N*N);
  int crnt = 0; 
  for(int i = 0; i < N; i++)
    {
      xaxis[i] = i+1;
      if(i < N/2)
        yaxis[i] = i+1;
      else
        yaxis[i] = i-N;
    }

  for(int i=0; i<N; i++)
    for(int j=0; j<N; j++)
      {
        u[crnt]  = xaxis[j];
        v[crnt]  = yaxis[i];
        crnt++;
      }

  Image<complexf> tempCmplxImg(N,N,NO_INIT);
  typename Image<complexf>::iterator cmplxItr = tempCmplxImg.beginw();
  float spectrum, phi;
  srand(time(NULL));
  for(int i=0; i < N*N; i++)
   {
     spectrum = pow((u[i]*u[i]) + (v[i]*v[i]),beta);
     
     phi = 2*M_PI*((double)rand()/((double)(RAND_MAX)+(double)(1)) );     
      *cmplxItr++    = complexf(pow(spectrum,0.5),0.0) * 
        complexf(cos(phi),sin(phi));
   }

  FourierInvEngine<double> ieng(Dims(2*N-1, N)); //twice as wide in the x-direction
  const Image<double> fullImg = ieng.ifft(tempCmplxImg); 
  //fullImg = remapRange(fullImg,rangeOf(fullImg),Range<T_or_RGB>((T_or_RGB)0,(T_or_RGB)255);
  
  Image<typename promote_trait<T, double>::TP> mask = crop(fullImg,Point2D<int>(0,0),Dims(inputW,inputH));
  
  return src + mask;
}

// ######################################################################
float getLocalMax(const Image<float>& src,
                  const Point2D<int>& center, const int radius)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());
  float val = src.getVal(center); // a bit wasteful but less bogus that numeric_limits
  const int r2 = radius * radius, ci = center.i, cj = center.j;
  for (int y = -radius; y <= radius; y ++)
    {
      int bound = int(sqrtf(float(r2 - y*y)));
      int sj = y + cj;
      for (int x = -bound; x <= bound; x ++)
        {
          int si = x + ci;
          if (src.coordsOk(si, sj))
            { float v = src.getVal(si, sj); if (v > val) val = v; }
        }
    }
  return val;
}

// ######################################################################
float getLocalAvg(const Image<float>& src,
                  const Point2D<int>& center, const int radius)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());
  float val = src.getVal(center); // a bit wasteful but less bogus that numeric_limits
  int count = 0;
  const int r2 = radius * radius, ci = center.i, cj = center.j;
  for (int y = -radius; y <= radius; y ++)
    {
      int bound = int(sqrtf(float(r2 - y*y)));
      int sj = y + cj;
      for (int x = -bound; x <= bound; x ++)
        {
          int si = x + ci;
          if (src.coordsOk(si, sj))
            { val += src.getVal(si, sj); ++count; }
        }
    }
  return count == 0? 0: val / (float)count;
}

// ######################################################################
template<class T>
void getMinMax(const Image<T>& src, T& xmini, T& xmaxi)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());
  typename Image<T>::const_iterator aptr = src.begin(), stop = src.end();
  xmini = *aptr; xmaxi = *aptr++;
  while (aptr != stop)
    {
      if (*aptr < xmini) xmini = *aptr;
      else if (*aptr > xmaxi) xmaxi = *aptr;
      ++aptr;
    }
}

// ######################################################################
template <class T>
void getMaskedMinMax(const Image<T>& src, const Image<byte>& mask,
                     T& min_in, T& max_in, T& min_out, T& max_out)
{
GVX_TRACE(__PRETTY_FUNCTION__);
 ASSERT(src.initialized());
 ASSERT(mask.initialized());
 ASSERT(src.isSameSize(mask));

  typename Image<T>::const_iterator aptr = src.begin(), stop = src.end();
  Image<byte>::const_iterator mptr = mask.begin();

  // we need initial values for the min_in and max_in that come from
  // within the mask, and initial values for min_out and max_out that
  // come from outside the mask. So in this first loop we just start
  // scanning the image until we have found these:

  // let's use the first point, it's either in or out:
  if (*mptr++) {
    // ok, we got the values for inside, let's look for the outside ones:
    min_in = *aptr; max_in = *aptr++;
    while (aptr != stop)
      {
        if (*mptr++) { // inside the mask? update our inside values
          if (*aptr < min_in) min_in = *aptr;
          else if (*aptr > max_in) max_in = *aptr;
        } else {     // outside the mask? initialize outside vals and move on
          min_out = *aptr; max_out = *aptr++; break;
        }
        ++aptr;
      }
  } else {
    // ok, we got the values for outside, let's look for the inside ones:
    min_out = *aptr; max_out = *aptr++;
    while (aptr != stop)
      {
        if (*mptr++) { // inside the mask? initialize inside vals and move on
          min_in = *aptr; max_in = *aptr++; break;
        } else {       // outside the mask? update our values
          if (*aptr < min_out) min_out = *aptr;
          else if (*aptr > max_out) max_out = *aptr;
        }
        ++aptr;
      }
  }

  // we are now ready to finish up the iterations, knowing that both
  // the inside and outside values have been initialized and just need
  // updating:
  while (aptr != stop)
    {
      if (*mptr++) { // inside the mask?
        if (*aptr < min_in) min_in = *aptr;
        else if (*aptr > max_in) max_in = *aptr;
      } else {     // outside the mask
        if (*aptr < min_out) min_out = *aptr;
        else if (*aptr > max_out) max_out = *aptr;
      }
      ++aptr;
    }
}

// ######################################################################
template<class T>
void getMinMaxAvg(const Image<T>& src, T& xmini, T& xmaxi, T& xavg)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());
  typename Image<T>::const_iterator aptr = src.begin(), stop = src.end();
  double avg = 0.0;
  xmini = *aptr; xmaxi = *aptr++;
  while (aptr != stop)
    {
      if (*aptr < xmini) xmini = *aptr;
      else if (*aptr > xmaxi) xmaxi = *aptr;
      avg += (double)(*aptr);
      aptr++;
    }
  xavg = clamped_convert<T>(avg / double(src.getSize()));
}

// ######################################################################
template <class T>
void getMaskedMinMaxAvg(const Image<T>& src, const Image<byte> &mask,
                        T& mi, T& ma, T& avg)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());
  ASSERT(src.isSameSize(mask));

  typename Image<T>::const_iterator aptr = src.begin(), stop = src.end();
  typename Image<byte>::const_iterator mptr = mask.begin();
  typedef typename promote_trait<T,T>::TP TF;

  TF sum(0); uint area = 0;

  // we need initial values for mi and ma that come from within the
  // mask. So in this first loop we just start scanning the image
  // until we hit a non-zero pixel in the mask:
  while (aptr != stop && *mptr == 0) { aptr ++; mptr ++; }

  // mask was empty?
  if (aptr == stop) { mi = T(0); ma = T(0); avg = T(0); return; }

  // set initial mi/ma/sum values:
  mi = *aptr; sum = *aptr; ma = *aptr ++; mptr ++; area ++;

  // loop over the remainder of the image:
  while (aptr != stop) {
    if (*mptr) {
      if (*aptr < mi) mi = *aptr; else if (*aptr > ma) ma = *aptr;
      sum += *aptr; area ++;
    }
    aptr++; mptr++;
  }

  if (area) avg = clamped_convert<T>(sum / area); else avg = T(0);
}

// ######################################################################
template <class T>
void getMaskedMinMaxSumArea(const Image<T>& src, const Image<float> &mask,
                            T& mi, T& ma,
                            typename promote_trait<T,float>::TP &sum,
                            typename promote_trait<T,float>::TP &area)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());
  ASSERT(src.isSameSize(mask));

  typename Image<T>::const_iterator aptr = src.begin(), stop = src.end();
  typename Image<float>::const_iterator mptr = mask.begin();
  typedef typename promote_trait<T,float>::TP TF;

  sum = TF(0); area = TF(0);

  // we need initial values for mi and ma that come from within the
  // mask. So in this first loop we just start scanning the image
  // until we hit a non-zero pixel in the mask:
  while (aptr != stop && *mptr <= 0.0F) { aptr ++; mptr ++; }

  // mask was empty?
  if (aptr == stop)
    { mi = T(0); ma = T(0); sum = TF(0); area = TF(0); return; }

  // set initial mi/ma values:
  mi = *aptr; ma = *aptr ++; mptr ++;

  // loop over the remainder of the image:
  while (aptr != stop) {
    if (*mptr > 0.0F) {
      if (*aptr < mi) mi = *aptr; else if (*aptr > ma) ma = *aptr;
      sum += *aptr * TF(*mptr); area += TF(*mptr);
    }
    aptr++; mptr++;
  }
}

// ######################################################################
template<class T>
void getMinMaxAvgEtc(const Image<T>& src, T& xmini, T& xmaxi, T& xavg, T& xstd,
                     ushort& minPosX, ushort& minPosY,
                     ushort& maxPosX, ushort& maxPosY,
                     uint& pixCount)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());
  typename Image<T>::const_iterator aptr = src.begin();
  double avg = 0.0;
  double ss  = 0.0;
  pixCount   = 0;
  const ushort width = src.getWidth();
  xmini = *aptr; xmaxi = *aptr++;
  while (aptr != src.end())
  {
    if (*aptr < xmini)
    {
      xmini   = *aptr;
      minPosX = (ushort)(pixCount%width);
      minPosY = (ushort)floor(pixCount/width);
    }
    else if (*aptr > xmaxi)
    {
      xmaxi   = *aptr;
      maxPosX = (ushort)(pixCount%width);
      maxPosY = (ushort)floor(pixCount/width);
    }
    avg += (double)(*aptr);
    ss  += (double)(pow(*aptr,2));
    aptr++; pixCount++;
  }
  xavg = clamped_convert<T>(avg / double(src.getSize()));
  xstd = clamped_convert<T>(sqrt((ss/double(src.getSize())) - pow(xavg,2)));
}

// ######################################################################
template<class T>
bool isFinite(const Image<T>& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());
  typename Image<T>::const_iterator aptr = src.begin(), stop = src.end();
  while (aptr != stop)
    {
      if (::isFinite(*aptr) == false) return false;
      aptr++;
    }
  return true;
}

// ######################################################################
template<class T>
void findMax(const Image<T>& src, Point2D<int>& p, T& maxval)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());
  typename Image<T>::const_iterator aptr = src.begin();
  const int w = src.getWidth(), h = src.getHeight();
  p.i = 0; p.j = 0; maxval = *aptr;
  for (int j = 0; j < h; j ++)
    for (int i = 0; i < w; i ++)
      {
        if (*aptr > maxval) { maxval = *aptr; p.i = i; p.j = j; }
        aptr++;
      }
}

// ######################################################################
template<class T>
void findMin(const Image<T>& src, Point2D<int>& p, T& minval)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());
  typename Image<T>::const_iterator aptr = src.begin();
  const int w = src.getWidth(), h = src.getHeight();
  p.i = 0; p.j = 0; minval = *aptr;
  for (int j = 0; j < h; j ++)
    for (int i = 0; i < w; i ++)
      {
        if (*aptr < minval) { minval = *aptr; p.i = i; p.j = j; }
        aptr++;
      }
}

// ######################################################################
template <class T>
void inplaceClamp(Image<T>& dst, const T cmin, const T cmax)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  typename Image<T>::iterator aptr = dst.beginw(), stop = dst.endw();
  while (aptr != stop)
    {
      if (*aptr < cmin) *aptr = cmin;
      else if (*aptr > cmax) *aptr = cmax;
      ++aptr;
    }
}

// ######################################################################
template <class T>
void inplaceNormalize(Image<T>& dst, const T nmin, const T nmax)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  T oldmin, oldmax;
  inplaceNormalize(dst, nmin, nmax, oldmin, oldmax);
}

// ######################################################################
template <class T>
void inplaceNormalize(Image<T>& dst, const T nmin, const T nmax,
                      T& oldmin, T& oldmax)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (!dst.initialized()) return;

  typedef typename promote_trait<T, float>::TP TF;

  getMinMax(dst, oldmin, oldmax);

  // check if the original range is empty; NOTE: don't try to check
  // (oldmax-oldmin)==0.0 here, because that will fail if oldmin=inf
  // and oldmax=inf, since inf-inf=nan, whereas a test of inf==inf
  // will still yield true
  if (oldmax == oldmin)
    {
      // OK, the input image was uniform, so let's just change it to
      // be uniform at the min of the desired new scale.

      // NOTE: Another sensible approach in this case might be to set
      // the new image to the midpoint of the new range, but this
      // causes backward-compatibility problems, particularly in cases
      // where we expect input images containing all 0's to stay at 0
      // rather than shifting to a higher non-zero value after
      // rescaling.
      dst.clear(nmin);
      return;
    }
  const TF scale = TF(oldmax) - TF(oldmin);
  const TF nscale = (TF(nmax) - TF(nmin)) / scale;

  typename Image<T>::iterator aptr = dst.beginw();
  typename Image<T>::iterator stop = dst.endw();
  const TF oldminf = TF(oldmin);

  while (aptr != stop)
    {
      *aptr = nmin + T( (TF(*aptr) - oldminf) * nscale );
      ++aptr;
    }
}

// ######################################################################
template <class T>
bool isLocalMax(const Image<T>& src, const Point2D<int>& p)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  typename Image<T>::const_iterator sptr = src.begin();
  const int w = src.getWidth(), h = src.getHeight();
  const int i = p.i, j = p.j;
  sptr += i + w * j;
  T val = *sptr;

  if (src.coordsOk(p) == false) return false; // outside the image
  if (i > 0 && sptr[-1] > val) return false;  // (i-1,j) is higher
  if (i < w-1 && sptr[1] > val) return false; // (i+1,j) is higher
  if (j > 0 && sptr[-w] > val) return false;  // (i,j-1) is higher
  if (j < h-1 && sptr[w] > val) return false; // (i,j+1) is higher

  // no neighbor was higher, so we must be the highest:
  return true;
}

// ######################################################################
template <class T>
void inplaceRectify(Image<T>& dst)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  typename Image<T>::iterator aptr = dst.beginw(), stop = dst.endw();
  T zero = T();
  while (aptr != stop)
    {
      if (*aptr < zero) *aptr = zero;
      ++aptr;
    }
}

// ######################################################################
template <class T>
void splitPosNeg(const Image<T>& src,
                 Image<T>& pos, Image<T>& neg)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  pos.resize(src.getDims(), NO_INIT);
  neg.resize(src.getDims(), NO_INIT);
  typename Image<T>::const_iterator sptr = src.begin(), stop = src.end();
  typename Image<T>::iterator pptr = pos.beginw(), nptr = neg.beginw();

  T zero = T();
  while (sptr != stop)
    if (*sptr <= zero) { *nptr++ = - (*sptr++); *pptr++ = zero; }
    else { *nptr++ = zero; *pptr++ = *sptr++; }
}

// ######################################################################
template <class T>
void inplaceLowThresh(Image<T>& dst, const T thresh, const T val)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  typename Image<T>::iterator aptr = dst.beginw(), stop = dst.endw();

  while (aptr != stop)
    {
      if (*aptr < thresh) *aptr = val;
      ++aptr;
    }
}

// ######################################################################
template <class T>
void inplaceLowThreshAbs(Image<T>& dst, const T thresh, const T val)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (thresh <= T(0))
    {
      if (thresh == T(0))
        // it's still a no-op with thresh = 0 but we won't report an
        // LERROR() here since 0 is a convenient default value when no
        // threshold should be applied...
        return;

      LERROR("With thresh = %f < 0.0, this is a no-op -- IGNORED",
             double(thresh));
    }

  typename Image<T>::iterator aptr = dst.beginw(), stop = dst.endw();

  while (aptr != stop)
    {
      if (std::abs(*aptr) < thresh) *aptr = val;
      ++aptr;
    }
}

// ######################################################################
template <class T>
void inplaceSigmoid(Image<T>& dst,
                    const float g, const float h, const float s)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // Sigmoidal normalization: f(x)=x^g/(s+x^h)
  typename Image<T>::iterator aptr = dst.beginw(), stop = dst.endw();

  while (aptr != stop) {
    T val = clamped_convert<T>(pow(*aptr, g) / (s + pow(*aptr, h)));
    *aptr++ = val;
  }
}

// ######################################################################
template <class T>
int emptyArea(const Image<T>& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return std::count(src.begin(), src.end(), T());
}

// ######################################################################
template <class T>
int countThresh(const Image<T>& src, const T thresh, const bool absol)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  int nb = 0;
  typename Image<T>::const_iterator ap = src.begin(), stop = src.end();

  if (absol)
    {
      float threshd = (float)thresh;
      while (ap != stop)
        {
          if (fabs(float(*ap)) > threshd) ++nb;
          ++ap;
        }
    }
  else
    {
      while (ap != stop)
        if (*ap++ > thresh) ++nb;
    }

  return nb;
}

// ######################################################################
Image<float> meanRow(const Image<float>& inp)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(inp.getHeight() > 0);

  Image<float> result(inp.getWidth(), 1, ZEROS);

  const int w = inp.getWidth();
  const int h = inp.getHeight();

  Point2D<int> loc;

  for (loc.j = 0; loc.j < h; ++loc.j)
    for (loc.i = 0; loc.i < w; ++loc.i)
      result[Point2D<int>(loc.i,0)] += inp[loc];

  result /= inp.getHeight();

  return result;
}

// ######################################################################
Image<float> stdevRow(const Image<float>& M)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  const Image<float> u = meanRow(M);

  const int w = M.getWidth();
  const int h = M.getHeight();

  Image<float> ssq(M.getWidth(), 1, ZEROS);

  Image<float>::iterator const dptr = ssq.beginw();
  Image<float>::const_iterator const uptr = u.begin();
  Image<float>::const_iterator Mptr = M.begin();

  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x)
      {
        dptr[x] += (*Mptr - uptr[x]) * (*Mptr - uptr[x]);
        ++Mptr;
      }

  ASSERT(h > 1);
  ssq /= float(h - 1);

  return sqrt(ssq);
}

// ######################################################################
Image<float> addRow(const Image<float>& M, const Image<float>& v)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(M.getWidth() == v.getWidth());
  ASSERT(v.getHeight() == 1);

  const int w = M.getWidth();
  const int h = M.getHeight();

  Image<float> result(M.getDims(), NO_INIT);

  Image<float>::iterator dptr = result.beginw();
  Image<float>::const_iterator Mptr = M.begin();
  Image<float>::const_iterator const vptr = v.begin();

  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x)
      {
        *dptr = (*Mptr) + vptr[x];
        ++dptr;
        ++Mptr;
      }

  return result;
}

// ######################################################################
Image<float> subtractRow(const Image<float>& M, const Image<float>& v)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(M.getWidth() == v.getWidth());
  ASSERT(v.getHeight() == 1);

  const int w = M.getWidth();
  const int h = M.getHeight();

  Image<float> result(M.getDims(), NO_INIT);

  Image<float>::iterator dptr = result.beginw();
  Image<float>::const_iterator Mptr = M.begin();
  Image<float>::const_iterator const vptr = v.begin();

  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x)
      {
        *dptr = (*Mptr) - vptr[x];
        ++dptr;
        ++Mptr;
      }

  return result;
}

// ######################################################################
Image<float> multiplyRow(const Image<float>& M, const Image<float>& v)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(M.getWidth() == v.getWidth());
  ASSERT(v.getHeight() == 1);

  const int w = M.getWidth();
  const int h = M.getHeight();

  Image<float> result(M.getDims(), NO_INIT);

  Image<float>::iterator dptr = result.beginw();
  Image<float>::const_iterator Mptr = M.begin();
  Image<float>::const_iterator const vptr = v.begin();

  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x)
      {
        *dptr = (*Mptr) * vptr[x];
        ++dptr;
        ++Mptr;
      }

  return result;
}

// ######################################################################
Image<float> divideRow(const Image<float>& M, const Image<float>& v)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(M.getWidth() == v.getWidth());
  ASSERT(v.getHeight() == 1);

  const int w = M.getWidth();
  const int h = M.getHeight();

  Image<float> result(M.getDims(), NO_INIT);

  Image<float>::iterator dptr = result.beginw();
  Image<float>::const_iterator Mptr = M.begin();
  Image<float>::const_iterator const vptr = v.begin();

  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x)
      {
        *dptr =
          vptr[x] == 0.0f
          ? 0.0f
          : (*Mptr) / vptr[x];
        ++dptr;
        ++Mptr;
      }

  return result;
}

// ######################################################################
std::vector<Point2D<int> >  approxPolyDP(std::vector<Point2D<int> >& points, float tol)
{

  std::vector<Point2D<int> > result;

  float tol2 = tol * tol;
  uint i,k, pv;

  if (points.size() < 2) //nothing to approximate
    return points;

  std::vector<Point2D<int> > vt(points.size());
  std::vector<int> mk(points.size(), 0);

  //Vertex reduction within tolerance of prior vertex cluster
  vt[0] = points[0];
  for(i=k=1, pv = 0; i<points.size(); i++)
  {
    if (points[i].squdist(points[pv]) < tol2)
      continue;
    vt[k++] = points[i];
    pv = i;
  }

  //Douglas-Peucker polyine simplification

  mk[0] = mk[k-1] = 1;
  recursePolyDP(tol, vt, 0, k-1, mk);

  //get the output
  for(i=0; i<k; i++)
  {
    if (mk[i])
      result.push_back(vt[i]);
  }

  return result;

}

void recursePolyDP(float tol, std::vector<Point2D<int> >& v, int j, int k, std::vector<int>& mk)
{

  if (k <= j+1) // there is nothing to simplify
    return;

  // check for adequate approximation by segment S from v[j] to v[k]
  int     maxi = j;          // index of vertex farthest from S
  float   maxd2 = 0;         // distance squared of farthest vertex
  float   tol2 = tol * tol;  // tolerance squared

  //Segment S = {v[j], v[k]};  // segment from v[j] to v[k]

  //Vector  u = S.P1 - S.P0;   
  Point2D<int> SP0 = v[k];
  Point2D<int> SP1 = v[j];
  Point2D<int> u = SP1 - SP0; // segment direction vector

  double  cu = u.i*u.i + u.j*u.j;     // segment length squared

  // test each vertex v[i] for max distance from S
  // compute using the Feb 2001 Algorithm's dist_Point_to_Segment()
  // Note: this works in any dimension (2D, 3D, ...)
  Point2D<int>  w;
  Point2D<int>   Pb;                // base of perpendicular from v[i] to S
  double  b, cw, dv2;        // dv2 = distance v[i] to S squared

  for (int i=j+1; i<k; i++)
  {
    // compute distance squared
    w = v[i] - SP0;
    cw = w.i*u.i + w.j*u.j; //dot product
    if ( cw <= 0 )
      dv2 = v[i].squdist(SP0);
    else if ( cu <= cw )
      dv2 = v[i].squdist(SP1);
    else {
      b = cw / cu;
      Pb = SP0 + Point2D<int>(int(u.i*b), int(u.j*b)); 
      dv2 = v[i].squdist(Pb);
    }
    // test with current max distance squared
    if (dv2 <= maxd2)
      continue;
    // v[i] is a new max vertex
    maxi = i;
    maxd2 = dv2;
  }
  if (maxd2 > tol2)        // error is worse than the tolerance
  {
    // split the polyline at the farthest vertex from S
    mk[maxi] = 1;      // mark v[maxi] for the simplified polyline
    // recursively simplify the two subpolylines at v[maxi]
    recursePolyDP( tol, v, j, maxi, mk );  // polyline v[j] to v[maxi]
    recursePolyDP( tol, v, maxi, k, mk );  // polyline v[maxi] to v[k]
  }
  // else the approximation is OK, so ignore intermediate vertices
  return;
}


namespace 
{
	const int NONE = -1;

	typedef struct range_bin Bin;
	struct range_bin {
		int    min;    // index of min point P[] in bin (>=0 or NONE)
		int    max;    // index of max point P[] in bin (>=0 or NONE)
	};


	inline float
		isLeft( Point2D<float> P0, Point2D<float> P1, Point2D<float> P2 )
		{
			return (P1.i - P0.i)*(P2.j - P0.j) - (P2.i - P0.i)*(P1.j - P0.j);
		}

}

std::vector<Point2D<float> > approximateHull(std::vector<Point2D<float> > P, int accuracy)
{
	std::vector<Point2D<float> >H(P.size());
	int n = P.size();
	int k = accuracy;
	int    minmin=0,  minmax=0;
	int    maxmin=0,  maxmax=0;
	float  xmin = P[0].i,  xmax = P[0].i;
	Point2D<float>* cP;        // the current point being considered
	int    bot=0, top=(-1);  // indices for bottom and top of the stack

	// Get the points with (1) min-max x-coord, and (2) min-max y-coord
	for (int i=1; i<n; i++) {
		cP = &P[i];
		if (cP->i <= xmin) {
			if (cP->i < xmin) {        // new xmin
				xmin = cP->i;
				minmin = minmax = i;
			}
			else {                      // another xmin
				if (cP->j < P[minmin].j)
					minmin = i;
				else if (cP->j > P[minmax].j)
					minmax = i;
			}
		}
		if (cP->i >= xmax) {
			if (cP->i > xmax) {        // new xmax
				xmax = cP->i;
				maxmin = maxmax = i;
			}
			else {                      // another xmax
				if (cP->j < P[maxmin].j)
					maxmin = i;
				else if (cP->j > P[maxmax].j)
					maxmax = i;
			}
		}
	}
	if (xmin == xmax) {      // degenerate case: all x-coords == xmin
		H[++top] = P[minmin];           // a point, or
		if (minmax != minmin)           // a nontrivial segment
			H[++top] = P[minmax];
		H.resize(top+1);
		return H;
	}

	// Next, get the max and min points in the k range bins
	Bin*   B = new Bin[k+2];   // first allocate the bins
	B[0].min = minmin;         B[0].max = minmax;        // set bin 0
	B[k+1].min = maxmin;       B[k+1].max = maxmax;      // set bin k+1
	for (int b=1; b<=k; b++) { // initially nothing is in the other bins
		B[b].min = B[b].max = NONE;
	}
	for (int b, i=0; i<n; i++) {
		cP = &P[i];
		if (cP->i == xmin || cP->i == xmax) // already have bins 0 and k+1 
			continue;
		// check if a lower or upper point
		if (isLeft( P[minmin], P[maxmin], *cP) < 0) {  // below lower line
			b = (int)( k * (cP->i - xmin) / (xmax - xmin) ) + 1;  // bin #
			if (B[b].min == NONE)       // no min point in this range
				B[b].min = i;           // first min
			else if (cP->j < P[B[b].min].j)
				B[b].min = i;           // new min
			continue;
		}
		if (isLeft( P[minmax], P[maxmax], *cP) > 0) {  // above upper line
			b = (int)( k * (cP->i - xmin) / (xmax - xmin) ) + 1;  // bin #
			if (B[b].max == NONE)       // no max point in this range
				B[b].max = i;           // first max
			else if (cP->j > P[B[b].max].j)
				B[b].max = i;           // new max
			continue;
		}
	}

	// Now, use the chain algorithm to get the lower and upper hulls
	// the output array H[] will be used as the stack
	// First, compute the lower hull on the stack H
	for (int i=0; i <= k+1; ++i)
	{
		if (B[i].min == NONE)  // no min point in this range
			continue;
		cP = &P[ B[i].min ];   // select the current min point

		while (top > 0)        // there are at least 2 points on the stack
		{
			// test if current point is left of the line at the stack top
			if (isLeft( H[top-1], H[top], *cP) > 0)
				break;         // cP is a new hull vertex
			else
				top--;         // pop top point off stack
		}
		H[++top] = *cP;        // push current point onto stack
	}

	// Next, compute the upper hull on the stack H above the bottom hull
	if (maxmax != maxmin)      // if distinct xmax points
		H[++top] = P[maxmax];  // push maxmax point onto stack
	bot = top;                 // the bottom point of the upper hull stack
	for (int i=k; i >= 0; --i)
	{
		if (B[i].max == NONE)  // no max point in this range
			continue;
		cP = &P[ B[i].max ];   // select the current max point

		while (top > bot)      // at least 2 points on the upper stack
		{
			// test if current point is left of the line at the stack top
			if (isLeft( H[top-1], H[top], *cP) > 0)
				break;         // current point is a new hull vertex
			else
				top--;         // pop top point off stack
		}
		H[++top] = *cP;        // push current point onto stack
	}
	if (minmax != minmin)
		H[++top] = P[minmin];  // push joining endpoint onto stack

	delete B;                  // free bins before returning
	H.resize(top+1);
	return H;
}


// Include the explicit instantiations
#include "inst/Image/MathOps.I"

// additional explicit instantiations:
template double sum(const Image<double>&);
template double mean(const Image<double>&);
template double stdev(const Image<double>&);
template Range<double> rangeOf(const Image<double>&);
template Image<double> remapRange(const Image<double>&,
                                  const Range<double>&, const Range<double>&);
template Image<double> takeMax(const Image<double>& a, const Image<double>& b);
template Image<int> takeMax(const Image<int>& a, const Image<int>& b);
template void findMax(const Image<double>& src, Point2D<int>& p, double& maxval);
template void getMinMax<int>(const Image<int>&, int&, int&);
template void getMinMax<double>(const Image<double>&, double&, double&);
template void inplaceLowThresh(Image<int>& dst, const int thresh, const int val);
template void inplaceLowThreshAbs(Image<int>& dst, const int thresh, const int val);
template void inplaceRectify(Image<int>&);
template void inplaceRectify(Image<double>&);
template void inplaceClamp(Image<double>&, double, double);
template Image<double> abs(const Image<double>&);
template Image<double> exp(const Image<double>&);
template Image<double> log(const Image<double>&);
template Image<double> log10(const Image<double>&);
template Image<byte> thresholdedMix(const Image<float>&,
                                    const float&,
                                    const Image<byte>&,
                                    const Image<byte>&);
template Image<double> toPower(const Image<double>&, double);
template Image<double> addPowerNoise(const Image<double>&, double);

template double RMSerr(const Image<double>&, const Image<double>&);
template Image<int> absDiff(const Image<int>& b, const Image<int>& c);
template void inplaceNormalize(Image<double>&, double, double);
template void inplaceNormalize(Image<double>&, double, double,
                               double&, double&);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !IMAGE_MATHOPS_C_DEFINED

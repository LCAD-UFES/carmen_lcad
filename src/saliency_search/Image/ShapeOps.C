/*!@file Image/ShapeOps.C Shape operations on Image
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/ShapeOps.C $
// $Id: ShapeOps.C 15273 2012-05-07 21:20:18Z kai $
//

// Some of the code in this file is based on public domain code from
// Graphics Gems III by Dale Schumacher and Ray Gardener, available
// from http://www.graphicsgems.org/ (filter_rcg.c)

#ifndef IMAGE_SHAPEOPS_C_DEFINED
#define IMAGE_SHAPEOPS_C_DEFINED

#include "Image/ShapeOps.H"

#include "Image/CutPaste.H"
#include "Image/FilterOps.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/vec2.h"
#include "Util/Assert.H"
#include "Util/StringUtil.H" // for toLowerCase()
#include "Util/log.H"
#include "Util/safecopy.H"
#include "rutz/trace.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#define ADD_RGB(buf,Weight,bWeight,in) do { \
        *buf += *in * (Weight); \
        *bWeight += Weight; } while (0);

// ######################################################################
template <class T>
Image<T> quickLocalAvg(const Image<T>& array, const int scale)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(array.initialized());
  int lw = array.getWidth(), lh = array.getHeight();
  int sw = std::max(1, lw / scale), sh = std::max(1, lh / scale);
  int scalex = lw / sw, scaley = lh / sh; // in case the image is very small
  int remx = lw - 1 - (lw % sw), remy = lh - 1 - (lh % sh);

  typedef typename promote_trait<T, float>::TP TF;
  Image<TF> result(sw, sh, ZEROS);

  int i, j, ci = 0, cj = 0; float fac = 1.0f / float(scale * scale);
  typename Image<T>::const_iterator lptr = array.begin();
  typename Image<TF>::iterator dptr = result.beginw();

  for (j = 0; j < lh; ++j)
    {
      for (i = 0; i < lw; ++i)
        {
          *dptr += (*lptr++);
          if ((++ci) == scalex && i != remx) { ci = 0; ++dptr; }
        }
      if (ci) { ci = 0; ++dptr; } // in case the reduction is not round
      if ( (++cj) == scaley && j != remy) cj = 0; else dptr -= sw;
    }

  Image<T> ret = result * fac;  // normalize by pixel area
  return ret;
}

// ######################################################################
template <class T>
Image<T> quickLocalAvg2x2(const Image<T>& src)
{
  const int w = src.getWidth();
  const int h = src.getHeight();

  const int nw = w / 2;
  const int nh = h / 2;

  Image<T> result(nw, nh, ZEROS);

  typename Image<T>::const_iterator sptr = src.begin();
  typename Image<T>::iterator dptr = result.beginw();

  for (int y = 0; y < nh; ++y)
    {
      typename Image<T>::const_iterator sptr2 = sptr + y*2*w;
      for (int x = 0; x < nw; ++x)
        {
          *dptr++ = T((sptr2[0] + sptr2[1] + sptr2[w] + sptr2[w+1]) * 0.25);
          sptr2 += 2;
        }
    }

  return result;
}

// ######################################################################

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

template <class T>
Image<T> quickLocalMax(const Image<T>& array, const int scale)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(array.initialized());
  int lw = array.getWidth(), lh = array.getHeight();
  int sw = std::max(1, lw / scale), sh = std::max(1, lh / scale);
  int scalex = lw / sw, scaley = lh / sh;  // in case the image is very small
  int remx = lw - 1 - (lw % sw), remy = lh - 1 - (lh % sh);
  Image<T> result(sw, sh, ZEROS); // clear dest
  int i, j, ci = 0, cj = 0;
  typename Image<T>::const_iterator lptr = array.begin();
  typename Image<T>::iterator dptr = result.beginw();

  for (j = 0; j < lh; ++j)
    {
      for (i = 0; i < lw; ++i)
        {
          *dptr = element_wise_max(*dptr, *lptr);
          ++lptr;
          if ((++ci) == scalex && i != remx) { ci = 0; ++dptr; }
        }
      if (ci) { ci = 0; ++dptr; } // in case the reduction is not round
      if ((++cj) == scaley && j != remy) cj = 0; else dptr -= sw;
    }
  return result;
}

// ######################################################################

namespace
{
  template <class T>
  T element_wise_min(const T& t1, const T& t2)
  {
    return std::min(t1, t2);
  }

  template <class T>
  PixRGB<T> element_wise_min(const PixRGB<T>& t1,
                             const PixRGB<T>& t2)
  {
    return PixRGB<T>(std::min(t1.red(), t2.red()),
                     std::min(t1.green(), t2.green()),
                     std::min(t1.blue(), t2.blue()));
  }
}

template <class T>
Image<T> quickLocalMin(const Image<T>& array, const int scale)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(array.initialized());
  int lw = array.getWidth(), lh = array.getHeight();
  int sw = std::max(1, lw / scale), sh = std::max(1, lh / scale);
  int scalex = lw / sw, scaley = lh / sh;  // in case the image is very small
  int remx = lw - 1 - (lw % sw), remy = lh - 1 - (lh % sh);
  Image<T> result(sw, sh, ZEROS); // clear dest
  int i, j, ci = 0, cj = 0;
  typename Image<T>::const_iterator lptr = array.begin();
  typename Image<T>::iterator dptr = result.beginw();

  for (j = 0; j < lh; ++j)
    {
      for (i = 0; i < lw; ++i)
        {
          *dptr = element_wise_min(*dptr, *lptr);
          ++lptr;
          if ((++ci) == scalex && i != remx) { ci = 0; ++dptr; }
        }
      if (ci) { ci = 0; ++dptr; } // in case the reduction is not round
      if ((++cj) == scaley && j != remy) cj = 0; else dptr -= sw;
    }
  return result;
}

// ######################################################################
template <class T>
Image<T> quickInterpolate(const Image<T>& src, const int sfactor)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(src.initialized()); ASSERT(sfactor >= 2);
  const int sw = src.getWidth();
  const int sh = src.getHeight();
  int lw = sw * sfactor, lh = sh * sfactor;
  Image<T> result(lw, lh, NO_INIT);
  int i, j, ci = 0, cj = 0;
  typename Image<T>::iterator lptr = result.beginw();
  typename Image<T>::const_iterator sptr = src.begin();

  for (j = 0; j < lh; ++j)
    {
      for (i = 0; i < lw; ++i)
        {
          *lptr++ = *sptr;
          if ((++ci) == sfactor) {  ci = 0; ++sptr; }
        }
      if ((++cj) == sfactor) cj = 0; else sptr -= sw;
    }

  return result;
}

// ######################################################################
template <class T>
Image<T> interpolate(const Image<T>& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(src.initialized());
  const int sw = src.getWidth(), sh = src.getHeight();
  const int dw = sw * 2, dh = sh * 2;
  Image<T> result(dw, dh, NO_INIT);

  typename Image<T>::const_iterator sptr = src.begin();
  typename Image<T>::iterator dptr = result.beginw();

  // do rows 1 .. sh-1 (last row will be special)
  for (int j = 0; j < sh-1; j ++)
    {
      // do columns 0 .. sw-1:
      for (int i = 0; i < sw-1; i ++)
        {
          // top-left pixel:
          dptr[0] = sptr[0];

          // bottom-left pixel:
          dptr[dw] = T( (sptr[0] + sptr[sw]) * 0.5F );

          // top-right pixel:
          dptr[1] = T( (sptr[0] + sptr[1]) * 0.5F );

          // bottom-right pixel:
          dptr[dw+1] = T( (sptr[0] + sptr[1] + sptr[sw] + sptr[sw+1])*0.25F );

          dptr += 2; ++sptr;
        }

      // now do last column:

      // top-left pixel:
      dptr[0] = sptr[0];

      // bottom-left pixel:
      dptr[dw] = T( (sptr[0] + sptr[sw]) * 0.5F );

      // top-right pixel:
      dptr[1] = T( sptr[0] * 0.75F );

      // bottom-right pixel:
      dptr[dw+1] = T( dptr[dw] * 0.75F );

      dptr += 2 + dw; ++sptr;
    }

  // now the last row, except for the last pixel:
  for (int i = 0; i < sw-1; i ++)
    {
      // top-left pixel:
      dptr[0] = sptr[0];

      // bottom-left pixel:
      dptr[dw] = T( sptr[0] * 0.75F );

      // top-right pixel:
      dptr[1] = T( (sptr[0] + sptr[1]) * 0.5F );

      // bottom-right pixel:
      dptr[dw+1] = T( dptr[1] * 0.75F );

      dptr += 2; ++sptr;
    }

  // finally the bottom-right corner of the image:

  // top-left pixel:
  dptr[0] = sptr[0];

  // bottom-left pixel:
  dptr[dw] = T( sptr[0] * 0.75F );

  // top-right pixel:
  dptr[1] = T( sptr[0] * 0.75F );

  // bottom-right pixel:
  dptr[dw+1] = T( sptr[0] * 0.5F );

  return result;
}

// ######################################################################
template <class T>
Image<T> rescaleBilinear(const Image<T>& src, const Dims& dims)
{
  return rescaleBilinear(src, dims.w(), dims.h());
}

// ######################################################################
template <class T>
Image<T> rescaleBilinear(const Image<T>& src, const int new_w, const int new_h)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(src.initialized()); ASSERT(new_w > 0 && new_h > 0);

  const int orig_w = src.getWidth();
  const int orig_h = src.getHeight();

  // check if same size already
  if (new_w == orig_w && new_h == orig_h) return src;

  const float sw = float(orig_w) / float(new_w);
  const float sh = float(orig_h) / float(new_h);

  Image<T> result(new_w, new_h, NO_INIT);
  typename Image<T>::iterator dptr = result.beginw();
  typename Image<T>::const_iterator const sptr = src.begin();

  // code inspired from one of the Graphics Gems book:
  /*
    (1) (x,y) are the original coords corresponding to scaled coords (i,j)
    (2) (x0,y0) are the greatest lower bound integral coords from (x,y)
    (3) (x1,y1) are the least upper bound integral coords from (x,y)
    (4) d00, d10, d01, d11 are the values of the original image at the corners
        of the rect (x0,y0),(x1,y1)
    (5) the value in the scaled image is computed from bilinear interpolation
        among d00,d10,d01,d11
  */

  for (int j = 0; j < new_h; ++j)
    {
      const float y = std::max(0.0f, (j+0.5f) * sh - 0.5f);

      const int y0 = int(y);
      const int y1 = std::min(y0 + 1, orig_h - 1);

      const float fy = y - float(y0);

      const int wy0 = orig_w * y0;
      const int wy1 = orig_w * y1;

      for (int i = 0; i < new_w; ++i)
        {
          const float x = std::max(0.0f, (i+0.5f) * sw - 0.5f);

          const int x0 = int(x);
          const int x1 = std::min(x0 + 1, orig_w - 1);

          const float fx = x - float(x0);

          typename promote_trait<T, float>::TP const
            d00( sptr[x0 + wy0] ), d10( sptr[x1 + wy0] ),
            d01( sptr[x0 + wy1] ), d11( sptr[x1 + wy1] ),
            dx0( d00 + (d10 - d00) * fx ),
            dx1( d01 + (d11 - d01) * fx );

          *dptr++ = T( dx0 + (dx1 - dx0) * fy );  // no need to clamp
        }
    }
  return result;
}

// specialization of rescaleBilinear() for PixRGB<byte>; here we
// unpack the PixRGB operations so that they can be better optimized
// by the compiler for at least a 2x speedup -- unfortunately (with
// gcc anyway) the overloaded PixRGB operators (i.e. PixRGB+PixRGB,
// PixRGB*float, etc.) make for slow code, so here we unpack things
// and do the interpolation one element at a time using builtin,
// scalar arithmetic only
template <>
Image<PixRGB<byte> > rescaleBilinear(const Image<PixRGB<byte> >& src,
                                     const int new_w, const int new_h)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(src.initialized()); ASSERT(new_w > 0 && new_h > 0);

  const int orig_w = src.getWidth();
  const int orig_h = src.getHeight();

  // check if same size already
  if (new_w == orig_w && new_h == orig_h) return src;

  const float sw = float(orig_w) / float(new_w);
  const float sh = float(orig_h) / float(new_h);

  Image<PixRGB<byte> > result(new_w, new_h, NO_INIT);
  Image<PixRGB<byte> >::iterator dptr = result.beginw();
  Image<PixRGB<byte> >::const_iterator const sptr = src.begin();

  for (int j = 0; j < new_h; ++j)
    {
      const float y = std::max(0.0f, (j+0.5f) * sh - 0.5f);

      const int y0 = int(y);
      const int y1 = std::min(y0 + 1, orig_h - 1);

      const float fy = y - float(y0);

      const int wy0 = orig_w * y0;
      const int wy1 = orig_w * y1;

      for (int i = 0; i < new_w; ++i)
        {
          const float x = std::max(0.0f, (i+0.5f) * sw - 0.5f);

          const int x0 = int(x);
          const int x1 = std::min(x0 + 1, orig_w - 1);

          const float fx = x - float(x0);

          // TODO if we need more speed, this would be a good place to
          // use sse2 -- could turn d00, d10, d01, d11, dx0, dx1, and
          // the PixRGB<float> result each into 128-bit sse registers
          // (with 3 32-floats, one each for r/g/b, the 4th would be
          // unused), then we could do all of the math in parallel
          // (gcc has thin C wrappers around the sse2 intrinsincs in a
          // header called "emmintrin.h", try "locate emmintrin.h" to
          // find it, on my system it is
          // /usr/lib/gcc/i386-redhat-linux/3.4.3/include/emmintrin.h

#define RGB_BILINEAR_INTERP(EL)                                 \
  do {                                                          \
    const float                                                 \
      d00( sptr[x0 + wy0].p[EL] ), d10( sptr[x1 + wy0].p[EL] ), \
      d01( sptr[x0 + wy1].p[EL] ), d11( sptr[x1 + wy1].p[EL] ); \
                                                                \
    const float                                                 \
      dx0( d00 + (d10 - d00) * fx ),                            \
      dx1( d01 + (d11 - d01) * fx );                            \
                                                                \
    dptr->p[EL] = byte( dx0 + (dx1 - dx0) * fy );               \
  } while(0)

          RGB_BILINEAR_INTERP(0);
          RGB_BILINEAR_INTERP(1);
          RGB_BILINEAR_INTERP(2);

#undef RGB_BILINEAR_INTERP

          ++dptr;
        }
    }
  return result;
}

// ######################################################################
template <class T>
Image<T> rescaleNI(const Image<T>& src, const Dims& dims)
{
  return rescaleNI(src, dims.w(), dims.h());
}

// ######################################################################
template <class T>
Image<T> rescaleNI(const Image<T>& src, const int new_w, const int new_h)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(src.initialized()); ASSERT(new_w > 0 && new_h > 0);

  const int orig_w = src.getWidth();
  const int orig_h = src.getHeight();

  // check if same size already
  if (new_w == orig_w && new_h == orig_h) return src;

  // are we doing an integral up-scaling? then we can use zoomXY() to
  // be more efficient:
  if (new_w >= orig_w && (new_w % orig_w) == 0
      &&
      new_h >= orig_h && (new_h % orig_h) == 0)
    return zoomXY(src, new_w / orig_w, new_h / orig_h);

  // are we doing an integral down-scaling? then we can use decXY() to
  // be more efficient:
  if (new_w <= orig_w && (orig_w % new_w) == 0
      &&
      new_h <= orig_h && (orig_h % new_h) == 0)
    return decXY(src, orig_w / new_w, orig_h / new_h);

  const float sw = float(orig_w) / float(new_w);
  const float sh = float(orig_h) / float(new_h);

  Image<T> result(new_w, new_h, NO_INIT);
  typename Image<T>::iterator dptr = result.beginw();
  typename Image<T>::const_iterator const sptr = src.begin();

  for (int j = 0; j < new_h; ++j)
    for (int i = 0; i < new_w; ++i)
      *dptr++ = sptr[int(j * sh) * orig_w + int(i * sw)];
  return result;
}

// ######################################################################
RescaleType getRescaleTypeFromChar(char ftype)
{
  switch (ftype)
    {
    case 'o': return RESCALE_SIMPLE_NOINTERP;
    case 'r': return RESCALE_SIMPLE_BILINEAR;
    case 'b': return RESCALE_FILTER_BOX;
    case 't': return RESCALE_FILTER_TRIANGLE;
    case 'q': return RESCALE_FILTER_BELL;
    case 'B': return RESCALE_FILTER_BSPLINE;
    case 'h': return RESCALE_FILTER_HERMITE;
    case 'l': return RESCALE_FILTER_LANCZOS3;
    case 'm': return RESCALE_FILTER_MITCHELL;
    default: LFATAL("invalid rescale type char '%c'", ftype);
    }
  ASSERT(0); return (RescaleType) -1;
}

// ######################################################################
std::string convertToString(RescaleType ftype)
{
  switch (ftype)
    {
    case RESCALE_SIMPLE_NOINTERP: return "Nointerp";
    case RESCALE_SIMPLE_BILINEAR: return "Bilinear";
    case RESCALE_FILTER_BOX     : return "Box";
    case RESCALE_FILTER_TRIANGLE: return "Triangle";
    case RESCALE_FILTER_BELL    : return "Bell";
    case RESCALE_FILTER_BSPLINE : return "Bspline";
    case RESCALE_FILTER_HERMITE : return "Hermite";
    case RESCALE_FILTER_LANCZOS3: return "Lanczos3";
    case RESCALE_FILTER_MITCHELL: return "Mitchell";
    default: LFATAL("invalid rescale type %d", int(ftype));
    }
  ASSERT(0); return std::string();
}

// ######################################################################
void convertFromString(const std::string& str1, RescaleType& ftype)
{
  const std::string str = toLowerCase(str1);

  if      (str.compare("nointerp") == 0) ftype = RESCALE_SIMPLE_NOINTERP;
  else if (str.compare("bilinear") == 0) ftype = RESCALE_SIMPLE_BILINEAR;
  else if (str.compare("box") == 0)      ftype = RESCALE_FILTER_BOX     ;
  else if (str.compare("triangle") == 0) ftype = RESCALE_FILTER_TRIANGLE;
  else if (str.compare("bell") == 0)     ftype = RESCALE_FILTER_BELL    ;
  else if (str.compare("bspline") == 0)  ftype = RESCALE_FILTER_BSPLINE ;
  else if (str.compare("hermite") == 0)  ftype = RESCALE_FILTER_HERMITE ;
  else if (str.compare("lanczos3") == 0) ftype = RESCALE_FILTER_LANCZOS3;
  else if (str.compare("mitchell") == 0) ftype = RESCALE_FILTER_MITCHELL;
  else
    LFATAL("invalid rescale type '%s'", str1.c_str());
}

// ######################################################################
namespace
{
  // helper structs and functions for generic image rescaling

  struct HermiteFilter
  {
    static double support() { return 1.0; }

    static double filter(double t)
    {
      /* f(t) = 2|t|^3 - 3|t|^2 + 1, -1 <= t <= 1 */
      if (t < 0.0) t = -t;
      if (t < 1.0) return((2.0 * t - 3.0) * t * t + 1.0);
      return(0.0);
    }
  };

  struct BoxFilter
  {
    static double support() { return 0.5; }

    static double filter(double t)
    {
      if ((t > -0.5) && (t <= 0.5)) return(1.0);
      return(0.0);
    }
  };

  struct TriangleFilter
  {
    static double support() { return 1.0; }

    static double filter(double t)
    {
      if (t < 0.0) t = -t;
      if (t < 1.0) return(1.0 - t);
      return(0.0);
    }
  };

  struct BellFilter
  {
    static double support() { return 1.5; }

    static double filter(double t)          /* box (*) box (*) box */
    {
      if (t < 0) t = -t;
      if (t < .5) return(.75 - (t * t));
      if (t < 1.5) {
        t = (t - 1.5);
        return(.5 * (t * t));
      }
      return(0.0);
    }
  };

  struct BsplineFilter
  {
    static double support() { return 2.0; }

    static double filter(double t)
    {
      double tt;

      if (t < 0) t = -t;
      if (t < 1) {
        tt = t * t;
        return((.5 * tt * t) - tt + (2.0 / 3.0));
      } else if (t < 2) {
        t = 2 - t;
        return((1.0 / 6.0) * (t * t * t));
      }
      return(0.0);
    }
  };

  inline double sinc(double x)
  {
    x *= M_PI;
    if (x != 0) return(sin(x) / x);
    return(1.0);
  }

  struct Lanczos3Filter
  {
    static double support() { return 3.0; }

    static double filter(double t)
    {
      if (t < 0) t = -t;
      if (t < 3.0) return(sinc(t) * sinc(t/3.0));
      return(0.0);
    }
  };

  struct MitchellFilter
  {
    static double support() { return 2.0; }

    static double filter(double t)
    {
#define B       (1.0 / 3.0)
#define C       (1.0 / 3.0)
      double tt;

      tt = t * t;
      if (t < 0) t = -t;
      if (t < 1.0) {
        t = (((12.0 - 9.0 * B - 6.0 * C) * (t * tt))
             + ((-18.0 + 12.0 * B + 6.0 * C) * tt)
             + (6.0 - 2 * B));
        return(t / 6.0);
      } else if (t < 2.0) {
        t = (((-1.0 * B - 6.0 * C) * (t * tt))
             + ((6.0 * B + 30.0 * C) * tt)
             + ((-12.0 * B - 48.0 * C) * t)
             + (8.0 * B + 24 * C));
        return(t / 6.0);
      }
      return(0.0);
#undef B
#undef C
    }
  };

  /*
   *      image rescaling routine
   */

  struct CONTRIB {
    CONTRIB() : pixel(0), weight(0.0) {}

    CONTRIB(int p, double w) : pixel(p), weight(w) {}

    int     pixel;
    double  weight;
  };

  struct CLIST {
    std::vector<CONTRIB> p;

    void normalize()
    {
      double sum = 0.0;
      for (size_t i = 0; i < p.size(); ++i)
        sum += p[i].weight;
      for (size_t i = 0; i < p.size(); ++i)
        p[i].weight /= sum;
    }
  };



  /*
    calc_x_contrib()

    Calculates the filter weights for a single target column.
    contribX->p must be freed afterwards.

    Returns -1 if error, 0 otherwise.
  */
  template <class F>
  void calc_x_contrib(CLIST* contribX, /* Receiver of contrib info */
                      double xscale,   /* Horizontal zooming scale */
                      int dstwidth,    /* Target bitmap width */
                      int srcwidth,    /* Source bitmap width */
                      int i)           /* Pixel column in source bitmap being processed */
  {
    if (xscale < 1.0)
      {
        /* Shrinking image */
        const double width = F::support() / xscale;

        contribX->p.resize(0);
        contribX->p.reserve(int(width * 2 + 1));

        const double center = (double) i / xscale;
        const int left = int(ceil(center - width)) - 1;
        const int right = int(floor(center + width)) + 1;
        for (int j = left; j <= right; ++j)
          {
            double weight = i - (double) j * xscale;
            weight = F::filter(weight) * xscale;
            const int n = clampValue(j, 0, srcwidth - 1);
            if (weight != 0.0)
              contribX->p.push_back(CONTRIB(n, weight));
          }
        contribX->normalize();
      }
    else
      {
        /* Expanding image */
        contribX->p.resize(0);
        contribX->p.reserve(int(F::support() * 2 + 1));

        const double center = (double) i / xscale;
        const int left = int(ceil(center - F::support())) - 1;
        const int right = int(floor(center + F::support())) + 1;

        for (int j = left; j <= right; ++j)
          {
            double weight = (double) i / xscale - (double) j;
            weight = F::filter(weight);
            const int n = clampValue(j, 0, srcwidth - 1);
            if (weight != 0.0)
              contribX->p.push_back(CONTRIB(n, weight));
          }
        contribX->normalize();
      }
  } /* calc_x_contrib */


  /*
    genericRescale()

    Resizes bitmaps while resampling them.
  */
  template <class F, class T>
  Image<T> genericRescale(const Image<T>& src, const Dims& newdims)
  {
    typedef typename promote_trait<T, double>::TP TF;

    Image<T> dst(newdims, ZEROS);

    const int src_w = src.getWidth();
    const int src_h = src.getHeight();
    const int dst_w = dst.getWidth();
    const int dst_h = dst.getHeight();

    /* create intermediate column to hold horizontal dst column zoom */
    std::vector<TF> tmp(src_h);

    const double xscale = (double) dst_w / (double) src_w;

    /* Build y weights */
    /* pre-calculate filter contributions for a column */
    std::vector<CLIST> contribY(dst_h);

    const double yscale = (double) dst_h / (double) src_h;

    if (yscale < 1.0)
      {
        const double width = F::support() / yscale;

        for (int i = 0; i < dst_h; ++i)
          {
            contribY[i].p.resize(0);
            contribY[i].p.reserve(int(width * 2 + 1));

            const double center = (double) i / yscale;
            const int left = int(ceil(center - width)) - 1;
            const int right = int(floor(center + width)) + 1;
            for (int j = left; j <= right; ++j) {
              double weight = i - (double) j * yscale;
              weight = F::filter(weight) * yscale;
              const int n = clampValue(j, 0, src_h -1);
              if (weight != 0.0)
                contribY[i].p.push_back(CONTRIB(n, weight));
            }
            contribY[i].normalize();
          }
      }
    else
      {
        for (int i = 0; i < dst_h; ++i) {
          contribY[i].p.resize(0);
          contribY[i].p.reserve(int(F::support() * 2 + 1));

          const double center = (double) i / yscale;
          const int left = int(ceil(center - F::support())) - 1;
          const int right = int(floor(center + F::support())) + 1;
          for (int j = left; j <= right; ++j) {
            double weight = (double) i / yscale - (double) j;
            weight = F::filter(weight);
            const int n = clampValue(j, 0, src_h -1);
            if (weight != 0.0)
              contribY[i].p.push_back(CONTRIB(n, weight));
          }
          contribY[i].normalize();
        }
      }


    for (int xx = 0; xx < dst_w; ++xx)
      {
        CLIST   contribX;
        calc_x_contrib<F>(&contribX, xscale,
                          dst_w, src_w, xx);

        /* Apply horz filter to make dst column in tmp. */
        for (int k = 0; k < src_h; ++k)
          {
            TF weight = TF();
            bool bPelDelta = false;

            const T* const srcrowptr =
              src.getArrayPtr() + k * src_w;

            ASSERT((contribX.p[0].pixel >= 0)
                   && (contribX.p[0].pixel < src_w));

            const T pel = srcrowptr[contribX.p[0].pixel];

            for (size_t j = 0; j < contribX.p.size(); ++j)
              {
                ASSERT((contribX.p[j].pixel >= 0)
                       && (contribX.p[j].pixel < src_w));

                const T pel2 = srcrowptr[contribX.p[j].pixel];
                if (pel2 != pel)
                  bPelDelta = true;
                weight += TF(pel2) * contribX.p[j].weight;
              }
            tmp[k] = bPelDelta ? weight : TF(pel);
          } /* next row in temp column */

        /* The temp column has been built. Now stretch it
           vertically into dst column. */
        for (int i = 0; i < dst_h; ++i)
          {
            TF weight = TF();
            bool bPelDelta = false;
            const TF pel = tmp[contribY[i].p[0].pixel];

            for (size_t j = 0; j < contribY[i].p.size(); ++j)
              {
                const TF pel2 = tmp[contribY[i].p[j].pixel];
                if (pel2 != pel)
                  bPelDelta = true;
                weight += pel2 * contribY[i].p[j].weight;
              }

            ASSERT(xx >= 0 && xx < dst_w);
            ASSERT(i >= 0 && i < dst_h);

            dst.getArrayPtr()[xx + i * dst_w] =
              bPelDelta
              ? clamped_rounded_convert<T>(weight)
              : clamped_rounded_convert<T>(pel);
          } /* next dst row */
      } /* next dst column */

    return dst;
  } /* genericRescale */
}

// ######################################################################
template <class T>
Image<T> rescale(const Image<T>& src, const Dims& newdims,
                 RescaleType ftype)
{
  switch (ftype)
    {
    case RESCALE_SIMPLE_NOINTERP: return rescaleNI(src, newdims);
    case RESCALE_SIMPLE_BILINEAR: return rescaleBilinear(src, newdims);
    case RESCALE_FILTER_BOX     : return genericRescale<BoxFilter>(src, newdims);
    case RESCALE_FILTER_TRIANGLE: return genericRescale<TriangleFilter>(src, newdims);
    case RESCALE_FILTER_BELL    : return genericRescale<BellFilter>(src, newdims);
    case RESCALE_FILTER_BSPLINE : return genericRescale<BsplineFilter>(src, newdims);
    case RESCALE_FILTER_HERMITE : return genericRescale<HermiteFilter>(src, newdims);
    case RESCALE_FILTER_LANCZOS3: return genericRescale<Lanczos3Filter>(src, newdims);
    case RESCALE_FILTER_MITCHELL: return genericRescale<MitchellFilter>(src, newdims);
    default: LFATAL("invalid ftype '%c'", ftype);
    }
  ASSERT(0); return Image<T>();
}

// ######################################################################
template <class T>
Image<T> rescale(const Image<T>& src, const int width, const int height,
                 RescaleType ftype)
{
  return rescale(src, Dims(width, height), ftype);
}

// ######################################################################
template <class T>
Image<T> rescaleOpt(const Image<T>& src, const Dims& dims, const bool interp)
{
  if (interp) return rescaleBilinear(src, dims);
  else return rescaleNI(src, dims);
}

// ######################################################################
template <class T>
Image<T> rescaleOpt(const Image<T>& src, const int new_w, const int new_h,
                 const bool interp)
{
  if (interp) return rescaleBilinear(src, new_w, new_h);
  else return rescaleNI(src, new_w, new_h);
}

// ######################################################################
// rescale an image using fancy widgets like anti-aliasing, resampling
template <class T>
Image<PixRGB<T> > downscaleFancy(const Image<PixRGB<T> >& src,
                                 int width, int height, int weighting_slope,
                                 bool no_weight_black)

{
GVX_TRACE(__PRETTY_FUNCTION__);

  PixRGB<T> pix(0);
  Image<PixRGB<T> > buffer;
  Image<PixRGB<T> > out;
  Image<T>          bufferWeight;

  buffer.resize(width,height,true);
  out.resize(width,height,true);
  bufferWeight.resize(width,height,true);

  T scalex = (T)width  / (T)src.getWidth();
  T scaley = (T)height / (T)src.getHeight();
  T dx, dy, weight, xweight, yweight, xfrac, yfrac;
  typename Image<PixRGB<T> >::iterator bufelem;
  typename Image<T>::iterator bufw;
  dx = (T)0.0;
  dy = (T)0.0;

  for (int sy = (int)0; sy < (int)src.getHeight();
       sy++, dy += scaley)
  {
    // outer loop for Y axis
    yfrac = dy - (T)floor(dy);
    switch (weighting_slope)
    {
    case 5: yweight = yfrac < 0.5 ? 0 : 1;
      break;
    case 4: yweight = (T)(0.5 + 0.5*tanh(15*(yfrac - 0.5)));
      break;
    case 3: yweight = (T)(0.5 + 0.5*tanh(8*(yfrac - 0.5)));
      break;
    case 2: yweight = (T)(0.5 + 0.5*tanh(5*(yfrac - 0.5)));
      break;
    case 1: yweight = (T)(0.5 - 0.5*cos(yfrac*M_PI));
      break;
    case 0: yweight = yfrac;
      break;
    default : LERROR("illegal weighting slope");
      yweight = yfrac;
    }
    // inner loop for X axis
    dx = (T)0;
    for (int sx = (int)0; sx < (int)src.getWidth();
         sx++, dx += scalex)
    {
      //LINFO("X %d",sx);
      xfrac = dx - (T)floor(dx);
      switch (weighting_slope)
      {
      case 5: xweight = xfrac < 0.5 ? 0 : 1;
        break;
      case 4: xweight = (T)(0.5 + 0.5*tanh(15*(xfrac - 0.5)));
        break;
      case 3: xweight = (T)(0.5 + 0.5*tanh(8*(xfrac - 0.5)));
        break;
      case 2: xweight = (T)(0.5 + 0.5*tanh(5*(xfrac - 0.5)));
        break;
      case 1: xweight = (T)(0.5 - 0.5*cos(xfrac*M_PI));
        /*almost same as tanh(4*x)*/
        break;
      case 0: xweight = xfrac;
        break;
      default : LINFO("illegal weighting slope");
        xweight = xfrac;
      }
      //LINFO("XWEIGHT %f",xweight);
      int floordx = (int)floor((T)dx);
      int floordy = (int)floor((T)dy);
      const PixRGB<T> *in_sy_sx = &src.getVal(sx,sy);

      if (no_weight_black)
        if (in_sy_sx->red()   == 0 &&
            in_sy_sx->green() == 0 &&
            in_sy_sx->blue()  == 0)
          continue;

      bufelem = buffer.beginw()
        + buffer.getWidth()*floordy + floordx;

      bufw    = bufferWeight.beginw()
        + bufferWeight.getWidth()*floordy + floordx;

      ADD_RGB(bufelem, ((T)1.0-xweight)*((T)1.0-yweight), bufw, in_sy_sx);

      if (dx < width - 1)
      {
        bufelem++; bufw++;
        ADD_RGB(bufelem, xweight*((T)1.0-yweight), bufw, in_sy_sx);
      }

      if (dy < height - 1)
      {
        bufelem = buffer.beginw()
          + buffer.getWidth()*(floordy+1) + floordx;
        bufw    = bufferWeight.beginw()
          + bufferWeight.getWidth()*(floordy+1) + floordx;

        ADD_RGB(bufelem, ((T)1.0-xweight)*yweight, bufw, in_sy_sx);
        if (dx < width - 1) {
          bufelem++;
          bufw++;
          //bufelem = &(buf[BUFIDX(floordx+1,floordy+1)]);
          ADD_RGB(bufelem, xweight*yweight, bufw, in_sy_sx);
        }
      }
    }
    if (floorf(dy + scaley) > floorf(dy))
    {
      /* line finished -> write to out */
      int dsty = (int)floor(dy);
      for (int dstx = 0; dstx < width; dstx++)
      {
        weight = bufferWeight.getVal(dstx,dsty);
        if(weight != 0.0)
        {
          pix = buffer.getVal(dstx,dsty);
          pix /= weight;
          out.setVal(dstx,dsty,pix);
        }
        PixRGB<T> zero(0);
        buffer.setVal(dstx,dsty,zero);
        bufferWeight.setVal(dstx,dsty,0);
      }
    }
  }
  return out;
}


// ######################################################################
template <class T>
Image<T> downSize(const Image<T>& src, const Dims& dims,
                  const int filterWidth)
{
  return downSize(src, dims.w(), dims.h(), filterWidth);
}

// ######################################################################
template <class T>
Image<T> downSize(const Image<T>& src, const int new_w, const int new_h,
                  const int filterWidth)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (src.getWidth() == new_w && src.getHeight() == new_h) return src;

  ASSERT(src.getWidth() / new_w > 1 && src.getHeight() / new_h > 1);

  const int wdepth = int(0.5+log(double(src.getWidth() / new_w)) / M_LN2);
  const int hdepth = int(0.5+log(double(src.getHeight() / new_h)) / M_LN2);

  if (wdepth != hdepth)
    LFATAL("arrays must have same proportions");

  Image<T> result = src;

  for (int i = 0; i < wdepth; ++i)
    {
      result = decX(lowPassX(filterWidth, result));
      result = decY(lowPassY(filterWidth, result));
    }

  return result;
}

// ######################################################################
Image<float> downSizeClean(const Image<float>& src, const Dims& new_dims,
                           const int filterWidth)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (src.getDims() == new_dims) return src;

  ASSERT(new_dims.isNonEmpty());
  ASSERT(filterWidth >= 1);

  Image<float> result = src;

  while (result.getWidth() > new_dims.w() * 2 &&
         result.getHeight() > new_dims.h() * 2)
    {
      if (filterWidth == 1)
        {
          result = decX(result);
          result = decY(result);
        }
      else if (filterWidth == 2)
        {
          result = quickLocalAvg2x2(result);
        }
      else
        {
          result = decX(lowPassX(filterWidth, result));
          result = decY(lowPassY(filterWidth, result));
        }
    }

  return rescaleBilinear(result, new_dims);
}

// ######################################################################
template <class T>
Image<T> concatArray(const Image<T> arr[], const int nbarr, const int Nx,
                     const int destX, const int destY)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  int width, height;
  // check if we are going to reshape:
  if (destX <= 0 || destY <= 0)
    {
      width = arr[0].getWidth(); height = arr[0].getHeight();
    }
  else { width = destX; height = destY; }
  int tw = width * Nx;
  int th = height * (int)ceil(((double)nbarr) / ((double)Nx));
  Image<T> result(tw, th, NO_INIT);
  T zeros[width]; for (int i = 0; i < width; ++i) zeros[i] = T();

  int arrnb = 0;
  for (int j = 0; j < th; j += height)
    for (int i = 0; i < tw; i += width)
      {
        if (destX <= 0 || destY <= 0)  // no reshaping
          {
            if (arrnb < nbarr)  // copy image
              {
                ASSERT(arr[arrnb].getWidth() == width &&
                       arr[arrnb].getHeight() == height);
                for (int jj = 0; jj < height; jj ++)
                  safecopy(result.beginw() + (i + (j + jj) * tw),
                           arr[arrnb].begin() + (jj * width),
                           width);
              }
            else                // put blanks at the end
              for (int jj = 0; jj < height; jj ++)
                safecopy(result.beginw() + (i + (j + jj) * tw),
                         zeros,
                         width);
          }
        else  // reshape first
          {
            if (arrnb < nbarr)  // copy image
              {
                Image<T> tmp = rescaleBilinear(arr[arrnb], width, height);
                for (int jj = 0; jj < height; jj ++)
                  safecopy(result.beginw() + (i + (j + jj) * tw),
                           tmp.begin() + (jj * width),
                           width);
              }
            else                // put blanks at the end
              for (int jj = 0; jj < height; jj ++)
                safecopy(result.beginw() + (i + (j + jj) * tw),
                         zeros, width);
          }
        arrnb ++;
      }

  return result;
}

// ######################################################################
template <class T>
Image<T> decXY(const Image<T>& src,
               const int xfactor, const int yfactor_raw)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  const int yfactor = yfactor_raw >= 0 ? yfactor_raw : xfactor;

  // do not go smaller than 1x1:
  if (src.getWidth() <= 1 && src.getHeight() <= 1) return src;

  if (src.getWidth() == 1)  // only thinout vertic
    {
      int h2 = src.getHeight() / yfactor;

      Image<T> result(1, h2, NO_INIT);

      for (int j = 0; j < h2; ++j)
        result.setVal(j, src.getVal(j * yfactor));

      return result;
    }
  if (src.getHeight() == 1)
    {
      int w2 = src.getWidth() / xfactor;

      Image<T> result(w2, 1, NO_INIT);

      for (int i = 0; i < w2; ++i)
        result.setVal(i, src.getVal(i * xfactor));

      return result;
    }

  const int w2 = src.getWidth() / xfactor;
  const int h2 = src.getHeight() / yfactor;

  Image<T> result(w2, h2, NO_INIT);

  typename Image<T>::const_iterator sptr = src.begin();
  typename Image<T>::iterator dptr = result.beginw();
  const int skip = src.getWidth() % xfactor + src.getWidth() * (yfactor - 1);

  for (int j = 0; j < h2; ++j)
    {
      for (int i = 0; i < w2; ++i)
        {
          *dptr++ = *sptr;   // copy one pixel
          sptr += xfactor;   // skip some pixels
        }
      sptr += skip;          // skip to start of next line
    }

  return result;
}

// ######################################################################
template <class T>
Image<T> decX(const Image<T>& src, const int factor)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(factor > 0);

  if (src.getWidth() <= 1) return src;  // do not go smaller than 1 pixel wide

  const int h = src.getHeight();
  int w2 = src.getWidth() / factor;
  int skip = src.getWidth() % factor;
  if (w2 == 0)
    {
      w2 = 1;
      skip = src.getWidth() - factor;
    }

  Image<T> result(w2, h, NO_INIT);

  typename Image<T>::const_iterator sptr = src.begin();
  typename Image<T>::iterator dptr = result.beginw();

  for (int j = 0; j < h; ++j)
    {
      for (int i = 0; i < w2; ++i)
        {
          *dptr++ = *sptr;   // copy one point
          sptr += factor;    // skip a few points
        }
      sptr += skip;
    }

  return result;
}

// ######################################################################
template <class T>
Image<T> decY(const Image<T>& src, const int factor)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(factor > 0);

  if (src.getHeight() <= 1) return src;  // do not go smaller than 1 pixel high

  const int w = src.getWidth();
  int h2 = src.getHeight() / factor;
  if (h2 == 0) h2 = 1;

  Image<T> result(w, h2, NO_INIT);

  typename Image<T>::const_iterator sptr = src.begin();
  typename Image<T>::iterator dptr = result.beginw();
  int skip = w * factor;

  for (int j = 0; j < h2; ++j)
    {
      safecopy(dptr, sptr, w);
      dptr += w; sptr += skip;
    }

  return result;
}

// ######################################################################
template <class T>
Image<T> blurAndDecY(const Image<T>& src, const int factor)
{
  GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(factor > 0);

  if (src.getHeight() <= 1) return src;  // do not go smaller than 1 pixel high

  const int w = src.getWidth();
  const int h1 = src.getHeight();
  int h2 = h1 / factor;
  if (h2 == 0) h2 = 1;

  Image<T> result(w, h2, ZEROS);

  typename Image<T>::const_iterator const sptr = src.begin();
  typename Image<T>::iterator dptr = result.beginw();

  for (int j2 = 0; j2 < h2; ++j2)
    {
      int j1start = j2 * factor - (factor/2);
      int j1end = j1start + factor;
      if (j1start < 0) j1start = 0;
      if (j1end > h1) j1end = h1;

      const int nj = j1end-j1start;

      ASSERT(nj > 0);

      const double factor = 1.0/nj;

      for (int j1 = j1start; j1 < j1end; ++j1)
        for (int i = 0; i < w; ++i)
          dptr[i] += T(factor * sptr[i + j1*w]);

      dptr += w;
    }

  return result;
}

// ######################################################################
template <class T>
Image<T> intXY(const Image<T>& src, const bool dupli)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(src.initialized());   // works only if there is an image

  const int w = src.getWidth();
  const int h = src.getHeight();

  const int w2 = w * 2;
  const int h2 = h * 2;

  Image<T> result(w2, h2, NO_INIT);

  typename Image<T>::const_iterator sptr = src.begin();
  typename Image<T>::iterator dptr = result.beginw();
  if (dupli)
    for (int j = 0; j < h; ++j)
      {
        for (int i = 0; i < w; ++i)
          {
            *dptr++ = *sptr;     // copy one point
            *dptr++ = *sptr++;   // again
          }
        // duplicate line we just wrote:
        safecopy(dptr, dptr - w2, w2);
        dptr += w2;
      }
  else
    {
      T zeros[w2];
      for (int i = 0; i < w2; ++i) zeros[i] = T();  // needed??
      for (int j = 0; j < h; ++j)
        {
          for (int i = 0; i < w; ++i)
            {
              *dptr++ = *sptr++;        // copy one point
              *dptr++ = zeros[0];   // put a blank
            }
          safecopy(dptr, zeros, w2);
          dptr += w2;
        }
    }

  return result;
}

// ######################################################################
template <class T>
Image<T> intX(const Image<T>& src, const bool dupli)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(src.initialized());

  const int w2 = src.getWidth() * 2;

  Image<T> result(w2, src.getHeight(), NO_INIT);

  typename Image<T>::const_iterator sptr = src.begin();
  typename Image<T>::iterator dptr = result.beginw();
  T zero = T();

  const int siz = src.getSize();

  if (dupli)
    for (int i = 0; i < siz; ++i)
      {
        *dptr++ = *sptr;     // copy one point
        *dptr++ = *sptr++;   // again
      }
  else
    for (int i = 0; i < siz; ++i)
      {
        *dptr++ = *sptr++;  // copy one point
        *dptr++ = zero;     // put a zero
      }

  return result;
}

// ######################################################################
template <class T>
Image<T> intY(const Image<T>& src, const bool dupli)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(src.initialized());

  const int w = src.getWidth();
  const int h = src.getHeight();
  const int h2 = h * 2;

  Image<T> result(w, h2, NO_INIT);

  typename Image<T>::const_iterator sptr = src.begin();
  typename Image<T>::iterator dptr = result.beginw();

  if (dupli)
    for (int j = 0; j < h; ++j)
      {
        safecopy(dptr, sptr, w);
        dptr += w;
        safecopy(dptr, sptr, w);
        dptr += w;
        sptr += w;
      }
  else
    {
      T zeros[w];
      for (int i = 0; i < w; ++i) zeros[i] = T(); // needed??
      for (int j = 0; j < h; ++j)
        {
          safecopy(dptr, sptr, w);
          dptr += w;
          safecopy(dptr, zeros, w);
          dptr += w;
          sptr += w;
        }
    }

  return result;
}

// ######################################################################
template <class T>
Image<T> zoomXY(const Image<T>& src, int xzoom, int yzoom)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (yzoom < 0) yzoom = xzoom;

  if (xzoom == 0 || yzoom == 0) return Image<T>();
  if (xzoom == 1 && yzoom == 1) return src;

  Image<T> result(src.getWidth()*xzoom, src.getHeight()*yzoom, NO_INIT);

  typename Image<T>::iterator dptr = result.beginw();

  for (int y = 0; y < src.getHeight(); ++y)
    {
      for (int yz = 0; yz < yzoom; ++yz)
        {
          typename Image<T>::const_iterator sptr =
            src.begin() + y*src.getWidth();
          for (int x = 0; x < src.getWidth(); ++x)
            {
              for (int xz = 0; xz < xzoom; ++xz)
                {
                  *dptr = *sptr;
                  ++dptr;
                }
              ++sptr;
            }
        }
    }

  return result;
}

// ######################################################################
template <class T>
Image<T> rotate(const Image<T>& srcImg, const int x, const int y,
                const float ang)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // make sure the source image is valid
  ASSERT(srcImg.initialized());

  // create and clear the return image
  int w = srcImg.getWidth(), h = srcImg.getHeight();
  Image<T> retImg(w, h, ZEROS);

  // create temporary working image, put srcImg in the middle
  // put some padding (values are 0) around the image
  // so we won't need to check out of bounds when rotating
  int pad = int(ceil(sqrt(w * w + h * h)));
  int wR  = 2 * pad + w;
  int hR  = 2 * pad + h;
  Image<T> tempImg(wR, hR, ZEROS);
  inplacePaste(tempImg, srcImg, Point2D<int>(pad,pad));

  typename Image<T>::iterator rBPtr = retImg.beginw();
  float cAng = cos(ang), sAng = sin(ang);

  // fill in the return image with the appropriate values
  float xR = 0.0; float yR = 0.0;
  int dx = pad + x, dy = pad + y;
  for(int j = -y; j < h-y; j++)
    for(int i = -x; i < w-x; i++)
      {
        xR = dx + i*cAng + j*sAng;
        yR = dy - i*sAng + j*cAng;
        *rBPtr++ = T(tempImg.getValInterp(xR,yR));
      }

  return retImg;
}

// Include the explicit instantiations (color instantiations are now
// requested by using "T_or_RGB" for the template formal parameter name in
// the declarations in the .H file).
#include "inst/Image/ShapeOps.I"

template Image<double> quickLocalAvg2x2(const Image<double>&);
template Image<int> decX(const Image<int>&, int);
template Image<int> decY(const Image<int>&, int);
template Image<int> decXY(const Image<int>&, int, int);
template Image<double> intXY(const Image<double>&, bool);
template Image<PixH2SV2<float> > decXY(const Image<PixH2SV2<float> >&, int, int);
template Image<PixH2SV2<float> > rescaleBilinear(const Image<PixH2SV2<float> >& src, int, int);
template Image<geom::vec2f> rotate(const Image<geom::vec2f>&, int, int,float);
template Image<double> rotate(const Image<double>&, int, int,float);
template Image<uint16> zoomXY(const Image<uint16>&, int, int);
template Image<uint16> rescale(const Image<uint16>&, const Dims&, RescaleType);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !IMAGE_SHAPEOPS_C_DEFINED

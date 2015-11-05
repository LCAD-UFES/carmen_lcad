/*!@file Image/LowPass.C low-pass filtering and smoothing functions */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/LowPass.C $
// $Id: LowPass.C 12962 2010-03-06 02:13:53Z irock $
//

#ifndef IMAGE_LOWPASS_C_DEFINED
#define IMAGE_LOWPASS_C_DEFINED

#include "Image/LowPass.H"

#include "Image/Convolutions.H"
#include "Image/Image.H"
#include "Image/Kernels.H"
#include "Image/Pixels.H"
#include "rutz/trace.h"

namespace
{
  // helper function for median3x()/median3y()/median3()
  template <class T>
  T median3scalar(T a, T b, T c)
  {
    if (b < a) { T t = b; b = a; a = t; }
    if (c < b) { T t = c; c = b; b = t; }
    if (b < a) { T t = b; b = a; a = t; }

    return b;
  }

  // helper function for median3x()/median3y()/median3()
  template <class T>
  PixRGB<T> median3scalar(PixRGB<T> a, PixRGB<T> b, PixRGB<T> c)
  {
    return PixRGB<T>(median3scalar(a.p[0], b.p[0], c.p[0]),
                     median3scalar(a.p[1], b.p[1], c.p[1]),
                     median3scalar(a.p[2], b.p[2], c.p[2]));
  }

  Image<PixRGB<float> > lowPass3x_rgb(const Image<PixRGB<float> >& src)
  {
    const int w = src.getWidth(), h = src.getHeight();
    if (w < 2) return src; // nothing to smooth
    Image<PixRGB<float> > result(w, h, NO_INIT);
    Image<PixRGB<float> >::const_iterator sptr = src.begin();
    Image<PixRGB<float> >::iterator dptr = result.beginw();

    // *** unfolded version (all particular cases treated) for max speed.
    // do not use access overload since can access the C array directly.
    // the bet is: float computations are faster than int (true for most
    // float copros), and int2float converts fast->do not have to divide
    // by the coeff sum since we directly use float coeffs.
    // notations: in () is the position of dest ptr, and ^ is src ptr
    // ########## horizontal pass
    for (int j = 0; j < h; j ++)
      {
        // leftmost point  [ (2^) 1 ] / 3
        dptr->setRed(sptr[0].red() * (2.0F / 3.0F)
          + sptr[1].red() * (1.0F / 3.0F));
        dptr->setGreen(sptr[0].green() * (2.0F / 3.0F)
          + sptr[1].green() * (1.0F / 3.0F));
        dptr->setBlue(sptr[0].blue() * (2.0F / 3.0F)
          + sptr[1].blue() * (1.0F / 3.0F));
        ++dptr;

        // rest of the line except last point  [ 1^ (2) 1 ] / 4
        for (int i = 0; i < w - 2; i ++)
          {
            dptr->setRed((sptr[0].red() + sptr[2].red())
              * 0.25F + sptr[1].red() * 0.5F);
            dptr->setGreen((sptr[0].green() + sptr[2].green())
              * 0.25F + sptr[1].green() * 0.5F);
            dptr->setBlue((sptr[0].blue() + sptr[2].blue())
              * 0.25F + sptr[1].blue() * 0.5F);
            ++dptr;
            ++sptr;
          }

        // last point [ 1^ (2) ] / 3
        dptr->setRed(sptr[0].red() * (1.0F / 3.0f)
          + sptr[1].red() * (2.0F / 3.0F));
        dptr->setGreen(sptr[0].green() * (1.0F / 3.0f)
          + sptr[1].green() * (2.0F / 3.0F));
        dptr->setBlue(sptr[0].blue() * (1.0F / 3.0f)
          + sptr[1].blue() * (2.0F / 3.0F));
        ++dptr;
        sptr += 2;  // sptr back to same position as dptr
      }
    return result;
  }

  Image<PixRGB<float> > lowPass3y_rgb(const Image<PixRGB<float> >& src)
  {
    const int w = src.getWidth(), h = src.getHeight();
    if (h < 2) return src; // nothing to smooth
    Image<PixRGB<float> > result(w, h, NO_INIT);
    Image<PixRGB<float> >::const_iterator sptr = src.begin();
    Image<PixRGB<float> >::iterator dptr = result.beginw();
    const int w2 = w * 2; // speedup

    // ########## vertical pass  (even though we scan horiz for speedup)
    // topmost points  ( [ (2^) 1 ] / 3 )^T
    for (int i = 0; i < w; i ++)
      {
        //*dptr = sptr[0] * (2.0F / 3.0F) + sptr[w] * (1.0F / 3.0F);
        dptr->setRed(sptr[0].red() * (2.0F / 3.0F)
          + sptr[w].red() * (1.0F / 3.0F));
        dptr->setGreen(sptr[0].green() * (2.0F / 3.0F)
          + sptr[w].green() * (1.0F / 3.0F));
        dptr->setBlue(sptr[0].blue() * (2.0F / 3.0F)
          + sptr[w].blue() * (1.0F / 3.0F));
        ++dptr;
        ++sptr;
      }
    sptr -= w;  // go back to top-left

    // rest of the column except last point ( [ 1^ (2) 1 ] / 4.0 )^T
    for (int j = 0; j < h - 2; j ++)
      for (int i = 0; i < w; i ++)
        {
          //*dptr = (sptr[0] + sptr[w2]) * 0.25F + sptr[w] * 0.5F;
          dptr->setRed((sptr[0].red() + sptr[w2].red())
            * 0.25F + sptr[w].red() * 0.5F);
          dptr->setGreen((sptr[0].green() + sptr[w2].green())
            * 0.25F + sptr[w].green() * 0.5F);
          dptr->setBlue((sptr[0].blue() + sptr[w2].blue())
            * 0.25F + sptr[w].blue() * 0.5F);
          ++dptr;
          ++sptr;
        }

    // last points ( [ 1^ (2) ] / 3 )^T
    for (int i = 0; i < w; i ++)
      {
        //*dptr = sptr[0] * (1.0F / 3.0F) + sptr[w] * (2.0F / 3.0F);
        dptr->setRed(sptr[0].red() * (1.0F / 3.0F) + sptr[w].red()
          * (2.0F / 3.0F));
        dptr->setGreen(sptr[0].green() * (1.0F / 3.0F) + sptr[w].green()
          * (2.0F / 3.0F));
        dptr->setBlue(sptr[0].blue() * (1.0F / 3.0F) + sptr[w].blue()
          * (2.0F / 3.0F));
        ++dptr;
        ++sptr;
      }

    // finished. here, sptr is one line before the end of array.
    return result;
  }
}

// ######################################################################
template <>
Image<promote_trait<PixRGB<float>, float>::TP>
lowPass3x(const Image<PixRGB<float> >& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return lowPass3x_rgb(src);
}

// ######################################################################
template <class T>
Image<typename promote_trait<T, float>::TP>
lowPass3x(const Image<T>& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const int w = src.getWidth(), h = src.getHeight();
  // promote the source image to float if necessary, so that we do the
  // promotion only once for all, rather than many times as we access
  // the pixels of the image; if no promotion is necessary, "source"
  // will just point to the original data of "src" through the
  // copy-on-write/ref-counting behavior of Image:
  typedef typename promote_trait<T, float>::TP TF;
  const Image<TF> source = src;
  if (w < 2) return source; // nothing to smooth
  Image<TF> result(w, h, NO_INIT);
  typename Image<TF>::const_iterator sptr = source.begin();
  typename Image<TF>::iterator dptr = result.beginw();

  // *** unfolded version (all particular cases treated) for max speed.
  // do not use access overload since can access the C array directly.
  // the bet is: float computations are faster than int (true for most
  // float copros), and int2float converts fast->do not have to divide
  // by the coeff sum since we directly use float coeffs.
  // notations: in () is the position of dest ptr, and ^ is src ptr
  // ########## horizontal pass
  for (int j = 0; j < h; j ++)
    {
      // leftmost point  [ (2^) 1 ] / 3
      *dptr++ = sptr[0] * (2.0F / 3.0F) + sptr[1] * (1.0F / 3.0F);

      // rest of the line except last point  [ 1^ (2) 1 ] / 4
      for (int i = 0; i < w - 2; i ++)
        {
          *dptr++ = (sptr[0] + sptr[2]) * 0.25F + sptr[1] * 0.5F;
          sptr++;
        }

      // last point [ 1^ (2) ] / 3
      *dptr++ = sptr[0] * (1.0F / 3.0F) + sptr[1] * (2.0F / 3.0F);
      sptr += 2;  // sptr back to same position as dptr
    }
  return result;
}

// ######################################################################
template <>
Image<promote_trait<PixRGB<float>, float>::TP>
lowPass3y(const Image<PixRGB<float> >& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return lowPass3y_rgb(src);
}

// ######################################################################
template <class T>
Image<typename promote_trait<T, float>::TP>
lowPass3y(const Image<T>& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const int w = src.getWidth(), h = src.getHeight();
  // promote the source image to float if necessary, so that we do the
  // promotion only once for all, rather than many times as we access
  // the pixels of the image; if no promotion is necessary, "source"
  // will just point to the original data of "src" through the
  // copy-on-write/ref-counting behavior of Image:
  typedef typename promote_trait<T, float>::TP TF;
  const Image<TF> source = src;
  if (h < 2) return source; // nothing to smooth
  Image<TF> result(w, h, NO_INIT);
  typename Image<TF>::const_iterator sptr = source.begin();
  typename Image<TF>::iterator dptr = result.beginw();
  const int w2 = w * 2; // speedup

  // ########## vertical pass  (even though we scan horiz for speedup)
  // topmost points  ( [ (2^) 1 ] / 3 )^T
  for (int i = 0; i < w; i ++)
    {
      *dptr++ = sptr[0] * (2.0F / 3.0F) + sptr[w] * (1.0F / 3.0F);
      sptr++;
    }
  sptr -= w;  // go back to top-left

  // rest of the column except last point ( [ 1^ (2) 1 ] / 4.0 )^T
  for (int j = 0; j < h - 2; j ++)
    for (int i = 0; i < w; i ++)
      {
        *dptr++ = (sptr[0] + sptr[w2]) * 0.25F + sptr[w] * 0.5F;
        sptr++;
      }

  // last points ( [ 1^ (2) ] / 3 )^T
  for (int i = 0; i < w; i ++)
    {
      *dptr++ = sptr[0] * (1.0F / 3.0F) + sptr[w] * (2.0F / 3.0F);
      sptr++;
    }

  // finished. here, sptr is one line before the end of array.
  return result;
}

// ######################################################################
template <class T> // Anderson's separable kernel: 1/16 * [1 4 6 4 1]
Image<typename promote_trait<T, float>::TP>
lowPass5x(const Image<T>& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const int w = src.getWidth(), h = src.getHeight();
  // promote the source image to float if necessary, so that we do the
  // promotion only once for all, rather than many times as we access
  // the pixels of the image; if no promotion is necessary, "source"
  // will just point to the original data of "src" through the
  // copy-on-write/ref-counting behavior of Image:
  typedef typename promote_trait<T, float>::TP TF;
  const Image<TF> source = src;
  if (w < 2) return source; // nothing to smooth
  Image<TF> result(w, h, NO_INIT);
  typename Image<TF>::const_iterator sptr = source.begin();
  typename Image<TF>::iterator dptr = result.beginw();

  if (w == 2) //////////////////////////////////////////////////
    for (int j = 0; j < h; j ++)
      {
        // leftmost point  [ (6^) 4 ] / 10
        *dptr++ = sptr[0] * (6.0F / 10.0F) + sptr[1] * (4.0F / 10.0F);

        // rightmost point  [ 4^ (6) ] / 10
        *dptr++ = sptr[0] * (4.0F / 10.0F) + sptr[1] * (6.0F / 10.0F);

        sptr += 2;  // sptr back to same position as dptr
      }
  else if (w == 3) //////////////////////////////////////////////////
    for (int j = 0; j < h; j ++)
      {
        // leftmost point  [ (6^) 4 1 ] / 11
        *dptr++ = sptr[0] * (6.0F / 11.0F) +
          sptr[1] * (4.0F / 11.0F) +
          sptr[2] * (1.0F / 11.0F);

        // middle point    [ 4^ (6) 4 ] / 14
        *dptr++ = (sptr[0] + sptr[2]) * (4.0F / 14.0F) +
          sptr[1] * (6.0F / 14.0F);

        // rightmost point  [ 1^ 4 (6) ] / 11
        *dptr++ = sptr[0] * (1.0F / 11.0F) +
          sptr[1] * (4.0F / 11.0F) +
          sptr[2] * (6.0F / 11.0F);

        sptr += 3;  // sptr back to same position as dptr
      }
  else  ////////////////////////////// general case for width() >= 4
        // *** unfolded version (all particular cases treated) for
        // max speed.  do not use access overloading since can
        // access the C array directly.  the bet is: float
        // computations are faster than int (true for most float
        // copros), and int2float convert is fast -> do not have to
        // divide by the coeff sum since we directly use float
        // coeffs.
        // notations: in () is the position of dest ptr, and ^ is src ptr
        // ########## horizontal pass
    for (int j = 0; j < h; j ++)
      {
        // leftmost point  [ (6^) 4 1 ] / 11
        *dptr++ = sptr[0] * (6.0F / 11.0F) +
          sptr[1] * (4.0F / 11.0F) +
          sptr[2] * (1.0F / 11.0F);

        // second point    [ 4^ (6) 4 1 ] / 15
        *dptr++ = (sptr[0] + sptr[2]) * (4.0F / 15.0F) +
          sptr[1] * (6.0F / 15.0F) +
          sptr[3] * (1.0F / 15.0F);

        // rest of the line except last 2 points  [ 1^ 4 (6) 4 1 ] / 16.0
        for (int i = 0; i < w - 4; i ++)
          {
            *dptr++ = (sptr[0] + sptr[4]) * (1.0F / 16.0F) +
              (sptr[1] + sptr[3]) * (4.0F / 16.0F) +
              sptr[2]  * (6.0F / 16.0F);
            sptr++;
          }

        // before last point [ 1^ 4 (6) 4 ] / 15
        *dptr++ = sptr[0] * (1.0F / 15.0F) +
          (sptr[1] + sptr[3]) * (4.0F / 15.0F) +
          sptr[2] * (6.0F / 15.0F);
        sptr++;

        // last point [ 1^ 4 (6) ] / 11
        *dptr++ = sptr[0] * (1.0F / 11.0F) +
          sptr[1] * (4.0F / 11.0F) +
          sptr[2] * (6.0F / 11.0F);
        sptr += 3;  // sptr back to same position as dptr
      }
  return result;
}

// ######################################################################
template <class T> // Anderson's separable kernel: 1/16 * [1 4 6 4 1]
Image<typename promote_trait<T, float>::TP>
lowPass5y(const Image<T>& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const int w = src.getWidth(), h = src.getHeight();
  // promote the source image to float if necessary, so that we do the
  // promotion only once for all, rather than many times as we access
  // the pixels of the image; if no promotion is necessary, "source"
  // will just point to the original data of "src" through the
  // copy-on-write/ref-counting behavior of Image:
  typedef typename promote_trait<T, float>::TP TF;
  const Image<TF> source = src;
  if (h < 2) return source; // nothing to smooth
  Image<TF> result(w, h, NO_INIT);
  typename Image<TF>::const_iterator sptr = source.begin();
  typename Image<TF>::iterator dptr = result.beginw();

  // ########## vertical pass  (even though we scan horiz for speedup)
  const int w2 = w * 2, w3 = w * 3, w4 = w * 4; // speedup

  if (h == 2) //////////////////////////////////////////////////
    {
      // topmost points  ( [ (6^) 4 ] / 10 )^T
      for (int i = 0; i < w; i ++)
        {
          *dptr++ = sptr[0] * (6.0F / 10.0F) +
            sptr[w] * (4.0F / 10.0F);
          sptr++;
        }
      sptr -= w;  // go back to top-left

      // bottommost points  ( [ 4^ (6) ] / 10 )^T
      for (int i = 0; i < w; i ++)
        {
          *dptr++ = sptr[0] * (4.0F / 10.0F) +
            sptr[w] * (6.0F / 10.0F);
          sptr++;
        }
    }
  else if (h == 3) //////////////////////////////////////////////////
    {
      // topmost points  ( [ (6^) 4 1 ] / 11 )^T
      for (int i = 0; i < w; i ++)
        {
          *dptr++ = sptr[ 0] * (6.0F / 11.0F) +
            sptr[ w] * (4.0F / 11.0F) +
            sptr[w2] * (1.0F / 11.0F);
          sptr++;
        }
      sptr -= w;  // go back to top-left

      // middle points  ( [ 4^ (6) 4 ] / 14 )^T
      for (int i = 0; i < w; i ++)
        {
          *dptr++ = (sptr[ 0] + sptr[w2]) * (4.0F / 14.0F) +
            sptr[ w] * (6.0F / 14.0F);
          sptr++;
        }
      sptr -= w;  // go back to top-left

      // bottommost points  ( [ 1^ 4 (6) ] / 11 )^T
      for (int i = 0; i < w; i ++)
        {
          *dptr++ = sptr[ 0] * (1.0F / 11.0F) +
            sptr[ w] * (4.0F / 11.0F) +
            sptr[w2] * (6.0F / 11.0F);
          sptr++;
        }
    }
  else  ///////////////////////////////// general case for height >= 4
    {
      // topmost points  ( [ (6^) 4 1 ] / 11 )^T
      for (int i = 0; i < w; i ++)
        {
          *dptr++ = sptr[ 0] * (6.0F / 11.0F) +
            sptr[ w] * (4.0F / 11.0F) +
            sptr[w2] * (1.0F / 11.0F);
          sptr++;
        }
      sptr -= w;  // go back to top-left

      // second topmost points  ( [ 4^ (6) 4 1 ] / 15 )^T
      for (int i = 0; i < w; i ++)
        {
          *dptr++ = (sptr[ 0] + sptr[w2]) * (4.0F / 15.0F) +
            sptr[ w] * (6.0F / 15.0F) +
            sptr[w3] * (1.0F / 15.0F);
          sptr++;
        }
      sptr -= w;  // go back to top-left

      // rest of the column except last 2 points ( [ 1^ 4 (6) 4 1 ] / 16 )T
      for (int j = 0; j < h - 4; j ++)
        for (int i = 0; i < w; i ++)
          {
            *dptr++ = (sptr[ 0] + sptr[w4]) * (1.0F / 16.0F) +
              (sptr[ w] + sptr[w3]) * (4.0F / 16.0F) +
              sptr[w2]  * (6.0F / 16.0F);
            sptr++;
          }

      // before last points ( [ 1^ 4 (6) 4 ] / 15 )T
      for (int i = 0; i < w; i ++)
        {
          *dptr++ = sptr[ 0] * (1.0F / 15.0F) +
            (sptr[ w] + sptr[w3]) * (4.0F / 15.0F) +
            sptr[w2] * (6.0F / 15.0F);
          sptr++;
        }

      // last points ( [ 1^ 4 (6) ] / 11 )T
      for (int i = 0; i < w; i ++)
        {
          *dptr++ = sptr[ 0] * (1.0F / 11.0F) +
            sptr[ w] * (4.0F / 11.0F) +
            sptr[w2] * (6.0F / 11.0F);
          sptr++;
        }
      // finished. here, sptr is two lines before the end of array.
    }
  return result;
}

// ######################################################################
template <class T> // Anderson's separable kernel: 1/16 * [1 4 6 4 1]
Image<typename promote_trait<T, float>::TP>
lowPass5xDecX(const Image<T>& src, const int factor)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(factor > 0);

  // if factor == 1 then we don't decimate:
  if (factor == 1) return lowPass5x(src);

  // now factor is guaranteed to be >= 2
  const int w = src.getWidth(), h = src.getHeight();
  int w2 = w / factor;
  if (w2 == 0) w2 = 1; // do not go smaller than 1 pixel

  // promote the source image to float if necessary, so that we do the
  // promotion only once for all, rather than many times as we access
  // the pixels of the image; if no promotion is necessary, "source"
  // will just point to the original data of "src" through the
  // copy-on-write/ref-counting behavior of Image:
  typedef typename promote_trait<T, float>::TP TF;
  const Image<TF> source = src;
  if (w < 2) return source; // nothing to smooth

  Image<TF> result(w2, h, NO_INIT);
  typename Image<TF>::const_iterator sptr = source.begin();
  typename Image<TF>::iterator dptr = result.beginw();

  if (w == 2) //////////////////////////////////////////////////
    for (int j = 0; j < h; j ++)
      {
        // leftmost point  [ (6^) 4 ] / 10
        *dptr++ = sptr[0] * (6.0F / 10.0F) + sptr[1] * (4.0F / 10.0F);

        sptr += 2;  // sptr back to same position as dptr
      }
  else if (w == 3) //////////////////////////////////////////////////
    for (int j = 0; j < h; j ++)
      {
        // need the left most point in any case
        // leftmost point  [ (6^) 4 1 ] / 11
        *dptr++ = sptr[0] * (6.0F / 11.0F) +
          sptr[1] * (4.0F / 11.0F) +
          sptr[2] * (1.0F / 11.0F);

        // since factor >= 2 we don't need any other point.

        sptr += 3;  // sptr back to same position as dptr
      }
  else  ////////////////////////////// general case for width() >= 4
        // *** unfolded version (all particular cases treated) for
        // max speed.  do not use access overloading since can
        // access the C array directly.  the bet is: float
        // computations are faster than int (true for most float
        // copros), and int2float convert is fast -> do not have to
        // divide by the coeff sum since we directly use float
        // coeffs.
        // notations: in () is the position of dest ptr, and ^ is src ptr
        // ########## horizontal pass
    for (int j = 0; j < h; j ++)
      {
        int i1 = 0, i2 = 0;
        typename Image<TF>::const_iterator sptr2 = sptr;


        // leftmost point  [ (6^) 4 1 ] / 11
        *dptr++ = sptr2[0] * (6.0F / 11.0F) +
          sptr2[1] * (4.0F / 11.0F) +
          sptr2[2] * (1.0F / 11.0F);
        ++i2;
        i1 += factor;

        // skip second point since factor >= 2:
        sptr2 += (factor-2);

        // rest of the line except last 2 points  [ 1^ 4 (6) 4 1 ] / 16.0
        while ((i1 < (w-2)) && (i2 < w2))
          {
            *dptr++ = (sptr2[0] + sptr2[4]) * (1.0F / 16.0F) +
              (sptr2[1] + sptr2[3]) * (4.0F / 16.0F) +
              sptr2[2]  * (6.0F / 16.0F);
            i1 += factor; sptr2 += factor;
            ++i2;
          }

        // need special case for second to last point?
        if ((i2 < w2) && (i1 == (w-2)))
          {
            sptr2 = sptr + w - 4;
            // before last point [ 1^ 4 (6) 4 ] / 15
            *dptr++ = sptr2[0] * (1.0F / 15.0F) +
              (sptr2[1] + sptr2[3]) * (4.0F / 15.0F) +
              sptr2[2] * (6.0F / 15.0F);
            i1 += factor;
            ++i2;
          }

        // need special case for last point?
        if ((i2 < w2) && (i1 == (w-1)))
          {
            sptr2 = sptr + w - 3;
            // last point [ 1^ 4 (6) ] / 11
            *dptr++ = sptr2[0] * (1.0F / 11.0F) +
              sptr2[1] * (4.0F / 11.0F) +
              sptr2[2] * (6.0F / 11.0F);
            ++i2;
          }
        sptr += w;
      }
  return result;
}

// ######################################################################
template <class T> // Anderson's separable kernel: 1/16 * [1 4 6 4 1]
Image<typename promote_trait<T, float>::TP>
lowPass5yDecY(const Image<T>& src, const int factor)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(factor > 0);

  // if factor == 1 then we don't decimate:
  if (factor == 1) return lowPass5y(src);

  // now factor is guaranteed to be >= 2
  const int w = src.getWidth(), h = src.getHeight();
  int h2 = h / factor;
  if (h2 == 0) h2 = 1; // do not go smaller than 1 pixel

  // promote the source image to float if necessary, so that we do the
  // promotion only once for all, rather than many times as we access
  // the pixels of the image; if no promotion is necessary, "source"
  // will just point to the original data of "src" through the
  // copy-on-write/ref-counting behavior of Image:
  typedef typename promote_trait<T, float>::TP TF;
  const Image<TF> source = src;
  if (h < 2) return source; // nothing to smooth

  Image<TF> result(w, h2, NO_INIT);
  typename Image<TF>::const_iterator sptr = source.begin();
  typename Image<TF>::iterator dptr = result.beginw();

  // ########## vertical pass  (even though we scan horiz for speedup)
  const int w2 = w * 2, w3 = w * 3, w4 = w * 4; // speedup

  if (h == 2) //////////////////////////////////////////////////
    {
      // topmost points  ( [ (6^) 4 ] / 10 )^T
      for (int i = 0; i < w; i ++)
        {
          *dptr++ = sptr[0] * (6.0F / 10.0F) + sptr[w] * (4.0F / 10.0F);
          sptr++;
        }
      sptr -= w;  // go back to top-left
    }
  else if (h == 3) //////////////////////////////////////////////////
    {
      // topmost points  ( [ (6^) 4 1 ] / 11 )^T
      for (int i = 0; i < w; i ++)
        {
          *dptr++ = sptr[ 0] * (6.0F / 11.0F) +
            sptr[ w] * (4.0F / 11.0F) +
            sptr[w2] * (1.0F / 11.0F);
          sptr++;
        }
      sptr -= w;  // go back to top-left

      // since factor >= 2 we don't need any other point.
    }
  else  ///////////////////////////////// general case for height >= 4
    {
      int i1 = 0, i2 = 0;
      const int skip = (factor - 1) * w;

      // topmost points  ( [ (6^) 4 1 ] / 11 )^T
      for (int i = 0; i < w; i ++)
        {
          *dptr++ = sptr[ 0] * (6.0F / 11.0F) +
            sptr[ w] * (4.0F / 11.0F) +
            sptr[w2] * (1.0F / 11.0F);
          sptr++;
        }
      sptr -= w;  // go back to top-left
      ++i2;
      i1 += factor;

      // second point skipped since factor >= 2
      sptr += (skip - w);

      // rest of the column except last 2 points ( [ 1^ 4 (6) 4 1 ] / 16 )T
      while((i1 < (h-2)) && (i2 < h2))
        {
          for (int i = 0; i < w; i ++)
            {
              *dptr++ = (sptr[ 0] + sptr[w4]) * (1.0F / 16.0F) +
                (sptr[ w] + sptr[w3]) * (4.0F / 16.0F) +
                sptr[w2]  * (6.0F / 16.0F);
              sptr++;
            }
          sptr += skip;
          i1 += factor;
          ++ i2;
        }

      // need special case for second to last point?
      if ((i2 < h2) && (i1 == (h-2)))
        {
          sptr = source.end() - w4;
          // before last points ( [ 1^ 4 (6) 4 ] / 15 )T
          for (int i = 0; i < w; i ++)
            {
              *dptr++ = sptr[ 0] * (1.0F / 15.0F) +
                (sptr[ w] + sptr[w3]) * (4.0F / 15.0F) +
                sptr[w2] * (6.0F / 15.0F);
              sptr++;
            }
          i1 += factor;
          ++i2;
        }

      // need special case for last point?
      if ((i2 < h2) && (i1 == (h-1)))
        {
          sptr = source.end() - w3;
          // last points ( [ 1^ 4 (6) ] / 11 )T
          for (int i = 0; i < w; i ++)
            {
              *dptr++ = sptr[ 0] * (1.0F / 11.0F) +
                sptr[ w] * (4.0F / 11.0F) +
                sptr[w2] * (6.0F / 11.0F);
              sptr++;
            }
        }
    }
  return result;
}

// ######################################################################
template <class T>
Image<typename promote_trait<T, float>::TP>
lowPass9x(const Image<T>& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const int w = src.getWidth(), h = src.getHeight();
  // promote the source image to float if necessary, so that we do the
  // promotion only once for all, rather than many times as we access
  // the pixels of the image; if no promotion is necessary, "source"
  // will just point to the original data of "src" through the
  // copy-on-write/ref-counting behavior of Image:
  typedef typename promote_trait<T, float>::TP TF;
  const Image<TF> source = src;
  if (w < 2) return source; // nothing to smooth
  Image<TF> result(w, h, NO_INIT);
  typename Image<TF>::const_iterator sptr = source.begin();
  typename Image<TF>::iterator dptr = result.beginw();

  if (w < 9)  // use inefficient implementation for small images
    {
      float kernel[9] = { 1.0F / 256.0F, 8.0F / 256.0F, 28.0F / 256.0F,
                          56.0F / 256.0F, 70.0F / 256.0F, 56.0F / 256.0F,
                          28.0F / 256.0F, 8.0F / 256.0F, 1.0F / 256.0F };
      return sepFilter(src, kernel, NULL, 9, 0,
                       CONV_BOUNDARY_CLEAN);
    }

  // boundary conditions: truncated filter
  for (int j = 0; j < h; j ++)
    {
      // leftmost points
      *dptr++ = sptr[0] * (70.0F / 163.0F) +
        sptr[1] * (56.0F / 163.0F) +
        sptr[2] * (28.0F / 163.0F) +
        sptr[3] * ( 8.0F / 163.0F) +
        sptr[4] * ( 1.0F / 163.0F);
      *dptr++ = (sptr[0] + sptr[2]) * (56.0F / 219.0F) +
        sptr[1] * (70.0F / 219.0F) +
        sptr[3] * (28.0F / 219.0F) +
        sptr[4] * ( 8.0F / 219.0F) +
        sptr[5] * ( 1.0F / 219.0F);
      *dptr++ = (sptr[0] + sptr[4]) * (28.0F / 247.0F) +
        (sptr[1] + sptr[3]) * (56.0F / 247.0F) +
        sptr[2] * (70.0F / 247.0F) +
        sptr[5] * ( 8.0F / 247.0F) +
        sptr[6] * ( 1.0F / 247.0F);
      *dptr++ = (sptr[0] + sptr[6]) * ( 8.0F / 255.0F) +
        (sptr[1] + sptr[5]) * (28.0F / 255.0F) +
        (sptr[2] + sptr[4]) * (56.0F / 255.0F) +
        sptr[3] * (70.0F / 255.0F) +
        sptr[7] * ( 1.0F / 255.0F);

      // far from the borders
      for (int i = 0; i < w - 8; i ++)
        {
          *dptr++ = (sptr[0] + sptr[8]) * ( 1.0F / 256.0F) +
            (sptr[1] + sptr[7]) * ( 8.0F / 256.0F) +
            (sptr[2] + sptr[6]) * (28.0F / 256.0F) +
            (sptr[3] + sptr[5]) * (56.0F / 256.0F) +
            sptr[4] * (70.0F / 256.0F);
          sptr ++;
        }

      // rightmost points
      *dptr++ = sptr[0] * ( 1.0F / 255.0F) +
        (sptr[1] + sptr[7]) * ( 8.0F / 255.0F) +
        (sptr[2] + sptr[6]) * (28.0F / 255.0F) +
        (sptr[3] + sptr[5]) * (56.0F / 255.0F) +
        sptr[4] * (70.0F / 255.0F);
      sptr ++;
      *dptr++ = sptr[0] * ( 1.0F / 247.0F) +
        sptr[1] * ( 8.0F / 247.0F) +
        (sptr[2] + sptr[6]) * (28.0F / 247.0F) +
        (sptr[3] + sptr[5]) * (56.0F / 247.0F) +
        sptr[4] * (70.0F / 247.0F);
      sptr ++;
      *dptr++ = sptr[0] * ( 1.0F / 219.0F) +
        sptr[1] * ( 8.0F / 219.0F) +
        sptr[2] * (28.0F / 219.0F) +
        (sptr[3] + sptr[5]) * (56.0F / 219.0F) +
        sptr[4] * (70.0F / 219.0F);
      sptr ++;
      *dptr++ = sptr[0] * ( 1.0F / 163.0F) +
        sptr[1] * ( 8.0F / 163.0F) +
        sptr[2] * (28.0F / 163.0F) +
        sptr[3] * (56.0F / 163.0F) +
        sptr[4] * (70.0F / 163.0F);
      sptr += 5;  // sptr back to same as dptr (start of next line)
    }
  return result;
}

// ######################################################################
template <class T>
Image<typename promote_trait<T, float>::TP>
lowPass9y(const Image<T>& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const int w = src.getWidth(), h = src.getHeight();
  // promote the source image to float if necessary, so that we do the
  // promotion only once for all, rather than many times as we access
  // the pixels of the image; if no promotion is necessary, "source"
  // will just point to the original data of "src" through the
  // copy-on-write/ref-counting behavior of Image:
  typedef typename promote_trait<T, float>::TP TF;
  const Image<TF> source = src;
  if (h < 2) return source; // nothing to smooth
  Image<TF> result(w, h, NO_INIT);
  typename Image<TF>::const_iterator sptr = source.begin();
  typename Image<TF>::iterator dptr = result.beginw();

  if (h < 9)  // use inefficient implementation for small images
    {
      float kernel[9] = { 1.0F / 256.0F, 8.0F / 256.0F, 28.0F / 256.0F,
                          56.0F / 256.0F, 70.0F / 256.0F, 56.0F / 256.0F,
                          28.0F / 256.0F, 8.0F / 256.0F, 1.0F / 256.0F };
      return sepFilter(src, NULL, kernel, 0, 9,
                       CONV_BOUNDARY_CLEAN);
    }

  // *** vertical pass ***
  const int w2 = w + w, w3 = w2 + w, w4 = w3 + w, w5 = w4 + w, w6 = w5 + w,
    w7 = w6 + w,  w8 = w7 + w;  // index computation speedup
  for (int i = 0; i < w; i ++)
    {
      *dptr++ = sptr[ 0] * (70.0F / 163.0F) +
        sptr[ w] * (56.0F / 163.0F) +
        sptr[w2] * (28.0F / 163.0F) +
        sptr[w3] * ( 8.0F / 163.0F) +
        sptr[w4] * ( 1.0F / 163.0F);
      sptr ++;
    }
  sptr -= w; // back to top-left
  for (int i = 0; i < w; i ++)
    {
      *dptr++ = (sptr[ 0] + sptr[w2]) * (56.0F / 219.0F) +
        sptr[ w] * (70.0F / 219.0F) +
        sptr[w3] * (28.0F / 219.0F) +
        sptr[w4] * ( 8.0F / 219.0F) +
        sptr[w5] * ( 1.0F / 219.0F);
      sptr ++;
    }
  sptr -= w; // back to top-left
  for (int i = 0; i < w; i ++)
    {
      *dptr++ = (sptr[ 0] + sptr[w4]) * (28.0F / 247.0F) +
        (sptr[ w] + sptr[w3]) * (56.0F / 247.0F) +
        sptr[w2] * (70.0F / 247.0F) +
        sptr[w5] * ( 8.0F / 247.0F) +
        sptr[w6] * ( 1.0F / 247.0F);
      sptr ++;
    }
  sptr -= w; // back to top-left
  for (int i = 0; i < w; i ++)
    {
      *dptr++ = (sptr[ 0] + sptr[w6]) * ( 8.0F / 255.0F) +
        (sptr[ w] + sptr[w5]) * (28.0F / 255.0F) +
        (sptr[w2] + sptr[w4]) * (56.0F / 255.0F) +
        sptr[w3] * (70.0F / 255.0F) +
        sptr[w7] * ( 1.0F / 255.0F);
      sptr ++;
    }
  sptr -= w;   // back to top-left
  for (int j = 0; j < h - 8; j ++)
    for (int i = 0; i < w; i ++)
      {
        *dptr++ = (sptr[ 0] + sptr[w8]) * ( 1.0F / 256.0F) +
          (sptr[ w] + sptr[w7]) * ( 8.0F / 256.0F) +
          (sptr[w2] + sptr[w6]) * (28.0F / 256.0F) +
          (sptr[w3] + sptr[w5]) * (56.0F / 256.0F) +
          sptr[w4]  * (70.0F / 256.0F);
        sptr ++;
      }
  for (int i = 0; i < w; i ++)
    {
      *dptr++ = sptr[ 0] * ( 1.0F / 255.0F) +
        (sptr[ w] + sptr[w7]) * ( 8.0F / 255.0F) +
        (sptr[w2] + sptr[w6]) * (28.0F / 255.0F) +
        (sptr[w3] + sptr[w5]) * (56.0F / 255.0F) +
        sptr[w4] * (70.0F / 255.0F);
      sptr ++;
    }
  for (int i = 0; i < w; i ++)
    {
      *dptr++ = sptr[ 0] * ( 1.0F / 247.0F) +
        sptr[ w] * ( 8.0F / 247.0F) +
        (sptr[w2] + sptr[w6]) * (28.0F / 247.0F) +
        (sptr[w3] + sptr[w5]) * (56.0F / 247.0F) +
        sptr[w4] * (70.0F / 247.0F);
      sptr ++;
    }
  for (int i = 0; i < w; i ++)
    {
      *dptr++ = sptr[ 0] * ( 1.0F / 219.0F) +
        sptr[ w] * ( 8.0F / 219.0F) +
        sptr[w2] * (28.0F / 219.0F) +
        (sptr[w3] + sptr[w5]) * (56.0F / 219.0F) +
        sptr[w4] * (70.0F / 219.0F);
      sptr ++;
    }
  for (int i = 0; i < w; i ++)
    {
      *dptr++ = sptr[ 0] * ( 1.0F / 163.0F) +
        sptr[ w] * ( 8.0F / 163.0F) +
        sptr[w2] * (28.0F / 163.0F) +
        sptr[w3] * (56.0F / 163.0F) +
        sptr[w4] * (70.0F / 163.0F);
      sptr ++;
    }
  return result;
}

// ######################################################################
template <class T>
Image<typename promote_trait<T, float>::TP>
lowPass3(const Image<T>& src, const bool go_x, const bool go_y)
{
  Image<typename promote_trait<T, float>::TP> result = src;
  if (go_x) result = lowPass3x(result);
  if (go_y) result = lowPass3y(result);
  return result;
}

// ######################################################################
template <class T>
Image<typename promote_trait<T, float>::TP>
lowPass5(const Image<T>& src, const bool go_x, const bool go_y)
{
  Image<typename promote_trait<T, float>::TP> result = src;
  if (go_x) result = lowPass5x(result);
  if (go_y) result = lowPass5y(result);
  return result;
}

// ######################################################################
template <class T>
Image<typename promote_trait<T, float>::TP>
lowPass9(const Image<T>& src, const bool go_x, const bool go_y)
{
  Image<typename promote_trait<T, float>::TP> result = src;
  if (go_x) result = lowPass9x(result);
  if (go_y) result = lowPass9y(result);
  return result;
}

// ######################################################################
template <class T>
Image<typename promote_trait<T, float>::TP>
lowPass(const int N, const Image<T>& src,
        const bool go_x = true, const bool go_y = true)
{
  Image<typename promote_trait<T, float>::TP> result = src;
  if (go_x) result = lowPassX(N, result);
  if (go_y) result = lowPassY(N, result);
  return result;
}

// ######################################################################
template <class T>
Image<typename promote_trait<T, float>::TP>
lowPassX(const int N, const Image<T>& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  switch (N)
    {
    case 3: return lowPass3x(src);
    case 5: return lowPass5x(src);
    case 9: return lowPass9x(src);
    default: break;
    }

  const Image<float> kern = binomialKernel(N);
  ASSERT(kern.getWidth() == N);
  ASSERT(kern.getHeight() == 1);
  return sepFilter(src, kern.getArrayPtr(), NULL, N, 0,
                   CONV_BOUNDARY_CLEAN);
}

// ######################################################################
template <class T>
Image<typename promote_trait<T, float>::TP>
lowPassY(const int N, const Image<T>& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  switch (N)
    {
    case 3: return lowPass3y(src);
    case 5: return lowPass5y(src);
    case 9: return lowPass9y(src);
    default: break;
    }

  const Image<float> kern = binomialKernel(N);
  ASSERT(kern.getWidth() == N);
  ASSERT(kern.getHeight() == 1);
  return sepFilter(src, NULL, kern.getArrayPtr(), 0, N,
                   CONV_BOUNDARY_CLEAN);
}

// ######################################################################
template <class T>
Image<T> median3x(const Image<T>& in)
{
  if (in.getWidth() < 3) return in;

  const int w = in.getWidth();
  const int h = in.getHeight();

  Image<T> result(in.getDims(), NO_INIT);

  typename Image<T>::const_iterator sptr = in.begin();
  typename Image<T>::iterator dptr = result.beginw();

  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x)
      {
        if (x == 0 || x == w-1)
          *dptr = *sptr;
        else
          *dptr = median3scalar(sptr[-1], sptr[0], sptr[1]);

        ++dptr;
        ++sptr;
      }

  return result;
}

// ######################################################################
template <class T>
Image<T> median3y(const Image<T>& in)
{
  if (in.getHeight() < 3) return in;

  const int w = in.getWidth();
  const int h = in.getHeight();

  Image<T> result(in.getDims(), NO_INIT);

  typename Image<T>::const_iterator sptr = in.begin();
  typename Image<T>::iterator dptr = result.beginw();

  for (int y = 0; y < h; ++y)
    {
      if (y == 0 || y == h-1)
        for (int x = 0; x < w; ++x)
            *dptr++ = *sptr++;
      else
        for (int x = 0; x < w; ++x)
          {
            *dptr = median3scalar(sptr[-w], sptr[0], sptr[w]);

            ++dptr;
            ++sptr;
          }
    }

  return result;
}

// ######################################################################
template <class T>
Image<T> median3(const Image<T>& in, bool go_x, bool go_y)
{
  Image<T> result = in;
  if (go_x) result = median3x(result);
  if (go_y) result = median3y(result);
  return result;
}

// ######################################################################
template <class T>
Image<typename promote_trait<T, float>::TP>
convGauss(const Image<T>& src, const float sigmaX, const float sigmaY,
          const float threshperc)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<typename promote_trait<T, float>::TP> result = src;
  Image <float> kernel;
  if (sigmaX > 0.0)
    {
      kernel = gaussian<float>(0.0F, sigmaX, result.getWidth(),threshperc);
      result = sepFilter(result, kernel, Image<float>(),
                         CONV_BOUNDARY_CLEAN);
    }
  if (sigmaY > 0.0)
    {
      kernel = gaussian<float>(0.0F, sigmaY, src.getHeight(), threshperc);
      result = sepFilter(result, Image<float>(), kernel,
                         CONV_BOUNDARY_CLEAN);
    }
  return result;
}

// Include the explicit instantiations
#include "inst/Image/LowPass.I"

template
Image<promote_trait<double, float>::TP>
lowPass3(const Image<double>& src, const bool, const bool);

template
Image<promote_trait<double, float>::TP>
lowPass5(const Image<double>& src, const bool, const bool);

template
Image<promote_trait<double, float>::TP>
lowPass3x(const Image<double>& src);

template
Image<promote_trait<double, float>::TP>
lowPass3y(const Image<double>& src);

template
Image<promote_trait<double, float>::TP>
lowPass5x(const Image<double>& src);

template
Image<promote_trait<double, float>::TP>
lowPass5y(const Image<double>& src);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_LOWPASS_C_DEFINED

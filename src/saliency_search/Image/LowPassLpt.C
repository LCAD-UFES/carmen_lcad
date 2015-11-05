/*!@file Image/LowPassLpt.C low-pass filtering and smoothing functions */

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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/LowPassLpt.C $
//

#ifndef IMAGE_LOWPASSLPT_C_DEFINED
#define IMAGE_LOWPASSLPT_C_DEFINED

#include "Image/LowPassLpt.H"

#include "Image/Convolutions.H"
#include "Image/LowPass.H"
#include "Image/Image.H"
#include "Image/Kernels.H"
#include "Image/Pixels.H"
#include "rutz/trace.h"

// ######################################################################
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPassLpt(const Image<T_or_RGB>& src, const uint taps, BorderPolicy policy)
{
  if (taps == 5)
    if (policy == CROSS_HEMI)
      return lowPassLpt5w(lowPass5x(src));
    else if (policy == SEPARATE_HEMI)
      return lowPass5y(lowPassLpt5r(src));
    else
      return lowPassLpt5w(lowPassLpt5r(src));
  else if (taps == 3)
    if (policy == CROSS_HEMI)
      return lowPassLpt3w(lowPass3x(src));
    else if (policy == SEPARATE_HEMI)
      return lowPass3y(lowPassLpt3r(src));
    else
      return lowPassLpt3w(lowPassLpt3r(src));
  else
    return Image<typename promote_trait<T_or_RGB, float>::TP>();
}


// ######################################################################
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPassLpt3(const Image<T_or_RGB>& src, BorderPolicy policy)
{
  if (policy == CROSS_HEMI)
    return lowPassLpt3w(lowPass3x(src));
  else if (policy == SEPARATE_HEMI)
    return lowPass3y(lowPassLpt3r(src));
  else
    return lowPassLpt3w(lowPassLpt3r(src));
}

// ######################################################################
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPassLpt3r(const Image<T_or_RGB>& src)
{
  //really this should only be for images with even width, but the
  // operator will still work so we wont disallow it explicitly.
  GVX_TRACE(__PRETTY_FUNCTION__);
  const int w = src.getWidth(), h = src.getHeight(),
    hf = w / 2, h2 = 2*h;
  // promote the source image to float if necessary, so that we do the
  // promotion only once for all, rather than many times as we access
  // the pixels of the image; if no promotion is necessary, "source"
  // will just point to the original data of "src" through the
  // copy-on-write/ref-counting behavior of Image:
  typedef typename promote_trait<T_or_RGB, float>::TP TF;
  const Image<TF> source = src;
  if (w < 2)
    return source; // nothing to smooth

  Image<TF> result(w, h, NO_INIT);
  typename Image<TF>::const_iterator sptr = source.begin();
  typename Image<TF>::iterator dptr = result.beginw();

// Do not use access overload since can access the C array directly.
  // the bet is: float computations are faster than int (true for most
  // float copros), and int2float converts fast->do not have to divide
  // by the coeff sum since we directly use float coeffs.
  // notations: in () is the position of dest ptr, and ^ is src ptr
  // ########## horizontal pass
  for (int j = 0; j < h2; j ++)
    {
      // leftmost point  [ (2^) 1 ] / 3
      *dptr++ = sptr[0] * (2.0F / 3.0F) + sptr[1] * (1.0F / 3.0F);

      // rest of the line up to the vertical meridian [ 1^ (2) 1] / 4
      for (int i = 0; i < hf - 2; i ++)
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
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPassLpt3w(const Image<T_or_RGB>& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const int w = src.getWidth(), h = src.getHeight();
  // promote the source image to float if necessary, so that we do the
  // promotion only once for all, rather than many times as we access
  // the pixels of the image; if no promotion is necessary, "source"
  // will just point to the original data of "src" through the
  // copy-on-write/ref-counting behavior of Image:
  typedef typename promote_trait<T_or_RGB, float>::TP TF;
  const Image<TF> source = src;
  if (h < 2)
    return source; // nothing to smooth
  Image<TF> result(w, h, NO_INIT);
  typename Image<TF>::const_iterator sptr = source.begin(),
    sptrh = source.begin() + w - 1;//the last point in the first image row
  typename Image<TF>::iterator dptr = result.beginw();
  const int w2 = w * 2; // speedup

  //'<' symbol means opposide side of the image
  // ########## vertical pass  (even though we scan horiz for speedup)
  // topmost points  ( [1< (2^) 1 ] / 4 )^T
  for (int i = 0; i < w; i ++)
    {
      //at the border get the pixel from the other hemiefield
      *dptr++ = (sptrh[0] + sptr[w]) * 0.25F + sptr[0] * 0.5F;
      ++sptr;
      --sptrh;
    }
  sptr -= w;  // go back to top-left

  // rest of the column except last point ( [ 1^ (2) 1 ] / 4.0 )^T
  for (int j = 0; j < h - 2; j ++)
    for (int i = 0; i < w; i ++)
      {
        *dptr++ = (sptr[0] + sptr[w2]) * 0.25F + sptr[w] * 0.5F;
        ++sptr;
      }

  // last points ( [ 1^ (2) 1<] / 4 )^T
  sptrh += w*h-1;//go to the last point in the image
  for (int i = 0; i < w; i ++)
    {
      *dptr++ = (sptr[0] + sptrh[0]) * 0.25F + sptr[w] * 0.5F;
      ++sptr;
      --sptrh;
    }

  // finished. here, sptr is one line before the end of array.
  return result;
}

// ######################################################################
template <class T_or_RGB>
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPassLpt5(const Image<T_or_RGB>& src, BorderPolicy policy)
{
  if (policy == CROSS_HEMI)
    return lowPassLpt5w(lowPass5x(src));
  else if (policy == SEPARATE_HEMI)
    return lowPass5y(lowPassLpt5r(src));
  else
    return lowPassLpt5w(lowPassLpt5r(src));
}

// ######################################################################
template <class T_or_RGB> // Anderson's separable kernel: 1/16 * [1 4 6 4 1]
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPassLpt5r(const Image<T_or_RGB>& src)
{
  //this operator really only makes sense on even width images but
  //since it will still work we wont explicity disallow it.
GVX_TRACE(__PRETTY_FUNCTION__);
 const int w = src.getWidth(), h = src.getHeight(),
   hf = w/2, h2 = 2*h;
  // promote the source image to float if necessary, so that we do the
  // promotion only once for all, rather than many times as we access
  // the pixels of the image; if no promotion is necessary, "source"
  // will just point to the original data of "src" through the
  // copy-on-write/ref-counting behavior of Image:
  typedef typename promote_trait<T_or_RGB, float>::TP TF;
  const Image<TF> source = src;
  if (w < 4)
    return source; // nothing to smooth
  Image<TF> result(w, h, NO_INIT);
  typename Image<TF>::const_iterator sptr = source.begin();
  typename Image<TF>::iterator dptr = result.beginw();

  //Do not use access overloading since can
  // access the C array directly.  the bet is: float
  // computations are faster than int (true for most float
  // copros), and int2float convert is fast -> do not have to
  // divide by the coeff sum since we directly use float
  // coeffs.
  // notations: in () is the position of dest ptr, and ^ is src ptr
  // ########## horizontal pass
  for (int j = 0; j < h2; j ++)
    {
      // leftmost point  [ (6^) 4 1 ] / 11
      *dptr++ = sptr[0] * (6.0F / 11.0F) +
        sptr[1] * (4.0F / 11.0F) +
        sptr[2] * (1.0F / 11.0F);

      // second point    [ 4^ (6) 4 1 ] / 15
      *dptr++ = (sptr[0] + sptr[2]) * (4.0F / 15.0F) +
        sptr[1] * (6.0F / 15.0F) +
        sptr[3] * (1.0F / 15.0F);

      // rest of the line to vertical meridian  [ 1^ 4 (6) 4 1 ] / 16.0
      for (int i = 0; i < hf - 4; i ++)
        {
          *dptr++ = (sptr[0] + sptr[4]) * (1.0F / 16.0F) +
            (sptr[1] + sptr[3]) * (4.0F / 16.0F) +
            sptr[2]  * (6.0F / 16.0F);
          ++sptr;
        }

      // before last point [ 1^ 4 (6) 4 ] / 15
      *dptr++ = sptr[0] * (1.0F / 15.0F) +
        (sptr[1] + sptr[3]) * (4.0F / 15.0F) +
        sptr[2] * (6.0F / 15.0F);
      ++sptr;

      // last point [ 1^ 4 (6) ] / 11
      *dptr++ = sptr[0] * (1.0F / 11.0F) +
        sptr[1] * (4.0F / 11.0F) +
        sptr[2] * (6.0F / 11.0F);
      sptr += 3;  // sptr back to same position as dptr
    }
  return result;
}

// ######################################################################
template <class T_or_RGB> // Anderson's separable kernel: 1/16 * [1 4 6 4 1]
Image<typename promote_trait<T_or_RGB, float>::TP>
lowPassLpt5w(const Image<T_or_RGB>& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const int w = src.getWidth(), h = src.getHeight();
  // promote the source image to float if necessary, so that we do the
  // promotion only once for all, rather than many times as we access
  // the pixels of the image; if no promotion is necessary, "source"
  // will just point to the original data of "src" through the
  // copy-on-write/ref-counting behavior of Image:
  typedef typename promote_trait<T_or_RGB, float>::TP TF;
  const Image<TF> source = src;
  if (h < 4)
    return source; // nothing to smooth
  Image<TF> result(w, h, NO_INIT);
  typename Image<TF>::const_iterator sptr = source.begin(),
    sptrh = source.begin()+w-1;//the last point in the first image row
  typename Image<TF>::iterator dptr = result.beginw();

  // ########## vertical pass  (even though we scan horiz for speedup)
  const int w2 = w * 2, w3 = w * 3, w4 = w * 4; // speedup
  //'<' symbol means opposite side of the image
  // topmost points  ( [1< 4< (6^) 4 1 ] / 16 )^T
  for (int i = 0; i < w; i ++)
    {
      *dptr++ = sptr[0] * (6.0F / 16.0F) +
        (sptr[w] + sptrh[0] ) * (4.0F / 16.0F) +
        (sptr[w2] + sptrh[w2] ) * (1.0F / 16.0F);
      ++sptr;
      --sptrh;
    }
  sptr -= w;  // go back to top-left
  sptrh += w;//go back to last point in first row

  // second topmost points  ( [1< 4^ (6) 4 1 ] / 16 )^T
  for (int i = 0; i < w; i ++)
    {
      *dptr++ = (sptr[0] + sptr[w2]) * (4.0F / 16.0F) +
        sptr[w] * (6.0F / 16.0F) +
        (sptr[w3] + sptrh[0]) * (1.0F / 16.0F);
      ++sptr;
      --sptrh;
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

  sptrh = source.end();//the last point in the image
  // before last points ( [ 1^ 4 (6) 4 1< ] / 16 )T
  for (int i = 0; i < w; i ++)
    {
      *dptr++ = (sptr[0] + sptrh[0]) * (1.0F / 16.0F) +
        (sptr[w] + sptr[w3]) * (4.0F / 16.0F) +
        sptr[w2] * (6.0F / 16.0F);
      ++sptr;
      --sptrh;
    }
  --sptrh;//how we are the last pixel in the second to last row.
  // last points ( [ 1^ 4 (6) 4< 1< ] / 16 )T
  for (int i = 0; i < w; i ++)
    {
      *dptr++ = (sptr[0] + sptrh[0]) * (1.0F / 16.0F) +
        (sptr[w] + sptrh[w]) * (4.0F / 16.0F) +
        sptr[w2] * (6.0F / 16.0F);
      ++sptr;
      --sptrh;
    }
  // finished. here, sptr is two lines before the end of array.
  return result;
}

// ######################################################################
// Include the explicit instantiations
// ######################################################################
#include "inst/Image/LowPassLpt.I"

template
Image<promote_trait<double, float>::TP>
lowPassLpt(const Image<double>& src, const uint taps,
           BorderPolicy policy);

template
Image<promote_trait<double, float>::TP>
lowPassLpt3(const Image<double>& src, BorderPolicy policy);

template
Image<promote_trait<double, float>::TP>
lowPassLpt3r(const Image<double>& src);

template
Image<promote_trait<double, float>::TP>
lowPassLpt3w(const Image<double>& src);

template
Image<promote_trait<double, float>::TP>
lowPassLpt5(const Image<double>& src, BorderPolicy policy);

template
Image<promote_trait<double, float>::TP>
lowPassLpt5r(const Image<double>& src);

template
Image<promote_trait<double, float>::TP>
lowPassLpt5w(const Image<double>& src);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_LOWPASSLPT_C_DEFINED

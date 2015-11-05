/*!@file Image/ColorOps.C Color operations on Image
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/ColorOps.C $
// $Id: ColorOps.C 15059 2011-11-11 00:55:58Z dberg $
//

#ifndef IMAGE_COLOROPS_C_DEFINED
#define IMAGE_COLOROPS_C_DEFINED

#include "Image/ColorOps.H"

#include "Image/ColorMap.H"
#include "Image/FilterOps.H"  // for lowPass3()
#include "Image/Image.H"
#include "Image/MathOps.H"    // for countThresh(), inplaceNormalize(), ...
#include "Image/Pixels.H"
#include "Image/ShapeOps.H"   // for decXY()
#include "Image/Transforms.H" // for dct()
#include "Util/Assert.H"
#include "Util/MathFunctions.H"
#include "Util/log.H"
#include "rutz/trace.h"

#include <algorithm>
#include <cmath>

// ######################################################################
template <class T>
Image<PixRGB<T> > makeRGB(const Image<T>& red,
                          const Image<T>& green,
                          const Image<T>& blue)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(red.isSameSize(green) && red.isSameSize(blue));

  Image<PixRGB<T> > result(red.getDims(), NO_INIT);
  typename Image<PixRGB<T> >::iterator aptr = result.beginw();
  typename Image<PixRGB<T> >::iterator stop = result.endw();

  typename Image<T>::const_iterator rptr = red.begin();
  typename Image<T>::const_iterator gptr = green.begin();
  typename Image<T>::const_iterator bptr = blue.begin();

  while (aptr != stop)
    {
      *aptr++ = PixRGB<T>(*rptr++, *gptr++, *bptr++);
    }

  return result;
}

// ######################################################################
Image< PixRGB<byte> > colorize(const Image<byte>& img, const ColorMap& cmap)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (cmap.getWidth() != 256) LFATAL("Need a ColorMap with 256 entries");

  Image< PixRGB<byte> > result(img.getDims(), NO_INIT);
  Image<byte>::const_iterator src = img.begin(), stop = img.end();
  Image< PixRGB<byte> >::iterator dst = result.beginw();

  const Image<PixRGB<byte> >::const_iterator cmapptr = cmap.begin();

  while (src != stop)
    *dst++ = cmapptr[*src++];

  return result;
}

// ######################################################################
void inplaceColorSpeckleNoise(Image< PixRGB<byte> >& dest, const int num)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(dest.initialized());
  const int w = dest.getWidth(), h = dest.getHeight();

  for (int i = 0; i < num; ++i)
    {
      int x = randomUpToNotIncluding(w);
      int y = randomUpToNotIncluding(h);
      dest.setVal(x, y,
                  PixRGB<byte>(randomUpToIncluding(255),
                               randomUpToIncluding(255),
                               randomUpToIncluding(255)));
    }
}

// ######################################################################
template <class T>
void getComponents(const Image<PixRGB<T> >& src,
                   Image<T>& red, Image<T>& green, Image<T>& blue)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());

  red = Image<T>(src.getDims(), NO_INIT);
  green = Image<T>(src.getDims(), NO_INIT);
  blue = Image<T>(src.getDims(), NO_INIT);

  typename Image<T>::iterator rptr = red.beginw();
  typename Image<T>::iterator gptr = green.beginw();
  typename Image<T>::iterator bptr = blue.beginw();

  typename Image<PixRGB<T> >::const_iterator aptr = src.begin();
  typename Image<PixRGB<T> >::const_iterator stop = src.end();

  while (aptr != stop)
    {
      *rptr++ = aptr->red();
      *gptr++ = aptr->green();
      *bptr++ = aptr->blue();
      ++aptr;
    }
}

// ######################################################################
Image<PixRGB<float> > normalizeRGPolar(const Image<float>& src,
                                       const float max,
                                       const float min)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return stainPosNeg(src, std::max(fabs(min), fabs(max)),
                     PixRGB<float>(0.0f), // background
                     PixRGB<float>(0.0f, 255.0f, 0.0f), // positive=green
                     PixRGB<float>(255.0f, 0.0f, 0.0f));// negative=red
}

// ######################################################################
Image<PixRGB<float> > normalizeRGPolarAuto(const Image<float>& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  float max, min;
  getMinMax(src,min,max);
  return stainPosNeg(src, std::max(fabs(min), fabs(max)),
                     PixRGB<float>(0.0f), // background
                     PixRGB<float>(0.0f, 255.0f, 0.0f), // positive=green
                     PixRGB<float>(255.0f, 0.0f, 0.0f));// negative=red
}

// ######################################################################
Image<PixRGB<float> > normalizeWithScale(const Image<float>& src,
                                         const float min = 0.0F,
                                         const float max = 255.0F,
                                         const float clamp = 255.0F,
                                         const char baseColor = 1,
                                         const char normColor = 3)
{
  ASSERT(src.initialized());
  ASSERT(baseColor < 4);
  ASSERT(normColor < 4);
  ASSERT(normColor != baseColor);
  ASSERT(clamp > 0.0F);
  float mi,ma;
  getMinMax(src,mi,ma);
  const float scale  = max - min;
  const float nscale = (ma - mi) / scale;
  Image<PixRGB<float> > outImage;
  outImage.resize(src.getWidth(),src.getHeight());

  Image<float>::const_iterator    aptr = src.begin();
  Image<float>::const_iterator    stop = src.end();
  Image<PixRGB<float> >::iterator optr = outImage.beginw();
  while (aptr != stop)
  {
    if(*aptr > clamp || *aptr < 0.0F)
      (*optr).p[(uint)baseColor] = clamp;
    else
      (*optr).p[(uint)baseColor] = *aptr;
    (*optr).p[(uint)normColor] = min + ((*aptr - mi)*nscale);
    ++aptr; ++optr;
  }
  return outImage;
}

// ######################################################################
Image<PixRGB<float> > normalizeScaleRainbow(const Image<float>& src,
                                            const float min = 0.0F,
                                            const float max = 255.0F)
{
  ASSERT(src.initialized());
  ASSERT(max > min);

  Image<PixHSV<float> > scaleImage;

  scaleImage.resize(src.getWidth(),src.getHeight());

  float mi, ma;
  getMinMax(src,mi,ma);
  const float interval = ma - mi;
  const float mult     = 330.0/(max - min);

  Image<PixHSV<float> >::iterator aptr = scaleImage.beginw();
  Image<float>::const_iterator    sptr = src.begin();

  // Set intensity as the normalized value of src
  // Set hue as the scale of the value of src with red as 0 and
  //     violet as 330

  while(sptr != src.end())
  {
    if(*sptr < min)     { aptr->setH(1); aptr->setS(1.0); aptr->setV(0);   }
    else if(*sptr > max){ aptr->setH(1); aptr->setS(1.0); aptr->setV(255); }
    else
    {
      aptr->setH(mult * (*sptr - min));
      aptr->setS(100.0);
      aptr->setV(((*sptr - mi)/interval) * 255.0);
    }
    ++aptr; ++sptr;
  }

  Image<PixRGB<float> > dest; dest.resize(src.getWidth(),src.getHeight());

                                  aptr = scaleImage.beginw();
  Image<PixRGB<float> >::iterator dptr = dest.beginw();

  while(aptr != scaleImage.end()) *dptr++ = PixRGB<float>(*aptr++);

  return dest;
}

// ######################################################################
template <class T>
Image<PixRGB<T> > stain(const Image<T>& src, PixRGB<float> color)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<PixRGB<T> > result(src.getDims(), NO_INIT);

  typename Image<T>::const_iterator sptr = src.begin();
  typename Image<PixRGB<T> >::iterator dptr = result.beginw(),
    stop = result.endw();

  while (dptr != stop)
    {
      *dptr = PixRGB<T>(color * (*sptr));
      ++sptr;
      ++dptr;
    }

  return result;
}

// ######################################################################
Image<PixRGB<float> > stainPosNeg(const Image<float>& src,
                                  const float maxval,
                                  const PixRGB<float>& background,
                                  const PixRGB<float>& pos_stain,
                                  const PixRGB<float>& neg_stain)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  Image<PixRGB<float> > result(src.getDims(), NO_INIT);

  Image<PixRGB<float> >::iterator dptr = result.beginw();
  Image<PixRGB<float> >::iterator stop = result.endw();
  Image<float>::const_iterator    sptr = src.begin();

  if(maxval > 0)
  {
    while (dptr != stop)
    {
      const float ratio = (*sptr++/maxval);
      if (ratio > 0)
        *dptr++ = pos_stain * ratio + background * (1.0f - ratio);
      else
        *dptr++ = neg_stain * (-ratio) + background * (1.0f + ratio);
    }
  }
  else
  {
    while (dptr != stop)
      *dptr++ = PixRGB<float>(0,0,0);
  }

  return result;
}

// ######################################################################
Image<PixRGB<float> > overlayStain(const Image<float>& top,
                                   const Image<float>& bottom,
                                   const float trans, const char channel)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // 0 to 100 percent only:
  ASSERT((trans >= 0.0F) && (trans <= 100.0F));
  // images must be same size:
  ASSERT(top.isSameSize(bottom));

  // result is the size of the input images:
  Image<PixRGB<float> > result(top.getDims(), NO_INIT);

  Image<PixRGB<float> >::iterator
    dptr = result.beginw(),
    stop = result.endw();

  Image<float>::const_iterator
    tptr = top.begin(),
    bptr = bottom.begin();

  float tfac = trans * 0.01F;

  while (dptr != stop)
    {
      float top_val = *tptr++;

      PixRGB<float> pix;

      if(channel == 'r'){pix.setRed(top_val);pix.setGreen(0);pix.setBlue(0);}
      if(channel == 'g'){pix.setRed(0);pix.setGreen(top_val);pix.setBlue(0);}
      if(channel == 'b'){pix.setRed(0);pix.setGreen(0);pix.setBlue(top_val);}

      *dptr++ = pix - ( pix - (*bptr++) ) * tfac;
    }

  return result;
}

// ######################################################################
template <class T>
void getMinMaxC(const Image<PixRGB<T> >& src, T& mi, T& ma)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());
  typename Image<PixRGB<T> >::const_iterator aptr = src.begin();
  typename Image<PixRGB<T> >::const_iterator stop = src.end();

  mi = aptr->red(); ma = aptr->red();

  while (aptr != stop)
    {
      T x = aptr->red();
      if (x < mi) mi = x; else if (x > ma) ma = x;

      x = aptr->green();
      if (x < mi) mi = x; else if (x > ma) ma = x;

      x = aptr->blue();
      if (x < mi) mi = x; else if (x > ma) ma = x;

      ++aptr;
    }
}

// ######################################################################
template <class T>
PixRGB<float> meanRGB(const Image<PixRGB<T> >& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());

  typename Image<PixRGB<T> >::const_iterator sptr = src.begin();
  typename Image<PixRGB<T> >::const_iterator stop = src.end();

  PixRGB<float> meanRGB;
  while (sptr != stop)
  {
    meanRGB[0] += sptr->p[0];
    meanRGB[1] += sptr->p[1];
    meanRGB[2] += sptr->p[2];
    ++sptr;
  }

  meanRGB[0] /= src.size();
  meanRGB[1] /= src.size();
  meanRGB[2] /= src.size();

  return meanRGB;
}

// ######################################################################
template <>
Image<byte> luminance(const Image<PixRGB<byte> >& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());

  Image<byte> result(src.getDims(), NO_INIT);

  Image<PixRGB<byte> >::const_iterator aptr = src.begin();

  Image<byte>::iterator dptr = result.beginw();
  Image<byte>::iterator stop = result.endw();

  while (dptr != stop)
    {
      *dptr++ = (aptr->p[0] + aptr->p[1] + aptr->p[2]) / 3;
      ++aptr;
    }

  return result;
}

// ######################################################################
template <class T>
Image<T> luminance(const Image<PixRGB<T> >& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());

  Image<T> result(src.getDims(), NO_INIT);

  typename Image<PixRGB<T> >::const_iterator aptr = src.begin();

  typename Image<T>::iterator dptr = result.beginw();
  typename Image<T>::iterator stop = result.endw();

  while (dptr != stop)
    *dptr++ = (*aptr++).luminance();

  return result;
}

// ######################################################################
template <class T>
Image<T> luminance(const Image<T>& src)
{ return src; }


// ######################################################################
template <class T>
Image<T> luminanceNTSC(const Image<PixRGB<T> >& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());
  Image<T> result(src.getDims(), NO_INIT);

  float coef1,coef2,coef3;
  //Taken from Matlab's rgb2gray() function
  // T = inv([1.0 0.956 0.621; 1.0 -0.272 -0.647; 1.0 -1.106 1.703]);
  // coef = T(1,:)';
  coef1 = 0.298936F; coef2 = 0.587043F; coef3 = 0.114021F;


  typename Image<PixRGB<T> >::const_iterator aptr = src.begin();

  typename Image<T>::iterator dptr = result.beginw();
  typename Image<T>::iterator stop = result.endw();

  while (dptr != stop){
    *dptr++ = T(round(aptr->p[0]*coef1 + aptr->p[1]*coef2 + aptr->p[2]*coef3));
    ++aptr;
  }
  return result;
}


// ######################################################################
template <class T>
Image< PixRGB<T> > toRGB(const Image<T>& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());

  Image< PixRGB<T> > result(src.getDims(), NO_INIT);
  typename Image<T>::const_iterator aptr = src.begin();
  typename Image< PixRGB<T> >::iterator dptr = result.beginw();
  typename Image< PixRGB<T> >::iterator stop = result.endw();

  while (dptr != stop) *dptr++ = PixRGB<T>(*aptr++);

  return result;
}

// ######################################################################
template <class T>
Image< PixRGB<T> > toRGB(const Image< PixRGB<T> >& src)
{ return src; }

// ######################################################################
template <class T>
Image<float> infoMeasure(const Image<PixRGB<T> >& src,
                         const float eps, const int size)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());

  Image< PixRGB<float> > float_copy(src);

  Image<float> y, i, q;
  getYIQ(float_copy, y, i, q);

  i = ::decXY(::lowPass3(i));
  q = ::decXY(::lowPass3(q));

  // eps is given between 0 and 1; modify it according to image size
  // we want that eps=0.01 corresponds to a cosine with amplitude 127.5*0.01
  float epsy = eps * ((float)(size*size) * 127.5);
  float epsiq = epsy / 4.0;

  Image<float> result(src.getWidth() / size, src.getHeight() / size, NO_INIT);

  for (int offy = 0; offy < src.getHeight(); offy += size)
    for (int offx = 0; offx < src.getWidth(); offx += size)
      {
        Image<float> ydct = dct(y, offx, offy, size);
        Image<float> idct = dct(i, offx/2, offy/2, size/2);
        Image<float> qdct = dct(q, offx/2, offy/2, size/2);

        result.setVal(offx/size, offy/size,
                      (float(countThresh(ydct, epsy)) +
                       4.0 * float(countThresh(idct, epsiq)) +
                       4.0 * float(countThresh(qdct, epsiq)))
                      / (3.0 * float(size*size)));
      }

  return result;
}

// ######################################################################
template <class T> inline
void getYIQ(const Image<PixRGB<T> >& src,
            Image<T>& y, Image<T>& i, Image<T>& q)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());
  y = Image<T>(src.getDims(), NO_INIT);
  i = Image<T>(src.getDims(), NO_INIT);
  q = Image<T>(src.getDims(), NO_INIT);

  typename Image<PixRGB<T> >::const_iterator aptr = src.begin();
  typename Image<PixRGB<T> >::const_iterator stop = src.end();

  typename Image<T>::iterator yptr = y.beginw();
  typename Image<T>::iterator iptr = i.beginw();
  typename Image<T>::iterator qptr = q.beginw();

  while (aptr != stop)
    {
      T yy, ii, qq;
      PixYIQ<T>(*aptr++).getYIQ(yy, ii, qq);
      *yptr++ = yy;
      *iptr++ = ii;
      *qptr++ = qq;
    }
}

// ######################################################################
template <class T> inline
void getJpegYUV(const Image<PixRGB<T> >& src,
                Image<T>& y, Image<T>& u, Image<T>& v)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());
  y = Image<T>(src.getDims(), NO_INIT);
  u = Image<T>(src.getDims(), NO_INIT);
  v = Image<T>(src.getDims(), NO_INIT);

  typename Image<PixRGB<T> >::const_iterator aptr = src.begin();
  typename Image<PixRGB<T> >::const_iterator stop = src.end();

  typename Image<T>::iterator yptr = y.beginw();
  typename Image<T>::iterator uptr = u.beginw();
  typename Image<T>::iterator vptr = v.beginw();

  while (aptr != stop)
    {
      const PixJpegYUV<T> yuvpix(*aptr++);
      *yptr++ = yuvpix.Y();
      *uptr++ = yuvpix.U();
      *vptr++ = yuvpix.V();
    }
}

// ######################################################################
template <class T>
Image<PixRGB<T> > luminanceNormalize(const Image<PixRGB<T> >& src,
                                     const T thresh)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());

  Image<PixRGB<T> > result(src.getDims(), NO_INIT);

  typename Image<PixRGB<T> >::const_iterator aptr = src.begin();
  typename Image<PixRGB<T> >::const_iterator stop = src.end();

  typename Image<PixRGB<T> >::iterator dptr = result.beginw();
  const PixRGB<T> zero(T(0));

  while (aptr != stop)
    {
      float v = float(aptr->luminance());
      if (v >= thresh)
        {
          const float fac = 255.0 / (v * 3.0);
          dptr->setRed  ( T(float(aptr->red()) * fac) );
          dptr->setGreen( T(float(aptr->green()) * fac) );
          dptr->setBlue ( T(float(aptr->blue()) * fac) );
        }
      else
        *dptr = zero;

      ++aptr; ++dptr;
    }

  return result;
}

// ######################################################################
template <class T>
void getRGBY(const Image<PixRGB<T> >& src,
             Image<typename promote_trait<T, float>::TP>& rg,
             Image<typename promote_trait<T, float>::TP>& by,
             const typename promote_trait<T, float>::TP thresh)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // red = [r - (g+b)/2]        [.] = clamp between 0 and 255
  // green = [g - (r+b)/2]
  // blue = [b - (r+g)/2]
  // yellow = [2*((r+g)/2 - |r-g| - b)]

  ASSERT(src.initialized());
  typedef typename promote_trait<T, float>::TP TF;
  rg = Image<TF>(src.getDims(), NO_INIT);
  by = Image<TF>(src.getDims(), NO_INIT);

  typename Image< PixRGB<T> >::const_iterator aptr = src.begin();
  typename Image< PixRGB<T> >::const_iterator stop = src.end();

  typename Image<TF>::iterator rgptr = rg.beginw(), byptr = by.beginw();
  TF zero = TF(); TF thresh3 = 3.0F * thresh;

  while (aptr != stop)
    {
      TF r = TF(aptr->red()), g = TF(aptr->green()), b = TF(aptr->blue());

      // first do the luminanceNormalization:
      TF lum = r + g + b;
      if (lum < thresh3)  // too dark... no response from anybody
        { *rgptr++ = zero; *byptr++ = zero; ++aptr; }
      else
        {
          // normalize chroma by luminance:
          TF fac = 255.0f / lum;
          r *= fac; g *= fac; b *= fac;

          // now compute color opponencies:
          // yellow gets a factor 2 to compensate for its previous attenuation
          // by luminanceNormalize():
          TF red = r - 0.5f * (g + b), green = g - 0.5f * (r + b);
          TF blue = b - 0.5f * (r + g), yellow = -2.0f * (blue + fabs(r-g));

          if (red < 0.0f) red = 0.0f;
          else if (red > 255.0f) red = 255.0f;
          if (green < 0.0f) green = 0.0f;
          else if (green > 255.0f) green = 255.0f;
          if (blue < 0.0f) blue = 0.0f;
          else if (blue > 255.0f) blue = 255.0f;
          if (yellow < 0.0f) yellow=0.0f;
          else if (yellow > 255.0f) yellow=255.0f;

          *rgptr++ = red - green; *byptr++ = blue - yellow;
          ++aptr;
        }
    }
}

// ######################################################################
template <class T>
void getRGBYsimple(const Image<PixRGB<T> >& src,
                   Image<typename promote_trait<T, float>::TP>& rg,
                   Image<typename promote_trait<T, float>::TP>& by,
                   const typename promote_trait<T, float>::TP thresh)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // I = max(R,G,B)
  // RG = (R-G)/I
  // BY = (B-min(R,G))/I
  // Dirk Walther, September 2004
  ASSERT(src.initialized());
  typedef typename promote_trait<T, float>::TP TF;
  rg = Image<TF>(src.getDims(), NO_INIT);
  by = Image<TF>(src.getDims(), NO_INIT);

  typename Image< PixRGB<T> >::const_iterator aptr = src.begin();
  typename Image< PixRGB<T> >::const_iterator stop = src.end();

  typename Image<TF>::iterator rgptr = rg.beginw(), byptr = by.beginw();
  TF zero = TF();

  while (aptr != stop)
    {
      TF r = aptr->red(), g = aptr->green(), b = aptr->blue();

      // first do in = max(R,G,B):
      TF in = r;
      if (g > in) in = g;
      if (b > in) in = b;
      if (in < thresh)  // too dark... no response from anybody
        { *rgptr++ = zero; *byptr++ = zero; ++aptr; }
      else
        {
          TF y = (r < g) ? r : g;
          *rgptr++ = (r - g) / in;
          *byptr++ = (b - y) / in;
          ++aptr;
        }
    }
}

// ######################################################################
template <class T>
void getRGBY(const Image<PixRGB<T> >& src, Image<T>& rr, Image<T>& gg,
             Image<T>& bb, Image<T>& yy, const T thresh)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // this is essentially the same code as the other getRGBY, just
  // duplicated for efficiency
  ASSERT(src.initialized());
  rr = Image<T>(src.getDims(), NO_INIT);
  gg = Image<T>(src.getDims(), NO_INIT);
  bb = Image<T>(src.getDims(), NO_INIT);
  yy = Image<T>(src.getDims(), NO_INIT);

  typename Image< PixRGB<T> >::const_iterator aptr = src.begin();
  typename Image< PixRGB<T> >::const_iterator stop = src.end();
  typename Image<T>::iterator rptr = rr.beginw(), gptr = gg.beginw(),
    bptr = bb.beginw(), yptr = yy.beginw();

  T zero = T(); float threshf = float(thresh);

  while (aptr != stop)
    {
      float r = aptr->red(), g = aptr->green(), b = aptr->blue();

      // first do the luminanceNormalization:
      float lum = (r + g + b) / 3.0f;
      if (lum < threshf)  // too dark... no response from anybody
        { *rptr++=zero; *gptr++=zero; *bptr++=zero; *yptr++=zero; ++aptr; }
      else
        {
          // normalize chroma by luminance:
          float fac = 255.0f / (3.0f * lum);
          r *= fac; g *= fac; b *= fac;

          // now compute color opponencies:
          // yellow gets a factor 2 to compensate for its previous attenuation
          // by luminanceNormalize():
          float red = r - 0.5f * (g + b), green = g - 0.5f * (r + b);
          float blue = b - 0.5f * (r + g), yellow = -2.0f * (blue + fabs(r-g));

          if (red < 0.0f) red = 0.0f;
          else if (red > 255.0f) red = 255.0f;
          if (green < 0.0f) green = 0.0f;
          else if (green > 255.0f) green = 255.0f;
          if (blue < 0.0f) blue = 0.0f;
          else if (blue > 255.0f) blue = 255.0f;
          if (yellow < 0.0f) yellow=0.0f;
          else if (yellow > 255.0f) yellow=255.0f;

          *rptr++ = T(red);     // no clamping necessary
          *gptr++ = T(green);
          *bptr++ = T(blue);
          *yptr++ = T(yellow);

          ++aptr;
        }
    }
}

// ######################################################################
template <class T>
void getRGBY(const Image<PixRGB<T> >& src,
             Image<T>& rg,  Image<T>& by,
             Image<T>& sat, Image<T>& val,
             const ushort H2SVtype)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());
  typename Image<PixRGB<T> >::const_iterator aptr = src.begin();

  rg  = Image<T>(src.getDims(), NO_INIT);
  by  = Image<T>(src.getDims(), NO_INIT);
  sat = Image<T>(src.getDims(), NO_INIT);
  val = Image<T>(src.getDims(), NO_INIT);

  typename Image<T>::iterator rptr = rg.beginw(),  bptr = by.beginw(),
                              sptr = sat.beginw(), vptr = val.beginw();

  while(aptr != src.end())
  {
    if(H2SVtype == 1)
    {
      PixH2SV1<T> pix = PixH2SV1<T>(*aptr++);
      *rptr++ = pix.H2(); *bptr++ = pix.H1();
      *sptr++ = pix.S();  *vptr++ = pix.V();
    }
    else if(H2SVtype == 2)
    {
      PixH2SV2<T> pix = PixH2SV2<T>(*aptr++);
      *rptr++ = pix.H2(); *bptr++ = pix.H1();
      *sptr++ = pix.S();  *vptr++ = pix.V();
    }
    else
    {
      LFATAL("This type of H2SV pixel not yet supported");
    }
  }
}

// ######################################################################
template <class T>
void getDKL(const Image<PixRGB<T> >& src,
            Image<typename promote_trait<T, float>::TP>& dimg,
            Image<typename promote_trait<T, float>::TP>& kimg,
            Image<typename promote_trait<T, float>::TP>& limg)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(src.initialized());
  typedef typename promote_trait<T, float>::TP TF;
  dimg = Image<TF>(src.getDims(), NO_INIT);
  kimg = Image<TF>(src.getDims(), NO_INIT);
  limg = Image<TF>(src.getDims(), NO_INIT);

  typename Image< PixRGB<T> >::const_iterator aptr = src.begin(), stop = src.end();
  typename Image<TF>::iterator dptr = dimg.beginw(), kptr = kimg.beginw(), lptr = limg.beginw();

  while (aptr != stop)
    {
      const PixDKL<TF> p(*aptr++);
      *dptr++ = p.D();
      *kptr++ = p.K();
      *lptr++ = p.L();
    }
}

// ######################################################################
template <class T>
void getDKLM(const Image<PixRGB<T> >& src,
             Image<typename promote_trait<T, float>::TP>& RGimg,
             Image<typename promote_trait<T, float>::TP>& BYimg,
             Image<typename promote_trait<T, float>::TP>& LUMimg)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(src.initialized());
  typedef typename promote_trait<T, float>::TP TF;
  RGimg = Image<TF>(src.getDims(), NO_INIT);
  BYimg = Image<TF>(src.getDims(), NO_INIT);
  LUMimg = Image<TF>(src.getDims(), NO_INIT);

  typename Image< PixRGB<T> >::const_iterator aptr = src.begin(), stop = src.end();
  typename Image<TF>::iterator rgptr = RGimg.beginw(), byptr = BYimg.beginw(), lumptr = LUMimg.beginw();

  while (aptr != stop)
    {
      const PixDKLM<TF> p(*aptr++);
      *rgptr++ = p.RG();
      *byptr++ = p.BY();
      *lumptr++ = p.LUM();
    }
}

// ######################################################################
template <class T>
void getLAB(const Image<PixRGB<T> >& src,
            Image<typename promote_trait<T, float>::TP>& limg,
            Image<typename promote_trait<T, float>::TP>& aimg,
            Image<typename promote_trait<T, float>::TP>& bimg)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(src.initialized());
  typedef typename promote_trait<T, float>::TP TF;
  limg = Image<TF>(src.getDims(), NO_INIT);
  aimg = Image<TF>(src.getDims(), NO_INIT);
  bimg = Image<TF>(src.getDims(), NO_INIT);

  typename Image< PixRGB<T> >::const_iterator aptr = src.begin(), stop = src.end();
  typename Image<TF>::iterator dptr = limg.beginw(), kptr = aimg.beginw(), lptr = bimg.beginw();

  while (aptr != stop)
    {

      const PixRGB<byte> c(*aptr++);
      float X, Y, Z, fX, fY, fZ;
      int L,a,b;
      float R = c.red(); //(((rgbColor & 0xf800) >> 11) * 255 / 31);
      float G = c.green(); //(((rgbColor & 0x07e0) >> 5) * 255 / 63);
      float B = c.blue(); //((rgbColor & 0x001f) * 255 / 31);

      X = 0.412453 * R + 0.357580 * G + 0.180423 * B;
      Y = 0.212671 * R + 0.715160 * G + 0.072169 * B;
      Z = 0.019334 * R + 0.119193 * G + 0.950227 * B;

      X /= (255 * 0.950456);
      Y /=  255;
      Z /= (255 * 1.088754);

      if (Y > 0.008856){
        fY = pow(Y, 1.0 / 3.0);
        L = (int)(116.0 * fY - 16.0 + 0.5);
      }else{
        fY = 7.787 * Y + 16.0 / 116.0;
        L = (int)(903.3 * Y + 0.5);
      }


      if (X > 0.008856)
        fX = pow(X, 1.0 / 3.0);
      else
        fX = 7.787 * X + 16.0 / 116.0;

      if (Z > 0.008856)
        fZ = pow(Z, 1.0 / 3.0);
      else
        fZ = 7.787 * Z + 16.0 / 116.0;

      a = (int)(500.0 * (fX - fY) + 0.5);
      b = (int)(200.0 * (fY - fZ) + 0.5);
      //-128~127
      //
      *dptr++ = L;
      *kptr++ = a;
      *lptr++ = b;


    }
}

// ######################################################################
void normalizeLAB
(Image<float>& limg,
 Image<float>& aimg,
 Image<float>& bimg)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // assert that all images are of same size
  ASSERT(limg.isSameSize(aimg));
  ASSERT(limg.isSameSize(bimg));

  // range for a, b channels
  const float ab_min = -73;
  const float ab_max = 95;
  const float ab_range = ab_max - ab_min;
   
  Image<float>::iterator lptr = limg.beginw(), stop = limg.endw();
  Image<float>::iterator aptr = aimg.beginw(), bptr = bimg.beginw();

  // normalize Lab image
  while (lptr != stop)
    {
      float l_val = *lptr / 100.0F;
      float a_val = (*aptr - ab_min) / ab_range;
      float b_val = (*bptr - ab_min) / ab_range;

      if (l_val < 0) { l_val = 0.0F; } else if (l_val > 1) { l_val = 1.0F; }
      if (a_val < 0) { a_val = 0.0F; } else if (a_val > 1) { a_val = 1.0F; }
      if (b_val < 0) { b_val = 0.0F; } else if (b_val > 1) { b_val = 1.0F; }

      *lptr++ = l_val;
      *aptr++ = a_val;
      *bptr++ = b_val;
   }
}

// ######################################################################
template <class T>
void getNormalizedLAB(const Image<PixRGB<T> >& src,
                      Image<typename promote_trait<T, float>::TP>& limg,
                      Image<typename promote_trait<T, float>::TP>& aimg,
                      Image<typename promote_trait<T, float>::TP>& bimg)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // just call get the LAB color conversion algorithm
  // then normalize 
  getLAB(src, limg,aimg,bimg);
  normalizeLAB(limg, aimg, bimg);
}

// ######################################################################
Image< PixRGB<byte> > contrastModulate(const Image< PixRGB<byte> >& img,
                                       const Image<float>& mask,
                                       float baseContrast,
                                       byte baseBright)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(img.isSameSize(mask));
  ASSERT((baseContrast >= 0.0) && (baseContrast <= 1.0));

  Image<float> contrast(mask);
  inplaceClamp(contrast, 0.01f, 1.0f);
  inplaceNormalize(contrast, 0.0f, 1.0f-baseContrast);
  contrast += baseContrast;
  Image<float> base(contrast * (-baseBright));
  base += baseBright;

  return img * contrast + base;
}

// ######################################################################
template <class T>
double pSNRcolor(const Image< PixRGB<T> >& img1,
                 const Image< PixRGB<T> >& img2)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(img1.isSameSize(img2));
  Image<T> r1, g1, b1, r2, g2, b2; int siz = img1.getSize();
  getComponents(img1, r1, g1, b1); getComponents(img2, r2, g2, b2);
  double mser = distance(r1, r2); mser = mser * mser / double(siz);
  double mseg = distance(g1, g2); mseg = mseg * mseg / double(siz);
  double mseb = distance(b1, b2); mseb = mseb * mseb / double(siz);

  double mse = (mser + mseg + mseb) / 3.0;

  return 10.0 * log10(255.0 * 255.0 / mse);
}

// ######################################################################
template <class T>
double pSNRcolor(const Image< PixRGB<T> >& img1,
                 const Image< PixRGB<T> >& img2,
                 const Image<float>& weight)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(img1.isSameSize(img2));
  Image<T> r1, g1, b1, r2, g2, b2; int siz = img1.getSize();
  getComponents(img1, r1, g1, b1); getComponents(img2, r2, g2, b2);
  double mser = distance(r1, r2, weight); mser = mser * mser / double(siz);
  double mseg = distance(g1, g2, weight); mseg = mseg * mseg / double(siz);
  double mseb = distance(b1, b2, weight); mseb = mseb * mseb / double(siz);

  double mse = (mser + mseg + mseb) / 3.0;

  return 10.0 * log10(255.0 * 255.0 / mse);
}

// ######################################################################
template <class T>
Image< PixRGB<T> > normalizeRGB(const Image< PixRGB<T> >& img,
                                PixRGB<T> min,
                                PixRGB<T> max)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<T> r,g,b;
  getComponents(img, r, g, b);
  inplaceNormalize(r, min.red(),   max.red());
  inplaceNormalize(g, min.green(), max.green());
  inplaceNormalize(b, min.blue(),  max.blue());
  return makeRGB(r, g, b);
}

// ######################################################################
template <class T>
Image<T> maxRGB(const Image< PixRGB<T> >& img)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<T> r,g,b;
  getComponents(img, r, g, b);
  return takeMax(r, takeMax(g, b));
}

// ######################################################################
template <class T>
Image< PixRGB<T> > colorStain (const Image<T>& src,
                               const T& min, const T& max,
                               const PixRGB<T>& color)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  float fmin = (float)min;
  float fdiff = (float)(max - min);
  ASSERT (fdiff != 0.0F);

  Image < PixRGB<T> > result(src.getDims(), NO_INIT);
  typename Image<T>::const_iterator sptr, stop = src.end();
  //typename Image< PixRGB<T> >::const_iterator stop = src.end();
  typename Image< PixRGB<T> >::iterator rptr = result.beginw();

  for (sptr = src.begin(); sptr != stop; ++sptr, ++rptr)
    {
      *rptr = color;
      *rptr *= ((float)(*sptr) - fmin) / fdiff;
    }
  return result;
}

// ######################################################################
void RGBtoCIE(const PixRGB<byte>& rgbColor,
              float& cr, float& cg, float& intens)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  PixRGB<float> fpix(rgbColor);
  intens = (fpix.red() + fpix.green() + fpix.blue());
  if (intens > 0.0f)
    {
      cr = fpix.red() / intens;
      cg = fpix.green() / intens;
    }
  else
    {
      cr = 0.0f;
      cg = 0.0f;
    }
}

// ######################################################################
Image<float> hueDistance(const Image< PixRGB<byte> >& img,
                         float muR, float muG,
                         float sigR, float sigG, float rho)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<float> result(img.getDims(), NO_INIT);
  Image< PixRGB<byte> >::const_iterator iptr, stop = img.end();
  Image<float>::iterator rptr = result.beginw();

  float sR2 = 1/sigR/sigR;
  float sG2 = 1/sigG/sigG;
  float sRG = 2*rho/sigG/sigR;

  for (iptr = img.begin(); iptr != stop; ++iptr, ++rptr)
    {
      float cr, cg, intens;
      RGBtoCIE(*iptr,cr,cg,intens);
      *rptr = exp(-0.5f*((cr-muR)*(cr-muR)*sR2 + (cg-muG)*(cg-muG)*sG2
                         - (cr-muR)*(cg-muG)*sRG));
    }
  return result;
}

template void getMinMaxC(const Image<PixRGB<int> >& src, int& mi, int& ma);
template void getMinMaxC(const Image<PixRGB<double> >& src, double& mi, double& ma);

// Include the explicit instantiations
#include "inst/Image/ColorOps.I"

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !IMAGE_COLOROPS_C_DEFINED

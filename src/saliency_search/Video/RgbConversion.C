/*!@file Video/RgbConversion.C Raw conversion between video formats and RGB images */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Video/RgbConversion.C $
// $Id: RgbConversion.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef VIDEO_RGBCONVERSION_C_DEFINED
#define VIDEO_RGBCONVERSION_C_DEFINED

#include "Video/RgbConversion.H"

#include "Image/JPEGUtil.H"
#include "Image/Dims.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/color_conversions.H" // for yv12_to_rgb24_c()
#include "Util/log.H"
#include "rutz/error_context.h" // for GVX_ERR_CONTEXT
#include "rutz/trace.h"
extern "C" {
#include "Video/MjpegDecoder/utils.h"
}
namespace
{
  // ######################################################################
  void checkBufferLength(const size_t actual, const size_t expected)
  {
    if (actual < expected)
      LFATAL("input buffer is too short (got %" ZU ", expected %" ZU ")",
             actual, expected);

    if (actual > expected)
      LINFO("input buffer is longer than expected (got %" ZU ", expected %" ZU ")\n"
            "(this is not a fatal error, but make sure the width and height are correct)",
            actual, expected);
  }
}

// ######################################################################
template <class T>
Image<PixRGB<T> > fromRGB(const T* data, const size_t length,
                          const Dims& dims, const bool byteswap)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  GVX_ERR_CONTEXT("converting from RGB to RGB");

  checkBufferLength(length, dims.sz() * 3);

  // dirty hack: assumes that we know the internal representation of PixRGB!
  Image< PixRGB<T> > dest(dims, NO_INIT);

  if (byteswap)
    {
      typename Image<PixRGB<T> >::iterator aptr = dest.beginw();
      typename Image<PixRGB<T> >::iterator stop = dest.endw();

      checkBufferLength(length, dims.sz() * 3);

      while(aptr != stop)
        {
          // data stored as: b, g, r
          (*aptr++).set(data[2], data[1], data[0]);
          data += 3;
        }
    }
  else
    {
      memcpy(dest.getArrayPtr(), data, dims.sz() * 3);
    }

  return dest;
}

// ######################################################################
template <class T>
Image<PixRGB<T> > fromARGB(const T* data, const size_t length,
                           const Dims& dims, const bool byteswap)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  GVX_ERR_CONTEXT("converting from ARGB to RGB");

  checkBufferLength(length, dims.sz() * 4);

  Image< PixRGB<T> > dest(dims, NO_INIT);
  typename Image<PixRGB<T> >::iterator aptr = dest.beginw();
  typename Image<PixRGB<T> >::iterator stop = dest.endw();

  if (byteswap)
    {
      while(aptr != stop)
        {
          // data stored as: b, g, r, alpha
          (*aptr++).set(data[2], data[1], data[0]);
          data += 4;
        }
    }
  else
    {
      while(aptr != stop)
        {
          // data stored as: alpha, r, g, b
          (*aptr++).set(data[1], data[2], data[3]);
          data += 4;
        }
    }

  return dest;
}

// ######################################################################
Image<PixRGB<byte> > fromRGB555(const byte* data, const size_t length,
                                const Dims& dims, const bool byteswap)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  GVX_ERR_CONTEXT("converting from RGB555 to RGB");

  checkBufferLength(length, dims.sz() * 2);

  Image<PixRGB<byte> > dest(dims, NO_INIT);
  Image<PixRGB<byte> >::iterator aptr = dest.beginw();
  Image<PixRGB<byte> >::iterator stop = dest.endw();

  if (byteswap)
    {
      while(aptr != stop)
        {
          // data stored as: 1 ignored bit, 5-bit R, 5-bit G, 5-bit B
          int rgb = (int(data[1]) << 8) | data[0];
          byte r = byte((rgb & 0x7C00) >> 7);
          byte g = byte((rgb & 0x03E0) >> 2);
          byte b = byte((rgb & 0x001F) << 3);
          (*aptr++).set(r, g, b);
          data += 2;
        }
    }
  else
    {
      while(aptr != stop)
        {
          // data stored as: 1 ignored bit, 5-bit R, 5-bit G, 5-bit B
          int rgb = (int(data[0]) << 8) | data[1];
          byte r = byte((rgb & 0x7C00) >> 7);
          byte g = byte((rgb & 0x03E0) >> 2);
          byte b = byte((rgb & 0x001F) << 3);
          (*aptr++).set(r, g, b);
          data += 2;
        }
    }

  return dest;
}

// ######################################################################
Image<PixRGB<byte> > fromRGB565(const byte* data, const size_t length,
                                const Dims& dims, const bool byteswap)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  GVX_ERR_CONTEXT("converting from RGB565 to RGB");

  checkBufferLength(length, dims.sz() * 2);

  Image< PixRGB<byte> > dest(dims, NO_INIT);
  Image<PixRGB<byte> >::iterator aptr = dest.beginw();
  Image<PixRGB<byte> >::iterator stop = dest.endw();

  if (byteswap)
    {
      while(aptr != stop)
        {
          // data stored as: 5-bit R, 6-bit G, 5-bit B
          int rgb = (int(data[1]) << 8) | data[0];
          byte r = byte((rgb & 0xF800) >> 8);
          byte g = byte((rgb & 0x07E0) >> 3);
          byte b = byte((rgb & 0x001F) << 3);
          (*aptr++).set(r, g, b);
          data += 2;
        }
    }
  else
    {
      while(aptr != stop)
        {
          // data stored as: 5-bit R, 6-bit G, 5-bit B
          int rgb = (int(data[0]) << 8) | data[1];
          byte r = byte((rgb & 0xF800) >> 8);
          byte g = byte((rgb & 0x07E0) >> 3);
          byte b = byte((rgb & 0x001F) << 3);
          (*aptr++).set(r, g, b);
          data += 2;
        }
    }

  return dest;
}

// ######################################################################
template <class T>
Image<PixRGB<T> > fromVideoYUV24(const T* data, const size_t length,
                                 const Dims& dims, const bool byteswap)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  GVX_ERR_CONTEXT("converting from YUV24 to RGB");

  checkBufferLength(length, dims.sz() * 3);

  Image< PixRGB<T> > dest(dims, NO_INIT);
  typename Image<PixRGB<T> >::iterator aptr = dest.beginw();
  typename Image<PixRGB<T> >::iterator stop = dest.endw();

  if (byteswap)
    {
      while (aptr != stop)
        {
          // data stored as: y0, v0, u0, v1, v1, u1
          (*aptr++) = PixRGB<T>(PixVideoYUV<double>(data[0], data[2], data[1]));
          (*aptr++) = PixRGB<T>(PixVideoYUV<double>(data[3], data[5], data[4]));
          data += 6;
        }
    }
  else
    {
      while (aptr != stop)
        {
          // data stored as: y0, u0, v0, v1, u1, v1
          (*aptr++) = PixRGB<T>(PixVideoYUV<double>(data[0], data[1], data[2]));
          (*aptr++) = PixRGB<T>(PixVideoYUV<double>(data[3], data[4], data[5]));
          data += 6;
        }
    }

  return dest;
}

// ######################################################################
template <class T>
Image<PixRGB<T> > fromVideoYUV444(const T* data, const size_t length,
                                  const Dims& dims, const bool byteswap)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  GVX_ERR_CONTEXT("converting from YUV444 to RGB");

  checkBufferLength(length, dims.sz() * 3);

  Image< PixRGB<T> > dest(dims, NO_INIT);
  typename Image<PixRGB<T> >::iterator aptr = dest.beginw();
  typename Image<PixRGB<T> >::iterator stop = dest.endw();

  if (byteswap)
    {
      while(aptr != stop)
        {
          // data stored as: y0, u0, u1, v0, v1, y1
          (*aptr++) = PixRGB<T>(PixVideoYUV<double>(data[0], data[1], data[3]));
          (*aptr++) = PixRGB<T>(PixVideoYUV<double>(data[5], data[2], data[4]));
          data += 6;
        }
    }
  else
    {
      while(aptr != stop)
        {
          // data stored as: u0, y0, v0, u1, y1, v1
          (*aptr++) = PixRGB<T>(PixVideoYUV<double>(data[1], data[0], data[2]));
          (*aptr++) = PixRGB<T>(PixVideoYUV<double>(data[4], data[3], data[5]));
          data += 6;
        }
    }

  return dest;
}

// ######################################################################
template <>
Image<PixRGB<byte> > fromVideoYUV422(const byte* data, const size_t length,
                                     const Dims& dims, const bool byteswap)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  GVX_ERR_CONTEXT("converting from YUV422(byte) to RGB");

  checkBufferLength(length, dims.sz() * 2);

  Image< PixRGB<byte> > dest(dims, NO_INIT);

  yuv422_to_rgb24_c(reinterpret_cast<byte*>(dest.getArrayPtr()),
                    dims.w(), dims.h(), data, byteswap);

  return dest;
}

// ######################################################################
template <class T>
Image<PixRGB<T> > fromVideoYUV422(const T* data, const size_t length,
                                  const Dims& dims, const bool byteswap)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  GVX_ERR_CONTEXT("converting from YUV422 to RGB");

  checkBufferLength(length, dims.sz() * 2);

  Image< PixRGB<T> > dest(dims, NO_INIT);
  typename Image<PixRGB<T> >::iterator aptr = dest.beginw();
  typename Image<PixRGB<T> >::iterator stop = dest.endw();

  if (byteswap)
    {
      while(aptr != stop)
        {
          // data stored as:  y0, u, y1, v
          (*aptr++) = PixRGB<T>(PixVideoYUV<double>(data[0], data[1], data[3]));
          (*aptr++) = PixRGB<T>(PixVideoYUV<double>(data[2], data[1], data[3]));
          data += 4;
        }
    }
  else
    {
      while(aptr != stop)
        {
          // data stored as: u, y0, v, y1
          (*aptr++) = PixRGB<T>(PixVideoYUV<double>(data[1], data[0], data[2]));
          (*aptr++) = PixRGB<T>(PixVideoYUV<double>(data[3], data[0], data[2]));
          data += 4;
        }
    }

  return dest;
}

// ######################################################################
template <class T>
Image<PixRGB<T> > fromVideoYUV411(const T* data, const size_t length,
                                  const Dims& dims, const bool byteswap)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  GVX_ERR_CONTEXT("converting from YUV411 to RGB");

  checkBufferLength(length, dims.sz() * 3 / 2);

  Image< PixRGB<T> > dest(dims, NO_INIT);
  typename Image<PixRGB<T> >::iterator aptr = dest.beginw();
  typename Image<PixRGB<T> >::iterator stop = dest.endw();

  if (byteswap)
    {
      while(aptr != stop)
        {
          // data stored as: y0, u, v, y1, y3, y2
          (*aptr++) = PixRGB<T>(PixVideoYUV<double>(data[0], data[1], data[2]));
          (*aptr++) = PixRGB<T>(PixVideoYUV<double>(data[3], data[1], data[2]));
          (*aptr++) = PixRGB<T>(PixVideoYUV<double>(data[5], data[1], data[2]));
          (*aptr++) = PixRGB<T>(PixVideoYUV<double>(data[4], data[1], data[2]));
          data += 6;
        }
    }
  else
    {
      while(aptr != stop)
        {
          // data stored as: u, y0, y1, v, y2, y3
          (*aptr++) = PixRGB<T>(PixVideoYUV<double>(data[1], data[0], data[3]));
          (*aptr++) = PixRGB<T>(PixVideoYUV<double>(data[2], data[0], data[3]));
          (*aptr++) = PixRGB<T>(PixVideoYUV<double>(data[4], data[0], data[3]));
          (*aptr++) = PixRGB<T>(PixVideoYUV<double>(data[5], data[0], data[3]));
          data += 6;
        }
    }

  return dest;
}

// ######################################################################
template <class T>
Image<PixRGB<T> > fromVideoYUV444P(const T* data, const size_t length,
                                   const Dims& dims)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  GVX_ERR_CONTEXT("converting from YUV444P to RGB");

  checkBufferLength(length, dims.sz() * 3);

  Image< PixRGB<T> > dest(dims, NO_INIT);
  int w = dims.w(), h = dims.h();
  typename Image<PixRGB<T> >::iterator aptr = dest.beginw();
  const T *yptr = data;
  const T *uptr = yptr + w * h;
  const T *vptr = uptr + w * h;

  for (int j = 0; j < h; j ++)
    for (int i = 0; i < w; i ++)
      {
        // we have a 1 luminance pixels per chroma pair
        (*aptr++) = PixRGB<T>(PixVideoYUV<double>(*yptr++, *uptr++, *vptr++));
      }

  return dest;
}

// ######################################################################
template <>
Image<PixRGB<byte> > fromVideoYUV422P(const byte* data, const size_t length,
                                      const Dims& dims)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  GVX_ERR_CONTEXT("converting from YUV422P(byte) to RGB");

  const int w = dims.w();
  const int h = dims.h();

  checkBufferLength(length, dims.sz() + 2*(w/2)*h);

  Image<PixRGB<byte> > dest(dims, NO_INIT);
  const byte* yptr = data;
  const byte* uptr = yptr + w * h;
  const byte* vptr = uptr + (w/2) * h;

  yuv422p_to_rgb24_c(reinterpret_cast<byte*>(dest.getArrayPtr()),
                     w, h,
                     yptr, uptr, vptr);

  return dest;
}

// ######################################################################
template <class T>
Image<PixRGB<T> > fromVideoYUV422P(const T* data, const size_t length,
                                   const Dims& dims)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  GVX_ERR_CONTEXT("converting from YUV422P to RGB");

  const int w = dims.w();
  const int h = dims.h();

  checkBufferLength(length, dims.sz() + 2*(w/2)*h);

  Image< PixRGB<T> > dest(dims, NO_INIT);
  typename Image<PixRGB<T> >::iterator aptr = dest.beginw();
  const T *yptr = data;
  const T *uptr = yptr + w * h;
  const T *vptr = uptr + (w/2) * h;

  for (int j = 0; j < h; j ++)
    for (int i = 0; i < w; i += 2)
      {
        // we have 2 luminance pixels per chroma pair

        const double yf1 = double(*yptr++) - VIDEOYUV_Y_OFFSET;
        const double uf = double(*uptr++) - VIDEOYUV_UV_OFFSET;
        const double vf = double(*vptr++) - VIDEOYUV_UV_OFFSET;

        const double rf1 = VIDEOYUV_RGB_Y*yf1                   + VIDEOYUV_R_V*vf;
        const double gf1 = VIDEOYUV_RGB_Y*yf1 + VIDEOYUV_G_U*uf + VIDEOYUV_G_V*vf;
        const double bf1 = VIDEOYUV_RGB_Y*yf1 + VIDEOYUV_B_U*uf;

        aptr->p[0] = clamped_rounded_convert<T>(clampValue(rf1, 0.0, 255.0));
        aptr->p[1] = clamped_rounded_convert<T>(clampValue(gf1, 0.0, 255.0));
        aptr->p[2] = clamped_rounded_convert<T>(clampValue(bf1, 0.0, 255.0));
        ++aptr;

        const double yf2 = double(*yptr++) - VIDEOYUV_Y_OFFSET;

        const double rf2 = VIDEOYUV_RGB_Y*yf2                   + VIDEOYUV_R_V*vf;
        const double gf2 = VIDEOYUV_RGB_Y*yf2 + VIDEOYUV_G_U*uf + VIDEOYUV_G_V*vf;
        const double bf2 = VIDEOYUV_RGB_Y*yf2 + VIDEOYUV_B_U*uf;

        aptr->p[0] = clamped_rounded_convert<T>(clampValue(rf2, 0.0, 255.0));
        aptr->p[1] = clamped_rounded_convert<T>(clampValue(gf2, 0.0, 255.0));
        aptr->p[2] = clamped_rounded_convert<T>(clampValue(bf2, 0.0, 255.0));
        ++aptr;
      }

  return dest;
}

// ######################################################################
template <class T>
Image<PixRGB<T> > fromVideoYUV411P(const T* data, const size_t length,
                                   const Dims& dims)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  GVX_ERR_CONTEXT("converting from YUV411P to RGB");

  const int w = dims.w();
  const int h = dims.h();

  checkBufferLength(length, dims.sz() + 2*(w/4)*h);

  Image< PixRGB<T> > dest(dims, NO_INIT);
  typename Image<PixRGB<T> >::iterator aptr = dest.beginw();
  const T *yptr = data;
  const T *uptr = yptr + w * h;
  const T *vptr = uptr + (w/4) * h;

  for (int j = 0; j < h; j ++)
    for (int i = 0; i < w; i += 4)
      {
        // we have a 4 luminance pixels per chroma pair
        (*aptr++) = PixRGB<T>(PixVideoYUV<double>(*yptr++, *uptr, *vptr));
        (*aptr++) = PixRGB<T>(PixVideoYUV<double>(*yptr++, *uptr, *vptr));
        (*aptr++) = PixRGB<T>(PixVideoYUV<double>(*yptr++, *uptr, *vptr));
        (*aptr++) = PixRGB<T>(PixVideoYUV<double>(*yptr++, *uptr++, *vptr++));
      }

  return dest;
}

// ######################################################################
template <>
Image<PixRGB<byte> > fromVideoYUV420P(const byte* data, const size_t length,
                                      const Dims& dims)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // Here we have a specialization of fromVideoYUV420P that uses the
  // more optimized implementation from yv12_to_rgb24_c(). However,
  // both the straightforward implementation of fromVideoYUV422P and
  // the optimized implementation use the same conversion factors
  // (VIDEOYUV_RGB_Y, VIDEOYUV_R_V, etc.)  given in Image/colorDefs.H,
  // so either implementation should give the same result.

  GVX_ERR_CONTEXT("converting from YUV420P(byte) to RGB");

  const int w = dims.w();
  const int h = dims.h();
  // we have to do (w+1)/2 instead of just w/2, because if e.g. the y
  // array has 5 pixels, then we want the u and v arrays to have 3
  // pixels, not 2:
  const int w2 = (w+1)/2;
  const int h2 = (h+1)/2;

  checkBufferLength(length, dims.sz() + 2*w2*h2);

  Image< PixRGB<byte> > dest(dims, NO_INIT);

  yv12_to_rgb24_c(reinterpret_cast<byte*>(dest.getArrayPtr()),
                  w /* dst_stride */,
                  data,
                  data + w*h,
                  data + w*h + w2*h2,
                  w /* y_stride */,
                  w2 /* uv_stride */,
                  w /* image width */,
                  h /* image height */);

  return dest;
}


// ######################################################################
template <class T>
Image<PixRGB<T> > fromVideoYUV420P(const T* data, const size_t length,
                                   const Dims& dims)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  GVX_ERR_CONTEXT("converting from YUV420P to RGB");

  const int w = dims.w();
  const int h = dims.h();
  // we have to do (w+1)/2 instead of just w/2, because if e.g. the y
  // array has 5 pixels, then we want the u and v arrays to have 3
  // pixels, not 2:
  const int w2 = (w+1)/2;
  const int h2 = (h+1)/2;

  checkBufferLength(length, dims.sz() + 2*w2*h2);

  Image< PixRGB<T> > dest(dims, NO_INIT);
  typename Image<PixRGB<T> >::iterator aptr = dest.beginw();
  const T *yptr = data;
  const T *uptr = yptr + w * h;
  const T *vptr = uptr + (w/2) * (h/2);

  for (int j = 0; j < h; j += 2)
    {
      for (int i = 0; i < w; i += 2)
        {
          T u = *uptr++, v = *vptr++;
          // we have a 2x2 luminance block per chroma pair
          (*aptr) = PixRGB<T>(PixVideoYUV<double>(*yptr, u, v));
          aptr[1] = PixRGB<T>(PixVideoYUV<double>(yptr[1], u, v));
          aptr[w] = PixRGB<T>(PixVideoYUV<double>(yptr[w], u, v));
          aptr[w+1] = PixRGB<T>(PixVideoYUV<double>(yptr[w + 1], u, v));
          aptr += 2; yptr += 2;
        }
      aptr += w; yptr += w;
    }

  return dest;
}

// ######################################################################
        template <>
Image<PixRGB<byte> > fromVideoHM12(const byte* data, const size_t length,
                const Dims& dims)
{
        GVX_TRACE(__PRETTY_FUNCTION__);

        // Here we have a specialization of fromVideoHM12 that uses the
        // more optimized implementation from yv12_to_rgb24_c(). However,
        // both the straightforward implementation of fromVideoYUV422P and
        // the optimized implementation use the same conversion factors
        // (VIDEOYUV_RGB_Y, VIDEOYUV_R_V, etc.)  given in Image/colorDefs.H,
        // so either implementation should give the same result.

        GVX_ERR_CONTEXT("converting from HM12(byte) to RGB");

        const int w = dims.w();
        const int h = dims.h();
        checkBufferLength(length, w*h*3/2);

        Image< PixRGB<byte> > dest(dims, NO_INIT);
        Image<PixRGB<byte> >::iterator aptr = dest.beginw();
        const byte *yptr = data;
        const byte *uvptr = yptr + w * h;

        unsigned char frameu[w*h / 4];
        unsigned char framev[w*h / 4];

        // descramble U/V plane
        // dstride = 720 / 2 = w
        // The U/V values are interlaced (UVUV...).
        // Again, the UV plane is divided into blocks of 16x16 UV values.
        // Each block in transmitted in turn, line-by-line.
        for (int y = 0; y < h/2; y += 16) {
                for (int x = 0; x < w/2; x += 8) {
                        for (int i = 0; i < 16; i++) {
                                int idx = x + (y + i) * (w/2);

                                frameu[idx+0] = uvptr[0];  framev[idx+0] = uvptr[1];
                                frameu[idx+1] = uvptr[2];  framev[idx+1] = uvptr[3];
                                frameu[idx+2] = uvptr[4];  framev[idx+2] = uvptr[5];
                                frameu[idx+3] = uvptr[6];  framev[idx+3] = uvptr[7];
                                frameu[idx+4] = uvptr[8];  framev[idx+4] = uvptr[9];
                                frameu[idx+5] = uvptr[10]; framev[idx+5] = uvptr[11];
                                frameu[idx+6] = uvptr[12]; framev[idx+6] = uvptr[13];
                                frameu[idx+7] = uvptr[14]; framev[idx+7] = uvptr[15];
                                uvptr += 16;
                        }
                }
        }

        for (int y = 0; y < h; y += 16)
        {
                for (int x = 0; x < w; x += 16)
                {
                        ////the Y plane is divided into blocks of 16x16
                        for(int i=0; i<16; i++)
                        {
                                for(int j=0; j<16; j++)
                                {
                                        int idx = (x/2)+ (((y/2)+i/2)*w/2) + j/2;
                                        *(aptr+x+(y+i)*w+j) = PixRGB<byte>(PixVideoYUV<double>(*(yptr+j), frameu[idx], framev[idx]));
                                }
                                yptr+=16;
                        }
                }
        }

  return dest;

}

// ######################################################################
template <class T>
Image<PixRGB<T> > fromVideoHM12(const T* data, const size_t length,
                                   const Dims& dims)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  GVX_ERR_CONTEXT("converting from YUV420P to RGB");

  const int w = dims.w();
  const int h = dims.h();
  // we have to do (w+1)/2 instead of just w/2, because if e.g. the y
  // array has 5 pixels, then we want the u and v arrays to have 3
  // pixels, not 2:
  const int w2 = (w+1)/2;
  const int h2 = (h+1)/2;

  checkBufferLength(length, dims.sz() + 2*w2*h2);

  Image< PixRGB<T> > dest(dims, NO_INIT);
  typename Image<PixRGB<T> >::iterator aptr = dest.beginw();
  const T *yptr = data;
  const T *uptr = yptr + w * h;
  const T *vptr = uptr + (w/2) * (h/2);

  for (int j = 0; j < h; j += 2)
    {
      for (int i = 0; i < w; i += 2)
        {
          T u = *uptr++, v = *vptr++;
          // we have a 2x2 luminance block per chroma pair
          (*aptr) = PixRGB<T>(PixVideoYUV<double>(*yptr, u, v));
          aptr[1] = PixRGB<T>(PixVideoYUV<double>(yptr[1], u, v));
          aptr[w] = PixRGB<T>(PixVideoYUV<double>(yptr[w], u, v));
          aptr[w+1] = PixRGB<T>(PixVideoYUV<double>(yptr[w + 1], u, v));
          aptr += 2; yptr += 2;
        }
      aptr += w; yptr += w;
    }

  return dest;
}

// ######################################################################
template <class T>
Image<PixRGB<T> > fromVideoMJPEG(const T* data, const size_t length, 
																 const Dims& dims, const bool byteswap)
{
	int frameSize = dims.w()* (dims.h() + 8)*2;
	int w = dims.w(),h = dims.h();

	unsigned char *outputframe = (unsigned char *) calloc(1, (size_t) frameSize);

	//use luvcview's decoder to convert MJPEG to YUV422
	jpeg_decode(&outputframe,(unsigned char *)data,&w,&h);

	Image<PixRGB<byte> > dest(w, h, ZEROS);
	//then convert the image from YUV422 to RGB24 
  yuv422_to_rgb24_c(reinterpret_cast<byte*>(dest.getArrayPtr()),
                    dims.w(), dims.h(), outputframe, byteswap);
  return dest;
}


// ######################################################################
template <class T>
Image<PixRGB<T> > fromVideoYUV410P(const T* data, const size_t length,
                                   const Dims& dims)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  GVX_ERR_CONTEXT("converting from YUV410P to RGB");

  checkBufferLength(length, dims.sz() * 9 / 8);

  Image< PixRGB<T> > dest(dims, NO_INIT);
  int w = dims.w(), h = dims.h(); int w2 = w * 2, w3 = w * 3;
  typename Image<PixRGB<T> >::iterator aptr = dest.beginw();
  const T *yptr = data;
  const T *uptr = yptr + w * h;
  const T *vptr = uptr + (w/4) * (h/4);

  for (int j = 0; j < h; j += 4)
    {
      for (int i = 0; i < w; i += 4)
        {
          T u = *uptr++, v = *vptr++;

          // we have a 4x4 luminance block per chroma pair
          (*aptr) = PixRGB<T>(PixVideoYUV<double>(*yptr, u, v));
          aptr[1] = PixRGB<T>(PixVideoYUV<double>(yptr[1], u, v));
          aptr[2] = PixRGB<T>(PixVideoYUV<double>(yptr[2], u, v));
          aptr[3] = PixRGB<T>(PixVideoYUV<double>(yptr[3], u, v));

          aptr[w] = PixRGB<T>(PixVideoYUV<double>(yptr[w], u, v));
          aptr[w+1] = PixRGB<T>(PixVideoYUV<double>(yptr[w+1], u, v));
          aptr[w+2] = PixRGB<T>(PixVideoYUV<double>(yptr[w+2], u, v));
          aptr[w+3] = PixRGB<T>(PixVideoYUV<double>(yptr[w+3], u, v));

          aptr[w2] = PixRGB<T>(PixVideoYUV<double>(yptr[w2], u, v));
          aptr[w2+1] = PixRGB<T>(PixVideoYUV<double>(yptr[w2+1], u, v));
          aptr[w2+2] = PixRGB<T>(PixVideoYUV<double>(yptr[w2+2], u, v));
          aptr[w2+3] = PixRGB<T>(PixVideoYUV<double>(yptr[w2+3], u, v));

          aptr[w3] = PixRGB<T>(PixVideoYUV<double>(yptr[w3], u, v));
          aptr[w3+1] = PixRGB<T>(PixVideoYUV<double>(yptr[w3+1], u, v));
          aptr[w3+2] = PixRGB<T>(PixVideoYUV<double>(yptr[w3+2], u, v));
          aptr[w3+3] = PixRGB<T>(PixVideoYUV<double>(yptr[w3+3], u, v));

          aptr += 4; yptr += 4;
        }
      aptr += w3; yptr += w3;
    }

  return dest;
}

// ######################################################################
template <class T>
Image<PixRGB<T> > fromMono(const T* data, const size_t length,
                           const Dims& dims)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  GVX_ERR_CONTEXT("converting from mono to RGB");

  checkBufferLength(length, dims.sz());

  Image< PixRGB<T> > dest(dims, NO_INIT);
  typename Image<PixRGB<T> >::iterator aptr = dest.beginw();
  typename Image<PixRGB<T> >::iterator stop = dest.endw();

  T m;
  while(aptr != stop)
    {
      m = (*data++); (*aptr++).set(m, m, m);
    }

  return dest;
}

// ######################################################################
Image<PixRGB<byte> > fromBayer(const byte* data, const size_t length,
                            const Dims& dims, BayerFormat ft)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  GVX_ERR_CONTEXT("converting from bayer_GB to RGB");

  checkBufferLength(length, dims.sz());

  Image<byte> src(data,dims);
  Image<PixRGB<byte> > dest(dims, NO_INIT);
  dest = deBayer(src, ft);

  return dest;
}

// ######################################################################
Image<PixRGB<uint16> > fromBayerU16(const uint16* data, const size_t length,
                            const Dims& dims, BayerFormat ft)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  GVX_ERR_CONTEXT("converting from bayer_GB12 to RGB");

  checkBufferLength(length, dims.sz()*sizeof(uint16));

  Image<uint16> src(data,dims);
  Image<PixRGB<uint16> > dest(dims, NO_INIT);
  dest = deBayer(src, ft);

  return dest;
}

// ######################################################################
void toVideoYUV422(const Image<PixRGB<byte> > img,
                   byte* y, byte *u, byte *v,
                   const int ystride,
                   const int ustride,
                   const int vstride)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const byte* src = reinterpret_cast<const byte*>(img.getArrayPtr());
  int w = img.getWidth(), h = img.getHeight();

  for (int j = 0; j < h; j += 2)
    {
      // NOTE: the formulas here is in Pixels.H, but we don't use the
      // functions from there just for higher speed here.

      // NOTE: this code will probably not work with odd image or sizes

      for (int i = 0; i < w; i += 2)
        {
          // fully convert first RGB pixel:
          double r = double(*src++);
          double g = double(*src++);
          double b = double(*src++);

          double yf = VIDEOYUV_Y_R*r + VIDEOYUV_Y_G*g + VIDEOYUV_Y_B*b;
          double uf = VIDEOYUV_U_R*r + VIDEOYUV_U_G*g + VIDEOYUV_U_B*b + VIDEOYUV_UV_OFFSET;
          double vf = VIDEOYUV_V_R*r + VIDEOYUV_V_G*g + VIDEOYUV_V_B*b + VIDEOYUV_UV_OFFSET;

          *y++ = clamped_rounded_convert<byte>(yf);
          *u++ = clamped_rounded_convert<byte>(uf);
          *v++ = clamped_rounded_convert<byte>(vf);

          // only get luminance for second RGB pixel:
          r = double(*src++);
          g = double(*src++);
          b = double(*src++);

          yf =  VIDEOYUV_Y_R*r + VIDEOYUV_Y_G*g + VIDEOYUV_Y_B*b;
          *y++ = clamped_rounded_convert<byte>(yf);
        }
      y += ystride; u += ustride; v += vstride;
      for (int i = 0; i < w; i += 2)
        {
          // only get luminance for third RGB pixel:
          double r = double(*src++);
          double g = double(*src++);
          double b = double(*src++);

          double yf =  VIDEOYUV_Y_R*r + VIDEOYUV_Y_G*g + VIDEOYUV_Y_B*b;
          *y++ = clamped_rounded_convert<byte>(yf);

          // only get luminance for fourth RGB pixel:
          r = double(*src++);
          g = double(*src++);
          b = double(*src++);

          yf =  VIDEOYUV_Y_R*r + VIDEOYUV_Y_G*g + VIDEOYUV_Y_B*b;
          *y++ = clamped_rounded_convert<byte>(yf);
        }
      y += ystride;
    }
}

// Include the explicit instantiations
#include "inst/Video/RgbConversion.I"

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // VIDEO_RGBCONVERSION_C_DEFINED

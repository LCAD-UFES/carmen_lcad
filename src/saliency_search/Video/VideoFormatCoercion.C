/*!@file Video/VideoFormatCoercion.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Video/VideoFormatCoercion.C $
// $Id: VideoFormatCoercion.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef VIDEO_VIDEOFORMATCOERCION_C_DEFINED
#define VIDEO_VIDEOFORMATCOERCION_C_DEFINED

#include "Video/VideoFormatCoercion.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Util/log.H"
#include "Util/sformat.H"
#include "Video/VideoFrame.H"
#include "Video/RgbConversion.H"

// ######################################################################
static void checkBufferLength(const size_t actual, const size_t expected)
{
  if (actual < expected)
    LFATAL("input buffer is too short (got %" ZU ", expected %" ZU ")",
           actual, expected);

  if (actual > expected)
    LINFO("input buffer is longer than expected (got %" ZU ", expected %" ZU ")\n"
          "(this is not a fatal error, but make sure the width and height are correct)",
          actual, expected);
}

// ######################################################################
// ######################################################################
// conversions to RGB24
// ######################################################################
// ######################################################################

// ######################################################################
VideoFrame RGB_to_RGB24(const byte* sptr, const size_t length,
                        const Dims& dims, const bool byteswap)
{
  return VideoFrame(fromRGB(sptr, length, dims, byteswap));
}

// ######################################################################
VideoFrame ARGB_to_RGB24(const byte* sptr, const size_t length,
                         const Dims& dims, const bool byteswap)
{
  return VideoFrame(fromARGB(sptr, length, dims, byteswap));
}

// ######################################################################
VideoFrame RGB555_to_RGB24(const byte* sptr, const size_t length,
                           const Dims& dims, const bool byteswap)
{
  return VideoFrame(fromRGB555(sptr, length, dims, byteswap));
}

// ######################################################################
VideoFrame RGB565_to_RGB24(const byte* sptr, const size_t length,
                           const Dims& dims, const bool byteswap)
{
  return VideoFrame(fromRGB565(sptr, length, dims, byteswap));
}

// ######################################################################
VideoFrame YUV24_to_RGB24(const byte* sptr, const size_t length,
                          const Dims& dims, const bool byteswap)
{
  return VideoFrame(fromVideoYUV24(sptr, length, dims, byteswap));
}

// ######################################################################
VideoFrame YUV444_to_RGB24(const byte* sptr, const size_t length,
                           const Dims& dims, const bool byteswap)
{
  return VideoFrame(fromVideoYUV444(sptr, length, dims, byteswap));
}

// ######################################################################
VideoFrame YUYV_to_RGB24(const byte* sptr, const size_t length,
                         const Dims& dims, const bool byteswap)
{
  return VideoFrame(fromVideoYUV422(sptr, length, dims, !byteswap));
}

// ######################################################################
VideoFrame YUV422_to_RGB24(const byte* sptr, const size_t length,
                           const Dims& dims, const bool byteswap)
{
  return VideoFrame(fromVideoYUV422(sptr, length, dims, byteswap));
}

// ######################################################################
VideoFrame YUV411_to_RGB24(const byte* sptr, const size_t length,
                           const Dims& dims, const bool byteswap)
{
  return VideoFrame(fromVideoYUV411(sptr, length, dims, byteswap));
}

// ######################################################################
VideoFrame YUV444P_to_RGB24(const byte* sptr, const size_t length,
                            const Dims& dims, const bool byteswap)
{
  return VideoFrame(fromVideoYUV444P(sptr, length, dims));
}

// ######################################################################
VideoFrame YUV422P_to_RGB24(const byte* sptr, const size_t length,
                            const Dims& dims, const bool byteswap)
{
  return VideoFrame(fromVideoYUV422P(sptr, length, dims));
}

// ######################################################################
VideoFrame YUV411P_to_RGB24(const byte* sptr, const size_t length,
                            const Dims& dims, const bool byteswap)
{
  return VideoFrame(fromVideoYUV411P(sptr, length, dims));
}

// ######################################################################
VideoFrame YUV420P_to_RGB24(const byte* sptr, const size_t length,
                            const Dims& dims, const bool byteswap)
{
  return VideoFrame(fromVideoYUV420P(sptr, length, dims));
}

// ######################################################################
VideoFrame YUV410P_to_RGB24(const byte* sptr, const size_t length,
                            const Dims& dims, const bool byteswap)
{
  return VideoFrame(fromVideoYUV410P(sptr, length, dims));
}

// ######################################################################
VideoFrame GREY_to_RGB24(const byte* sptr, const size_t length,
                         const Dims& dims, const bool byteswap)
{
  return VideoFrame(fromMono(sptr, length, dims));
}




// ######################################################################
// ######################################################################
// conversions from RGB24
// ######################################################################
// ######################################################################

// ######################################################################
VideoFrame RGB24_to_GREY(const byte* sptr, const size_t length,
                         const Dims& dims, const bool byteswap)
{
  checkBufferLength(length, dims.sz() * 3);

  Image<byte> dst(dims, NO_INIT);

  Image<byte>::iterator dptr = dst.beginw();
  Image<byte>::iterator stop = dst.endw();

  while (dptr != stop)
    {
      // byteswap does not matter here since (R+G+B)/3 == (B+G+R)/3
      *dptr++ = byte(0.5 + (sptr[0] + sptr[1] + sptr[2]) / 3.0);
      sptr += 3;
    }

  return VideoFrame(dst);
}

// ######################################################################
VideoFrame RGB24_to_RGB555(const byte* sptr, const size_t length,
                           const Dims& dims, const bool byteswap)
{
  checkBufferLength(length, dims.sz() * 3);

  ArrayHandle<byte> dst(new ArrayData<byte>(Dims(dims.sz() * 2, 1), NO_INIT));
  byte* dptr = dst.uniq().dataw();
  byte* stop = dst.uniq().endw();

  if (byteswap)
    while (dptr != stop)
      {
        // data stored as: 1 ignored bit, 5-bit R, 5-bit G, 5-bit B
        const int r = sptr[2];
        const int g = sptr[1];
        const int b = sptr[0];

        const uint16 val = ((r >> 3) << 10) + ((g >> 3) << 5) + (b >> 3);
        // big-endian:
        *dptr++ = (val & 0xff00) >> 8;
        *dptr++ = (val & 0x00ff);
        sptr += 3;
      }
  else
    while (dptr != stop)
      {
        // data stored as: 1 ignored bit, 5-bit R, 5-bit G, 5-bit B
        const int r = sptr[0];
        const int g = sptr[1];
        const int b = sptr[2];

        const uint16 val = ((r >> 3) << 10) + ((g >> 3) << 5) + (b >> 3);
        // big-endian:
        *dptr++ = (val & 0xff00) >> 8;
        *dptr++ = (val & 0x00ff);
        sptr += 3;
      }

  return VideoFrame(dst, dims, VIDFMT_RGB555, false);
}

// ######################################################################
VideoFrame RGB24_to_RGB565(const byte* sptr, const size_t length,
                           const Dims& dims, const bool byteswap)
{
  checkBufferLength(length, dims.sz() * 3);

  ArrayHandle<byte> dst(new ArrayData<byte>(Dims(dims.sz() * 2, 1), NO_INIT));
  byte* dptr = dst.uniq().dataw();
  byte* stop = dst.uniq().endw();

  if (byteswap)
    while (dptr != stop)
      {
        // data stored as: 5-bit R, 6-bit G, 5-bit B
        const int r = sptr[2];
        const int g = sptr[1];
        const int b = sptr[0];

        const uint16 val = ((r >> 3) << 11) + ((g >> 2) << 5) + (b >> 3);
        // big-endian:
        *dptr++ = (val & 0xff00) >> 8;
        *dptr++ = (val & 0x00ff);
        sptr += 3;
      }
  else
    while (dptr != stop)
      {
        // data stored as: 5-bit R, 6-bit G, 5-bit B
        const int r = sptr[0];
        const int g = sptr[1];
        const int b = sptr[2];

        const uint16 val = ((r >> 3) << 11) + ((g >> 2) << 5) + (b >> 3);
        // big-endian:
        *dptr++ = (val & 0xff00) >> 8;
        *dptr++ = (val & 0x00ff);
        sptr += 3;
      }

  return VideoFrame(dst, dims, VIDFMT_RGB565, false);
}

// ######################################################################
VideoFrame RGB24_to_RGB32(const byte* sptr, const size_t length,
                          const Dims& dims, const bool byteswap)
{
  checkBufferLength(length, dims.sz() * 3);

  ArrayHandle<byte> dst(new ArrayData<byte>(Dims(dims.sz() * 4, 1), NO_INIT));
  byte* dptr = dst.uniq().dataw();
  byte* stop = dst.uniq().endw();

  if (byteswap)
    while (dptr != stop)
      {
        *dptr++ = 0;
        *dptr++ = sptr[2];
        *dptr++ = sptr[1];
        *dptr++ = sptr[0];
        sptr += 3;
      }
  else
    while (dptr != stop)
      {
        *dptr++ = 0;
        *dptr++ = sptr[0];
        *dptr++ = sptr[1];
        *dptr++ = sptr[2];
        sptr += 3;
      }

  return VideoFrame(dst, dims, VIDFMT_RGB32, false);
}

// ######################################################################
// ######################################################################
// conversions to YUV24
// ######################################################################
// ######################################################################

// ######################################################################
VideoFrame GREY_to_YUV24(const byte* sptr, const size_t length,
                         const Dims& dims, const bool byteswap)
{
  const int sz = dims.sz();

  checkBufferLength(length, sz);

  Image<PixVideoYUV<byte> > dst(dims, NO_INIT);
  Image<PixVideoYUV<byte> >::iterator dptr = dst.beginw();

  for (int i = 0; i < sz; ++i)
    *dptr++ = PixVideoYUV<byte>(PixRGB<double>(sptr[i], sptr[i], sptr[i]));

  ASSERT(dptr == dst.endw());

  return VideoFrame(dst);
}

// ######################################################################
VideoFrame RGB24_to_YUV24(const byte* sptr, const size_t length,
                          const Dims& dims, const bool byteswap)
{
  const int sz = dims.sz() * 3;

  checkBufferLength(length, sz);

  Image<PixVideoYUV<byte> > dst(dims, NO_INIT);
  Image<PixVideoYUV<byte> >::iterator dptr = dst.beginw();

  if (byteswap)
    for (int i = 0; i < sz; i += 3)
      *dptr++ = PixVideoYUV<byte>(PixRGB<double>(sptr[i+2], sptr[i+1], sptr[i]));
  else
    for (int i = 0; i < sz; i += 3)
      *dptr++ = PixVideoYUV<byte>(PixRGB<double>(sptr[i], sptr[i+1], sptr[i+2]));

  ASSERT(dptr == dst.endw());

  return VideoFrame(dst);
}

// ######################################################################
VideoFrame RGB32_to_YUV24(const byte* sptr, const size_t length,
                          const Dims& dims, const bool byteswap)
{
  const int sz = dims.sz() * 4;

  checkBufferLength(length, sz);

  Image<PixVideoYUV<byte> > dst(dims, NO_INIT);
  Image<PixVideoYUV<byte> >::iterator dptr = dst.beginw();

  if (byteswap)
    for (int i = 0; i < sz; i += 4)
      *dptr++ = PixVideoYUV<byte>(PixRGB<double>(sptr[i+2], sptr[i+1], sptr[i]));
  else
    for (int i = 0; i < sz; i += 4)
      *dptr++ = PixVideoYUV<byte>(PixRGB<double>(sptr[i+1], sptr[i+2], sptr[i+3]));

  ASSERT(dptr == dst.endw());

  return VideoFrame(dst);
}

// ######################################################################
VideoFrame YUV444_to_YUV24(const byte* sptr, const size_t length,
                           const Dims& dims, const bool byteswap)
{
  const int sz = dims.sz() * 3;

  checkBufferLength(length, sz);

  Image<PixVideoYUV<byte> > dst(dims, NO_INIT);
  Image<PixVideoYUV<byte> >::iterator dptr = dst.beginw();

  if (byteswap)
    for (int i = 0; i < sz; i += 6)
      {
        // data stored as: y0, u0, u1, v0, v1, y1
        *dptr++ = PixVideoYUV<byte>(PixRGB<double>(sptr[i], sptr[i+1], sptr[i+3]));
        *dptr++ = PixVideoYUV<byte>(PixRGB<double>(sptr[i+5], sptr[i+2], sptr[i+4]));
      }
  else
    for (int i = 0; i < sz; i += 6)
      {
        // data stored as: u0, y0, v0, u1, y1, v1
        *dptr++ = PixVideoYUV<byte>(PixRGB<double>(sptr[i+1], sptr[i+0], sptr[i+2]));
        *dptr++ = PixVideoYUV<byte>(PixRGB<double>(sptr[i+4], sptr[i+3], sptr[i+5]));
      }

  ASSERT(dptr == dst.endw());

  return VideoFrame(dst);
}

// ######################################################################
VideoFrame YUYV_to_YUV24(const byte* sptr, const size_t length,
                         const Dims& dims, const bool byteswap)
{
  const int sz = dims.sz() * 2;

  checkBufferLength(length, sz);

  Image<PixVideoYUV<byte> > dst(dims, NO_INIT);
  Image<PixVideoYUV<byte> >::iterator dptr = dst.beginw();
  Image<PixVideoYUV<byte> >::iterator stop = dst.endw();

  if (byteswap)
    while (dptr != stop)
      {
        // data stored as: u, y0, v, y1
        *dptr++ = PixVideoYUV<byte>(sptr[1], sptr[0], sptr[2]);
        *dptr++ = PixVideoYUV<byte>(sptr[3], sptr[0], sptr[2]);
        sptr += 4;
      }
  else
    while (dptr != stop)
      {
        // data stored as:  y0, u, y1, v
        *dptr++ = PixVideoYUV<byte>(sptr[0], sptr[1], sptr[3]);
        *dptr++ = PixVideoYUV<byte>(sptr[2], sptr[1], sptr[3]);
        sptr += 4;
      }

  return VideoFrame(dst);
}

// ######################################################################
VideoFrame YUV422_to_YUV24(const byte* sptr, const size_t length,
                           const Dims& dims, const bool byteswap)
{
  return YUYV_to_YUV24(sptr, length, dims, !byteswap);
}

// ######################################################################
VideoFrame YUV411_to_YUV24(const byte* sptr, const size_t length,
                           const Dims& dims, const bool byteswap)
{
  checkBufferLength(length, dims.sz() * 3 / 2);

  Image<PixVideoYUV<byte> > dst(dims, NO_INIT);
  Image<PixVideoYUV<byte> >::iterator dptr = dst.beginw();
  Image<PixVideoYUV<byte> >::iterator stop = dst.endw();

  if (byteswap)
    while (dptr != stop)
      {
        // data stored as: y0, u, v, y1, y3, y2
        *dptr++ = PixVideoYUV<byte>(sptr[0], sptr[1], sptr[2]);
        *dptr++ = PixVideoYUV<byte>(sptr[3], sptr[1], sptr[2]);
        *dptr++ = PixVideoYUV<byte>(sptr[5], sptr[1], sptr[2]);
        *dptr++ = PixVideoYUV<byte>(sptr[4], sptr[1], sptr[2]);
        sptr += 6;
      }
  else
    while (dptr != stop)
      {
        // data stored as: u, y0, y1, v, y2, y3
        *dptr++ = PixVideoYUV<byte>(sptr[1], sptr[0], sptr[3]);
        *dptr++ = PixVideoYUV<byte>(sptr[2], sptr[0], sptr[3]);
        *dptr++ = PixVideoYUV<byte>(sptr[4], sptr[0], sptr[3]);
        *dptr++ = PixVideoYUV<byte>(sptr[5], sptr[0], sptr[3]);
        sptr += 6;
      }

  return VideoFrame(dst);
}

// ######################################################################
VideoFrame YUV444P_to_YUV24(const byte* sptr, const size_t length,
                            const Dims& dims, const bool byteswap)
{
  checkBufferLength(length, dims.sz() * 3);

  Image<PixVideoYUV<byte> > dst(dims, NO_INIT);
  Image<PixVideoYUV<byte> >::iterator dptr = dst.beginw();
  Image<PixVideoYUV<byte> >::iterator stop = dst.endw();

  const byte* yptr = sptr;
  const byte* uptr = yptr + dims.sz();
  const byte* vptr = uptr + dims.sz();

  if (byteswap)
    std::swap(uptr, vptr);

  while (dptr != stop)
    *dptr++ = PixVideoYUV<byte>(*yptr++, *uptr++, *vptr++);

  return VideoFrame(dst);
}

// ######################################################################
VideoFrame YUV422P_to_YUV24(const byte* sptr, const size_t length,
                            const Dims& dims, const bool byteswap)
{
  const int w = dims.w();
  const int h = dims.h();

  checkBufferLength(length, dims.sz() + 2*(w/2)*h);

  Image<PixVideoYUV<byte> > dst(dims, NO_INIT);
  Image<PixVideoYUV<byte> >::iterator dptr = dst.beginw();

  const byte* yptr = sptr;
  const byte* uptr = yptr + w * h;
  const byte* vptr = uptr + (w/2) * h;

  if (byteswap)
    std::swap(uptr, vptr);

  for (int j = 0; j < h; ++j)
    for (int i = 0; i < w; i += 2)
      {
        // we have 2 luminance pixels per chroma pair

        const byte yf1 = *yptr++;
        const byte yf2 = *yptr++;
        const byte uf = *uptr++;
        const byte vf = *vptr++;

        *dptr++ = PixVideoYUV<byte>(yf1, uf, vf);
        *dptr++ = PixVideoYUV<byte>(yf2, uf, vf);
      }
  ASSERT(dptr == dst.endw());

  return VideoFrame(dst);
}

// ######################################################################
VideoFrame YUV411P_to_YUV24(const byte* sptr, const size_t length,
                            const Dims& dims, const bool byteswap)
{
  const int w = dims.w();
  const int h = dims.h();

  checkBufferLength(length, dims.sz() + 2*(w/4)*h);

  Image<PixVideoYUV<byte> > dst(dims, NO_INIT);
  Image<PixVideoYUV<byte> >::iterator dptr = dst.beginw();

  const byte* yptr = sptr;
  const byte* uptr = yptr + w * h;
  const byte* vptr = uptr + (w/4) * h;

  if (byteswap)
    std::swap(uptr, vptr);

  for (int j = 0; j < h; ++j)
    for (int i = 0; i < w; i += 4)
      {
        // we have a 4 luminance pixels per chroma pair
        const byte uf = *uptr++;
        const byte vf = *vptr++;

        *dptr++ = PixVideoYUV<byte>(*yptr++, uf, vf);
        *dptr++ = PixVideoYUV<byte>(*yptr++, uf, vf);
        *dptr++ = PixVideoYUV<byte>(*yptr++, uf, vf);
        *dptr++ = PixVideoYUV<byte>(*yptr++, uf, vf);
      }
  ASSERT(dptr == dst.endw());

  return VideoFrame(dst);
}

// ######################################################################
VideoFrame YUV420P_to_YUV24(const byte* sptr, const size_t length,
                            const Dims& dims, const bool byteswap)
{

  const int w = dims.w();
  const int h = dims.h();
  // we have to do (w+1)/2 instead of just w/2, because if e.g. the y
  // array has 5 pixels, then we want the u and v arrays to have 3
  // pixels, not 2:
  const int w2 = (w+1)/2;
  const int h2 = (h+1)/2;

  checkBufferLength(length, dims.sz() + 2*w2*h2);

  Image<PixVideoYUV<byte> > dst(dims, NO_INIT);
  Image<PixVideoYUV<byte> >::iterator dptr = dst.beginw();

  const byte* yptr = sptr;
  const byte* uptr = yptr + w * h;
  const byte* vptr = uptr + w2 * h2;

  if (byteswap)
    std::swap(uptr, vptr);

  for (int j = 0; j < h; j += 2)
    {
      for (int i = 0; i < w; i += 2)
        {
          const byte u = *uptr++;
          const byte v = *vptr++;
          // we have a 2x2 luminance block per chroma pair
          dptr[0] = PixVideoYUV<byte>(yptr[0], u, v);
          dptr[1] = PixVideoYUV<byte>(yptr[1], u, v);
          dptr[w] = PixVideoYUV<byte>(yptr[w], u, v);
          dptr[w+1] = PixVideoYUV<byte>(yptr[w+1], u, v);

          dptr += 2;
          yptr += 2;
        }
      dptr += w;
      yptr += w;
    }
  ASSERT(dptr == dst.endw());

  return VideoFrame(dst);
}

// ######################################################################
VideoFrame YUV410P_to_YUV24(const byte* sptr, const size_t length,
                            const Dims& dims, const bool byteswap)
{
  const int w = dims.w();
  const int h = dims.h();
  const int w2 = w * 2;
  const int w3 = w * 3;

  checkBufferLength(length, dims.sz() * 9 / 8);

  Image<PixVideoYUV<byte> > dst(dims, NO_INIT);
  Image<PixVideoYUV<byte> >::iterator dptr = dst.beginw();

  const byte* yptr = sptr;
  const byte* uptr = yptr + w * h;
  const byte* vptr = uptr + (w/4) * (h/4);

  for (int j = 0; j < h; j += 4)
    {
      for (int i = 0; i < w; i += 4)
        {
          const byte u = *uptr++;
          const byte v = *vptr++;

          // we have a 4x4 luminance block per chroma pair
          dptr[0] = PixVideoYUV<byte>(*yptr, u, v);
          dptr[1] = PixVideoYUV<byte>(yptr[1], u, v);
          dptr[2] = PixVideoYUV<byte>(yptr[2], u, v);
          dptr[3] = PixVideoYUV<byte>(yptr[3], u, v);

          dptr[w] = PixVideoYUV<byte>(yptr[w], u, v);
          dptr[w+1] = PixVideoYUV<byte>(yptr[w+1], u, v);
          dptr[w+2] = PixVideoYUV<byte>(yptr[w+2], u, v);
          dptr[w+3] = PixVideoYUV<byte>(yptr[w+3], u, v);

          dptr[w2] = PixVideoYUV<byte>(yptr[w2], u, v);
          dptr[w2+1] = PixVideoYUV<byte>(yptr[w2+1], u, v);
          dptr[w2+2] = PixVideoYUV<byte>(yptr[w2+2], u, v);
          dptr[w2+3] = PixVideoYUV<byte>(yptr[w2+3], u, v);

          dptr[w3] = PixVideoYUV<byte>(yptr[w3], u, v);
          dptr[w3+1] = PixVideoYUV<byte>(yptr[w3+1], u, v);
          dptr[w3+2] = PixVideoYUV<byte>(yptr[w3+2], u, v);
          dptr[w3+3] = PixVideoYUV<byte>(yptr[w3+3], u, v);

          dptr += 4;
          yptr += 4;
        }
      dptr += w3;
      yptr += w3;
    }
  ASSERT(dptr == dst.endw());

  return VideoFrame(dst);
}

// ######################################################################
// ######################################################################
// conversions from YUV24
// ######################################################################
// ######################################################################

// ######################################################################
VideoFrame YUV24_to_YUV444(const byte* sptr, const size_t length,
                           const Dims& dims, const bool byteswap)
{
  checkBufferLength(length, dims.sz() * 3);

  ArrayHandle<byte> dst(new ArrayData<byte>(Dims(dims.sz() * 3, 1), NO_INIT));
  byte* dptr = dst.uniq().dataw();
  byte* stop = dst.uniq().endw();

  // output data stored as: u0, y0, v0, u1, y1, v1

  if (byteswap)
    while (dptr != stop)
      {
        // input data stored as: y0, v0, u0, y1, v1, u1
        dptr[0] = sptr[2];
        dptr[1] = sptr[0];
        dptr[2] = sptr[1];
        dptr[3] = sptr[5];
        dptr[4] = sptr[3];
        dptr[5] = sptr[4];

        dptr += 6;
        sptr += 6;
      }
  else
    while (dptr != stop)
      {
        // input data stored as: y0, u0, v0, y1, u1, v1
        dptr[0] = sptr[1];
        dptr[1] = sptr[0];
        dptr[2] = sptr[2];
        dptr[3] = sptr[4];
        dptr[4] = sptr[3];
        dptr[5] = sptr[5];

        dptr += 6;
        sptr += 6;
      }

  return VideoFrame(dst, dims, VIDFMT_YUV444, false);
}

// ######################################################################
VideoFrame YUV24_to_YUYV(const byte* sptr, const size_t length,
                         const Dims& dims, const bool byteswap)
{
  checkBufferLength(length, dims.sz() * 3);

  ASSERT(dims.w() % 2 == 0);

  ArrayHandle<byte> dst(new ArrayData<byte>(Dims(dims.sz() * 2, 1), NO_INIT));
  byte* dptr = dst.uniq().dataw();
  byte* stop = dst.uniq().endw();

  // output data stored as: y0, u, y1, v

  if (byteswap)
    while (dptr != stop)
      {
        // input data stored as: y0, v0, u0, y1, v1, u1
        dptr[0] = sptr[0];
        dptr[1] = byte((sptr[2] + sptr[5])/2.0 + 0.5);
        dptr[2] = sptr[3];
        dptr[3] = byte((sptr[1] + sptr[4])/2.0 + 0.5);

        dptr += 4;
        sptr += 6;
      }
  else
    while (dptr != stop)
      {
        // input data stored as: y0, u0, v0, y1, u1, v1
        dptr[0] = sptr[0];
        dptr[1] = byte((sptr[1] + sptr[4])/2.0 + 0.5);
        dptr[2] = sptr[3];
        dptr[3] = byte((sptr[2] + sptr[5])/2.0 + 0.5);

        dptr += 4;
        sptr += 6;
      }

  return VideoFrame(dst, dims, VIDFMT_YUYV, false);
}

// ######################################################################
ArrayHandle<byte> YUV24_to_UYVYx(const byte* sptr, const size_t length,
                                 const Dims& dims, const bool byteswap)
{
  checkBufferLength(length, dims.sz() * 3);

  ASSERT(dims.w() % 2 == 0);

  ArrayHandle<byte> dst(new ArrayData<byte>(Dims(dims.sz() * 2, 1), NO_INIT));
  byte* dptr = dst.uniq().dataw();
  byte* stop = dst.uniq().endw();

  // output data stored as: u, y0, v, y1

  if (byteswap)
    while (dptr != stop)
      {
        // input data stored as: y0, v0, u0, y1, v1, u1
        dptr[0] = byte((sptr[2] + sptr[5])/2.0 + 0.5);
        dptr[1] = sptr[0];
        dptr[2] = byte((sptr[1] + sptr[4])/2.0 + 0.5);
        dptr[3] = sptr[3];

        dptr += 4;
        sptr += 6;
      }
  else
    while (dptr != stop)
      {
        // input data stored as: y0, u0, v0, y1, u1, v1
        dptr[0] = byte((sptr[1] + sptr[4])/2.0 + 0.5);
        dptr[1] = sptr[0];
        dptr[2] = byte((sptr[2] + sptr[5])/2.0 + 0.5);
        dptr[3] = sptr[3];

        dptr += 4;
        sptr += 6;
      }

  return dst;
}

// ######################################################################
VideoFrame YUV24_to_UYVY(const byte* sptr, const size_t length,
                         const Dims& dims, const bool byteswap)
{
  return VideoFrame(YUV24_to_UYVYx(sptr, length, dims, byteswap),
                    dims, VIDFMT_UYVY, false);
}

// ######################################################################
VideoFrame YUV24_to_YUV422(const byte* sptr, const size_t length,
                           const Dims& dims, const bool byteswap)
{
  return VideoFrame(YUV24_to_UYVYx(sptr, length, dims, byteswap),
                    dims, VIDFMT_YUV422, false);
}

// ######################################################################
VideoFrame YUV24_to_YUV411(const byte* sptr, const size_t length,
                           const Dims& dims, const bool byteswap)
{
  checkBufferLength(length, dims.sz() * 3);

  ASSERT(dims.w() % 4 == 0);

  ArrayHandle<byte> dst(new ArrayData<byte>(Dims(dims.sz() * 3 / 2, 1), NO_INIT));
  byte* dptr = dst.uniq().dataw();
  byte* stop = dst.uniq().endw();

  // output data stored as: u, y0, y1, v, y2, y3

  if (byteswap)
    while (dptr != stop)
      {
        // input data stored as: y0, v0, u0, y1, v1, u1, y2, v2, y2, y3, v3, u3
        dptr[0] = byte((sptr[2] + sptr[5] + sptr[8] + sptr[11])/4.0 + 0.5);
        dptr[1] = sptr[0];
        dptr[2] = sptr[3];
        dptr[3] = byte((sptr[1] + sptr[4] + sptr[7] + sptr[10])/4.0 + 0.5);
        dptr[4] = sptr[6];
        dptr[5] = sptr[9];

        dptr += 6;
        sptr += 12;
      }
  else
    while (dptr != stop)
      {
        // input data stored as: y0, u0, v0, y1, u1, v1, y2, u2, v2, y3, u3, v3
        dptr[0] = byte((sptr[1] + sptr[4] + sptr[7] + sptr[10])/4.0 + 0.5);
        dptr[1] = sptr[0];
        dptr[2] = sptr[3];
        dptr[3] = byte((sptr[2] + sptr[5] + sptr[8] + sptr[11])/4.0 + 0.5);
        dptr[4] = sptr[6];
        dptr[5] = sptr[9];

        dptr += 6;
        sptr += 12;
      }

  return VideoFrame(dst, dims, VIDFMT_YUV411, false);
}

// ######################################################################
VideoFrame YUV24_to_YUV444P(const byte* sptr, const size_t length,
                            const Dims& dims, const bool byteswap)
{
  checkBufferLength(length, dims.sz() * 3);

  ArrayHandle<byte> dst(new ArrayData<byte>(Dims(dims.sz() * 3, 1), NO_INIT));
  byte* yptr = dst.uniq().dataw();
  byte* uptr = yptr + dims.sz();
  byte* vptr = uptr + dims.sz();

  if (byteswap)
    std::swap(uptr, vptr);

  const byte* sstop = sptr + length;

  while (sptr != sstop)
    {
      *yptr++ = *sptr++;
      *uptr++ = *sptr++;
      *vptr++ = *sptr++;
    }

  return VideoFrame(dst, dims, VIDFMT_YUV444P, false);
}

// ######################################################################
VideoFrame YUV24_to_YUV422P(const byte* sptr, const size_t length,
                            const Dims& dims, const bool byteswap)
{
  checkBufferLength(length, dims.sz() * 3);

  ASSERT(dims.w() % 2 == 0);

  const int w = dims.w();
  const int h = dims.h();

  ArrayHandle<byte> dst(new ArrayData<byte>(Dims(dims.sz() + 2 * (w/2)*h, 1), NO_INIT));
  byte* yptr = dst.uniq().dataw();
  byte* uptr = yptr + w * h;
  byte* vptr = uptr + (w/2) * h;

  if (byteswap)
    std::swap(uptr, vptr);

  const byte* sstop = sptr + length;

  while (sptr != sstop)
    {
      *yptr++ = sptr[0];
      *yptr++ = sptr[3];
      *uptr++ = byte((sptr[1]+sptr[4])/2.0 + 0.5);
      *vptr++ = byte((sptr[2]+sptr[5])/2.0 + 0.5);
      sptr += 6;
    }

  return VideoFrame(dst, dims, VIDFMT_YUV422P, false);
}

// ######################################################################
VideoFrame YUV24_to_YUV411P(const byte* sptr, const size_t length,
                            const Dims& dims, const bool byteswap)
{
  checkBufferLength(length, dims.sz() * 3);

  ASSERT(dims.w() % 4 == 0);

  const int w = dims.w();
  const int h = dims.h();

  ArrayHandle<byte> dst(new ArrayData<byte>(Dims(dims.sz() + 2 * (w/4)*h, 1), NO_INIT));
  byte* yptr = dst.uniq().dataw();
  byte* uptr = yptr + w * h;
  byte* vptr = uptr + (w/4) * h;

  if (byteswap)
    std::swap(uptr, vptr);

  const byte* sstop = sptr + length;

  while (sptr != sstop)
    {
      *yptr++ = sptr[0];
      *yptr++ = sptr[3];
      *yptr++ = sptr[6];
      *yptr++ = sptr[9];
      *uptr++ = byte((sptr[1]+sptr[4]+sptr[7]+sptr[10])/4.0 + 0.5);
      *vptr++ = byte((sptr[2]+sptr[5]+sptr[8]+sptr[11])/4.0 + 0.5);
      sptr += 12;
    }

  return VideoFrame(dst, dims, VIDFMT_YUV411P, false);
}

// ######################################################################
VideoFrame YUV24_to_YUV420P(const byte* sptr, const size_t length,
                            const Dims& dims, const bool byteswap)
{
  checkBufferLength(length, dims.sz() * 3);

  ASSERT(dims.w() % 2 == 0);
  ASSERT(dims.h() % 2 == 0);

  const int w = dims.w();
  const int h = dims.h();

  const int w2 = (w+1)/2;
  const int h2 = (h+1)/2;

  ArrayHandle<byte> dst(new ArrayData<byte>(Dims(dims.sz() + 2*w2*h2, 1), NO_INIT));
  byte* yptr = dst.uniq().dataw();
  byte* uptr = yptr + w * h;
  byte* vptr = uptr + w2 * h2;

  if (byteswap)
    std::swap(uptr, vptr);

  const byte* sstop = sptr + length;

  for (int j = 0; j < h; j += 2)
    {
      for (int i = 0; i < w; i += 2)
        {
          yptr[0] = sptr[0];
          yptr[1] = sptr[3];
          yptr[w] = sptr[3*w];
          yptr[w+1] = sptr[3*w+3];
          *uptr++ = byte((sptr[1] + sptr[4] + sptr[3*w+1] + sptr[3*w+4])/4.0 + 0.5);
          *vptr++ = byte((sptr[2] + sptr[5] + sptr[3*w+2] + sptr[3*w+5])/4.0 + 0.5);

          yptr += 2;
          sptr += 6;
        }

      yptr += w;
      sptr += 3*w;
    }

  ASSERT(sptr == sstop);

  return VideoFrame(dst, dims, VIDFMT_YUV420P, false);
}

// ######################################################################
VideoFrame YUV24_to_YUV410P(const byte* sptr, const size_t length,
                            const Dims& dims, const bool byteswap)
{
  checkBufferLength(length, dims.sz() * 3);

  ASSERT(dims.w() % 4 == 0);
  ASSERT(dims.h() % 4 == 0);

  const int w = dims.w();
  const int h = dims.h();

  ArrayHandle<byte> dst(new ArrayData<byte>(Dims(dims.sz() * 9 / 8, 1), NO_INIT));
  byte* yptr = dst.uniq().dataw();
  byte* uptr = yptr + w * h;
  byte* vptr = uptr + (w/4) * (h/4);

  if (byteswap)
    std::swap(uptr, vptr);

  const byte* sstop = sptr + length;

  for (int j = 0; j < h; j += 4)
    {
      for (int i = 0; i < w; i += 4)
        {
          yptr[0] = sptr[0];
          yptr[1] = sptr[3];
          yptr[2] = sptr[6];
          yptr[3] = sptr[9];
          yptr[w] = sptr[3*w];
          yptr[w+1] = sptr[3*w+3];
          yptr[w+2] = sptr[3*w+6];
          yptr[w+3] = sptr[3*w+9];
          yptr[2*w] = sptr[6*w];
          yptr[2*w+1] = sptr[6*w+3];
          yptr[2*w+2] = sptr[6*w+6];
          yptr[2*w+3] = sptr[6*w+9];
          yptr[3*w] = sptr[9*w];
          yptr[3*w+1] = sptr[9*w+3];
          yptr[3*w+2] = sptr[9*w+6];
          yptr[3*w+3] = sptr[9*w+9];

          *uptr++ = byte((sptr[1] + sptr[4] + sptr[7] + sptr[10] +
                          sptr[3*w+1] + sptr[3*w+4] + sptr[3*w+7] + sptr[3*w+10] +
                          sptr[6*w+1] + sptr[6*w+4] + sptr[6*w+7] + sptr[6*w+10] +
                          sptr[9*w+1] + sptr[9*w+4] + sptr[9*w+7] + sptr[9*w+10])
                         / 16.0 + 0.5);
          *vptr++ = byte((sptr[2] + sptr[5] + sptr[8] + sptr[11] +
                          sptr[3*w+2] + sptr[3*w+5] + sptr[3*w+8] + sptr[3*w+11] +
                          sptr[6*w+2] + sptr[6*w+5] + sptr[6*w+8] + sptr[6*w+11] +
                          sptr[9*w+2] + sptr[9*w+5] + sptr[9*w+8] + sptr[9*w+11])
                         / 16.0 + 0.5);

          yptr += 4;
          sptr += 12;
        }

      yptr += 3*w;
      sptr += 9*w;
    }

  ASSERT(sptr == sstop);

  return VideoFrame(dst, dims, VIDFMT_YUV410P, false);
}

// ######################################################################
// ######################################################################
// VideoFormatConverter
// ######################################################################
// ######################################################################

// ######################################################################
VideoFormatConverter::VideoFormatConverter(VideoFormatConverter::Func* f,
                                           unsigned int w,
                                           VideoFormat s, VideoFormat d,
                                           const char* fn)
  : func(f), weight(w), src(s), dst(d), fname(fn)
{}

// ######################################################################
VideoFrame VideoFormatConverter::apply(const VideoFrame& in) const
{
  if (func == NULL)
    LFATAL("oops! this is an invalid VideoFormatConverter");

  if (in.getMode() != this->src)
    LFATAL("oops! src frame is %s, but converter (%s) expects %s",
           convertToString(in.getMode()).c_str(),
           this->fname.c_str(),
           convertToString(this->src).c_str());

  const VideoFrame result =
    (*this->func)(in.getBuffer(), in.getBufSize(),
                  in.getDims(), in.getByteSwap());

  ASSERT(result.getMode() == this->dst);

  return result;
}


// ######################################################################
// ######################################################################
// VideoFormatCoercion
// ######################################################################
// ######################################################################

// ######################################################################
VideoFormatCoercion::VideoFormatCoercion()
{}

// ######################################################################
VideoFormatCoercion::VideoFormatCoercion(unsigned int w,
                                         VideoFormat s, VideoFormat d,
                                         VideoFormatConverter::Func* f,
                                         const char* fname)
{
  if (f != NULL)
    nodes.push_back(VideoFormatConverter(f, w, s, d, fname));
}

// ######################################################################
std::string VideoFormatCoercion::describe() const
{
  if (nodes.empty())
    return std::string("nil");

  unsigned int w = 0;
  for (size_t i = 0; i < nodes.size(); ++i)
    w += nodes[i].weight;

  std::string result = sformat("%s -> %s [wt=%u] (",
                               convertToString(nodes.front().src).c_str(),
                               convertToString(nodes.back().dst).c_str(),
                               w);

  for (size_t i = 0; i < nodes.size(); ++i)
    {
      result += sformat("%s [wt=%u]",
                        nodes[i].fname.c_str(), nodes[i].weight);

      if (i+1 < nodes.size())
        result += "; ";
    }

  result += ")";
  return result;
}

// ######################################################################
VideoFrame VideoFormatCoercion::apply(const VideoFrame& src) const
{
  if (nodes.size() == 0)
    LFATAL("oops! this is an invalid VideoFormat converter");

  VideoFrame result = src;
  for (size_t i = 0; i < nodes.size(); ++i)
    result = nodes[i].apply(result);

  return result;
}


// ######################################################################
// ######################################################################
// conversion table
// ######################################################################
// ######################################################################

static VideoFormatCoercion pathtab[VIDFMT_AUTO+1][VIDFMT_AUTO+1];

static const unsigned int PRECISION_PENALTY = 100; // loss of pixel bit depth (e.g. rgb24->rgb565)
static const unsigned int RESOLUTION_PENALTY = 10000; // loss of spatial resolution (e.g. yuv24 -> yuv411)
static const unsigned int COLORSPACE_PENALTY = 1000000; // lossy colorspace conversion (e.g. rgb->yuv)
static const unsigned int COLORDIMENSION_PENALTY = 100000000; // reduction in the number of color planes (e.g., rgb->grey)

static void initDirectConversions(VideoFormatCoercion table[VIDFMT_AUTO+1][VIDFMT_AUTO+1])
{
#define CONVERT(vf1, vf2, wt, f)                                \
  table[vf1][vf2] = VideoFormatCoercion(wt, vf1, vf2, f, #f)

  // conversions to RGB24
  CONVERT(VIDFMT_GREY,      VIDFMT_RGB24, 1, &GREY_to_RGB24);
  CONVERT(VIDFMT_RGB555,    VIDFMT_RGB24, 1, &RGB555_to_RGB24);
  CONVERT(VIDFMT_RGB565,    VIDFMT_RGB24, 1, &RGB565_to_RGB24);
  CONVERT(VIDFMT_RGB32,     VIDFMT_RGB24, 1, &ARGB_to_RGB24);
  CONVERT(VIDFMT_YUYV,      VIDFMT_RGB24, 1+COLORSPACE_PENALTY, &YUYV_to_RGB24);
  CONVERT(VIDFMT_UYVY,      VIDFMT_RGB24, 1+COLORSPACE_PENALTY, &YUV422_to_RGB24);
  CONVERT(VIDFMT_YUV444,    VIDFMT_RGB24, 1+COLORSPACE_PENALTY, &YUV444_to_RGB24);
  CONVERT(VIDFMT_YUV422,    VIDFMT_RGB24, 1+COLORSPACE_PENALTY, &YUV422_to_RGB24);
  CONVERT(VIDFMT_YUV411,    VIDFMT_RGB24, 1+COLORSPACE_PENALTY, &YUV411_to_RGB24);
  //CONVERT(VIDFMT_YUV420,    VIDFMT_RGB24, 1+COLORSPACE_PENALTY, NULL); /* not implemented: what is YUV420? */
  //CONVERT(VIDFMT_YUV410,    VIDFMT_RGB24, 1+COLORSPACE_PENALTY, NULL); /* not implemented: what is YUV410? */
  CONVERT(VIDFMT_YUV444P,   VIDFMT_RGB24, 1+COLORSPACE_PENALTY, &YUV444P_to_RGB24);
  CONVERT(VIDFMT_YUV422P,   VIDFMT_RGB24, 1+COLORSPACE_PENALTY, &YUV422P_to_RGB24);
  CONVERT(VIDFMT_YUV411P,   VIDFMT_RGB24, 1+COLORSPACE_PENALTY, &YUV411P_to_RGB24);
  CONVERT(VIDFMT_YUV420P,   VIDFMT_RGB24, 1+COLORSPACE_PENALTY, &YUV420P_to_RGB24);
  CONVERT(VIDFMT_YUV410P,   VIDFMT_RGB24, 1+COLORSPACE_PENALTY, &YUV410P_to_RGB24);

  // conversions from RGB24
  CONVERT(VIDFMT_RGB24, VIDFMT_GREY,      1+COLORDIMENSION_PENALTY, &RGB24_to_GREY);
  CONVERT(VIDFMT_RGB24, VIDFMT_RGB555,    1+PRECISION_PENALTY, &RGB24_to_RGB555);
  CONVERT(VIDFMT_RGB24, VIDFMT_RGB565,    1+PRECISION_PENALTY, &RGB24_to_RGB565);
  CONVERT(VIDFMT_RGB24, VIDFMT_RGB32,     1, &RGB24_to_RGB32);
  CONVERT(VIDFMT_RGB24, VIDFMT_YUV24,     1+COLORSPACE_PENALTY, &RGB24_to_YUV24);
  //CONVERT(VIDFMT_RGB24, VIDFMT_YUYV,      1+COLORSPACE_PENALTY+RESOLUTION_PENALTY, NULL); /* could be implemented in the future */
  //CONVERT(VIDFMT_RGB24, VIDFMT_UYVY,      1+COLORSPACE_PENALTY+RESOLUTION_PENALTY, NULL); /* could be implemented in the future */
  //CONVERT(VIDFMT_RGB24, VIDFMT_YUV444,    1+COLORSPACE_PENALTY, NULL); /* could be implemented in the future */
  //CONVERT(VIDFMT_RGB24, VIDFMT_YUV422,    1+COLORSPACE_PENALTY+RESOLUTION_PENALTY, NULL); /* could be implemented in the future */
  //CONVERT(VIDFMT_RGB24, VIDFMT_YUV411,    1+COLORSPACE_PENALTY+RESOLUTION_PENALTY, NULL); /* could be implemented in the future */
  //CONVERT(VIDFMT_RGB24, VIDFMT_YUV420,    1+COLORSPACE_PENALTY+RESOLUTION_PENALTY, NULL); /* not implemented: what is YUV420? */
  //CONVERT(VIDFMT_RGB24, VIDFMT_YUV410,    1+COLORSPACE_PENALTY+RESOLUTION_PENALTY, NULL); /* not implemented: what is YUV410? */
  //CONVERT(VIDFMT_RGB24, VIDFMT_YUV444P,   1+COLORSPACE_PENALTY, NULL); /* could be implemented in the future */
  //CONVERT(VIDFMT_RGB24, VIDFMT_YUV422P,   1+COLORSPACE_PENALTY+RESOLUTION_PENALTY, NULL); /* could be implemented in the future */
  //CONVERT(VIDFMT_RGB24, VIDFMT_YUV411P,   1+COLORSPACE_PENALTY+RESOLUTION_PENALTY, NULL); /* could be implemented in the future */
  //CONVERT(VIDFMT_RGB24, VIDFMT_YUV420P,   1+COLORSPACE_PENALTY+RESOLUTION_PENALTY, NULL); /* could be implemented in the future */
  //CONVERT(VIDFMT_RGB24, VIDFMT_YUV410P,   1+COLORSPACE_PENALTY+RESOLUTION_PENALTY, NULL); /* could be implemented in the future */

  // conversions to YUV24
  CONVERT(VIDFMT_GREY,      VIDFMT_YUV24, 1+COLORSPACE_PENALTY, &GREY_to_YUV24);
  //CONVERT(VIDFMT_RGB555,    VIDFMT_YUV24, 1+COLORSPACE_PENALTY, NULL); /* could be implemented in the future */
  //CONVERT(VIDFMT_RGB565,    VIDFMT_YUV24, 1+COLORSPACE_PENALTY, NULL); /* could be implemented in the future */
  CONVERT(VIDFMT_RGB24,     VIDFMT_YUV24, 1+COLORSPACE_PENALTY, &RGB24_to_YUV24);
  CONVERT(VIDFMT_RGB32,     VIDFMT_YUV24, 1+COLORSPACE_PENALTY, &RGB32_to_YUV24);
  CONVERT(VIDFMT_YUYV,      VIDFMT_YUV24, 1, &YUYV_to_YUV24);
  CONVERT(VIDFMT_UYVY,      VIDFMT_YUV24, 1, &YUV422_to_YUV24);
  CONVERT(VIDFMT_YUV444,    VIDFMT_YUV24, 1, &YUV444_to_YUV24);
  CONVERT(VIDFMT_YUV422,    VIDFMT_YUV24, 1, &YUV422_to_YUV24);
  CONVERT(VIDFMT_YUV411,    VIDFMT_YUV24, 1, &YUV411_to_YUV24);
  //CONVERT(VIDFMT_YUV420,    VIDFMT_YUV24, 1, NULL); /* not implemented: what is YUV420? */
  //CONVERT(VIDFMT_YUV410,    VIDFMT_YUV24, 1, NULL); /* not implemented: what is YUV410? */
  CONVERT(VIDFMT_YUV444P,   VIDFMT_YUV24, 1, &YUV444P_to_YUV24);
  CONVERT(VIDFMT_YUV422P,   VIDFMT_YUV24, 1, &YUV422P_to_YUV24);
  CONVERT(VIDFMT_YUV411P,   VIDFMT_YUV24, 1, &YUV411P_to_YUV24);
  CONVERT(VIDFMT_YUV420P,   VIDFMT_YUV24, 1, &YUV420P_to_YUV24);
  CONVERT(VIDFMT_YUV410P,   VIDFMT_YUV24, 1, &YUV410P_to_YUV24);

  // conversions from YUV24
  //CONVERT(VIDFMT_YUV24, VIDFMT_GREY,      1+COLORSPACE_PENALTY, NULL); /* could be implemented in the future */
  //CONVERT(VIDFMT_YUV24, VIDFMT_RGB555,    1+COLORSPACE_PENALTY+PRECISION_PENALTY, NULL); /* could be implemented in the future */
  //CONVERT(VIDFMT_YUV24, VIDFMT_RGB565,    1+COLORSPACE_PENALTY+PRECISION_PENALTY, NULL); /* could be implemented in the future */
  CONVERT(VIDFMT_YUV24, VIDFMT_RGB24,     1+COLORSPACE_PENALTY, &YUV24_to_RGB24);
  //CONVERT(VIDFMT_YUV24, VIDFMT_RGB32,     1+COLORSPACE_PENALTY, NULL); /* could be implemented in the future */
  CONVERT(VIDFMT_YUV24, VIDFMT_YUYV,      1+RESOLUTION_PENALTY, &YUV24_to_YUYV);
  CONVERT(VIDFMT_YUV24, VIDFMT_UYVY,      1+RESOLUTION_PENALTY, &YUV24_to_UYVY);
  CONVERT(VIDFMT_YUV24, VIDFMT_YUV444,    1, &YUV24_to_YUV444);
  CONVERT(VIDFMT_YUV24, VIDFMT_YUV422,    1+RESOLUTION_PENALTY, &YUV24_to_YUV422);
  CONVERT(VIDFMT_YUV24, VIDFMT_YUV411,    1+RESOLUTION_PENALTY, &YUV24_to_YUV411);
  //CONVERT(VIDFMT_YUV24, VIDFMT_YUV420,    1+RESOLUTION_PENALTY, NULL); /* could be implemented in the future */
  //CONVERT(VIDFMT_YUV24, VIDFMT_YUV410,    1+RESOLUTION_PENALTY, NULL); /* could be implemented in the future */
  CONVERT(VIDFMT_YUV24, VIDFMT_YUV444P,   1, &YUV24_to_YUV444P);
  CONVERT(VIDFMT_YUV24, VIDFMT_YUV422P,   1+RESOLUTION_PENALTY, &YUV24_to_YUV422P);
  CONVERT(VIDFMT_YUV24, VIDFMT_YUV411P,   1+RESOLUTION_PENALTY, &YUV24_to_YUV411P);
  CONVERT(VIDFMT_YUV24, VIDFMT_YUV420P,   1+RESOLUTION_PENALTY, &YUV24_to_YUV420P);
  CONVERT(VIDFMT_YUV24, VIDFMT_YUV410P,   1+RESOLUTION_PENALTY, &YUV24_to_YUV410P);

#undef CONVERT
}

namespace
{
  struct Heap
  {
    int deletemin()
    {
      ASSERT(!h.empty());

      unsigned int best = h[0].second;
      size_t bestpos = 0;

      for (size_t i = 1; i < h.size(); ++i)
        if (h[i].second < best)
          {
            best = h[i].second;
            bestpos = i;
          }

      int result = h[bestpos].first;
      h.erase(h.begin() + bestpos);
      return result;
    }

    void decreasekey(int v, unsigned int dist)
    {
      for (size_t i = 0; i < h.size(); ++i)
        if (h[i].first == v)
          {
            h[i].second = dist;
            return;
          }
    }

    std::vector<std::pair<int, unsigned int> > h;
  };
}

void initIndirectConversions(VideoFormatCoercion table[VIDFMT_AUTO+1][VIDFMT_AUTO+1])
{
  // Dijkstra's shortest-path algorithm to find the optimal sequence
  // of atomic conversions to achieve an arbitrary coercion

  for (int src = 0; src <= VIDFMT_AUTO; ++src)
    {
      unsigned int dist[VIDFMT_AUTO+1];
      int prev[VIDFMT_AUTO+1];

      for (int u = 0; u <= VIDFMT_AUTO; ++u)
        {
          dist[u] = std::numeric_limits<unsigned int>::max();
          prev[u] = -1;
        }
      dist[src] = 0;

      Heap H;
      for (int u = 0; u <= VIDFMT_AUTO; ++u)
        if (u == src)
          H.h.push_back(std::make_pair(u, 0));
        else if (table[src][u].isDirect())
          H.h.push_back(std::make_pair(u, table[src][u].nodes[0].weight));
        else
          H.h.push_back(std::make_pair(u, std::numeric_limits<unsigned int>::max()));

      while (!H.h.empty())
        {
          const int u = H.deletemin();
          for (int v = 0; v <= VIDFMT_AUTO; ++v)
            {
              if (table[u][v].nodes.size() != 1)
                continue;

              if (double(dist[v]) > double(dist[u]) + double(table[u][v].nodes[0].weight))
                {
                  dist[v] = dist[u] + table[u][v].nodes[0].weight;
                  prev[v] = u;

                  H.decreasekey(v, dist[v]);
                }
            }
        }

      for (int u = 0; u <= VIDFMT_AUTO; ++u)
        {
          if (u != src && prev[u] != -1)
            {
              VideoFormatCoercion p;
              int v = u;
              while (v != src)
                {
                  ASSERT(table[prev[v]][v].isDirect());
                  p.nodes.push_front(table[prev[v]][v].nodes[0]);
                  v = prev[v];
                }

              table[src][u] = p;
            }
        }
    }
}

// ######################################################################
const VideoFormatCoercion& findConverter(const VideoFormat srcformat,
                                         const VideoFormat dstformat)
{
  static bool inited = false;
  if (!inited)
    {
      initDirectConversions(pathtab);
      initIndirectConversions(pathtab);
      inited = true;
    }

  ASSERT(srcformat >= 0);
  ASSERT(srcformat <= VIDFMT_AUTO);
  ASSERT(dstformat >= 0);
  ASSERT(dstformat <= VIDFMT_AUTO);

  return pathtab[srcformat][dstformat];
}

// ######################################################################
VideoFrame coerceVideoFormat(const VideoFrame& src,
                             const VideoFormat dstformat)
{
  if (src.getMode() == dstformat)
    return src;

  const VideoFormatCoercion& c = findConverter(src.getMode(), dstformat);

  return c.apply(src);
}

// ######################################################################
void printCoercionTable()
{
  for (size_t n = 7; n-- > 0; )
    {
      std::string line = sformat("%7s ", "");

      for (int i = 0; i <= VIDFMT_AUTO; ++i)
        {
          const std::string m = convertToString(VideoFormat(i));
          if (n < m.size())
            line += sformat(" %c", m[m.size() - 1 - n]);
          else
            line += "  ";
        }

      LINFO("%s", line.c_str());
    }

  {
    std::string line = sformat("%7s ", "");
    for (int i = 0; i <= VIDFMT_AUTO; ++i)
      line += " -";

    LINFO("%s", line.c_str());
  }

  for (int i = 0; i <= VIDFMT_AUTO; ++i)
    {
      std::string line =
        sformat("%7s|", convertToString(VideoFormat(i)).c_str());
      for (int j = 0; j <= VIDFMT_AUTO; ++j)
        if (i == j)
          line += " 0";
        else
          {
            const VideoFormatCoercion& c =
              findConverter(VideoFormat(i), VideoFormat(j));
            if (c.nodes.empty())
              line += " .";
            else
              line += sformat(" %d", int(c.nodes.size()));
          }

      LINFO("%s", line.c_str());
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // VIDEO_VIDEOFORMATCOERCION_C_DEFINED

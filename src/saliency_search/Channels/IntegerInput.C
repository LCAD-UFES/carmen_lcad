/*!@file Channels/IntegerInput.C */

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
// Primary maintainer for this file:
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/IntegerInput.C $
// $Id: IntegerInput.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef CHANNELS_INTEGERINPUT_C_DEFINED
#define CHANNELS_INTEGERINPUT_C_DEFINED

#include "Channels/IntegerInput.H"

#include "Image/IntegerMathOps.H"
#include "Raster/GenericFrame.H"
#include "Util/MathFunctions.H"
#include "Video/VideoFrame.H"

#include <pthread.h>

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

  // Use this many extra bits in our fixed-point approximation to
  // floating-point arithmetic:
  const uint BITS_OUT = 16;

  // rgb lookup tables

  int SCALED_Y_tab[256];
  int SCALED_UV_tab[256];
  int RGB_Y_tab[256];
  int B_U_tab[256];
  int G_U_tab[256];
  int G_V_tab[256];
  int R_V_tab[256];

  pthread_once_t colorspace_init_once = PTHREAD_ONCE_INIT;

  void colorspace_init()
  {
    // so that we get proper rounding in fixed-point integer arithmetic:
    const int half =
      BITS_OUT > 0
      ? 1<<(BITS_OUT-1)
      : 0;

    const int scale = 1<<BITS_OUT;

    for (int i = 0; i < 256; i++)
      {
        const int y  = i-VIDEOYUV_Y_OFFSET;
        const int uv = i-VIDEOYUV_UV_OFFSET;

        SCALED_Y_tab[i] = half+int(0.5 + scale * ((256.0 * y) / VIDEOYUV_Y_RANGE));
        SCALED_UV_tab[i] =
          uv >= 0
          ? half+int(0.5 + scale * ((256.0 * uv) / VIDEOYUV_UV_RANGE))
          : -half-int(0.5 + scale * ((256.0 * (-uv)) / VIDEOYUV_UV_RANGE));
        RGB_Y_tab[i] = half+int(0.5 + y  * VIDEOYUV_RGB_Y * scale);
        R_V_tab[i]   =      int(0.5 + uv * VIDEOYUV_R_V   * scale);
        // FIXME should probably have -0.5 instead of +0.5 here and
        // flip the sign of G_U_tab and G_V_tab:
        G_U_tab[i]   =      int(0.5 - uv * VIDEOYUV_G_U   * scale);
        G_V_tab[i]   =      int(0.5 - uv * VIDEOYUV_G_V   * scale);
        B_U_tab[i]   =      int(0.5 + uv * VIDEOYUV_B_U   * scale);
      }
  }

  int clampInt(int val, int lo, int hi)
  {
    if (val < lo) return lo;
    else if (val > hi) return hi;
    return val;
  }
}

static void yuv420p_decode(int* rgb_dst,
                           int* bw_dst,
                           int* rg_dst,
                           int* by_dst,
                           const byte* y_src,
                           const byte* u_src,
                           const byte* v_src,
                           const int uv_stride,
                           const int width,
                           const int height,
                           const int lumthresh,
                           const uint inputbits)
{
  if (width <= 0)
    LFATAL("width must be positive");

  if (height <= 0)
    LFATAL("height must be positive");

  if (width & 1)
    LFATAL("width must be even");

  if (height & 1)
    LFATAL("height must be even");

  pthread_once(&colorspace_init_once, &colorspace_init);

  const int rgb_dst_dif = 3 * width;
  const int bw_dst_dif = width;
  const int y_dif = width;

  int* rgb_dst2 = rgb_dst + 3 * width;
  int* bw_dst2 = bw_dst + width;
  int* rg_dst2 = rg_dst + width;
  int* by_dst2 = by_dst + width;
  const byte* y_src2 = y_src + width;

  const uint srcbits = BITS_OUT + 8;

  if (inputbits > srcbits)
    LFATAL("Oops! Can't compute with inputbits (%u) > %u",
           inputbits, srcbits);

  const int rshiftbits = srcbits - inputbits;

  const uint lumbits = (inputbits * 2) / 3;
  const uint restorebits = (inputbits - lumbits - 1);

  const int shiftedlumthresh = lumthresh << rshiftbits;

  for (int y = height / 2; y; y--) {
    for (int x = 0; x < width / 2; x++) {
      const int u = u_src[x];
      const int v = v_src[x];

      const int r_v = R_V_tab[v];
      const int g_uv = - G_U_tab[u] - G_V_tab[v];
      const int b_u = B_U_tab[u];

      const int scaled_u = SCALED_UV_tab[u] >> 1;
      const int scaled_v = SCALED_UV_tab[v] >> 1;

#define DO_LUM(rgptr, byptr)                                            \
      do {                                                              \
        if (lum < shiftedlumthresh)                                     \
          {                                                             \
            *rgptr++ = 0;                                               \
            *byptr++ = 0;                                               \
          }                                                             \
        else                                                            \
          {                                                             \
            /* compute differences and normalize chroma by luminance: */ \
            *rgptr++ = (scaled_v / (lum >> lumbits)) << restorebits;    \
            *byptr++ = (scaled_u / (lum >> lumbits)) << restorebits;    \
          }                                                             \
      } while (0)

      {
        const int rgb_y = RGB_Y_tab[*y_src];
        const int r = (rgb_y + r_v);
        const int g = (rgb_y + g_uv);
        const int b = (rgb_y + b_u);
        const int lum = SCALED_Y_tab[*y_src];
        *bw_dst++ = clampInt(lum >> rshiftbits, 0, (1<<inputbits)-1);
        rgb_dst[0] = clampInt(r >> rshiftbits, 0, (1<<inputbits)-1);
        rgb_dst[1] = clampInt(g >> rshiftbits, 0, (1<<inputbits)-1);
        rgb_dst[2] = clampInt(b >> rshiftbits, 0, (1<<inputbits)-1);

        DO_LUM(rg_dst, by_dst);

        y_src++;
      }

      {
        const int rgb_y = RGB_Y_tab[*y_src];
        const int r = (rgb_y + r_v);
        const int g = (rgb_y + g_uv);
        const int b = (rgb_y + b_u);
        const int lum = SCALED_Y_tab[*y_src];
        *bw_dst++ = clampInt(lum >> rshiftbits, 0, (1<<inputbits)-1);
        rgb_dst[3] = clampInt(r >> rshiftbits, 0, (1<<inputbits)-1);
        rgb_dst[4] = clampInt(g >> rshiftbits, 0, (1<<inputbits)-1);
        rgb_dst[5] = clampInt(b >> rshiftbits, 0, (1<<inputbits)-1);

        DO_LUM(rg_dst, by_dst);

        y_src++;
      }

      {
        const int rgb_y = RGB_Y_tab[*y_src2];
        const int r = (rgb_y + r_v);
        const int g = (rgb_y + g_uv);
        const int b = (rgb_y + b_u);
        const int lum = SCALED_Y_tab[*y_src2];
        *bw_dst2++ = clampInt(lum >> rshiftbits, 0, (1<<inputbits)-1);
        rgb_dst2[0] = clampInt(r >> rshiftbits, 0, (1<<inputbits)-1);
        rgb_dst2[1] = clampInt(g >> rshiftbits, 0, (1<<inputbits)-1);
        rgb_dst2[2] = clampInt(b >> rshiftbits, 0, (1<<inputbits)-1);

        DO_LUM(rg_dst2, by_dst2);

        y_src2++;
      }

      {
        const int rgb_y = RGB_Y_tab[*y_src2];
        const int r = (rgb_y + r_v);
        const int g = (rgb_y + g_uv);
        const int b = (rgb_y + b_u);
        const int lum = SCALED_Y_tab[*y_src2];
        *bw_dst2++ = clampInt(lum >> rshiftbits, 0, (1<<inputbits)-1);
        rgb_dst2[3] = clampInt(r >> rshiftbits, 0, (1<<inputbits)-1);
        rgb_dst2[4] = clampInt(g >> rshiftbits, 0, (1<<inputbits)-1);
        rgb_dst2[5] = clampInt(b >> rshiftbits, 0, (1<<inputbits)-1);

        DO_LUM(rg_dst2, by_dst2);

        y_src2++;
      }

      rgb_dst += 6;
      rgb_dst2 += 6;
    }

    rgb_dst += rgb_dst_dif;
    rgb_dst2 += rgb_dst_dif;
    bw_dst += bw_dst_dif;
    bw_dst2 += bw_dst_dif;
    rg_dst += bw_dst_dif;
    rg_dst2 += bw_dst_dif;
    by_dst += bw_dst_dif;
    by_dst2 += bw_dst_dif;

    y_src += y_dif;
    y_src2 += y_dif;

    u_src += uv_stride;
    v_src += uv_stride;
  }
}

static void yuv422p_decode(int* rgb_dst,
                           int* bw_dst,
                           int* rg_dst,
                           int* by_dst,
                           const byte* y_src,
                           const byte* u_src,
                           const byte* v_src,
                           const int width,
                           const int height,
                           const int lumthresh,
                           const uint inputbits)
{
  if (width <= 0)
    LFATAL("width must be positive");

  if (height <= 0)
    LFATAL("height must be positive");

  if (width & 1)
    LFATAL("width must be even");

  if (height & 1)
    LFATAL("height must be even");

  pthread_once(&colorspace_init_once, &colorspace_init);

  const uint srcbits = BITS_OUT + 8;

  if (inputbits > srcbits)
    LFATAL("Oops! Can't compute with inputbits (%u) > %u",
           inputbits, srcbits);

  const int rshiftbits = srcbits - inputbits;

  const uint lumbits = (inputbits * 2) / 3;
  const uint restorebits = (inputbits - lumbits - 1);

  const int shiftedlumthresh = lumthresh << rshiftbits;


  for (int j = 0; j < height; ++j)
    for (int i = 0; i < width; i += 2)
      {
        // we have 2 luminance pixels per chroma pair

        const int r_v = R_V_tab[*v_src];
        const int g_uv = - G_U_tab[*u_src] - G_V_tab[*v_src];
        const int b_u = B_U_tab[*u_src];

        const int scaled_u = SCALED_UV_tab[*u_src] >> 1;
        const int scaled_v = SCALED_UV_tab[*v_src] >> 1;

        ++u_src;
        ++v_src;

        // first luminance pixel:

        {
          const int lum = SCALED_Y_tab[*y_src];
          const int rgb_y1 = RGB_Y_tab[*y_src++];

          const int r1 = (rgb_y1 + r_v);
          const int g1 = (rgb_y1 + g_uv);
          const int b1 = (rgb_y1 + b_u);

          *bw_dst++ = clampInt(lum >> rshiftbits, 0, (1<<inputbits)-1);
          *rgb_dst++ = clampInt(r1 >> rshiftbits, 0, (1<<inputbits)-1);
          *rgb_dst++ = clampInt(g1 >> rshiftbits, 0, (1<<inputbits)-1);
          *rgb_dst++ = clampInt(b1 >> rshiftbits, 0, (1<<inputbits)-1);

          DO_LUM(rg_dst, by_dst);
        }

        // second luminance pixel:

        {
          const int lum = SCALED_Y_tab[*y_src];
          const int rgb_y2 = RGB_Y_tab[*y_src++];

          const int r2 = (rgb_y2 + r_v);
          const int g2 = (rgb_y2 + g_uv);
          const int b2 = (rgb_y2 + b_u);

          *bw_dst++ = clampInt(lum >> rshiftbits, 0, (1<<inputbits)-1);
          *rgb_dst++ = clampInt(r2 >> rshiftbits, 0, (1<<inputbits)-1);
          *rgb_dst++ = clampInt(g2 >> rshiftbits, 0, (1<<inputbits)-1);
          *rgb_dst++ = clampInt(b2 >> rshiftbits, 0, (1<<inputbits)-1);

          DO_LUM(rg_dst, by_dst);
        }
      }
}

// ######################################################################
IntegerInput::IntegerInput()
  :
  itsDims(),
  itsRgbByte(),
  itsInputBits(0),
  itsGrayInt(),
  itsRgInt(),
  itsByInt()
{}

// ######################################################################
IntegerInput::IntegerInput(const Dims& dims)
  :
  itsDims(dims),
  itsRgbByte(),
  itsInputBits(0),
  itsGrayInt(dims, NO_INIT),
  itsRgInt(dims, NO_INIT),
  itsByInt(dims, NO_INIT)
{}

// ######################################################################
IntegerInput IntegerInput::fromRgb(const Image<PixRGB<byte> >& f,
                                   const uint inputbits)
{
  IntegerInput result;
  result.itsDims = f.getDims();
  result.itsRgbByte = f;
  result.itsInputBits = inputbits;
  result.itsGrayInt = intgScaleLuminanceFromByte(&f, inputbits);

  // don't compute itsRgInt and itsByInt here; instead, compute them
  // lazily if/when the user calls rgInt() or byInt()

  return result;
}

// ######################################################################
IntegerInput IntegerInput::fromVideo(const VideoFrame& vf,
                                     const uint inputbits)
{
  const byte* data = vf.getBuffer();
  const size_t buflen = vf.getBufSize();
  const Dims dims = vf.getDims();

  const int w = dims.w();
  const int h = dims.h();

  IntegerInput result(dims);

  result.itsInputBits = inputbits;

  Image<PixRGB<int> > irgb(dims, NO_INIT);

  const float lumthreshf = 25.5f;
  const int lumthresh = intgScaleFromFloat(&lumthreshf, inputbits);

  switch (vf.getMode())
    {
    case VIDFMT_YUV420P:
      {
        // we have to do (w+1)/2 instead of just w/2, because if e.g. the y
        // array has 5 pixels, then we want the u and v arrays to have 3
        // pixels, not 2:
        const int w2 = (w+1)/2;
        const int h2 = (h+1)/2;

        checkBufferLength(buflen, dims.sz() + 2*w2*h2);

        yuv420p_decode(reinterpret_cast<int*>(irgb.getArrayPtr()),
                       result.itsGrayInt.getArrayPtr(),
                       result.itsRgInt.getArrayPtr(),
                       result.itsByInt.getArrayPtr(),
                       data,
                       data + w*h,
                       data + w*h + w2*h2,
                       w2 /* uv_stride */,
                       w /* image width */,
                       h /* image height */,
                       lumthresh,
                       inputbits);

        return result;
      }
      break;

    case VIDFMT_YUV422P:
      {
        checkBufferLength(buflen, dims.sz() + 2*(w/2)*h);

        yuv422p_decode(reinterpret_cast<int*>(irgb.getArrayPtr()),
                       result.itsGrayInt.getArrayPtr(),
                       result.itsRgInt.getArrayPtr(),
                       result.itsByInt.getArrayPtr(),
                       data,
                       data + w*h,
                       data + w*h + (w/2) * h,
                       w /* image width */,
                       h /* image height */,
                       lumthresh,
                       inputbits);

        return result;
      }
      break;

    default:
      LFATAL("Oops! I don't have any specialized decoder for "
             "VideoFormat %s; just try using fromRgb() instead",
             convertToString(vf.getMode()).c_str());
      break;
    }

  return result;
}

// ######################################################################
IntegerInput IntegerInput::decode(const GenericFrame& f,
                                  const IntegerDecodeType t,
                                  const unsigned int bits)
{
  switch (t)
    {
    case INTG_DECODE_RGB: return IntegerInput::fromRgb(f.asRgb(), bits);
    case INTG_DECODE_VIDEO: return IntegerInput::fromVideo(f.asVideo(), bits);
    }

  LFATAL("Invalid IntegerDecodeType value %d", int(t));
  /* can't happen */ return IntegerInput();
}

// ######################################################################
IntegerInput IntegerInput::fromGrayOnly(const Image<int>& bw)
{
  IntegerInput result;
  result.itsDims = bw.getDims();
  result.itsGrayInt = bw;
  return result;
}

// ######################################################################
const Image<int>& IntegerInput::rgInt() const
{
  if (!itsRgInt.initialized() && itsRgbByte.initialized())
    {
      ASSERT(itsInputBits > 0);
      const int threshfactor = 10;
      intgGetRGBY(itsRgbByte, itsRgInt, itsByInt,
                  threshfactor, itsInputBits);
    }

  return itsRgInt;
}

// ######################################################################
const Image<int>& IntegerInput::byInt() const
{
  if (!itsByInt.initialized() && itsRgbByte.initialized())
    {
      ASSERT(itsInputBits > 0);
      const int threshfactor = 10;
      intgGetRGBY(itsRgbByte, itsRgInt, itsByInt,
                  threshfactor, itsInputBits);
    }

  return itsByInt;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_INTEGERINPUT_C_DEFINED

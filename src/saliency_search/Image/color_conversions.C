/*!@file Image/color_conversions.C low-level routines for colorspace conversions */

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
// Primary maintainer for this file: Rob Peters <rjpeters@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/color_conversions.C $
// $Id: color_conversions.C 8279 2007-04-20 21:44:59Z rjpeters $
//

#include "Image/color_conversions.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/colorDefs.H"  // for rgb<->yuv conversion factors
#include "Util/Promotions.H" // for clamped_convert()
#include "rutz/trace.h"

#include <pthread.h>

namespace
{
  // Use this many extra bits in our fixed-point approximation to
  // floating-point arithmetic:
  const int BITS_OUT = 16;

  // rgb lookup tables

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

        RGB_Y_tab[i] = half+int(0.5 + y  * VIDEOYUV_RGB_Y * scale);
        R_V_tab[i]   =      int(0.5 + uv * VIDEOYUV_R_V   * scale);
        // FIXME should probably have -0.5 instead of +0.5 here and
        // flip the sign of G_U_tab and G_V_tab:
        G_U_tab[i]   =      int(0.5 - uv * VIDEOYUV_G_U   * scale);
        G_V_tab[i]   =      int(0.5 - uv * VIDEOYUV_G_V   * scale);
        B_U_tab[i]   =      int(0.5 + uv * VIDEOYUV_B_U   * scale);
      }
  }
}

void yv12_to_rgb24_c(unsigned char* dst,
                     int dst_stride,
                     const unsigned char* y_src,
                     const unsigned char* u_src,
                     const unsigned char* v_src,
                     int y_stride,
                     int uv_stride,
                     int width,
                     int height)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (width & 1)
    LFATAL("width must be even");

  if (height & 1)
    LFATAL("height must be even");

  pthread_once(&colorspace_init_once, &colorspace_init);

  const int dst_dif = 6 * dst_stride - 3 * width;
  int y_dif = 2 * y_stride - width;

  unsigned char* dst2 = dst + 3 * dst_stride;
  const unsigned char* y_src2 = y_src + y_stride;

  if (height < 0) {                     /* flip image? */
    height = -height;
    y_src += (height - 1) * y_stride;
    y_src2 = y_src - y_stride;
    u_src += (height / 2 - 1) * uv_stride;
    v_src += (height / 2 - 1) * uv_stride;
    y_dif = -width - 2 * y_stride;
    uv_stride = -uv_stride;
  }

  for (int y = height / 2; y; y--) {
    for (int x = 0; x < width / 2; x++) {
      const int u = u_src[x];
      const int v = v_src[x];

      const int r_v = R_V_tab[v];
      const int g_uv = - G_U_tab[u] - G_V_tab[v];
      const int b_u = B_U_tab[u];

      {
        const int rgb_y = RGB_Y_tab[*y_src];
        const int r = (rgb_y + r_v) >> BITS_OUT;
        const int g = (rgb_y + g_uv) >> BITS_OUT;
        const int b = (rgb_y + b_u) >> BITS_OUT;
        dst[0] = clamped_convert<unsigned char>(r);
        dst[1] = clamped_convert<unsigned char>(g);
        dst[2] = clamped_convert<unsigned char>(b);
        y_src++;
      }
      {
        const int rgb_y = RGB_Y_tab[*y_src];
        const int r = (rgb_y + r_v) >> BITS_OUT;
        const int g = (rgb_y + g_uv) >> BITS_OUT;
        const int b = (rgb_y + b_u) >> BITS_OUT;
        dst[3] = clamped_convert<unsigned char>(r);
        dst[4] = clamped_convert<unsigned char>(g);
        dst[5] = clamped_convert<unsigned char>(b);
        y_src++;
      }
      {
        const int rgb_y = RGB_Y_tab[*y_src2];
        const int r = (rgb_y + r_v) >> BITS_OUT;
        const int g = (rgb_y + g_uv) >> BITS_OUT;
        const int b = (rgb_y + b_u) >> BITS_OUT;
        dst2[0] = clamped_convert<unsigned char>(r);
        dst2[1] = clamped_convert<unsigned char>(g);
        dst2[2] = clamped_convert<unsigned char>(b);
        y_src2++;
      }
      {
        const int rgb_y = RGB_Y_tab[*y_src2];
        const int r = (rgb_y + r_v) >> BITS_OUT;
        const int g = (rgb_y + g_uv) >> BITS_OUT;
        const int b = (rgb_y + b_u) >> BITS_OUT;
        dst2[3] = clamped_convert<unsigned char>(r);
        dst2[4] = clamped_convert<unsigned char>(g);
        dst2[5] = clamped_convert<unsigned char>(b);
        y_src2++;
      }

      dst += 6;
      dst2 += 6;
    }

    dst += dst_dif;
    dst2 += dst_dif;

    y_src += y_dif;
    y_src2 += y_dif;

    u_src += uv_stride;
    v_src += uv_stride;
  }
}

// NOTE: we have more precise values for these YUV/RGB conversion
// factors in Image/colorDefs.H, but for now we stick with the lower
// precision values here to maintain backward compatibility for
// callers of rgb24_to_yv12_c() (mainly FfmpegEncoder).

#define Y_R_IN   0.257 // VIDEOYUV_Y_R from Image/colorDefs.H
#define Y_G_IN   0.504 // VIDEOYUV_Y_G
#define Y_B_IN   0.098 // VIDEOYUV_Y_B
#define Y_ADD_IN 16

#define U_R_IN   0.148 // -VIDEOYUV_U_R
#define U_G_IN   0.291 // -VIDEOYUV_U_G
#define U_B_IN   0.439 // VIDEOYUV_U_B
#define U_ADD_IN 128

#define V_R_IN   0.439 // VIDEOYUV_V_R
#define V_G_IN   0.368 // -VIDEOYUV_V_G
#define V_B_IN   0.071 // -VIDEOYUV_V_B
#define V_ADD_IN 128

#define SCALEBITS_IN 8

#define FIX_IN(x) (int((x) * (1L<<SCALEBITS_IN) + 0.5))

void rgb24_to_yv12_c(const Image<PixRGB<byte> >& img,
                     byte* const y_out,
                     byte* u_out,
                     byte* v_out)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  const int width = img.getWidth();
  const int height = img.getHeight();

  const int width2 = width/2;
  const int height2 = height/2;

  typedef Image<PixRGB<byte> >::const_iterator src_iterator;

  const src_iterator in_begin = img.begin();

  for (int y = 0; y < height2; ++y)
    {
      // iterate in parallel over adjacent rows of both the input
      // image and the output Y image

      src_iterator src1 = in_begin + 2 * y * width;
      src_iterator src2 = src1 + width;

      byte* y_out1 = y_out + 2 * y * width;
      byte* y_out2 = y_out1 + width;

      for (int x = 0; x < width2; ++x)
        {
          uint r, g, b, r4, g4, b4;

          r4 = r = src1->red();
          g4 = g = src1->green();
          b4 = b = src1->blue();
          ++src1;
          *y_out1++ =
            (byte) ((FIX_IN(Y_R_IN) * r + FIX_IN(Y_G_IN) * g +
                     FIX_IN(Y_B_IN) * b) >> SCALEBITS_IN) + Y_ADD_IN;

          r4 += (r = src1->red());
          g4 += (g = src1->green());
          b4 += (b = src1->blue());
          ++src1;
          *y_out1++ =
            (byte) ((FIX_IN(Y_R_IN) * r + FIX_IN(Y_G_IN) * g +
                     FIX_IN(Y_B_IN) * b) >> SCALEBITS_IN) + Y_ADD_IN;

          r4 += (r = src2->red());
          g4 += (g = src2->green());
          b4 += (b = src2->blue());
          ++src2;
          *y_out2++ =
            (byte) ((FIX_IN(Y_R_IN) * r + FIX_IN(Y_G_IN) * g +
                     FIX_IN(Y_B_IN) * b) >> SCALEBITS_IN) + Y_ADD_IN;

          r4 += (r = src2->red());
          g4 += (g = src2->green());
          b4 += (b = src2->blue());
          ++src2;
          *y_out2++ =
            (byte) ((FIX_IN(Y_R_IN) * r + FIX_IN(Y_G_IN) * g +
                     FIX_IN(Y_B_IN) * b) >> SCALEBITS_IN) + Y_ADD_IN;

          *u_out++ =
            (byte) ((-FIX_IN(U_R_IN) * r4 - FIX_IN(U_G_IN) * g4 +
                     FIX_IN(U_B_IN) * b4) >> (SCALEBITS_IN + 2)) +
            U_ADD_IN;


          *v_out++ =
            (byte) ((FIX_IN(V_R_IN) * r4 - FIX_IN(V_G_IN) * g4 -
                     FIX_IN(V_B_IN) * b4) >> (SCALEBITS_IN + 2)) +
            V_ADD_IN;


          if (x==(width2-1) && (width&1))
            {
              r4 = r = src1->red();
              g4 = g = src1->green();
              b4 = b = src1->blue();
              ++src1;
              *y_out1++ =
                (byte) ((FIX_IN(Y_R_IN) * r + FIX_IN(Y_G_IN) * g +
                         FIX_IN(Y_B_IN) * b) >> SCALEBITS_IN) + Y_ADD_IN;

              r4 += (r = src2->red());
              g4 += (g = src2->green());
              b4 += (b = src2->blue());
              ++src2;
              *y_out2++ =
                (byte) ((FIX_IN(Y_R_IN) * r + FIX_IN(Y_G_IN) * g +
                         FIX_IN(Y_B_IN) * b) >> SCALEBITS_IN) + Y_ADD_IN;

              *u_out++ =
                (byte) ((-FIX_IN(U_R_IN) * r4 - FIX_IN(U_G_IN) * g4 +
                         FIX_IN(U_B_IN) * b4) >> (SCALEBITS_IN + 1)) +
                U_ADD_IN;

              *v_out++ =
                (byte) ((FIX_IN(V_R_IN) * r4 - FIX_IN(V_G_IN) * g4 -
                         FIX_IN(V_B_IN) * b4) >> (SCALEBITS_IN + 1)) +
                V_ADD_IN;
            }
        }
    }
}

void yuv422p_to_rgb24_c(byte* dst,
                        const int w, const int h,
                        const byte* yptr,
                        const byte* uptr,
                        const byte* vptr)
{
  pthread_once(&colorspace_init_once, &colorspace_init);

  for (int j = 0; j < h; ++j)
    for (int i = 0; i < w; i += 2)
      {
        // we have 2 luminance pixels per chroma pair

        const int r_v = R_V_tab[*vptr];
        const int g_uv = - G_U_tab[*uptr] - G_V_tab[*vptr];
        const int b_u = B_U_tab[*uptr];

        ++uptr;
        ++vptr;

        // first luminance pixel:

        {
          const int rgb_y1 = RGB_Y_tab[*yptr++];

          const int r1 = (rgb_y1 + r_v) >> BITS_OUT;
          const int g1 = (rgb_y1 + g_uv) >> BITS_OUT;
          const int b1 = (rgb_y1 + b_u) >> BITS_OUT;

          *dst++ = clamped_convert<byte>(r1);
          *dst++ = clamped_convert<byte>(g1);
          *dst++ = clamped_convert<byte>(b1);

        }

        // second luminance pixel:

        {
          const int rgb_y2 = RGB_Y_tab[*yptr++];

          const int r2 = (rgb_y2 + r_v) >> BITS_OUT;
          const int g2 = (rgb_y2 + g_uv) >> BITS_OUT;
          const int b2 = (rgb_y2 + b_u) >> BITS_OUT;

          *dst++ = clamped_convert<byte>(r2);
          *dst++ = clamped_convert<byte>(g2);
          *dst++ = clamped_convert<byte>(b2);
        }
      }
}

void yuv422_to_rgb24_c(byte* dst,
                       const int w, const int h,
                       const byte* yuv422ptr,
                       const bool byteswap)
{
  pthread_once(&colorspace_init_once, &colorspace_init);

  if (byteswap)
    for (int j = 0; j < h; ++j)
      for (int i = 0; i < w; i += 2)
        {
          // we have 2 luminance pixels per chroma pair

          const byte y1 = yuv422ptr[0];
          const byte u = yuv422ptr[1];
          const byte y2 = yuv422ptr[2];
          const byte v = yuv422ptr[3];

          yuv422ptr += 4;

          const int r_v = R_V_tab[v];
          const int g_uv = - G_U_tab[u] - G_V_tab[v];
          const int b_u = B_U_tab[u];

          // first luminance pixel:
          const int rgb_y1 = RGB_Y_tab[y1];

          *dst++ = clamped_convert<byte>((rgb_y1 + r_v) >> BITS_OUT);
          *dst++ = clamped_convert<byte>((rgb_y1 + g_uv) >> BITS_OUT);
          *dst++ = clamped_convert<byte>((rgb_y1 + b_u) >> BITS_OUT);

          // second luminance pixel:
          const int rgb_y2 = RGB_Y_tab[y2];

          *dst++ = clamped_convert<byte>((rgb_y2 + r_v) >> BITS_OUT);
          *dst++ = clamped_convert<byte>((rgb_y2 + g_uv) >> BITS_OUT);
          *dst++ = clamped_convert<byte>((rgb_y2 + b_u) >> BITS_OUT);
        }

  else // no byteswap
    for (int j = 0; j < h; ++j)
      for (int i = 0; i < w; i += 2)
        {
          // we have 2 luminance pixels per chroma pair

          const byte y1 = yuv422ptr[1];
          const byte u = yuv422ptr[0];
          const byte y2 = yuv422ptr[3];
          const byte v = yuv422ptr[2];

          yuv422ptr += 4;

          const int r_v = R_V_tab[v];
          const int g_uv = - G_U_tab[u] - G_V_tab[v];
          const int b_u = B_U_tab[u];

          // first luminance pixel:
          const int rgb_y1 = RGB_Y_tab[y1];

          *dst++ = clamped_convert<byte>((rgb_y1 + r_v) >> BITS_OUT);
          *dst++ = clamped_convert<byte>((rgb_y1 + g_uv) >> BITS_OUT);
          *dst++ = clamped_convert<byte>((rgb_y1 + b_u) >> BITS_OUT);

          // second luminance pixel:
          const int rgb_y2 = RGB_Y_tab[y2];

          *dst++ = clamped_convert<byte>((rgb_y2 + r_v) >> BITS_OUT);
          *dst++ = clamped_convert<byte>((rgb_y2 + g_uv) >> BITS_OUT);
          *dst++ = clamped_convert<byte>((rgb_y2 + b_u) >> BITS_OUT);
        }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/*!@file Raster/Debayer.C is the general debayer class */

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
// Primary maintainer for this file: Zhicheng Li <zhicheng@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/DeBayer.C $
// $Id: DeBayer.C 10794 2009-02-08 06:21:09Z itti $
//

#include <stdio.h>
#include <stdint.h>

#include "Util/log.H"
#include "Raster/DeBayer.H"
#include "Raster/DeBayerREG.H"

#ifdef INVT_USE_SSEDB
#include <emmintrin.h>
#include "Raster/DeBayerSSE2.H"
#ifdef INVT_USE_SSE3
#include <pmmintrin.h>
#include "Raster/DeBayerSSE3.H"
#endif
#endif

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Util/Types.H"

template <class T>
Image<PixRGB<T> > deBayer(const Image<T>& src, BayerFormat format)
{
  Image<PixRGB<T> >  res(src.getDims(), NO_INIT);
  int bitDepth = 8*sizeof(T);

  if(bitDepth == 8)
    {
      // use accelerated deBayer code?
      // now SSE2 and SSE3 only support 8 bit depth image
      // so althogh they use the template T type they only work for byte type
#ifdef INVT_USE_SSEDB

#ifdef INVT_USE_SSE3
      res = debayerSSE3(src,format);
#else
      res = debayerSSE2(src,format);
#endif

#else
      res = debayerREG(src,format);
#endif
    }
  else
    res = debayerREG(src, format);
  return res;
}

template Image<PixRGB<byte> >  deBayer(const Image<byte>& src, BayerFormat format);
template Image<PixRGB<uint16> >  deBayer(const Image<uint16>& src, BayerFormat format);

// #######################################################################//
int
replicateBorder_8u (uint8_t * src, int sstride, int width, int height)
{

    memcpy (src - sstride, src, width);
    memcpy (src + height * sstride, src + (height-1)*sstride, width);

    int i;
    for (i = -1; i < height+1; i++) {
        src[i*sstride-1] = src[i*sstride];
        src[i*sstride + width] = src[i*sstride + width - 1];
    }
    return 0;
}

// #######################################################################//
int
copy_8u_generic (const uint8_t *src, int sstride,
        uint8_t *dst, int dstride,
        int src_x, int src_y,
        int dst_x, int dst_y,
        int width, int height,
        int bits_per_pixel)
{
    if (bits_per_pixel % 8) return -1;
    int bytes_per_pixel = bits_per_pixel / 8;

    int i;
    for (i=0; i<height; i++) {
        uint8_t *dst_row = dst + (dst_y + i) * dstride;
        const uint8_t *src_row = src + (src_y + i) * sstride;

        memcpy (dst_row + dst_x * bytes_per_pixel,
                src_row + src_x * bytes_per_pixel,
                width * bytes_per_pixel);
    }
    return 0;
}


// ################### get the 4 plane of R GR GB B plane #################//
// #######################################################################//
// #######################################################################//
int
splitBayerPlanes_8u (uint8_t *dst[4], int dstride,
                     const uint8_t * src, int sstride,
                     int width, int height)
{
#ifndef INVT_USE_SSEDB
  LINFO("you need to have sse2 support");
  return -1;
#else
  __m128i mask;
  int i, j;

  for (i = 0; i < 4; i++) {
    if (!IS_ALIGNED16(dstride)) {
      LFATAL("splitBayerPlanes_8u: dst[%d] is not "
             "16-byte aligned\n",i);
      return -1;
    }
  }
  if (!IS_ALIGNED16(sstride)) {
    LFATAL("splitBayerPlanes_8u: src is not 16-byte "
             "aligned\n");
    return -1;
  }

  /* If the source stride is not a multiple of 32 pixels, we need to
   * fix up the last column at the end. */
  if (!IS_ALIGNED32 (sstride))
    width -= 8;

  mask = _mm_set1_epi16 (0xff);
  for (i = 0; i < height; i++) {
    uint8_t * drow1 = dst[0] + i * dstride;
    uint8_t * drow2 = dst[1] + i * dstride;
    uint8_t * drow3 = dst[2] + i * dstride;
    uint8_t * drow4 = dst[3] + i * dstride;
    const uint8_t * srow = src + 2*i*sstride;
    for (j = 0; j < width; j += 16) {
      __m128i s1, s2, t1, t2, out;
      s1 = _mm_load_si128 ((__m128i *)(srow + 2*j));
      s2 = _mm_load_si128 ((__m128i *)(srow + 2*j + 16));

      t1 = _mm_and_si128 (s1, mask);
      t2 = _mm_and_si128 (s2, mask);

      out = _mm_packus_epi16 (t1, t2);
      _mm_store_si128 ((__m128i *)(drow1 + j), out);

      t1 = _mm_srli_epi16 (s1, 8);
      t2 = _mm_srli_epi16 (s2, 8);

      out = _mm_packus_epi16 (t1, t2);
      _mm_store_si128 ((__m128i *)(drow2 + j), out);

      s1 = _mm_load_si128 ((__m128i *)(srow + sstride + 2*j));
      s2 = _mm_load_si128 ((__m128i *)(srow + sstride + 2*j + 16));

      t1 = _mm_and_si128 (s1, mask);
      t2 = _mm_and_si128 (s2, mask);

      out = _mm_packus_epi16 (t1, t2);
      _mm_store_si128 ((__m128i *)(drow3 + j), out);

      t1 = _mm_srli_epi16 (s1, 8);
      t2 = _mm_srli_epi16 (s2, 8);

      out = _mm_packus_epi16 (t1, t2);
      _mm_store_si128 ((__m128i *)(drow4 + j), out);
    }
  }
  if (IS_ALIGNED32 (sstride))
    return 0;

  /* Fix up the last column if necessary */
  const uint8_t * scol1 = src + 2*width;
  const uint8_t * scol2 = src + 2*width + sstride;
  uint8_t * dcol1 = dst[0] + width;
  uint8_t * dcol2 = dst[1] + width;
  uint8_t * dcol3 = dst[2] + width;
  uint8_t * dcol4 = dst[3] + width;
  __m128i t2 = _mm_set1_epi16 (0);
  for (i = 0; i < height; i++) {
    __m128i s1, t1, out;
    s1 = _mm_load_si128 ((__m128i *)(scol1 + 2*i*sstride));
    t1 = _mm_and_si128 (s1, mask);

    out = _mm_packus_epi16 (t1, t2);
    _mm_store_si128 ((__m128i *)(dcol1 + i*dstride), out);

    t1 = _mm_srli_epi16 (s1, 8);

    out = _mm_packus_epi16 (t1, t2);
    _mm_store_si128 ((__m128i *)(dcol2 + i*dstride), out);

    s1 = _mm_load_si128 ((__m128i *)(scol2 + 2*i*sstride));
    t1 = _mm_and_si128 (s1, mask);

    out = _mm_packus_epi16 (t1, t2);
    _mm_store_si128 ((__m128i *)(dcol3 + i*dstride), out);

    t1 = _mm_srli_epi16 (s1, 8);

    out = _mm_packus_epi16 (t1, t2);
    _mm_store_si128 ((__m128i *)(dcol4 + i*dstride), out);
  }
  return 0;
#endif
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

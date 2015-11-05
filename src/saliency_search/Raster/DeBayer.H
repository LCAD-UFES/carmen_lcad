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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/DeBayer.H $
// $Id: DeBayer.H 10794 2009-02-08 06:21:09Z itti $
//

#ifndef __DEBAYER_H__
#define __DEBAYER_H__

#include <stdio.h>
#include <stdint.h>
#ifdef INVT_USE_SSEDB
#include <emmintrin.h>
#ifdef INVT_USE_SSE3
#include <pmmintrin.h>
#endif
#endif
#include "Image/Image.H"
#include "Image/Pixels.H"

#define CPUID(func,ax,bx,cx,dx)                                \
  __asm__ __volatile__ (                                 \
                        "xchgl %%ebx, %1    \n\t"         \
                        "cpuid              \n\t"                        \
                        "xchgl %%ebx, %1    \n\t"                        \
                        : "=a" (ax), "=r" (bx), "=c" (cx), "=d" (dx)        \
                        : "a" (func)                                        \
                        : "cc")

#define IS_ALIGNED16(x) (((uintptr_t)(x) & 0xf) == 0)
#define IS_ALIGNED32(x) (((uintptr_t)(x) & 0x1f) == 0)
#define IS_ALIGNED128(x) (((uintptr_t)(x) & 0x7f) == 0)

#ifdef __cplusplus
extern "C" {
#endif
}

enum BayerFormat
  {
    //! 8 bit depth (1 byte) bayer type
    BAYER_GBRG,
    BAYER_GRBG,
    BAYER_RGGB,
    BAYER_BGGR,
    //! 12 bit depth (actually use 2 byte) bayer type
    BAYER_GBRG12,
    BAYER_GRBG12,
    BAYER_RGGB12,
    BAYER_BGGR12
  };

inline void cpuid_detect (int * sse2, int * sse3)
{
  int a, b, c, d;
  CPUID (1, a, b, c, d);

  if (sse2)
    *sse2 = (d >> 26) & 1;
  if (sse3)
    *sse3 = c & 1;
}

template<class T>
Image<PixRGB<T> > deBayer(const Image<T>& src,
                             BayerFormat format =BAYER_GBRG);

int
splitBayerPlanes_8u (uint8_t *dst[4], int dstride,
                                 const uint8_t * src, int sstride,
                                 int width, int height);

int
replicateBorder_8u (uint8_t * src, int sstride, int width, int height);

int
copy_8u_generic (const uint8_t *src, int sstride,
                           uint8_t *dst, int dstride,
                           int src_x, int src_y,
                           int dst_x, int dst_y,
                           int width, int height,
                           int bits_per_pixel);


/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif

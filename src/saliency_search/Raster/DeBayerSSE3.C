/*!@file Raster/DebayerSSE3.C is the debayer class with sse3 */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/DeBayerSSE3.C $
// $Id: DeBayerSSE3.C 10794 2009-02-08 06:21:09Z itti $
//
#include <stdio.h>
#include <stdint.h>
#include <emmintrin.h>
#include <stdlib.h>

// on some platforms, memalign is defined in <malloc.h>, but that file
// does not exist on Darwin. On Darwin, including stdlib.h is sufficient.
// Let's here also include malloc.h unless we are on Darwin:
#ifndef MACHINE_OS_DARWIN
#include <malloc.h>
#endif

#include "Image/Image.H"
#include "Image/CutPaste.H"
#include "Raster/DeBayerSSE3.H"
#include "Raster/DeBayerSSE2.H"

// ########################  debayer with SSE3 accelerate   ##############//
// #######################################################################//


/* BOX_FILT evaluates this kernel:
 *     1  1
 *     1  1
 * For a 1x16 strip of pixels of an 8u image.  v1 and v2 hold the result of
 * the computation (stored as 16s).  ptr points to the first pixel of the
 * strip, and must be 16-byte aligned.  str is the stride of image rows in
 * bytes.  If stride is positive, the origin of the kernel is in the top
 * row, if negative, the origin is in the bottom row.  off is 1 to put
 * the origin in the left column of the kernel, or -1 to put the origin
 * in the right column.
 */
#define BOX_FILT(v1,v2,ptr,str,off) do { \
    __m128i t1, t2, t3; \
    t1 = _mm_load_si128 ((__m128i *)(ptr)); \
    v1 = _mm_unpacklo_epi8 (t1, z); \
    v2 = _mm_unpackhi_epi8 (t1, z); \
    t1 = _mm_load_si128 ((__m128i *)((ptr) + (str))); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_add_epi16 (v1, t2); \
    v2 = _mm_add_epi16 (v2, t3); \
    t1 = _mm_lddqu_si128 ((__m128i *)((ptr) + off)); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_add_epi16 (v1, t2); \
    v2 = _mm_add_epi16 (v2, t3); \
    t1 = _mm_lddqu_si128 ((__m128i *)((ptr) + (str) + off)); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_add_epi16 (v1, t2); \
    v2 = _mm_add_epi16 (v2, t3); \
} while (0)

/* CROSS_FILT_VERT evaluates this kernel:
 *         1/2
 *     -1   5  -1
 *         1/2
 * For a 1x16 strip of pixels of an 8u image.  v1 and v2 hold the result of
 * the computation (stored as 16s).  ptr points to the first pixel of the
 * strip, and must be 16-byte aligned.  str is the stride of image rows in
 * bytes.  The origin of the kernel is at the center.
 */
#define CROSS_FILT_VERT(v1,v2,ptr,str) do { \
    __m128i t1, t2, t3, c10; \
    c10 = _mm_set1_epi16 (10); \
    t1 = _mm_load_si128 ((__m128i *)(ptr)); \
    v1 = _mm_unpacklo_epi8 (t1, z); \
    v2 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_mullo_epi16 (v1, c10); \
    v2 = _mm_mullo_epi16 (v2, c10); \
    t1 = _mm_load_si128 ((__m128i *)((ptr) - (str))); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_add_epi16 (v1, t2); \
    v2 = _mm_add_epi16 (v2, t3); \
    t1 = _mm_load_si128 ((__m128i *)((ptr) + (str))); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_add_epi16 (v1, t2); \
    v2 = _mm_add_epi16 (v2, t3); \
    v1 = _mm_srli_epi16 (v1, 1); \
    v2 = _mm_srli_epi16 (v2, 1); \
    t1 = _mm_lddqu_si128 ((__m128i *)((ptr) - 1)); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_subs_epi16 (v1, t2); \
    v2 = _mm_subs_epi16 (v2, t3); \
    t1 = _mm_lddqu_si128 ((__m128i *)((ptr) + 1)); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_subs_epi16 (v1, t2); \
    v2 = _mm_subs_epi16 (v2, t3); \
} while (0)

/* HORIZ2_FILT evaluates this kernel:
 *     1  1
 * For a 1x16 strip of pixels of an 8u image.  v1 and v2 hold the result of
 * the computation (stored as 16s).  ptr points to the first pixel of the
 * strip, and must be 16-byte aligned.  str is the stride of image rows in
 * bytes (unused).  off is 1 to put the origin in the left column of the
 * kernel, or -1 to put the origin in the right column.
 */
#define HORIZ2_FILT(v1,v2,ptr,str,off) do { \
    __m128i t1, t2, t3; \
    t1 = _mm_load_si128 ((__m128i *)(ptr)); \
    v1 = _mm_unpacklo_epi8 (t1, z); \
    v2 = _mm_unpackhi_epi8 (t1, z); \
    t1 = _mm_lddqu_si128 ((__m128i *)((ptr) + off)); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_add_epi16 (v1, t2); \
    v2 = _mm_add_epi16 (v2, t3); \
} while (0)

/* VERT2_FILT evaluates this kernel:
 *     1
 *     1
 * For a 1x16 strip of pixels of an 8u image.  v1 and v2 hold the result of
 * the computation (stored as 16s).  ptr points to the first pixel of the
 * strip, and must be 16-byte aligned.  str is the stride of image rows in
 * bytes.  If stride is positive, the origin of the kernel is in the top
 * row, if negative, the origin is in the bottom row.
 */
#define VERT2_FILT(v1,v2,ptr,str) do { \
    __m128i t1, t2, t3; \
    t1 = _mm_load_si128 ((__m128i *)(ptr)); \
    v1 = _mm_unpacklo_epi8 (t1, z); \
    v2 = _mm_unpackhi_epi8 (t1, z); \
    t1 = _mm_load_si128 ((__m128i *)((ptr) + (str))); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_add_epi16 (v1, t2); \
    v2 = _mm_add_epi16 (v2, t3); \
} while (0)

/* CROSS_FILT_SYM evaluates this kernel:
 *         -1
 *     -1   4  -1
 *         -1
 * For a 1x16 strip of pixels of an 8u image.  v1 and v2 hold the result of
 * the computation (stored as 16s).  ptr points to the first pixel of the
 * strip, and must be 16-byte aligned.  str is the stride of image rows in
 * bytes.  The origin of the kernel is at the center.
 */
#define CROSS_FILT_SYM(v1,v2,ptr,str) do { \
    __m128i t1, t2, t3; \
    t1 = _mm_load_si128 ((__m128i *)(ptr)); \
    v1 = _mm_unpacklo_epi8 (t1, z); \
    v2 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_slli_epi16 (v1, 2); \
    v2 = _mm_slli_epi16 (v2, 2); \
    t1 = _mm_load_si128 ((__m128i *)((ptr) - (str))); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_subs_epi16 (v1, t2); \
    v2 = _mm_subs_epi16 (v2, t3); \
    t1 = _mm_load_si128 ((__m128i *)((ptr) + (str))); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_subs_epi16 (v1, t2); \
    v2 = _mm_subs_epi16 (v2, t3); \
    t1 = _mm_lddqu_si128 ((__m128i *)((ptr) - 1)); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_subs_epi16 (v1, t2); \
    v2 = _mm_subs_epi16 (v2, t3); \
    t1 = _mm_lddqu_si128 ((__m128i *)((ptr) + 1)); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_subs_epi16 (v1, t2); \
    v2 = _mm_subs_epi16 (v2, t3); \
} while (0)

/* CROSS_FILT_HORIZ evaluates this kernel:
 *         -1
 *     1/2  5  1/2
 *         -1
 * For a 1x16 strip of pixels of an 8u image.  v1 and v2 hold the result of
 * the computation (stored as 16s).  ptr points to the first pixel of the
 * strip, and must be 16-byte aligned.  str is the stride of image rows in
 * bytes.  The origin of the kernel is at the center.
 */
#define CROSS_FILT_HORIZ(v1,v2,ptr,str) do { \
    __m128i t1, t2, t3, c10; \
    c10 = _mm_set1_epi16 (10); \
    t1 = _mm_load_si128 ((__m128i *)(ptr)); \
    v1 = _mm_unpacklo_epi8 (t1, z); \
    v2 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_mullo_epi16 (v1, c10); \
    v2 = _mm_mullo_epi16 (v2, c10); \
    t1 = _mm_lddqu_si128 ((__m128i *)((ptr) - 1)); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_add_epi16 (v1, t2); \
    v2 = _mm_add_epi16 (v2, t3); \
    t1 = _mm_lddqu_si128 ((__m128i *)((ptr) + 1)); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_add_epi16 (v1, t2); \
    v2 = _mm_add_epi16 (v2, t3); \
    v1 = _mm_srli_epi16 (v1, 1); \
    v2 = _mm_srli_epi16 (v2, 1); \
    t1 = _mm_load_si128 ((__m128i *)((ptr) - (str))); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_subs_epi16 (v1, t2); \
    v2 = _mm_subs_epi16 (v2, t3); \
    t1 = _mm_load_si128 ((__m128i *)((ptr) + (str))); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_subs_epi16 (v1, t2); \
    v2 = _mm_subs_epi16 (v2, t3); \
} while (0)

#define INTERPOLATE_GB_ROW(kstride, off) do { \
    CROSS_FILT_VERT (v1, v2, gb_plane + j*sstride, kstride); \
    HORIZ2_FILT (w1, w2, b_plane + j*sstride, kstride, -off); \
    w1 = _mm_slli_epi16 (w1, 2); \
    w2 = _mm_slli_epi16 (w2, 2); \
    v1 = _mm_add_epi16 (v1, w1); \
    v2 = _mm_add_epi16 (v2, w2); \
    BOX_FILT (w1, w2, gr_plane + j*sstride, -kstride, -off); \
    v1 = _mm_subs_epi16 (v1, w1); \
    v2 = _mm_subs_epi16 (v2, w2); \
    v1 = _mm_srai_epi16 (v1, 3); \
    v2 = _mm_srai_epi16 (v2, 3); \
    bg = _mm_packus_epi16 (v1, v2); \
    \
    VERT2_FILT (v1, v2, gr_plane + j*sstride, -kstride); \
    HORIZ2_FILT (w1, w2, gb_plane + j*sstride, kstride, off); \
    v1 = _mm_add_epi16 (v1, w1); \
    v2 = _mm_add_epi16 (v2, w2); \
    v1 = _mm_slli_epi16 (v1, 1); \
    v2 = _mm_slli_epi16 (v2, 1); \
    CROSS_FILT_SYM (w1, w2, b_plane + j*sstride, kstride); \
    v1 = _mm_add_epi16 (v1, w1); \
    v2 = _mm_add_epi16 (v2, w2); \
    v1 = _mm_srai_epi16 (v1, 3); \
    v2 = _mm_srai_epi16 (v2, 3); \
    gb = _mm_packus_epi16 (v1, v2); \
    \
    CROSS_FILT_HORIZ (v1, v2, gb_plane + j*sstride, kstride); \
    VERT2_FILT (w1, w2, r_plane + j*sstride, -kstride); \
    w1 = _mm_slli_epi16 (w1, 2); \
    w2 = _mm_slli_epi16 (w2, 2); \
    v1 = _mm_add_epi16 (v1, w1); \
    v2 = _mm_add_epi16 (v2, w2); \
    BOX_FILT (w1, w2, gr_plane + j*sstride, -kstride, -off); \
    v1 = _mm_subs_epi16 (v1, w1); \
    v2 = _mm_subs_epi16 (v2, w2); \
    v1 = _mm_srai_epi16 (v1, 3); \
    v2 = _mm_srai_epi16 (v2, 3); \
    rg = _mm_packus_epi16 (v1, v2); \
    \
    CROSS_FILT_SYM (v1, v2, b_plane + j*sstride, kstride); \
    v1 = _mm_mullo_epi16 (v1, c3); \
    v2 = _mm_mullo_epi16 (v2, c3); \
    BOX_FILT (w1, w2, r_plane + j*sstride, -kstride, off); \
    w1 = _mm_slli_epi16 (w1, 2); \
    w2 = _mm_slli_epi16 (w2, 2); \
    v1 = _mm_add_epi16 (v1, w1); \
    v2 = _mm_add_epi16 (v2, w2); \
    v1 = _mm_srai_epi16 (v1, 4); \
    v2 = _mm_srai_epi16 (v2, 4); \
    rb = _mm_packus_epi16 (v1, v2); \
    \
    gg = _mm_load_si128 ((__m128i *)(gb_plane + j*sstride)); \
    bgl1 = _mm_unpacklo_epi8 (bg, gg); \
    bgl2 = _mm_unpackhi_epi8 (bg, gg); \
    \
    a = _mm_set1_epi8 (0xff); \
    ral1 = _mm_unpacklo_epi8 (rg, a); \
    ral2 = _mm_unpackhi_epi8 (rg, a); \
    \
    bb = _mm_load_si128 ((__m128i *)(b_plane + j*sstride)); \
    bgr1 = _mm_unpacklo_epi8 (bb, gb); \
    bgr2 = _mm_unpackhi_epi8 (bb, gb); \
    \
    rar1 = _mm_unpacklo_epi8 (rb, a); \
    rar2 = _mm_unpackhi_epi8 (rb, a); \
    \
    bgral1 = _mm_unpacklo_epi16 (bgl1, ral1); \
    bgral2 = _mm_unpackhi_epi16 (bgl1, ral1); \
    bgral3 = _mm_unpacklo_epi16 (bgl2, ral2); \
    bgral4 = _mm_unpackhi_epi16 (bgl2, ral2); \
    \
    bgrar1 = _mm_unpacklo_epi16 (bgr1, rar1); \
    bgrar2 = _mm_unpackhi_epi16 (bgr1, rar1); \
    bgrar3 = _mm_unpacklo_epi16 (bgr2, rar2); \
    bgrar4 = _mm_unpackhi_epi16 (bgr2, rar2); \
} while (0)

#define INTERPOLATE_RG_ROW(kstride,off) do { \
    CROSS_FILT_SYM (v1, v2, r_plane + j*sstride, kstride); \
    v1 = _mm_mullo_epi16 (v1, c3); \
    v2 = _mm_mullo_epi16 (v2, c3); \
    BOX_FILT (w1, w2, b_plane + j*sstride, kstride, -off); \
    w1 = _mm_slli_epi16 (w1, 2); \
    w2 = _mm_slli_epi16 (w2, 2); \
    v1 = _mm_add_epi16 (v1, w1); \
    v2 = _mm_add_epi16 (v2, w2); \
    v1 = _mm_srai_epi16 (v1, 4); \
    v2 = _mm_srai_epi16 (v2, 4); \
    br = _mm_packus_epi16 (v1, v2); \
    \
    VERT2_FILT (v1, v2, gb_plane + j*sstride, kstride); \
    HORIZ2_FILT (w1, w2, gr_plane + j*sstride, kstride, -off); \
    v1 = _mm_add_epi16 (v1, w1); \
    v2 = _mm_add_epi16 (v2, w2); \
    v1 = _mm_slli_epi16 (v1, 1); \
    v2 = _mm_slli_epi16 (v2, 1); \
    CROSS_FILT_SYM (w1, w2, r_plane + j*sstride, kstride); \
    v1 = _mm_add_epi16 (v1, w1); \
    v2 = _mm_add_epi16 (v2, w2); \
    v1 = _mm_srai_epi16 (v1, 3); \
    v2 = _mm_srai_epi16 (v2, 3); \
    gr = _mm_packus_epi16 (v1, v2); \
    \
    CROSS_FILT_HORIZ (v1, v2, gr_plane + j*sstride, kstride); \
    VERT2_FILT (w1, w2, b_plane + j*sstride, kstride); \
    w1 = _mm_slli_epi16 (w1, 2); \
    w2 = _mm_slli_epi16 (w2, 2); \
    v1 = _mm_add_epi16 (v1, w1); \
    v2 = _mm_add_epi16 (v2, w2); \
    BOX_FILT (w1, w2, gb_plane + j*sstride, kstride, off); \
    v1 = _mm_subs_epi16 (v1, w1); \
    v2 = _mm_subs_epi16 (v2, w2); \
    v1 = _mm_srai_epi16 (v1, 3); \
    v2 = _mm_srai_epi16 (v2, 3); \
    bg = _mm_packus_epi16 (v1, v2); \
    \
    CROSS_FILT_VERT (v1, v2, gr_plane + j*sstride, kstride); \
    HORIZ2_FILT (w1, w2, r_plane + j*sstride, kstride, off); \
    w1 = _mm_slli_epi16 (w1, 2); \
    w2 = _mm_slli_epi16 (w2, 2); \
    v1 = _mm_add_epi16 (v1, w1); \
    v2 = _mm_add_epi16 (v2, w2); \
    BOX_FILT (w1, w2, gb_plane + j*sstride, kstride, off); \
    v1 = _mm_subs_epi16 (v1, w1); \
    v2 = _mm_subs_epi16 (v2, w2); \
    v1 = _mm_srai_epi16 (v1, 3); \
    v2 = _mm_srai_epi16 (v2, 3); \
    rg = _mm_packus_epi16 (v1, v2); \
    \
    bgl1 = _mm_unpacklo_epi8 (br, gr); \
    bgl2 = _mm_unpackhi_epi8 (br, gr); \
    \
    rr = _mm_load_si128 ((__m128i *)(r_plane + j*sstride)); \
    a = _mm_set1_epi8 (0xff); \
    ral1 = _mm_unpacklo_epi8 (rr, a); \
    ral2 = _mm_unpackhi_epi8 (rr, a); \
    \
    gg = _mm_load_si128 ((__m128i *)(gr_plane + j*sstride)); \
    bgr1 = _mm_unpacklo_epi8 (bg, gg); \
    bgr2 = _mm_unpackhi_epi8 (bg, gg); \
    \
    rar1 = _mm_unpacklo_epi8 (rg, a); \
    rar2 = _mm_unpackhi_epi8 (rg, a); \
    \
    bgral1 = _mm_unpacklo_epi16 (bgl1, ral1); \
    bgral2 = _mm_unpackhi_epi16 (bgl1, ral1); \
    bgral3 = _mm_unpacklo_epi16 (bgl2, ral2); \
    bgral4 = _mm_unpackhi_epi16 (bgl2, ral2); \
    \
    bgrar1 = _mm_unpacklo_epi16 (bgr1, rar1); \
    bgrar2 = _mm_unpackhi_epi16 (bgr1, rar1); \
    bgrar3 = _mm_unpacklo_epi16 (bgr2, rar2); \
    bgrar4 = _mm_unpackhi_epi16 (bgr2, rar2); \
} while (0)

template <class T> Image<PixRGB<T> >
debayerSSE3 (const Image<T>& src1,
             BayerFormat format)
{
# ifndef INVT_USE_SSE3
  LFATAL("you must have sse3 support");
  return Image<PixRGB<T> >();
#else

  /* make sure that the source image stride can be divied by 32 */
  bool isAligned32 = true;
  int patchWidth = 0;
  Image<T> src;
  if ((src1.getWidth() % 32) != 0)
    {
      patchWidth = 32 - (src1.getWidth() % 32);
      src = concatX(src1, Image<T>(patchWidth, src1.getHeight(), ZEROS));
      isAligned32 = false;
    }
  else
    src = src1;

  int width = src.getWidth();
  int height = src.getHeight();
  ASSERT(width % 2 == 0);
  ASSERT(height % 2 == 0);
  int dstride = width * 4;
  int sstride = width;

  /* ensure stride is 16-byte aligned and add 32 extra bytes for the
   * border padding */
  uint8_t *bayer_planes[4];
  int plane_stride = ((width + 0xf)&(~0xf)) + 32;
  for (int i = 0; i < 4; i++) {
    bayer_planes[i] = (uint8_t*)memalign(16,plane_stride * (height + 2));
  }

  // alocate a 16-byte aligned buffer for the interpolated image
  int bgra_stride = width*4;
  uint8_t *bgra_img = (uint8_t*)memalign(16,height * bgra_stride);

  // allocate a 16-byte aligned buffer for the source image
  int bayer_stride = width;
  uint8_t *bayer_img = (uint8_t*) memalign(16,height * bayer_stride);

  // copy the source image into the 16-byte aligned buffer
  copy_8u_generic ((uint8_t*)src.getArrayPtr(), sstride,
                             bayer_img, bayer_stride,
                             0, 0, 0, 0, width, height, 8);

  // split the bayer image
  uint8_t * planes[4] = {
    bayer_planes[0] + plane_stride + 16,
    bayer_planes[1] + plane_stride + 16,
    bayer_planes[2] + plane_stride + 16,
    bayer_planes[3] + plane_stride + 16,
  };
  int p_width = width / 2;
  int p_height = height / 2;

  splitBayerPlanes_8u (planes, plane_stride,
                                   bayer_img, bayer_stride, p_width, p_height);
  for (int j = 0; j < 4; j++)
    replicateBorder_8u (planes[j], plane_stride, p_width, p_height);


  if( bayerInterpolateTo_8u_bgra_sse3 (planes,plane_stride,
                                       bgra_img, bgra_stride,
                                       width, height, format) < 0)
    LFATAL("error in debayer with sse3");
  // copy to destination
  uint8_t * dest = (uint8_t*)memalign(16, dstride*height);
  copy_8u_generic (bgra_img, bgra_stride,
                             dest, dstride, 0, 0, 0, 0, width, height, 8 * 4);

  Image<PixRGB<T> > res(width, height, NO_INIT);
  typename Image<PixRGB<T> >::iterator dptr = res.beginw();
  T* sptr = (T*)dest;

  for(int y =0; y < height; y++)
    {
      for(int x =0; x < width; x++)
        {
          dptr[0].p[2] = *sptr++;
          dptr[0].p[1] = *sptr++;
          dptr[0].p[0] = *sptr++;
          dptr++;
          sptr++; // for the A channel
        }
    }

  for (int i=0; i<4; i++) {
    free (bayer_planes[i]);
  }
  free(dest);
  free(bayer_img);
  free (bgra_img);

  if(!isAligned32)
    res = crop(res, Point2D<int>(0,0), Dims(width-patchWidth, height));
  return res;
#endif //INVT_USE_SSE3
}

int
bayerInterpolateTo_8u_bgra_sse3 (uint8_t ** src, int sstride,
                                 uint8_t * dst, int dstride, int width, int height,
                                 BayerFormat format)
{
# ifndef INVT_USE_SSE3
  LFATAL("you must have sse3 support");
  return -1;
#else
  int i, j;
  for (i = 0; i < 4; i++) {
    if (!IS_ALIGNED16(src[i]) || !IS_ALIGNED16(sstride)) {
      LERROR("%s: src[%d] is not 16-byte aligned\n",
               __FUNCTION__, i);
      return -1;
    }
  }
  if (!IS_ALIGNED16(dst) || !IS_ALIGNED128(dstride)) {
    LERROR("%s: dst is not 16-byte aligned or 128-byte stride "
             "aligned\n", __FUNCTION__);
    return -1;
  }

    __m128i z = _mm_set1_epi32 (0);
    __m128i c3 = _mm_set1_epi16 (3);
    __m128i bg, gb, rg, rb, gg, a, bb, br, gr, rr;
    __m128i bgl1, bgl2, ral1, ral2;
    __m128i bgr1, bgr2, rar1, rar2;
    __m128i bgral1, bgral2, bgral3, bgral4;
    __m128i bgrar1, bgrar2, bgrar3, bgrar4;
    __m128i v1, v2, w1, w2;

    if (format ==  BAYER_GBRG ||
            format ==  BAYER_RGGB) {
        int drow_offset1 = 0;
        int drow_offset2 = dstride;
        int kernel_stride = sstride;
        uint8_t * gb_plane = src[0];
        uint8_t * b_plane = src[1];
        uint8_t * r_plane = src[2];
        uint8_t * gr_plane = src[3];
        if (format ==  BAYER_RGGB) {
            drow_offset1 = dstride;
            drow_offset2 = 0;
            kernel_stride = -sstride;
            r_plane = src[0];
            gr_plane = src[1];
            gb_plane = src[2];
            b_plane = src[3];
        }

        for (i = 0; i < width/2; i += 16) {
            uint8_t * dcol = dst + i*8;

            for (j = 0; j < height/2; j++) {
                INTERPOLATE_GB_ROW (kernel_stride, 1);

                uint8_t * drow = dcol + j*2*dstride + drow_offset1;
                _mm_store_si128 ((__m128i *)drow,
                        _mm_unpacklo_epi32 (bgral1, bgrar1));
                _mm_store_si128 ((__m128i *)(drow+16),
                        _mm_unpackhi_epi32 (bgral1, bgrar1));
                _mm_store_si128 ((__m128i *)(drow+32),
                        _mm_unpacklo_epi32 (bgral2, bgrar2));
                _mm_store_si128 ((__m128i *)(drow+48),
                        _mm_unpackhi_epi32 (bgral2, bgrar2));
                _mm_store_si128 ((__m128i *)(drow+64),
                        _mm_unpacklo_epi32 (bgral3, bgrar3));
                _mm_store_si128 ((__m128i *)(drow+80),
                        _mm_unpackhi_epi32 (bgral3, bgrar3));
                _mm_store_si128 ((__m128i *)(drow+96),
                        _mm_unpacklo_epi32 (bgral4, bgrar4));
                _mm_store_si128 ((__m128i *)(drow+112),
                        _mm_unpackhi_epi32 (bgral4, bgrar4));

                INTERPOLATE_RG_ROW (kernel_stride, 1);

                drow = dcol + j*2*dstride + drow_offset2;
                _mm_store_si128 ((__m128i *)drow,
                        _mm_unpacklo_epi32 (bgral1, bgrar1));
                _mm_store_si128 ((__m128i *)(drow+16),
                        _mm_unpackhi_epi32 (bgral1, bgrar1));
                _mm_store_si128 ((__m128i *)(drow+32),
                        _mm_unpacklo_epi32 (bgral2, bgrar2));
                _mm_store_si128 ((__m128i *)(drow+48),
                        _mm_unpackhi_epi32 (bgral2, bgrar2));
                _mm_store_si128 ((__m128i *)(drow+64),
                        _mm_unpacklo_epi32 (bgral3, bgrar3));
                _mm_store_si128 ((__m128i *)(drow+80),
                        _mm_unpackhi_epi32 (bgral3, bgrar3));
                _mm_store_si128 ((__m128i *)(drow+96),
                        _mm_unpacklo_epi32 (bgral4, bgrar4));
                _mm_store_si128 ((__m128i *)(drow+112),
                        _mm_unpackhi_epi32 (bgral4, bgrar4));

            }
            gb_plane += 16;
            b_plane += 16;
            r_plane += 16;
            gr_plane += 16;
        }
    }
    else {
        int drow_offset1 = 0;
        int drow_offset2 = dstride;
        int kernel_stride = sstride;
        uint8_t * b_plane = src[0];
        uint8_t * gb_plane = src[1];
        uint8_t * gr_plane = src[2];
        uint8_t * r_plane = src[3];
        if (format ==  BAYER_GRBG) {
            drow_offset1 = dstride;
            drow_offset2 = 0;
            kernel_stride = -sstride;
            gr_plane = src[0];
            r_plane = src[1];
            b_plane = src[2];
            gb_plane = src[3];
        }

        for (i = 0; i < width/2; i += 16) {
            uint8_t * dcol = dst + i*8;

            for (j = 0; j < height/2; j++) {
                INTERPOLATE_GB_ROW (kernel_stride, -1);

                uint8_t * drow = dcol + j*2*dstride + drow_offset1;
                _mm_store_si128 ((__m128i *)drow,
                        _mm_unpacklo_epi32 (bgrar1, bgral1));
                _mm_store_si128 ((__m128i *)(drow+16),
                        _mm_unpackhi_epi32 (bgrar1, bgral1));
                _mm_store_si128 ((__m128i *)(drow+32),
                        _mm_unpacklo_epi32 (bgrar2, bgral2));
                _mm_store_si128 ((__m128i *)(drow+48),
                        _mm_unpackhi_epi32 (bgrar2, bgral2));
                _mm_store_si128 ((__m128i *)(drow+64),
                        _mm_unpacklo_epi32 (bgrar3, bgral3));
                _mm_store_si128 ((__m128i *)(drow+80),
                        _mm_unpackhi_epi32 (bgrar3, bgral3));
                _mm_store_si128 ((__m128i *)(drow+96),
                        _mm_unpacklo_epi32 (bgrar4, bgral4));
                _mm_store_si128 ((__m128i *)(drow+112),
                        _mm_unpackhi_epi32 (bgrar4, bgral4));

                INTERPOLATE_RG_ROW (kernel_stride, -1);

                drow = dcol + j*2*dstride + drow_offset2;
                _mm_store_si128 ((__m128i *)drow,
                        _mm_unpacklo_epi32 (bgrar1, bgral1));
                _mm_store_si128 ((__m128i *)(drow+16),
                        _mm_unpackhi_epi32 (bgrar1, bgral1));
                _mm_store_si128 ((__m128i *)(drow+32),
                        _mm_unpacklo_epi32 (bgrar2, bgral2));
                _mm_store_si128 ((__m128i *)(drow+48),
                        _mm_unpackhi_epi32 (bgrar2, bgral2));
                _mm_store_si128 ((__m128i *)(drow+64),
                        _mm_unpacklo_epi32 (bgrar3, bgral3));
                _mm_store_si128 ((__m128i *)(drow+80),
                        _mm_unpackhi_epi32 (bgrar3, bgral3));
                _mm_store_si128 ((__m128i *)(drow+96),
                        _mm_unpacklo_epi32 (bgrar4, bgral4));
                _mm_store_si128 ((__m128i *)(drow+112),
                        _mm_unpackhi_epi32 (bgrar4, bgral4));

            }
            gb_plane += 16;
            b_plane += 16;
            r_plane += 16;
            gr_plane += 16;
        }
    }
    return 0;
#endif
}

template Image<PixRGB<byte> >  debayerSSE3(const Image<byte>& src, BayerFormat format);
template Image<PixRGB<uint16> >  debayerSSE3(const Image<uint16>& src, BayerFormat format);
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

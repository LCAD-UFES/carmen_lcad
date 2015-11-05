/*!@file Util/mmx-sse.H Optimized implementations of low-level functions
  for MMX/SSE */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: Nitin Dhavale <dhavale@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/mmx-sse.H $
// $Id: mmx-sse.H 10118 2008-08-18 23:51:38Z ilab24 $
//


#ifndef __SSE_MMX_H__
#define __SSE_MMX_H__
#include <string.h>
typedef int int32;
typedef unsigned char byte;
typedef float float32;

//! In all the functions

#ifdef INVT_USE_MMX
#endif

#ifdef INVT_USE_SSE
/*! All the sse_absDiff() functions subtract image 'b' from 'a' and put the absolute
  value of the result into 'diff'.
*/
void sse_absDiff(const double *a, const double *b, double *diff, const int32 sz);

//! The pixel values composing the image 'img' are summed up to give 'sum'
void sse_sum(const double *img, double *sum, const int32 sz);

/*! sse_clampedDiff() routines take the clamped difference of 2 images
  'a' and 'b'
*/
void sse_clampedDiff(const byte *a, const byte *b, byte *result, const int32 sz);
void sse_clampedDiff(const float32 *a, const float32 *b, float32 *result, const int sz);
void sse_clampedDiff(const int32 *a, const int32 *b, int32 *c, const int32 sz);

/*! sse_binaryReverse() takes the distance of each pixel in 'src' from 'val' and the
   resulting image is stored in 'result'
*/
void sse_binaryReverse(const byte *src, byte *result, const byte val, const int sz);
void sse_binaryReverse(const float *src, float *result, const float val, const int sz);
void sse_binaryReverse(const int32 *src, int32 *result, const int32 val, const int sz);

//! sse_cvt_byte_to_int() converts an image<byte> into a image<int>
void sse_cvt_byte_to_int(const byte *a, int32 *b, const int32 sz);


/*! The filter functions take 4 params, the 'src' one indicating the image
  to be filtered, dest is a pre-allocated array that holds the results
  after processing, 'h' & 'w' are respectively the height and width of the
  source image
*/
void sse_lowPass3x(const float *src, float *dest, int w, int h);
void sse_lowPass3y(const float *src, float *dest, const int w, const int h);
void sse_lowPass5x(const float *src, float *dest, const int h, const int w);
void sse_lowPass5y(const float *src, float *dest, const int h, const int w);

/*! The yuv2rgb conversion functions take 3 params, the 'src' indicates
  the yuv image, 'dest' is a byte array that will hold the rgb values
  after conversion. 'nbpix2' is the number of bytes in the yuv image.
*/
void sse_yuv411_to_rgb_mode_640x480(const byte *src, byte *dest,
                                    const int nbpix2);
#endif

#ifdef INVT_USE_MMXSSE2
void sse2_sum(const float *a, double *sum, const int32 sz);
void sse2_sum(const int32 *a, double *sum, const int32 sz);
void sse2_sum(const byte *a, double *sum, const int32 sz);
void sse2_cvt_byte_to_float(const byte *a, float32 *b, const int32 sz);
void sse2_cvt_byte_to_double(const byte *a, double *b, int32 sz);
void sse2_cvt_int_to_float(const int32 *a, float *b, const int32 sz);
void sse2_cvt_int_to_double(const int32 *a, double *b, const int32 sz);
void sse2_cvt_float_to_int(const float *a, int *b, const int32 sz);
void sse2_cvt_float_to_double(const float *a, double *b, const int32 sz);
void sse2_absDiff(const float *a, const float *b, float *diff, const int32 sz);
void sse2_absDiff(const int32 *a, const int32 *b, int32 *diff, const int32 sz);
void sse2_absDiff(const byte *a, const byte *b, byte *diff, const int32 sz);
#endif

void sse_lowPass9x(const float *, float *, int, int );

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

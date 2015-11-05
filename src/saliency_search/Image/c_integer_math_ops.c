/*!@file Image/c_integer_math_ops.c Fixed-point integer math versions of some of our floating-point Image functions */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/c_integer_math_ops.c $
// $Id: c_integer_math_ops.c 7848 2007-02-07 17:46:06Z rjpeters $
//

#ifndef IMAGE_C_INTEGER_MATH_OPS_C_DEFINED
#define IMAGE_C_INTEGER_MATH_OPS_C_DEFINED

#include "Image/c_integer_math_ops.h"

#include "Util/Assert.H"

// ######################################################################
void c_intg_low_pass_5_x_dec_x_manybits(const int* src,
                                         const int w, const int h,
                                         int* dst,
                                         const int w2)
{
  if (w == 2) //////////////////////////////////////////////////
    for (int j = 0; j < h; ++j)
      {
        // leftmost point  [ (6^) 4 ] / 10
        *dst++ = (src[0] / 10) * 6 + (src[1] / 10) * 4;

        src += 2;  // src back to same position as dst
      }
  else if (w == 3) //////////////////////////////////////////////////
    for (int j = 0; j < h; ++j)
      {
        // need the left most point in any case
        // leftmost point  [ (6^) 4 1 ] / 11
        *dst++ =
          (src[0] / 11) * 6 +
          (src[1] / 11) * 4 +
          (src[2] / 11) * 1;

        src += 3;  // src back to same position as dst
      }
  else  ////////////////////////////// general case for width() >= 4
        // *** unfolded version (all particular cases treated) for
        // max speed.
        // notations: in () is the position of dest ptr, and ^ is src ptr
        // ########## horizontal pass
    for (int j = 0; j < h; ++j)
      {
        int i1 = 0, i2 = 0;
        const int* src2 = src;

        // leftmost point  [ (6^) 4 1 ] / 11
        *dst++ =
          (src2[0] / 11) * 6 +
          (src2[1] / 11) * 4 +
          (src2[2] / 11) * 1;
        ++i2;
        i1 += 2;

        // skip second point

        // rest of the line except last 2 points  [ 1^ 4 (6) 4 1 ] / 16.0
        while ((i1 < (w-2)) && (i2 < w2))
          {
            *dst++ =
              ((src2[0] >> 4) + (src2[4] >> 4)) * 1 +
              ((src2[1] >> 4) + (src2[3] >> 4)) * 4 +
              (src2[2] >> 4) * 6;
            i1 += 2; src2 += 2;
            ++i2;
          }

        // need special case for second to last point?
        if ((i2 < w2) && (i1 == (w-2)))
          {
            src2 = src + w - 4;
            // before last point [ 1^ 4 (6) 4 ] / 15
            *dst++ =
              (src2[0] / 15) * 1 +
              (src2[1] / 15 + src2[3] / 15) * 4 +
              (src2[2] / 15) * 6;
            i1 += 2;
            ++i2;
          }

        // need special case for last point?
        if ((i2 < w2) && (i1 == (w-1)))
          {
            src2 = src + w - 3;
            // last point [ 1^ 4 (6) ] / 11
            *dst++ =
              (src2[0] / 11) * 1 +
              (src2[1] / 11) * 4 +
              (src2[2] / 11) * 6;
            ++i2;
          }
        src += w;
      }
}

// ######################################################################
void c_intg_low_pass_5_y_dec_y_manybits(const int* src,
                                         const int w, const int h,
                                         int* dst,
                                         const int h2)
{
  const int* const src_end = src + w*h;

  // ########## vertical pass  (even though we scan horiz for speedup)
  const int w2 = w * 2, w3 = w * 3, w4 = w * 4; // speedup

  if (h == 2) //////////////////////////////////////////////////
    {
      // topmost points  ( [ (6^) 4 ] / 10 )^T
      for (int i = 0; i < w; ++i)
        {
          *dst++ =
            (src[0] / 10) * 6 +
            (src[w] / 10) * 4;
          src++;
        }
      src -= w;  // go back to top-left
    }
  else if (h == 3) //////////////////////////////////////////////////
    {
      // topmost points  ( [ (6^) 4 1 ] / 11 )^T
      for (int i = 0; i < w; ++i)
        {
          *dst++ =
            (src[ 0] / 11) * 6 +
            (src[ w] / 11) * 4 +
            (src[w2] / 11) * 1;
          src++;
        }
      src -= w;  // go back to top-left
    }
  else  ///////////////////////////////// general case for height >= 4
    {
      int i1 = 0, i2 = 0;

      // topmost points  ( [ (6^) 4 1 ] / 11 )^T
      for (int i = 0; i < w; ++i)
        {
          *dst++ =
            (src[ 0] / 11) * 6 +
            (src[ w] / 11) * 4 +
            (src[w2] / 11) * 1;
          src++;
        }
      src -= w;  // go back to top-left
      ++i2;
      i1 += 2;

      // second point skipped

      // rest of the column except last 2 points ( [ 1^ 4 (6) 4 1 ] / 16 )T
      while ((i1 < (h-2)) && (i2 < h2))
        {
          for (int i = 0; i < w; ++i)
            {
              *dst++ =
                ((src[ 0] >> 4) + (src[w4] >> 4)) * 1 +
                ((src[ w] >> 4) + (src[w3] >> 4)) * 4 +
                (src[w2] >> 4) * 6;
              src++;
            }
          src += w;
          i1 += 2;
          ++ i2;
        }

      // need special case for second to last point?
      if ((i2 < h2) && (i1 == (h-2)))
        {
          src = src_end - w4;
          // before last points ( [ 1^ 4 (6) 4 ] / 15 )T
          for (int i = 0; i < w; ++i)
            {
              *dst++ =
                (src[ 0] / 15) * 1 +
                (src[ w] / 15 + src[w3] / 15) * 4 +
                (src[w2] / 15) * 6;
              src++;
            }
          i1 += 2;
          ++i2;
        }

      // need special case for last point?
      if ((i2 < h2) && (i1 == (h-1)))
        {
          src = src_end - w3;
          // last points ( [ 1^ 4 (6) ] / 11 )T
          for (int i = 0; i < w; ++i)
            {
              *dst++ =
                (src[ 0] / 11) * 1 +
                (src[ w] / 11) * 4 +
                (src[w2] / 11) * 6;
              src++;
            }
        }
    }
}

// ######################################################################
void c_intg_low_pass_5_x_dec_x_fewbits(const int* src,
                                         const int w, const int h,
                                         int* dst,
                                         const int w2)
{
  if (w == 2) //////////////////////////////////////////////////
    for (int j = 0; j < h; ++j)
      {
        // leftmost point  [ (6^) 4 ] / 10
        *dst++ =
          (src[0] * 6 +
           src[1] * 4
           ) / 10;

        src += 2;  // src back to same position as dst
      }
  else if (w == 3) //////////////////////////////////////////////////
    for (int j = 0; j < h; ++j)
      {
        // need the left most point in any case
        // leftmost point  [ (6^) 4 1 ] / 11
        *dst++ =
          (src[0] * 6 +
           src[1] * 4 +
           src[2] * 1
           ) / 11;

        src += 3;  // src back to same position as dst
      }
  else  ////////////////////////////// general case for width() >= 4
        // *** unfolded version (all particular cases treated) for
        // max speed.
        // notations: in () is the position of dest ptr, and ^ is src ptr
        // ########## horizontal pass
    for (int j = 0; j < h; ++j)
      {
        int i1 = 0, i2 = 0;
        const int* src2 = src;

        // leftmost point  [ (6^) 4 1 ] / 11
        *dst++ =
          (src2[0] * 6 +
           src2[1] * 4 +
           src2[2] * 1
           ) / 11;
        ++i2;
        i1 += 2;

        // skip second point

        // rest of the line except last 2 points  [ 1^ 4 (6) 4 1 ] / 16.0
        while ((i1 < (w-2)) && (i2 < w2))
          {
            *dst++ =
              ((src2[0] + src2[4]) * 1 +
               (src2[1] + src2[3]) * 4 +
               src2[2] * 6
               ) >> 4;
            i1 += 2; src2 += 2;
            ++i2;
          }

        // need special case for second to last point?
        if ((i2 < w2) && (i1 == (w-2)))
          {
            src2 = src + w - 4;
            // before last point [ 1^ 4 (6) 4 ] / 15
            *dst++ =
              (src2[0] * 1 +
               (src2[1] + src2[3]) * 4 +
               src2[2] * 6
               ) / 15;
            i1 += 2;
            ++i2;
          }

        // need special case for last point?
        if ((i2 < w2) && (i1 == (w-1)))
          {
            src2 = src + w - 3;
            // last point [ 1^ 4 (6) ] / 11
            *dst++ =
              (src2[0] * 1 +
               src2[1] * 4 +
               src2[2] * 6
               ) / 11;
            ++i2;
          }
        src += w;
      }
}

// ######################################################################
void c_intg_low_pass_5_y_dec_y_fewbits(const int* src,
                                         const int w, const int h,
                                         int* dst,
                                         const int h2)
{
  const int* const src_end = src + w*h;

  // ########## vertical pass  (even though we scan horiz for speedup)
  const int w2 = w * 2, w3 = w * 3, w4 = w * 4; // speedup

  if (h == 2) //////////////////////////////////////////////////
    {
      // topmost points  ( [ (6^) 4 ] / 10 )^T
      for (int i = 0; i < w; ++i)
        {
          *dst++ =
            (src[0] * 6 +
             src[w] * 4
             ) / 10;
          src++;
        }
      src -= w;  // go back to top-left
    }
  else if (h == 3) //////////////////////////////////////////////////
    {
      // topmost points  ( [ (6^) 4 1 ] / 11 )^T
      for (int i = 0; i < w; ++i)
        {
          *dst++ =
            (src[ 0] * 6 +
             src[ w] * 4 +
             src[w2] * 1
             ) / 11;
          src++;
        }
      src -= w;  // go back to top-left
    }
  else  ///////////////////////////////// general case for height >= 4
    {
      int i1 = 0, i2 = 0;

      // topmost points  ( [ (6^) 4 1 ] / 11 )^T
      for (int i = 0; i < w; ++i)
        {
          *dst++ =
            (src[ 0] * 6 +
             src[ w] * 4 +
             src[w2] * 1
             ) / 11;
          src++;
        }
      src -= w;  // go back to top-left
      ++i2;
      i1 += 2;

      // second point skipped

      // rest of the column except last 2 points ( [ 1^ 4 (6) 4 1 ] / 16 )T
      while ((i1 < (h-2)) && (i2 < h2))
        {
          for (int i = 0; i < w; ++i)
            {
              *dst++ =
                ((src[ 0] + src[w4]) * 1 +
                 (src[ w] + src[w3]) * 4 +
                 src[w2] * 6
                 ) >> 4;
              src++;
            }
          src += w;
          i1 += 2;
          ++ i2;
        }

      // need special case for second to last point?
      if ((i2 < h2) && (i1 == (h-2)))
        {
          src = src_end - w4;
          // before last points ( [ 1^ 4 (6) 4 ] / 15 )T
          for (int i = 0; i < w; ++i)
            {
              *dst++ =
                (src[ 0] * 1 +
                 (src[ w] + src[w3]) * 4 +
                 src[w2] * 6
                 ) / 15;
              src++;
            }
          i1 += 2;
          ++i2;
        }

      // need special case for last point?
      if ((i2 < h2) && (i1 == (h-1)))
        {
          src = src_end - w3;
          // last points ( [ 1^ 4 (6) ] / 11 )T
          for (int i = 0; i < w; ++i)
            {
              *dst++ =
                (src[ 0] * 1 +
                 src[ w] * 4 +
                 src[w2] * 6
                 ) / 11;
              src++;
            }
        }
    }
}

#define LP5_APPROX_LEVEL 0

#if LP5_APPROX_LEVEL == 0

#  define LP5_APPROX_2_3 2
#  define LP5_APPROX_1_3 1
#  define LP5_DIV_3              / 3

#elif LP5_APPROX_LEVEL == 1

#  define LP5_APPROX_2_3 1
#  define LP5_APPROX_1_3 1
#  define LP5_DIV_3              / 2

#else
#error bogus LP5_APPROX_LEVEL (must be 0 or 1)
#endif

// ######################################################################
void c_intg_low_pass_5_x_dec_x_fewbits_optim(const int* src,
                                         const int w, const int h,
                                         int* dst,
                                         const int w2)
{
  ASSERT(w2 == w/2);

  if (w == 2) //////////////////////////////////////////////////
    for (int j = 0; j < h; ++j)
      {
        // leftmost point  [ (6^) 4 ] / 10
        *dst++ =
          (src[0] * 3 +
           src[1] * 2
           ) / 5;

        src += 2;  // src back to same position as dst
      }
  else if (w == 3) //////////////////////////////////////////////////
    for (int j = 0; j < h; ++j)
      {
        // need the left most point in any case
        // leftmost point  [ (6^) 4 1 ] / 11
        *dst++ =
          (src[0] * 6 +
           src[1] * 4 +
           src[2] * 1
           ) / 11;

        src += 3;  // src back to same position as dst
      }
  else  ////////////////////////////// general case for width() >= 4
        // *** unfolded version (all particular cases treated) for
        // max speed.
        // notations: in () is the position of dest ptr, and ^ is src ptr
        // ########## horizontal pass
    for (int j = 0; j < h; ++j)
      {
        const int* src2 = src;

        // leftmost point  [ (8^) 4 ] / 12
        *dst++ =
          (src2[0] * LP5_APPROX_2_3 +
           src2[1] * LP5_APPROX_1_3
           ) LP5_DIV_3;

        // skip second point

        // rest of the line except last 2 points  [ .^ 4 (8) 4 ] / 16
        for (int i = 0; i < w-3; i += 2)
          {
            *dst++ =
              ((src2[1] + src2[3]) +
               src2[2] * 2
               ) >> 2;
            src2 += 2;
          }

        src += w;
      }
}

// ######################################################################
void c_intg_low_pass_5_y_dec_y_fewbits_optim(const int* src,
                                         const int w, const int h,
                                         int* dst,
                                         const int h2)
{
  ASSERT(h2 == h/2);

  // ########## vertical pass  (even though we scan horiz for speedup)
  const int w2 = w * 2, w3 = w * 3; // speedup

  if (h == 2) //////////////////////////////////////////////////
    {
      // topmost points  ( [ (6^) 4 ] / 10 )^T
      for (int i = 0; i < w; ++i)
        {
          *dst++ =
            (src[0] * 3 +
             src[w] * 2
             ) / 5;
          src++;
        }
      src -= w;  // go back to top-left
    }
  else if (h == 3) //////////////////////////////////////////////////
    {
      // topmost points  ( [ (6^) 4 1 ] / 11 )^T
      for (int i = 0; i < w; ++i)
        {
          *dst++ =
            (src[ 0] * 6 +
             src[ w] * 4 +
             src[w2] * 1
             ) / 11;
          src++;
        }
      src -= w;  // go back to top-left
    }
  else  ///////////////////////////////// general case for height >= 4
    {
      // topmost points  ( [ (8^) 4 ] / 12 )^T
      for (int k = 0; k < w; ++k)
        {
          *dst++ =
            (src[ 0] * LP5_APPROX_2_3 +
             src[ w] * LP5_APPROX_1_3
             ) LP5_DIV_3;
          src++;
        }
      src -= w;  // go back to top-left

      // second point skipped

      // rest of the column except last 2 points ( [ .^ 4 (8) 4 ] / 16 )T
      for (int i = 0; i < h-3; i += 2)
        {
          for (int k = 0; k < w; ++k)
            {
              *dst++ =
                ((src[ w] + src[w3]) +
                 src[w2] * 2
                 ) >> 2;
              src++;
            }
          src += w;
        }
    }
}

// ######################################################################
void c_intg_low_pass_9_x_manybits(const int* src,
                                   const int w, const int h,
                                   int* dst)
{
  ASSERT(w >= 9);

  // boundary conditions: truncated filter
  for (int j = 0; j < h; ++j)
    {
      // leftmost points
      *dst++ =
        (src[0] / 163) * 70 +
        (src[1] / 163) * 56 +
        (src[2] / 163) * 28 +
        (src[3] / 163) *  8 +
        (src[4] / 163) *  1;
      *dst++ =
        (src[0] / 219 + src[2] / 219) * 56 +
        (src[1] / 219) * 70 +
        (src[3] / 219) * 28 +
        (src[4] / 219) *  8 +
        (src[5] / 219) *  1;
      *dst++ =
        (src[0] / 247 + src[4] / 247) * 28 +
        (src[1] / 247 + src[3] / 247) * 56 +
        (src[2] / 247) * 70 +
        (src[5] / 247) *  8 +
        (src[6] / 247) *  1;
      *dst++ =
        (src[0] / 255 + src[6] / 255) *  8 +
        (src[1] / 255 + src[5] / 255) * 28 +
        (src[2] / 255 + src[4] / 255) * 56 +
        (src[3] / 255) * 70 +
        (src[7] / 255) *  1;

      // far from the borders
      for (int i = 0; i < w - 8; ++i)
        {
          *dst++ =
            ((src[0] >> 8) + (src[8] >> 8)) *  1 +
            ((src[1] >> 8) + (src[7] >> 8)) *  8 +
            ((src[2] >> 8) + (src[6] >> 8)) * 28 +
            ((src[3] >> 8) + (src[5] >> 8)) * 56 +
            (src[4] >> 8) * 70;
          ++src;
        }

      // rightmost points
      *dst++ =
        (src[0] / 255) *  1 +
        (src[1] / 255 + src[7] / 255) *  8 +
        (src[2] / 255 + src[6] / 255) * 28 +
        (src[3] / 255 + src[5] / 255) * 56 +
        (src[4] / 255) * 70;
      ++src;
      *dst++ =
        (src[0] / 247) *  1 +
        (src[1] / 247) *  8 +
        (src[2] / 247 + src[6] / 247) * 28 +
        (src[3] / 247 + src[5] / 247) * 56 +
        (src[4] / 247) * 70;
      ++src;
      *dst++ =
        (src[0] / 219) *  1 +
        (src[1] / 219) *  8 +
        (src[2] / 219) * 28 +
        (src[3] / 219 + src[5] / 219) * 56 +
        (src[4] / 219) * 70;
      ++src;
      *dst++ =
        (src[0] / 163) *  1 +
        (src[1] / 163) *  8 +
        (src[2] / 163) * 28 +
        (src[3] / 163) * 56 +
        (src[4] / 163) * 70;
      src += 5;  // src back to same as dst (start of next line)
    }
}

// ######################################################################
void c_intg_low_pass_9_y_manybits(const int* src,
                                   const int w, const int h,
                                   int* dst)
{
  ASSERT(h >= 9);

  // *** vertical pass ***
  const int w2 = w + w, w3 = w2 + w, w4 = w3 + w, w5 = w4 + w, w6 = w5 + w,
    w7 = w6 + w,  w8 = w7 + w;  // index computation speedup
  for (int i = 0; i < w; ++i)
    {
      *dst++ =
        (src[ 0] / 163) * 70 +
        (src[ w] / 163) * 56 +
        (src[w2] / 163) * 28 +
        (src[w3] / 163) *  8 +
        (src[w4] / 163) *  1;
      ++src;
    }
  src -= w; // back to top-left
  for (int i = 0; i < w; ++i)
    {
      *dst++ =
        (src[ 0] / 219 + src[w2] / 219) * 56 +
        (src[ w] / 219) * 70 +
        (src[w3] / 219) * 28 +
        (src[w4] / 219) *  8 +
        (src[w5] / 219) *  1;
      ++src;
    }
  src -= w; // back to top-left
  for (int i = 0; i < w; ++i)
    {
      *dst++ =
        (src[ 0] / 247 + src[w4] / 247) * 28 +
        (src[ w] / 247 + src[w3] / 247) * 56 +
        (src[w2] / 247) * 70 +
        (src[w5] / 247) *  8 +
        (src[w6] / 247) *  1;
      ++src;
    }
  src -= w; // back to top-left
  for (int i = 0; i < w; ++i)
    {
      *dst++ =
        (src[ 0] / 255 + src[w6] / 255) *  8 +
        (src[ w] / 255 + src[w5] / 255) * 28 +
        (src[w2] / 255 + src[w4] / 255) * 56 +
        (src[w3] / 255) * 70 +
        (src[w7] / 255) *  1;
      ++src;
    }
  src -= w;   // back to top-left
  for (int j = 0; j < h - 8; j ++)
    for (int i = 0; i < w; ++i)
      {
        *dst++ =
          ((src[ 0] >> 8) + (src[w8] >> 8)) *  1 +
          ((src[ w] >> 8) + (src[w7] >> 8)) *  8 +
          ((src[w2] >> 8) + (src[w6] >> 8)) * 28 +
          ((src[w3] >> 8) + (src[w5] >> 8)) * 56 +
          (src[w4] >> 8)  * 70;
        ++src;
      }
  for (int i = 0; i < w; ++i)
    {
      *dst++ =
        (src[ 0] / 255) *  1 +
        (src[ w] / 255 + src[w7] / 255) *  8 +
        (src[w2] / 255 + src[w6] / 255) * 28 +
        (src[w3] / 255 + src[w5] / 255) * 56 +
        (src[w4] / 255) * 70;
      ++src;
    }
  for (int i = 0; i < w; ++i)
    {
      *dst++ =
        (src[ 0] / 247) *  1 +
        (src[ w] / 247) *  8 +
        (src[w2] / 247 + src[w6] / 247) * 28 +
        (src[w3] / 247 + src[w5] / 247) * 56 +
        (src[w4] / 247) * 70;
      ++src;
    }
  for (int i = 0; i < w; ++i)
    {
      *dst++ =
        (src[ 0] / 219) *  1 +
        (src[ w] / 219) *  8 +
        (src[w2] / 219) * 28 +
        (src[w3] / 219 + src[w5] / 219) * 56 +
        (src[w4] / 219) * 70;
      ++src;
    }
  for (int i = 0; i < w; ++i)
    {
      *dst++ =
        (src[ 0] / 163) *  1 +
        (src[ w] / 163) *  8 +
        (src[w2] / 163) * 28 +
        (src[w3] / 163) * 56 +
        (src[w4] / 163) * 70;
      ++src;
    }
}

// ######################################################################
void c_intg_low_pass_9_x_fewbits(const int* src,
                                   const int w, const int h,
                                   int* dst)
{
  ASSERT(w >= 9);

  // boundary conditions: truncated filter
  for (int j = 0; j < h; ++j)
    {
      // leftmost points
      *dst++ =
        (src[0] * 70 +
         src[1] * 56 +
         src[2] * 28 +
         src[3] *  8 +
         src[4] *  1
         ) / 163;
      *dst++ =
        ((src[0] + src[2]) * 56 +
         src[1] * 70 +
         src[3] * 28 +
         src[4] *  8 +
         src[5] *  1
         ) / 219;
      *dst++ =
        ((src[0] + src[4]) * 28 +
         (src[1] + src[3]) * 56 +
         src[2] * 70 +
         src[5] *  8 +
         src[6] *  1
         ) / 247;
      *dst++ =
        ((src[0] + src[6]) *  8 +
         (src[1] + src[5]) * 28 +
         (src[2] + src[4]) * 56 +
         src[3] * 70 +
         src[7] *  1
         ) / 255;

      // far from the borders
      for (int i = 0; i < w - 8; ++i)
        {
          *dst++ =
            ((src[0] + src[8]) *  1 +
             (src[1] + src[7]) *  8 +
             (src[2] + src[6]) * 28 +
             (src[3] + src[5]) * 56 +
             src[4] * 70
             ) >> 8;
          ++src;
        }

      // rightmost points
      *dst++ =
        (src[0] *  1 +
         (src[1] + src[7]) *  8 +
         (src[2] + src[6]) * 28 +
         (src[3] + src[5]) * 56 +
         src[4] * 70
         ) / 255;
      ++src;
      *dst++ =
        (src[0] *  1 +
         src[1] *  8 +
         (src[2] + src[6]) * 28 +
         (src[3] + src[5]) * 56 +
         src[4] * 70
         ) / 247;
      ++src;
      *dst++ =
        (src[0] *  1 +
         src[1] *  8 +
         src[2] * 28 +
         (src[3] + src[5]) * 56 +
         src[4] * 70
         ) / 219;
      ++src;
      *dst++ =
        (src[0] *  1 +
         src[1] *  8 +
         src[2] * 28 +
         src[3] * 56 +
         src[4] * 70
         ) / 163;
      src += 5;  // src back to same as dst (start of next line)
    }
}

// ######################################################################
void c_intg_low_pass_9_y_fewbits(const int* src,
                                   const int w, const int h,
                                   int* dst)
{
  ASSERT(h >= 9);

  // *** vertical pass ***
  const int w2 = w + w, w3 = w2 + w, w4 = w3 + w, w5 = w4 + w, w6 = w5 + w,
    w7 = w6 + w,  w8 = w7 + w;  // index computation speedup
  for (int i = 0; i < w; ++i)
    {
      *dst++ =
        (src[ 0] * 70 +
         src[ w] * 56 +
         src[w2] * 28 +
         src[w3] *  8 +
         src[w4] *  1
         ) / 163;
      ++src;
    }
  src -= w; // back to top-left
  for (int i = 0; i < w; ++i)
    {
      *dst++ =
        ((src[ 0] + src[w2]) * 56 +
         src[ w] * 70 +
         src[w3] * 28 +
         src[w4] *  8 +
         src[w5] *  1
         ) / 219;
      ++src;
    }
  src -= w; // back to top-left
  for (int i = 0; i < w; ++i)
    {
      *dst++ =
        ((src[ 0] + src[w4]) * 28 +
         (src[ w] + src[w3]) * 56 +
         src[w2] * 70 +
         src[w5] *  8 +
         src[w6] *  1
         ) / 247;
      ++src;
    }
  src -= w; // back to top-left
  for (int i = 0; i < w; ++i)
    {
      *dst++ =
        ((src[ 0] + src[w6]) *  8 +
         (src[ w] + src[w5]) * 28 +
         (src[w2] + src[w4]) * 56 +
         src[w3] * 70 +
         src[w7] *  1
         ) / 255;
      ++src;
    }
  src -= w;   // back to top-left
  for (int j = 0; j < h - 8; j ++)
    for (int i = 0; i < w; ++i)
      {
        *dst++ =
          ((src[ 0] + src[w8]) *  1 +
           (src[ w] + src[w7]) *  8 +
           (src[w2] + src[w6]) * 28 +
           (src[w3] + src[w5]) * 56 +
           src[w4]  * 70
           ) >> 8;
        ++src;
      }
  for (int i = 0; i < w; ++i)
    {
      *dst++ =
        (src[ 0] *  1 +
         (src[ w] + src[w7]) *  8 +
         (src[w2] + src[w6]) * 28 +
         (src[w3] + src[w5]) * 56 +
         src[w4] * 70
         ) / 255;
      ++src;
    }
  for (int i = 0; i < w; ++i)
    {
      *dst++ =
        (src[ 0] *  1 +
         src[ w] *  8 +
         (src[w2] + src[w6]) * 28 +
         (src[w3] + src[w5]) * 56 +
         src[w4] * 70
         ) / 247;
      ++src;
    }
  for (int i = 0; i < w; ++i)
    {
      *dst++ =
        (src[ 0] *  1 +
         src[ w] *  8 +
         src[w2] * 28 +
         (src[w3] + src[w5]) * 56 +
         src[w4] * 70
         ) / 219;
      ++src;
    }
  for (int i = 0; i < w; ++i)
    {
      *dst++ =
        (src[ 0] *  1 +
         src[ w] *  8 +
         src[w2] * 28 +
         src[w3] * 56 +
         src[w4] * 70
         ) / 163;
      ++src;
    }
}

#define LP9_APPROX_LEVEL 0

#if LP9_APPROX_LEVEL == 0

#  define LP9_APPROX_72_164  72
#  define LP9_APPROX_56_164  56
#  define LP9_APPROX_28_164  28
#  define LP9_APPROX_8_164   8
#  define LP9_DIV_164            / 164

#  define LP9_APPROX_72_220  72
#  define LP9_APPROX_56_220  56
#  define LP9_APPROX_28_220  28
#  define LP9_APPROX_8_220   8
#  define LP9_DIV_220            / 220

#  define LP9_APPROX_72_248  72
#  define LP9_APPROX_56_248  56
#  define LP9_APPROX_28_248  28
#  define LP9_APPROX_8_248   8
#  define LP9_DIV_248            / 248

#elif LP9_APPROX_LEVEL == 1

#  define LP9_APPROX_72_164  28
#  define LP9_APPROX_56_164  22
#  define LP9_APPROX_28_164  11
#  define LP9_APPROX_8_164    3
#  define LP9_DIV_164             >> 6

#  define LP9_APPROX_72_220  22
#  define LP9_APPROX_56_220  16
#  define LP9_APPROX_28_220   8
#  define LP9_APPROX_8_220    2
#  define LP9_DIV_220             >> 6

#  define LP9_APPROX_72_248  20
#  define LP9_APPROX_56_248  14
#  define LP9_APPROX_28_248   7
#  define LP9_APPROX_8_248    2
#  define LP9_DIV_248             >> 6

#elif LP9_APPROX_LEVEL == 2

#  define LP9_APPROX_72_164  8
#  define LP9_APPROX_56_164  (4+1)
#  define LP9_APPROX_28_164  2
#  define LP9_APPROX_8_164   1
#  define LP9_DIV_164             >> 4

#  define LP9_APPROX_72_220  (4+1)
#  define LP9_APPROX_56_220  4
#  define LP9_APPROX_28_220  2
#  define LP9_APPROX_8_220   1
#  define LP9_DIV_220             >> 4

#  define LP9_APPROX_72_248  (4+1)
#  define LP9_APPROX_56_248  3
#  define LP9_APPROX_28_248  2
#  define LP9_APPROX_8_248   1
#  define LP9_DIV_248             >> 4

#elif LP9_APPROX_LEVEL == 3

#  define LP9_APPROX_72_164  0
#  define LP9_APPROX_56_164  0
#  define LP9_APPROX_28_164  0
#  define LP9_APPROX_8_164   0
#  define LP9_DIV_164

#  define LP9_APPROX_72_220  0
#  define LP9_APPROX_56_220  0
#  define LP9_APPROX_28_220  0
#  define LP9_APPROX_8_220   0
#  define LP9_DIV_220

#  define LP9_APPROX_72_248  0
#  define LP9_APPROX_56_248  0
#  define LP9_APPROX_28_248  0
#  define LP9_APPROX_8_248   0
#  define LP9_DIV_248

#  define LP9_APPROX_SHIFTR  0

#else
#error bogus LP9_APPROX_LEVEL (must be 0, 1, 2, or 3)
#endif

// ######################################################################
void c_intg_low_pass_9_x_fewbits_optim(const int* src,
                                   const int w, const int h,
                                   int* dst)
{
  ASSERT(w >= 9);

  // boundary conditions: truncated filter
  for (int j = 0; j < h; ++j)
    {
      // leftmost points
      *dst++ =                  // [ (72^) 56 28 8 ]
        (src[0] * LP9_APPROX_72_164 +
         src[1] * LP9_APPROX_56_164 +
         src[2] * LP9_APPROX_28_164 +
         src[3] * LP9_APPROX_8_164
         ) LP9_DIV_164;
      *dst++ =                  // [ 56^ (72) 56 28 8 ]
        ((src[0] + src[2]) * LP9_APPROX_56_220 +
         src[1] * LP9_APPROX_72_220 +
         src[3] * LP9_APPROX_28_220 +
         src[4] * LP9_APPROX_8_220
         ) LP9_DIV_220;
      *dst++ =                  // [ 28^ 56 (72) 56 28 8 ]
        ((src[0] + src[4]) * LP9_APPROX_28_248 +
         (src[1] + src[3]) * LP9_APPROX_56_248 +
         src[2] * LP9_APPROX_72_248 +
         src[5] * LP9_APPROX_8_248
         ) LP9_DIV_248;

      // far from the borders
      for (int i = 0; i < w - 6; ++i)
        {
          *dst++ =              // [ 8^ 28 56 (72) 56 28 8 ]
            ((src[0] + src[6]) *  8 +
             (src[1] + src[5]) * 28 +
             (src[2] + src[4]) * 56 +
             src[3] * 72
             ) >> 8;
          ++src;
        }

      // rightmost points
      *dst++ =                  // [ 8^ 28 56 (72) 56 28 ]
        (src[0] *  LP9_APPROX_8_248 +
         (src[1] + src[5]) * LP9_APPROX_28_248 +
         (src[2] + src[4]) * LP9_APPROX_56_248 +
         src[3] * LP9_APPROX_72_248
         ) LP9_DIV_248;
      ++src;
      *dst++ =                  // [ 8^ 28 56 (72) 56 ]
        (src[0] * LP9_APPROX_8_220 +
         src[1] * LP9_APPROX_28_220 +
         (src[2] + src[4]) * LP9_APPROX_56_220 +
         src[3] * LP9_APPROX_72_220
         ) LP9_DIV_220;
      ++src;
      *dst++ =                  // [ 8^ 28 56 (72) ]
        (src[0] * LP9_APPROX_8_164 +
         src[1] * LP9_APPROX_28_164 +
         src[2] * LP9_APPROX_56_164 +
         src[3] * LP9_APPROX_72_164
         ) LP9_DIV_164;
      src += 4;  // src back to same as dst (start of next line)
    }
}

// ######################################################################
void c_intg_low_pass_9_y_fewbits_optim(const int* src,
                                   const int w, const int h,
                                   int* dst)
{
  ASSERT(h >= 9);

  // index computation speedup:
  const int w2 = w + w, w3 = w2 + w, w4 = w3 + w, w5 = w4 + w, w6 = w5 + w;

  // *** vertical pass ***
  for (int i = 0; i < w; ++i)
    {
      *dst++ =
        (src[ 0] * LP9_APPROX_72_164 +
         src[ w] * LP9_APPROX_56_164 +
         src[w2] * LP9_APPROX_28_164 +
         src[w3] * LP9_APPROX_8_164
         ) LP9_DIV_164;
      ++src;
    }
  src -= w; // back to top-left
  for (int i = 0; i < w; ++i)
    {
      *dst++ =
        ((src[ 0] + src[w2]) * LP9_APPROX_56_220 +
         src[ w] * LP9_APPROX_72_220 +
         src[w3] * LP9_APPROX_28_220 +
         src[w4] * LP9_APPROX_8_220
         ) LP9_DIV_220;
      ++src;
    }
  src -= w; // back to top-left
  for (int i = 0; i < w; ++i)
    {
      *dst++ =
        ((src[ 0] + src[w4]) * LP9_APPROX_28_248 +
         (src[ w] + src[w3]) * LP9_APPROX_56_248 +
         src[w2] * LP9_APPROX_72_248 +
         src[w5] * LP9_APPROX_8_248
         ) LP9_DIV_248;
      ++src;
    }
  src -= w; // back to top-left

  for (int j = 0; j < h - 6; j ++)
    for (int i = 0; i < w; ++i)
      {
        *dst++ =
          ((src[ 0] + src[w6]) *  8 +
           (src[ w] + src[w5]) * 28 +
           (src[w2] + src[w4]) * 56 +
           src[w3]  * 72
           ) >> 8;
        ++src;
      }

  for (int i = 0; i < w; ++i)
    {
      *dst++ =
        (src[ 0] * LP9_APPROX_8_248 +
         (src[ w] + src[w5]) * LP9_APPROX_28_248 +
         (src[w2] + src[w4]) * LP9_APPROX_56_248 +
         src[w3] * LP9_APPROX_72_248
         ) LP9_DIV_248;
      ++src;
    }
  for (int i = 0; i < w; ++i)
    {
      *dst++ =
        (src[ 0] * LP9_APPROX_8_220 +
         src[ w] * LP9_APPROX_28_220 +
         (src[w2] + src[w4]) * LP9_APPROX_56_220 +
         src[w3] * LP9_APPROX_72_220
         ) LP9_DIV_220;
      ++src;
    }
  for (int i = 0; i < w; ++i)
    {
      *dst++ =
        (src[ 0] * LP9_APPROX_8_164 +
         src[ w] * LP9_APPROX_28_164 +
         src[w2] * LP9_APPROX_56_164 +
         src[w3] * LP9_APPROX_72_164
         ) LP9_DIV_164;
      ++src;
    }
}

// ######################################################################
void c_intg_x_filter_clean_manybits(const int* src,
                                     const int w, const int h,
                                     const int* hf_flipped, const int hfs,
                                     const int shiftbits,
                                     int* dst)
{
  ASSERT(w >= hfs);
  ASSERT(hfs > 0);

  ASSERT(hfs & 1);
  const int hfs2 = (hfs - 1) / 2;

  for (int jj = 0; jj < h; ++jj)
    {
      // leftmost points:
      for (int j = hfs2; j < hfs - 1; ++j)
        {
          const int fp = hfs - 1 - j;
          int sum = 0;
          for (int k = 0; k <= j; k ++)
            sum += hf_flipped[fp + k];

          int val = 0;
          for (int k = 0; k <= j; k ++)
            val += (src[k] / sum) * hf_flipped[fp + k];

          *dst++ = val;
        }

      // bulk (far from the borders):
      for (int i = 0; i < w - hfs + 1; ++i)
        {
          int val = 0;
          for (int k = 0; k < hfs; k ++)
            val += (src[k] >> shiftbits) * hf_flipped[k];
          *dst++ = val;
          src++;
        }

      // rightmost points:
      for (int j = hfs - 2; j >= hfs2; j --)
        {
          int sum = 0;
          for (int k = 0; k <= j; k ++)
            sum += hf_flipped[k];

          int val = 0;
          for (int k = 0; k <= j; k ++)
            val += (src[k] / sum) * hf_flipped[k];

          *dst++ = val;
          src++;
        }
      src += hfs2;  // src back to same as dst (start of next line)
    }
}

// ######################################################################
void c_intg_x_filter_clean_fewbits(const int* src,
                                     const int w, const int h,
                                     const int* hf_flipped, const int hfs,
                                     const int shiftbits,
                                     int* dst)
{
  ASSERT(w >= hfs);
  ASSERT(hfs > 0);

  ASSERT(hfs & 1);
  const int hfs2 = (hfs - 1) / 2;

  for (int jj = 0; jj < h; ++jj)
    {
      // leftmost points:
      for (int j = hfs2; j < hfs - 1; ++j)
        {
          const int fp = hfs - 1 - j;
          int sum = 0;
          int val = 0;
          for (int k = 0; k <= j; k ++)
            {
              val += src[k] * hf_flipped[fp + k];
              sum += hf_flipped[fp + k];
            }

          *dst++ = val / sum;
        }

      // bulk (far from the borders):
      for (int i = 0; i < w - hfs + 1; ++i)
        {
          int val = 0;
          for (int k = 0; k < hfs; k ++)
            val += src[k] * hf_flipped[k];
          *dst++ = (val >> shiftbits);
          src++;
        }

      // rightmost points:
      for (int j = hfs - 2; j >= hfs2; j --)
        {
          int sum = 0;
          int val = 0;
          for (int k = 0; k <= j; k ++)
            {
              val += src[k] * hf_flipped[k];
              sum += hf_flipped[k];
            }

          *dst++ = val / sum;
          src++;
        }
      src += hfs2;  // src back to same as dst (start of next line)
    }
}

// ######################################################################
void c_intg_x_filter_clean_small_manybits(const int* src,
                                           const int w, const int h,
                                           const int* hf_flipped, const int hfs,
                                           const int shiftbits,
                                           int* dst)
{
  ASSERT(w < hfs);
  ASSERT(hfs > 0);

  ASSERT(hfs & 1);
  const int hfs2 = (hfs - 1) / 2;

  for (int j = 0; j < h; ++j)
    for (int i = 0; i < w; ++i)
      {
        int sum = 0;
        for (int k = 0; k < hfs; k ++)
          if (i + k - hfs2 >= 0 && i + k - hfs2 < w)
            sum += hf_flipped[k];

        int val = 0;
        for (int k = 0; k < hfs; k ++)
          if (i + k - hfs2 >= 0 && i + k - hfs2 < w)
            val += (src[k - hfs2] / sum) * hf_flipped[k];

        *dst++ = val;
        src ++;
      }
}

// ######################################################################
void c_intg_x_filter_clean_small_fewbits(const int* src,
                                           const int w, const int h,
                                           const int* hf_flipped, const int hfs,
                                           const int shiftbits,
                                           int* dst)
{
  ASSERT(w < hfs);
  ASSERT(hfs > 0);

  ASSERT(hfs & 1);
  const int hfs2 = (hfs - 1) / 2;

  for (int j = 0; j < h; ++j)
    for (int i = 0; i < w; ++i)
      {
        int sum = 0;
        int val = 0;
        for (int k = 0; k < hfs; k ++)
          if (i + k - hfs2 >= 0 && i + k - hfs2 < w)
            {
              val += src[k - hfs2] * hf_flipped[k];
              sum += hf_flipped[k];
            }

        *dst++ = val / sum;
        src ++;
      }
}

// ######################################################################
void c_intg_y_filter_clean_manybits(const int* src,
                                     const int w, const int h,
                                     const int* vf_flipped, const int vfs,
                                     const int shiftbits,
                                     int* dst)
{
  ASSERT(h >= vfs);
  ASSERT(vfs > 0);

  ASSERT(vfs & 1);
  const int vfs2 = (vfs - 1) / 2;

  int ww[vfs]; // precompute row offsets
  for (int x = 0; x < vfs; x ++)
    ww[x] = w * x;

  // top points:
  for (int j = vfs2; j < vfs - 1; ++j)
    {
      int fp = vfs - 1 - j;
      for (int i = 0; i < w; ++i)  // scan all points of given horiz
        {
          int sum = 0;
          for (int k = 0; k <= j; k ++)
            sum += vf_flipped[fp + k];

          int val = 0;
          for (int k = 0; k <= j; k ++)
            val += (src[ww[k]] / sum) * vf_flipped[fp + k];

          *dst++ = val;
          src++;
        }
      src -= w;   // back to top-left corner
    }

  // bulk (far from edges):
  for (int j = 0; j < h - vfs + 1; ++j)
    for (int i = 0; i < w; ++i)
      {
        int val = 0;
        for (int k = 0; k < vfs; k ++)
          val += (src[ww[k]] >> shiftbits) * vf_flipped[k];
        *dst++ = val;
        src++;
      }

  // bottommost points:
  for (int j = vfs - 2; j >= vfs2; j --)
    for (int i = 0; i < w; ++i)
      {
        int sum = 0;
        for (int k = 0; k <= j; k ++)
          sum += vf_flipped[k];

        int val = 0;
        for (int k = 0; k <= j; k ++)
          val += (src[ww[k]] / sum) * vf_flipped[k];

        *dst++ = val;
        ++src;
      }
}

// ######################################################################
void c_intg_y_filter_clean_fewbits(const int* src,
                                     const int w, const int h,
                                     const int* vf_flipped, const int vfs,
                                     const int shiftbits,
                                     int* dst)
{
  ASSERT(h >= vfs);
  ASSERT(vfs > 0);

  ASSERT(vfs & 1);
  const int vfs2 = (vfs - 1) / 2;

  int ww[vfs]; // precompute row offsets
  for (int x = 0; x < vfs; x ++)
    ww[x] = w * x;

  // top points:
  for (int j = vfs2; j < vfs - 1; ++j)
    {
      int fp = vfs - 1 - j;
      for (int i = 0; i < w; ++i)  // scan all points of given horiz
        {
          int sum = 0;
          int val = 0;
          for (int k = 0; k <= j; k ++)
            {
              val += src[ww[k]] * vf_flipped[fp + k];
              sum += vf_flipped[fp + k];
            }

          *dst++ = val / sum;
          src++;
        }
      src -= w;   // back to top-left corner
    }

  // bulk (far from edges):
  for (int j = 0; j < h - vfs + 1; ++j)
    for (int i = 0; i < w; ++i)
      {
        int val = 0;
        for (int k = 0; k < vfs; k ++)
          val += src[ww[k]] * vf_flipped[k];
        *dst++ = (val >> shiftbits);
        src++;
      }

  // bottommost points:
  for (int j = vfs - 2; j >= vfs2; j --)
    for (int i = 0; i < w; ++i)
      {
        int sum = 0;
        int val = 0;
        for (int k = 0; k <= j; k ++)
          {
            val += src[ww[k]] * vf_flipped[k];
            sum += vf_flipped[k];
          }

        *dst++ = val / sum;
        ++src;
      }
}

// ######################################################################
void c_intg_y_filter_clean_small_manybits(const int* src,
                                           const int w, const int h,
                                           const int* vf_flipped, const int vfs,
                                           const int shiftbits,
                                           int* dst)
{
  ASSERT(h < vfs);
  ASSERT(vfs > 0);

  ASSERT(vfs & 1);
  const int vfs2 = (vfs - 1) / 2;

  for (int j = 0; j < h; ++j)
    for (int i = 0; i < w; ++i)
      {
        int sum = 0;
        for (int k = 0; k < vfs; k ++)
          if (j + k - vfs2 >= 0 && j + k - vfs2 < h)
            sum += vf_flipped[k];

        int val = 0;
        for (int k = 0; k < vfs; k ++)
          if (j + k - vfs2 >= 0 && j + k - vfs2 < h)
            val += (src[w * (k - vfs2)] / sum) * vf_flipped[k];

        *dst++ = val;
        ++src;
      }
}

// ######################################################################
void c_intg_y_filter_clean_small_fewbits(const int* src,
                                           const int w, const int h,
                                           const int* vf_flipped, const int vfs,
                                           const int shiftbits,
                                           int* dst)
{
  ASSERT(h < vfs);
  ASSERT(vfs > 0);

  ASSERT(vfs & 1);
  const int vfs2 = (vfs - 1) / 2;

  for (int j = 0; j < h; ++j)
    for (int i = 0; i < w; ++i)
      {
        int sum = 0;
        int val = 0;
        for (int k = 0; k < vfs; k ++)
          if (j + k - vfs2 >= 0 && j + k - vfs2 < h)
            {
              val += src[w * (k - vfs2)] * vf_flipped[k];
              sum += vf_flipped[k];
            }

        *dst++ = val / sum;
        ++src;
      }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_C_INTEGER_MATH_OPS_C_DEFINED

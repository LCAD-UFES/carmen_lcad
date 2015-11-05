/*!@file Envision/env_c_math_ops.c Fixed-point integer math versions of some of our floating-point image functions */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_c_math_ops.c $
// $Id: env_c_math_ops.c 11331 2009-06-23 17:57:49Z itti $
//

#ifndef ENVISION_ENV_C_MATH_OPS_C_DEFINED
#define ENVISION_ENV_C_MATH_OPS_C_DEFINED

#include "Envision/env_c_math_ops.h"

#include "Envision/env_log.h"
#include "Envision/env_types.h"

// ######################################################################
void env_c_lowpass_5_x_dec_x_fewbits_optim(const intg32* src,
                                           const env_size_t w,
                                           const env_size_t h,
                                           intg32* dst,
                                           const env_size_t w2)
{
        ENV_ASSERT(w2 == w/2);

        if (w == 2 || w == 3) //////////////////////////////////////////////////
                for (env_size_t j = 0; j < h; ++j)
                {
                        // leftmost point  [ (6^) 4 ] / 10
                        *dst++ = (src[0] * 3 + src[1] * 2) / 5;
                        src += w;  // src back to same position as dst
                }
        else  ////////////////////////////// general case for width() >= 4
                // *** unfolded version (all particular cases treated) for
                // max speed.
                // notations: in () is the position of dest ptr, and ^ is src ptr
                // ########## horizontal pass
                for (env_size_t j = 0; j < h; ++j)
                {
                        const intg32* src2 = src;
                        // leftmost point  [ (8^) 4 ] / 12
                        *dst++ = (src2[0] * 2 + src2[1]) / 3;

                        // skip second point

                        // rest of the line except last 2 points  [ .^ 4 (8) 4 ] / 16
                        for (env_size_t i = 0; i < w-3; i += 2)
                        {
                                *dst++ = (src2[1] + src2[3] + src2[2] * 2) >> 2;
                                src2 += 2;
                        }

                        src += w;
                }
}

// ######################################################################
void env_c_lowpass_5_y_dec_y_fewbits_optim(const intg32* src,
                                           const env_size_t w,
                                           const env_size_t h,
                                           intg32* dst,
                                           const env_size_t h2)
{
        ENV_ASSERT(h2 == h/2);

        // ########## vertical pass  (even though we scan horiz for speedup)
        const env_size_t w2 = w + w, w3 = w2 + w; // speedup

        if (h == 2 || h == 3) //////////////////////////////////////////////////
        {
                // topmost points  ( [ (6^) 4 ] / 10 )^T
                for (env_size_t i = 0; i < w; ++i)
                {
                        *dst++ = (src[0] * 3 + src[w] * 2) / 5;
                        src++;
                }
                src -= w;  // go back to top-left
        }
        else  ///////////////////////////////// general case for height >= 4
        {
                // topmost points  ( [ (8^) 4 ] / 12 )^T
                for (env_size_t k = 0; k < w; ++k)
                {
                        *dst++ = (src[ 0] * 2 + src[ w]) / 3;
                        src++;
                }
                src -= w;  // go back to top-left

                // second point skipped

                // rest of the column except last 2 points ( [ .^ 4 (8) 4 ] / 16 )T
                for (env_size_t i = 0; i < h-3; i += 2)
                {
                        for (env_size_t k = 0; k < w; ++k)
                        {
                                *dst++ = (src[ w] + src[w3] + src[w2] * 2) >> 2;
                                src++;
                        }
                        src += w;
                }
        }
}

// ######################################################################
void env_c_lowpass_9_x_fewbits_optim(const intg32* src,
                                     const env_size_t w,
                                     const env_size_t h,
                                     intg32* dst)
{
        ENV_ASSERT(w >= 9);

        // boundary conditions: truncated filter
        for (env_size_t j = 0; j < h; ++j)
        {
                // leftmost points
                *dst++ =                  // [ (72^) 56 28 8 ]
                        (src[0] * 72 +
                         src[1] * 56 +
                         src[2] * 28 +
                         src[3] *  8
                                ) / 164;
                *dst++ =                  // [ 56^ (72) 56 28 8 ]
                        ((src[0] + src[2]) * 56 +
                         src[1] * 72 +
                         src[3] * 28 +
                         src[4] *  8
                                ) / 220;
                *dst++ =                  // [ 28^ 56 (72) 56 28 8 ]
                        ((src[0] + src[4]) * 28 +
                         (src[1] + src[3]) * 56 +
                         src[2] * 72 +
                         src[5] *  8
                                ) / 248;

                // far from the borders
                for (env_size_t i = 0; i < w - 6; ++i)
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
                        (src[0] *  8 +
                         (src[1] + src[5]) * 28 +
                         (src[2] + src[4]) * 56 +
                         src[3] * 72
                                ) / 248;
                ++src;
                *dst++ =                  // [ 8^ 28 56 (72) 56 ]
                        (src[0] *  8 +
                         src[1] * 28 +
                         (src[2] + src[4]) * 56 +
                         src[3] * 72
                                ) / 220;
                ++src;
                *dst++ =                  // [ 8^ 28 56 (72) ]
                        (src[0] *  8 +
                         src[1] * 28 +
                         src[2] * 56 +
                         src[3] * 72
                                ) / 164;
                src += 4;  // src back to same as dst (start of next line)
        }
}

// ######################################################################
void env_c_lowpass_9_y_fewbits_optim(const intg32* src,
                                     const env_size_t w,
                                     const env_size_t h,
                                     intg32* dst)
{
        ENV_ASSERT(h >= 9);

        // index computation speedup:
        const env_size_t w2 = w + w, w3 = w2 + w, w4 = w3 + w, w5 = w4 + w, w6 = w5 + w;

        // *** vertical pass ***
        for (env_size_t i = 0; i < w; ++i)
        {
                *dst++ =
                        (src[ 0] * 72 +
                         src[ w] * 56 +
                         src[w2] * 28 +
                         src[w3] *  8
                                ) / 164;
                ++src;
        }
        src -= w; // back to top-left
        for (env_size_t i = 0; i < w; ++i)
        {
                *dst++ =
                        ((src[ 0] + src[w2]) * 56 +
                         src[ w] * 72 +
                         src[w3] * 28 +
                         src[w4] *  8
                                ) / 220;
                ++src;
        }
        src -= w; // back to top-left
        for (env_size_t i = 0; i < w; ++i)
        {
                *dst++ =
                        ((src[ 0] + src[w4]) * 28 +
                         (src[ w] + src[w3]) * 56 +
                         src[w2] * 72 +
                         src[w5] *  8
                                ) / 248;
                ++src;
        }
        src -= w; // back to top-left

        for (env_size_t j = 0; j < h - 6; j ++)
                for (env_size_t i = 0; i < w; ++i)
                {
                        *dst++ =
                                ((src[ 0] + src[w6]) *  8 +
                                 (src[ w] + src[w5]) * 28 +
                                 (src[w2] + src[w4]) * 56 +
                                 src[w3]  * 72
                                        ) >> 8;
                        ++src;
                }

        for (env_size_t i = 0; i < w; ++i)
        {
                *dst++ =
                        (src[ 0] *  8 +
                         (src[ w] + src[w5]) * 28 +
                         (src[w2] + src[w4]) * 56 +
                         src[w3] * 72
                                ) / 248;
                ++src;
        }
        for (env_size_t i = 0; i < w; ++i)
        {
                *dst++ =
                        (src[ 0] *  8 +
                         src[ w] * 28 +
                         (src[w2] + src[w4]) * 56 +
                         src[w3] * 72
                                ) / 220;
                ++src;
        }
        for (env_size_t i = 0; i < w; ++i)
        {
                *dst++ =
                        (src[ 0] *  8 +
                         src[ w] * 28 +
                         src[w2] * 56 +
                         src[w3] * 72
                                ) / 164;
                ++src;
        }
}

// ######################################################################
void env_c_get_min_max(const intg32* src, const env_size_t sz,
                       intg32* xmini, intg32* xmaxi)
{
        ENV_ASSERT(sz > 0);
        *xmini = *xmaxi = src[0];
        for (env_size_t i = 0; i < sz; ++i)
        {
                if (src[i] < *xmini) *xmini = src[i];
                else if (src[i] > *xmaxi) *xmaxi = src[i];
        }
}

// ######################################################################
void env_c_inplace_rectify(intg32* dst, const env_size_t sz)
{
        for (env_size_t i = 0; i < sz; ++i)
                if (dst[i] < 0) dst[i] = 0;
}

// ######################################################################
void env_c_inplace_normalize(intg32* const dst, const env_size_t sz,
                             const intg32 nmin, const intg32 nmax,
                             intg32* const actualmin_p,
                             intg32* const actualmax_p,
                             const intg32 rangeThresh)
{
        ENV_ASSERT(sz > 0);
        ENV_ASSERT(nmax >= nmin);

        intg32 mi, ma;
        env_c_get_min_max(dst, sz, &mi, &ma);
        const intg32 old_scale = ma - mi;
        if (old_scale == 0 || old_scale < rangeThresh) // input image is uniform
        { for (env_size_t i = 0; i < sz; ++i) dst[i] = 0; return; }
        const intg32 new_scale = nmax - nmin;

        if (new_scale == 0) // output range is uniform
        { for (env_size_t i = 0; i < sz; ++i) dst[i] = nmin; return; }


        intg32 actualmin, actualmax;

        if (old_scale == new_scale)
        {
                const intg32 add = nmin - mi;
                if (add != 0)
                        for (env_size_t i = 0; i < sz; ++i)
                                dst[i] += add;

                actualmin = nmin;
                actualmax = nmax;
        }
#if defined(ENV_INTG64_TYPE)
        else
        {
                for (env_size_t i = 0; i < sz; ++i)
                        dst[i] = nmin + ((((intg64)(dst[i] - mi)) * new_scale)
                                         / old_scale);

                actualmin = nmin;
                actualmax = nmin + ((((intg64)(old_scale)) * new_scale)
                                    / old_scale);
        }
#else
        else if (old_scale > new_scale)
        {
                // we want to do new = (old*new_scale)/oscale;
                // however, the old*new_scale part might overflow if
                // old or new_scale is too large, so let's reduce both
                // new_scale and oscale until new_scale*old won't
                // overflow

                const intg32 nscale_max = INTG32_MAX / old_scale;

                intg32 new_scale_reduced = new_scale;
                intg32 old_scale_reduced = old_scale;

                while (new_scale_reduced > nscale_max)
                {
                        new_scale_reduced /= 2;
                        old_scale_reduced /= 2;
                }

                ENV_ASSERT(new_scale_reduced >= 1);
                ENV_ASSERT(old_scale_reduced >= 1);

                for (env_size_t i = 0; i < sz; ++i)
                        dst[i] = nmin
                                + (((dst[i] - mi) * new_scale_reduced)
                                   / (old_scale_reduced+1));

                actualmin = nmin;
                actualmax = nmin + ((old_scale * new_scale_reduced)
                                    / (old_scale_reduced+1));
        }
        else // (old_scale < new_scale)
        {
                const intg32 mul = new_scale / old_scale;
                for (env_size_t i = 0; i < sz; ++i)
                        dst[i] = nmin + ((dst[i] - mi) * mul);

                actualmin = nmin;
                actualmax = nmin + (old_scale * mul);
        }
#endif // !defined(ENV_INTG64_TYPE)

        // Don't assign to the pointers until the very end, in case
        // the user passes pointers to nmin,nmax as the
        // actualmin/actualmax pointers
        if (actualmin_p)
                *actualmin_p = actualmin;
        if (actualmax_p)
                *actualmax_p = actualmax;

}

// ######################################################################
void env_c_luminance_from_byte(const struct env_rgb_pixel* const src,
                               const env_size_t sz,
                               const env_size_t nbits,
                               intg32* const dst)
{
        if (nbits > 8)
                for (env_size_t i = 0; i < sz; ++i)
                dst[i] = ((src[i].p[0] + src[i].p[1] + src[i].p[2]) / 3) << (nbits - 8);
        else if (nbits < 8)
                for (env_size_t i = 0; i < sz; ++i)
                dst[i] = ((src[i].p[0] + src[i].p[1] + src[i].p[2]) / 3) >> (8 - nbits);
        else // nbits == 8
                for (env_size_t i = 0; i < sz; ++i)
                dst[i] = (src[i].p[0] + src[i].p[1] + src[i].p[2]) / 3;
}

// ######################################################################
void env_c_image_div_scalar(const intg32* const a,
                            const env_size_t sz,
                            intg32 val,
                            intg32* const dst)
{
        for (env_size_t i = 0; i < sz; ++i)
                dst[i] = a[i] / val;
}

// ######################################################################
void env_c_image_div_scalar_accum(const intg32* const a,
                                  const env_size_t sz,
                                  intg32 val,
                                  intg32* const dst)
{
        for (env_size_t i = 0; i < sz; ++i)
                dst[i] += a[i] / val;
}

// ######################################################################
void env_c_image_minus_image(const intg32* const a,
                             const intg32* const b,
                             const env_size_t sz,
                             intg32* const dst)
{
        for (env_size_t i = 0; i < sz; ++i)
                dst[i] = a[i] - b[i];
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* c-file-style: "linux" */
/* End: */

#endif // ENVISION_ENV_C_MATH_OPS_C_DEFINED

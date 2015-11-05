/*!@file Envision/env_image_ops.c Fixed-point integer math versions of some of our floating-point image functions */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_image_ops.c $
// $Id: env_image_ops.c 11874 2009-10-21 01:03:31Z dparks $
//

#ifndef ENVISION_ENV_IMAGE_OPS_C_DEFINED
#define ENVISION_ENV_IMAGE_OPS_C_DEFINED

#include "Envision/env_image_ops.h"

#include "Envision/env_c_math_ops.h"
#include "Envision/env_log.h"


// ######################################################################
void env_dec_xy(const struct env_image* src, struct env_image* result)
{
        // do not go smaller than 1x1:
        if (src->dims.w <= 1 && src->dims.h <= 1)
        {
                env_img_copy_src_dst(src, result);
                return;
        }

        if (src->dims.w == 1)  // only thinout vertic
        {
                env_dec_y(src, result);
                return;
        }

        if (src->dims.h == 1)
        {
                env_dec_x(src, result);
                return;
        }

        const struct env_dims dims2 = { src->dims.w / 2, src->dims.h / 2 };

        env_img_resize_dims(result, dims2);

        const intg32* sptr = env_img_pixels(src);
        intg32* dptr = env_img_pixelsw(result);
        const env_size_t skip = src->dims.w % 2 + src->dims.w;

        for (env_size_t j = 0; j < dims2.h; ++j)
        {
                for (env_size_t i = 0; i < dims2.w; ++i)
                {
                        *dptr++ = *sptr;   // copy one pixel
                        sptr += 2;    // skip some pixels
                }
                sptr += skip;          // skip to start of next line
        }
}

// ######################################################################
void env_dec_x(const struct env_image* src, struct env_image* result)
{
        if (src->dims.w <= 1) // do not go smaller than 1 pixel wide
        {
                env_img_copy_src_dst(src, result);
                return;
        }

        const struct env_dims dims2 = { src->dims.w / 2, src->dims.h };
        const env_size_t skip = src->dims.w % 2;
        ENV_ASSERT(dims2.w > 0);

        env_img_resize_dims(result, dims2);

        const intg32* sptr = env_img_pixels(src);
        intg32* dptr = env_img_pixelsw(result);

        for (env_size_t j = 0; j < dims2.h; ++j)
        {
                for (env_size_t i = 0; i < dims2.w; ++i)
                {
                        *dptr++ = *sptr;   // copy one point
                        sptr += 2;    // skip a few points
                }
                sptr += skip;
        }
}

// ######################################################################
void env_dec_y(const struct env_image* src, struct env_image* result)
{
        if (src->dims.h <= 1) // do not go smaller than 1 pixel high
        {
                env_img_copy_src_dst(src, result);
                return;
        }

        const struct env_dims dims2 = { src->dims.w, src->dims.h / 2 };
        ENV_ASSERT(dims2.h > 0);

        env_img_resize_dims(result, dims2);

        const intg32* sptr = env_img_pixels(src);
        intg32* dptr = env_img_pixelsw(result);
        const env_size_t skip = dims2.w * 2;

        for (env_size_t j = 0; j < dims2.h; ++j)
        {
                for (env_size_t i = 0; i < dims2.w; ++i)
                        dptr[i] = sptr[i];

                dptr += dims2.w;
                sptr += skip;
        }
}

// ######################################################################
// Anderson's separable kernel: 1/16 * [1 4 6 4 1]
void env_lowpass_5_x_dec_x(const struct env_image* src,
                           const struct env_math* imath,
                           struct env_image* result)
{
        const env_size_t w = src->dims.w;
        const env_size_t h = src->dims.h;

        if (w < 2) // nothing to smooth
        {
                env_img_copy_src_dst(src, result);
                return;
        }

        const struct env_dims dims2 = { w / 2, h };
        ENV_ASSERT(dims2.w > 0);

        env_img_resize_dims(result, dims2);

#ifndef ENV_NO_DEBUG
        const env_size_t filterbits = 4; // log2(16)
        const env_size_t accumbits = 3; // ceil(log2(5))
#endif

        ENV_ASSERT((imath->nbits + filterbits + accumbits + 1) < (8*sizeof(intg32)));

        env_c_lowpass_5_x_dec_x_fewbits_optim
                (env_img_pixels(src), w, h,
                 env_img_pixelsw(result), dims2.w);
}

// ######################################################################
// Anderson's separable kernel: 1/16 * [1 4 6 4 1]
void env_lowpass_5_y_dec_y(const struct env_image* src,
                           const struct env_math* imath,
                           struct env_image* result)
{
        const env_size_t w = src->dims.w;
        const env_size_t h = src->dims.h;

        if (h < 2) // nothing to smooth
        {
                env_img_copy_src_dst(src, result);
                return;
        }

        const struct env_dims dims2 = { w, h / 2 };
        ENV_ASSERT(dims2.h > 0);

        env_img_resize_dims(result, dims2);

#ifndef ENV_NO_DEBUG
        const env_size_t filterbits = 4; // log2(16)
        const env_size_t accumbits = 3; // ceil(log2(5))
#endif

        ENV_ASSERT((imath->nbits + filterbits + accumbits + 1) < (8*sizeof(intg32)));

        env_c_lowpass_5_y_dec_y_fewbits_optim
                (env_img_pixels(src), w, h,
                 env_img_pixelsw(result), dims2.h);
}

// ######################################################################
void env_lowpass_9_x(const struct env_image* source,
                     const struct env_math* imath,
                     struct env_image* result)
{
        ENV_ASSERT(env_dims_equal(result->dims, source->dims));

#ifndef ENV_NO_DEBUG
        const env_size_t filterbits = 8; // log2(256)
        const env_size_t accumbits = 4; // ceil(log2(9))
#endif

        ENV_ASSERT((imath->nbits + filterbits + accumbits + 1) < (8*sizeof(intg32)));

        const env_size_t w = source->dims.w;
        const env_size_t h = source->dims.h;

        if (w < 2) // nothing to smooth
        {
                env_img_copy_src_dst(source, result);
                return;
        }

        if (w < 9)  // use inefficient implementation for small images
        {
                const intg32 hf_flipped[9] = { 1, 8, 28, 56, 70, 56, 28, 8, 1 };
                const env_size_t hfs = 9;

                const intg32* src = env_img_pixels(source);
                intg32* dst = env_img_pixelsw(result);

                ENV_ASSERT(hfs & 1); // filter size must be odd
                const env_size_t hfs2 = (hfs - 1) / 2;

                for (env_size_t j = 0; j < h; ++j)
                        for (env_size_t i = 0; i < w; ++i)
                        {
                                intg32 sum = 0;
                                intg32 val = 0;
                                for (env_size_t k = 0; k < hfs; ++k)
                                {
                                        if (i + k < hfs2
                                            || i + k >= w + hfs2)
                                                continue;

                                        // convert to signed integers
                                        // to avoid wraparound when
                                        // k<hfs2
                                        val += src[(env_ssize_t) k
                                                   - (env_ssize_t) hfs2]
                                                * hf_flipped[k];
                                        sum += hf_flipped[k];
                                }

                                *dst++ = val / sum;
                                ++src;
                        }
                return;
        }

        env_c_lowpass_9_x_fewbits_optim(env_img_pixels(source), w, h,
                                        env_img_pixelsw(result));
}

// ######################################################################
void env_lowpass_9_y(const struct env_image* source,
                     const struct env_math* imath,
                     struct env_image* result)
{
        ENV_ASSERT(env_dims_equal(result->dims, source->dims));

#ifndef ENV_NO_DEBUG
        const env_size_t filterbits = 8; // log2(256)
        const env_size_t accumbits = 4; // ceil(log2(9))
#endif

        ENV_ASSERT((imath->nbits + filterbits + accumbits + 1) < (8*sizeof(intg32)));

        const env_size_t w = source->dims.w;
        const env_size_t h = source->dims.h;

        // if the height is less than 2, then the caller should handle
        // that condition differently since no smoothing need be done
        // (so the caller could either copy or swap the source into
        // the result location)
        ENV_ASSERT(h >= 2);

        if (h < 9)  // use inefficient implementation for small images
        {
                const intg32 vf_flipped[9] = { 1, 8, 28, 56, 70, 56, 28, 8, 1 };
                const env_size_t vfs = 9;

                const intg32* src = env_img_pixels(source);
                intg32* dst = env_img_pixelsw(result);

                ENV_ASSERT(vfs & 1); // filter size must be odd
                const env_size_t vfs2 = (vfs - 1) / 2;

                for (env_size_t j = 0; j < h; ++j)
                        for (env_size_t i = 0; i < w; ++i)
                        {
                                intg32 sum = 0;
                                intg32 val = 0;
                                for (env_size_t k = 0; k < vfs; ++k)
                                {
                                        if (j + k < vfs2
                                            || j + k >= h + vfs2)
                                                continue;

                                        // convert to signed integers
                                        // to avoid wraparound when
                                        // k<vfs2
                                        val += src[w *
                                                   ((env_ssize_t) k
                                                    - (env_ssize_t) vfs2)]
                                                * vf_flipped[k];
                                        sum += vf_flipped[k];
                                }

                                *dst++ = val / sum;
                                ++src;
                        }

                return;
        }

        env_c_lowpass_9_y_fewbits_optim(env_img_pixels(source), w, h,
                                        env_img_pixelsw(result));
}

// ######################################################################
void env_lowpass_9(const struct env_image* src,
                   const struct env_math* imath,
                   struct env_image* result)
{
        ENV_ASSERT(env_dims_equal(result->dims, src->dims));

        struct env_image tmp1;
        env_img_init(&tmp1, src->dims);
        env_lowpass_9_x(src, imath, &tmp1);
        if (tmp1.dims.h >= 2)
                env_lowpass_9_y(&tmp1, imath, result);
        else
                env_img_swap(&tmp1, result);
        env_img_make_empty(&tmp1);
}

// ######################################################################
void env_quad_energy(const struct env_image* img1,
                     const struct env_image* img2,
                     struct env_image* result)
{
        ENV_ASSERT(env_dims_equal(img1->dims, img2->dims));
        ENV_ASSERT(env_dims_equal(img1->dims, result->dims));

        const intg32* s1ptr = env_img_pixels(img1);
        const intg32* s2ptr = env_img_pixels(img2);
        intg32* dptr = env_img_pixelsw(result);

        const env_size_t sz = env_img_size(img1);

        for (env_size_t i = 0; i < sz; ++i)
        {
                const intg32 s1 = ENV_ABS(s1ptr[i]);
                const intg32 s2 = ENV_ABS(s2ptr[i]);

                /* "A Fast Approximation to the Hypotenuse" by Alan Paeth, from
                   "Graphics Gems", Academic Press, 1990

                   http://www.acm.org/pubs/tog/GraphicsGems/gems/HypotApprox.c

                   gives approximate value of sqrt(s1*s1+s2*s2) with only
                   overestimations, and then never by more than (9/8) + one bit
                   uncertainty
                */
                dptr[i] = (s1 > s2) ? (s1 + (s2 >> 1)) : ((s1 >> 1) + s2);
        }
}

// ######################################################################
void env_steerable_filter(const struct env_image* src,
                          const intg32 kxnumer, const intg32 kynumer,
                          const env_size_t kdenombits,
                          const struct env_math* imath,
                          struct env_image* result)
{
        ENV_ASSERT(env_dims_equal(result->dims, src->dims));

        struct env_image re; env_img_init(&re, src->dims);
        struct env_image im; env_img_init(&im, src->dims);
        const intg32* sptr = env_img_pixels(src);
        intg32* reptr = env_img_pixelsw(&re);
        intg32* imptr = env_img_pixelsw(&im);

        // (x,y) = (0,0) at center of image:
        const env_ssize_t w2l = ((env_ssize_t) src->dims.w) / 2;
        const env_ssize_t w2r = ((env_ssize_t) src->dims.w) - w2l;
        const env_ssize_t h2l = ((env_ssize_t) src->dims.h) / 2;
        const env_ssize_t h2r = ((env_ssize_t) src->dims.h) - h2l;

        // let's do a conservative check to make sure that we won't overflow
        // when we compute "arg" later on -- as a very rough estimate,
        // kxnumer and kynumer are on the order of 2^16 (8 bits from
        // kdenombits=8, 8 bits from ENV_TRIG_TABSIZ=256), which gives
        // room for w+h to be up to about 2^15
        ENV_ASSERT((INTG32_MAX / (ENV_ABS(kxnumer) + ENV_ABS(kynumer))) > (w2r + h2r));

        ENV_ASSERT((2 * ENV_TRIG_NBITS + 1) < 8*sizeof(intg32));

#ifndef ENV_NO_DEBUG
        const intg32 mdcutoff = INTG32_MAX >> (ENV_TRIG_NBITS+1);
#endif

        for (env_ssize_t j = -h2l; j < h2r; ++j)
                for (env_ssize_t i = -w2l; i < w2r; ++i)
                {
                        const intg32 arg = (i * kxnumer + j * kynumer) >> kdenombits;

                        env_ssize_t idx = arg % ENV_TRIG_TABSIZ;
                        if (idx < 0) idx += ENV_TRIG_TABSIZ;

                        const intg32 sval = *sptr++;

                        ENV_ASSERT(ENV_ABS(sval) < mdcutoff);

                        *reptr++ = (sval * imath->costab[idx]) >> (ENV_TRIG_NBITS+1);
                        *imptr++ = (sval * imath->sintab[idx]) >> (ENV_TRIG_NBITS+1);
                }

        env_lowpass_9(&re, imath, result);
        env_img_swap(&re, result);

        env_lowpass_9(&im, imath, result);
        env_img_swap(&im, result);

        env_quad_energy(&re, &im, result);

        env_img_make_empty(&re);
        env_img_make_empty(&im);
}

// ######################################################################
void env_attenuate_borders_inplace(struct env_image* a, env_size_t size)
{
        ENV_ASSERT(env_img_initialized(a));

        struct env_dims dims = a->dims;

        if (size * 2 > dims.w) size = dims.w / 2;
        if (size * 2 > dims.h) size = dims.h / 2;
        if (size < 1) return;  // forget it

        const intg32 size_plus_1 = (intg32) (size+1);

        // top lines:
        intg32 coeff = 1;
        intg32* aptr = env_img_pixelsw(a);
        for (env_size_t y = 0; y < size; y ++)
        {
                for (env_size_t x = 0; x < dims.w; x ++)
                {
                        *aptr = (*aptr / size_plus_1) * coeff;
                        ++aptr;
                }
                ++coeff;
        }
        // normal lines: start again from beginning to attenuate corners twice:
        aptr = env_img_pixelsw(a);
        for (env_size_t y = 0; y < dims.h; y ++)
        {
                coeff = 1;
                for (env_size_t x = 0; x < size; x ++)
                {
                        *(aptr + dims.w - 1 - x * 2) =
                                (*(aptr + dims.w - 1 - x * 2) / size_plus_1) * coeff;

                        *aptr = (*aptr / size_plus_1) * coeff;
                        ++aptr;
                        ++coeff;
                }
                aptr += dims.w - size;
        }
        // bottom lines
        aptr = env_img_pixelsw(a) + (dims.h - size) * dims.w;
        coeff = size;
        for (env_size_t y = dims.h - size; y < dims.h; y ++)
        {
                for (env_size_t x = 0; x < dims.w; ++x)
                {
                        *aptr = (*aptr / size_plus_1) * coeff;
                        ++aptr;
                }
                --coeff;
        }
}

// ######################################################################
void env_pyr_build_hipass_9(const struct env_image* image,
                            env_size_t firstlevel,
                            const struct env_math* imath,
                            struct env_pyr* result)
{
        ENV_ASSERT(env_img_initialized(image));

        // compute hipass as image - lowpass(image)

        const env_size_t depth = env_pyr_depth(result);

        if (depth == 0)
                return;

        struct env_image lpfima = env_img_initializer;

        // special case for the zero'th pyramid level so that we don't
        // have to make an extra copy of the input image at its
        // largest resolution
        env_img_resize_dims(&lpfima, image->dims);
        env_lowpass_9(image, imath, &lpfima);

        if (0 == firstlevel)
        {
                env_img_resize_dims(env_pyr_imgw(result, 0),
                                    image->dims);
                env_c_image_minus_image
                        (env_img_pixels(image),
                         env_img_pixels(&lpfima),
                         env_img_size(image),
                         env_img_pixelsw(env_pyr_imgw(result, 0)));
        }

        // now do the rest of the pyramid levels starting from level 1:
        for (env_size_t lev = 1; lev < depth; ++lev)
        {
                struct env_image dec = env_img_initializer;
                env_dec_xy(&lpfima, &dec);
                env_img_resize_dims(&lpfima, dec.dims);
                env_lowpass_9(&dec, imath, &lpfima);

                if (lev >= firstlevel)
                {
                        env_img_resize_dims(env_pyr_imgw(result, lev),
                                            dec.dims);
                        env_c_image_minus_image
                                (env_img_pixels(&dec),
                                 env_img_pixels(&lpfima),
                                 env_img_size(&dec),
                                 env_img_pixelsw(env_pyr_imgw(result, lev)));
                }

                env_img_make_empty(&dec);
        }

        env_img_make_empty(&lpfima);
}

// ######################################################################
void env_pyr_build_steerable_from_hipass_9(const struct env_pyr* hipass,
                                           const intg32 kxnumer,
                                           const intg32 kynumer,
                                           const env_size_t kdenombits,
                                           const struct env_math* imath,
                                           struct env_pyr* out)
{
        const env_size_t attenuation_width = 5;
        const env_size_t depth = env_pyr_depth(hipass);

        struct env_pyr result;
        env_pyr_init(&result, depth);

        for (env_size_t lev = 0; lev < depth; ++lev)
        {
                // if the hipass is empty at a given level, then just
                // leave the output empty at that level, too
                if (!env_img_initialized(env_pyr_img(hipass, lev)))
                        continue;

                env_img_resize_dims(env_pyr_imgw(&result, lev),
                                    env_pyr_img(hipass, lev)->dims);

                env_steerable_filter(env_pyr_img(hipass, lev),
                                     kxnumer, kynumer, kdenombits,
                                     imath, env_pyr_imgw(&result, lev));
                // attenuate borders that are overestimated due to filter trunctation:
                env_attenuate_borders_inplace(env_pyr_imgw(&result, lev),
                                              attenuation_width);
        }

        env_pyr_swap(out, &result);

        env_pyr_make_empty(&result);
}


// ######################################################################
void env_pyr_build_lowpass_5(const struct env_image* image,
                                 env_size_t firstlevel,
                                 const struct env_math* imath,
                                 struct env_pyr* result)
{
        ENV_ASSERT(env_img_initialized(image));
        ENV_ASSERT(env_pyr_depth(result) > 0);

        if (firstlevel == 0)
                env_img_copy_src_dst(image, env_pyr_imgw(result, 0));

        const env_size_t depth = env_pyr_depth(result);

        for (env_size_t lev = 1; lev < depth; ++lev)
        {
                struct env_image tmp1 = env_img_initializer;

                const struct env_image* prev =
                        lev == 1
                        ? image
                        : env_pyr_img(result, lev-1);

                env_lowpass_5_x_dec_x(prev,
                                      imath, &tmp1);
                env_lowpass_5_y_dec_y(&tmp1,
                                      imath, env_pyr_imgw(result, lev));

                if ((lev - 1) < firstlevel)
                {
                        env_img_make_empty(env_pyr_imgw(result, lev-1));
                }

                env_img_make_empty(&tmp1);
        }
}

// ######################################################################
void env_downsize_9_inplace(struct env_image* src, const env_size_t depth,
                            const struct env_math* imath)
{
        for (env_size_t i = 0; i < depth; ++i)
        {
                {
                        struct env_image tmp1;
                        env_img_init(&tmp1, src->dims);
                        env_lowpass_9_x(src, imath, &tmp1);
                        env_dec_x(&tmp1, src);
                        env_img_make_empty(&tmp1);
                }
                {
                        struct env_image tmp2;
                        env_img_init(&tmp2, src->dims);
                        if (src->dims.h >= 2)
                                env_lowpass_9_y(src, imath, &tmp2);
                        else
                                env_img_swap(src, &tmp2);
                        env_dec_y(&tmp2, src);
                        env_img_make_empty(&tmp2);
                }
        }
}

// ######################################################################
void env_rescale(const struct env_image* src, struct env_image* result)
{
        const env_ssize_t new_w = (env_ssize_t) result->dims.w;
        const env_ssize_t new_h = (env_ssize_t) result->dims.h;

        ENV_ASSERT(env_img_initialized(src));
        ENV_ASSERT(new_w > 0 && new_h > 0);

        const env_ssize_t orig_w = (env_ssize_t) src->dims.w;
        const env_ssize_t orig_h = (env_ssize_t) src->dims.h;

        // check if same size already
        if (new_w == orig_w && new_h == orig_h)
        {
                env_img_copy_src_dst(src, result);
                return;
        }

        intg32* dptr = env_img_pixelsw(result);
        const intg32* const sptr = env_img_pixels(src);

        // code inspired from one of the Graphics Gems book:
        /*
          (1) (x,y) are the original coords corresponding to scaled coords (i,j)
          (2) (x0,y0) are the greatest lower bound integral coords from (x,y)
          (3) (x1,y1) are the least upper bound integral coords from (x,y)
          (4) d00, d10, d01, d11 are the values of the original image at the corners
          of the rect (x0,y0),(x1,y1)
          (5) the value in the scaled image is computed from bilinear interpolation
          among d00,d10,d01,d11
        */
        for (env_ssize_t j = 0; j < new_h; ++j)
        {
                const env_ssize_t y_numer = ENV_MAX(((env_ssize_t) 0), j*2*orig_h+orig_h-new_h);
                const env_ssize_t y_denom = 2*new_h;

                const env_ssize_t y0 = y_numer / y_denom;
                const env_ssize_t y1 = ENV_MIN(y0 + 1, orig_h - 1);

                const env_ssize_t fy_numer = y_numer - y0 * y_denom;
                const env_ssize_t fy_denom = y_denom;
                ENV_ASSERT(fy_numer == (y_numer % y_denom));

                const env_ssize_t wy0 = orig_w * y0;
                const env_ssize_t wy1 = orig_w * y1;

                for (env_ssize_t i = 0; i < new_w; ++i)
                {
                        const env_ssize_t x_numer = ENV_MAX(((env_ssize_t) 0), i*2*orig_w+orig_w-new_w);
                        const env_ssize_t x_denom = 2*new_w;

                        const env_ssize_t x0 = x_numer / x_denom;
                        const env_ssize_t x1 = ENV_MIN(x0 + 1, orig_w - 1);

                        const env_ssize_t fx_numer = x_numer - x0 * x_denom;
                        const env_ssize_t fx_denom = x_denom;
                        ENV_ASSERT(fx_numer == (x_numer % x_denom));

                        const intg32 d00 = sptr[x0 + wy0];
                        const intg32 d10 = sptr[x1 + wy0];

                        const intg32 d01 = sptr[x0 + wy1];
                        const intg32 d11 = sptr[x1 + wy1];

                        const intg32 dx0 =
                                d00 + ((d10 - d00) / fx_denom) * fx_numer;
                        const intg32 dx1 =
                                d01 + ((d11 - d01) / fx_denom) * fx_numer;

                        // no need to clamp
                        *dptr++ = dx0 + ((dx1 - dx0) / fy_denom) * fy_numer;
                }
        }
}

// ######################################################################
void env_max_normalize_inplace(struct env_image* src,
                               const intg32 mi, const intg32 ma,
                               const enum env_maxnorm_type normtyp,
                               const intg32 rangeThresh)
{
        // do normalization depending on desired type:
        switch(normtyp)
        {
        case ENV_VCXNORM_NONE:
                env_max_normalize_none_inplace(src, mi, ma, rangeThresh);
                break;
        case ENV_VCXNORM_MAXNORM:
                env_max_normalize_std_inplace(src, mi, ma, rangeThresh);
                break;
        default:
                ENV_ASSERT2(0, "Invalid normalization type");
        }
}

// ######################################################################
void env_max_normalize_none_inplace(struct env_image* src,
                                    const intg32 nmi, const intg32 nma,
                                    const intg32 rangeThresh)
{
        // first clamp negative values to zero
        env_c_inplace_rectify(env_img_pixelsw(src), env_img_size(src));

        // then, normalize between mi and ma if not zero
        intg32 mi = nmi;
        intg32 ma = nma;
        if (mi != 0 || ma != 0)
                env_c_inplace_normalize(env_img_pixelsw(src),
                                        env_img_size(src),
                                        nmi, nma, &mi, &ma, rangeThresh);
}

// ######################################################################
void env_max_normalize_std_inplace(struct env_image* src,
                                   const intg32 nmi, const intg32 nma,
                                   const intg32 rangeThresh)
{
        if (!env_img_initialized(src))
                return;

        // first clamp negative values to zero
        env_c_inplace_rectify(env_img_pixelsw(src), env_img_size(src));

        // then, normalize between mi and ma if not zero
        intg32 mi = nmi;
        intg32 ma = nma;
        if (nmi != 0 || nma != 0)
                env_c_inplace_normalize(env_img_pixelsw(src),
                                        env_img_size(src),
                                        nmi, nma, &mi, &ma, rangeThresh);

        const env_size_t w = src->dims.w;
        const env_size_t h = src->dims.h;

        // normalize between mi and ma and multiply by (max - mean)^2

        // we want to detect quickly local maxes, but avoid getting local mins
        const intg32 thresh = mi + (ma - mi) / 10;

        // then get the mean value of the local maxima:
        const intg32* const dptr = env_img_pixels(src);
        intg32 lm_mean = 0;
        env_size_t numlm = 0;
        for (env_size_t j = 1; j+1 < h; ++j)
                for (env_size_t i = 1; i+1 < w; ++i)
                {
                        const env_size_t index = i + w * j;
                        const intg32 val = dptr[index];
                        if (val >= thresh &&
                            val >= dptr[index - w] &&
                            val >= dptr[index + w] &&
                            val >= dptr[index - 1] &&
                            val >= dptr[index + 1])  // local max
                        {
                                ++numlm;
                                ENV_ASSERT2(INTG32_MAX - val >= lm_mean,
                                            "integer overflow");
                                lm_mean += val;
                        }
                }

        if (numlm > 0)
                lm_mean /= numlm;

        ENV_ASSERT(ma >= lm_mean);

        intg32 factor = 1;

        // scale factor is (max - mean_local_max)^2:
        if (numlm > 1)
        {
                // make sure that (ma - lm_mean)^2 won't overflow:
                ENV_ASSERT((ma == lm_mean) ||
                           ((INTG32_MAX / (ma - lm_mean)) > (ma - lm_mean)));

                factor = ((ma - lm_mean) * (ma - lm_mean)) / ma;
        }
        else if (numlm == 1)  // a single narrow peak
        {
                factor = ma;
        }
        else
        {
                /* LERROR("No local maxes found !!"); */
        }

        if (factor != 1)
        {
                intg32* const itr = env_img_pixelsw(src);
                const env_size_t sz = env_img_size(src);
                for (env_size_t i = 0; i < sz; ++i)
                        itr[i] *= factor;
        }
}

// ######################################################################
void env_center_surround(const struct env_image* center,
                         const struct env_image* surround,
                         const int absol,
                         struct env_image* result)
{
        // result has the size of the larger image:
        ENV_ASSERT(env_dims_equal(result->dims, center->dims));

        const env_size_t lw = center->dims.w, lh = center->dims.h;
        const env_size_t sw = surround->dims.w, sh = surround->dims.h;

        ENV_ASSERT2(lw >= sw && lh >= sh,
                    "center must be larger than surround");

        const env_size_t scalex = lw / sw, remx = lw - 1 - (lw % sw);
        const env_size_t scaley = lh / sh, remy = lh - 1 - (lh % sh);

        // scan large image and subtract corresponding pixel from small image:
        env_size_t ci = 0, cj = 0;
        const intg32* lptr = env_img_pixels(center);
        const intg32* sptr = env_img_pixels(surround);
        intg32* dptr = env_img_pixelsw(result);

        if (absol)  // compute abs(hires - lowres):
        {
          for (env_size_t j = 0; j < lh; ++j)
          {
            for (env_size_t i = 0; i < lw; ++i)
            {
              if (*lptr > *sptr)
                *dptr++ = (*lptr++ - *sptr);
              else
                *dptr++ = (*sptr - *lptr++);

              if ((++ci) == scalex && i != remx) { ci = 0; ++sptr; }
            }
            if (ci) { ci = 0; ++sptr; }  // in case the reduction is not round
            if ((++cj) == scaley && j != remy) cj = 0; else sptr -= sw;
          }
        }
        else  // compute hires - lowres, clamped to 0:
        {
          for (env_size_t j = 0; j < lh; ++j)
          {
            for (env_size_t i = 0; i < lw; ++i)
            {
              if (*lptr > *sptr)
                *dptr++ = (*lptr++ - *sptr);
              else
              { *dptr++ = 0; lptr++; }

              if ((++ci) == scalex && i != remx) { ci = 0; ++sptr; }
            }
            if (ci) { ci = 0; ++sptr; } // in case the reduction is not round
            if ((++cj) == scaley && j != remy) cj = 0; else sptr -= sw;
          }
        }

        // attenuate borders:
        env_attenuate_borders_inplace
          (result, ENV_MAX(result->dims.w, result->dims.h) / 20);
}

// ######################################################################
void env_get_rgby(const struct env_rgb_pixel* const src,
                  const struct env_rgb_pixel* const src2,
                  const env_size_t sz,
                  struct env_image* rg,
                  struct env_image* by, const intg32 thresh,
                  const env_size_t inputbits)
{
        // red = [r - (g+b)/2]        [.] = clamp between 0 and 255
        // green = [g - (r+b)/2]
        // blue = [b - (r+g)/2]
        // yellow = [2*((r+g)/2 - |r-g| - b)]

        ENV_ASSERT(env_img_size(rg) == sz);
        ENV_ASSERT(env_img_size(by) == sz);

        intg32* rgptr = env_img_pixelsw(rg);
        intg32* byptr = env_img_pixelsw(by);

        const env_ssize_t lshift = ((env_ssize_t)inputbits) - 3;

        for (env_size_t i = 0; i < sz; ++i)
        {
                intg32 r, g, b;

                if (src2 == 0)
                {
                        r = (intg32) src[i].p[0];
                        g = (intg32) src[i].p[1];
                        b = (intg32) src[i].p[2];
                }
                else
                {
                        r = ((intg32) src[i].p[0] + (intg32) src2[i].p[0]) / 2;
                        g = ((intg32) src[i].p[1] + (intg32) src2[i].p[1]) / 2;
                        b = ((intg32) src[i].p[2] + (intg32) src2[i].p[2]) / 2;
                }

                // first do the luminanceNormalization:
                const intg32 lum = r + g + b;

                if (lum < thresh)  // too dark... no response from anybody
                {
                        rgptr[i] = byptr[i] = 0;
                }
                else
                {
                        // now compute color opponencies:
                        intg32 red = (2*r - g - b);
                        intg32 green = (2*g - r - b);
                        intg32 blue = (2*b - r - g);
                        intg32 yellow = (-2*blue - 4*ENV_ABS(r-g));

                        if (red < 0) red = 0;
                        if (green < 0) green = 0;
                        if (blue < 0) blue = 0;
                        if (yellow < 0) yellow=0;

                        // compute differences and normalize chroma by luminance:
                        if (lshift > 0)
                        {
                                rgptr[i] = (3*(red - green) << lshift) / lum;
                                byptr[i] = (3*(blue - yellow) << lshift) / lum;
                        }
                        else if (lshift < 0)
                        {
                                rgptr[i] = ((3*(red - green)) / lum) >> (-lshift);
                                byptr[i] = ((3*(blue - yellow)) / lum) >> (-lshift);
                        }
                        else // lshift == 0
                        {
                                rgptr[i] = (3*(red - green)) / lum;
                                byptr[i] = (3*(blue - yellow)) / lum;
                        }
                }
        }
}

// ######################################################################
void env_merge_range(const struct env_image* src,
                     intg32* mi, intg32* ma)
{
        if (src == 0)
                return;

        const intg32* sptr = env_img_pixels(src);
        const env_size_t sz = env_img_size(src);

        if (sz == 0)
                return;

        if (sptr[0] < *mi) *mi = sptr[0];
        if (sptr[0] > *ma) *ma = sptr[0];

        for (env_size_t i = 1; i < sz; ++i)
        {
                if (sptr[i] < *mi) *mi = sptr[i];
                else if (sptr[i] > *ma) *ma = sptr[i];
        }
}

// ######################################################################
void env_rescale_range_inplace(struct env_image* src,
                               const intg32 mi, const intg32 ma)
{
        if (src == 0)
                return;

        intg32* sptr = env_img_pixelsw(src);
        const env_size_t sz = env_img_size(src);

        ENV_ASSERT(ma >= mi);

        const intg32 scale = ma - mi;

        if (scale < 1) // image is uniform
        {
                for (env_size_t i = 0; i < sz; ++i)
                        sptr[i] = 0;
        }
        else if ((INTG32_MAX / 255) > scale)
        {
                for (env_size_t i = 0; i < sz; ++i)
                {
                        sptr[i] = ((sptr[i] - mi) * 255) / scale;
                }
        }
        else
        {
                const intg32 div = scale / 255;
                ENV_ASSERT(div > 0);
                for (env_size_t i = 0; i < sz; ++i)
                {
                        sptr[i] = (sptr[i] - mi) / div;
                }
        }
}

#ifdef ENV_WITH_DYNAMIC_CHANNELS

// ######################################################################
void env_shift_clean(const struct env_image* srcImg,
                     const env_ssize_t dx, const env_ssize_t dy,
                     struct env_image* result)
{
        if (!env_img_initialized(srcImg))
        {
                env_img_make_empty(result);
                return;
        }

        ENV_ASSERT(env_dims_equal(result->dims, srcImg->dims));

        const env_ssize_t w = (env_ssize_t) srcImg->dims.w;
        const env_ssize_t h = (env_ssize_t) srcImg->dims.h;

        if (ENV_ABS(dx) >= w || ENV_ABS(dy) >= h)
                // the shifts are so large that the resulting image
                // will just be empty:
                return;

        // find range of pixels to copy:
        const env_ssize_t startx = ENV_MAX(((env_ssize_t) 0), -dx);
        const env_ssize_t endx = ENV_MIN(w, w - dx);
        ENV_ASSERT(startx < w);
        ENV_ASSERT(endx > 0);
        const env_ssize_t starty = ENV_MAX(((env_ssize_t) 0), -dy);
        const env_ssize_t endy = ENV_MIN(h, h - dy);
        ENV_ASSERT(starty < h);
        ENV_ASSERT(endy > 0);

        // create the source and destination pointers
        const intg32* src = env_img_pixels(srcImg);
        intg32* dst = env_img_pixelsw(result);

        src += startx + starty * w;
        dst += (startx + dx) + (starty + dy) * w;

        const env_ssize_t skip = w - endx + startx;

        // do the copy:
        for (env_ssize_t j = starty; j < endy; ++j)
        {
                for (env_ssize_t i = startx; i < endx; ++i)
                        *dst++ = *src++;

                // ready for next row of pixels:
                src += skip; dst += skip;
        }
}

// ######################################################################
void env_shift_image(const struct env_image* srcImg,
                     const env_ssize_t dxnumer, const env_ssize_t dynumer,
                     const env_size_t denombits,
                     struct env_image* result)
{
        if (!env_img_initialized(srcImg))
        {
                env_img_make_empty(result);
                return;
        }

        ENV_ASSERT(env_dims_equal(result->dims, srcImg->dims));

        ENV_ASSERT(denombits < 8*sizeof(intg32));

        const env_ssize_t denom = (1 << denombits);

        const env_ssize_t w = (env_ssize_t) srcImg->dims.w;
        const env_ssize_t h = (env_ssize_t) srcImg->dims.h;

        // prepare a couple of variable for the x direction
        env_ssize_t xt = dxnumer >= 0
                ? (dxnumer >> denombits)
                : - ((-dxnumer + denom-1) >> denombits);
        env_ssize_t xfrac_numer = dxnumer - (xt << denombits);
        const env_ssize_t startx = ENV_MAX(((env_ssize_t) 0),xt);
        const env_ssize_t endx = ENV_MIN(((env_ssize_t) 0),xt) + w - 1;

        // prepare a couple of variable for the y direction
        env_ssize_t yt = dynumer >= 0
                ? (dynumer >> denombits)
                : - ((-dynumer + denom-1) >> denombits);
        env_ssize_t yfrac_numer = dynumer - (yt << denombits);
        const env_ssize_t starty = ENV_MAX(((env_ssize_t) 0),yt);
        const env_ssize_t endy = ENV_MIN(((env_ssize_t) 0),yt) + h - 1;

        // clear the return image
        {
                const env_size_t sz = w * h;
                intg32* const rptr = env_img_pixelsw(result);
                for (env_size_t i = 0; i < sz; ++i)
                        rptr[i] = 0;
        }

        // dispatch to faster env_shift_clean() if displacements are
        // roughly integer:
        if (xfrac_numer == 0 && yfrac_numer == 0)
        {
                env_shift_clean(srcImg, xt, yt, result);
                return;
        }

        if (xfrac_numer > 0)
        {
                xfrac_numer = denom - xfrac_numer;
                ++xt;
        }

        if (yfrac_numer > 0)
        {
                yfrac_numer = denom - yfrac_numer;
                ++yt;
        }

        // prepare the pointers
        const intg32* src2 = env_img_pixels(srcImg);
        intg32* ret2 = env_img_pixelsw(result);
        if (xt > 0) ret2 += xt;
        else if (xt < 0) src2 -= xt;
        if (yt > 0) ret2 += yt * w;
        else if (yt < 0) src2 -= yt * w;

        // now loop over the images
        for (env_ssize_t y = starty; y < endy; ++y)
        {
                const intg32* src = src2;
                intg32* ret = ret2;
                for (env_ssize_t x = startx; x < endx; ++x)
                {
                        *ret = (((src[0] >> denombits) * (denom - xfrac_numer)) >> denombits) * (denom - yfrac_numer);
                        *ret += (((src[1] >> denombits) * xfrac_numer) >> denombits) * (denom - yfrac_numer);
                        *ret += (((src[w] >> denombits) * (denom - xfrac_numer)) >> denombits) * yfrac_numer;
                        *ret += (((src[w+1] >> denombits) * xfrac_numer) >> denombits) * yfrac_numer;
                        ++src; ++ret;
                }
                src2 += w; ret2 += w;
        }
}

#endif // ENV_WITH_DYNAMIC_CHANNELS

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* c-file-style: "linux" */
/* End: */

#endif // ENVISION_ENV_IMAGE_OPS_C_DEFINED

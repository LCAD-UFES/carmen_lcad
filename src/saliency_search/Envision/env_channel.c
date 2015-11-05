/*!@file Envision/env_channel.c Base class for channels that will use integer math */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_channel.c $
// $Id: env_channel.c 9830 2008-06-18 18:50:22Z lior $
//

#ifndef ENVISION_ENV_CHANNEL_C_DEFINED
#define ENVISION_ENV_CHANNEL_C_DEFINED

#include "Envision/env_channel.h"

#include "Envision/env_c_math_ops.h"
#include "Envision/env_image_ops.h"
#include "Envision/env_log.h"
#include "Envision/env_params.h"

#ifndef ENV_NO_DEBUG

//! Check whether the pyramid is dyadic.
/*! A dyadic pyramid is one in which each level is one half
  the width and one half the height of the preceding level. */
static int is_dyadic(const struct env_pyr* pyr,
                     const env_size_t first, const env_size_t last)
{
        if (first == last) return 0;

        for (env_size_t i = first + 1; i < last; ++i)
        {
                const struct env_dims prevdims = env_pyr_img(pyr, i-1)->dims;
                const struct env_dims curdims = env_pyr_img(pyr, i)->dims;

                // make sure we don't go below 1
                const env_size_t pw2 = ENV_MAX(prevdims.w/2,((env_size_t) 1));
                const env_size_t ph2 = ENV_MAX(prevdims.h/2,((env_size_t) 1));

                if (curdims.w != pw2) return 0;
                if (curdims.h != ph2) return 0;
        }

        return 1;
}

#endif

#ifdef ENV_WITH_DYNAMIC_CHANNELS

// ######################################################################
static void abs_diff_thresh(const struct env_image* b,
                            const struct env_image* c,
                            const intg32 thresh,
                            struct env_image* result)
{
        ENV_ASSERT(env_dims_equal(b->dims, c->dims));
        ENV_ASSERT(env_dims_equal(b->dims, result->dims));

        const intg32* const bptr = env_img_pixels(b);
        const intg32* const cptr = env_img_pixels(c);
        intg32* const dptr = env_img_pixelsw(result);

        const env_size_t sz = env_img_size(b);

        for (env_size_t i = 0; i < sz; ++i)
        {
                dptr[i] =
                        (bptr[i] < cptr[i])
                        ? cptr[i] - bptr[i]
                        : bptr[i] - cptr[i];

                if (dptr[i] < thresh) dptr[i] = 0;
        }
}

// ######################################################################
static void abs_diff_thresh_pyr(const struct env_pyr* b,
                                const struct env_pyr* c,
                                const intg32 thresh,
                                struct env_pyr* result)
{
        ENV_ASSERT(env_pyr_depth(b) == env_pyr_depth(c));
        ENV_ASSERT(env_pyr_depth(c) == env_pyr_depth(result));

        const env_size_t n = env_pyr_depth(b);

        if (env_pyr_depth(result) != n)
        {
                env_pyr_make_empty(result);
                env_pyr_init(result, n);
        }

        for (env_size_t i = 0; i < n; ++i)
        {
                const struct env_image* bimg = env_pyr_img(b, i);

                if (!env_img_initialized(bimg))
                        continue;

                // else...

                const struct env_image* cimg = env_pyr_img(c, i);
                struct env_image* rimg = env_pyr_imgw(result, i);

                env_img_resize_dims(rimg, bimg->dims);

                abs_diff_thresh(bimg, cimg, thresh, rimg);
        }
}

#endif // ENV_WITH_DYNAMIC_CHANNELS

// ######################################################################
void env_chan_process_pyr(const char* tagName,
                          const struct env_dims inputDims,
                          const struct env_pyr* pyr,
                          const struct env_params* envp,
                          const struct env_math* imath,
                          const int takeAbs,
                          const int normalizeOutput,
                          struct env_image* result)
{
        const struct env_dims mapDims =
                { ENV_MAX(inputDims.w / (1 << envp->output_map_level), 1),
                  ENV_MAX(inputDims.h / (1 << envp->output_map_level), 1) };

        if (env_pyr_depth(pyr) == 0)
                // OK, our pyramid wasn't ready to give us any output
                // yet, so just return an empty output image:
        {
                env_img_make_empty(result);
                return;
        }

        // We only want dyadic pyramids here:
        ENV_ASSERT(is_dyadic(pyr, envp->cs_lev_min, env_max_pyr_depth(envp)));

        env_img_resize_dims(result, mapDims);

        {
                const env_size_t mapSize = mapDims.w * mapDims.h;
                intg32* const rptr = env_img_pixelsw(result);
                for (env_size_t i = 0; i < mapSize; ++i)
                        rptr[i] = 0;
        }

        // compute max-normalized weighted sum of center-surround at all levels:
        for (env_size_t clev = envp->cs_lev_min; clev <= envp->cs_lev_max; ++clev)
                for (env_size_t delta = envp->cs_del_min; delta <= envp->cs_del_max; ++delta)
                {
                        const env_size_t slev = clev + delta;

                        // submap is computed from a center-surround difference:
                        struct env_image submap;
                        env_img_init(&submap, env_pyr_img(pyr, clev)->dims);
                        env_center_surround(env_pyr_img(pyr, clev),
                                            env_pyr_img(pyr, slev),
                                            takeAbs, &submap);
#ifdef ENV_WITH_VISIT_CHANNEL
                        if (envp->submapPreProc != 0)
                          (*envp->submapPreProc)(tagName, clev, slev, &submap,
                             env_pyr_img(pyr, clev), env_pyr_img(pyr, slev) );
#endif

                        // resize submap to fixed scale if necessary:
                        if (submap.dims.w > mapDims.w
                            || submap.dims.h > mapDims.h)
                        {
                                // how many levels to we need to
                                // downscale the current submap to get
                                // to the output map resolution?
                                const env_size_t n =
                                        envp->output_map_level - clev;

                                env_downsize_9_inplace(&submap, n, imath);
                        }
                        else if (submap.dims.w < mapDims.w
                                 || submap.dims.h < mapDims.h)
                        {
                                struct env_image tmp;
                                env_img_init(&tmp, mapDims);
                                env_rescale(&submap, &tmp);
                                env_img_swap(&submap, &tmp);
                        }

                        // make sure that the resizing came out
                        // precisely:
                        ENV_ASSERT(env_dims_equal(submap.dims, mapDims));

                        // first normalize the submap to a fixed
                        // dynamic range and then apply spatial
                        // competition for salience to the submap:
                        env_max_normalize_inplace
                                (&submap, INTMAXNORMMIN, INTMAXNORMMAX,
                                 envp->maxnorm_type,
                                 envp->range_thresh);

#ifdef ENV_WITH_VISIT_CHANNEL
                        if (envp->submapPostNormProc != 0)
                          (*envp->submapPostNormProc)(tagName, clev, slev, &submap,
                             env_pyr_img(pyr, clev), env_pyr_img(pyr, slev) );
#endif

                        // add submap to our sum
                        env_c_image_div_scalar_accum
                                (env_img_pixels(&submap),
                                 env_img_size(&submap),
                                 (intg32) env_max_cs_index(envp),
                                 env_img_pixelsw(result));

                        env_img_make_empty(&submap);
                }

#ifdef ENV_WITH_VISIT_CHANNEL
        if (envp->submapPostProc != 0)
          (*envp->submapPostProc)(tagName, result);
#endif

        // apply max-normalization on the result as needed:
        if (normalizeOutput)
                env_max_normalize_inplace
                        (result, INTMAXNORMMIN, INTMAXNORMMAX,
                         envp->maxnorm_type,
                         envp->range_thresh);
}

// ######################################################################
void env_chan_intensity(const char* tagName,
                        const struct env_params* envp,
                        const struct env_math* imath,
                        const struct env_dims inputdims,
                        const struct env_pyr* lowpass5,
                        const int normalizeOutput,
                        env_chan_status_func* status_func,
                        void* status_userdata,
                        struct env_image* result)
{
        env_chan_process_pyr(tagName, inputdims, lowpass5,
                             envp,
                             imath,
                             1, // takeAbs
                             normalizeOutput,
                             result);

        if (status_func)
                (*status_func)(status_userdata, tagName, result);
}

// ######################################################################
void env_chan_color(const char* tagName,
                    const struct env_params* envp,
                    const struct env_math* imath,
                    const struct env_rgb_pixel* const colimg,
                    const struct env_rgb_pixel* const prev_colimg,
                    const struct env_dims dims,
                    env_chan_status_func* status_func,
                    void* status_userdata,
                    struct env_image* result)
{
        struct env_image rg; env_img_init(&rg, dims);
        struct env_image by; env_img_init(&by, dims);

        const intg32 lumthresh = (3*255) / 10;
        env_get_rgby(colimg, prev_colimg, dims.w * dims.h,
                     &rg, &by, lumthresh, imath->nbits);

        const env_size_t firstlevel = envp->cs_lev_min;
        const env_size_t depth = env_max_pyr_depth(envp);

        {
                struct env_pyr rgpyr;
                env_pyr_init(&rgpyr, depth);
                env_pyr_build_lowpass_5(&rg, firstlevel, imath, &rgpyr);

                env_chan_intensity("red/green", envp, imath,
                                   rg.dims, &rgpyr, 0,
                                   status_func, status_userdata, result);

                env_pyr_make_empty(&rgpyr);
        }

        struct env_image byOut = env_img_initializer;

        {
                struct env_pyr bypyr;
                env_pyr_init(&bypyr, depth);
                env_pyr_build_lowpass_5(&by, firstlevel, imath, &bypyr);

                env_chan_intensity("blue/yellow", envp, imath,
                                   by.dims, &bypyr, 0,
                                   status_func, status_userdata, &byOut);
                env_pyr_make_empty(&bypyr);
        }

        env_img_make_empty(&rg);
        env_img_make_empty(&by);

        const intg32* const byptr = env_img_pixels(&byOut);
        intg32* const dptr = env_img_pixelsw(result);
        const env_size_t sz = env_img_size(result);

        for (env_size_t i = 0; i < sz; ++i)
                dptr[i] = (dptr[i] / 2) + (byptr[i] / 2);

        env_max_normalize_inplace(result, INTMAXNORMMIN, INTMAXNORMMAX,
                                  envp->maxnorm_type,
                                  envp->range_thresh);
        if (status_func)
                (*status_func)(status_userdata, tagName, result);

        env_img_make_empty(&byOut);
}

// ######################################################################
void env_chan_steerable(const char* tagName,
                        const struct env_params* envp,
                        const struct env_math* imath,
                        const struct env_dims inputdims,
                        const struct env_pyr* hipass9,
                        const env_size_t thetaidx,
                        env_chan_status_func* status_func,
                        void* status_userdata,
                        struct env_image* result)
{
        const env_size_t kdenombits = ENV_TRIG_NBITS;

        // spatial_freq = 2.6 / (2*pi) ~= 0.41380285203892792 ~= 2069/5000

        const intg32 sfnumer = 2069;
        const intg32 sfdenom = 5000;

        const intg32 kxnumer = ((intg32) (sfnumer * imath->costab[thetaidx] * ENV_TRIG_TABSIZ)) / sfdenom;
        const intg32 kynumer = ((intg32) (sfnumer * imath->sintab[thetaidx] * ENV_TRIG_TABSIZ)) / sfdenom;

        // Compute our pyramid:
        struct env_pyr pyr = env_pyr_initializer;
        env_pyr_build_steerable_from_hipass_9(hipass9,
                                              kxnumer, kynumer, kdenombits,
                                              imath,
                                              &pyr);

        env_chan_process_pyr(tagName, inputdims, &pyr,
                             envp,
                             imath,
                             0, // takeAbs
                             1, // normalizeOutput
                             result);

        if (status_func)
                (*status_func)(status_userdata, tagName, result);

        env_pyr_make_empty(&pyr);
}

// ######################################################################
void env_chan_orientation(const char* tagName,
                          const struct env_params* envp,
                          const struct env_math* imath,
                          const struct env_image* img,
                          env_chan_status_func* status_func,
                          void* status_userdata,
                          struct env_image* result)
{
        env_img_make_empty(result);

        if (envp->num_orientations == 0)
                return;

        struct env_pyr hipass9;
        env_pyr_init(&hipass9, env_max_pyr_depth(envp));
        env_pyr_build_hipass_9(img,
                               envp->cs_lev_min,
                               imath,
                               &hipass9);

        struct env_image chanOut = env_img_initializer;

        char buf[17] =
                {
                        's', 't', 'e', 'e', 'r', 'a', 'b', 'l', 'e', // 0--8
                        '(', '_', '_', // 9--11
                        '/', '_', '_', ')', '\0' // 12--16
                };

        ENV_ASSERT(envp->num_orientations <= 99);

        buf[13] = '0' + (envp->num_orientations / 10);
        buf[14] = '0' + (envp->num_orientations % 10);

        for (env_size_t i = 0; i < envp->num_orientations; ++i)
        {
                // theta = (180.0 * i) / envp->num_orientations +
                // 90.0, where ENV_TRIG_TABSIZ is equivalent to 360.0
                // or 2*pi
                const env_size_t thetaidx =
                        (ENV_TRIG_TABSIZ * i)
                        / (2 * envp->num_orientations)
                        + (ENV_TRIG_TABSIZ / 4);

                ENV_ASSERT(thetaidx < ENV_TRIG_TABSIZ);

                buf[10] = '0' + ((i+1) / 10);
                buf[11] = '0' + ((i+1) % 10);

                env_chan_steerable
                        (buf, envp, imath, img->dims,
                         &hipass9, thetaidx,
                         status_func, status_userdata, &chanOut);

                ENV_ASSERT(env_img_initialized(&chanOut));

                if (!env_img_initialized(result))
                {
                        env_img_resize_dims(result, chanOut.dims);
                        env_c_image_div_scalar
                                (env_img_pixels(&chanOut),
                                 env_img_size(&chanOut),
                                 (intg32) envp->num_orientations,
                                 env_img_pixelsw(result));
                }
                else
                {
                        ENV_ASSERT(env_dims_equal(chanOut.dims,
                                                  result->dims));
                        env_c_image_div_scalar_accum
                                (env_img_pixels(&chanOut),
                                 env_img_size(&chanOut),
                                 (intg32) envp->num_orientations,
                                 env_img_pixelsw(result));
                }
        }

        env_img_make_empty(&chanOut);
        env_pyr_make_empty(&hipass9);

        ENV_ASSERT(env_img_initialized(result));

        env_max_normalize_inplace(result, INTMAXNORMMIN, INTMAXNORMMAX,
                                  envp->maxnorm_type,
                                  envp->range_thresh);

        if (status_func)
                (*status_func)(status_userdata, tagName, result);
}

#ifdef ENV_WITH_DYNAMIC_CHANNELS

// ######################################################################
void env_chan_flicker(const char* tagName,
                      const struct env_params* envp,
                      const struct env_math* imath,
                      const struct env_image* prev,
                      const struct env_image* cur,
                      env_chan_status_func* status_func,
                      void* status_userdata,
                      struct env_image* result)
{
        // If this is the first time the flicker channel has seen input,
        // then prev will be uninitialized; obviously we can't compute any
        // flicker with only one frame, so we just store the current input
        // as the next iteration's previous input
        if (!env_img_initialized(prev))
        {
                env_img_make_empty(result);
        }
        else
        {
                const intg32 lowthresh =
                        (envp->scale_bits > 8)
                        ? (envp->flicker_thresh << (envp->scale_bits - 8))
                        : (envp->flicker_thresh >> (8 - envp->scale_bits));

                // take thresholded abs difference between current and
                // previous frame:
                struct env_image fli;
                env_img_init(&fli, prev->dims);
                abs_diff_thresh(cur, prev, lowthresh, &fli);

                const env_size_t firstlevel = envp->cs_lev_min;
                const env_size_t depth = env_max_pyr_depth(envp);

                // Compute our pyramid:
                struct env_pyr pyr;
                env_pyr_init(&pyr, depth);
                env_pyr_build_lowpass_5(&fli, firstlevel, imath, &pyr);

                env_chan_process_pyr(tagName, fli.dims, &pyr,
                                     envp,
                                     imath,
                                     1, // takeAbs
                                     1, // normalizeOutput
                                     result);

                if (status_func)
                        (*status_func)(status_userdata, tagName, result);

                env_img_make_empty(&fli);
                env_pyr_make_empty(&pyr);
        }
}

// ######################################################################
void env_chan_msflicker(const char* tagName,
                        const struct env_params* envp,
                        const struct env_math* imath,
                        const struct env_dims inputDims,
                        const struct env_pyr* prev_lowpass5,
                        const struct env_pyr* cur_lowpass5,
                        env_chan_status_func* status_func,
                        void* status_userdata,
                        struct env_image* result)
{
        // If this is the first time the flicker channel has seen
        // input, then prev will be uninitialized; obviously we can't
        // compute any flicker with only one frame, so we just store
        // the current input as the next iteration's previous input
        if (env_pyr_depth(prev_lowpass5) == 0)
        {
                env_img_make_empty(result);
        }
        else
        {
                const intg32 lowthresh =
                        (envp->scale_bits > 8)
                        ? (envp->flicker_thresh << (envp->scale_bits - 8))
                        : (envp->flicker_thresh >> (8 - envp->scale_bits));

                // take thresholded abs difference between current and
                // previous frame:
                struct env_pyr fli;
                env_pyr_init(&fli, env_pyr_depth(cur_lowpass5));
                abs_diff_thresh_pyr(cur_lowpass5, prev_lowpass5,
                                    lowthresh, &fli);

                env_chan_process_pyr(tagName, inputDims, &fli,
                                     envp,
                                     imath,
                                     1, // takeAbs
                                     1, // normalizeOutput
                                     result);

                if (status_func)
                        (*status_func)(status_userdata, tagName, result);

                env_pyr_make_empty(&fli);
        }
}

// ######################################################################
void env_chan_direction(const char* tagName,
                        const struct env_params* envp,
                        const struct env_math* imath,
                        const struct env_dims inputdims,
                        const struct env_pyr* unshiftedPrev,
                        const struct env_pyr* unshiftedCur,
                        const struct env_pyr* shiftedPrev,
                        const struct env_pyr* shiftedCur,
                        env_chan_status_func* status_func,
                        void* status_userdata,
                        struct env_image* result)
{
        const env_size_t firstlevel = envp->cs_lev_min;
        const env_size_t depth = env_max_pyr_depth(envp);

        const env_size_t nshift = (imath->nbits+1)/2;

        if (env_pyr_depth(unshiftedPrev) == 0)
        {
                // it's our first time, so just return an empty image:
                env_img_make_empty(result);
        }
        else
        {
                struct env_pyr pyr;
                env_pyr_init(&pyr, depth);

                const intg32 lowthresh =
                        (envp->scale_bits > 8)
                        ? (envp->motion_thresh << (envp->scale_bits - 8))
                        : (envp->motion_thresh >> (8 - envp->scale_bits));

                // compute the Reichardt maps
                for (env_size_t i = firstlevel; i < depth; i++)
                {
                        env_img_resize_dims
                                (env_pyr_imgw(&pyr, i),
                                 env_pyr_img(unshiftedCur, i)->dims);

                        const intg32* const ucurr = env_img_pixels(env_pyr_img(unshiftedCur, i));
                        const intg32* const uprev = env_img_pixels(env_pyr_img(unshiftedPrev, i));
                        const intg32* const scurr = env_img_pixels(env_pyr_img(shiftedCur, i));
                        const intg32* const sprev = env_img_pixels(env_pyr_img(shiftedPrev, i));
                        intg32* const dptr = env_img_pixelsw(env_pyr_imgw(&pyr, i));

                        const env_size_t sz = env_img_size(env_pyr_img(&pyr, i));

                        for (env_size_t c = 0; c < sz; ++c)
                        {
                                dptr[c] =
                                        ((ucurr[c] >> nshift) * (sprev[c] >> nshift)) -
                                        ((uprev[c] >> nshift) * (scurr[c] >> nshift));

                                if (dptr[c] < lowthresh) dptr[c] = 0;
                        }
                }

                env_chan_process_pyr(tagName, inputdims, &pyr,
                                     envp,
                                     imath,
                                     1, // takeAbs
                                     1, // normalizeOutput
                                     result);

                if (status_func)
                        (*status_func)(status_userdata, tagName, result);

                env_pyr_make_empty(&pyr);
        }
}

#endif // ENV_WITH_DYNAMIC_CHANNELS

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* c-file-style: "linux" */
/* End: */

#endif // ENVISION_ENV_CHANNEL_C_DEFINED

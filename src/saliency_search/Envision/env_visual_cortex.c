/*!@file Envision/env_visual_cortex.c */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_visual_cortex.c $
// $Id: env_visual_cortex.c 8054 2007-03-07 00:47:08Z rjpeters $
//

#ifndef ENVISION_ENV_VISUAL_CORTEX_C_DEFINED
#define ENVISION_ENV_VISUAL_CORTEX_C_DEFINED

#include "Envision/env_visual_cortex.h"

#include "Envision/env_c_math_ops.h"
#include "Envision/env_channel.h"
#include "Envision/env_image_ops.h"
#include "Envision/env_log.h"
#include "Envision/env_params.h"

#define WEIGHT_SCALEBITS ((env_size_t) 8)

static void combine_output(struct env_image* chanOut,
                           const intg32 iweight,
                           struct env_image* result)
{
        if (!env_img_initialized(chanOut))
                return;

        intg32* const sptr = env_img_pixelsw(chanOut);
        const env_size_t sz = env_img_size(chanOut);

        if (!env_img_initialized(result))
        {
                env_img_resize_dims(result, chanOut->dims);
                intg32* const dptr = env_img_pixelsw(result);
                for (env_size_t i = 0; i < sz; ++i)
                {
                        sptr[i] = (sptr[i] >> WEIGHT_SCALEBITS) * iweight;
                        dptr[i] = sptr[i];
                }
        }
        else
        {
                ENV_ASSERT(env_dims_equal(chanOut->dims, result->dims));
                intg32* const dptr = env_img_pixelsw(result);
                const env_size_t sz = env_img_size(result);
                for (env_size_t i = 0; i < sz; ++i)
                {
                        sptr[i] = (sptr[i] >> WEIGHT_SCALEBITS) * iweight;
                        dptr[i] += sptr[i];
                }
        }
}

// ######################################################################
void env_visual_cortex_init(struct env_visual_cortex* vcx,
                            const struct env_params* envp)
{
        env_params_validate(envp);

        env_init_integer_math(&vcx->imath, envp);

#ifdef ENV_WITH_DYNAMIC_CHANNELS
        env_img_init_empty(&vcx->prev_input);
        env_pyr_init(&vcx->prev_lowpass5, 0);
#endif

#ifdef ENV_WITH_DYNAMIC_CHANNELS
        env_motion_channel_init(&vcx->motion_chan, envp);
#endif

}

// ######################################################################
void env_visual_cortex_destroy(struct env_visual_cortex* vcx)
{
#ifdef ENV_WITH_DYNAMIC_CHANNELS
        env_img_make_empty(&vcx->prev_input);
        env_pyr_make_empty(&vcx->prev_lowpass5);
        env_motion_channel_destroy(&vcx->motion_chan);
#endif
}

// ######################################################################
void env_visual_cortex_input(
        struct env_visual_cortex* vcx,
        const struct env_params* envp,
        const char* tagName,
        const struct env_rgb_pixel* const colimg,
        const struct env_rgb_pixel* const prev_colimg,
        const struct env_dims dims,
        env_chan_status_func* status_func,
        void* status_userdata,
        struct env_image* result,
        struct env_image* intens_result,
        struct env_image* color_result,
        struct env_image* ori_result
#ifdef ENV_WITH_DYNAMIC_CHANNELS
        ,
        struct env_image* flicker_result,
        struct env_image* motion_result
#endif
        )
{
        env_img_make_empty(result);

        const intg32 total_weight = env_total_weight(envp);

        ENV_ASSERT(total_weight > 0);

        /* We want to compute

         *                 weight
         *        img * ------------
         *              total_weight
         *
         *
         *        To do that without overflowing, we compute it as
         *
         *
         *                 weight      256
         *        img * ------------ * ---
         *              total_weight   256
         *
         *            img       weight * 256
         *        = ( --- ) * ( ------------ )
         *            256       total_weight
         *
         * where 256 is an example of (1<<WEIGHT_SCALEBITS) for
         * WEIGHT_SCALEBITS=8.
         */

        if (envp->chan_c_weight > 0)
        {
                const intg32 color_weight =
                        envp->chan_c_weight*(1<<WEIGHT_SCALEBITS) / total_weight;

                struct env_image colorOut = env_img_initializer;
                env_chan_color
                        ("color", envp, &vcx->imath, colimg, prev_colimg,
                         dims, status_func, status_userdata, &colorOut);
                combine_output(&colorOut, color_weight, result);
                if (color_result != 0)
                        env_img_swap(&colorOut, color_result);
                env_img_make_empty(&colorOut);
        }

        // don't compute the luminance image and luminance lowpass5
        // pyramid until AFTER we've done the color channel, so that
        // we minimize the number of simultaneous temporary images at
        // the highest resolution

        struct env_image bwimg;
        env_img_init(&bwimg, dims);
        env_c_luminance_from_byte(colimg, dims.w * dims.h,
                                  vcx->imath.nbits, env_img_pixelsw(&bwimg));

        struct env_pyr lowpass5;
        env_pyr_init(&lowpass5, env_max_pyr_depth(envp));
        env_pyr_build_lowpass_5(&bwimg,
                                envp->cs_lev_min,
                                &vcx->imath,
                                &lowpass5);

        if (envp->chan_i_weight > 0)
        {
                const intg32 intensity_weight =
                        envp->chan_i_weight*(1<<WEIGHT_SCALEBITS) / total_weight;

                struct env_image intensityOut = env_img_initializer;
                env_chan_intensity
                        ("intensity", envp, &vcx->imath, bwimg.dims,
                         &lowpass5, 1, status_func, status_userdata,
                         &intensityOut);
                combine_output(&intensityOut, intensity_weight, result);
                if (intens_result != 0)
                        env_img_swap(&intensityOut, intens_result);
                env_img_make_empty(&intensityOut);
        }

        if (envp->chan_o_weight > 0)
        {
                const intg32 orientation_weight =
                        envp->chan_o_weight*(1<<WEIGHT_SCALEBITS) / total_weight;

                struct env_image orientationOut = env_img_initializer;
                env_chan_orientation
                        ("orientation", envp, &vcx->imath,
                         &bwimg, status_func, status_userdata,
                         &orientationOut);
                combine_output(&orientationOut, orientation_weight, result);
                if (ori_result != 0)
                        env_img_swap(&orientationOut, ori_result);
                env_img_make_empty(&orientationOut);
        }

#ifdef ENV_WITH_DYNAMIC_CHANNELS

        if (envp->chan_f_weight > 0)
        {
                const intg32 flicker_weight =
                        envp->chan_f_weight*(1<<WEIGHT_SCALEBITS) / total_weight;

                struct env_image flickerOut = env_img_initializer;
                if (envp->multiscale_flicker)
                        env_chan_msflicker
                                ("flicker", envp, &vcx->imath,
                                 bwimg.dims,
                                 &vcx->prev_lowpass5, &lowpass5,
                                 status_func, status_userdata,
                                 &flickerOut);
                else
                        env_chan_flicker
                                ("flicker", envp, &vcx->imath,
                                 &vcx->prev_input, &bwimg,
                                 status_func, status_userdata,
                                 &flickerOut);
                combine_output(&flickerOut, flicker_weight, result);
                if (flicker_result != 0)
                        env_img_swap(&flickerOut, flicker_result);
                env_img_make_empty(&flickerOut);

                if (envp->multiscale_flicker)
                        env_pyr_copy_src_dst
                                (&lowpass5, &vcx->prev_lowpass5);
                else
                        env_pyr_make_empty(&vcx->prev_lowpass5);
        }

        if (envp->chan_m_weight > 0)
        {
                const intg32 motion_weight =
                        envp->chan_m_weight*(1<<WEIGHT_SCALEBITS) / total_weight;

                struct env_image motionOut = env_img_initializer;
                env_motion_channel_input_and_consume_pyr
                        (&vcx->motion_chan,
                         "motion", envp, &vcx->imath, bwimg.dims,
                         &lowpass5, status_func, status_userdata,
                         &motionOut);
                combine_output(&motionOut, motion_weight, result);
                if (motion_result != 0)
                        env_img_swap(&motionOut, motion_result);
                env_img_make_empty(&motionOut);
        }

        if (!envp->multiscale_flicker)
                env_img_swap(&vcx->prev_input, &bwimg);
        else
                env_img_make_empty(&vcx->prev_input);

#endif

        if (status_func)
                (*status_func)(status_userdata, tagName, result);

        env_pyr_make_empty(&lowpass5);
        env_img_make_empty(&bwimg);
}

// ######################################################################
void env_visual_cortex_merge_ranges(
        const struct env_image* intens_result,
        const struct env_image* color_result,
        const struct env_image* ori_result,
#ifdef ENV_WITH_DYNAMIC_CHANNELS
        const struct env_image* flicker_result,
        const struct env_image* motion_result,
#endif
        intg32* mi,
        intg32* ma
        )
{
        env_merge_range(intens_result, mi, ma);
        env_merge_range(color_result, mi, ma);
        env_merge_range(ori_result, mi, ma);
#ifdef ENV_WITH_DYNAMIC_CHANNELS
        env_merge_range(flicker_result, mi, ma);
        env_merge_range(motion_result, mi, ma);
#endif
}

// ######################################################################
void env_visual_cortex_rescale_ranges(
        struct env_image* result,
        struct env_image* intens_result,
        struct env_image* color_result,
        struct env_image* ori_result
#ifdef ENV_WITH_DYNAMIC_CHANNELS
        ,
        struct env_image* flicker_result,
        struct env_image* motion_result
#endif
        )
{
        intg32 mi = INTG32_MAX;
        intg32 ma = INTG32_MIN;

        // first rescale the overall output on its own:
        env_merge_range(result, &mi, &ma);
        env_rescale_range_inplace(result, mi, ma);

        // now compute a single min/max range covering ALL of the
        // individual channel outputs, and rescale the each of the
        // outputs using that same range:
        mi = INTG32_MAX;
        ma = INTG32_MIN;

        env_visual_cortex_merge_ranges(intens_result,
                                       color_result,
                                       ori_result,
#ifdef ENV_WITH_DYNAMIC_CHANNELS
                                       flicker_result,
                                       motion_result,
#endif
                                       &mi, &ma);

        env_rescale_range_inplace(intens_result, mi, ma);
        env_rescale_range_inplace(color_result, mi, ma);
        env_rescale_range_inplace(ori_result, mi, ma);
#ifdef ENV_WITH_DYNAMIC_CHANNELS
        env_rescale_range_inplace(flicker_result, mi, ma);
        env_rescale_range_inplace(motion_result, mi, ma);
#endif

}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* c-file-style: "linux" */
/* End: */

#endif // ENVISION_ENV_VISUAL_CORTEX_C_DEFINED

/*!@file Envision/env_mt_visual_cortex.c */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_mt_visual_cortex.c $
// $Id: env_mt_visual_cortex.c 10990 2009-03-06 02:44:38Z itti $
//

#ifndef ENVISION_ENV_MT_VISUAL_CORTEX_C_DEFINED
#define ENVISION_ENV_MT_VISUAL_CORTEX_C_DEFINED

#include "Envision/env_mt_visual_cortex.h"

#include "Envision/env_c_math_ops.h"
#include "Envision/env_channel.h"
#include "Envision/env_image_ops.h"
#include "Envision/env_job_server.h"
#include "Envision/env_log.h"
#include "Envision/env_mt_channel.h"
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

struct env_color_job_data
{
        const struct env_params* envp;
        const struct env_math* imath;
        const struct env_rgb_pixel* const colimg;
        const struct env_rgb_pixel* const prev_colimg /* or null is fine here */;
        const struct env_dims dims;
        env_chan_status_func* status_func;
        void* status_userdata;

        struct env_image colorOut;
};

static void env_color_job_run(void* p)
{
        struct env_color_job_data* j = (struct env_color_job_data*)(p);
        env_chan_color
                ("color", j->envp, j->imath, j->colimg, j->prev_colimg,
                 j->dims, j->status_func, j->status_userdata,
                 &j->colorOut);
}

struct env_intens_job_data
{
        const struct env_params* envp;
        const struct env_math* imath;
        const struct env_dims dims;
        const struct env_pyr* lowpass5;
        const int normalizeOutput;
        env_chan_status_func* status_func;
        void* status_userdata;

        struct env_image intensityOut;
};

static void env_intens_job_run(void* p)
{
        struct env_intens_job_data* j = (struct env_intens_job_data*)(p);
        env_chan_intensity
                ("intensity", j->envp, j->imath, j->dims,
                 j->lowpass5, j->normalizeOutput,
                 j->status_func, j->status_userdata,
                 &j->intensityOut);
}

struct env_ori_job_data
{
        const struct env_params* envp;
        const struct env_math* imath;
        const struct env_image* bwimg;
        env_chan_status_func* status_func;
        void* status_userdata;

        struct env_image orientationOut;
};

static void env_ori_job_run(void* p)
{
        struct env_ori_job_data* j = (struct env_ori_job_data*)(p);
        env_mt_chan_orientation
                ("orientation", j->envp, j->imath, j->bwimg,
                 j->status_func, j->status_userdata,
                 &j->orientationOut);
}

struct env_flicker_job_data
{
        const struct env_params* envp;
        const struct env_math* imath;
        const struct env_dims dims;
        const struct env_pyr* prev_lowpass5;
        const struct env_pyr* lowpass5;
        const struct env_image* prev_bwimg;
        const struct env_image* bwimg;
        env_chan_status_func* status_func;
        void* status_userdata;

        struct env_image flickerOut;
};

static void env_flicker_job_run(void* p)
{
        struct env_flicker_job_data* j = (struct env_flicker_job_data*)(p);
        if (j->envp->multiscale_flicker)
                env_chan_msflicker
                        ("flicker", j->envp, j->imath, j->dims,
                         j->prev_lowpass5, j->lowpass5,
                         j->status_func, j->status_userdata,
                         &j->flickerOut);
        else
                env_chan_flicker
                        ("flicker", j->envp, j->imath,
                         j->prev_bwimg, j->bwimg,
                         j->status_func, j->status_userdata,
                         &j->flickerOut);
}

struct env_motion_job_data
{
        struct env_motion_channel* motion_chan;
        const struct env_params* envp;
        const struct env_math* imath;
        const struct env_dims dims;
        struct env_pyr lowpass5;
        env_chan_status_func* status_func;
        void* status_userdata;

        struct env_image motionOut;
};

static void env_motion_job_run(void* p)
{
        struct env_motion_job_data* j = (struct env_motion_job_data*)(p);
        env_mt_motion_channel_input_and_consume_pyr
                (j->motion_chan,
                 "motion", j->envp, j->imath, j->dims, &j->lowpass5,
                 j->status_func, j->status_userdata,
                 &j->motionOut);
}

// ######################################################################
void env_mt_visual_cortex_input(
        const int do_multithreaded,
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
        if (!do_multithreaded)
        {
                env_visual_cortex_input(
                        vcx, envp, tagName, colimg, prev_colimg, dims,
                        status_func, status_userdata,
                        result,
                        intens_result, color_result, ori_result
#ifdef ENV_WITH_DYNAMIC_CHANNELS
                        , flicker_result, motion_result
#endif
                        );
                return;
        }

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

        struct env_job jobs[5];
        env_size_t njobs = 0;

        struct env_color_job_data c_data =
                {
                        envp,
                        &vcx->imath,
                        colimg,
                        prev_colimg,
                        dims,
                        status_func,
                        status_userdata,
                        env_img_initializer
                };

        if (envp->chan_c_weight > 0)
        {
                jobs[njobs].callback = &env_color_job_run;
                jobs[njobs].userdata = &c_data;
                ++njobs;
        }

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

        struct env_intens_job_data i_data =
                {
                        envp,
                        &vcx->imath,
                        bwimg.dims,
                        &lowpass5,
                        1,
                        status_func,
                        status_userdata,
                        env_img_initializer
                };

        if (envp->chan_i_weight > 0)
        {
                jobs[njobs].callback = &env_intens_job_run;
                jobs[njobs].userdata = &i_data;
                ++njobs;
        }

        struct env_ori_job_data o_data =
                {
                        envp,
                        &vcx->imath,
                        &bwimg,
                        status_func,
                        status_userdata,
                        env_img_initializer
                };

        if (envp->chan_o_weight > 0)
        {
                jobs[njobs].callback = &env_ori_job_run;
                jobs[njobs].userdata = &o_data;
                ++njobs;
        }

        struct env_flicker_job_data f_data =
                {
                        envp,
                        &vcx->imath,
                        bwimg.dims,
                        &vcx->prev_lowpass5,
                        &lowpass5,
                        &vcx->prev_input,
                        &bwimg,
                        status_func,
                        status_userdata,
                        env_img_initializer
                };

        if (envp->chan_f_weight > 0)
        {
                jobs[njobs].callback = &env_flicker_job_run;
                jobs[njobs].userdata = &f_data;
                ++njobs;
        }

        struct env_motion_job_data m_data =
                {
                        &vcx->motion_chan,
                        envp,
                        &vcx->imath,
                        bwimg.dims,
                        env_pyr_initializer,
                        status_func,
                        status_userdata,
                        env_img_initializer
                };

        if (envp->chan_m_weight > 0)
        {
                env_pyr_copy_src_dst(&lowpass5, &m_data.lowpass5);
                jobs[njobs].callback = &env_motion_job_run;
                jobs[njobs].userdata = &m_data;
                ++njobs;
        }

        env_run_jobs(&jobs[0], njobs);

        if (env_img_initialized(&c_data.colorOut))
        {
                const intg32 color_weight =
                        envp->chan_c_weight*(1<<WEIGHT_SCALEBITS) / total_weight;

                combine_output(&c_data.colorOut, color_weight, result);
                if (color_result != 0)
                        env_img_swap(&c_data.colorOut, color_result);
                env_img_make_empty(&c_data.colorOut);
        }

        if (env_img_initialized(&i_data.intensityOut))
        {
                const intg32 intensity_weight =
                        envp->chan_i_weight*(1<<WEIGHT_SCALEBITS) / total_weight;

                combine_output(&i_data.intensityOut,
                               intensity_weight, result);
                if (intens_result != 0)
                        env_img_swap(&i_data.intensityOut, intens_result);
                env_img_make_empty(&i_data.intensityOut);
        }

        if (env_img_initialized(&o_data.orientationOut))
        {
                const intg32 orientation_weight =
                        envp->chan_o_weight*(1<<WEIGHT_SCALEBITS) / total_weight;

                combine_output(&o_data.orientationOut,
                               orientation_weight, result);
                if (ori_result != 0)
                        env_img_swap(&o_data.orientationOut, ori_result);
                env_img_make_empty(&o_data.orientationOut);
        }

#ifdef ENV_WITH_DYNAMIC_CHANNELS

        if (env_img_initialized(&f_data.flickerOut))
        {
                const intg32 flicker_weight =
                        envp->chan_f_weight*(1<<WEIGHT_SCALEBITS) / total_weight;

                combine_output(&f_data.flickerOut, flicker_weight, result);
                if (flicker_result != 0)
                        env_img_swap(&f_data.flickerOut, flicker_result);
                env_img_make_empty(&f_data.flickerOut);
        }

        if (envp->chan_f_weight > 0)
        {
                if (envp->multiscale_flicker)
                        env_pyr_copy_src_dst
                                (&lowpass5, &vcx->prev_lowpass5);
                else
                        env_pyr_make_empty(&vcx->prev_lowpass5);
        }

        if (env_img_initialized(&m_data.motionOut))
        {
                const intg32 motion_weight =
                        envp->chan_m_weight*(1<<WEIGHT_SCALEBITS) / total_weight;

                combine_output(&m_data.motionOut, motion_weight, result);
                if (motion_result != 0)
                        env_img_swap(&m_data.motionOut, motion_result);
                env_img_make_empty(&m_data.motionOut);

                env_pyr_make_empty(&m_data.lowpass5);
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
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* c-file-style: "linux" */
/* End: */

#endif // ENVISION_ENV_MT_VISUAL_CORTEX_C_DEFINED

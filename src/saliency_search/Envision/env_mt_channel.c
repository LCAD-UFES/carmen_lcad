/*!@file Envision/env_mt_channel.c */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_mt_channel.c $
// $Id: env_mt_channel.c 9830 2008-06-18 18:50:22Z lior $
//

#ifndef ENVISION_ENV_MT_CHANNEL_C_DEFINED
#define ENVISION_ENV_MT_CHANNEL_C_DEFINED

#include "Envision/env_mt_channel.h"

#include "Envision/env_alloc.h"
#include "Envision/env_c_math_ops.h"
#include "Envision/env_image_ops.h"
#include "Envision/env_job_server.h"
#include "Envision/env_log.h"
#include "Envision/env_params.h"

// ######################################################################
struct env_ori_subjob_data
{
        const struct env_params* envp;
        const struct env_math* imath;
        struct env_dims dims;
        const struct env_pyr* hipass9;
        env_size_t thetaidx;
        env_chan_status_func* status_func;
        void* status_userdata;
        struct env_image chanOut;
        char channame[17];
};

static void env_ori_subjob_run(void* p)
{
        struct env_ori_subjob_data* j = (struct env_ori_subjob_data*)(p);

        env_chan_steerable
                (j->channame, j->envp, j->imath, j->dims,
                 j->hipass9, j->thetaidx,
                 j->status_func, j->status_userdata, &j->chanOut);

        ENV_ASSERT(env_img_initialized(&j->chanOut));
}

// ######################################################################
void env_mt_chan_orientation(const char* tagName,
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

        char buf[17] =
                {
                        's', 't', 'e', 'e', 'r', 'a', 'b', 'l', 'e', // 0--8
                        '(', '_', '_', // 9--11
                        '/', '_', '_', ')', '\0' // 12--16
                };

        ENV_ASSERT(envp->num_orientations <= 99);

        buf[13] = '0' + (envp->num_orientations / 10);
        buf[14] = '0' + (envp->num_orientations % 10);

        struct env_job* jobs =
                env_allocate(envp->num_orientations * sizeof(struct env_job));

        ENV_ASSERT2(jobs != 0, "env_allocate failed");

        struct env_ori_subjob_data* jobdata =
                env_allocate(envp->num_orientations
                             * sizeof(struct env_ori_subjob_data));

        ENV_ASSERT2(jobdata != 0, "env_allocate failed");

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

                jobdata[i].envp = envp;
                jobdata[i].imath = imath;
                jobdata[i].dims = img->dims;
                jobdata[i].hipass9 = &hipass9;
                jobdata[i].thetaidx = thetaidx;
                jobdata[i].status_func = status_func;
                jobdata[i].status_userdata = status_userdata;
                env_img_init_empty(&jobdata[i].chanOut);
                for (env_size_t c = 0; c < sizeof(buf); ++c)
                        jobdata[i].channame[c] = buf[c];

                jobs[i].callback = &env_ori_subjob_run;
                jobs[i].userdata = &jobdata[i];
        }

        env_run_jobs(&jobs[0], envp->num_orientations);

        for (env_size_t i = 0; i < envp->num_orientations; ++i)
        {
                struct env_image* chanOut = &jobdata[i].chanOut;

                if (!env_img_initialized(result))
                {
                        env_img_resize_dims(result, chanOut->dims);
                        env_c_image_div_scalar
                                (env_img_pixels(chanOut),
                                 env_img_size(chanOut),
                                 (intg32) envp->num_orientations,
                                 env_img_pixelsw(result));
                }
                else
                {
                        ENV_ASSERT(env_dims_equal(chanOut->dims,
                                                  result->dims));
                        env_c_image_div_scalar_accum
                                (env_img_pixels(chanOut),
                                 env_img_size(chanOut),
                                 (intg32) envp->num_orientations,
                                 env_img_pixelsw(result));
                }

                env_img_make_empty(chanOut);
        }

        env_pyr_make_empty(&hipass9);

        ENV_ASSERT(env_img_initialized(result));

        env_max_normalize_inplace(result, INTMAXNORMMIN, INTMAXNORMMAX,
                                  envp->maxnorm_type,
                                  envp->range_thresh);

        if (status_func)
                (*status_func)(status_userdata, tagName, result);

        env_deallocate(jobdata);
        env_deallocate(jobs);
}

// ######################################################################
struct env_direction_job_data
{
        const struct env_motion_channel* chan;
        env_size_t dir;
        const struct env_params* envp;
        const struct env_math* imath;
        struct env_dims inputdims;
        const struct env_pyr* unshiftedCur;
        env_chan_status_func* status_func;
        void* status_userdata;
        struct env_image chanOut;
        char channame[17];
};

static void env_direction_job_run(void* p)
{
        struct env_direction_job_data* j = (struct env_direction_job_data*)(p);

        const env_size_t firstlevel = j->envp->cs_lev_min;
        const env_size_t depth = env_max_pyr_depth(j->envp);

        // theta = (360.0 * i) / chan->num_directions;
        const env_size_t thetaidx =
                (j->dir * ENV_TRIG_TABSIZ) / j->chan->num_directions;

        ENV_ASSERT(thetaidx < ENV_TRIG_TABSIZ);

        // create an empty pyramid
        struct env_pyr shiftedCur;
        env_pyr_init(&shiftedCur, depth);

        // fill the empty pyramid with the shifted version
        for (env_size_t i = firstlevel; i < depth; ++i)
        {
                env_img_resize_dims(env_pyr_imgw(&shiftedCur, i),
                                    env_pyr_img(j->unshiftedCur, i)->dims);
                env_shift_image(env_pyr_img(j->unshiftedCur, i),
                                j->imath->costab[thetaidx],
                                -j->imath->sintab[thetaidx],
                                ENV_TRIG_NBITS,
                                env_pyr_imgw(&shiftedCur, i));
        }

        env_chan_direction(j->channame, j->envp, j->imath,
                           j->inputdims,
                           &j->chan->unshifted_prev, j->unshiftedCur,
                           &j->chan->shifted_prev[j->dir], &shiftedCur,
                           j->status_func, j->status_userdata, &j->chanOut);

        env_pyr_swap(&j->chan->shifted_prev[j->dir], &shiftedCur);
        env_pyr_make_empty(&shiftedCur);
}

// ######################################################################
void env_mt_motion_channel_input_and_consume_pyr(
        struct env_motion_channel* chan,
        const char* tagName,
        const struct env_params* envp,
        const struct env_math* imath,
        const struct env_dims inputdims,
        struct env_pyr* unshiftedCur,
        env_chan_status_func* status_func,
        void* status_userdata,
        struct env_image* result)
{
        env_img_make_empty(result);

        if (chan->num_directions != envp->num_motion_directions)
        {
                env_motion_channel_destroy(chan);
                env_motion_channel_init(chan, envp);
        }

        if (chan->num_directions == 0)
                return;

        char buf[17] =
                {
                        'r', 'e', 'i', 'c', 'h', 'a', 'r', 'd', 't', // 0--8
                        '(', '_', '_', // 9--11
                        '/', '_', '_', ')', '\0' // 12--16
                };

        ENV_ASSERT(chan->num_directions <= 99);

        buf[13] = '0' + (chan->num_directions / 10);
        buf[14] = '0' + (chan->num_directions % 10);

        struct env_job* jobs =
                env_allocate(chan->num_directions * sizeof(struct env_job));

        ENV_ASSERT2(jobs != 0, "env_allocate failed");

        struct env_direction_job_data* jobdata =
                env_allocate(chan->num_directions
                             * sizeof(struct env_direction_job_data));

        ENV_ASSERT2(jobdata != 0, "env_allocate failed");

        // compute Reichardt motion detection into several directions
        for (env_size_t dir = 0; dir < chan->num_directions; ++dir)
        {
                buf[10] = '0' + ((dir+1) / 10);
                buf[11] = '0' + ((dir+1) % 10);

                jobdata[dir].chan = chan;
                jobdata[dir].dir = dir;
                jobdata[dir].envp = envp;
                jobdata[dir].imath = imath;
                jobdata[dir].inputdims = inputdims;
                jobdata[dir].unshiftedCur = unshiftedCur;
                jobdata[dir].status_func = status_func;
                jobdata[dir].status_userdata = status_userdata;
                env_img_init_empty(&jobdata[dir].chanOut);
                for (env_size_t c = 0; c < sizeof(buf); ++c)
                        jobdata[dir].channame[c] = buf[c];

                jobs[dir].callback = &env_direction_job_run;
                jobs[dir].userdata = &jobdata[dir];
        }

        env_run_jobs(&jobs[0], chan->num_directions);

        for (env_size_t dir = 0; dir < chan->num_directions; ++dir)
        {
                struct env_image* chanOut = &jobdata[dir].chanOut;

                if (env_img_initialized(chanOut))
                {
                        if (!env_img_initialized(result))
                        {
                                env_img_resize_dims(result, chanOut->dims);
                                env_c_image_div_scalar
                                        (env_img_pixels(chanOut),
                                         env_img_size(chanOut),
                                         (intg32) chan->num_directions,
                                         env_img_pixelsw(result));
                        }
                        else
                        {
                                ENV_ASSERT
                                        (env_dims_equal(chanOut->dims,
                                                        result->dims));
                                env_c_image_div_scalar_accum
                                        (env_img_pixels(chanOut),
                                         env_img_size(chanOut),
                                         (intg32) chan->num_directions,
                                         env_img_pixelsw(result));
                        }
                }

                env_img_make_empty(chanOut);
        }

        if (env_img_initialized(result))
        {
                env_max_normalize_inplace(result,
                                          INTMAXNORMMIN, INTMAXNORMMAX,
                                          envp->maxnorm_type,
                                          envp->range_thresh);
                if (status_func)
                        (*status_func)(status_userdata, tagName, result);
        }

        env_pyr_swap(unshiftedCur, &chan->unshifted_prev);
        env_pyr_make_empty(unshiftedCur);

        env_deallocate(jobdata);
        env_deallocate(jobs);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* c-file-style: "linux" */
/* End: */

#endif // ENVISION_ENV_MT_CHANNEL_C_DEFINED

/*!@file Envision/env_motion_channel.c */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_motion_channel.c $
// $Id: env_motion_channel.c 9830 2008-06-18 18:50:22Z lior $
//

#ifndef ENVISION_ENV_MOTION_CHANNEL_C_DEFINED
#define ENVISION_ENV_MOTION_CHANNEL_C_DEFINED

#include "Envision/env_motion_channel.h"

#include "Envision/env_c_math_ops.h"
#include "Envision/env_channel.h"
#include "Envision/env_image_ops.h"
#include "Envision/env_log.h"
#include "Envision/env_params.h"

#ifdef ENV_WITH_DYNAMIC_CHANNELS

// ######################################################################
// env_motion_channel function definitions:
// ######################################################################

// ######################################################################
void env_motion_channel_init(struct env_motion_channel* chan,
                             const struct env_params* envp)
{
        env_pyr_init_empty(&chan->unshifted_prev);
        chan->num_directions = envp->num_motion_directions;
        chan->shifted_prev = (struct env_pyr*)
                env_allocate(envp->num_motion_directions
                             * sizeof(struct env_pyr));
        for (env_size_t i = 0; i < envp->num_motion_directions; ++i)
                env_pyr_init_empty(&chan->shifted_prev[i]);
}

// ######################################################################
void env_motion_channel_destroy(struct env_motion_channel* chan)
{
        env_pyr_make_empty(&chan->unshifted_prev);
        for (env_size_t i = 0; i < chan->num_directions; ++i)
                env_pyr_make_empty(&chan->shifted_prev[i]);
        env_deallocate(chan->shifted_prev);
        chan->num_directions = 0;
        chan->shifted_prev = 0;
}

// ######################################################################
void env_motion_channel_input_and_consume_pyr(
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

        const env_size_t firstlevel = envp->cs_lev_min;
        const env_size_t depth = env_max_pyr_depth(envp);

        struct env_image chanOut = env_img_initializer;

        char buf[17] =
                {
                        'r', 'e', 'i', 'c', 'h', 'a', 'r', 'd', 't', // 0--8
                        '(', '_', '_', // 9--11
                        '/', '_', '_', ')', '\0' // 12--16
                };

        ENV_ASSERT(chan->num_directions <= 99);

        buf[13] = '0' + (chan->num_directions / 10);
        buf[14] = '0' + (chan->num_directions % 10);

        // compute Reichardt motion detection into several directions
        for (env_size_t dir = 0; dir < chan->num_directions; ++dir)
        {
                // theta = (360.0 * i) / chan->num_directions;
                const env_size_t thetaidx =
                        (dir * ENV_TRIG_TABSIZ) / chan->num_directions;

                ENV_ASSERT(thetaidx < ENV_TRIG_TABSIZ);

                buf[10] = '0' + ((dir+1) / 10);
                buf[11] = '0' + ((dir+1) % 10);

                // create an empty pyramid
                struct env_pyr shiftedCur;
                env_pyr_init(&shiftedCur, depth);

                // fill the empty pyramid with the shifted version
                for (env_size_t i = firstlevel; i < depth; ++i)
                {
                        env_img_resize_dims(env_pyr_imgw(&shiftedCur, i),
                                            env_pyr_img(unshiftedCur, i)->dims);
                        env_shift_image(env_pyr_img(unshiftedCur, i),
                                        imath->costab[thetaidx],
                                        -imath->sintab[thetaidx],
                                        ENV_TRIG_NBITS,
                                        env_pyr_imgw(&shiftedCur, i));
                }

                env_chan_direction(buf, envp, imath,
                                   inputdims,
                                   &chan->unshifted_prev, unshiftedCur,
                                   &chan->shifted_prev[dir], &shiftedCur,
                                   status_func, status_userdata, &chanOut);

                env_pyr_swap(&chan->shifted_prev[dir], &shiftedCur);
                env_pyr_make_empty(&shiftedCur);

                if (env_img_initialized(&chanOut))
                {
                        if (!env_img_initialized(result))
                        {
                                env_img_resize_dims(result, chanOut.dims);
                                env_c_image_div_scalar
                                        (env_img_pixels(&chanOut),
                                         env_img_size(&chanOut),
                                         (intg32) chan->num_directions,
                                         env_img_pixelsw(result));
                        }
                        else
                        {
                                ENV_ASSERT
                                        (env_dims_equal(chanOut.dims,
                                                        result->dims));
                                env_c_image_div_scalar_accum
                                        (env_img_pixels(&chanOut),
                                         env_img_size(&chanOut),
                                         (intg32) chan->num_directions,
                                         env_img_pixelsw(result));
                        }
                }
        }

        env_img_make_empty(&chanOut);

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
}

#endif // ENV_WITH_DYNAMIC_CHANNELS

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* c-file-style: "linux" */
/* End: */

#endif // ENVISION_ENV_MOTION_CHANNEL_C_DEFINED

/*!@file Envision/envision.c */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/envision.c $
// $Id: envision.c 8341 2007-05-04 18:49:06Z rjpeters $
//

#ifndef ENVISION_APP_ENVISION_C_DEFINED
#define ENVISION_APP_ENVISION_C_DEFINED

#include "Envision/env_alloc.h"
#include "Envision/env_c_math_ops.h"
#include "Envision/env_image.h"
#include "Envision/env_image_ops.h"
#include "Envision/env_log.h"
#include "Envision/env_mt_visual_cortex.h"
#include "Envision/env_params.h"
#include "Envision/env_pthread_interface.h"
#include "Envision/env_stdio_interface.h"
#include "Envision/env_visual_cortex.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h> // for atoi(), malloc(), free()

// ######################################################################
// Thunk to convert from env_size_t to size_t
static void* malloc_thunk(env_size_t n)
{
        return malloc(n);
}

// ######################################################################
struct status_data
{
        int frame_number;
};

static void print_chan_status(void* userdata,
                              const char* tagName,
                              const struct env_image* img)
{
        struct status_data* p = (struct status_data*) userdata;

        if (env_img_initialized(img))
        {
                intg32 mi, ma;
                env_c_get_min_max(env_img_pixels(img), env_img_size(img),
                                  &mi, &ma);
                fprintf(stderr,
                        "frame %06d channel status: "
                        "%20s: range [%ld .. %ld]\n",
                        p->frame_number, tagName, (long) mi, (long) ma);
        }
}

// ######################################################################
int main(int argc, const char** argv)
{
        if (argc != 5 && argc != 6)
        {
                fprintf(stderr,
                        "usage: %s instem outstem firstframe lastframe ?multi-threaded?\n",
                        argv[0]);
                return 1;
        }

        const char* instem = argv[1];
        const char* outstem = argv[2];
        const int first = atoi(argv[3]);
        const int last = atoi(argv[4]);
        const int multithreaded = argc < 6 ? 0 : atoi(argv[5]);

        // Instantiate our various ModelComponents:
        struct env_params envp;
        env_params_set_defaults(&envp);

        envp.maxnorm_type = ENV_VCXNORM_MAXNORM;
        envp.scale_bits = 16;

        env_assert_set_handler(&env_stdio_assert_handler);
        if (multithreaded)
        {
                env_init_pthread_alloc();
                env_init_pthread_job_server();
        }
        env_allocation_init(&malloc_thunk, &free);

        {
                struct env_visual_cortex ivc;
                env_visual_cortex_init(&ivc, &envp);

                env_size_t npixels = 0;

                for (int c = first; c <= last; ++c)
                {
                        struct env_dims indims;
                        struct env_rgb_pixel* input = 0;

                        char fname[256];
                        snprintf(fname, sizeof(fname),
                                 "%s%06d.pnm", instem, c);
                        input = env_stdio_parse_rgb(fname, &indims);

                        npixels = indims.w * indims.h;

                        struct env_image ivcout = env_img_initializer;
                        struct env_image intens = env_img_initializer;
                        struct env_image color = env_img_initializer;
                        struct env_image ori = env_img_initializer;
#ifdef ENV_WITH_DYNAMIC_CHANNELS
                        struct env_image flicker = env_img_initializer;
                        struct env_image motion = env_img_initializer;
#endif

                        struct status_data userdata;
                        userdata.frame_number = c;

                        env_mt_visual_cortex_input(multithreaded,
                                                &ivc, &envp,
                                                "visualcortex",
                                                input, 0, indims,
                                                &print_chan_status,
                                                &userdata,
                                                &ivcout,
                                                &intens, &color, &ori
#ifdef ENV_WITH_DYNAMIC_CHANNELS
                                                , &flicker, &motion
#endif
                                );

                        env_deallocate(input);
                        input = 0;

                        env_visual_cortex_rescale_ranges(
                                &ivcout, &intens, &color, &ori
#ifdef ENV_WITH_DYNAMIC_CHANNELS
                                , &flicker, &motion
#endif
                                );

                        env_stdio_write_gray(&ivcout, outstem, "vcx", c);

                        env_stdio_write_gray(&intens, outstem, "intens", c);
                        env_stdio_write_gray(&color, outstem, "color", c);
                        env_stdio_write_gray(&ori, outstem, "ori", c);
#ifdef ENV_WITH_DYNAMIC_CHANNELS
                        env_stdio_write_gray(&flicker, outstem, "flicker", c);
                        env_stdio_write_gray(&motion, outstem, "motion", c);
#endif

                        env_img_make_empty(&ivcout);
                        env_img_make_empty(&intens);
                        env_img_make_empty(&color);
                        env_img_make_empty(&ori);
#ifdef ENV_WITH_DYNAMIC_CHANNELS
                        env_img_make_empty(&flicker);
                        env_img_make_empty(&motion);
#endif
                }

                struct env_alloc_stats stats;
                env_allocation_get_stats(&stats);
                env_stdio_print_alloc_stats(&stats, npixels ? npixels : 1);
                env_visual_cortex_destroy(&ivc);
        }

        env_allocation_cleanup();

        return 0;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* c-file-style: "linux" */
/* End: */

#endif // ENVISION_APP_ENVISION_C_DEFINED

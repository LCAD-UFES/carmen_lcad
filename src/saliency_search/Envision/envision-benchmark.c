/*!@file Envision/envision-benchmark.c Benchmark envision */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/envision-benchmark.c $
// $Id: envision-benchmark.c 9096 2007-12-18 21:27:33Z rjpeters $
//

#ifndef ENVISION_APP_ENVISION_C_DEFINED
#define ENVISION_APP_ENVISION_C_DEFINED

#include "Envision/env_alloc.h"
#include "Envision/env_c_math_ops.h"
#include "Envision/env_image.h"
#include "Envision/env_image_ops.h"
#include "Envision/env_mt_visual_cortex.h"
#include "Envision/env_params.h"
#include "Envision/env_pthread_interface.h"
#include "Envision/env_visual_cortex.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h> // for atoi(), malloc(), free()
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>

// ######################################################################
// Thunk to convert from env_size_t to size_t
static void* malloc_thunk(env_size_t n)
{
        return malloc(n);
}

// ######################################################################
static void assert_handler(const char* what, int custom_msg,
                           const char* where, int line_no)
        __attribute__((noreturn));

static void assert_handler(const char* what, int custom_msg,
                           const char* where, int line_no)
{
        if (custom_msg)
                fprintf(stderr, "Assertion failed (%s:%d):\n\t%s\n\n",
                        where, line_no, what);
        else
                fprintf(stderr, "Assertion failed (%s:%d):\n\texpected '%s'\n\n",
                        where, line_no, what);
        abort();
}

// ######################################################################
int main(int argc, const char** argv)
{
        if (argc != 1 && argc != 2 && argc != 4 && argc != 5)
        {
                fprintf(stderr,
                        "usage: %s [numframes=100] [width=512] [height=512] [multithreaded?]\n",
                        argv[0]);
                return 1;
        }

        const int nframes = argc > 1 ? atoi(argv[1]) : 100;
        const int w = argc > 2 ? atoi(argv[2]) : 512;
        const int h = argc > 3 ? atoi(argv[3]) : 512;
        const int multithreaded = argc > 4 ? atoi(argv[4]) : 0;
        const struct env_dims indims = { w, h };
        const env_size_t insize = w * h;

        // Instantiate our various ModelComponents:
        struct env_params envp;
        env_params_set_defaults(&envp);

        envp.maxnorm_type = ENV_VCXNORM_MAXNORM;
        envp.scale_bits = 16;

        env_assert_set_handler(&assert_handler);
        if (multithreaded)
        {
                env_init_pthread_alloc();
                env_init_pthread_job_server();
        }
        env_allocation_init(&malloc_thunk, &free);

        if (nframes > 0)
        {
                // allocate two images with different random
                // (uninitialized) content (so that we excite the
                // dynamic channels with non-static inputs, which may
                // some day make a difference in execute time):
                struct env_rgb_pixel *in1 = (struct env_rgb_pixel*)
                        env_allocate(insize * sizeof(struct env_rgb_pixel));
                struct env_rgb_pixel *in2 = (struct env_rgb_pixel*)
                        env_allocate(insize * sizeof(struct env_rgb_pixel));

                struct env_visual_cortex ivc;
                env_visual_cortex_init(&ivc, &envp);

                fprintf(stderr, "%s: START: %d frames %dx%d... ",
                        argv[0], nframes, w, h);
                fflush(stderr);

                struct timeval real1, real2;
                gettimeofday(&real1, /* timezone */ 0);

                struct rusage ru1, ru2;
                getrusage(RUSAGE_SELF, &ru1);

                for (int c = 0; c < nframes; ++c)
                {
                        struct env_image ivcout = env_img_initializer;
                        struct env_image intens = env_img_initializer;
                        struct env_image color = env_img_initializer;
                        struct env_image ori = env_img_initializer;
#                       ifdef ENV_WITH_DYNAMIC_CHANNELS
                        struct env_image flicker = env_img_initializer;
                        struct env_image motion = env_img_initializer;
#                       endif

                        env_mt_visual_cortex_input(multithreaded,
                                                &ivc, &envp, "visualcortex",
                                                c & 1 ? in2 : in1, 0,
                                                indims,
                                                0,
                                                0,
                                                &ivcout,
                                                &intens, &color, &ori
#                                               ifdef ENV_WITH_DYNAMIC_CHANNELS
                                                , &flicker, &motion
#                                               endif
                                );

                        env_visual_cortex_rescale_ranges(
                                &ivcout, &intens, &color, &ori
#                               ifdef ENV_WITH_DYNAMIC_CHANNELS
                                , &flicker, &motion
#                               endif
                                );

                        env_img_make_empty(&ivcout);
                        env_img_make_empty(&intens);
                        env_img_make_empty(&color);
                        env_img_make_empty(&ori);
#                       ifdef ENV_WITH_DYNAMIC_CHANNELS
                        env_img_make_empty(&flicker);
                        env_img_make_empty(&motion);
#                       endif
                }

                getrusage(RUSAGE_SELF, &ru2);

                gettimeofday(&real2, 0);

                const double real_secs =
                        (real2.tv_sec - real1.tv_sec)
                        + (real2.tv_usec - real1.tv_usec)
                        / 1000000.0;

                const double user_secs =
                        (ru2.ru_utime.tv_sec - ru1.ru_utime.tv_sec)
                        + (ru2.ru_utime.tv_usec - ru1.ru_utime.tv_usec)
                        / 1000000.0;

                const double sys_secs =
                        (ru2.ru_stime.tv_sec - ru1.ru_stime.tv_sec)
                        + (ru2.ru_stime.tv_usec - ru1.ru_stime.tv_usec)
                        / 1000000.0;

                const double frame_rate = nframes / real_secs;
                const double msec_per_frame = (1000.0*real_secs) / nframes;

                fprintf(stderr, "DONE.\n");
                fprintf(stderr, "%s: real %.3fs; user %.3fs; "
                        "sys %.3fs\n", argv[0], real_secs,
                        user_secs, sys_secs);
                fprintf(stderr, "%s: %.3ffps; %.3fmsec/frame\n",
                        argv[0], frame_rate, msec_per_frame);
                env_visual_cortex_destroy(&ivc);
                env_deallocate(in1);
                env_deallocate(in2);
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

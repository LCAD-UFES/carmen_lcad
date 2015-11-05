/*!@file Envision/envision-smoketest.c Run env_visual_cortex with random inputs to weed out bugs */

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
// Primary maintainer for this file:
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/envision-smoketest.c $
// $Id: envision-smoketest.c 14376 2011-01-11 02:44:34Z pez $
//

#ifndef ENVISION_ENVISION_SMOKETEST_C_DEFINED
#define ENVISION_ENVISION_SMOKETEST_C_DEFINED

#include "Envision/env_alloc.h"
#include "Envision/env_c_math_ops.h"
#include "Envision/env_image.h"
#include "Envision/env_image_ops.h"
#include "Envision/env_log.h"
#include "Envision/env_params.h"
#include "Envision/env_stdio_interface.h"
#include "Envision/env_visual_cortex.h"

#include <stdio.h>
#include <stdlib.h> // for atoi(), malloc(), free()
#include <sys/types.h> // for pid_t
#include <sys/wait.h>
#include <unistd.h> // for fork()

// ######################################################################
// Thunk to convert from env_size_t to size_t
static void* malloc_thunk(env_size_t n)
{
        return malloc(n);
}

// ######################################################################
static void run_evc(char **fnames, size_t nnames)
{
        struct env_params envp;
        env_params_set_defaults(&envp);

        envp.maxnorm_type = ENV_VCXNORM_MAXNORM;
        envp.scale_bits = 16;

        env_assert_set_handler(&env_stdio_assert_handler);
        env_allocation_init(&malloc_thunk, &free);

        struct env_visual_cortex ivc;
        env_visual_cortex_init(&ivc, &envp);

        for (int n = 0; n < nnames; ++n)
        {
                struct env_dims indims;
                struct env_rgb_pixel* input = 0;

                input = env_stdio_parse_rgb(fnames[n], &indims);

                struct env_image ivcout = env_img_initializer;
                struct env_image intens = env_img_initializer;
                struct env_image color = env_img_initializer;
                struct env_image ori = env_img_initializer;
#ifdef ENV_WITH_DYNAMIC_CHANNELS
                struct env_image flicker = env_img_initializer;
                struct env_image motion = env_img_initializer;
#endif

                env_visual_cortex_input(&ivc, &envp,
                                        "visualcortex",
                                        input, 0, indims,
                                        0,
                                        0,
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

                env_img_make_empty(&ivcout);
                env_img_make_empty(&intens);
                env_img_make_empty(&color);
                env_img_make_empty(&ori);
#ifdef ENV_WITH_DYNAMIC_CHANNELS
                env_img_make_empty(&flicker);
                env_img_make_empty(&motion);
#endif
        }

        env_visual_cortex_destroy(&ivc);

        env_allocation_cleanup();
}

// ######################################################################
int main(int argc, const char** argv)
{
        if (argc != 2)
        {
                fprintf(stderr,
                        "usage: %s niter\n",
                        argv[0]);
                return 1;
        }

        const int niter = atoi(argv[1]);

        const pid_t self = getpid();

        srand(self);

        for (int i = 0; i < niter; ++i)
        {
                char fnames0[3][64];
                char *fnames[3];
                fnames[0] = &fnames0[0][0];
                fnames[1] = &fnames0[1][0];
                fnames[2] = &fnames0[2][0];
                const int w = 1 + rand() % 2000;
                const int h = 1 + rand() % 2000;

                for (int n = 0; n < 3; ++n)
                {
                        snprintf(fnames[n], sizeof(fnames[n]),
                                 "smoketest-%d-%d-%06d.pnm",
                                 (int) self, i, n);

                        FILE* f = fopen(fnames[n], "w");
                        if (f == 0)
                        {
                                fprintf(stderr, "couldn't open %s for writing\n",
                                        fnames[n]);
                                exit(-1);
                        }

                        fprintf(f, "P6\n%d %d\n255\n", w, h);

                        for (int x = 0; x < 3*w*h; ++x)
                        {
                                const unsigned char val = rand() % 256;
                                fwrite(&val, 1, 1, f);
                        }

                        fclose(f);
                }

                fprintf(stderr, "running iter %d/%d with dims %dx%d... ",
                        i+1, niter, w, h);
                fflush(stderr);

                const pid_t child = fork();

                if (child < 0)
                {
                        fprintf(stderr, "fork() failed\n");
                        exit(-1);
                }
                else if (child > 0)
                {
                        // in parent
                        int status;
                        const pid_t p = waitpid(child, &status, 0);
                        if (p != child)
                        {
                                fprintf(stderr, "waitpid() failed\n");
                                exit(-1);
                        }

                        if (WIFEXITED(status) && WEXITSTATUS(status) == 0)
                        {
                                // ok, normal exit with status == 0
                                for (int n = 0; n < 3; ++n)
                                {
                                        if (unlink(fnames[n]) != 0)
                                        {
                                                fprintf(stderr,
                                                        "unlink() failed\n");
                                                exit(-1);
                                        }
                                }
                                fprintf(stderr, "iter %d/%d ok\n",
                                        i+1, niter);
                        }
                        else
                        {
                                fprintf(stderr, "iter %d/%d FAILED!\n",
                                        i+1, niter);
                        }

                        continue;
                }

                // else... in child

                run_evc(fnames, 3);

                exit(0);
        }

        return 0;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* c-file-style: "linux" */
/* End: */

#endif // ENVISION_ENVISION_SMOKETEST_C_DEFINED

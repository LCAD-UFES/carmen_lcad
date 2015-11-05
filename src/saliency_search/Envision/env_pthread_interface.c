/*!@file Envision/env_pthread_interface.c */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_pthread_interface.c $
// $Id: env_pthread_interface.c 8341 2007-05-04 18:49:06Z rjpeters $
//

#ifndef ENVISION_ENV_PTHREAD_INTERFACE_C_DEFINED
#define ENVISION_ENV_PTHREAD_INTERFACE_C_DEFINED

#include "Envision/env_pthread_interface.h"

#include "Envision/env_alloc.h"
#include "Envision/env_job_server.h"
#include "Envision/env_log.h"

#include <pthread.h>
#include <stdlib.h>

static pthread_mutex_t g_alloc_mutex = PTHREAD_MUTEX_INITIALIZER;

// ######################################################################
static void mutex_lock(void* p)
{
        pthread_mutex_t* m = (pthread_mutex_t*) p;
        const int code = pthread_mutex_lock(m);
        ENV_ASSERT2(code == 0, "pthread_mutex_lock failed");
}

// ######################################################################
static void mutex_unlock(void* p)
{
        pthread_mutex_t* m = (pthread_mutex_t*) p;
        const int code = pthread_mutex_unlock(m);
        ENV_ASSERT2(code == 0, "pthread_mutex_unlock failed");
}

// ######################################################################
static void* pthread_job_server_thread_func(void* env_job_pointer)
{
        struct env_job* j = (struct env_job*)(env_job_pointer);
        (*j->callback)(j->userdata);
        return (void*)0;
}

// ######################################################################
static void pthread_job_server(void* job_server_data,
                               const struct env_job* jobs,
                               const env_size_t njobs)
{
        if (njobs == 0)
                return;

        pthread_t* const threads = malloc(njobs * sizeof(pthread_t));
        ENV_ASSERT2(threads != 0, "malloc failed");

        for (env_size_t i = 0; i < njobs; ++i)
        {
                const int code =
                        pthread_create(threads+i, NULL,
                                       &pthread_job_server_thread_func,
                                       (void*) (jobs+i));
                ENV_ASSERT2(code == 0, "pthread_create failed");
        }

        for (env_size_t i = 0; i < njobs; ++i)
        {
                void* status = 0;
                const int code =
                        pthread_join(threads[i], &status);
                ENV_ASSERT2(code == 0, "pthread_join failed");
                ENV_ASSERT2(status == 0, "thread exited with non-zero status");
        }

        free(threads);
}

// ######################################################################
void env_init_pthread_alloc(void)
{
        env_allocation_init_mutex_funcs((void*) &g_alloc_mutex,
                                        &mutex_lock,
                                        &mutex_unlock);
}

// ######################################################################
void env_init_pthread_job_server(void)
{
        env_set_job_server(&pthread_job_server, 0);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* c-file-style: "linux" */
/* End: */

#endif // ENVISION_ENV_PTHREAD_INTERFACE_C_DEFINED

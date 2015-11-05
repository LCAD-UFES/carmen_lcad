/*!@file Envision/env_job_server.c */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_job_server.c $
// $Id: env_job_server.c 8341 2007-05-04 18:49:06Z rjpeters $
//

#ifndef ENVISION_ENV_JOB_SERVER_C_DEFINED
#define ENVISION_ENV_JOB_SERVER_C_DEFINED

#include "Envision/env_job_server.h"

static env_job_server* g_job_server_func = 0;
static void*           g_job_server_data = 0;

// ######################################################################
void env_set_job_server(env_job_server* server_func,
                        void* server_data)
{
        g_job_server_func = server_func;
        g_job_server_data = server_data;
}

// ######################################################################
void env_run_jobs(const struct env_job* jobs,
                  const env_size_t njobs)
{
        if (g_job_server_func != 0)
        {
                // is there a user-supplied job server? if so, then
                // let that server handle the jobs
                (*g_job_server_func)(g_job_server_data,
                                     jobs, njobs);
        }
        else
        {
                // otherwise, just run the jobs synchronously here
                for (env_size_t i = 0; i < njobs; ++i)
                {
                        (*jobs[i].callback)(jobs[i].userdata);
                }
        }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* c-file-style: "linux" */
/* End: */

#endif // ENVISION_ENV_JOB_SERVER_C_DEFINED

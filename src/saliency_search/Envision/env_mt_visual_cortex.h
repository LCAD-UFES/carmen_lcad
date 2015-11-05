/*!@file Envision/env_mt_visual_cortex.h */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_mt_visual_cortex.h $
// $Id: env_mt_visual_cortex.h 8341 2007-05-04 18:49:06Z rjpeters $
//

#ifndef ENVISION_ENV_MT_VISUAL_CORTEX_H_DEFINED
#define ENVISION_ENV_MT_VISUAL_CORTEX_H_DEFINED

#include "Envision/env_visual_cortex.h"

#ifdef __cplusplus
extern "C"
{
#endif

        /// Do visual-cortex input with optional multi-threading
        /** @param do_multithreaded if false, then just call the
            standard env_visual_cortex_input(); if true, then run
            individual sub-jobs with env_run_jobs() from
            Envision/env_job_server.h (note that you still must
            install a multi-threaded job server with
            env_set_job_server() in order to get actual asynchronous
            multi-thread performance)
         */
        void env_mt_visual_cortex_input(
                const int do_multithreaded,
                struct env_visual_cortex* vcx,
                const struct env_params* envp,
                const char* tagName,
                const struct env_rgb_pixel* const colimg,
                const struct env_rgb_pixel* const prev_colimg /* or null is fine here */,
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
                );

#ifdef __cplusplus
}
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* c-file-style: "linux" */
/* End: */

#endif // ENVISION_ENV_MT_VISUAL_CORTEX_H_DEFINED

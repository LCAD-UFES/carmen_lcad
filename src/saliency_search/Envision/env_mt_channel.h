/*!@file Envision/env_mt_channel.h */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_mt_channel.h $
// $Id: env_mt_channel.h 8422 2007-05-23 19:20:28Z rjpeters $
//

#ifndef ENVISION_ENV_MT_CHANNEL_H_DEFINED
#define ENVISION_ENV_MT_CHANNEL_H_DEFINED

#include "Envision/env_channel.h"
#include "Envision/env_motion_channel.h"

#ifdef __cplusplus
extern "C"
{
#endif

        //! A composite channel with a set of steerable-filter subchannels
        void env_mt_chan_orientation(const char* tagName,
                                     const struct env_params* envp,
                                     const struct env_math* imath,
                                     const struct env_image* img,
                                     env_chan_status_func* status_func,
                                     void* status_userdata,
                                     struct env_image* result);

        /// env_motion_channel only requires luminosity input
        /** for efficiency, the motion channel takes ownership of the
            lowpass5 pyramid (rather than needing to make a copy of
            it), so that after this function the lowpass5 argument
            will point to an empty pyramid */
        void env_mt_motion_channel_input_and_consume_pyr(
                struct env_motion_channel* chan,
                const char* tagName,
                const struct env_params* envp,
                const struct env_math* imath,
                const struct env_dims inputdims,
                struct env_pyr* lowpass5,
                env_chan_status_func* status_func,
                void* status_userdata,
                struct env_image* result);

#ifdef __cplusplus
}
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* c-file-style: "linux" */
/* End: */

#endif // ENVISION_ENV_MT_CHANNEL_H_DEFINED

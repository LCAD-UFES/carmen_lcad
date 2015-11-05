/*!@file Envision/env_channel.h Base class for channels that will use integer math */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_channel.h $
// $Id: env_channel.h 8054 2007-03-07 00:47:08Z rjpeters $
//

#ifndef ENVISION_ENV_CHANNEL_H_DEFINED
#define ENVISION_ENV_CHANNEL_H_DEFINED

#include "Envision/env_config.h"
#include "Envision/env_types.h"

struct env_dims;
struct env_image;
struct env_math;
struct env_params;
struct env_pyr;
struct env_rgb_pixel;

#ifdef __cplusplus
extern "C"
{
#endif

        typedef void (env_chan_status_func)(void* userdata,
                                            const char* tagName,
                                            const struct env_image* img);

        void env_chan_process_pyr(const char* tagName,
                                  const struct env_dims inputDims,
                                  const struct env_pyr* pyr,
                                  const struct env_params* envp,
                                  const struct env_math* imath,
                                  const int takeAbs,
                                  const int normalizeOutput,
                                  struct env_image* result);

        //! An intensity channel.
        void env_chan_intensity(const char* tagName,
                                const struct env_params* envp,
                                const struct env_math* imath,
                                const struct env_dims inputdims,
                                const struct env_pyr* lowpass5,
                                const int normalizeOutput,
                                env_chan_status_func* status_func,
                                void* status_userdata,
                                struct env_image* result);

        //! A double opponent color channel that combines r/g, b/y subchannels
        void env_chan_color(const char* tagName,
                            const struct env_params* envp,
                            const struct env_math* imath,
                            const struct env_rgb_pixel* const colimg,
                            const struct env_rgb_pixel* const prev_colimg /* or null is fine here */,
                            const struct env_dims dims,
                            env_chan_status_func* status_func,
                            void* status_userdata,
                            struct env_image* result);

        //! An orientation filtering channel
        void env_chan_steerable(const char* tagName,
                            const struct env_params* envp,
                            const struct env_math* imath,
                            const struct env_dims inputdims,
                            const struct env_pyr* hipass9,
                            const env_size_t thetaidx,
                            env_chan_status_func* status_func,
                            void* status_userdata,
                            struct env_image* result);

        //! A composite channel with a set of steerable-filter subchannels
        void env_chan_orientation(const char* tagName,
                                  const struct env_params* envp,
                                  const struct env_math* imath,
                                  const struct env_image* img,
                                  env_chan_status_func* status_func,
                                  void* status_userdata,
                                  struct env_image* result);

#ifdef ENV_WITH_DYNAMIC_CHANNELS

        //! A temporal flicker channel.
        void env_chan_flicker(const char* tagName,
                              const struct env_params* envp,
                              const struct env_math* imath,
                              const struct env_image* prev,
                              const struct env_image* cur,
                              env_chan_status_func* status_func,
                              void* status_userdata,
                              struct env_image* result);

        //! A true multi-scale temporal flicker channel.
        void env_chan_msflicker(const char* tagName,
                                const struct env_params* envp,
                                const struct env_math* imath,
                                const struct env_dims inputDims,
                                const struct env_pyr* prev_lowpass5,
                                const struct env_pyr* cur_lowpass5,
                                env_chan_status_func* status_func,
                                void* status_userdata,
                                struct env_image* result);

        //! A motion sensitive channel with direction selectivity
        void env_chan_direction(const char* tagName,
                                const struct env_params* envp,
                                const struct env_math* imath,
                                const struct env_dims inputdims,
                                const struct env_pyr* unshiftedPrev,
                                const struct env_pyr* unshiftedCur,
                                const struct env_pyr* shiftedPrev,
                                const struct env_pyr* shiftedCur,
                                env_chan_status_func* status_func,
                                void* status_userdata,
                                struct env_image* result);

#endif // ENV_WITH_DYNAMIC_CHANNELS

#ifdef __cplusplus
}
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* c-file-style: "linux" */
/* End: */

#endif // ENVISION_ENV_CHANNEL_H_DEFINED

/*!@file Envision/env_stdio_interface.h */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_stdio_interface.h $
// $Id: env_stdio_interface.h 7896 2007-02-10 01:12:29Z rjpeters $
//

#ifndef ENVISION_ENV_STDIO_INTERFACE_H_DEFINED
#define ENVISION_ENV_STDIO_INTERFACE_H_DEFINED

#include "Envision/env_types.h"

struct env_alloc_stats;
struct env_image;

#ifdef __cplusplus
extern "C"
{
#endif

        /// Print an error message to stderr and abort
        void env_stdio_assert_handler(
                const char* what, int custom_msg,
                const char* where, int line_no)
                __attribute__((noreturn));

        void env_stdio_print_alloc_stats(
                const struct env_alloc_stats* p,
                const env_size_t block_size);

        /// NOTE: caller must deallocate the result with env_deallocate()
        struct env_rgb_pixel* env_stdio_parse_rgb(
                const char* fname,
                struct env_dims* outdims);

        void env_stdio_write_gray(
                const struct env_image* iimage,
                const char* outstem, const char* name, int c);

#ifdef __cplusplus
}
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* c-file-style: "linux" */
/* End: */

#endif // ENVISION_ENV_STDIO_INTERFACE_H_DEFINED

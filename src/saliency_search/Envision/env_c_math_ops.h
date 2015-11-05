/*!@file Envision/env_c_math_ops.h Fixed-point integer math versions of some of our floating-point image functions */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_c_math_ops.h $
// $Id: env_c_math_ops.h 9830 2008-06-18 18:50:22Z lior $
//

#ifndef ENVISION_ENV_C_MATH_OPS_H_DEFINED
#define ENVISION_ENV_C_MATH_OPS_H_DEFINED

#include "Envision/env_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

        void env_c_lowpass_5_x_dec_x_fewbits_optim(const intg32* src,
                                                   const env_size_t w,
                                                   const env_size_t h,
                                                   intg32* dst,
                                                   const env_size_t w2);

        void env_c_lowpass_5_y_dec_y_fewbits_optim(const intg32* src,
                                                   const env_size_t w,
                                                   const env_size_t h,
                                                   intg32* dst,
                                                   const env_size_t h2);

        /// Like env_c_lowpass_9_x_fewbits() but uses optimized filter coefficients
        void env_c_lowpass_9_x_fewbits_optim(const intg32* src,
                                             const env_size_t w,
                                             const env_size_t h,
                                             intg32* dst);

        /// Like env_c_lowpass_9_y_fewbits() but uses optimized filter coefficients
        void env_c_lowpass_9_y_fewbits_optim(const intg32* src,
                                             const env_size_t w,
                                             const env_size_t h,
                                             intg32* dst);

        //! Get min and max values
        void env_c_get_min_max(const intg32* src, const env_size_t sz,
                               intg32* mini, intg32* maxi);

        //! Saturate values < 0
        void env_c_inplace_rectify(intg32* dst, const env_size_t sz);

        void env_c_inplace_normalize(intg32* dst, const env_size_t sz,
                                     const intg32 nmin, const intg32 nmax,
                                     intg32* actualmin, intg32* actualmax,
                                     intg32 rangeThresh);

        /// get the luminance with nbits of precision of the input image
        void env_c_luminance_from_byte(const struct env_rgb_pixel* const src,
                                       const env_size_t sz,
                                       const env_size_t nbits,
                                       intg32* const dst);

        /// result = a / val
        void env_c_image_div_scalar(const intg32* const a,
                                    const env_size_t sz,
                                    intg32 val,
                                    intg32* const dst);

        /// result += a / val
        void env_c_image_div_scalar_accum(const intg32* const a,
                                          const env_size_t sz,
                                          intg32 val,
                                          intg32* const dst);

        /// result = a - b
        void env_c_image_minus_image(const intg32* const a,
                                     const intg32* const b,
                                     const env_size_t sz,
                                     intg32* const dst);

#ifdef __cplusplus
}
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* c-file-style: "linux" */
/* End: */

#endif // ENVISION_ENV_C_MATH_OPS_H_DEFINED

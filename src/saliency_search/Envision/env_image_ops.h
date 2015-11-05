/*!@file Envision/env_image_ops.h Fixed-point integer math versions of some of our floating-point image functions */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_image_ops.h $
// $Id: env_image_ops.h 11338 2009-06-24 06:58:51Z itti $
//

#ifndef ENVISION_ENV_IMAGE_OPS_H_DEFINED
#define ENVISION_ENV_IMAGE_OPS_H_DEFINED

#include "Envision/env_config.h"
#include "Envision/env_image.h"
#include "Envision/env_math.h"
#include "Envision/env_pyr.h"

struct env_rgb_pixel;

#define INTMAXNORMMIN ((intg32) 0)
#define INTMAXNORMMAX ((intg32) 32768)

#ifdef __cplusplus
extern "C"
{
#endif

        //! Decimate in X and Y (take one every 'factor' pixels).
        void env_dec_xy(const struct env_image* src, struct env_image* result);

        //! Decimate in X (take one every 'factor' pixels).
        void env_dec_x(const struct env_image* src, struct env_image* result);

        //! Decimate in Y (take one every 'factor' pixels).
        void env_dec_y(const struct env_image* src, struct env_image* result);

        void env_lowpass_5_x_dec_x(const struct env_image* src,
                                   const struct env_math* imath,
                                   struct env_image* result);

        void env_lowpass_5_y_dec_y(const struct env_image* src,
                                   const struct env_math* imath,
                                   struct env_image* result);

        void env_lowpass_9_x(const struct env_image* src,
                             const struct env_math* imath,
                             struct env_image* result);
        void env_lowpass_9_y(const struct env_image* src,
                             const struct env_math* imath,
                             struct env_image* result);
        void env_lowpass_9(const struct env_image* src,
                           const struct env_math* imath,
                           struct env_image* result);
        void env_quad_energy(const struct env_image* img1,
                             const struct env_image* img2,
                             struct env_image* result);
        void env_steerable_filter(const struct env_image* src,
                                  const intg32 kxnumer, const intg32 kynumer,
                                  const env_size_t kdenombits,
                                  const struct env_math* imath,
                                  struct env_image* result);
        void env_attenuate_borders_inplace(struct env_image* a, env_size_t size);

        void env_pyr_build_hipass_9(const struct env_image* image,
                                    env_size_t firstlevel,
                                    const struct env_math* imath,
                                    struct env_pyr* result);

        void env_pyr_build_steerable_from_hipass_9(const struct env_pyr* hipass,
                                                   const intg32 kxnumer,
                                                   const intg32 kynumer,
                                                   const env_size_t kdenombits,
                                                   const struct env_math* imath,
                                                   struct env_pyr* result);
        //! Wrapper for _cpu or _cuda version
        void env_pyr_build_lowpass_5(const struct env_image* image,
                                     env_size_t firstlevel,
                                     const struct env_math* imath,
                                     struct env_pyr* result);

        //! _cpu version implemented here, see CUDA/env_cuda.h for GPU version
        void env_pyr_build_lowpass_5_cpu(const struct env_image* image,
                                         env_size_t firstlevel,
                                         const struct env_math* imath,
                                         struct env_pyr* result);

        void env_downsize_9_inplace(struct env_image* src, const env_size_t depth,
                                    const struct env_math* imath);
        void env_rescale(const struct env_image* src, struct env_image* result);
        void env_max_normalize_inplace(struct env_image* src,
                                       intg32 min, intg32 max,
                                       enum env_maxnorm_type typ,
                                       const intg32 rangeThresh);
        void env_max_normalize_none_inplace(struct env_image* src,
                                            intg32 min, intg32 max,
                                            const intg32 rangeThresh);
        void env_max_normalize_std_inplace(struct env_image* src,
                                           intg32 min, intg32 max,
                                           const intg32 rangeThresh);

        void env_center_surround(const struct env_image* center,
                                 const struct env_image* surround,
                                 const int absol,
                                 struct env_image* result);

        /// Compute R-G and B-Y opponent color maps
        /** If src2 is non-null, then average values from src and src2
            together before any further processing (this may be useful
            in averaging noise out of noisy color input sources). */
        void env_get_rgby(const struct env_rgb_pixel* const src,
                          const struct env_rgb_pixel* const src2 /* or null is fine here */,
                          const env_size_t sz,
                          struct env_image* rg,
                          struct env_image* by, const intg32 thresh,
                          const env_size_t inputbits);

        /// Update the range [mi,ma] to include the range of values in src
        void env_merge_range(const struct env_image* src,
                             intg32* mi, intg32* ma);

        /// rescale the src image to a [0..255] result
        void env_rescale_range_inplace(struct env_image* src,
                                       const intg32 mi, const intg32 ma);

#ifdef ENV_WITH_DYNAMIC_CHANNELS

        //! Shift an image by (dx, dy), without wraparound
        void env_shift_clean(const struct env_image* srcImg,
                             const env_ssize_t dx, const env_ssize_t dy,
                             struct env_image* result);

        void env_shift_image(const struct env_image* srcImg,
                             const env_ssize_t dxnumer, const env_ssize_t dynumer,
                             const env_size_t denombits,
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

#endif // ENVISION_ENV_IMAGE_OPS_H_DEFINED

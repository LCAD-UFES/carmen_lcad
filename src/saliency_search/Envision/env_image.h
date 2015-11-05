/*!@file Envision/env_image.h A basic image class */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_image.h $
// $Id: env_image.h 7645 2007-01-04 21:07:32Z rjpeters $
//

#ifndef ENVISION_ENV_IMAGE_H_DEFINED
#define ENVISION_ENV_IMAGE_H_DEFINED

#include "Envision/env_alloc.h"
#include "Envision/env_types.h"

//! Basic image class
struct env_image
{
        struct env_dims dims;   // width+height of data array
        intg32* pixels;         // data array
};

#define env_img_initializer { {0,0}, 0 }

#ifdef __cplusplus
extern "C"
{
#endif

        void env_img_init(struct env_image* img,
                          const struct env_dims d);

        static inline void env_img_init_empty(struct env_image* img);

        //! Get image size (width * height)
        static inline env_size_t env_img_size(const struct env_image* img);

        //! Check whether image is non-empty (i.e., non-zero height and width).
        static inline int env_img_initialized(const struct env_image* img);

        void env_img_swap(struct env_image* img1,
                          struct env_image* img2);

        void env_img_make_empty(struct env_image* img);

        void env_img_resize_dims(struct env_image* img,
                                 const struct env_dims d);

        static inline const intg32* env_img_pixels(const struct env_image* img);

        static inline intg32* env_img_pixelsw(struct env_image* img);

        void env_img_copy_src_dst(const struct env_image* src,
                                  struct env_image* dst);

        static inline int env_dims_equal(const struct env_dims d1,
                                         const struct env_dims d2);

#ifdef __cplusplus
}
#endif

// ######################################################################
static inline void env_img_init_empty(struct env_image* img)
{
        img->pixels = 0;
        img->dims.w = img->dims.h = 0;
}

// ######################################################################
static inline env_size_t env_img_size(const struct env_image* img)
{
        return img->dims.w * img->dims.h;
}

// ######################################################################
static inline int env_img_initialized(const struct env_image* img)
{
        return (img->dims.w * img->dims.h) > 0;
}

// ######################################################################
static inline const intg32* env_img_pixels(const struct env_image* img)
{
        return img->pixels;
}

// ######################################################################
static inline intg32* env_img_pixelsw(struct env_image* img)
{
        return img->pixels;
}

// ######################################################################
static inline int env_dims_equal(const struct env_dims d1,
                                 const struct env_dims d2)
{
        return d1.w == d2.w && d1.h == d2.h;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* c-file-style: "linux" */
/* End: */

#endif

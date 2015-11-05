/*!@file Envision/env_image.c */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_image.c $
// $Id: env_image.c 7638 2007-01-03 21:23:18Z rjpeters $
//

#ifndef ENVISION_ENV_IMAGE_C_DEFINED
#define ENVISION_ENV_IMAGE_C_DEFINED

#include "Envision/env_image.h"

// ######################################################################
void env_img_init(struct env_image* img,
                  const struct env_dims d)
{
        img->pixels = (intg32*) env_allocate(d.w * d.h * sizeof(intg32));
        img->dims = d;
}

// ######################################################################
void env_img_swap(struct env_image* img1,
                  struct env_image* img2)
{
        const struct env_image img1copy = *img1;
        *img1 = *img2;
        *img2 = img1copy;
}

// ######################################################################
void env_img_make_empty(struct env_image* img)
{
        env_deallocate(img->pixels);
        img->dims.w = img->dims.h = 0;
        img->pixels = 0;
}

// ######################################################################
void env_img_resize_dims(struct env_image* img,
                         const struct env_dims d)
{
        if (d.w != img->dims.w || d.h != img->dims.h)
        {
                env_deallocate(img->pixels);
                img->pixels =
                        (intg32*) env_allocate(d.w * d.h * sizeof(intg32));
                img->dims = d;
        }
}

// ######################################################################
void env_img_copy_src_dst(const struct env_image* src,
                          struct env_image* dst)
{
        if (src == dst)
                return;

        env_img_resize_dims(dst, src->dims);

        const env_size_t sz = env_img_size(src);
        const intg32* const sptr = env_img_pixels(src);
        intg32* const dptr = env_img_pixelsw(dst);

        for (env_size_t i = 0; i < sz; ++i)
          dptr[i] = sptr[i];
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* c-file-style: "linux" */
/* End: */

#endif // ENVISION_ENV_IMAGE_C_DEFINED

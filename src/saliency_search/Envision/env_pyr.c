/*!@file Envision/env_pyr.c */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_pyr.c $
// $Id: env_pyr.c 7628 2007-01-03 04:07:21Z rjpeters $
//

#ifndef ENVISION_ENV_PYR_C_DEFINED
#define ENVISION_ENV_PYR_C_DEFINED

#include "Envision/env_pyr.h"

// ######################################################################
void env_pyr_init(struct env_pyr* pyr, const env_size_t n)
{
        pyr->images = (struct env_image*)
                env_allocate(n * sizeof(struct env_image));

        pyr->depth = n;

        for (env_size_t i = 0; i < pyr->depth; ++i)
                env_img_init_empty(&pyr->images[i]);
}

// ######################################################################
void env_pyr_make_empty(struct env_pyr* dst)
{
        for (env_size_t i = 0; i < dst->depth; ++i)
                env_img_make_empty(&dst->images[i]);
        env_deallocate(dst->images);
        dst->images = 0;
        dst->depth = 0;
}

// ######################################################################
void env_pyr_swap(struct env_pyr* pyr1, struct env_pyr* pyr2)
{
        const struct env_pyr pyr1copy = *pyr1;
        *pyr1 = *pyr2;
        *pyr2 = pyr1copy;
}

// ######################################################################
void env_pyr_copy_src_dst(const struct env_pyr* src,
                          struct env_pyr* dst)
{
        if (dst == src)
                return;

        if (dst->depth != src->depth)
        {
                for (env_size_t i = 0; i < dst->depth; ++i)
                        env_img_make_empty(&dst->images[i]);
                env_deallocate(dst->images);
                dst->images = (struct env_image*)
                        env_allocate(src->depth
                                     * sizeof(struct env_image));
                dst->depth = src->depth;
                for (env_size_t i = 0; i < dst->depth; ++i)
                        env_img_init_empty(&dst->images[i]);
        }

        for (env_size_t i = 0; i < dst->depth; ++i)
        {
                env_img_copy_src_dst(&src->images[i],
                                     &dst->images[i]);
        }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* c-file-style: "linux" */
/* indent-tabs-mode: nil */
/* End: */

#endif // ENVISION_ENV_PYR_C_DEFINED

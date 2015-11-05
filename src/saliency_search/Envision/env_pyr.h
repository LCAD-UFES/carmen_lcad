/*!@file Envision/env_pyr.h */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_pyr.h $
// $Id: env_pyr.h 7628 2007-01-03 04:07:21Z rjpeters $
//

#ifndef ENVISION_ENV_PYR_H_DEFINED
#define ENVISION_ENV_PYR_H_DEFINED

#include "Envision/env_image.h"
#include "Envision/env_log.h"

// ######################################################################
//! This class implements a set of images, often used as a dyadic pyramid.
struct env_pyr
{
        struct env_image* images;
        env_size_t depth;
};

#define env_pyr_initializer { 0, 0 }

#ifdef __cplusplus
extern "C"
{
#endif

        //! Construct an empty pyramid
        static inline void env_pyr_init_empty(struct env_pyr* pyr);

        //! Construct with a given number of empty images.
        void env_pyr_init(struct env_pyr* pyr, const env_size_t n);

        void env_pyr_make_empty(struct env_pyr* dst);

        void env_pyr_copy_src_dst(const struct env_pyr* src,
                                  struct env_pyr* dst);

        //! Swap contents with another env_pyr
        void env_pyr_swap(struct env_pyr* pyr1,
                          struct env_pyr* pyr2);

        //! Return number of images in image set.
        static inline env_size_t env_pyr_depth(const struct env_pyr* pyr);

        //! Get image from a given level.
        static inline const struct env_image* env_pyr_img(const struct env_pyr* pyr,
                                                          const env_size_t lev);

        //! Get mutable image from a given level.
        static inline struct env_image* env_pyr_imgw(struct env_pyr* pyr,
                                                     const env_size_t lev);

#ifdef __cplusplus
}
#endif



// ######################################################################
static inline void env_pyr_init_empty(struct env_pyr* pyr)
{
        pyr->images = 0;
        pyr->depth = 0;
}

// ######################################################################
static inline env_size_t env_pyr_depth(const struct env_pyr* pyr)
{
        return pyr->depth;
}

// ######################################################################
static inline const struct env_image* env_pyr_img(const struct env_pyr* pyr,
                                                  const env_size_t lev)
{
        return &pyr->images[lev];
}

// ######################################################################
static inline struct env_image* env_pyr_imgw(struct env_pyr* pyr,
                                             const env_size_t lev)
{
        return &pyr->images[lev];
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* c-file-style: "linux" */
/* End: */

#endif // !ENVISION_ENV_PYR_H_DEFINED

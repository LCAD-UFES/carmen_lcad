/*!@file Envision/env_types.h  Basic integer types */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_types.h $
// $Id: env_types.h 7975 2007-02-22 04:29:11Z rjpeters $
//

#ifndef ENVISION_TYPES_H_DEFINED
#define ENVISION_TYPES_H_DEFINED

#include "Envision/env_config.h"

//! 8-bit unsigned integer
typedef unsigned char byte;

//! 16-bit signed integer
typedef short intg16;

typedef char env_intg16_must_be_16_bits[sizeof(intg16) == 2
                                        ? 1 : -1];

//! 32-bit signed integer
typedef ENV_INTG32_TYPE intg32;

#define INTG32_MAX ((ENV_INTG32_TYPE)(((unsigned ENV_INTG32_TYPE)(-1)) >> 1))
#define INTG32_MIN ((ENV_INTG32_TYPE)((((unsigned ENV_INTG32_TYPE)(-1)) >> 1) + 1))

typedef char env_intg32_must_be_32_bits[sizeof(intg32) == 4
                                        ? 1 : -1];

#ifdef ENV_INTG64_TYPE

//! 64-bit signed integer
typedef long long intg64;

typedef char env_intg64_must_be_64_bits[sizeof(intg64) == 8
                                        ? 1 : -1];

#endif

typedef long env_ssize_t;
typedef unsigned long env_size_t;

//! RGB pixel class
struct env_rgb_pixel
{
        byte p[3];
};

//! A simple struct to hold a pair of width/height dimensions.
struct env_dims
{
        env_size_t w;  //!< The width.
        env_size_t h;  //!< The height.
};

/// Types of normalization
enum env_maxnorm_type
{
        ENV_VCXNORM_NONE    = 0,  //!< no max-normalization, but may change range
        ENV_VCXNORM_MAXNORM = 1,  //!< non-iterative maxnorm
};

#define ENV_MAX(a, b) ( (a) > (b) ? (a) : (b) )
#define ENV_MIN(a, b) ( (a) < (b) ? (a) : (b) )
#define ENV_ABS(a)    ( (a) > 0 ? (a) : -(a) )

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* c-file-style: "linux" */
/* End: */

#endif // !ENVISION_TYPES_H_DEFINED

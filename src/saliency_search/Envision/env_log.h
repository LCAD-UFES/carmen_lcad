/*!@file Envision/env_log.h Interface to syslog */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Envision/env_log.h $
// $Id: env_log.h 7629 2007-01-03 04:07:38Z rjpeters $
//

#ifndef ENVISION_ENV_LOG_H_DEFINED
#define ENVISION_ENV_LOG_H_DEFINED

#ifdef __cplusplus
extern "C"
{
#endif

        typedef void (env_assert_handler)(const char* what,
                                          int custom_msg,
                                          const char* where,
                                          int line_no);

        void env_assert(const char* what,
                        int custom_msg,
                        const char* where,
                        int line_no);

        void env_assert_set_handler(
                env_assert_handler __attribute__((noreturn))* handler);

#ifdef __cplusplus
}
#endif

#endif

#if !defined(ENV_NO_DEBUG)

#  define ENV_ASSERT(expr)       do { if ( !(expr) ) env_assert(#expr, 0, __FILE__, __LINE__); } while(0)
#  define ENV_ASSERT2(expr, msg) do { if ( !(expr) ) env_assert(msg,   1, __FILE__, __LINE__); } while(0)

#else // defined(ENV_NO_DEBUG)

#  define ENV_ASSERT(expr)          do {} while (0)
#  define ENV_ASSERT2(expr, msg)    do {} while (0)

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* c-file-style: "linux" */
/* End: */

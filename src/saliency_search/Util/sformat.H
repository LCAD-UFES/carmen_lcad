/*!@file Util/sformat.H convert varargs to a std::string as if by snprintf() */

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
// Primary maintainer for this file: Rob Peters <rjpeters@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/sformat.H $
// $Id: sformat.H 4730 2005-06-29 19:03:01Z rjpeters $
//

#ifndef SFORMAT_H_DEFINED
#define SFORMAT_H_DEFINED

#include <stdarg.h>
#include <string>

//! snprintf() the specified format string + varargs into a std::string.
/*! NOTE: The CALLER is responsible for doing va_end(ap); it is not
    done internally in vsformat(). */
std::string vsformat(const char* fmt, va_list ap);

//! snprintf() the specified format string + varargs into a std::string.
std::string sformat(const char* fmt, ...)
  // NOTE: this __attribute__ tells gcc that it should issue
  // printf-style warnings when compiling calls to sformat(),
  // treating the 1st argument (fmt) as the format string, and the
  // 2nd and subsequent arguments as the printf-style parameters
  __attribute__((format(__printf__, 1, 2)));
  ;

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // SFORMAT_H_DEFINED

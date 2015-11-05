/*!@file Util/sformat.C convert varargs to a std::string as if by snprintf() */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/sformat.C $
// $Id: sformat.C 6100 2006-01-17 01:59:41Z rjpeters $
//

#ifndef SFORMAT_C_DEFINED
#define SFORMAT_C_DEFINED

#include "Util/sformat.H"

#include "Util/Assert.H"
#include "Util/log.H"
#include "rutz/compat_snprintf.h"

std::string vsformat(const char* fmt, va_list ap)
{
  // if we have a null pointer or an empty string, then just return an
  // empty std::string
  if (fmt == 0 || fmt[0] == '\0')
    {
      return std::string();
    }

  int bufsize = 512;
  while (1)
    {
      // relying on a gcc extension to create stack arrays whose size
      // is not a compile-time constant -- could also use alloca or
      // std::vector here
      char buf[bufsize];

      const int nchars = vsnprintf(buf, bufsize,
                                   fmt, ap);

      if (nchars < 0)
        {
          // Better leave this as LFATAL() rather than LERROR(),
          // otherwise we have to return a bogus std::string (e.g. an
          // empty string, or "none", or...), which might be dangerous
          // if it is later used as a filename, for example.
          LFATAL("vsnprintf failed for format '%s' with bufsize = %d",
                 fmt, bufsize);
        }
      else if (nchars >= bufsize)
        {
          // buffer was too small, so let's double the bufsize and try
          // again:
          bufsize *= 2;
          continue;
        }
      else
        {
          // OK, the vsnprintf() succeeded:
          return std::string(&buf[0], nchars);
        }
    }
  // should never get here:
  ASSERT(0);
  return std::string(); // can't happen, but placate the compiler
}

std::string sformat(const char* fmt, ...)
{
  va_list a;
  va_start(a, fmt);
  std::string result = vsformat(fmt, a);
  va_end(a);
  return result;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // SFORMAT_C_DEFINED

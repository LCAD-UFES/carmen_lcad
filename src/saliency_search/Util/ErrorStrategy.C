/*!@file Util/ErrorStrategy.C An enum for the classic abort/retry/ignore error-handling choices */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/ErrorStrategy.C $
// $Id: ErrorStrategy.C 7844 2007-02-07 00:18:49Z rjpeters $
//

#ifndef UTIL_ERRORSTRATEGY_C_DEFINED
#define UTIL_ERRORSTRATEGY_C_DEFINED

#include "Util/ErrorStrategy.H"

#include "Util/StringConversions.H"
#include "Util/StringUtil.H"
#include "Util/log.H"

// ######################################################################
std::string convertToString(const ErrorStrategy& val)
{
  switch (val)
    {
    case ERR_ABORT: return std::string("ABORT");
    case ERR_RETRY: return std::string("RETRY");
    case ERR_IGNORE: return std::string("IGNORE");
    }

  LFATAL("Bogus ErrorStrategy value: %d", int(val));
  /* can't happen */ return std::string();
}

// ######################################################################
void convertFromString(const std::string& str, ErrorStrategy& val)
{
  const std::string lower = toLowerCase(str);

  if (lower.compare("abort") == 0)
    val = ERR_ABORT;
  else if (lower.compare("retry") == 0)
    val = ERR_RETRY;
  else if (lower.compare("ignore") == 0)
    val = ERR_IGNORE;
  else
    conversion_error::raise<ErrorStrategy>(str);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_ERRORSTRATEGY_C_DEFINED

/*!@file Util/SimTime.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/SimTime.C $
// $Id: SimTime.C 12074 2009-11-24 07:51:51Z itti $
//

#ifndef UTIL_SIMTIME_C_DEFINED
#define UTIL_SIMTIME_C_DEFINED

#include "Util/SimTime.H"

#include "Util/StringConversions.H"
#include "Util/log.H"
#include "Util/sformat.H"

#include <cstdio> // for sscanf()


// ######################################################################
std::string SimTime::toString(bool with_suffix) const
{
  if (with_suffix)
    return sformat("%g", this->secs());
  else
    return sformat("%gs", this->secs());
}

// ######################################################################
SimTime SimTime::fromString(const std::string& str,
                            const char* default_suffix)
{
  // handle special magic words
  if (str.compare("max") == 0 || str.compare("MAX") == 0 ||
      str.compare("forever") == 0 || str.compare("FOREVER") == 0 ||
      str.compare("unlimited") == 0 || str.compare("UNLIMITED") == 0)
    return SimTime::MAX();

  std::string::size_type last_digit = str.find_last_of("0123456789");

  const std::string num = str.substr(0, last_digit+1);

  double value; char junk[2];
  // make an attempt to scan extra characters after the floating-point
  // number; we expect that there should be none, otherwise the input
  // is trashy (this way we can reject input like "3.x43"):
  if (sscanf(num.c_str(), "%lf%1s", &value, &junk[0]) != 1)
    LFATAL("while parsing SimTime string '%s':\n"
           "\tnumeric conversion failed from '%s'",
           str.c_str(), num.c_str());

  if (default_suffix==0 && last_digit+1 >= str.length())
    LFATAL("while parsing SimTime string '%s':\n"
           "\tno suffix found in '%s' and no default suffix was given\n"
           "\tin the code -- to avoid this error, either add a 's' or\n"
           "\t'ms' suffix (i.e., '%ss' or '%sms'), or change the code\n"
           "\tto specify a default suffix",
           str.c_str(), str.c_str(), str.c_str(), str.c_str());

  const std::string suffix =
    last_digit+1 < str.length()
    ? str.substr(last_digit+1, str.npos)
    : default_suffix;

  // now, parse the suffix and convert so that we have a number of
  // nanoseconds:

  if (suffix == "s" ||
      suffix == "sec" ||
      suffix == "secs" ||
      suffix == "seconds")
    {
      value *= 1000000000.0; // was seconds, now is nanoseconds
    }
  else if (suffix == "ms" ||
           suffix == "msec" ||
           suffix == "msecs" ||
           suffix == "milliseconds")
    {
      value *= 1000000.0; // was milliseconds, now is nanoseconds
    }
  else if (suffix == "us" ||
           suffix == "usec" ||
           suffix == "usecs" ||
           suffix == "microseconds")
    {
      value *= 1000.0; // was microseconds, now is nanoseconds
    }
  else if (suffix == "ns" ||
           suffix == "nsec" ||
           suffix == "nsecs" ||
           suffix == "nanoseconds")
    {
      /* do nothing, value is already in nanoseconds */
    }
  else if (suffix == "Hz")
    {
      value = 1.0/value;  // was seconds^-1, now is seconds
      value *= 1000000000.0; // was seconds, now is nanoseconds
    }
  else if (suffix == "kHz")
    {
      value = 1.0/value;  // was kiloseconds^-1, now is milliseconds
      value *= 1000000.0; // was milliseconds, now is nanoseconds
    }
  else if (suffix == "MHz")
    {
      value = 1.0/value;  // was megaseconds^-1, now is microseconds
      value *= 1000.0; // was microseconds, now is nanoseconds
    }
  else
    {
      LFATAL("while parsing SimTime string '%s':\n"
             "\tunknown units suffix '%s'",
             str.c_str(), suffix.c_str());
    }

  const double int64min = double(std::numeric_limits<int64>::min());
  const double int64max = double(std::numeric_limits<int64>::max());

  if (value < int64min)
    {
      LFATAL("time value '%s' (%.2ens) is too small to be represented "
             "(min value is %.2ens)",
             str.c_str(), value, int64min);
    }
  else if (value > int64max)
    {
      LFATAL("time value '%s' (%.2ens) is too large to be represented "
             "(max value is %.2ens)",
             str.c_str(), value, int64max);
    }

  return SimTime::NSECS(int64(value));
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_SIMTIME_C_DEFINED

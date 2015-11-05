/*!@file Util/ByteCount.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/ByteCount.C $
// $Id: ByteCount.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef UTIL_BYTECOUNT_C_DEFINED
#define UTIL_BYTECOUNT_C_DEFINED

#include "Util/ByteCount.H"

#include "Util/StringConversions.H"
#include "Util/log.H"

#include <sstream>

ByteCount::ByteCount(const std::string& str)
{
  std::istringstream iss(str);

  double nbytes = -1.0;

  iss >> nbytes;

  if (nbytes < 0.0)
    conversion_error::raise<ByteCount>
      (str, "byte count must be positive");

  std::string suffix;

  iss >> suffix;

  if (suffix.empty() || suffix == "B")
    {
      // nothing to do here
    }
  else if (suffix == "KiB") // kibibyte = 2^10B
    {
      nbytes *= 1024;
    }
  else if (suffix == "MiB") // mebibyte = 2^20B
    {
      nbytes *= 1024*1024;
    }
  else if (suffix == "GiB") // gibibyte = 2^30B
    {
      nbytes *= 1024*1024*1024;
    }
#if 0
  // we could add these extra suffixes, but they'll just be bogus on
  // any machine where size_t is 32 bits -- need to wait til we all
  // have 64-bit machines... ;)
  else if (suffix == "TiB") // tebibyte = 2^40B
    {
      nbytes *= 1024*1024*1024*1024;
    }
  else if (suffix == "PiB") // pebibyte = 2^50B
    {
      nbytes *= 1024*1024*1024*1024*1024;
    }
  else if (suffix == "EiB") // exbibyte = 2^60B
    {
      nbytes *= 1024*1024*1024*1024*1024*1024;
    }
#endif
  else
    {
      conversion_error::raise<ByteCount>
        (str, sformat("unknown ByteCount suffix '%s', expected "
                      "B, KiB, MiB, or GiB", suffix.c_str()));
    }

  itsCount = size_t(nbytes);

  LDEBUG("parsed ByteCount string '%s' as %" ZU " bytes (prettyPrint=%s)",
         str.c_str(), itsCount, this->prettyPrint().c_str());
}

std::string ByteCount::prettyPrint() const
{
  if (itsCount < 1024)
    return sformat("%" ZU "B", itsCount);
  else if (itsCount < 1024*1024)
    return sformat("%.3fKiB", itsCount/1024.0);
  else if (itsCount < 1024*1024*1024)
    return sformat("%.3fMiB", itsCount/(1024.0*1024.0));
  else
    return sformat("%.3fGiB", itsCount/(1024.0*1024.0*1024.0));
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_BYTECOUNT_C_DEFINED

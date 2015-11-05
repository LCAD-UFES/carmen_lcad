/*!@file Image/ResizeSpec.C Represents multiple ways of transforming image dimensions */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/ResizeSpec.C $
// $Id: ResizeSpec.C 8828 2007-10-12 21:20:18Z rjpeters $
//

#ifndef IMAGE_RESIZESPEC_C_DEFINED
#define IMAGE_RESIZESPEC_C_DEFINED

#include "Image/ResizeSpec.H"

#include "Util/Assert.H"
#include "Util/StringConversions.H"
#include "Util/StringUtil.H"
#include "Util/log.H"
#include "Util/sformat.H"

#include <iterator>
#include <vector>

// ######################################################################
bool ResizeSpec::operator==(const ResizeSpec& that) const
{
  if (this->itsMethod != that.itsMethod)
    return false;

  // OK, now what if the methods are the same:

  switch (this->itsMethod)
    {
    case NOOP:
      return true;
      break;

    case FIXED:
      return this->itsNewDims == that.itsNewDims;
      break;

    case SCALE_UP:     // fall-through
    case SCALE_DOWN:
      return (this->itsFactorW == that.itsFactorW
              && this->itsFactorH == that.itsFactorH);
      break;
    }

  ASSERT(0); /* can't happen */ return false;
}

// ######################################################################
std::string ResizeSpec::toString() const
{
  switch (itsMethod)
    {
    case NOOP: return "noop"; break;

    case FIXED: return convertToString(itsNewDims); break;

    case SCALE_UP:
      if (itsFactorW == itsFactorH)
        return sformat("*%g", itsFactorW);
      else
        return sformat("*%gx%g", itsFactorW, itsFactorH);
      break;

    case SCALE_DOWN:
      if (itsFactorW == itsFactorH)
        return sformat("/%g", itsFactorW);
      else
        return sformat("/%gx%g", itsFactorW, itsFactorH);
      break;
    }

  ASSERT(0); /* can't happen */ return std::string();
}

// ######################################################################
ResizeSpec ResizeSpec::fromString(const std::string& origstr)
{
  const std::string str = toLowerCase(origstr);

  if (str.length() == 0
      || str.compare("none") == 0
      || str.compare("noop") == 0)
    {
      return ResizeSpec(); // no-op ResizeSpec
    }
  else if (str[0] == '*' || str[0] == '/')
    {
      const Method m = (str[0] == '*' ? SCALE_UP : SCALE_DOWN);

      std::vector<std::string> parts;
      split(str.substr(1), "x", std::back_inserter(parts));

      if (parts.size() == 1)
        {
          const double f = fromStr<double>(parts[0]);
          if (f < 0.0)
            LFATAL("while parsing '%s' as a ResizeSpec: expected "
                   "a non-negative scale factor, but got %s",
                   origstr.c_str(), parts[0].c_str());

          if (f == 0.0 || f == 1.0)
            return ResizeSpec(); // no-op ResizeSpec

          return ResizeSpec(m, Dims(), f, f);
        }
      else if (parts.size() == 2)
        {
          const double fw = fromStr<double>(parts[0]);
          const double fh = fromStr<double>(parts[1]);

          if (fw < 0.0)
            LFATAL("while parsing '%s' as a ResizeSpec: expected "
                   "a non-negative scale factor, but got %s",
                   origstr.c_str(), parts[0].c_str());

          if (fh < 0.0)
            LFATAL("while parsing '%s' as a ResizeSpec: expected "
                   "a non-negative scale factor, but got %s",
                   origstr.c_str(), parts[1].c_str());

          if ((fw == 0.0 || fw == 1.0) && (fh == 0.0 || fh == 1.0))
            return ResizeSpec(); // no-op ResizeSpec

          return ResizeSpec(m, Dims(), fw, fh);
        }
      else
        LFATAL("while parsing '%s' as a ResizeSpec: after '%c', "
               "expected either one floating-point value or "
               "two values separated by 'x', but got '%s'",
               origstr.c_str(), str[0], str.substr(1).c_str());
    }
  else
    {
      const Dims d = fromStr<Dims>(str);
      if (d.isEmpty())
        return ResizeSpec(); // no-op ResizeSpec
      return ResizeSpec(FIXED, d, 0.0, 0.0);
    }

  conversion_error::raise<ResizeSpec>(origstr);
  ASSERT(0); /* can't happen */ return ResizeSpec();
}

// ######################################################################
Dims ResizeSpec::transformDims(const Dims& in)
{
  switch (itsMethod)
    {
    case NOOP:
      return in;
      break;

    case FIXED:
      return itsNewDims;
      break;

    case SCALE_UP:
      // if a scale factor is 0, then that dimension just passes
      // through untouched
      return Dims(itsFactorW > 0.0
                  ? int(0.5 + in.w() * itsFactorW)
                  : in.w(),
                  itsFactorH > 0.0
                  ? int(0.5 + in.h() * itsFactorH)
                  : in.h());
      break;

    case SCALE_DOWN:
      // if a scale factor is 0, then that dimension just passes
      // through untouched
      return Dims(itsFactorW > 0.0
                  ? int(0.5 + in.w() / itsFactorW)
                  : in.w(),
                  itsFactorH > 0.0
                  ? int(0.5 + in.h() / itsFactorH)
                  : in.h());
      break;
    }

  // we should never get here, because even if the user gave bogus
  // input, we should have caught that in convertFromString() or
  // wherever, so that once we have a ResizeSpec object, it should be
  // guaranteed to have a valid itsMethod value:
  ASSERT(0); /* can't happen */ return Dims();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_RESIZESPEC_C_DEFINED

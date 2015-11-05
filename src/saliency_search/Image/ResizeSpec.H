/*!@file Image/ResizeSpec.H Represents multiple ways of transforming image dimensions */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/ResizeSpec.H $
// $Id: ResizeSpec.H 8828 2007-10-12 21:20:18Z rjpeters $
//

#ifndef IMAGE_RESIZESPEC_H_DEFINED
#define IMAGE_RESIZESPEC_H_DEFINED

#include "Image/Dims.H"

#include <string>

/// Represents multiple methods for transforming image dimensions
/** A ResizeSpec object represents some method for transforming an
    original set of image dimensions (A) into a new set of image
    dimensions (B). Supported methods include:


    - METHOD               EFFECT                STRING REPRESENTATION(S)
    - pass-through:        B = A                 "noop", "0x0", "*0", "/0", "*1", "/1"
    - fixed resize:        B = newW x newH       "64x48"
    - uniform scale up:    B = A.w*k x A.h*k     "*2.5"
    - uniform scale down:  B = A.w/k x A.h/k     "/3"
    - mixed scale up:      B = A.w*k1 x A.h*k2   "*3x4"
    - mixed scale down:    B = A.w/k1 x A.h/k2   "/1.5x2"
*/
class ResizeSpec
{
private:
  enum Method { NOOP, FIXED, SCALE_UP, SCALE_DOWN };

  Method itsMethod;
  Dims itsNewDims;   // used with FIXED
  double itsFactorW; // used with SCALE_UP and SCALE_DOWN
  double itsFactorH; // used with SCALE_UP and SCALE_DOWN

  /// Private constructor; use a pseudo-constructor instead
  ResizeSpec(const Method m, const Dims& d,
             const double fw, const double fh)
    : itsMethod(m), itsNewDims(d), itsFactorW(fw), itsFactorH(fh)
  {}

public:
  ResizeSpec()
    : itsMethod(NOOP), itsNewDims(), itsFactorW(0.0), itsFactorH(0.0)
  {}

  // default copy ctor, assignment operator, destructor all OK

  /// implicit conversion constructor from Dims to a fixed ResizeSpec
  ResizeSpec(const Dims& d)
    : itsMethod(FIXED), itsNewDims(d), itsFactorW(0.0), itsFactorH(0.0)
  {}

  bool operator==(const ResizeSpec& that) const;

  // pseudo-constructors for building the various ResizeSpec types

  static ResizeSpec noOp()
  { return ResizeSpec(); }

  static ResizeSpec fixed(const Dims& d)
  { return ResizeSpec(FIXED, d, 0.0, 0.0); }

  static ResizeSpec scaleUp(const double f)
  { return ResizeSpec(SCALE_UP, Dims(), f, f); }

  static ResizeSpec scaleUp(const double fw, const double fh)
  { return ResizeSpec(SCALE_UP, Dims(), fw, fh); }

  static ResizeSpec scaleDown(const double f)
  { return ResizeSpec(SCALE_DOWN, Dims(), f, f); }

  static ResizeSpec scaleDown(const double fw, const double fh)
  { return ResizeSpec(SCALE_DOWN, Dims(), fw, fh); }

  std::string toString() const;

  static ResizeSpec fromString(const std::string& str);

  Dims transformDims(const Dims& in);
};

//! ResizeSpec -> string
inline std::string convertToString(const ResizeSpec val)
{ return val.toString(); }

//! string -> ResizeSpec
inline void convertFromString(const std::string& str, ResizeSpec& val)
{ val = ResizeSpec::fromString(str); }


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_RESIZESPEC_H_DEFINED

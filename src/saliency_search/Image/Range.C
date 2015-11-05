/*!@file Image/Range.C Manage min/max range information  */
// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2002   //
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
// Primary maintainer for this file: David Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Range.C $

#include "Image/Range.H"
#include "Util/StringConversions.H"

#include <sstream>

// ######################################################################
// converters for Range
// ######################################################################
template<class T>
std::string convertToString(const Range<T>& val)
{ std::stringstream s; s<<val.min()<<'-'<<val.max(); return s.str(); }

template<class T>
void convertFromString(const std::string& str, Range<T>& val)
{
  std::stringstream s; T min = (T)-999, max = (T)-999; char c;
  s<<str; s>>min; s>>c; s>>max;
  if (min == (T)-999 || max == (T)-999 || c != '-')
    conversion_error::raise<Range<T> >(str);
  val = Range<T>(min, max);
}

//explicit instantiations
template std::string convertToString(const Range<int>& val);
template void convertFromString(const std::string& str, Range<int>& val);

template std::string convertToString(const Range<double>& val);
template void convertFromString(const std::string& str, Range<double>& val);

template std::string convertToString(const Range<float>& val);
template void convertFromString(const std::string& str, Range<float>& val);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/*!@file Image/Point3D.C A basic 3D point class */

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
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Point3D.C $
// $Id: Point3D.C 14612 2011-03-15 20:22:26Z jshen $
//

#include "Image/Point3D.H"

#include "Util/StringConversions.H"
#include "Util/StringUtil.H"
#include "Util/Types.H"
#include "Util/log.H"

#include <iterator>
#include <string>
#include <vector>

// ######################################################################
// converters for Point3D
// ######################################################################

template <class T>
std::string convertToString(const Point3D<T>& val)
{
  return convertToString(val.x) + ',' + convertToString(val.y) + ',' + convertToString(val.z);
}

template <class T>
void convertFromString(const std::string& str, Point3D<T>& val)
{
  std::vector<std::string> toks;

  split(str, ",", std::back_inserter(toks));

  if (toks.size() != 3)
    conversion_error::raise<Point3D<T> >
      (str, "expected two comma-separated numbers");

  val.x = fromStr<T>(toks[0]);
  val.y = fromStr<T>(toks[1]);
  val.z = fromStr<T>(toks[2]);
}

template std::string convertToString(const Point3D<int>&);
template void convertFromString(const std::string&, Point3D<int>&);

template std::string convertToString(const Point3D<float>&);
template void convertFromString(const std::string&, Point3D<float>&);

template std::string convertToString(const Point3D<double>&);
template void convertFromString(const std::string&, Point3D<double>&);


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

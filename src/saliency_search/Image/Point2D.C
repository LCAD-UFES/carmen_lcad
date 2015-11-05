/*!@file Image/Point2D.C A basic 2D point class */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Point2D.C $
// $Id: Point2D.C 9412 2008-03-10 23:10:15Z farhan $
//

#include "Image/Point2D.H"

#include "Util/StringConversions.H"
#include "Util/StringUtil.H"
#include "Util/Types.H"
#include "Util/log.H"

#include <iterator>
#include <string>
#include <vector>

// ######################################################################
// converters for Point2D
// ######################################################################

template <class T>
std::string convertToString(const Point2D<T>& val)
{
  return convertToString(val.i) + ',' + convertToString(val.j);
}

template <class T>
void convertFromString(const std::string& str, Point2D<T>& val)
{
  std::vector<std::string> toks;

  split(str, ",", std::back_inserter(toks));

  if (toks.size() != 2)
    conversion_error::raise<Point2D<T> >
      (str, "expected two comma-separated numbers");

  val.i = fromStr<T>(toks[0]);
  val.j = fromStr<T>(toks[1]);
}

template std::string convertToString(const Point2D<int>&);
template void convertFromString(const std::string&, Point2D<int>&);

template std::string convertToString(const Point2D<float>&);
template void convertFromString(const std::string&, Point2D<float>&);

template std::string convertToString(const Point2D<double>&);
template void convertFromString(const std::string&, Point2D<double>&);


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

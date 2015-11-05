/*!@file Image/Pixels.C Basic pixel types version 2.0 */

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
// Primary maintainer for this file: T. Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Pixels.C $
// $Id: Pixels.C 12598 2010-01-19 03:03:03Z dberg $
//

#include "Image/Pixels.H"

#include "Util/StringConversions.H"

#include <sstream>

// ######################################################################
// converters for PixRGB
// ######################################################################
std::string convertToString(const PixRGB<byte>& val)
{
  std::stringstream s;
  s<<int(val.red())<<','<<int(val.green())<<','<<int(val.blue());
  return s.str();
}

// ######################################################################
void convertFromString(const std::string& str, PixRGB<byte>& val)
{
  if (str.length() > 0 && str[0] == '#') {
    // #RRGGBB format
    std::string str2 = str.substr(1);
    std::stringstream s; s<<std::hex<<str2; int rgb; s>>rgb;
    val.setRed((rgb & 0xff0000) >> 16);
    val.setGreen((rgb & 0xff00) >> 8);
    val.setBlue(rgb & 0xff);
  } else {
    // <byte>,<byte>,<byte> format
    std::stringstream s; int r = -1, g = -1, b = -1;
    char c; s<<str; s>>r>>c>>g>>c>>b;
    if (r == -1 || g == -1 || b == -1)
      conversion_error::raise<PixRGB<byte> >(str);
    val.set(r, g, b);
  }
}

// ######################################################################
// converters for PixHSV
// ######################################################################
std::string convertToString(const PixHSV<byte>& val)
{
  std::stringstream s;
  s<<int(val.H())<<','<<int(val.S())<<','<<int(val.V());
  return s.str();
}

// ######################################################################
void convertFromString(const std::string& str, PixHSV<byte>& val)
{
  // <byte>,<byte>,<byte> format
  std::stringstream ss; int h = -1, s = -1, v = -1;
  char c; ss<<str; ss>>h>>c>>s>>c>>v;
  if (h == -1 || s == -1 || v == -1)
    conversion_error::raise<PixHSV<byte> >(str);
  val.setH(h);
  val.setS(s);
  val.setV(v);
}

// ######################################################################
// converters for PixDKL
// ######################################################################
std::string convertToString(const PixDKL<byte>& val)
{
  std::stringstream s;
  s<<int(val.D())<<','<<int(val.K())<<','<<int(val.L());
  return s.str();
}
// ######################################################################
void convertFromString(const std::string& str, PixDKL<byte>& val)
{
  // <byte>,<byte>,<byte> format
  std::stringstream s; int d = -1, k = -1, l = -1;
  char c; s<<str; s>>d>>c>>k>>c>>l;
  if (d == -1 || k == -1 || l == -1)
    conversion_error::raise<PixDKL<byte> >(str);
  val.setD(d);
  val.setK(k);
  val.setL(l);
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

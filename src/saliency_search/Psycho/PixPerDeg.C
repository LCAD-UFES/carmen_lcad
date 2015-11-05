/*!@file Psycho/PixPerDeg.C */

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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu:/software/invt/trunk/saliency/src/Psycho/PixPerDeg.C $

#include "Psycho/PixPerDeg.H"
#include "Util/StringConversions.H"
#include "Util/log.H"

#include <sstream>

// ######################################################################
PixPerDeg::PixPerDeg() :
  itsPPDx(0.0F), itsPPDy(0.0F) { }

// ######################################################################
PixPerDeg::PixPerDeg(const float& ppdx, const float& ppdy) :
  itsPPDx(ppdx), itsPPDy(ppdy) { }

// ######################################################################
PixPerDeg::~PixPerDeg() { }

// ######################################################################
float PixPerDeg::ppd()  const
{ return (itsPPDy == 0.0F) ? itsPPDx : (itsPPDx + itsPPDy)/2.0F; }

// ######################################################################
float PixPerDeg::ppdx() const
{ return itsPPDx; }

// ######################################################################
float PixPerDeg::ppdy() const
{ return (itsPPDy == 0.0F) ? itsPPDx : itsPPDy; }

// ######################################################################
bool PixPerDeg::operator==(const PixPerDeg& x) const throw()
{ return itsPPDx == x.itsPPDx && itsPPDy == x.itsPPDy; }

// ######################################################################
bool PixPerDeg::operator!=(const PixPerDeg& x) const throw()
{ return itsPPDx != x.itsPPDx || itsPPDy != x.itsPPDy; }

// ######################################################################
std::string convertToString(const PixPerDeg& val)
{
  std::stringstream s; s << val.ppdx() << ',' << val.ppdy(); return s.str();
}

// ######################################################################
void convertFromString(const std::string& str, PixPerDeg& val)
{
 std::stringstream s; float w = -1.0F, h = -1.0F; char c;
  s<<str; s>>w; s>>c; s>>h;
  if (w < 0.0F || h < 0.0F || c != ',')
    conversion_error::raise<PixPerDeg>(str);
  val = PixPerDeg(w, h);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

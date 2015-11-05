/*!@file Channels/MapCombineType.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/MapCombineType.C $
// $Id: MapCombineType.C 8827 2007-10-12 17:45:29Z rjpeters $
//

#ifndef CHANNELS_MAPCOMBINETYPE_C_DEFINED
#define CHANNELS_MAPCOMBINETYPE_C_DEFINED

#include "Channels/MapCombineType.H"

#include "Image/Image.H"
#include "Image/MathOps.H"
#include "Util/StringConversions.H"
#include "Util/StringUtil.H"
#include "Util/log.H"

// ######################################################################
Image<float> mapCombine(const MapCombineType typ,
                        const Image<float>& a, const Image<float>& b)
{
  // a+[] -> a; []+b -> b
  if (!a.initialized())
    return b;
  else if (!b.initialized())
    return a;

  if (a.getDims() != b.getDims())
    LFATAL("can't combine images of unequal dimensions "
           "%dx%d and %dx%d",
           a.getWidth(), a.getHeight(), b.getWidth(), b.getHeight());

  switch (typ)
    {
    case MAPCOMBINE_SUM: return a + b; break;
    case MAPCOMBINE_MAX: return takeMax(a, b); break;
    }

  LFATAL("invalid MapCombineType '%d'", int(typ));

  /* can't happen */ return Image<float>();
}

// ######################################################################
std::string convertToString(const MapCombineType typ)
{
  switch (typ)
    {
    case MAPCOMBINE_SUM: return "Sum"; break;
    case MAPCOMBINE_MAX: return "Max"; break;
    }

  LFATAL("invalid MapCombineType '%d'", int(typ));
  /* can't happen */ return std::string();
}

// ######################################################################
void convertFromString(const std::string& str1, MapCombineType& typ)
{
  const std::string str = toLowerCase(str1);

  if      (str.compare("sum") == 0) typ = MAPCOMBINE_SUM;
  else if (str.compare("max") == 0) typ = MAPCOMBINE_MAX;
  else
    conversion_error::raise<MapCombineType>(str1);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_MAPCOMBINETYPE_C_DEFINED

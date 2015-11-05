/*!@file Neuro/IORtypes.C Definition of the inhibitionof-return (IOR) types */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/IORtypes.C $
// $Id: IORtypes.C 5755 2005-10-19 20:57:27Z rjpeters $
//

#include "Neuro/IORtypes.H"

#include "Util/StringConversions.H"
#include "Util/log.H"

// ######################################################################
// converters for IORtype
// ######################################################################

std::string convertToString(const IORtype val)
{
  switch (val)
    {
    case IORnone:     return std::string("None");
    case IORdisc:     return std::string("Disc");
    case IORshapeEst: return std::string("ShapeEst");
    }
  LFATAL("Unknown IOR type: %d", int(val));
  /* can't happen */ return std::string();
}

void convertFromString(const std::string& str, IORtype& val)
{
  if      (str.compare("None") == 0)     val = IORnone;
  else if (str.compare("Disc") == 0)     val = IORdisc;
  else if (str.compare("ShapeEst") == 0) val = IORshapeEst;
  else
    conversion_error::raise<IORtype>(str);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

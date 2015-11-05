/*!@file Raster/RasterFileFormat.C file formats supported by Raster */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/RasterFileFormat.C $
// $Id: RasterFileFormat.C 9203 2008-02-01 18:17:14Z rjpeters $
//

#include "Raster/RasterFileFormat.H"

#include "Util/StringConversions.H"
#include "Util/log.H"

// ######################################################################
// converters for RasterFileFormat
// ######################################################################

std::string convertToString(const RasterFileFormat val)
{
  switch (val)
    {
    case RASFMT_PNM:          return "PNM";
    case RASFMT_PNG:          return "PNG";
    case RASFMT_PFM:          return "PFM";
    case RASFMT_PFZ:          return "PFZ";
    case RASFMT_RAW_VIDEO:    return "RAWVIDEO";
    case RASFMT_RAW_IMAGE:    return "RAWIMAGE";
    case RASFMT_JPEG:         return "JPEG";
    case RASFMT_TXT:          return "TXT";
    case RASFMT_CCODE:        return "CCODE";
    case RASFMT_DPX:          return "DPX";
    case RASFMT_AUTO:         return "Auto";
    }
  LFATAL("invalid RasterFileFormat value (%d)", int(val));
  /* can't happen */ return std::string();
}

void convertFromString(const std::string& str, RasterFileFormat& val)
{
  if      (str.compare("PNM")      == 0) { val = RASFMT_PNM; }
  else if (str.compare("PNG")      == 0) { val = RASFMT_PNG; }
  else if (str.compare("PFM")      == 0) { val = RASFMT_PFM; }
  else if (str.compare("PFZ")      == 0) { val = RASFMT_PFZ; }
  else if (str.compare("RAWVIDEO") == 0) { val = RASFMT_RAW_VIDEO; }
  else if (str.compare("RAWIMAGE") == 0) { val = RASFMT_RAW_IMAGE; }
  else if (str.compare("JPEG")     == 0) { val = RASFMT_JPEG; }
  else if (str.compare("TXT")      == 0) { val = RASFMT_TXT; }
  else if (str.compare("CCODE")    == 0) { val = RASFMT_CCODE; }
  else if (str.compare("DPX")      == 0) { val = RASFMT_DPX; }
  else if (str.compare("Auto")     == 0) { val = RASFMT_AUTO; }
  else
    conversion_error::raise<RasterFileFormat>(str);
}

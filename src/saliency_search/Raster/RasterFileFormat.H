/*!@file Raster/RasterFileFormat.H file formats supported by Raster */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/RasterFileFormat.H $
// $Id: RasterFileFormat.H 9203 2008-02-01 18:17:14Z rjpeters $
//

#ifndef RASTERFILEFORMAT_H_DEFINED
#define RASTERFILEFORMAT_H_DEFINED

#include <string>

//! Definition of supported Raster file formats
/*! See Raster.H for functions using these file formats. If you add
  new formats here, be sure to also update StringConversions.C */
enum RasterFileFormat
  {
    RASFMT_PNM = 0,      //!< see PnmWriter, PnmParser
    RASFMT_PNG,          //!< see PngWriter, PngParser
    RASFMT_PFM,          //!< see PfmWriter, PfmParser
    RASFMT_PFZ,          //!< see PfzWriter, PfzParser
    RASFMT_RAW_VIDEO,    //!< see YuvWriter, YuvParser
    RASFMT_RAW_IMAGE,    //!< see RawWriter  (no parser class currently)
    RASFMT_JPEG,         //!< see JpegParser (no writer class currently)
    RASFMT_TXT,          //!< see PlaintextWriter
    RASFMT_CCODE,        //!< see CcodeWriter
    RASFMT_DPX,          //!< see DpxParser, DpxFile
    RASFMT_AUTO
  };


//! RasterFileFormat string conversion
/*! Use it, for example, if you got a file format as text from the
  command line and need to convert that to a RasterFileFormat value
  for a ModelParam. Text strings should match the symbolic names of
  the RasterFileFormat enum. */
std::string convertToString(const RasterFileFormat val);
//! RasterFileFormat string conversion
/*! The names used in the definition of the enum RasterFileFormat
    will be returned */
void convertFromString(const std::string& str, RasterFileFormat& val);

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

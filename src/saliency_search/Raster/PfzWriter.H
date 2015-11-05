/*!@file Raster/PfzWriter.H Write pfm image files */

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
// Primary maintainer for this file: T Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/PfzWriter.H $
// $Id: PfzWriter.H 6795 2006-06-29 20:45:32Z rjpeters $
//

#ifndef RASTER_PFZWRITER_H_DEFINED
#define RASTER_PFZWRITER_H_DEFINED

// zlib buffer size
#define PFZ_CHUNK 262144
// zlib defaults to 6, 9 is max
#define PFZ_LEVEL 6

#include "Raster/RasterWriter.H"
#include "Util/Promotions.H"
#include "Util/Types.H"
#include <zlib.h>
#include <deque>
#include <string>

template <class T> class Image;
template <class T> class PixRGB;

/// Write pfz image files
/*! Compressed portable float image writer
    T. Nathan Mundhenk mundhenk@usc.edu
*/
class PfzWriter : public RasterWriter
{
public:
  PfzWriter();

  virtual ~PfzWriter();

  virtual std::string writeFrame(const GenericFrame& image,
                                 const std::string& fname);

  static void writeFloat(const Image<float>& image,
                         const std::string& fname,
                         const std::deque<std::string> tagName,
                         const std::deque<std::string> tagValue);

  static void writeFloat(const Image<float>& image,
                         const std::string& fname,
                         const bool useTag = false);

  static std::deque<std::string> itsTagName;
  static std::deque<std::string> itsTagValue;
  static z_stream                zstrm;
  static unsigned char           buff_uin[PFZ_CHUNK];
  static unsigned char           buff_uout[PFZ_CHUNK];
  static int                     itsCompressionLevel;
};



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // RASTER_PFMWRITER_H_DEFINED

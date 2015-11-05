/*!@file Raster/JpegWriter.H Write Jpeg image files */

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
// Primary maintainer for this file: Chin-Kai Chang <chinkaic@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/JpegWriter.H $
// $Id: JpegWriter.H 8141 2012-04-30 06:43:45Z kai $
//

#ifndef JPEGWRITER_H_DEFINED
#define JPEGWRITER_H_DEFINED

#ifdef INVT_HAVE_LIBJPEG

#include "Image/JPEGUtil.H"
#include "Raster/RasterWriter.H"
#include "Util/Types.H"

template <class T> class Image;
template <class T> class PixRGB;

/// Write PNG image files
class JpegWriter : public RasterWriter
{
public:
  JpegWriter();

  virtual ~JpegWriter();

  virtual std::string writeFrame(const GenericFrame& image,
                                 const std::string& fname);

  static void writeRGB(const Image<PixRGB<byte> >& image,
                       const std::string& fname);

  static void writeGray(const Image<byte>& image,
                        const std::string& fname);
};

#endif // INVT_HAVE_LIBJPEG


#endif // !JPEGWRITER_H_DEFINED

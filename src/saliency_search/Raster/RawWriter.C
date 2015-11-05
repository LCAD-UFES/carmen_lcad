/*!@file Raster/RawWriter.C Write RAW image files  */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/RawWriter.C $
// $Id: RawWriter.C 14290 2010-12-01 21:44:03Z itti $
//

#ifndef RASTER_RAWWRITER_C_DEFINED
#define RASTER_RAWWRITER_C_DEFINED

#include "Raster/RawWriter.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"

#include <fstream>

RawWriter::RawWriter() {}

RawWriter::~RawWriter() {}

std::string RawWriter::writeFrame(const GenericFrame& image,
                                  const std::string& fname)
{
  switch (image.nativeType())
    {
    case GenericFrame::NONE:
      LFATAL("cannot write an empty image to '%s'", fname.c_str());
      break;

    case GenericFrame::RGB_U8:
    case GenericFrame::RGBD:
      RawWriter::writeRGB(image.asRgbU8(), fname);
      break;

    case GenericFrame::RGB_U16:
      RawWriter::writeRGB(image.asRgbU16(), fname);
      break;

    case GenericFrame::RGB_F32:
      RawWriter::writeRGB(image.asRgbU8(), fname);
      break;

    case GenericFrame::GRAY_U8:
      RawWriter::writeGray(image.asGrayU8(), fname);
      break;

    case GenericFrame::GRAY_U16:
      RawWriter::writeGray(image.asGrayU16(), fname);
      break;

    case GenericFrame::GRAY_F32:
      RawWriter::writeFloat(image.asGrayF32(), fname);
      break;

    case GenericFrame::VIDEO:
      LFATAL("cannot write a video frame '%s' in RAW format", fname.c_str());
      break;
    }

  return fname;
}

void RawWriter::writeRGB(const Image<PixRGB<byte> >& image,
                         const std::string& fname)
{
  std::ofstream ofs(fname.c_str(), std::ios::binary);
  if (!ofs.is_open())
    LFATAL("Couldn't open RAW file '%s' for writing.", fname.c_str());
  ofs.write(reinterpret_cast<const char*>(image.getArrayPtr()),
            image.getSize() * sizeof(PixRGB<byte>));
  ofs.close();
  if (ofs.fail())
    LFATAL("Output stream failure while writing '%s'.", fname.c_str());
}

void RawWriter::writeGray(const Image<byte>& image,
                          const std::string& fname)
{
  std::ofstream ofs(fname.c_str(), std::ios::binary);
  if (!ofs.is_open())
    LFATAL("Couldn't open RAW file '%s' for writing.", fname.c_str());
  ofs.write(reinterpret_cast<const char*>(image.getArrayPtr()),
            image.getSize());
  if (ofs.fail())
    LFATAL("Output stream failure while writing '%s'.", fname.c_str());
}

void RawWriter::writeFloat(const Image<float>& image,
                           const std::string& fname)
{
  std::ofstream ofs(fname.c_str(), std::ios::binary);
  if (!ofs.is_open())
    LFATAL("Couldn't open RAW file '%s' for writing.", fname.c_str());
  ofs.write(reinterpret_cast<const char*>(image.getArrayPtr()),
            image.getSize() * sizeof(float));
  if (ofs.fail())
    LFATAL("Output stream failure while writing '%s'.", fname.c_str());
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // RASTER_RAWWRITER_C_DEFINED

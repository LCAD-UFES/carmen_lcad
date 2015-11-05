/*!@file Raster/PnmWriter.C Write pbm/pgm/ppm image files */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/PnmWriter.C $
// $Id: PnmWriter.C 15424 2012-11-02 08:02:51Z kai $
//

#ifndef RASTER_PNMWRITER_C_DEFINED
#define RASTER_PNMWRITER_C_DEFINED

#include "Raster/PnmWriter.H"

#include "Image/Image.H"
#include "Image/Normalize.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "Raster/PfmWriter.H"

#include <fstream>

// ######################################################################
PnmWriter::PnmWriter(const bool forcepbm, const byte pbmthresh)
  :
  itsForcePBM(forcepbm),
  itsPBMThresh(pbmthresh)
{}

// ######################################################################
PnmWriter::~PnmWriter() {}

// ######################################################################
std::string PnmWriter::writeFrame(const GenericFrame& image,
                                  const std::string& fnameorig)
{
  if (itsForcePBM)
    {
      PnmWriter::writeRawBW(image.asGray(), itsPBMThresh, fnameorig);
      return fnameorig;
    }

  std::string fname = fnameorig;

  switch (image.nativeType())
    {
    case GenericFrame::NONE:
      LFATAL("can't write an empty image to '%s'", fname.c_str());
      break;

    case GenericFrame::RGB_U8:
    case GenericFrame::RGBD:
      PnmWriter::writeRGB(image.asRgbU8(), fname);
      break;

    case GenericFrame::RGB_U16:
      PnmWriter::writeRGB(image.asRgbU16(), fname);
      break;

    case GenericFrame::RGB_F32:
      PnmWriter::writeRGB(image.asRgbU8(), fname);
      break;

    case GenericFrame::GRAY_U8:
      PnmWriter::writeGray(image.asGrayU8(), fname);
      break;

    case GenericFrame::GRAY_U16:
      PnmWriter::writeGray(image.asGrayU16(), fname);
      break;

    case GenericFrame::GRAY_F32:
      {
        if (image.floatFlags() & FLOAT_NORM_PRESERVE)
          {
            // make a new filename with a ".pfm" extension, either by
            // replacing an existing ".pgm" or ".ppm" or ".pnm"
            // extension, or by just tacking on a new ".pfm" extension
            std::string::size_type len = fname.length();
            if ((len > 4)
                && (fname.compare(len-4, 4, ".pgm") == 0 ||
                    fname.compare(len-4, 4, ".ppm") == 0 ||
                    fname.compare(len-4, 4, ".pnm") == 0))
              {
                fname[len-2] = 'f';
              }
            else
              {
                fname += ".pfm";
              }
            PfmWriter::writeFloat(image.asGrayF32(), fname);
          }
        else
          {
            PnmWriter::writeGray(image.asGrayU8(), fname);
          }
      }
      break;

    case GenericFrame::VIDEO:
      PnmWriter::writeRGB(image.asRgb(), fname);
      break;
    }

  return fname;
}

// ######################################################################
void PnmWriter::writeRGB(const Image<PixRGB<byte> >& image,
                         std::ostream& strm)
{
  strm << "P6\n" << image.getWidth() << ' ' << image.getHeight()
       << "\n255\n";
  strm.write(reinterpret_cast<const char*>(image.getArrayPtr()),
             3 * image.getSize());
}

// ######################################################################
void PnmWriter::writeGray(const Image<byte>& image,
                          std::ostream& strm)
{
  strm << "P5\n" << image.getWidth() << ' ' << image.getHeight()
       << "\n255\n";
  strm.write(reinterpret_cast<const char*>(image.getArrayPtr()),
            image.getSize());
}
// ######################################################################
void PnmWriter::writeGray(const Image<uint16>& image,
                          std::ostream& strm)
{
  strm << "P5\n" << image.getWidth() << ' ' << image.getHeight()
       << "\n65535\n";
  strm.write(reinterpret_cast<const char*>(image.getArrayPtr()),
            image.getSize()*2);
}

// ######################################################################
void PnmWriter::writeRawBW(const Image<byte>& image, const byte thresh,
                           std::ostream& strm)
{
  strm << "P4\n" << image.getWidth() << ' ' << image.getHeight() << '\n';

  Image<byte>::const_iterator itr = image.begin(), stop = image.end();

  int c = 0;
  int pos = 7;

  while (itr != stop)
    {
      if (*itr++ >= thresh)
        { /* leave bit at zero */ } // "0" ==> white pixel
      else
        c |= (1 << pos);            // "1" ==> black pixel

      if (pos == 0 || (itr == stop))
        {
          strm.put(byte(c));
          c = 0;
          pos = 7;
        }
      else
        --pos;
    }
}

// ######################################################################
void PnmWriter::writeAsciiBW(const Image<byte>& image, const byte thresh,
                             std::ostream& strm)
{
  strm << "P1\n" << image.getWidth() << ' ' << image.getHeight() << '\n';

  Image<byte>::const_iterator itr = image.begin();

  for (int j = 0; j < image.getHeight(); ++j)
    {
      for (int i = 0; i < image.getWidth(); ++i)
        {
          if (*itr++ >= thresh)
            strm << "0 "; // "0" ==> white pixel
          else
            strm << "1 "; // "1" ==> black pixel
        }
      strm << '\n';
    }
}

// ######################################################################
void PnmWriter::writeRGB(const Image<PixRGB<byte> >& image,
                         const std::string& fname)
{
  std::ofstream ofs(fname.c_str(), std::ios::binary);
  if (!ofs.is_open())
    LFATAL("Couldn't open PNM file '%s' for writing.", fname.c_str());

  PnmWriter::writeRGB(image, ofs);

  ofs.close();
  if (ofs.fail())
    LFATAL("Output stream failure while writing '%s'.", fname.c_str());
}

// ######################################################################
void PnmWriter::writeGray(const Image<byte>& image,
                          const std::string& fname)
{
  std::ofstream ofs(fname.c_str(), std::ios::binary);
  if (!ofs.is_open())
    LFATAL("Couldn't open PNM file '%s' for writing.", fname.c_str());

  PnmWriter::writeGray(image, ofs);

  if (ofs.fail())
    LFATAL("Output stream failure while writing '%s'.", fname.c_str());
}
// ######################################################################
void PnmWriter::writeGray(const Image<uint16>& image,
                          const std::string& fname)
{
  std::ofstream ofs(fname.c_str(), std::ios::binary);
  if (!ofs.is_open())
    LFATAL("Couldn't open PNM file '%s' for writing.", fname.c_str());

  PnmWriter::writeGray(image, ofs);

  if (ofs.fail())
    LFATAL("Output stream failure while writing '%s'.", fname.c_str());
}

// ######################################################################
void PnmWriter::writeRawBW(const Image<byte>& image, const byte thresh,
                           const std::string& fname)
{
  std::ofstream ofs(fname.c_str(), std::ios::binary);
  if (!ofs.is_open())
    LFATAL("Couldn't open PNM file '%s' for writing.", fname.c_str());

  PnmWriter::writeRawBW(image, thresh, ofs);

  if (ofs.fail())
    LFATAL("Output stream failure while writing '%s'.", fname.c_str());
}

// ######################################################################
void PnmWriter::writeAsciiBW(const Image<byte>& image, const byte thresh,
                             const std::string& fname)
{
  std::ofstream ofs(fname.c_str(), std::ios::binary);
  if (!ofs.is_open())
    LFATAL("Couldn't open PNM file '%s' for writing.", fname.c_str());

  PnmWriter::writeAsciiBW(image, thresh, ofs);

  if (ofs.fail())
    LFATAL("Output stream failure while writing '%s'.", fname.c_str());
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // RASTER_PNMWRITER_C_DEFINED

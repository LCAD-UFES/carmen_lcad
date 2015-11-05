/*!@file Raster/PlaintextWriter.C Plaintext image-file writer class; resulting files usable with matlab's 'load' */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/PlaintextWriter.C $
// $Id: PlaintextWriter.C 8312 2007-04-27 22:05:07Z rjpeters $
//

#ifndef RASTER_PLAINTEXTWRITER_C_DEFINED
#define RASTER_PLAINTEXTWRITER_C_DEFINED

#include "Raster/PlaintextWriter.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "Util/log.H"

#include <cstdio>

// ######################################################################
PlaintextWriter::PlaintextWriter()
{}

// ######################################################################
PlaintextWriter::~PlaintextWriter()
{}

// ######################################################################
std::string PlaintextWriter::writeFrame(const GenericFrame& image,
                                        const std::string& fname)
{
  switch (image.nativeType())
    {
    case GenericFrame::NONE:    // fall-through
    case GenericFrame::VIDEO:   // fall-through
    case GenericFrame::RGB_U8:
      writePlaintextRgbU8(image.asRgbU8(), fname);
      break;

    case GenericFrame::RGB_F32:
      writePlaintextRgbF32(image.asRgbF32(), fname);
      break;

    case GenericFrame::GRAY_U8:
      writePlaintextGrayU8(image.asGrayU8(), fname);
      break;

    case GenericFrame::GRAY_F32:
      writePlaintextGrayF32(image.asGrayF32(), fname);
      break;

    default:
      LFATAL("unknown GenericFrame::NativeType value %d",
             int(image.nativeType()));
    }

  return fname;
}

// ######################################################################
void PlaintextWriter::writePlaintextRgbU8(const Image<PixRGB<byte> >& img,
                                          const std::string& fname)
{
  FILE* f = fopen(fname.c_str(), "w");
  if (f == 0)
    LFATAL("couldn't open %s for writing", fname.c_str());

  const int w = img.getWidth();
  const int h = img.getHeight();
  Image<PixRGB<byte> >::const_iterator itr = img.begin();
  for (int y = 0; y < h; ++y)
    {
      const char* sep = "";
      for (int x = 0; x < w; ++x)
        {
          fprintf(f, "%s%3d %3d %3d", sep, (*itr).p[0], (*itr).p[1], (*itr).p[2]);
          sep = "  ";
          ++itr;
        }
      fprintf(f, "\n");
    }

  fclose(f);
}

// ######################################################################
void PlaintextWriter::writePlaintextRgbF32(const Image<PixRGB<float> >& img,
                                           const std::string& fname)
{
  FILE* f = fopen(fname.c_str(), "w");
  if (f == 0)
    LFATAL("couldn't open %s for writing", fname.c_str());

  const int w = img.getWidth();
  const int h = img.getHeight();
  Image<PixRGB<float> >::const_iterator itr = img.begin();
  for (int y = 0; y < h; ++y)
    {
      const char* sep = "";
      for (int x = 0; x < w; ++x)
        {
          fprintf(f, "%s%.9e %.9e %.9e", sep, (*itr).p[0], (*itr).p[1], (*itr).p[2]);
          sep = "  ";
          ++itr;
        }
      fprintf(f, "\n");
    }

  fclose(f);
}

// ######################################################################
void PlaintextWriter::writePlaintextGrayU8(const Image<byte>& img,
                                           const std::string& fname)
{
  FILE* f = fopen(fname.c_str(), "w");
  if (f == 0)
    LFATAL("couldn't open %s for writing", fname.c_str());

  const int w = img.getWidth();
  const int h = img.getHeight();
  Image<byte>::const_iterator itr = img.begin();
  for (int y = 0; y < h; ++y)
    {
      const char* sep = "";
      for (int x = 0; x < w; ++x)
        {
          fprintf(f, "%s%3d", sep, *itr);
          sep = " ";
          ++itr;
        }
      fprintf(f, "\n");
    }

  fclose(f);
}

// ######################################################################
void PlaintextWriter::writePlaintextGrayF32(const Image<float>& img,
                                            const std::string& fname)
{
  FILE* f = fopen(fname.c_str(), "w");
  if (f == 0)
    LFATAL("couldn't open %s for writing", fname.c_str());

  const int w = img.getWidth();
  const int h = img.getHeight();
  Image<float>::const_iterator itr = img.begin();
  for (int y = 0; y < h; ++y)
    {
      const char* sep = "";
      for (int x = 0; x < w; ++x)
        {
          fprintf(f, "%s%.9e", sep, *itr);
          sep = " ";
          ++itr;
        }
      fprintf(f, "\n");
    }

  fclose(f);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // RASTER_PLAINTEXTWRITER_C_DEFINED

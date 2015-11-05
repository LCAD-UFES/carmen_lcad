/*!@file Raster/CcodeWriter.C Writes images as C-language arrays, suitable for copy+paste back into source code */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/CcodeWriter.C $
// $Id: CcodeWriter.C 8334 2007-05-03 23:24:15Z rjpeters $
//

#ifndef RASTER_CCODEWRITER_C_DEFINED
#define RASTER_CCODEWRITER_C_DEFINED

#include "Raster/CcodeWriter.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "Util/log.H"

#include <cctype>
#include <cstdio>

namespace
{
  std::string makeCleanArrayName(const std::string& fname)
  {
    std::string s = "ccode_";
    bool prev_underscore = true;
    for (size_t i = 0; i < fname.size(); ++i)
      {
        if (isalnum(fname[i]))
          {
            s += fname[i];
            prev_underscore = false;
          }
        else if (!prev_underscore)
          {
            s += "_";
            prev_underscore = true;
          }
      }
    return s;
  }

  void dumpPixel(FILE* f, const byte* p)
  { fprintf(f, "%3d", int(*p)); }

  void dumpPixel(FILE* f, const float* p)
  { fprintf(f, "%.9e", *p); }

  void dumpPixel(FILE* f, const PixRGB<byte>* p)
  { fprintf(f, "PixRGB<byte>(%3d, %3d, %3d)",
            int(p->p[0]), int(p->p[1]), int(p->p[2])); }

  void dumpPixel(FILE* f, const PixRGB<float>* p)
  { fprintf(f, "PixRGB<byte>(%.9e, %.9e, %.9e)",
            p->p[0], p->p[1], p->p[2]); }

  const char* pixTypeName(byte*) { return "byte"; }
  const char* pixTypeName(float*) { return "float"; }
  const char* pixTypeName(PixRGB<byte>*) { return "PixRGB<byte>"; }
  const char* pixTypeName(PixRGB<float>*) { return "PixRGB<float>"; }

  template <class T>
  void dumpImage(const Image<T>& img,
                 const std::string& fname)
  {
    FILE* f = fopen(fname.c_str(), "w");
    if (f == 0)
      LFATAL("couldn't open %s for writing", fname.c_str());

    const std::string aname = makeCleanArrayName(fname);

    const int w = img.getWidth();
    const int h = img.getHeight();

    fprintf(f, "const int w_%s = %d;\n", aname.c_str(), w);
    fprintf(f, "const int h_%s = %d;\n", aname.c_str(), h);
    fprintf(f, "const %s %s[%d] = {\n",
            pixTypeName((T*)0), aname.c_str(), w*h);

    typename Image<T>::const_iterator itr = img.begin();
    for (int y = 0; y < h; ++y)
      {
        fprintf(f, "\t/* row %6d */ ", y);
        for (int x = 0; x < w; ++x)
          {
            dumpPixel(f, &itr[0]);
            ++itr;
            if (x == w-1)
              {
                if (y == h-1) fprintf(f, "\n");
                else          fprintf(f, ",\n");
              }
            else              fprintf(f, ", ");
          }
      }

    fprintf(f, "};\n");

    fclose(f);
  }
}

// ######################################################################
CcodeWriter::CcodeWriter()
{}

// ######################################################################
CcodeWriter::~CcodeWriter()
{}

// ######################################################################
std::string CcodeWriter::writeFrame(const GenericFrame& image,
                                    const std::string& fname)
{
  switch (image.nativeType())
    {
    case GenericFrame::NONE:    // fall-through
    case GenericFrame::VIDEO:   // fall-through
    case GenericFrame::RGB_U8:
      writeCcodeRgbU8(image.asRgbU8(), fname);
      break;

    case GenericFrame::RGB_F32:
      writeCcodeRgbF32(image.asRgbF32(), fname);
      break;

    case GenericFrame::GRAY_U8:
      writeCcodeGrayU8(image.asGrayU8(), fname);
      break;

    case GenericFrame::GRAY_F32:
      writeCcodeGrayF32(image.asGrayF32(), fname);
      break;

    default:
      LFATAL("unknown GenericFrame::NativeType value %d",
             int(image.nativeType()));
    }

  return fname;
}

// ######################################################################
void CcodeWriter::writeCcodeRgbU8(const Image<PixRGB<byte> >& img,
                                  const std::string& fname)
{
  dumpImage(img, fname);
}

// ######################################################################
void CcodeWriter::writeCcodeRgbF32(const Image<PixRGB<float> >& img,
                                   const std::string& fname)
{
  dumpImage(img, fname);
}

// ######################################################################
void CcodeWriter::writeCcodeGrayU8(const Image<byte>& img,
                                   const std::string& fname)
{
  dumpImage(img, fname);
}

// ######################################################################
void CcodeWriter::writeCcodeGrayF32(const Image<float>& img,
                                    const std::string& fname)
{
  dumpImage(img, fname);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // RASTER_CCODEWRITER_C_DEFINED

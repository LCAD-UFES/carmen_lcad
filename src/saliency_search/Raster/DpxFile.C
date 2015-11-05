/*!@file Raster/DpxFile.C DPX (Digital Picture Exchange) image file format */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/DpxFile.C $
// $Id: DpxFile.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef RASTER_DPXFILE_C_DEFINED
#define RASTER_DPXFILE_C_DEFINED

#include "Raster/DpxFile.H"
#include "Util/log.H"
#include <cstdio>

DpxFile::DpxFile(const char* fname)
{
  LDEBUG("sizeof(DpxFile::FileHeader) = %" ZU , sizeof(DpxFile::FileHeader));

  FILE* fp = fopen(fname, "r");
  if (fp == 0)
    LFATAL("couldn't open '%s' for reading", fname);

  if (fread(&this->fh, sizeof(this->fh), 1, fp) != 1)
    LFATAL("couldn't read file header from '%s'", fname);

  // bool doswap = false;

  if (this->fh.MagicNumber == 0x53445058)
    // OK, no byte-swapping
    ; //doswap = false;
  else if (this->fh.MagicNumber == 0x58504453)
    {
      // doswap = true;
      LFATAL("byte-swapping not yet implemented for DPX images");
    }
  else
    LFATAL("invalid DPX magic number '%d' in '%s'",
           this->fh.MagicNumber, fname);

#define SHOW(x) LDEBUG("(%s) " #x " = %u", fname, x)
#define SHOWF(x) LDEBUG("(%s) " #x " = %f", fname, x)
#define SHOWS(x,w) LDEBUG("(%s) " #x " = '%." #w "s'", fname, x)

  SHOW(this->fh.MagicNumber);
  SHOW(this->fh.ImageDataOffset);
  SHOW(this->fh.FileSize);
  SHOW(this->fh.DittoKey);
  SHOW(this->fh.GenericHeaderSize);
  SHOW(this->fh.IndustryHeaderSize);
  SHOW(this->fh.UserDefinedDataSize);
  SHOW(this->fh.EncryptKey);

  SHOWS(this->fh.HeaderVersion, 8);
  SHOWS(this->fh.ImageFilename, 100);
  SHOWS(this->fh.CreateTime, 23);
  SHOWS(this->fh.Creator, 100);
  SHOWS(this->fh.Project, 200);
  SHOWS(this->fh.Copyright, 200);

  LDEBUG("sizeof(DpxFile::ImageHeader) = %" ZU ,
         sizeof(DpxFile::ImageHeader));

  if (fread(&this->ih, sizeof(this->ih), 1, fp) != 1)
    LFATAL("couldn't read image header from '%s'", fname);

  SHOW(this->ih.Orientation);
  SHOW(this->ih.NumberElements);
  SHOW(this->ih.PixelsPerLine);
  SHOW(this->ih.LinesPerElement);
  SHOW(this->ih.ImageElement[0].DataSign);
  SHOW(this->ih.ImageElement[0].LowData);
  SHOWF(this->ih.ImageElement[0].LowQuantity);
  SHOW(this->ih.ImageElement[0].HighData);
  SHOWF(this->ih.ImageElement[0].HighQuantity);
  SHOW(this->ih.ImageElement[0].Descriptor);
  SHOW(this->ih.ImageElement[0].Transfer);
  SHOW(this->ih.ImageElement[0].Colorimetric);
  SHOW(this->ih.ImageElement[0].BitSize);
  SHOW(this->ih.ImageElement[0].Packing);
  SHOW(this->ih.ImageElement[0].Encoding);
  SHOW(this->ih.ImageElement[0].DataOffset);
  SHOW(this->ih.ImageElement[0].EndOfLinePadding);
  SHOW(this->ih.ImageElement[0].EndOfImagePadding);
  SHOWS(this->ih.ImageElement[0].Description, 32);

  LDEBUG("sizeof(DpxFile::OrientationHeader) = %" ZU ,
         sizeof(DpxFile::OrientationHeader));

  if (fread(&this->oh, sizeof(this->oh), 1, fp) != 1)
    LFATAL("couldn't read orientation header from '%s'", fname);

  SHOW(this->oh.XOffset);
  SHOW(this->oh.YOffset);
  SHOWF(this->oh.XCenter);
  SHOWF(this->oh.YCenter);
  SHOW(this->oh.XOriginalSize);
  SHOW(this->oh.YOriginalSize);
  SHOWS(this->oh.FileName, 100);
  SHOWS(this->oh.TimeDate, 24);
  SHOWS(this->oh.InputName, 32);
  SHOWS(this->oh.InputSN, 32);
  SHOW(this->oh.Border[0]);
  SHOW(this->oh.Border[1]);
  SHOW(this->oh.Border[2]);
  SHOW(this->oh.Border[3]);
  SHOW(this->oh.AspectRatio[0]);
  SHOW(this->oh.AspectRatio[1]);

  LDEBUG("sizeof(DpxFile::FilmIndustryHeader) = %" ZU ,
         sizeof(DpxFile::FilmIndustryHeader));

  if (fread(&this->fih, sizeof(this->fih), 1, fp) != 1)
    LFATAL("couldn't read film industry header from '%s'", fname);

  SHOWS(this->fih.FilmMfgId, 2);
  SHOWS(this->fih.FilmType, 2);
  SHOWS(this->fih.Offset, 2);
  SHOWS(this->fih.Prefix, 6);
  SHOWS(this->fih.Count, 4);
  SHOWS(this->fih.Format, 32);
  SHOW(this->fih.FramePosition);
  SHOW(this->fih.SequenceLen);
  SHOW(this->fih.HeldCount);
  SHOWF(this->fih.FrameRate);
  SHOWF(this->fih.ShutterAngle);
  SHOWS(this->fih.FrameId, 32);
  SHOWS(this->fih.SlateInfo, 100);

  LDEBUG("sizeof(DpxFile::TelevisionIndustryHeader) = %" ZU ,
         sizeof(DpxFile::TelevisionIndustryHeader));

  if (fread(&this->tih, sizeof(this->tih), 1, fp) != 1)
    LFATAL("couldn't read television industry header from '%s'", fname);

  SHOW(this->tih.TimeCode);
  SHOW(this->tih.UserBits);
  SHOW(this->tih.Interlace);
  SHOW(this->tih.FiledNumber);
  SHOW(this->tih.VideoSignal);
  SHOW(this->tih.Padding);
  SHOWF(this->tih.HorzSampleRate);
  SHOWF(this->tih.VertSampleRate);
  SHOWF(this->tih.FrameRate);
  SHOWF(this->tih.TimeOffset);
  SHOWF(this->tih.Gamma);
  SHOWF(this->tih.BlackLevel);
  SHOWF(this->tih.BlackGain);
  SHOWF(this->tih.Breakpoint);
  SHOWF(this->tih.WhiteLevel);
  SHOWF(this->tih.IntegrationTimes);

  if (fseek(fp, this->fh.ImageDataOffset, SEEK_SET) != 0)
    LFATAL("couldn't fseek() to start of image data in '%s'", fname);

  ASSERT(this->ih.PixelsPerLine > 0);
  ASSERT(this->ih.LinesPerElement > 0);

  this->dims = Dims(this->ih.PixelsPerLine, this->ih.LinesPerElement);
  this->rawimage = Image<uint16>(this->dims, NO_INIT);

  ASSERT(this->ih.ImageElement[0].BitSize == 16);
  ASSERT(this->ih.ImageElement[0].EndOfLinePadding == 0xFFFF);

  if (fread(this->rawimage.getArrayPtr(),
            sizeof(*(this->rawimage.getArrayPtr())),
            this->dims.sz(), fp)
      != size_t(this->dims.sz()))
    LFATAL("couldn't read image data from '%s'", fname);

  fclose(fp);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // RASTER_DPXFILE_C_DEFINED

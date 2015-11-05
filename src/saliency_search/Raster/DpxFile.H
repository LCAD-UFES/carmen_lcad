/*!@file Raster/DpxFile.H DPX (Digital Picture Exchange) image file format */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/DpxFile.H $
// $Id: DpxFile.H 9204 2008-02-01 18:46:27Z rjpeters $
//

#ifndef RASTER_DPXFILE_H_DEFINED
#define RASTER_DPXFILE_H_DEFINED

#include "Image/Dims.H"
#include "Image/Image.H"
#include "Util/Types.H"

/// DPX (Digital Picture Exchange) image file format
/** NOTE: This class currently contains only a partial DPX file parser
    and does NOT provide full support for all DPX files.

    For details, see http://www.fileformat.info/format/dpx/
*/
class DpxFile
{
public:
  DpxFile(const char* fname);

  struct FileHeader
  {
    uint32 MagicNumber;
    uint32 ImageDataOffset;
    char HeaderVersion[8];
    uint32 FileSize;
    uint32 DittoKey;
    uint32 GenericHeaderSize;
    uint32 IndustryHeaderSize;
    uint32 UserDefinedDataSize;
    char ImageFilename[100];
    char CreateTime[24];
    char Creator[100];
    char Project[200];
    char Copyright[200];
    uint32 EncryptKey;
    char Reserved[104];
  };

  struct ImageElementInfo
  {
    uint32 DataSign;
    uint32 LowData;
    float LowQuantity;
    uint32 HighData;
    float HighQuantity;
    byte Descriptor;
    byte Transfer;
    byte Colorimetric;
    byte BitSize;
    uint16 Packing;
    uint16 Encoding;
    uint32 DataOffset;
    uint32 EndOfLinePadding;
    uint32 EndOfImagePadding;
    char Description[32];
  };

  struct ImageHeader
  {
    uint16 Orientation;
    uint16 NumberElements;
    uint32 PixelsPerLine;
    uint32 LinesPerElement;
    ImageElementInfo ImageElement[8];
    byte Reserved[52];
  };

  struct OrientationHeader
  {
    uint32 XOffset;
    uint32 YOffset;
    float XCenter;
    float YCenter;
    uint32 XOriginalSize;
    uint32 YOriginalSize;
    char FileName[100];
    char TimeDate[24];
    char InputName[32];
    char InputSN[32];
    uint16 Border[4];
    uint32 AspectRatio[2];
    byte Reserved[28];
  };

  struct FilmIndustryHeader
  {
    char FilmMfgId[2];
    char FilmType[2];
    char Offset[2];
    char Prefix[6];
    char Count[4];
    char Format[32];
    uint32 FramePosition;
    uint32 SequenceLen;
    uint32 HeldCount;
    float FrameRate;
    float ShutterAngle;
    char FrameId[32];
    char SlateInfo[100];
    byte Reserved[56];
  };

  struct TelevisionIndustryHeader
  {
    uint32 TimeCode;
    uint32 UserBits;
    byte Interlace;
    byte FiledNumber;
    byte VideoSignal;
    byte Padding;
    float HorzSampleRate;
    float VertSampleRate;
    float FrameRate;
    float TimeOffset;
    float Gamma;
    float BlackLevel;
    float BlackGain;
    float Breakpoint;
    float WhiteLevel;
    float IntegrationTimes;
    byte Reserved[76];
  };

  Dims dims;

  FileHeader fh;
  ImageHeader ih;
  OrientationHeader oh;
  FilmIndustryHeader fih;
  TelevisionIndustryHeader tih;

  Image<uint16> rawimage;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // RASTER_DPXFILE_H_DEFINED

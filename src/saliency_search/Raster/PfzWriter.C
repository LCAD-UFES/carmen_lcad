/*!@file Raster/PfzWriter.C Write pfm image files */

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
// Primary maintainer for this file: T. Nathan Mundenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Raster/PfzWriter.C $
// $Id: PfzWriter.C 14290 2010-12-01 21:44:03Z itti $
//

#ifndef RASTER_PFMWRITER_C_DEFINED
#define RASTER_PFMWRITER_C_DEFINED

#include "Raster/PfzWriter.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"

#include <fstream>
#include <iostream> // for std::cerr
#include <sstream>

PfzWriter::PfzWriter() {}

PfzWriter::~PfzWriter() {}

std::deque<std::string> PfzWriter::itsTagName(0);
std::deque<std::string> PfzWriter::itsTagValue(0);
z_stream                PfzWriter::zstrm;
unsigned char           PfzWriter::buff_uin[PFZ_CHUNK];
unsigned char           PfzWriter::buff_uout[PFZ_CHUNK];
int                     PfzWriter::itsCompressionLevel(PFZ_LEVEL);

std::string PfzWriter::writeFrame(const GenericFrame& image,
                                  const std::string& fname)
{
  switch (image.nativeType())
    {
    case GenericFrame::NONE:
      LFATAL("cannot write an empty image to '%s'", fname.c_str());
      break;

    case GenericFrame::RGB_U8:
    case GenericFrame::RGBD:
      LFATAL("cannot write an RGB byte image '%s' in PFZ format", fname.c_str());
      break;

    case GenericFrame::RGB_U16:
      LFATAL("cannot write an RGB byte image '%s' in PFZ format", fname.c_str());
      break;

    case GenericFrame::RGB_F32:
      LFATAL("cannot write an RGB float image '%s' in PFZ format", fname.c_str());
      break;

    case GenericFrame::GRAY_U8:
      PfzWriter::writeFloat(image.asFloat(), fname);
      break;

    case GenericFrame::GRAY_U16:
      LFATAL("cannot write an 16 bit gray image '%s' in PFZ format", fname.c_str());
      break;

    case GenericFrame::GRAY_F32:
      PfzWriter::writeFloat(image.asFloat(), fname);
      break;

    case GenericFrame::VIDEO:
      LFATAL("cannot write a video frame '%s' in PFZ format", fname.c_str());
      break;
    }

  return fname;
}

void PfzWriter::writeFloat(const Image<float>& image,
                           const std::string& fname,
                           const std::deque<std::string> tagName,
                           const std::deque<std::string> tagValue)
{
  itsTagName  = tagName;
  itsTagValue = tagValue;
  writeFloat(image,fname,true);
}

void PfzWriter::writeFloat(const Image<float>& image,
                           const std::string& fname,
                           const bool useTag)
{
  int flush = 0;

  /* allocate deflate state */
  zstrm.zalloc   = Z_NULL;
  zstrm.zfree    = Z_NULL;
  zstrm.opaque   = Z_NULL;

  //
  if((itsCompressionLevel < -1) || (itsCompressionLevel > 9))
    LFATAL("Invalid compression level '%d' given for pfz image '%s'",
           itsCompressionLevel,fname.c_str());

  const int retI = deflateInit(&zstrm, itsCompressionLevel);

  if (retI != Z_OK)
    LFATAL("Unable to allocate memory and set up zlib compression in PFZ '%s'",
           fname.c_str());

  // first we take the image into a standard stream for formating

  std::stringstream os;
  std::ofstream ofs(fname.c_str(), std::ios::binary);
  if (!ofs.is_open())
    LFATAL("Couldn't open PFZ file '%s' for writing.", fname.c_str());

  ofs << "PZ\n"
      << "# PFZ image - Mundhenk, Itti (C) 2006\n";
  os  << image.getWidth() << ' ' << image.getHeight()
      << "\n1.0\n"
      << "!Image Info\n"
      << "PFZ Image compressed floating point image for INVT\n"
      << "!URL\n"
      << "http://www.nerd-cam.com\n"
      << "!Compression Level\n"
      << itsCompressionLevel << "\n";

  // write extra tags if supplied, these will be compressed
  if(useTag)
  {
    std::deque<std::string>::iterator tagNameItr  = itsTagName.begin();
    std::deque<std::string>::iterator tagValueItr = itsTagValue.begin();
    while(tagNameItr != itsTagName.end())
    {
      os << "!" << *tagNameItr << "\n" << *tagValueItr << "\n";
      ++tagNameItr; ++tagValueItr;
    }
  }

  os << " ";

  // push the image into the sstream
  os.write(reinterpret_cast<const char*>(image.getArrayPtr()),
            image.getSize() * sizeof(float));

  // sync the string stream buffers
  os.flush();

  // buffer each PFZ_CHUNK of the image stream into zlibs stream
  while(os.good())
  {
    os.read(reinterpret_cast<char*>(buff_uin),PFZ_CHUNK);
    zstrm.avail_in = os.gcount();
    /*
    unsigned char *uin = &buff_uin[0];
    char           *in = &buff_in[0];
    for(uint i = 0; i < PFZ_CHUNK; i++)
    {
      *uin = clamped_convert<byte>(*in);
      ++uin; ++in;
    }
    */
    zstrm.next_in = buff_uin;
    // for each in chunk, compress them, we use this inner loop
    // because zlib may not compress the entire chunk in one go and may
    // need to take it in several pieces
    // avail_out is set via pointer to 0 when we are done
    zstrm.avail_out = 0;

    if(os.good())
      flush = Z_NO_FLUSH;
    else
      flush = Z_FINISH;

    while(zstrm.avail_out == 0)
    {
      zstrm.avail_out = PFZ_CHUNK;
      zstrm.next_out  = buff_uout;
      const int ret   = deflate(&zstrm, flush);

      if(ret == Z_STREAM_ERROR)
        LFATAL("Error in PFZ zlib compression stream of image '%s'",
               fname.c_str());

      const uint have = PFZ_CHUNK - zstrm.avail_out;
      /*
      unsigned char *uout = &buff_uout[0];
      char           *out = &buff_out[0];
      for(uint i = 0; i < have; i++)
      {
        *out = clamped_convert<char>(*uout);
        ++uout; ++out;
      }
      */
      ofs.write(reinterpret_cast<char*>(buff_uout),have);
      if (ofs.fail())
        LFATAL("Output stream failure while writing '%s'.", fname.c_str());
    }
  }
  ofs.close();
  deflateEnd(&zstrm);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // RASTER_PFMWRITER_C_DEFINED

/*!@file Media/SequenceFileStream.C Read frames from .seq video files */

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
// Primary maintainer for this file: Rand Voorhies <voorhies at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/SequenceFileStream.C $
// $Id: SequenceFileStream.C 15310 2012-06-01 02:29:24Z itti $
//

#include "Media/SequenceFileStream.H"
#include "Component/ModelOptionDef.H"
#include "Raster/DeBayer.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "Transport/TransportOpts.H"   // for MOC_INPUT
#include "Util/log.H"
#include "rutz/trace.h"

#include <stdio.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>

const ModelOptionDef OPT_SequenceFileStreamUseMmap =
  { MODOPT_ARG(bool), "SequenceFileStreamUseMmap", &MOC_INPUT, OPTEXP_CORE,
    "Whether to use mmap() instead of read() to stream data from disk. The"
    "default is to use mmap(), which may be significantly faster, but may"
    "be somewhat less portable.",
    "seqfile-use-mmap", '\0', "<true|false>", "false" };

// ######################################################################
SequenceFileStream::SequenceFileStream(OptionManager& mgr,
    const std::string& descrName, const std::string& tagName) :
  FrameIstream(mgr, descrName, tagName),
  itsUseMmap(&OPT_SequenceFileStreamUseMmap, this),
  itsFrame(), itsFrameSpec(), itsFrameSpecValid(false), itsFileHandle(-1),
  itsMmapFile()
{ }

// ######################################################################
SequenceFileStream::~SequenceFileStream()
{ }

// ######################################################################
void SequenceFileStream::setConfigInfo(const std::string& filename)
{
  this->setFileName(filename);
}


// ######################################################################
GenericFrameSpec SequenceFileStream::peekFrameSpec()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (!itsFrameSpecValid)
  {
    if (itsFrame.initialized() == false) itsFrame = readFrame();

    itsFrameSpec = itsFrame.frameSpec();
    itsFrameSpecValid = true;
  }

  return itsFrameSpec;
}

// ######################################################################
void SequenceFileStream::setFileName(const std::string& fname)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // Open the file
  if (itsFileHandle != -1) close(itsFileHandle);

#ifdef HAVE_O_LARGEFILE
  itsFileHandle = open(fname.c_str(), O_LARGEFILE | O_RDONLY);
#else
  itsFileHandle = open(fname.c_str(), O_RDONLY);
#endif

  if(itsFileHandle == -1) PLFATAL("Could Not Open File %s", fname.c_str());

  int32_t header[146];
  ssize_t headerSize;

  // Read and parse the file header
  headerSize = read(itsFileHandle, &header, (size_t)(sizeof(int32_t) * 146) );

  // Make sure then entire header was read
  if (headerSize != size_t(sizeof(int32_t) * 146) )
    LFATAL("Could not fully read .seq file header. File: %s", fname.c_str());

  // Check the version number on the header
  if(header[7] != 3)
    LFATAL("Wrong version number found in .seq file header (%d != 3).", header[7]);

  // Check the magic number on the header
  if(header[0] != 0xfeed)
    LFATAL("Missing magic number from .seq file header (%x != 0xfeed).", header[0]);

  if(header[139] != 8 || header[140] != 8 || (header[142] != 100 && header[142] != 101))
    LFATAL("seq file in a format other than 8-bit bayer\n");

  itsDataOffset      = header[8];    // need for seek
  itsWidth           = header[137];  // stored image dimensions, for read
  itsHeight          = header[138];
  itsFrameCount      = header[143];  // number of frames
  itsFirstFrame      = header[144];  // start number, always 0?
  itsTrueImageSize   = header[145];  // need for seek
  itsImageSizeBytes  = header[141];  // amount to read

  LINFO("Opening Sequence File with %d Frames", itsFrameCount);

  // do we want to use mmap'd I/O?
  if (itsUseMmap.getVal()) {
    // close the file and instead get a mapped_file going:
    close(itsFileHandle); itsFileHandle = -1;
    itsMmapFile.reset(new rutz::mapped_infile(fname.c_str()));
  }
}

bool SequenceFileStream::setFrameNumber(int n)
{
  if ((unsigned int)n > itsFrameCount) return false;

  itsFrameOffset = n;
  return true;
}

// ######################################################################
GenericFrame SequenceFileStream::readFrame()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (itsUseMmap.getVal()) { if (itsMmapFile.is_invalid()) LFATAL("File not mmap'd"); }
  else { if (itsFileHandle == -1) LFATAL("File not open"); }

  GenericFrame ret;

  // do we already have a frame because we peeked its specs?
  if (itsFrame.initialized()) {
    ret = itsFrame;
    itsFrame = GenericFrame();
  } else {
    // Calculate the desired seek position
    off_t seekPosition = itsDataOffset + (itsFirstFrame + itsFrameOffset) * itsTrueImageSize;

    if (itsUseMmap.getVal()) {
      // use mmap I/O. In fact, we don't read anything proper until we
      // call deBayer() which will automatically swap in data as it
      // processes the pixels. Yeah the attach() below is hacky with
      // the casting, we need to remember that the mmap'd memory is
      // read-only so we should not try to modify any pixel in that
      // temporary image:

      //FIXME: this has issues:
      //      Image<byte> img;
      //      img.attach(const_cast<byte*>(reinterpret_cast<const byte *>(itsMmapFile->memory())) +
      //                 seekPosition, itsWidth, itsHeight);

      //FIXME: I believe this makes a deep copy...
      Image<byte> img(reinterpret_cast<const byte *>(itsMmapFile->memory()) + seekPosition, itsWidth, itsHeight);

      // Debayer the image to make it RGB; this will swap in the
      // necessary pages as the memory is being accessed by the
      // dBayer function:
      Image<PixRGB<byte> > frame = deBayer(img, BAYER_GRBG);
      ret = GenericFrame(frame);

      //img.detach();
    } else {
      // use regular I/O; Seek to the desired frame
#ifdef HAVE_O_LARGEFILE
      lseek64(itsFileHandle, seekPosition, SEEK_SET);
#else
      lseek(itsFileHandle, seekPosition, SEEK_SET);
#endif

      // Read the frame as raw bytes
      Image<byte> tempImage(itsWidth, itsHeight, NO_INIT);

      size_t numRead = read(itsFileHandle, tempImage.getArrayPtr(), itsWidth * itsHeight);

      if (numRead <= 0) { LERROR("Frame Not Read From Sequence File"); return GenericFrame(); }

      // Debayer the image to make it RGB
      Image<PixRGB<byte> > frame = deBayer(tempImage, BAYER_GRBG);
      ret = GenericFrame(frame);
    }
    // ready for next frame:
    ++itsFrameOffset;
  }

  return ret;
}



/*!@file Media/MpegInputStream.C Read frames from movie files */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/MpegInputStream.C $
// $Id: MpegInputStream.C 13800 2010-08-18 20:58:25Z dberg $
//

#ifndef MEDIA_MPEGINPUTSTREAM_C_DEFINED
#define MEDIA_MPEGINPUTSTREAM_C_DEFINED

#include "Media/MpegInputStream.H"

#include "Component/ModelOptionDef.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Media/MediaOpts.H"
#include "Media/MovieDecoder.H"
#include "Raster/GenericFrame.H"
#include "Transport/TransportOpts.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include "Video/VideoFrame.H"
#include "rutz/trace.h"

#if defined(INVT_HAVE_AVFORMAT)
// We need to be sure to #include <ffmpeg/avformat.h> before we try to
// test LIBAVFORMAT_BUILD:
#  define attribute_deprecated
extern "C"
{
#ifdef HAVE_LIBAVFORMAT_AVFORMAT_H
  #include <libavformat/avformat.h>
#else
  #ifdef HAVE_FFMPEG_AVFORMAT_H
    #include <ffmpeg/avformat.h>
  #endif
#endif
}
#  if defined(LIBAVFORMAT_BUILD) && (LIBAVFORMAT_BUILD >= 4610)
#    include "Media/FfmpegPacketDecoder.H"
#  else
#    include "Media/FfmpegDecoder.H"
#  endif
#endif

#ifdef HAVE_QUICKTIME_QUICKTIME_H
#  include "Media/QuartzQuickTimeDecoder.H"
#endif

#include <cstdio>
#include <string>

// private command-line option defs

// Used by: InputMPEGStream
static const ModelOptionDef OPT_InputMPEGStreamCodec =
  { MODOPT_ARG_STRING, "InputMPEGStreamCodec", &MOC_INPUT, OPTEXP_CORE,
    "Type of video input codec to use (use value 'List' to see list of "
    "available codecs on your system, or 'Auto' to automatically determine "
    "which codec to use)",
    "input-codec", '\0', "<Auto|List|name>", "Auto" };

// ######################################################################
InputMPEGStream::InputMPEGStream(OptionManager& mgr,
                                 const std::string& descrName,
                                 const std::string& tagName)
  :
  FrameIstream(mgr, descrName, tagName),
  itsBuffLen(tagName+"BuffLen", this, 65536),
  itsCodecName(&OPT_InputMPEGStreamCodec, this),
  itsDoPreload(&OPT_InputMPEGStreamPreload, this),
  itsFileName(""),
  dec(0)
{}

// ######################################################################
InputMPEGStream::~InputMPEGStream()
{
  delete dec;
}

// ######################################################################
void InputMPEGStream::stop2()
{
  delete dec;
  dec = 0;
}

// ######################################################################
void InputMPEGStream::setConfigInfo(const std::string& filename)
{
  // NOTE: if you modify any behavior here, then please update the
  // corresponding documentation for the global "--in" option inside
  // the OPT_InputFrameSource definition in Media/MediaOpts.C

  this->setFileName(filename);
}

// ######################################################################
bool InputMPEGStream::setFrameNumber(const int n)
{
  createDecoder();

  LDEBUG("external/internal frame# %d/%d", n, dec->apparentFrameNumber());

  return dec->setFrameNumber(n);
}

// ######################################################################
GenericFrameSpec InputMPEGStream::peekFrameSpec()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  createDecoder();
  ASSERT(dec != 0);

  return dec->peekFrameSpec();
}

// ######################################################################
void InputMPEGStream::setFileName(std::string fname)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // if we already had an open decoder, nuke it:
  if (dec != 0) { delete dec; dec = 0; }

  itsFileName = fname;
}

// ######################################################################
GenericFrame InputMPEGStream::readFrame()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  createDecoder();
  ASSERT(dec != 0);

  return GenericFrame(dec->readVideoFrame());
}

// ######################################################################
Image<PixRGB<byte> > InputMPEGStream::readRGB()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  createDecoder();
  ASSERT(dec != 0);

  return dec->readRGB();
}

// ######################################################################
bool InputMPEGStream::readAndDiscardFrame()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  createDecoder();
  ASSERT(dec != 0);

  return dec->readAndDiscardFrame();
}

// ######################################################################
VideoFrame InputMPEGStream::readVideoFrame()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  createDecoder();
  ASSERT(dec != 0);

  return dec->readVideoFrame();
}

// ######################################################################
void InputMPEGStream::createDecoder()
{
GVX_TRACE(__PRETTY_FUNCTION__);

  if (dec == 0)
    {
      if (itsFileName.length() == 0)
        {
          LFATAL("no input mpeg filename specified");
        }

#if defined(INVT_HAVE_AVFORMAT)
#if defined(LIBAVFORMAT_BUILD) && (LIBAVFORMAT_BUILD >= 4610)
      dec = new FfmpegPacketDecoder(itsFileName.c_str(),
                                    itsDoPreload.getVal());
#else
      // this is the old stream-based FfmpegDecoder, which doesn't
      // handle mpeg-4 streams robustly
      dec = new FfmpegDecoder(itsCodecName.getVal().c_str(),
                              itsBuffLen.getVal(),
                              itsFileName.c_str(),
                              itsDoPreload.getVal());
#endif
#elif defined(HAVE_QUICKTIME_QUICKTIME_H)
      dec = new QuartzQuickTimeDecoder(itsFileName.c_str());
#else
      LFATAL("you must have an movie-reading library (such as ffmpeg or QuickTime) "
             "installed in order to be able to read movie files");
#endif
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_MPEGINPUTSTREAM_C_DEFINED

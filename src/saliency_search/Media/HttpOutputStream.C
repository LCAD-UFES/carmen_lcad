/*!@file Media/HttpOutputStream.C Write frames to movie files */

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
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/HttpOutputStream.C $
// $Id: HttpOutputStream.C 12962 2010-03-06 02:13:53Z irock $
//

#ifndef MEDIA_HTTPOUTPUTSTREAM_C_DEFINED
#define MEDIA_HTTPOUTPUTSTREAM_C_DEFINED

#include "Media/HttpOutputStream.H"


#include "Component/ModelOptionDef.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Media/HttpEncoder.H"
#include "Raster/GenericFrame.H"
#include "Transport/TransportOpts.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include "rutz/shared_ptr.h"
#include "rutz/trace.h"

#include <map>
#include <string>

#ifdef INVT_HAVE_AVCODEC
// private command-line option defs

// Used by: OutputHttpStream
const ModelOptionDef OPT_OutputHttpStreamFrameRate =
  { MODOPT_ARG(int), "OutputHttpStreamFrameRate", &MOC_OUTPUT, OPTEXP_CORE,
    "The frame rate, in frames-per-second, for output mpeg streams",
    "output-http-framerate", '\0', "<frames-per-second>", "25" };

// Used by: OutputStream
const ModelOptionDef OPT_OutputHttpStreamBitRate =
  { MODOPT_ARG(int), "OutputHttpStreamBitRate", &MOC_OUTPUT, OPTEXP_CORE,
    "The bit rate for output mpeg streams",
    // FIXME it's not clear from the ffmpeg docs whether the bitrate
    // is bits-per-second or bits-per-frame
    "output-http-bitrate", '\0', "<bits-per-?>", "750000" };

// Used by: OutputHttpStream
const ModelOptionDef OPT_OutputHttpStreamBufSize =
  { MODOPT_ARG(ByteCount), "OutputHttpStreamBufSize", &MOC_OUTPUT, OPTEXP_CORE,
    "The size, in bytes, of the per-frame output buffer for output "
    "mpeg streams (you can try increasing the buffer size if you "
    "are seeing warnings like 'encoded frame too large')",
    "output-http-bufsize", '\0', "<bytes>", "2MiB" };

// Used by: OutputHttpStream
static const ModelOptionDef OPT_OutputHttpStreamCodec =
  { MODOPT_ARG_STRING, "OutputHttpStreamCodec", &MOC_OUTPUT, OPTEXP_CORE,
    "Type of video output codec to use (use value 'List' to see list of "
    "available codecs on your system)",
    "output-http-codec", '\0', "<List|name>", "mpeg1video" };

// ######################################################################
OutputHttpStream::OutputHttpStream(OptionManager& mgr,
                                   const std::string& descrName,
                                   const std::string& tagName)
  :
  LowLevelEncoderMap(mgr, descrName, tagName),
        itsHttpServer(new HttpServer(mgr)),
  itsFrameRate(&OPT_OutputHttpStreamFrameRate, this),
  itsFrameRateBase(tagName+"FrameRateBase", this, 1),
  itsBitRate(&OPT_OutputHttpStreamBitRate, this),
  itsBufSize(&OPT_OutputHttpStreamBufSize, this),
  itsCodecName(&OPT_OutputHttpStreamCodec, this),
  itsStem()
{
        addSubComponent(itsHttpServer);
}

// ######################################################################
void OutputHttpStream::setConfigInfo(const std::string& filestem)
{
  // NOTE: if you modify any behavior here, then please update the
  // corresponding documentation for the global "--out" option inside
  // the OPT_OutputFrameSink definition in Media/MediaOpts.C

  this->setFileStem(filestem);
}

// ######################################################################
void OutputHttpStream::setFileStem(const std::string& s)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsStem = s;
}


#include "Media/MediaOpts.H"
#include "Media/FrameRange.H"
#include "Component/ModelManager.H"

// ######################################################################
rutz::shared_ptr<LowLevelEncoder>
OutputHttpStream::makeEncoder(const GenericFrameSpec& spec,
                              const std::string& shortname,
                              const FrameInfo& auxinfo)
{
  // NOTE: for now, we just ignore the FrameInfo auxinfo here; could
  // we use it to embed a comment in the mpeg file, though?

  return rutz::shared_ptr<HttpEncoder>
    (new HttpEncoder(itsHttpServer,
                     itsStem+shortname,
                     itsCodecName.getVal(),
                     itsBitRate.getVal(),
                     itsFrameRate.getVal(),
                     itsFrameRateBase.getVal(),
                     spec.dims,
                     itsBufSize.getVal().bytes()));
}

#endif // HAVE_FFMPEG_AVCODEC_H

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_HTTPOUTPUTSTREAM_C_DEFINED

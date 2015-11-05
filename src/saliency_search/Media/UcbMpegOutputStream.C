/*!@file Media/UcbMpegOutputStream.C Write mpeg video frames using mpeg_encode/ppmtompeg */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/UcbMpegOutputStream.C $
// $Id: UcbMpegOutputStream.C 8905 2007-10-25 23:21:41Z rjpeters $
//

#ifndef MEDIA_UCBMPEGOUTPUTSTREAM_C_DEFINED
#define MEDIA_UCBMPEGOUTPUTSTREAM_C_DEFINED

#include "Media/UcbMpegOutputStream.H"

#include "Component/ModelOptionDef.H"
#include "Media/UcbMpegEncoder.H"
#include "Raster/GenericFrame.H"
#include "Transport/TransportOpts.H"
#include "Util/Assert.H"
#include "Util/FileUtil.H"
#include "Util/log.H"
#include "rutz/shared_ptr.h"
#include "rutz/trace.h"

#include <map>
#include <string>

// Used by: OutputMPEGStream
const ModelOptionDef OPT_UcbMpegFrameRate =
  { MODOPT_ARG(SimTime), "UcbMpegFrameRate", &MOC_OUTPUT, OPTEXP_CORE,
    "The frame rate, in frames-per-second, for ucb mpeg streams",
    "ucbmpeg-framerate", '\0',
    "<23.976, 24, 25, 29.97, 30, 50, 59.94, 60>Hz", "29.97Hz" };

// Used by: OutputMPEGStream
const ModelOptionDef OPT_UcbMpegQuality =
  { MODOPT_ARG_STRING, "UcbMpegQuality", &MOC_OUTPUT, OPTEXP_CORE,
    "The baseline quality settings for ucb mpeg streams (these "
    "settings may be further refined by additional command-line "
    "options)",
    "ucbmpeg-quality", '\0',
    "<basic|hq|superhq>", "basic" };

// ######################################################################
UcbMpegOutputStream::UcbMpegOutputStream(OptionManager& mgr,
                                   const std::string& descrName,
                                   const std::string& tagName)
  :
  LowLevelEncoderMap(mgr, descrName, tagName),
  itsFrameRate(&OPT_UcbMpegFrameRate, this),
  itsBaselineQuality(&OPT_UcbMpegQuality, this),
  itsStem()
{}

// ######################################################################
void UcbMpegOutputStream::setConfigInfo(const std::string& filestem)
{
  // NOTE: if you modify any behavior here, then please update the
  // corresponding documentation for the global "--out" option inside
  // the OPT_OutputFrameSink definition in Media/MediaOpts.C

  this->setFileStem(filestem);
}

// ######################################################################
void UcbMpegOutputStream::setFileStem(const std::string& s)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsStem = s;
}

// ######################################################################
rutz::shared_ptr<LowLevelEncoder>
UcbMpegOutputStream::makeEncoder(const GenericFrameSpec& spec,
                                 const std::string& shortname,
                                 const FrameInfo& auxinfo)
{
  UcbMpegParams params;

  if (itsBaselineQuality.getVal().compare("basic") == 0)
    params = UcbMpegParams::basic();
  else if (itsBaselineQuality.getVal().compare("hq") == 0)
    params = UcbMpegParams::hq();
  else if (itsBaselineQuality.getVal().compare("superhq") == 0)
    params = UcbMpegParams::superhq();
  else
    LFATAL("invalid setting for --ucbmpeg-quality: %s",
           itsBaselineQuality.getVal().c_str());

  std::string fname = itsStem + shortname;

  if (!hasExtension(fname, ".mpg") &&
      !hasExtension(fname, ".mpeg"))
    fname += ".mpg";

#ifndef MPEGENCODE_PROG
  LFATAL("MPEGENCODE_PROG is undefined (the configure script "
         "did not find an mpeg_encode or ppmtompeg program)");
  /* can't happen */ return rutz::shared_ptr<LowLevelEncoder>();
#else

  // NOTE: for now, we just ignore the FrameInfo auxinfo here; could
  // we use it to embed a comment in the mpeg file, though?

  return rutz::shared_ptr<LowLevelEncoder>
    (new UcbMpegEncoder(MPEGENCODE_PROG, fname, params,
                        itsFrameRate.getVal().hertz()));
#endif
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_UCBMPEGOUTPUTSTREAM_C_DEFINED

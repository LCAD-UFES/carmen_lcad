/*!@file Media/HttpOutputStream.H Write frames to movie files */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/HttpOutputStream.H $
// $Id: HttpOutputStream.H 12962 2010-03-06 02:13:53Z irock $
//

#ifndef MEDIA_HTTPOUTPUTSTREAM_H_DEFINED
#define MEDIA_HTTPOUTPUTSTREAM_H_DEFINED

#ifdef INVT_HAVE_AVCODEC
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Transport/LowLevelEncoderMap.H"
#include "Devices/HTTPServer.H"
#include "Util/ByteCount.H"
#include "Util/Types.H"      // for byte

// ######################################################################
// ######################################################################
//! OutputHttpStream writes mpeg video frames using the avcodec library through http
class OutputHttpStream : public LowLevelEncoderMap
{
public:
  //! Construct an mpegstream object for mpeg output
  OutputHttpStream(OptionManager& mgr,
                   const std::string& descrName = "Output http Stream",
                   const std::string& tagName = "OutputHttpStream");

  //! Override from FrameOstream; just calls setFileStem()
  virtual void setConfigInfo(const std::string& filestem);

  // Implementations of writeFrame() and closeStream() are inherited
  // from LowLevelEncoderMap

  //! Specify a filename prefix to be used for all generated movie files
  /*! By default, the prefix is the empty string. */
  void setFileStem(const std::string& s);

private:
  /// Implemented to return an FfmpegEncoder object
  virtual rutz::shared_ptr<LowLevelEncoder>
  makeEncoder(const GenericFrameSpec& spec,
              const std::string& shortname,
              const FrameInfo& auxinfo);

        nub::soft_ref<HttpServer>                         itsHttpServer;
  OModelParam<int> itsFrameRate;
  NModelParam<int> itsFrameRateBase;
  OModelParam<int> itsBitRate;
  OModelParam<ByteCount> itsBufSize;
  OModelParam<std::string> itsCodecName;

  std::string itsStem;
};

#endif // HAVE_FFMPEG_AVCODEC_H

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_HTTPOUTPUTSTREAM_H_DEFINED

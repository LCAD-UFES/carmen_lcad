/*!@file Media/MpegOutputStream.H Write frames to movie files */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/MpegOutputStream.H $
// $Id: MpegOutputStream.H 12467 2010-01-04 19:25:26Z lior $
//

#ifndef MEDIA_MPEGOUTPUTSTREAM_H_DEFINED
#define MEDIA_MPEGOUTPUTSTREAM_H_DEFINED

#ifdef INVT_HAVE_AVCODEC

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Transport/LowLevelEncoderMap.H"
#include "Util/ByteCount.H"
#include "Util/Types.H"      // for byte

// ######################################################################
// ######################################################################
//! OutputMPEGStream writes mpeg video frames using the avcodec library
class OutputMPEGStream : public LowLevelEncoderMap
{
public:
  //! Construct an mpegstream object for mpeg output
  OutputMPEGStream(OptionManager& mgr,
                   const std::string& descrName = "Output MPEG Stream",
                   const std::string& tagName = "OutputMPEGStream");

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

  OModelParam<int> itsFrameRate;
  NModelParam<int> itsFrameRateBase;
  OModelParam<int> itsBitRate;
  OModelParam<ByteCount> itsBufSize;
  OModelParam<std::string> itsCodecName;
  OModelParam<bool> itsUseFormatContext;

  std::string itsStem;
};

#endif // HAVE_FFMPEG_AVCODEC_H

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_MPEGOUTPUTSTREAM_H_DEFINED

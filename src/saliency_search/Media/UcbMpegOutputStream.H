/*!@file Media/UcbMpegOutputStream.H Write mpeg video frames using mpeg_encode/ppmtompeg */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Media/UcbMpegOutputStream.H $
// $Id: UcbMpegOutputStream.H 8905 2007-10-25 23:21:41Z rjpeters $
//

#ifndef MEDIA_UCBMPEGOUTPUTSTREAM_H_DEFINED
#define MEDIA_UCBMPEGOUTPUTSTREAM_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Transport/LowLevelEncoderMap.H"
#include "Util/SimTime.H"

// ######################################################################
// ######################################################################
//! Write mpeg video frames using mpeg_encode/ppmtompeg
class UcbMpegOutputStream : public LowLevelEncoderMap
{
public:
  //! Construct an mpegstream object for mpeg output
  UcbMpegOutputStream(OptionManager& mgr,
                      const std::string& descrName = "Berkeley MPEG Output Stream",
                      const std::string& tagName = "UcbMpegOutputStream");

  //! Override from FrameOstream; just calls setFileStem()
  virtual void setConfigInfo(const std::string& filestem);

  // Implementations of writeFrame() and closeStream() are inherited
  // from LowLevelEncoderMap

  //! Specify a filename prefix to be used for all generated movie files
  /*! By default, the prefix is the empty string. */
  void setFileStem(const std::string& s);

protected:
  /// Create an appropriate UcbMpegEncoder
  virtual rutz::shared_ptr<LowLevelEncoder>
  makeEncoder(const GenericFrameSpec& spec,
              const std::string& shortname,
              const FrameInfo& auxinfo);

private:
  OModelParam<SimTime> itsFrameRate;
  OModelParam<std::string> itsBaselineQuality;

  std::string itsStem;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_UCBMPEGOUTPUTSTREAM_H_DEFINED

/*!@file Channels/ZeroChannel.C Trivial channel that just returns a map full of zeroes */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ZeroChannel.C $
// $Id: ZeroChannel.C 12820 2010-02-11 05:44:51Z itti $
//

#ifndef CHANNELS_ZEROCHANNEL_C_DEFINED
#define CHANNELS_ZEROCHANNEL_C_DEFINED

#include "Channels/ZeroChannel.H"

#include "Channels/ChannelOpts.H"

// ######################################################################
ZeroChannel::ZeroChannel(OptionManager& mgr,
                         const std::string& descrName,
                         const std::string& tagName)
  :
  ChannelBase(mgr, descrName, tagName, UNKNOWN),
  itsLevelSpec(&OPT_LevelSpec, this)
{}

// ######################################################################
ZeroChannel::~ZeroChannel()
{}

// ######################################################################
bool ZeroChannel::outputAvailable() const
{
  return true;
}

// ######################################################################
Dims ZeroChannel::getMapDims() const
{
  if (!this->hasInput())
    LFATAL("Oops! I haven't received any input yet");

  const Dims indims = this->getInputDims();

  return Dims(indims.w() >> itsLevelSpec.getVal().mapLevel(),
              indims.h() >> itsLevelSpec.getVal().mapLevel());
}

// ######################################################################
uint ZeroChannel::numSubmaps() const
{
  return 1;
}

// ######################################################################
Image<float> ZeroChannel::getSubmap(const uint index) const
{
  if (index != 0)
    LFATAL("got submap index = %u, but I have only one submap", index);

  return Image<float>(this->getMapDims(), ZEROS);
}

// ######################################################################
std::string ZeroChannel::getSubmapName(const uint index) const
{
  return "Zero";
}

// ######################################################################
std::string ZeroChannel::getSubmapNameShort(const uint index) const
{
  return "Zero";
}

// ######################################################################
void ZeroChannel::getFeatures(const Point2D<int>& locn,
                              std::vector<float>& mean) const
{
  LFATAL("not implemented");
}

// ######################################################################
void ZeroChannel::getFeaturesBatch(std::vector<Point2D<int>*> *locn,
                                   std::vector<std::vector<float> > *mean,
                                   int *count) const
{
  LFATAL("not implemented");
}

// ######################################################################
Image<float> ZeroChannel::getOutput()
{
  return Image<float>(this->getMapDims(), ZEROS);
}

// ######################################################################
void ZeroChannel::doInput(const InputFrame& inframe)
{
  // nothing to do here
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_ZEROCHANNEL_C_DEFINED

/*!@file Transport/FrameIstream.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/FrameIstream.C $
// $Id: FrameIstream.C 13273 2010-04-21 22:08:00Z rand $
//

#ifndef TRANSPORT_FRAMEISTREAM_C_DEFINED
#define TRANSPORT_FRAMEISTREAM_C_DEFINED

#include "Transport/FrameIstream.H"
#include "Media/FrameRange.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "Util/SimTime.H"

FrameListener::~FrameListener()
{}

FrameIstream::FrameIstream(OptionManager& mgr,
                           const std::string& descrName,
                           const std::string& tag)
  :
  ModelComponent(mgr, descrName, tag)
{}

FrameIstream::~FrameIstream()
{}

void FrameIstream::setConfigInfo(const std::string& cfg)
{
  // do nothing, subclasses can override if they need to handle this
  // user input
}

void FrameIstream::setListener(rutz::shared_ptr<FrameListener> listener)
{
  // do nothing, just ignore the listener; subclasses can override if
  // they want to do something with the listener
}

bool FrameIstream::setFrameNumber(int)
{
  // do nothing, subclasses can override if they need to know the
  // frame number

  return true;
}

SimTime FrameIstream::getNaturalFrameTime() const
{
  return SimTime::ZERO();
}

Dims FrameIstream::peekDims()
{
  return this->peekFrameSpec().dims;
}

int FrameIstream::getWidth()
{
  return this->peekFrameSpec().dims.w();
}

int FrameIstream::getHeight()
{
  return this->peekFrameSpec().dims.h();
}

void FrameIstream::startStream()
{
  // do nothing, subclasses can override if they need special
  // preparation to start the frame stream
}

Image<PixRGB<byte> > FrameIstream::readRGB()
{
  return this->readFrame().asRgb();
}

Image<PixRGB<uint16> > FrameIstream::readRGBU16()
{
  return this->readFrame().asRgbU16();
}

Image<byte> FrameIstream::readGray()
{
  return this->readFrame().asGray();
}

Image<uint16> FrameIstream::readGrayU16()
{
  return this->readFrame().asGrayU16();
}

Image<float> FrameIstream::readFloat()
{
  return this->readFrame().asFloat();
}

bool FrameIstream::readAndDiscardFrame()
{
  const GenericFrame frame = this->readFrame();
  return frame.initialized();
}

bool FrameIstream::supportsSeek()
{
  return false;
}

FrameRange FrameIstream::getFrameRange()
{
  return FrameRange();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // TRANSPORT_FRAMEISTREAM_C_DEFINED

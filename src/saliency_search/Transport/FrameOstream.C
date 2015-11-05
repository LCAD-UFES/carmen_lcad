/*!@file Transport/FrameOstream.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Transport/FrameOstream.C $
// $Id: FrameOstream.C 9547 2008-03-28 23:32:43Z rjpeters $
//

#ifndef FRAMEOSTREAM_C_DEFINED
#define FRAMEOSTREAM_C_DEFINED

#include "Transport/FrameOstream.H"

#include "Image/Image.H"
#include "Image/Layout.H"
#include "Image/Pixels.H"
#include "Image/Normalize.H"
#include "Raster/GenericFrame.H"
#include "Transport/FrameInfo.H"

FrameOstream::FrameOstream(OptionManager& mgr,
                           const std::string& descrName,
                           const std::string& tag)
  :
  ModelComponent(mgr, descrName, tag)
{}

FrameOstream::~FrameOstream() {}

void FrameOstream::setConfigInfo(const std::string& cfg)
{
  // do nothing; subclasses can override if they need to parse this
  // user input
}

bool FrameOstream::setFrameNumber(int)
{
  // do nothing; many FrameOstream classes won't care about, or even
  // have any concept of, a "frame number"

  return true;
}

void FrameOstream::writeRGB(const Image<PixRGB<byte> >& in,
                            const std::string& fname,
                            const FrameInfo& auxinfo)
{
  this->writeFrame(GenericFrame(in), fname, auxinfo);
}

void FrameOstream::writeGray(const Image<byte>& in,
                             const std::string& fname,
                             const FrameInfo& auxinfo)
{
  this->writeFrame(GenericFrame(in), fname, auxinfo);
}

void FrameOstream::writeFloat(const Image<float>& in,
                              const int flags,
                              const std::string& fname,
                              const FrameInfo& auxinfo)
{
  this->writeFrame(GenericFrame(in, flags), fname, auxinfo);
}

void FrameOstream::writeRgbLayout(const Layout<PixRGB<byte> >& layout,
                                  const std::string& shortname,
                                  const FrameInfo& auxinfo)
{
  this->writeFrame(GenericFrame(layout), shortname, auxinfo);
}

void FrameOstream::writeGrayLayout(const Layout<byte>& layout,
                                   const std::string& shortname,
                                   const FrameInfo& auxinfo)
{
  this->writeFrame(GenericFrame(layout), shortname, auxinfo);
}

bool FrameOstream::isVoid() const { return false; }

const FrameInfo FrameOstream::defaultInfo;

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // FRAMEOSTREAM_C_DEFINED

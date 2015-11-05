/*!@file Media/MgzJInputStream.C Read frames from movie files */

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
// Primary maintainer for this file: Randolph Voorhies <voorhies at usc dot edu>

#ifndef MEDIA_MGZJINPUTSTREAM_C_DEFINED
#define MEDIA_MGZJINPUTSTREAM_C_DEFINED

#include "Media/MgzJInputStream.H"

#include "Component/ModelOptionDef.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Media/MediaOpts.H"
#include "Media/MgzJDecoder.H"
#include "Media/FrameRange.H"
#include "Raster/GenericFrame.H"
#include "Transport/TransportOpts.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include "rutz/trace.h"

// ######################################################################
MgzJInputStream::MgzJInputStream(OptionManager& mgr,
                                 const std::string& descrName,
                                 const std::string& tagName) :
  FrameIstream(mgr, descrName, tagName),
  itsDecoder(NULL), itsFrame(), itsFrameSpec(), itsFrameSpecValid(false)
{ }

// ######################################################################
MgzJInputStream::~MgzJInputStream()
{
  delete itsDecoder;
}

// ######################################################################
void MgzJInputStream::setConfigInfo(const std::string& filename)
{
  // NOTE: if you modify any behavior here, then please update the
  // corresponding documentation for the global "--in" option inside
  // the OPT_InputFrameSource definition in Media/MediaOpts.C

  this->setFileName(filename);
}

// ######################################################################
GenericFrameSpec MgzJInputStream::peekFrameSpec()
{
  GVX_TRACE(__PRETTY_FUNCTION__);

  if (!itsFrameSpecValid)
  {
    if (itsFrame.initialized() == false)
      itsFrame = readFrame();

    itsFrameSpec = itsFrame.frameSpec();
    itsFrameSpecValid = true;
  }

  return itsFrameSpec;
}

// ######################################################################
void MgzJInputStream::setFileName(std::string fname)
{
  GVX_TRACE(__PRETTY_FUNCTION__);

  if (itsDecoder)
    {
      delete itsDecoder;
    }
  itsDecoder = new MgzJDecoder(fname);
}

// ######################################################################
GenericFrame MgzJInputStream::readFrame()
{
  GVX_TRACE(__PRETTY_FUNCTION__);

  if (itsDecoder == NULL) LFATAL("You need to setFileName() first");
  GenericFrame ret;

  // do we already have a frame because we peeked its specs?
  if (itsFrame.initialized()) { ret = itsFrame; itsFrame = GenericFrame(); }
  else ret = itsDecoder->readFrame();

  //Auto-increment the frame number
  this->setFrameNumber(itsFrameNumber+1);
  return ret;
}

// ######################################################################
bool MgzJInputStream::setFrameNumber(int n)
{
  if(itsDecoder->setFrameNumber(n))
  {
    itsFrameNumber = n;
    return true;
  }

  return false;
}

// ######################################################################
bool MgzJInputStream::readAndDiscardFrame()
{
  return this->setFrameNumber(itsFrameNumber+1);
}

// ######################################################################
FrameRange MgzJInputStream::getFrameRange()
{
  return FrameRange(0, 1, itsDecoder->getNumFrames());
}

// ######################################################################

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // MEDIA_MGZJINPUTSTREAM_C_DEFINED


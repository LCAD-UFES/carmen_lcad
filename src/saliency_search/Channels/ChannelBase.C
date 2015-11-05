/*!@file Channels/ChannelBase.C The base class for all channel classes */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ChannelBase.C $
// $Id: ChannelBase.C 10794 2009-02-08 06:21:09Z itti $
//

#include "Channels/ChannelBase.H"

#include "Channels/ChannelVisitor.H"
#include "Channels/InputFrame.H"
#include "Channels/VisualFeatures.H"
#include "Component/ParamMap.H"
#include "Image/Image.H"
#include "rutz/error_context.h"
#include "rutz/sfmt.h"
#include "rutz/trace.h"

#include <string>

// ######################################################################
// ######################################################################
// ChannelBase member definitions:
// ######################################################################
// ######################################################################

ChannelBase::ChannelBase(OptionManager& mgr, const std::string& descrName,
                         const std::string& tag, const VisualFeature vs) :
  // ModelComponent::ModelComponent() auto called - virtual base
  itsVisFeature("VisualFeature", this, vs),
  itsInputDims()
{
GVX_TRACE(__PRETTY_FUNCTION__);
 ModelComponent::init(mgr, descrName, tag);
}

// ######################################################################
ChannelBase::~ChannelBase()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void ChannelBase::reset1()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  this->killCaches();
  itsInputDims = Dims();
  ModelComponent::reset1();
}

// ######################################################################
void ChannelBase::accept(ChannelVisitor& v)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  v.visitChannelBase(*this);
}

// ######################################################################
VisualFeature ChannelBase::visualFeature() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsVisFeature.getVal();
}

// ######################################################################
bool ChannelBase::isHomogeneous() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // default returns true; subclasses override if they want to return
  // something different
  return true;
}

// ######################################################################
void ChannelBase::readFrom(const ParamMap& pmap)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  killCaches();

  if (!pmap.hasParam("descriptivename"))
    LFATAL("Missing descriptivename parameter (expected %s)",
           descriptiveName().c_str());
  const std::string descriptive =
    pmap.getStringParam("descriptivename", "bug");
  if (descriptive.compare(descriptiveName()) != 0)
    LFATAL("Wrong descriptivename %s (expected %s)",
           descriptive.c_str(), descriptiveName().c_str());
}

// ######################################################################
void ChannelBase::writeTo(ParamMap& pmap) const
{
GVX_TRACE(__PRETTY_FUNCTION__);

  pmap.putStringParam("descriptivename", descriptiveName());
}

// ######################################################################
void ChannelBase::input(const InputFrame& inframe)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  GVX_ERR_CONTEXT(rutz::sfmt("receiving input in channel %s",
                             this->tagName().c_str()));

  killCaches();
  itsInputDims = inframe.getDims();
  doInput(inframe);
}

// ######################################################################
bool ChannelBase::hasInput() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsInputDims.isNonEmpty();
}

// ######################################################################
Dims ChannelBase::getInputDims() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (itsInputDims.isEmpty())
    CLFATAL("I haven't received any input yet");

  return itsInputDims;
}

// ######################################################################
void ChannelBase::saveResults(const nub::ref<FrameOstream>& ofs)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // default is a no-op
}

// ######################################################################
void ChannelBase::killCaches()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  /* no caches to worry about here -- in particular, don't try to
     reset itsInputDims, since we still want to be able to retrieve
     that value in between frames, when caches might otherwise be
     killed */
}

// ######################################################################
void ChannelBase::setInputDims(const Dims& dims)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (dims.isEmpty())
    CLFATAL("input image must be non-empty!");
  itsInputDims = dims;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

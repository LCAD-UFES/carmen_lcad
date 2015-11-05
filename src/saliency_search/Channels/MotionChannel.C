/*!@file Channels/MotionChannel.C */

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
// Primary maintainer for this file:
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/MotionChannel.C $
// $Id: MotionChannel.C 11208 2009-05-20 02:03:21Z itti $
//

#ifndef MOTIONCHANNEL_C_DEFINED
#define MOTIONCHANNEL_C_DEFINED

#include "Channels/MotionChannel.H"

#include "Channels/ChannelOpts.H"
#include "Channels/DirectionChannel.H"
#include "Component/OptionManager.H"
#include "rutz/trace.h"

// ######################################################################
// MotionChannel member definitions:
// ######################################################################

// ######################################################################
MotionChannel::MotionChannel(OptionManager& mgr) :
  ComplexChannel(mgr, "Motion", "motion", MOTION),
  itsPyrType("MotionChannelPyramidType", this, Gaussian5),
  itsNumDirs(&OPT_NumDirections, this) // see Channels/ChannelOpts.{H,C}
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // let's create our subchannels (may be reconfigured later if our
  // number of directions changes):
  buildSubChans();
}

// ######################################################################
MotionChannel::~MotionChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
DirectionChannel& MotionChannel::dirChan(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return *(dynCast<DirectionChannel>(subChan(idx)));
}

// ######################################################################
void MotionChannel::buildSubChans()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // kill any subchans we may have had...
  this->removeAllSubChans();

  // let's instantiate our subchannels now that we know how many we
  // want. They will inherit the current values (typically
  // post-command-line parsing) of all their options as they are
  // constructed:
  LINFO("Using %d directions spanning [0..360]deg", itsNumDirs.getVal());
  for (uint i = 0; i < itsNumDirs.getVal(); ++i)
    {
      nub::ref<DirectionChannel> chan =
        makeSharedComp
        (new DirectionChannel(getManager(), i,
                              360.0 * double(i) /
                              double(itsNumDirs.getVal()),
                              itsPyrType.getVal()));
      this->addSubChan(chan);

      chan->exportOptions(MC_RECURSE);
    }
}

// ######################################################################
void MotionChannel::paramChanged(ModelParamBase* const param,
                                 const bool valueChanged,
                                 ParamClient::ChangeStatus* status)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ComplexChannel::paramChanged(param, valueChanged, status);

  // if the param is our number of orientations and it has become
  // different from our number of channels, let's reconfigure:
  if (param == &itsNumDirs &&
      numChans() != itsNumDirs.getVal())
    buildSubChans();
}

// ######################################################################
void MotionChannel::doInput(const InputFrame& inframe)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(inframe.grayFloat().initialized());

  // compute Reichardt motion detection into several directions
  for (uint dir = 0; dir < numChans(); ++dir)
    {
      subChan(dir)->input(inframe);
      LINFO("Motion pyramid (%d/%d) ok.", dir+1, numChans());
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // MOTIONCHANNEL_C_DEFINED

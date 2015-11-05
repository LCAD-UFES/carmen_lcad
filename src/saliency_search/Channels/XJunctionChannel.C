/*!@file Channels/XJunctionChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/XJunctionChannel.C $
// $Id: XJunctionChannel.C 9504 2008-03-19 23:23:43Z rjpeters $
//

#ifndef XJUNCTIONCHANNEL_C_DEFINED
#define XJUNCTIONCHANNEL_C_DEFINED

#include "Channels/XJunctionChannel.H"

#include "Channels/ChannelOpts.H"
#include "Channels/JunctionChannel.H"
#include "Channels/OrientationChannel.H"
#include "Component/OptionManager.H"
#include "rutz/trace.h"

// ######################################################################
// X Junction Channel member definitions:
// ######################################################################
XJunctionChannel::XJunctionChannel(OptionManager& mgr,
                                   nub::soft_ref<OrientationChannel> oc)
  :
  ComplexChannel(mgr, "XJunction", "x-junction", XJUNCTION),
  itsNumOrients(&OPT_NumXOrients, this), // see Channels/ChannelOpts.{H,C}
  itsOriChan(oc)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // let's build our channels
  buildSubChans();
}

// ######################################################################
XJunctionChannel::~XJunctionChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void XJunctionChannel::buildSubChans()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // kill any subchans we may have had...
  this->removeAllSubChans();

  LINFO("Using %d orientations spanning [0..90]deg", itsNumOrients.getVal());
  if (itsNumOrients.getVal() >= 1)
    addSubChan(makeSharedComp
               (new JunctionChannel(getManager(), itsOriChan,
                                    visualFeature(),
                                    1,0,1,0,1,0,1,0)),
               "", 1.0, /* exportOpts = */ true);

  if (itsNumOrients.getVal() >= 2)
    addSubChan(makeSharedComp
               (new JunctionChannel(getManager(), itsOriChan,
                                    visualFeature(),
                                    0,1,0,1,0,1,0,1)),
               "", 1.0, /* exportOpts = */ true);
}

// ######################################################################
void XJunctionChannel::paramChanged(ModelParamBase* const param,
                                    const bool valueChanged,
                                    ParamClient::ChangeStatus* status)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ComplexChannel::paramChanged(param, valueChanged, status);

  // if the param is our number of orientations and it has become
  // different from our number of channels, let's reconfigure:
  if (param == &itsNumOrients &&
      numChans() != itsNumOrients.getVal())
    {
      if (itsNumOrients.getVal() != 1 && itsNumOrients.getVal() != 2)
        {
          LERROR("%s must be 1 or 2", itsNumOrients.getName().c_str());
          *status = ParamClient::CHANGE_REJECTED;
        }
      else
        buildSubChans();
    }
}

// ######################################################################
void XJunctionChannel::doInput(const InputFrame& inframe)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  for (uint i = 0; i < numChans(); ++i)
    {
      subChan(i)->input(inframe);
      LINFO("XJunction pyramid (%d/%d) ok.", i+1, numChans());
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // XJUNCTIONCHANNEL_C_DEFINED

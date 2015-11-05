/*!@file Channels/TJunctionChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/TJunctionChannel.C $
// $Id: TJunctionChannel.C 9503 2008-03-19 23:10:33Z rjpeters $
//

#ifndef TJUNCTIONCHANNEL_C_DEFINED
#define TJUNCTIONCHANNEL_C_DEFINED

#include "Channels/TJunctionChannel.H"

#include "Channels/ChannelOpts.H"
#include "Channels/JunctionChannel.H"
#include "Channels/OrientationChannel.H"
#include "Component/OptionManager.H"
#include "rutz/trace.h"

// ######################################################################
// T Junction Channel member definitions:
// ######################################################################
TJunctionChannel::TJunctionChannel(OptionManager& mgr,
                                   nub::soft_ref<OrientationChannel> oc)
  :
  ComplexChannel(mgr, "TJunction", "t-junction", TJUNCTION),
  itsNumOrients(&OPT_NumTOrients, this), // see Channels/ChannelOpts.{H,C}
  itsOriChan(oc)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // let's build our channels
  buildSubChans();
}

// ######################################################################
TJunctionChannel::~TJunctionChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void TJunctionChannel::buildSubChans()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // kill any subchans we may have had...
  this->removeAllSubChans();

  LINFO("Using %d orientations spanning [0..360]deg", itsNumOrients.getVal());
  for (uint ori = 0; ori < 8; )
    {
      switch (ori)
        {
        case 0:
          addSubChan(makeSharedComp
                     (new JunctionChannel(getManager(), itsOriChan,
                                          visualFeature(),
                                          1,0,0,0,1,0,1,0 )),
                     "", 1.0, /* exportOpts = */ true);
          break;
        case 1:
          addSubChan(makeSharedComp
                     (new JunctionChannel(getManager(), itsOriChan,
                                          visualFeature(),
                                          0,1,0,0,0,1,0,1 )),
                     "", 1.0, /* exportOpts = */ true);
          break;
        case 2:
          addSubChan(makeSharedComp
                     (new JunctionChannel(getManager(), itsOriChan,
                                          visualFeature(),
                                          1,0,1,0,0,0,1,0 )),
                     "", 1.0, /* exportOpts = */ true);
          break;
        case 3:
          addSubChan(makeSharedComp
                     (new JunctionChannel(getManager(), itsOriChan,
                                          visualFeature(),
                                          0,1,0,1,0,0,0,1 )),
                     "", 1.0, /* exportOpts = */ true);
          break;
        case 4:
          addSubChan(makeSharedComp
                     (new JunctionChannel(getManager(), itsOriChan,
                                          visualFeature(),
                                          1,0,1,0,1,0,0,0 )),
                     "", 1.0, /* exportOpts = */ true);
          break;
        case 5:
          addSubChan(makeSharedComp
                     (new JunctionChannel(getManager(), itsOriChan,
                                          visualFeature(),
                                          0,1,0,1,0,1,0,0 )),
                     "", 1.0, /* exportOpts = */ true);
          break;
        case 6:
          addSubChan(makeSharedComp
                     (new JunctionChannel(getManager(), itsOriChan,
                                          visualFeature(),
                                          0,0,1,0,1,0,1,0 )),
                     "", 1.0, /* exportOpts = */ true);
          break;
        case 7:
          addSubChan(makeSharedComp
                     (new JunctionChannel(getManager(), itsOriChan,
                                          visualFeature(),
                                          0,0,0,1,0,1,0,1 )),
                     "", 1.0, /* exportOpts = */ true);
          break;
        default:
          break;
        }
      ori += 8 / itsNumOrients.getVal();
    }
}

// ######################################################################
void TJunctionChannel::paramChanged(ModelParamBase* const param,
                                    const bool valueChanged,
                                    ParamClient::ChangeStatus* status)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ComplexChannel::paramChanged(param, valueChanged, status);

  // if the param is our number of orientations and it has become
  // different from our number of channels, let's reconfigure:
  if (param == &itsNumOrients &&
      numChans() != itsNumOrients.getVal())
    buildSubChans();
}

// ######################################################################
void TJunctionChannel::doInput(const InputFrame& inframe)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  for (uint i = 0; i < numChans(); ++i)
    {
      subChan(i)->input(inframe);
      LINFO("TJunction pyramid (%d/%d) ok.", i+1, numChans());
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // TJUNCTIONCHANNEL_C_DEFINED

/*!@file Channels/FoeMSTChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/FoeMSTChannel.C $
// $Id: FoeMSTChannel.C 12962 2010-03-06 02:13:53Z irock $
//

#ifndef FOEMSTCHANNEL_C_DEFINED
#define FOEMSTCHANNEL_C_DEFINED

#include "Channels/FoeMSTChannel.H"

#include "Channels/ChannelOpts.H"
#include "Channels/MSTChannel.H"
#include "Channels/MotionChannel.H"
#include "Channels/DirectionChannel.H"
#include "Component/OptionManager.H"
#include "rutz/trace.h"

// ######################################################################
// L MST Channel member definitions:
// ######################################################################
FoeMSTChannel::FoeMSTChannel(OptionManager& mgr,
                                   nub::soft_ref<MotionChannel> oc)
  :
  ComplexChannel(mgr, "FoeMST", "foe-mst", FOEMST),
  itsNumDirs(&OPT_NumDirections, this), // see Channels/ChannelOpts.{H,C}
  itsOriChan(oc)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // let's build our channels
  buildSubChans();
}

// ######################################################################
FoeMSTChannel::~FoeMSTChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void FoeMSTChannel::buildSubChans()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // kill any subchans we may have had...
  this->removeAllSubChans();

  LINFO("Using %d directions spanning [0..360]deg", itsNumDirs.getVal());
  for (uint ori = 0; ori < 8; )
    {
      switch (ori)
        {
        case 0: //The point in the center POINT 0
          addSubChan(makeSharedComp
                     (new MSTChannel(getManager(), itsOriChan,
                                          visualFeature(),
                                          1,1,1,1,1,1,1,1 )),
                     "", 1.0, /* exportOpts = */ true);
          break;
        case 1: //The point in the center POINT 1
          addSubChan(makeSharedComp
                     (new MSTChannel(getManager(), itsOriChan,
                                          visualFeature(),
                                          1,0,0,0,0,0,0,0 )),
                     "", 1.0, /* exportOpts = */ true);
          break;
        case 2: //The point in the center POINT 2
          addSubChan(makeSharedComp
                     (new MSTChannel(getManager(), itsOriChan,
                                          visualFeature(),
                                          1,1,1,0,0,0,0,0 )),
                     "", 1.0, /* exportOpts = */ true);
          break;
        case 3://The point in the center POINT 3
          addSubChan(makeSharedComp
                     (new MSTChannel(getManager(), itsOriChan,
                                          visualFeature(),
                                          0,0,1,0,0,0,0,0 )),
                     "", 1.0, /* exportOpts = */ true);
          break;
        case 4: //The point in the center POINT 4
          addSubChan(makeSharedComp
                     (new MSTChannel(getManager(), itsOriChan,
                                          visualFeature(),
                                          0,1,1,1,0,0,0,0 )),
                     "", 1.0, /* exportOpts = */ true);
          break;
        case 5:  //The point in the center POINT 5
          addSubChan(makeSharedComp
                     (new MSTChannel(getManager(), itsOriChan,
                                          visualFeature(),
                                          0,0,0,1,0,0,0,0 )),
                     "", 1.0, /* exportOpts = */ true);
          break;
        case 6: //The point in the center POINT 6
          addSubChan(makeSharedComp
                     (new MSTChannel(getManager(), itsOriChan,
                                          visualFeature(),
                                          0,0,0,0,1,1,1,0 )),
                     "", 1.0, /* exportOpts = */ true);
          break;
        case 7: //The point in the center POINT 7
          addSubChan(makeSharedComp
                     (new MSTChannel(getManager(), itsOriChan,
                                          visualFeature(),
                                          0,1,0,0,0,0,1,0 )),
                     "", 1.0, /* exportOpts = */ true);
          break;
        case 8: //The point in the center POINT 8
          addSubChan(makeSharedComp
                     (new MSTChannel(getManager(), itsOriChan,
                                          visualFeature(),
                                          1,0,0,0,0,0,1,1 )),
                     "", 1.0, /* exportOpts = */ true);
          break;

        default:
          break;
        }
      ori += 8 / itsNumDirs.getVal();
    }
}

// ######################################################################
void FoeMSTChannel::paramChanged(ModelParamBase* const param,
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
void FoeMSTChannel::doInput(const InputFrame& inframe)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  for (uint i = 0; i < numChans(); ++i)
    {
      subChan(i)->input(inframe);
      LINFO("FoeMST pyramid (%d/%d) ok.", i+1, numChans());
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // FOEMSTCHANNEL_C_DEFINED

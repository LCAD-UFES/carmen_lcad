/*!@file Channels/IntegerMotionChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/IntegerMotionChannel.C $
// $Id: IntegerMotionChannel.C 8160 2007-03-21 21:34:16Z rjpeters $
//

#ifndef CHANNELS_INTEGERMOTIONCHANNEL_C_DEFINED
#define CHANNELS_INTEGERMOTIONCHANNEL_C_DEFINED

#include "Channels/IntegerMotionChannel.H"

#include "Channels/ChannelOpts.H"
#include "Channels/IntegerDirectionChannel.H"
#include "Component/OptionManager.H"
#include "Image/IntegerMathOps.H"
#include "rutz/trace.h"

namespace
{
  IntgTrigTable<256, 8> trig;
}

// ######################################################################
// IntegerMotionChannel member definitions:
// ######################################################################

// ######################################################################
IntegerMotionChannel::
IntegerMotionChannel(OptionManager& mgr,
                     nub::ref<IntegerMathEngine> eng) :
  IntegerComplexChannel(mgr, "Integer Motion", "int-motion", MOTION, eng),
  itsNumDirs(&OPT_NumDirections, this) // see Channels/ChannelOpts.{H,C}
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // let's create our subchannels (may be reconfigured later if our
  // number of directions changes):
  buildSubChans();
}

// ######################################################################
IntegerMotionChannel::~IntegerMotionChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
IntegerDirectionChannel& IntegerMotionChannel::dirChan(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return *(dynCast<IntegerDirectionChannel>(subChan(idx)));
}

// ######################################################################
void IntegerMotionChannel::buildSubChans()
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
      const double dir =
        360.0 * double(i) / double(itsNumDirs.getVal());

      nub::ref<IntegerDirectionChannel> chan =
        makeSharedComp
        (new IntegerDirectionChannel(getManager(), i, dir,
                                     trig.costab[trig.indexDegrees(dir)],
                                     -trig.sintab[trig.indexDegrees(dir)],
                                     trig.nbits,
                                     this->getMathEngine()));
      this->addSubChan(chan);

      chan->exportOptions(MC_RECURSE);
    }
}

// ######################################################################
void IntegerMotionChannel::paramChanged(ModelParamBase* const param,
                                        const bool valueChanged,
                                        ParamClient::ChangeStatus* status)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  IntegerComplexChannel::paramChanged(param, valueChanged, status);

  // if the param is our number of orientations and it has become
  // different from our number of channels, let's reconfigure:
  if (param == &itsNumDirs &&
      numChans() != itsNumDirs.getVal())
    buildSubChans();
}

// ######################################################################
void IntegerMotionChannel::doInputInt(const IntegerInput& inp,
                                      const SimTime& t,
                                      PyramidCache<int>* cache,
                                      const Image<byte>& clipMask)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // compute Reichardt motion detection into several directions
  for (uint dir = 0; dir < numChans(); ++dir)
    {
      dirChan(dir).inputInt(inp, t, cache, clipMask);
      LINFO("Integer Motion pyramid (%d/%d) ok.", dir+1, numChans());
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_INTEGERMOTIONCHANNEL_C_DEFINED

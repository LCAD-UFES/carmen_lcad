/*!@file Channels/IntegerOrientationChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/IntegerOrientationChannel.C $
// $Id: IntegerOrientationChannel.C 8199 2007-03-30 17:58:46Z rjpeters $
//

#ifndef CHANNELS_INTEGERORIENTATIONCHANNEL_C_DEFINED
#define CHANNELS_INTEGERORIENTATIONCHANNEL_C_DEFINED

#include "Channels/IntegerOrientationChannel.H"

#include "Channels/ChannelOpts.H"
#include "Channels/IntegerSimpleChannel.H"
#include "Component/OptionManager.H"
#include "Image/ImageSetOps.H"
#include "Image/IntegerMathOps.H"
#include "Image/PyramidCache.H"
#include "Util/sformat.H"
#include "rutz/mutex.h"
#include "rutz/trace.h"

// ######################################################################
// IntegerOrientationChannel member definitions:
// ######################################################################

// ######################################################################
IntegerOrientationChannel::
IntegerOrientationChannel(OptionManager& mgr,
                          nub::ref<IntegerMathEngine> eng) :
  IntegerComplexChannel(mgr, "Integer Orientation",
                        "int-orientation", ORI, eng),
  itsNumOrients(&OPT_NumOrientations, this) // see Channels/ChannelOpts.{H,C}
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // let's build our channels; we may have to re-build them if
  // itsNumOrient get changed on us before we start():
  buildSubChans();
}

// ######################################################################
void IntegerOrientationChannel::buildSubChans()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // kill any subchans we may have had...
  this->removeAllSubChans();

  // let's instantiate our Gabor subchannels now that we know how many
  // we want. They will inherit the current values (typically
  // post-command-line parsing) of all their options as they are
  // constructed:
  LINFO("Using %d orientations spanning [0..180]deg", itsNumOrients.getVal());
  for (uint ori = 0; ori < itsNumOrients.getVal(); ++ori)
    {
      const double theta =
        180.0 * double(ori) / double(itsNumOrients.getVal());

      nub::ref<IntegerSimpleChannel> chan
        (new IntegerSimpleChannel
         (getManager(),
          sformat("Integer Gabor (%d)", int(theta)),
          sformat("int-ori_%u", ori), UNKNOWN,
          rutz::make_shared(new IntgOrientedPyrBuilder
                            (9, theta, this->getImath())),
          this->getMathEngine()));

      chan->setNormalizeOutput(true);

      this->addSubChan(chan);

      // let's export options on our newly built channels:
      chan->exportOptions(MC_RECURSE);
    }
}

// ######################################################################
void IntegerOrientationChannel::paramChanged(ModelParamBase* const param,
                                             const bool valueChanged,
                                             ParamClient::ChangeStatus* status)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  IntegerComplexChannel::paramChanged(param, valueChanged, status);

  // if the param is our number of orientations and it has become
  // different from our number of channels, let's reconfigure:
  if (param == &itsNumOrients && valueChanged)
    buildSubChans();
}

// ######################################################################
IntegerOrientationChannel::~IntegerOrientationChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
IntegerSimpleChannel& IntegerOrientationChannel::gabor(const uint idx) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // Since we are dynamic_cast'ing a reference, this operation will
  // either succeed or throw an exception.
  return *(dynCast<IntegerSimpleChannel>(subChan(idx)));
}

// ######################################################################
void IntegerOrientationChannel::doInputInt(const IntegerInput& inp,
                                           const SimTime& t,
                                           PyramidCache<int>* cache,
                                           const Image<byte>& clipMask)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(inp.grayInt().initialized());

  if (numChans() == 0)
    return;

  rutz::mutex_lock_class lock;
  if (cache && cache->laplacian9.beginSet(inp.grayInt(), &lock))
    {
      cache->laplacian9.endSet
        (inp.grayInt(),
         intgBuildPyrLaplacian
         (inp.grayInt(), gabor(0).getMinPyrLevel(),
          gabor(0).getMaxPyrLevel(), 9,
          this->getImath()),
         &lock);
    }

  for (uint i = 0; i < numChans(); ++i)
    {
      gabor(i).inputInt(inp, t, cache, clipMask);
      LINFO("Orientation pyramid (%d/%d) ok.", i+1, numChans());
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_INTEGERORIENTATIONCHANNEL_C_DEFINED

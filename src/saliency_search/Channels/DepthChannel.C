/*!@file Channels/DepthChannel.C A depth channel. */

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
// Primary maintainer for this file: Laurent Itti
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/DepthChannel.C $
// $Id: DepthChannel.C 14293 2010-12-02 01:57:25Z itti $

#include "Channels/DepthChannel.H"

#include "Channels/OrientationChannel.H"
#include "Channels/FlickerChannel.H"
#include "Channels/IntensityChannel.H"
#include "Channels/ChannelOpts.H"
#include "Component/OptionManager.H"
#include "Image/ColorOps.H"
#include "Image/PyramidOps.H" // for buildPyrGaussian()
#include "rutz/mutex.h"
#include "rutz/trace.h"


DepthChannel::DepthChannel(OptionManager& mgr) :
  ComplexChannel(mgr, "Depth", "Depth", DEPTH),
  itsLevelSpec(&OPT_LevelSpec, this)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // create a bunch of subchannels:
  this->addSubChan(makeSharedComp(new IntensityChannel(mgr, "depi", "DepthIntensity")));
  //this->addSubChan(makeSharedComp(new FlickerChannel(mgr, "depf", "DepthFlicker")));
  //this->addSubChan(makeSharedComp(new OrientationChannel(mgr, "depo", "DepthOrientation", "depth")));
}

// ######################################################################
DepthChannel::~DepthChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void DepthChannel::doInput(const InputFrame& origframe)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  /* Look for a special depth image from the retina. This is held in
     the retina along with the standard image in certain special
     instances such as when using an Open Scene Graph scene, or a
     Kinect FrameGrabber and is passed along as part of the
     InputFrame. In the future this might be placed as a new region
     (LGN?) since it can be cross used with a stereo channel. We need
     it here so we can switch pass a new depth-based InputFrame to our
     subchannels. */
  if (origframe.hasDepthImage()) {
    const Image<uint16>  idi = origframe.getDepthImage();
    const Image<byte>    cm  = origframe.clipMask();

    // convert depth to float and normalize it:
    Image<float> df = idi;
    df *= 0.125F; // NOTE: Assumes 12-bit depth image

    InputFrame depthframe = InputFrame::fromGrayFloat(&df, origframe.time(), &cm, InputFrame::emptyCache);

    rutz::mutex_lock_class lock;
    if (depthframe.pyrCache().get() != 0 && depthframe.pyrCache()->gaussian5.beginSet(depthframe.grayFloat(), &lock)) {
      LINFO("Computing depth pyramid");
      depthframe.pyrCache()->
	gaussian5.endSet(depthframe.grayFloat(),
			 buildPyrGaussian(depthframe.grayFloat(), 0, itsLevelSpec.getVal().maxDepth(), 5), &lock);
    } else {
      LINFO("Problem with depth pyramid");
      depthframe = origframe;
    }

    // send input to all our subchans:
    for (uint i = 0; i < numChans(); ++i) subChan(i)->input(depthframe);
    LINFO("Depth channel ok.");

  } else LINFO("No depth image from retina -- IGNORING");
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

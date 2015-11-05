/*!@file Channels/FlickerNBackChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/FlickerNBackChannel.C $

#ifndef FLICKERNBACKCHANNEL_C_DEFINED
#define FLICKERNBACKCHANNEL_C_DEFINED

#include "Channels/FlickerNBackChannel.H"

#include "Image/MathOps.H" // for absDiff()
#include "rutz/trace.h"

// ######################################################################
// FlickerNBackChannel member definitions:
// ######################################################################

// ######################################################################
FlickerNBackChannel::FlickerNBackChannel(OptionManager& mgr) :
  GaussianPyrChannel(mgr, "FlickerNBack", "flicker", FLICKER),
  itsQueueLength("FlickerNBackQueueLength", this, SimTime::MSECS(50)),
  itsQueue()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsTakeAbs.setVal(true);
  itsNormalizeOutput.setVal(true);
}

// ######################################################################
FlickerNBackChannel::~FlickerNBackChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void FlickerNBackChannel::doInput(const InputFrame& inframe)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(inframe.grayFloat().initialized());

  //add onto our buffer
  itsQueue.push_back(ImageHolder(inframe.grayFloat(), inframe.time()));

  //if we are greater than our queue length we can take a difference
  if ( (itsQueue.back().time() - itsQueue.front().time()) >= itsQueueLength.getVal())
  {
    // take simple abs difference between the front and back of our queue. 
    Image<float> fli = absDiff(itsQueue.back().image(), 
                               itsQueue.front().image());

    //empty one off the front, our oldest frame
    itsQueue.pop_front();

    SingleChannel::doInput(InputFrame::fromGrayFloat
                           (&fli, inframe.time(),
                            &inframe.clipMask(), inframe.pyrCache()));
  }
}

// ######################################################################
void FlickerNBackChannel::reset1()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // reset some stuff in FlickerNBackChannel
  itsQueue.clear();

  // propagate to our base class:
  SingleChannel::reset1();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // FLICKERCHANNEL_C_DEFINED

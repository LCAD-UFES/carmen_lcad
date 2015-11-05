/*!@file Channels/IntegerChannel.C Base class for channels that will use integer math */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/IntegerChannel.C $
// $Id: IntegerChannel.C 7858 2007-02-07 21:36:56Z rjpeters $
//

#ifndef CHANNELS_INTEGERCHANNEL_C_DEFINED
#define CHANNELS_INTEGERCHANNEL_C_DEFINED

#include "Channels/IntegerChannel.H"

#include "Channels/ChannelOpts.H"
#include "Image/IntegerMathOps.H" // for intgScaleFromFloat()
#include "Util/log.H"

// ######################################################################
IntegerChannel::IntegerChannel(OptionManager& mgr,
                               const std::string& descrName,
                               const std::string& tagName,
                               const VisualFeature vs,
                               nub::ref<IntegerMathEngine> eng)
  :
  ChannelBase(mgr, descrName, tagName, vs),
  itsMathEngine(eng)
{}

// ######################################################################
void IntegerChannel::inputInt(const IntegerInput& inp,
                              const SimTime& t,
                              PyramidCache<int>* cache,
                              const Image<byte>& clipMask)
{
  ASSERT(inp.initialized());

  if (!this->started())
    CLFATAL("must be start()-ed before using receiving any input");

  this->killCaches();

  this->setInputDims(inp.getDims());

  this->doInputInt(inp, t, cache, clipMask);
}

// ######################################################################
void IntegerChannel::doInput(const InputFrame& inframe)
{
  this->doInputInt(IntegerInput::fromRgb(inframe.colorByte(),
                                         this->getImath()->nbits),
                   inframe.time(), 0, inframe.clipMask());
}

// ######################################################################
const integer_math* IntegerChannel::getImath() const
{
  return itsMathEngine->getImath();
}

// ######################################################################
nub::ref<IntegerMathEngine> IntegerChannel::getMathEngine() const
{
  return itsMathEngine;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_INTEGERCHANNEL_C_DEFINED

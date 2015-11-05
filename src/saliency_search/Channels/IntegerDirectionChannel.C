/*!@file Channels/IntegerDirectionChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/IntegerDirectionChannel.C $
// $Id: IntegerDirectionChannel.C 8744 2007-09-05 21:17:53Z rjpeters $
//

#ifndef CHANNELS_INTEGERDIRECTIONCHANNEL_C_DEFINED
#define CHANNELS_INTEGERDIRECTIONCHANNEL_C_DEFINED

#include "Channels/IntegerDirectionChannel.H"

#include "Channels/ChannelOpts.H"
#include "Component/ModelOptionDef.H"
#include "Component/OptionManager.H"
#include "Image/ImageSetOps.H"
#include "Image/IntegerMathOps.H"
#include "Image/MathOps.H"
#include "Util/sformat.H"
#include "rutz/compat_cmath.h" // for M_PI
#include "rutz/trace.h"

// ######################################################################
// IntegerDirectionChannel member definitions:
// ######################################################################

// ######################################################################
IntegerDirectionChannel::
IntegerDirectionChannel(OptionManager& mgr,
                        const uint dirIndex,
                        const double direction,
                        const int dxnumer, const int dynumer,
                        const uint denombits,
                        nub::ref<IntegerMathEngine> eng) :
  IntegerSimpleChannel(mgr, "", "", MOTION,
                       rutz::make_shared
                       (new IntgReichardtPyrBuilder
                        (dxnumer, dynumer, denombits,
                         eng->getImath())),
                       eng),
  itsIndex("IntegerDirectionChannelIndex", this, dirIndex),
  itsDirection("IntegerDirectionChannelDirection", this, direction),
  itsThresh(&OPT_DirectionChannelLowThresh, this)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsTakeAbs.setVal(true);
  itsNormalizeOutput.setVal(true);
  itsScaleNoiseToMax.setVal(true);
  // kill small values in pyramid:
  const float fthresh = itsThresh.getVal();
  itsLowThresh.setVal(intgScaleFromFloat(&fthresh,
                                         this->getImath()->nbits));
  // kill negative values in pyramid:
  itsRectifyPyramid.setVal(true);

  setDescriptiveName(sformat("Integer Direction(%d)", int(direction)));
  setTagName(sformat("int-dir_%d", dirIndex));
}

// ######################################################################
void IntegerDirectionChannel::start1()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // kill small values in pyramid:
  const float fthresh = itsThresh.getVal();
  itsLowThresh.setVal(intgScaleFromFloat(&fthresh,
                                         this->getImath()->nbits));

  IntegerSimpleChannel::start1();
}

// ######################################################################
IntegerDirectionChannel::~IntegerDirectionChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_INTEGERDIRECTIONCHANNEL_C_DEFINED

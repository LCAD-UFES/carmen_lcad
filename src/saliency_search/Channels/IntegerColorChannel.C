/*!@file Channels/IntegerColorChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/IntegerColorChannel.C $
// $Id: IntegerColorChannel.C 7859 2007-02-07 21:56:06Z rjpeters $
//

#ifndef CHANNELS_INTEGERCOLORCHANNEL_C_DEFINED
#define CHANNELS_INTEGERCOLORCHANNEL_C_DEFINED

#include "Channels/IntegerColorChannel.H"

#include "Component/OptionManager.H"
#include "Image/ColorOps.H"
#include "Image/IntegerMathOps.H"
#include "Image/PyramidCache.H"
#include "rutz/trace.h"

// ######################################################################
// Double Opponent IntegerColorChannel member definitions:
// ######################################################################

IntegerColorChannel::
IntegerColorChannel(OptionManager& mgr,
                    nub::ref<IntegerMathEngine> eng)
  :
  IntegerComplexChannel(mgr, "Integer Color", "int-color", COLOR, eng),
  itsLumThresh("IntegerColorChannelLuminanceThreshold", this, 25.5F),
  itsRG(new IntegerSimpleChannel
        (mgr, "Integer Red/Green", "int-rg", RG,
         rutz::make_shared(new IntgGaussianPyrBuilder
                           (5, eng->getImath())),
         eng)),
  itsBY(new IntegerSimpleChannel
        (mgr, "Integer Blue/Yellow", "int-by", BY,
         rutz::make_shared(new IntgGaussianPyrBuilder
                           (5, eng->getImath())),
         eng))
{
GVX_TRACE(__PRETTY_FUNCTION__);
  this->addSubChan(itsRG);
  this->addSubChan(itsBY);

  itsRG->setTakeAbs(true);
  itsBY->setTakeAbs(true);
}

// ######################################################################
IntegerSimpleChannel& IntegerColorChannel::rg() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return *itsRG;
}

// ######################################################################
IntegerSimpleChannel& IntegerColorChannel::by() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return *itsBY;
}

// ######################################################################
IntegerColorChannel::~IntegerColorChannel()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void IntegerColorChannel::doInputInt(const IntegerInput& inp,
                                     const SimTime& t,
                                     PyramidCache<int>* cache,
                                     const Image<byte>& clipMask)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(inp.rgInt().initialized());
  ASSERT(inp.byInt().initialized());

  itsRG->inputInt(IntegerInput::fromGrayOnly(inp.rgInt()), t, cache, clipMask);
  itsBY->inputInt(IntegerInput::fromGrayOnly(inp.byInt()), t, cache, clipMask);
  LINFO("Double Opponent Color channel ok.");
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_INTEGERCOLORCHANNEL_C_DEFINED

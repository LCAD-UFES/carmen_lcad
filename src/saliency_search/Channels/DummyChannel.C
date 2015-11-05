/*!@file Channels/DummyChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/DummyChannel.C $
// $Id: DummyChannel.C 7434 2006-11-11 02:15:19Z rjpeters $
//

#ifndef DUMMYCHANNEL_C_DEFINED
#define DUMMYCHANNEL_C_DEFINED

#include "Channels/DummyChannel.H"

#include "Channels/ChannelOpts.H"
#include "Component/OptionManager.H"

// ######################################################################
// Dummy channel member definitions
// ######################################################################
DummyChannel::DummyChannel(OptionManager& mgr) :
  SingleChannel(mgr, "Dummy", "dummy", DUMMY,
                rutz::shared_ptr< PyrBuilder<float> >(NULL)),
  itsFactor(&OPT_DummyChannelFactor, this),
  itsMap()
{ }

// ######################################################################
DummyChannel::~DummyChannel()
{ }

// ######################################################################
bool DummyChannel::outputAvailable() const
{ return itsMap.initialized(); }

// ######################################################################
Dims DummyChannel::getMapDims() const
{ return itsMap.getDims(); }

// ######################################################################
void DummyChannel::doInput(const InputFrame& inframe)
{
  ASSERT(inframe.grayFloat().initialized());
  if (itsFactor.getVal()) itsMap = inframe.grayFloat() * itsFactor.getVal();
  else itsMap = inframe.grayFloat();
}

// ######################################################################
Image<float> DummyChannel::centerSurround(const uint cntrlev,
                                          const uint surrlev) const
{
  ASSERT(itsLevelSpec.getVal().csOK(cntrlev, surrlev));
  return itsMap;
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // DUMMYCHANNEL_C_DEFINED

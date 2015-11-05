/*!@file Channels/CompositeColorChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/CompositeColorChannel.C $
// $Id: CompositeColorChannel.C 7890 2007-02-09 19:43:37Z rjpeters $
//

#ifndef COMPOSITECOLORCHANNEL_C_DEFINED
#define COMPOSITECOLORCHANNEL_C_DEFINED

#include "Channels/CompositeColorChannel.H"

#include "Channels/ChannelOpts.H"

// ######################################################################
// Composite Color Channel member definitions: contains single and double
// opponent color channels
// ######################################################################
CompositeColorChannel::CompositeColorChannel(OptionManager& mgr)
  :
  ComplexChannel(mgr, "CompositeColor", "composite-color", COMPOSITECOLOR),
  itsDoubleOppWeight(&OPT_CompColorDoubleOppWeight, this),
  itsSingleOppWeight(&OPT_CompColorSingleOppWeight, this),
  itsDoubleOppChan(new ColorChannel(mgr)),
  itsSingleOppChan(new SOColorChannel(mgr))
{
  this->addSubChan(itsDoubleOppChan);
  setSubchanTotalWeight(*itsDoubleOppChan, 0.0); // we set the real weight in start1()

  this->addSubChan(itsSingleOppChan);
  setSubchanTotalWeight(*itsSingleOppChan, 0.0); // we set the real weight in start1()
}

// ######################################################################
CompositeColorChannel::~CompositeColorChannel() {}

// ######################################################################
void CompositeColorChannel::doInput(const InputFrame& inframe)
{
  for (uint i = 0; i < numChans(); ++i)
    subChan(i)->input(inframe);

  LINFO("CompositeColor channel ok.");
}

// ######################################################################
void CompositeColorChannel::start1()
{
  setSubchanTotalWeight(*itsDoubleOppChan, itsDoubleOppWeight.getVal());
  setSubchanTotalWeight(*itsSingleOppChan, itsSingleOppWeight.getVal());
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // COMPOSITECOLORCHANNEL_C_DEFINED

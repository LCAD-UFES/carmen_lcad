/*!@file Channels/InformationChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/InformationChannel.C $
// $Id: InformationChannel.C 7434 2006-11-11 02:15:19Z rjpeters $
//

#ifndef INFORMATIONCHANNEL_C_DEFINED
#define INFORMATIONCHANNEL_C_DEFINED

#include "Channels/InformationChannel.H"

#include "Image/ColorOps.H"

// ######################################################################
// Information channel member definitions
// ######################################################################
InformationChannel::InformationChannel(OptionManager& mgr) :
  SingleChannel(mgr, "Information", "information", INFORMATION,
                rutz::shared_ptr< PyrBuilder<float> >(NULL)),
  itsEps("InformationChannelEps", this, 0.01f),
  itsMap()
{ }

// ######################################################################
InformationChannel::~InformationChannel()
{ }

// ######################################################################
bool InformationChannel::outputAvailable() const
{ return itsMap.initialized(); }

// ######################################################################
void InformationChannel::doInput(const InputFrame& inframe)
{
  const LevelSpec ls = itsLevelSpec.getVal();
  ASSERT(ls.levMin() == ls.levMax());
  ASSERT(ls.delMin() == 0 && ls.delMax() == 0);
  ASSERT(ls.levMin() == ls.mapLevel());
  ASSERT(inframe.colorFloat().initialized());

  const uint lev = ls.mapLevel();

  // figure out the tile size and compute map:
  const int siz = 1 << lev;
  itsMap = infoMeasure(inframe.colorFloat(), itsEps.getVal(), siz); // Image_ColorOps.C
}

// ######################################################################
Image<float> InformationChannel::getOutput()
{ return itsMap; }

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // INFORMATIONCHANNEL_C_DEFINED

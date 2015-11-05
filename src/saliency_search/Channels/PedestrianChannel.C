/*!@file Channels/PedestrianChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/PedestrianChannel.C $
// $Id: PedestrianChannel.C 4679 2005-06-24 04:59:53Z rjpeters $
//

#ifndef PEDESTRIANCHANNEL_C_DEFINED
#define PEDESTRIANCHANNEL_C_DEFINED

#include "Channels/PedestrianChannel.H"

// ######################################################################
// PedestrianChannel member definitions:
// ######################################################################

namespace
{
  const int pedX = 9;
  const int pedY = 15;
  const float pedFILT[pedX*pedY] = {
    -3.0, -3.0, -3.0, -3.0, -3.0, -3.0, -3.0, -3.0, -3.0,
    -3.0, -3.0, -3.0,  0.0,  1.0,  0.0, -3.0, -3.0, -3.0,
    -3.0, -3.0,  0.0,  2.0,  3.0,  2.0,  0.0, -3.0, -3.0,
    -3.0,  0.0,  0.0,  1.0,  3.0,  1.0,  0.0,  0.0, -3.0,
    -3.0,  1.0,  2.0,  3.0,  3.0,  3.0,  2.0,  1.0, -3.0,
    -3.0,  1.0,  2.0,  3.0,  3.0,  3.0,  2.0,  1.0, -3.0,
    -3.0,  1.0,  2.0,  3.0,  3.0,  3.0,  2.0,  1.0, -3.0,
    -3.0,  1.0,  2.0,  3.0,  3.0,  3.0,  2.0,  1.0, -3.0,
    -3.0,  1.0,  2.0,  3.0,  3.0,  3.0,  2.0,  1.0, -3.0,
    -3.0,  0.0,  2.0,  3.0,  2.0,  3.0,  2.0,  0.0, -3.0,
    -3.0,  0.0,  2.0,  3.0,  0.0,  3.0,  2.0,  0.0, -3.0,
    -3.0,  0.0,  2.0,  3.0,  0.0,  3.0,  2.0,  0.0, -3.0,
    -3.0,  0.0,  2.0,  3.0,  0.0,  3.0,  2.0,  0.0, -3.0,
    -3.0,  0.0,  2.0,  3.0,  0.0,  3.0,  2.0,  0.0, -3.0,
    -3.0, -3.0, -3.0, -3.0, -3.0, -3.0, -3.0, -3.0, -3.0
  };
}

PedestrianChannel::PedestrianChannel(OptionManager& mgr) :
  TemplateMatchChannel(mgr, Image<float>(pedFILT, pedX, pedY)),
  itsMinScale("PedestrianChannelMinScale", this, 1),
  itsMaxScale("PedestrianChannelMaxScale", this, 3)
{  }

// ######################################################################
PedestrianChannel::~PedestrianChannel()
{  }

// ######################################################################
void PedestrianChannel::start1()
{
  TemplateMatchChannel::start1();

  // refuse to have our LevelSpec changed by the command line, because
  // we are special:
  itsLevelSpec.setVal(LevelSpec(itsMinScale.getVal(), itsMaxScale.getVal(),
                                0, 1, 2));
  LINFO("Using scales [%d..%d]", itsMinScale.getVal(), itsMaxScale.getVal());
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // PEDESTRIANCHANNEL_C_DEFINED

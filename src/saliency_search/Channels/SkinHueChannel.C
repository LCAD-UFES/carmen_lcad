/*!@file Channels/SkinHueChannel.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/SkinHueChannel.C $
// $Id: SkinHueChannel.C 4679 2005-06-24 04:59:53Z rjpeters $
//

#ifndef SKINHUECHANNEL_C_DEFINED
#define SKINHUECHANNEL_C_DEFINED

#include "Channels/SkinHueChannel.H"

#include "Image/colorDefs.H"

// ######################################################################
SkinHueChannel::SkinHueChannel(OptionManager& mgr) :
  HueChannel(mgr,COL_SKIN_MUR,COL_SKIN_MUG,
             COL_SKIN_SIGR,COL_SKIN_SIGG,COL_SKIN_RHO)
{}
/*
  itsMean[0] = 0.434904;
  itsMean[1] = 0.301983;
  itsInvCov[0] = 533.839800;
  itsInvCov[1] = 684.827315;
  itsInvCov[2] = 2565.283958;
*/

SkinHueChannel::~SkinHueChannel() {}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // SKINHUECHANNEL_C_DEFINED

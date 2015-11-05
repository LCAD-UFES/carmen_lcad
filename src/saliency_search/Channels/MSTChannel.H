/*!@file Channels/MSTChannel.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/MSTChannel.H $
// $Id: MSTChannel.H 13756 2010-08-04 21:57:32Z siagian $
//

#ifndef MSTCHANNEL_H_DEFINED
#define MSTCHANNEL_H_DEFINED

#include "Channels/SingleChannel.H"
#include "rutz/shared_ptr.h"

class OrientationChannel;
class MotionChannel;

//######################################################################
//! A MST channel.
class MSTChannel : public SingleChannel
{
public:
  /*! Construct with the manager, visual cortex, visual feature and
    relevance of the different oriented features (0 => 0 deg., 1 => 45
    deg, ... , 7 => 315 deg., and whether we want a full or partial
    implementation of the MST */
  MSTChannel(OptionManager& mgr, nub::soft_ref<MotionChannel> oc,
                  const VisualFeature vs = UNKNOWN,
                  const int R0 = 0,  const int R1 = 0,  const int R2 = 0,
                  const int R3 = 0,  const int R4 = 0,  const int R5 = 0,
                  const int R6 = 0,  const int R7 = 0 );

  //! Destructor
  virtual ~MSTChannel();

protected:
  //! Computes the corners from the gabor channels accessed via the
  //visual cortex
  virtual void doInput(const InputFrame& inframe);

  /*! do we want a slow, full implementation of this MST which
  checks for both the presence of relevant features and the absence of
  irrelevant features; or a quick, partial implementation which checks
  only for the presence of the relevant features. */
  OModelParam<bool> itsFull;

  /// dx and dy value to pass to MSTFilterFull() or MSTFilterPartial()
  OModelParam<uint> itsDelta;

  //! keep track of our OrientationChannel
  nub::soft_ref<MotionChannel> itsOriChan;

private:
  int R0; // relevance of   0 deg. feature
  int R1; // relevance of  45 deg. feature
  int R2; // relevance of  90 deg. feature
  int R3; // relevance of 135 deg. feature
  int R4; // relevance of 180 deg. feature
  int R5; // relevance of 225 deg. feature
  int R6; // relevance of 270 deg. feature
  int R7; // relevance of 315 deg. feature
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // MSTCHANNEL_H_DEFINED

/*!@file Psycho/StimMakerParam.H make different kind of visual test stimuli
 */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// Primary maintainer for this file: T. Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/StimMakerParam.H $
// $Id: StimMakerParam.H 6347 2006-03-06 15:18:42Z rjpeters $
//

#ifndef STIM_MAKER_PARAM_H_DEFINED
#define STIM_MAKER_PARAM_H_DEFINED

// Shape types
#define SM_STIM_RAND  0
#define SM_STIM_DISK  1
#define SM_STIM_RECT  2
#define SM_STIM_LINE  3
#define SM_STIM_CROSS 4
#define SM_STIM_PATCH 5
#define SM_STIM_ARROW 6
#define SM_STIM_CIRC  7

#define SM_SHAPES     7

// color types
#define SM_COLOR_RAND   0
#define SM_COLOR_RED    1
#define SM_COLOR_ORANGE 2
#define SM_COLOR_YELLOW 3
#define SM_COLOR_GREEN  4
#define SM_COLOR_BLUE   5
#define SM_COLOR_PURPLE 6
#define SM_COLOR_BLACK  7
#define SM_COLOR_WHITE  8
#define SM_COLOR_CUSTOM 9

#define SM_COLORS       8

// rates
#define SM_SLOW_RATE 10
#define SM_FAST_RATE 4
#define SM_STOP_RATE 0

// rate to accelerate or decelerate if needed
#define SM_SMOOTH_ACCEL 2

// rate types
#define SM_FAST_STIM 0
#define SM_SLOW_STIM 1
#define SM_NSPD_STIM 2

// state types
#define SM_STATE_STEADY 0
#define SM_STATE_STOP   1
#define SM_STATE_START  2

// define to use a random start time
#define SM_NO_USE_RANDOM_START 0
#define SM_USE_RANDOM_START    1

// define the type of rate change to use
#define SM_NO_USE_SMOOTH_RATE_CHANGE 0
#define SM_USE_SMOOTH_RATE_CHANGE    1

// define if we should use a hexagonal lattice
#define SM_NO_USE_HEXAGON            0
#define SM_USE_HEXAGON               1

#include "GUI/XWindow.H"
#include "Raster/Raster.H"
#include "Util/Assert.H"
#include "Util/Timer.H"
#include "Util/log.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/DrawOps.H"

#include <fstream>
#include <iostream>
#include <math.h>
#include <string>
#include <vector>

//! holder class for parameters used in StimMaker
class StimMakerParam
{
public:
  //! should the distractors be on (visible) at start
  bool           SMP_distOn;
  //! should the target be on (visible) at the start
  bool           SMP_targetOn;
  //! the width of the distractors in pixels (if zero then random)
  unsigned char  SMP_distSizeX;
  //! the height of the distractors in pixels (if zero then random)
  unsigned char  SMP_distSizeY;
  //! the color of the distractors from the table , 0 for random
  unsigned char  SMP_distColor;
  //! the shape of the distractors from the table , 0 for random
  unsigned char  SMP_distShape;
  //! the rate of blink for distractors from the table
  unsigned char  SMP_distRate;
  //! the state change to the distractor from the table
  unsigned char  SMP_distState;
  //! the width of the target in pixels (if zero then random)
  unsigned char  SMP_targetSizeX;
  //! the height of the target in pixels (if zero then random)
  unsigned char  SMP_targetSizeY;
  //! the color of the target from the table in, 0 for random
  unsigned char  SMP_targetColor;
  //! the shape of the target from the table in , 0 for random
  unsigned char  SMP_targetShape;
  //! the rate of blink for target from the table in
  unsigned char  SMP_targetRate;
  //! the state change to the target from the table in
  unsigned char  SMP_targetState;
  //! should the start "on" times be randomized
  unsigned char  SMP_useRandomStart;
  //! should we change rates gradually
  unsigned char  SMP_useSmoothRateChange;
  //! what kind of latice should we use
  unsigned char  SMP_useHexagon;
  //! the targets position row item number (not pixels in rows)
  unsigned short SMP_targetPosI;
  //! the targets position column item number (not pixels in column)
  unsigned short SMP_targetPosJ;
  //! the number of distractors per row
  unsigned short SMP_distPerRow;
  //! the number of distractors per column
  unsigned short SMP_distPerCol;
  //! random number seed to use, if 0 then ignore
  unsigned int   SMP_randomSeed;
  //! orientation of the distractors in radians 0.0F for no rotation
  float          SMP_distOri;
  //! orientation of the target in radians 0.0F for no rotation
  float          SMP_targetOri;
  //! jitter the position of objects in the video by this much
  float          SMP_shapePositionJitter;
  //! jitter the orientation of objects in the video by this much
  float          SMP_shapeOrientationJitter;
  //! jitter the position of objects in the video by this much
  float          SMP_shapePositionJitterStatic;
  //! jitter the orientation of objects in the video by this much
  float          SMP_shapeOrientationJitterStatic;
  //! default constructor
  StimMakerParam();
  //! default destructor
  ~StimMakerParam();
  //! set values for a nice default demo
  void setDemoParams1();
  //! set values for a nice default demo
  void setDemoParams2();
  //! set values for a nice default demo
  void setDemoParams3();
  //! set values for a nice default demo
  void setDemoParams4();
  //! set values for a nice default, some basics
  void setBasicParams1();
};

#endif // STIM_MAKER_PARAM_H_DEFINED

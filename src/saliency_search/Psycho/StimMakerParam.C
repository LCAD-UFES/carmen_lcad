/*!@file Psycho/StimMakerParam.C make different kind of visual test stimuli
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/StimMakerParam.C $
// $Id: StimMakerParam.C 6262 2006-02-17 22:40:07Z rjpeters $
//

#ifndef STIM_MAKER_PARAM_C_DEFINED
#define STIM_MAKER_PARAM_C_DEFINED

#include "Psycho/StimMakerParam.H"

/*****************************************************************************/

StimMakerParam::StimMakerParam()
{}

/*****************************************************************************/

StimMakerParam::~StimMakerParam()
{}

/*****************************************************************************/

void StimMakerParam::setDemoParams1()
{
  SMP_distOn                 = true;
  SMP_targetOn               = true;
  SMP_distSizeX              = 50;
  SMP_distSizeY              = 50;
  SMP_distColor              = SM_COLOR_RED;
  SMP_distShape              = SM_STIM_DISK;
  SMP_distRate               = SM_SLOW_STIM;
  SMP_distState              = SM_STATE_STEADY;
  SMP_targetSizeX            = 50;
  SMP_targetSizeY            = 50;
  SMP_targetColor            = SM_COLOR_RED;
  SMP_targetShape            = SM_STIM_DISK;
  SMP_targetRate             = SM_SLOW_STIM;
  SMP_targetState            = SM_STATE_START;
  SMP_useRandomStart         = SM_NO_USE_RANDOM_START;
  SMP_useSmoothRateChange    = SM_NO_USE_SMOOTH_RATE_CHANGE;
  SMP_targetPosI             = 3;
  SMP_targetPosJ             = 3;
  SMP_distPerRow             = 6;
  SMP_distPerCol             = 6;
  SMP_distOri                = 0.0F;
  SMP_targetOri              = 0.0F;
  SMP_shapePositionJitter    = 0.0F;
  SMP_shapeOrientationJitter = 0.0F;
  SMP_shapePositionJitterStatic    = 0.4F;
  SMP_shapeOrientationJitterStatic = 0.0F;
  SMP_randomSeed             = 100;
  SMP_useHexagon             = SM_NO_USE_HEXAGON;
}

/*****************************************************************************/

void StimMakerParam::setDemoParams2()
{
  SMP_distOn                 = true;
  SMP_targetOn               = true;
  SMP_distSizeX              = 50;
  SMP_distSizeY              = 50;
  SMP_distColor              = SM_COLOR_RAND;
  SMP_distShape              = SM_STIM_DISK;
  SMP_distRate               = SM_NSPD_STIM;
  SMP_distState              = SM_STATE_START;
  SMP_targetSizeX            = 50;
  SMP_targetSizeY            = 50;
  SMP_targetColor            = SM_COLOR_RAND;
  SMP_targetShape            = SM_STIM_DISK;
  SMP_targetRate             = SM_SLOW_STIM;
  SMP_targetState            = SM_STATE_STEADY;
  SMP_useRandomStart         = SM_NO_USE_RANDOM_START;
  SMP_useSmoothRateChange    = SM_NO_USE_SMOOTH_RATE_CHANGE;
  SMP_targetPosI             = 3;
  SMP_targetPosJ             = 3;
  SMP_distPerRow             = 6;
  SMP_distPerCol             = 6;
  SMP_distOri                = 0.0F;
  SMP_targetOri              = 0.0F;
  SMP_shapePositionJitter    = 0.2F;
  SMP_shapeOrientationJitter = 0.0F;
  SMP_shapePositionJitterStatic    = 0.0F;
  SMP_shapeOrientationJitterStatic = 0.0F;
  SMP_randomSeed             = 200;
  SMP_useHexagon             = SM_NO_USE_HEXAGON;
}

/*****************************************************************************/

void StimMakerParam::setDemoParams3()
{
  SMP_distOn                 = true;
  SMP_targetOn               = true;
  SMP_distSizeX              = 50;
  SMP_distSizeY              = 50;
  SMP_distColor              = SM_COLOR_RAND;
  SMP_distShape              = SM_STIM_RAND;
  SMP_distRate               = SM_NSPD_STIM;
  SMP_distState              = SM_STATE_START;
  SMP_targetSizeX            = 50;
  SMP_targetSizeY            = 50;
  SMP_targetColor            = SM_COLOR_RAND;
  SMP_targetShape            = SM_STIM_RAND;
  SMP_targetRate             = SM_NSPD_STIM;
  SMP_targetState            = SM_STATE_STEADY;
  SMP_useRandomStart         = SM_NO_USE_RANDOM_START;
  SMP_useSmoothRateChange    = SM_NO_USE_SMOOTH_RATE_CHANGE;
  SMP_targetPosI             = 3;
  SMP_targetPosJ             = 3;
  SMP_distPerRow             = 6;
  SMP_distPerCol             = 6;
  SMP_distOri                = M_PI/3.0F;
  SMP_targetOri              = -M_PI/4.0F;
  SMP_shapePositionJitter    = 0.0F;
  SMP_shapeOrientationJitter = 0.0F;
  SMP_shapePositionJitterStatic    = 0.0F;
  SMP_shapeOrientationJitterStatic = 0.1F;
  SMP_randomSeed             = 300;
  SMP_useHexagon             = SM_NO_USE_HEXAGON;
}

/*****************************************************************************/

void StimMakerParam::setDemoParams4()
{
  SMP_distOn                 = true;
  SMP_targetOn               = true;
  SMP_distSizeX              = 16;
  SMP_distSizeY              = 16;
  SMP_distColor              = SM_COLOR_RED;
  SMP_distShape              = SM_STIM_RECT;
  SMP_distRate               = SM_FAST_STIM;
  SMP_distState              = SM_STATE_STEADY;
  SMP_targetSizeX            = 16;
  SMP_targetSizeY            = 16;
  SMP_targetColor            = SM_COLOR_BLUE;
  SMP_targetShape            = SM_STIM_RECT;
  SMP_targetRate             = SM_NSPD_STIM;
  SMP_targetState            = SM_STATE_STEADY;
  SMP_useRandomStart         = SM_NO_USE_RANDOM_START;
  SMP_useSmoothRateChange    = SM_NO_USE_SMOOTH_RATE_CHANGE;
  SMP_targetPosI             = 9;
  SMP_targetPosJ             = 9;
  SMP_distPerRow             = 18;
  SMP_distPerCol             = 18;
  SMP_distOri                = M_PI/3.0F;
  SMP_targetOri              = -M_PI/4.0F;
  SMP_shapePositionJitter    = 0.2F;
  SMP_shapeOrientationJitter = 0.2F;
  SMP_shapePositionJitterStatic    = 0.2F;
  SMP_shapeOrientationJitterStatic = 0.2F;
  SMP_randomSeed             = 100;
  SMP_useHexagon             = SM_USE_HEXAGON;
}

/*****************************************************************************/

void StimMakerParam::setBasicParams1()
{
  SMP_distOn                 = true;
  SMP_targetOn               = true;
  SMP_distSizeX              = 16;
  SMP_distSizeY              = 16;
  SMP_targetSizeX            = 16;
  SMP_targetSizeY            = 16;
  SMP_useRandomStart         = SM_NO_USE_RANDOM_START;
  SMP_useSmoothRateChange    = SM_NO_USE_SMOOTH_RATE_CHANGE;
  SMP_targetPosI             = 5;
  SMP_targetPosJ             = 5;
  SMP_distPerRow             = 9;
  SMP_distPerCol             = 9;
  SMP_randomSeed             = 100;
  SMP_useHexagon             = SM_USE_HEXAGON;
}
#endif

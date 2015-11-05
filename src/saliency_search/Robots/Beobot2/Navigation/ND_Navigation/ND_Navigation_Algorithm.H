/*!@file
   Robots2/Beobot2/Navigation/ND_Navigation/ND_Navigation_algorithm.H
   Ice Nearness Difference algorithm to navigate indoors using LRF  */
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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Navigation/ND_Navigation/ND_Navigation.H
// $Id: ND_Navigation.H 12962 2010-03-06 02:13:53Z irock $
//

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Util/Timer.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include <IceUtil/Thread.h>

#include "Robots/Beobot2/BeoCommon.H"

#include "Media/FrameSeries.H"

#include "Image/Image.H"
#include "GUI/XWinManaged.H"

#include <vector>

#ifndef ND_NAVIGATION_ALGORITHM_H
#define ND_NAVIGATION_ALGORITHM_H

typedef struct _RegionInformation
{
  uint start;
  uint end;

  float minDistance;
  uint minIndex;
}
RegionInformation;

class ND_Navigation_Algorithm
{
public:

  ND_Navigation_Algorithm();

  ~ND_Navigation_Algorithm();

  void setGoalSector(uint goal);

  Beobot2::MotorCommand computeNavigationCommand
  (std::vector<double> distances, std::vector<double> angles);

  void start1();

  //! check whether we are safe to move forward
  bool isSafe();

private:
  void findGapsAndRegions();

  void setMinimumDistances();

  void findGoalRegion();

  void biasForSafety();

  void drawSituation();

  void drawRangeLine(float dist, int angle, PixRGB<byte> c);

  bool isNavigableRegion(uint index);

  std::string itsLogFilename;
  IceUtil::Mutex its_logFilename_mutex;        //!< locking log filename


  std::vector<double> itsDistances;
  std::vector<double> itsAngles;

  float itsMinDistance;
  float itsMaxDistance;
  uint itsGoalSector;
  uint itsGoalRegion;

  uint itsNextDirection;
  uint itsPrevDirection;
  uint itsLeftEnd;
  uint itsRightEnd;

  uint itsBigJumpCount;

  std::vector<uint> itsGaps;
  std::vector<RegionInformation > itsRegions;

  nub::soft_ref<OutputFrameSeries> itsOfs;
  Image<PixRGB<byte> > itsDispImg;

  Timer itsTimer;
  int itsCurrMessageID;

  bool itsIsSafe;

  //! debug window
  rutz::shared_ptr<XWinManaged> itsWin;
};

#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/*!@file Robots/Beobot2/Navigation/BeoNavigator.H integrates various
 vision, LRF, and encoder data to create and navigate using a single
 local grid occupancy map. We use A* shortest path algorithm and takes
 into account the closest obstacle to generate the motor command */
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
// Primary maintainer for this file: Christian Siagian <siagian@caltech.edu>
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Navigation/BeoNavigator.H
// $Id: $
//

// has openCV must be put at first line
#include "Robots/Beobot2/Navigation/LocalMap.H"

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Robots/Beobot2/BeoCommon.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include <IceUtil/Thread.h>
#include "Util/Timer.H"

#include "Image/Image.H"
#include "GUI/Button.H"
#include "Media/FrameSeries.H"


#ifndef BEOBOT2_BEONAVIGATORI_H
#define BEOBOT2_BEONAVIGATORI_H

// ######################################################################


// ######################################################################
//! Navigation by integrating various sensory
class BeoNavigator : public RobotBrainComponent
{
public:

  BeoNavigator(OptionManager& mgr,
               const std::string& descrName = "BeoNavigator",
               const std::string& tagName   = "BeoNavigator");

  ~BeoNavigator();

  virtual void evolve();

  //! Get a message
  virtual void updateMessage
  (const RobotSimEvents::EventMessagePtr& eMsg,
   const Ice::Current&);

  virtual void registerTopics();

  void start1();
  
private:

  //! send motor command to BeoPilot
  void updateMotor(double tran,double rot);

  //! draw local map display
  Image<PixRGB<byte> > getLocalMapImage
  ( double gridWidth, double gridHeight, 
    Image<double> gridOccupancyMap,
    Image<double> immediateGridOccupancyMap,
    std::vector<double> distances, std::vector<double> angles,
    std::vector<double> immediateDistances,
    std::vector<double> immediateAngles,
    std::vector<Point3D<double> >predictedNextLocations,
    Point2D<double> currAccTrajectory,
    std::vector<Point2D<float> > path,
    std::vector<Point2D<float> > smoothedPath,
    double desiredHeading, double selfLocalHeading,
    Beobot2::MotorCommand currentMotorCommand,
    double currentTransVelCommand, double currentRotVelCommand,
    std::vector<Point2D<int> > goalNodes,
    Point2D<int> goalNode,
    std::vector<Point2D<double> > centers, 
    int dead_end_indicator, Image<float> histogram,
    double curr_global_heading, double road_global_heading, 
    double road_confidence,
    float lateral_deviation, float lateral_deviation_angle,
    std::vector<float> obstacleDist 
);

  //! draw angled rectangles
  std::vector<Point2D<int> > drawAngledRectangle
  (Rectangle body, Point2D<float> pivot, double robotAngle, 
   Image<PixRGB<byte> > &disp, PixRGB<byte> color);

  void publishScreenShot(Image<PixRGB<byte> > img);

  //! handle user input event from keyboard and mouse
  void handleUserEvent();

  //! reset the navigator
  void reset();

  //! display image
  Image<PixRGB<byte> > itsDispImg;

  //!Debug windows
  DebugWin itsDebugWin;
  //! odometry inputs  
  double          itsCurrentIMUHeading;
  double          itsCurrentGlobalHeading;
  Point2D<double> itsDiffPosition;

  IceUtil::Mutex its_Curr_Mtr_mutex;        //!< locking motor related vars
  IceUtil::Mutex its_Curr_Lrf_mutex;        //!< locking LRF related vars
  IceUtil::Mutex its_Local_Map_mutex;       //!< locking Local Map related vars
  IceUtil::Mutex its_Traversal_Map_mutex;   //!< locking Local Map related vars
  IceUtil::Mutex its_Update_Index_mutex;    //!< locking update index vars
 
  //! LRF information
  std::vector<double> itsDistances;
  std::vector<double> itsAngles;

  //! last inputted robot heading
  Image<float> itsTraversalMap;
  int itsTraversalMapIndex;
  //int itsLastProcTraversalMapIndex;

  //! estimated vanishing point heading
  double itsEstimatedRobotHeading; 

  //! estimated lateral deviation
  float itsLateralDeviation;   

  //! information to keep track of display
  int itsLastDisplayIndex;  
  int itsLastUpdateIndex;

  //! local map
  rutz::shared_ptr<LocalMap> itsLocalMap;

  //! display related variables
  nub::soft_ref<OutputFrameSeries> itsOfs;
  Timer itsTimer;
  int itsCurrentMessageID;

  //! buttons
  Button itsResetGoalButton;
};
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

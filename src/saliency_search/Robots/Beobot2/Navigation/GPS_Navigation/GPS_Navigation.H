/*!@file Robots2/Beobot2/Navigation/GPS_Navigation/GPS_Navigation.H Ice Module to Log data    */
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
#include "Robots/Beobot2/Navigation/ND_Navigation/ND_Navigation_Algorithm.H"
#include "Util/Timer.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include <IceUtil/Thread.h>

#include "Robots/Beobot2/BeoCommon.H"

#include "Media/FrameSeries.H"

#include "Image/Image.H"

#include <vector>

#ifndef GPS_NAVIGATIONI_H
#define GPS_NAVIGATIONI_H

class GPS_Navigation : public RobotBrainComponent
{
public:

  GPS_Navigation(OptionManager& mgr,
                const std::string& descrName = "ND_Navigation",
                const std::string& tagName = "ND_Navigation");

  ~GPS_Navigation();
  virtual void evolve();

  //! Get a message
  virtual void updateMessage
  (const RobotSimEvents::EventMessagePtr& eMsg,
   const Ice::Current&);

  virtual void registerTopics();

  void start1();

private:
  Beobot2::MotorCommand computeGPS_Navigation
  (double lat, double lon, double xin, double yin);

  Point2D<double> getCurrentHeading
  (double lat, double lon, double x, double y);

  Beobot2::MotorCommand computeHeading
  (double heading, double idealHeading);

  Beobot2::MotorCommand recover
  (double heading, double idealHeading);

  void updateMotor(double tran,double rot);

  std::vector<Point2D<double> > itsGoalWayPoints;
  std::vector<double >          itsGoalHeadings;

  Point2D<double> itsCurrentStartLocation;
  Point2D<double> itsCurrentGoalLocation;
  double          itsCurrentGoalHeading;
  bool            itsAtGoal;

  std::vector<Point2D<double> > itsPrevGPScoords;

  nub::soft_ref<OutputFrameSeries> itsOfs;
  Image<PixRGB<byte> > itsDispImg;

  Timer itsTimer;
  int itsCurrNumGPScoords;

  std::vector<double> itsDistances;
  std::vector<double> itsAngles;
  
  double itsCurrentHeading;

  rutz::shared_ptr<ND_Navigation_Algorithm> itsNDNavigationAlgorithm;
  Point2D<double> itsAvoidanceTrajectory;
  Point2D<double> itsCurrentMovement;
  Timer itsAvoidanceTimer;
  bool itsIsSafe;
  uint itsOffPathState;
  double itsDistanceToPath;

  Beobot2::MotorCommand itsNDcommand;
  Beobot2::MotorCommand itsHeadingCommand;
};

#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/*!@file SupplementaryMotorArea.H generate a sequance of movment for the robot to follow */


//////////////////////////////////////////////////////////////////// //
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
// Primary maintainer for this file: Lior Elazary <lelazary@yahoo.com>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/RobotBrain/SupplementaryMotorAreaI.H $
// $Id: SupplementaryMotorAreaI.H 12281 2009-12-17 09:00:36Z itti $
//

#ifndef SupplementaryMotorAreaI_H_DEFINED
#define SupplementaryMotorAreaI_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "GUI/XWinManaged.H"
#include "GUI/ImageDisplayStream.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Controllers/PID.H"
#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/IceImageUtils.H"
#include "Ice/SimEventsUtils.H"
#include <IceUtil/Thread.h>


class SupplementaryMotorAreaI : public ModelComponent, public RobotBrainObjects::SupplementaryMotorArea,
  public IceUtil::Thread
{

public:

  SupplementaryMotorAreaI(OptionManager& mgr,
          const std::string& descrName = "SupplementaryMotorArea",
          const std::string& tagName = "SupplementaryMotorArea");
  ~SupplementaryMotorAreaI();

  virtual void run();

  void evolve(); //Our evolve function

  //!Get a gps message and compute the location
  virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
      const Ice::Current&);

  //!Initalize the object
  void init(Ice::CommunicatorPtr ic, Ice::ObjectAdapterPtr adapter);


private:
  Ice::ObjectPrx    itsObjectPrx;
  RobotSimEvents::EventsPrx itsActionEventsPub;
  RobotSimEvents::EventsPrx itsGoalProgressEventsPub;
  std::vector<SimEventsUtils::TopicInfo> itsTopicsSubscriptions;
  RobotSimEvents::StateMessagePtr itsCurrentState;
  RobotSimEvents::StateMessagePtr itsGoalState;
  RobotSimEvents::GoalProgressMessagePtr itsGoalProgress;
  RobotSimEvents::ActionMessagePtr itsCurrentAction;
  PID<float> itsTransPID;
  PID<Angle> itsRotPID;

};

#endif

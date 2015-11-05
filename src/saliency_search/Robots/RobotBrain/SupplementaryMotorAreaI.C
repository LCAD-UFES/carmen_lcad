/*!@file SupplementaryMotorAreaI.C generate a sequance of movment for the robot to follow */
//This is the controller which will have a path to follow



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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/RobotBrain/SupplementaryMotorAreaI.C $
// $Id: SupplementaryMotorAreaI.C 10794 2009-02-08 06:21:09Z itti $
//

#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "GUI/ImageDisplayStream.H"
#include "Robots/RobotBrain/SupplementaryMotorAreaI.H"


// ######################################################################
SupplementaryMotorAreaI::SupplementaryMotorAreaI(OptionManager& mgr,
    const std::string& descrName, const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsCurrentState(new RobotSimEvents::StateMessage),
  itsGoalState(new RobotSimEvents::StateMessage),
  itsGoalProgress(new RobotSimEvents::GoalProgressMessage),
  itsCurrentAction(new RobotSimEvents::ActionMessage),
  itsTransPID(1.0f, 0.0, 0.0, //PID
      -20, 20, //iMin and iMax
      0, 0, 0, //ErrThreash //posThreash //negThresh,
      1, -1), //max and min motor commands
  itsRotPID(0.60f, 0.0, 0.0,  //PID
      -20, 20, //iMin and iMax
      0, 0, 0, //ErrThreash //posThreash //negThresh,
      1, -1) //max and min motor commands
{

  itsGoalState->xPos = 0;
  itsGoalState->yPos = 0;
  itsGoalState->orientation = 0;

  itsCurrentState->xPos = 0;
  itsCurrentState->yPos = 0;
  itsCurrentState->orientation = 0;

}

// ######################################################################
SupplementaryMotorAreaI::~SupplementaryMotorAreaI()
{
  SimEventsUtils::unsubscribeSimEvents(itsTopicsSubscriptions, itsObjectPrx);
}

// ######################################################################
void SupplementaryMotorAreaI::init(Ice::CommunicatorPtr ic, Ice::ObjectAdapterPtr adapter)
{
        Ice::ObjectPtr objPtr = this;
  itsObjectPrx = adapter->add(objPtr,
      ic->stringToIdentity("SupplementaryMotorArea"));


  IceStorm::TopicPrx topicPrx;

  itsTopicsSubscriptions.push_back(SimEventsUtils::TopicInfo("StateMessageTopic", topicPrx));
  itsTopicsSubscriptions.push_back(SimEventsUtils::TopicInfo("GoalStateMessageTopic", topicPrx));

  SimEventsUtils::initSimEvents(ic, itsObjectPrx, itsTopicsSubscriptions);

  itsActionEventsPub = RobotSimEvents::EventsPrx::uncheckedCast(
      SimEventsUtils::getPublisher(ic, "ActionMessageTopic")
      );

  itsGoalProgressEventsPub = RobotSimEvents::EventsPrx::uncheckedCast(
      SimEventsUtils::getPublisher(ic, "GoalProgressMessageTopic")
      );

  IceUtil::ThreadPtr thread = this;
  thread->start();

  usleep(10000);
}

// ######################################################################
void SupplementaryMotorAreaI::run()
{

  while(1)
  {
    evolve();
    usleep(10000);
  }

}

// ######################################################################
void SupplementaryMotorAreaI::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{
  if(eMsg->ice_isA("::RobotSimEvents::StateMessage"))
  {
    RobotSimEvents::StateMessagePtr stateMsg = RobotSimEvents::StateMessagePtr::dynamicCast(eMsg);
    itsCurrentState->xPos = stateMsg->xPos;
    itsCurrentState->yPos = stateMsg->yPos;
    itsCurrentState->orientation = stateMsg->orientation;
  } else if(eMsg->ice_isA("::RobotSimEvents::GoalStateMessage"))
  {
    RobotSimEvents::GoalStateMessagePtr goalStateMsg = RobotSimEvents::GoalStateMessagePtr::dynamicCast(eMsg);
    itsGoalState->xPos = goalStateMsg->xPos;
    itsGoalState->yPos = goalStateMsg->yPos;
    itsGoalState->orientation = goalStateMsg->orientation;
  }
}

// ######################################################################
void SupplementaryMotorAreaI::evolve()
{


  double err = (itsGoalState->xPos-itsCurrentState->xPos)*(itsGoalState->xPos-itsCurrentState->xPos);
  err += (itsGoalState->yPos-itsCurrentState->yPos)*(itsGoalState->yPos-itsCurrentState->yPos);
  err = sqrt(err);

  LDEBUG("Desired: %fx%fx%f pos %fx%fx%f err=%0.2f",
      itsGoalState->xPos, itsGoalState->yPos, itsGoalState->orientation,
      itsCurrentState->xPos, itsCurrentState->yPos, itsCurrentState->orientation,
      err);

  itsCurrentAction->transVel = 0.0;
  itsCurrentAction->rotVel = 0;


  if (err < 0.1F) //err is 10 cm
  {
    itsGoalProgress->err = -1;
    LDEBUG("Goal reached, next");
  } else {

    itsGoalProgress->err = err;
    float desiredHeading =
      atan2((itsGoalState->yPos-itsCurrentState->yPos),(itsGoalState->xPos-itsCurrentState->xPos));
    desiredHeading += M_PI;
    //if (desiredHeading > M_PI*2) desiredHeading -= M_PI*2;


    //We basicly want the err to be 0, so if its not then we need to move
    itsCurrentAction->transVel = itsTransPID.update(err, 0);
    //LINFO("Trans %f", itsDesiredTransVel);
    if (itsCurrentAction->transVel > 0.2)
      itsCurrentAction->transVel = 0.2;

    Angle g(desiredHeading);
    Angle c(itsCurrentState->orientation);
    Angle e = itsRotPID.update(g, c);
    LDEBUG("%f-%f =>%f", desiredHeading*180/M_PI, itsCurrentState->orientation*180/M_PI, e.getVal());

    itsCurrentAction->rotVel = (double)e.getVal();

    //The more steering we need, the less we should move
    //If we are moving straight, the move the fastest,
    //If all we need is to turn, then turn in place
    itsCurrentAction->transVel -= fabs(e.getVal()); //the more sterring we need, the less we should move
    if (itsCurrentAction->transVel < 0)
      itsCurrentAction->transVel = 0;
    //If we are moveing
  }

  LDEBUG("Setting action to  transVel=%f rotVel=%f",
  itsCurrentAction->transVel,
  itsCurrentAction->rotVel);

  itsActionEventsPub->updateMessage(itsCurrentAction);
  itsGoalProgressEventsPub->updateMessage(itsGoalProgress);

}

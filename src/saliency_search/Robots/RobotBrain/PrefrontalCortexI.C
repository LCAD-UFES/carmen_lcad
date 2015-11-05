/*!@file Hippocampus.C maintains the current thought location of the robot */
//This modules invovles in the perception

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/RobotBrain/PrefrontalCortexI.C $
// $Id: PrefrontalCortexI.C 10794 2009-02-08 06:21:09Z itti $
//

#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "GUI/ImageDisplayStream.H"
#include "Robots/RobotBrain/PrefrontalCortexI.H"


// ######################################################################
PrefrontalCortexI::PrefrontalCortexI(OptionManager& mgr,
    const std::string& descrName, const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsGoalState(new RobotSimEvents::GoalStateMessage),
  itsGoalProgress(new RobotSimEvents::GoalProgressMessage),
  itsCurrentWaypointId(0)
{
  itsGoalState->xPos = 0;
  itsGoalState->yPos = 0;
  itsGoalState->orientation = 0;

  itsGoalProgress->err = -1.0f;
}

// ######################################################################
PrefrontalCortexI::~PrefrontalCortexI()
{
  SimEventsUtils::unsubscribeSimEvents(itsTopicsSubscriptions, itsObjectPrx);
}


// ######################################################################
void PrefrontalCortexI::init(Ice::CommunicatorPtr ic, Ice::ObjectAdapterPtr adapter)
{
        Ice::ObjectPtr objPtr = this;
  itsObjectPrx = adapter->add(objPtr,
      ic->stringToIdentity("PreforntalCortex"));


  IceStorm::TopicPrx topicPrx;

  itsTopicsSubscriptions.push_back(SimEventsUtils::TopicInfo("GoalProgressMessageTopic", topicPrx));

  SimEventsUtils::initSimEvents(ic, itsObjectPrx, itsTopicsSubscriptions);

  itsEventsPub = RobotSimEvents::EventsPrx::uncheckedCast(
      SimEventsUtils::getPublisher(ic, "GoalStateMessageTopic")
      );

  IceUtil::ThreadPtr thread = this;
  thread->start();

  usleep(10000);
}


// ######################################################################
void PrefrontalCortexI::run()
{

  while(1)
  {
    evolve();
    usleep(10000);
  }

}

// ######################################################################
void PrefrontalCortexI::evolve()
{
  return; //Dont drive the robot

  if (itsGoalProgress->err == -1)
  {
    switch(itsCurrentWaypointId)
    {
      case 0:
        itsGoalState->xPos = -18;
        itsGoalState->yPos = -1.2;
        itsGoalState->orientation = 0;
        break;
      case 1:
        itsGoalState->xPos = -18;
        itsGoalState->yPos = 20;
        itsGoalState->orientation = 0;
        break;
      case 2:
        itsGoalState->xPos = 0;
        itsGoalState->yPos = 18;
        itsGoalState->orientation = 0;
        break;
      case 3:
        itsGoalState->xPos = 0;
        itsGoalState->yPos = 0;
        itsGoalState->orientation = 0;
        break;
    }
    itsCurrentWaypointId++;
    itsCurrentWaypointId = itsCurrentWaypointId%4;

    itsEventsPub->updateMessage(itsGoalState);
  }

  LDEBUG("Got goal progress Waypoint:%i err:%f",
      itsCurrentWaypointId,
      itsGoalProgress->err);

}

// ######################################################################
void PrefrontalCortexI::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{
  //Get a gps message
  if(eMsg->ice_isA("::RobotSimEvents::GoalProgressMessage"))
  {
    RobotSimEvents::GoalProgressMessagePtr gpMsg = RobotSimEvents::GoalProgressMessagePtr::dynamicCast(eMsg);
    itsGoalProgress->err = gpMsg->err;
  }
}


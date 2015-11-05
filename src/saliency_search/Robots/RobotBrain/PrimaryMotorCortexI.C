/*!@file PrimaryMotorCortexI.C drive the actual robot */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/RobotBrain/PrimaryMotorCortexI.C $
// $Id: PrimaryMotorCortexI.C 10794 2009-02-08 06:21:09Z itti $
//

#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "GUI/ImageDisplayStream.H"
#include "Robots/RobotBrain/PrimaryMotorCortexI.H"


#include "Ice/IceImageUtils.H"

// ######################################################################
PrimaryMotorCortexI::PrimaryMotorCortexI(OptionManager& mgr,
    const std::string& descrName, const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName)
{

}



// ######################################################################
void PrimaryMotorCortexI::init(Ice::CommunicatorPtr ic, Ice::ObjectAdapterPtr adapter)
{
  Ice::ObjectPtr pmcPtr = this;
  itsObjectPrx = adapter->add(pmcPtr,
      ic->stringToIdentity("PrimaryMotorCortex"));

  IceStorm::TopicPrx topicPrx;
  itsTopicsSubscriptions.push_back(SimEventsUtils::TopicInfo("ActionMessageTopic", topicPrx));
  SimEventsUtils::initSimEvents(ic, itsObjectPrx, itsTopicsSubscriptions);

  Ice::ObjectPrx base = ic->stringToProxy(
      "IRobotService:default -p 10000 -h roomba");
  itsRobot = Robots::IRobotPrx::checkedCast(base);

  if(!itsRobot) LFATAL("Invalid Robot Proxy");
  itsRobot->sendStart();
  itsRobot->setMode(Robots::SafeMode);
}

// ######################################################################
PrimaryMotorCortexI::~PrimaryMotorCortexI()
{
  //TODO:Convert to itsRobot->Off()
  LINFO("Motors off");
  itsRobot->motorsOff(0);

  SimEventsUtils::unsubscribeSimEvents(itsTopicsSubscriptions, itsObjectPrx);

}

// ######################################################################
void PrimaryMotorCortexI::stop2()
{
  LINFO("Motors off");
  itsRobot->motorsOff(0);
}

// ######################################################################
void PrimaryMotorCortexI::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{
  if(eMsg->ice_isA("::RobotSimEvents::ActionMessage"))
  {
    RobotSimEvents::ActionMessagePtr actionMsg = RobotSimEvents::ActionMessagePtr::dynamicCast(eMsg);

    if (!itsRobot)
      LFATAL("Invalid robot proxy");

    itsRobot->setSteering(actionMsg->rotVel);
    itsRobot->setSpeed(actionMsg->transVel);
  }
}

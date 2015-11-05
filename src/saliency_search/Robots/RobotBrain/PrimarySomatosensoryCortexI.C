/*!@file PrimarySomatosensoryCortex.C drive the actual robot */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/RobotBrain/PrimarySomatosensoryCortexI.C $
// $Id: PrimarySomatosensoryCortexI.C 10794 2009-02-08 06:21:09Z itti $
//

#include "Component/ModelManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "GUI/ImageDisplayStream.H"
#include "Robots/RobotBrain/PrimarySomatosensoryCortexI.H"
#include "Robots/RobotBrain/RobotCommon.H"

#include "Ice/IceImageUtils.H"

// ######################################################################
PrimarySomatosensoryCortexI::PrimarySomatosensoryCortexI(OptionManager& mgr,
    const std::string& descrName, const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName)
{
}

// ######################################################################
void PrimarySomatosensoryCortexI::init(Ice::CommunicatorPtr ic, Ice::ObjectAdapterPtr adapter)
{
  Ice::ObjectPtr pscPtr = this;
  itsObjectPrx = adapter->add(pscPtr,
      ic->stringToIdentity("PrimarySomatosensoryCortex"));

  itsGPSPublisher = RobotSimEvents::EventsPrx::uncheckedCast(
        SimEventsUtils::getPublisher(ic, "GPSMessageTopic")
        );
  itsMotionPublisher = RobotSimEvents::EventsPrx::uncheckedCast(
        SimEventsUtils::getPublisher(ic, "MotionMessageTopic")
        );

  Ice::ObjectPrx base = ic->stringToProxy(
      "IRobotService:default -p 10000 -h " ROBOT_IP);
  itsRobot = Robots::IRobotPrx::checkedCast(base);

  if(!itsRobot) LFATAL("Invalid Robot Proxy");

  IceUtil::ThreadPtr thread = this;
  thread->start();

  usleep(10000);
}

// ######################################################################
PrimarySomatosensoryCortexI::~PrimarySomatosensoryCortexI()
{
  //SimEventsUtils::unsubscribeSimEvents(itsTopicsSubscriptions, itsObjectPrx);
}

// ######################################################################
void PrimarySomatosensoryCortexI::run() {

  while(1) {
    //Get the GPS data from our robot implementation
    float xPos; float yPos; float orientation;
    if (itsRobot->getSensors(xPos,yPos,orientation))
    {
      //Pack that data up and publish it
      RobotSimEvents::GPSMessagePtr gpsMessage =
        new RobotSimEvents::GPSMessage;

      gpsMessage->xPos = xPos;
      gpsMessage->yPos = yPos;
      gpsMessage->orientation = orientation;

      itsGPSPublisher->updateMessage(gpsMessage);
    }

    //Get an odometry sensor
    float dist; float ang;
    if (itsRobot->getDistanceAngle(dist, ang))
    {
      RobotSimEvents::MotionMessagePtr motionMessage =
        new RobotSimEvents::MotionMessage;

      motionMessage->distance = dist;
      motionMessage->angle = ang;

      itsMotionPublisher->updateMessage(motionMessage);
    }

    usleep(10000);
  }
}

// ######################################################################
void PrimarySomatosensoryCortexI::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{
}

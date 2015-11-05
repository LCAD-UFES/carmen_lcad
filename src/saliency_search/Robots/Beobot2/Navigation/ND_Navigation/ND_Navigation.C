/*!@file Robots2/Beobot2/Navigation/ND_Navigation/ND_Navigation.C Ice Module to navigate indoors using LRF    */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/ND_Navigation.C
// $ $Id: ND_Navigation.C 15046 2011-11-03 04:09:28Z beobot $
//
//////////////////////////////////////////////////////////////////////////

#include "Robots/Beobot2/Navigation/ND_Navigation/ND_Navigation.H"
#include "Ice/BeobotEvents.ice.H"

#include "Raster/Raster.H"
#include "Util/sformat.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "Transport/FrameInfo.H"

#include "Ice/IceImageUtils.H"

#define  SecurityNearnessRad 1000.0

// ######################################################################
ND_Navigation::ND_Navigation(OptionManager& mgr,
               const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsOfs(new OutputFrameSeries(mgr)),
  itsTimer(1000000),
  itsNDNavigationAlgorithm(new ND_Navigation_Algorithm())
{
  addSubComponent(itsOfs);

  // goal is always in front of the robot FOR NOW
  // this should be set by the Localizer at some point
  itsGoalSector = 141;
  itsNDNavigationAlgorithm->setGoalSector(itsGoalSector);
}

// ######################################################################
ND_Navigation::~ND_Navigation()
{ }

// ######################################################################
void ND_Navigation::start1()
{
}

// ######################################################################
void ND_Navigation::registerTopics()
{
  // subscribe to all sensor data
  this->registerSubscription("LRFMessageTopic");
  this->registerPublisher("MotorRequestTopic");
}

// ######################################################################
void ND_Navigation::evolve()
{ }

// ######################################################################
void ND_Navigation::updateMessage
(const RobotSimEvents::EventMessagePtr& eMsg, const Ice::Current&)
{
  // LRF message
  if(eMsg->ice_isA("::BeobotEvents::LRFMessage"))
  {
    // we got LRF data
    BeobotEvents::LRFMessagePtr lrfMsg =
      BeobotEvents::LRFMessagePtr::dynamicCast(eMsg);

    itsDistances = lrfMsg->distances;
    itsAngles    = lrfMsg->angles;

    LINFO("received LRF message: %d,", lrfMsg->RequestID);

    // compute navigation
    Beobot2::MotorCommand cmd = 
      itsNDNavigationAlgorithm->computeNavigationCommand
      (itsDistances, itsAngles);

    LINFO("received: %f %f", cmd.translation, cmd.rotation);

    // send to BeoPilot
    updateMotor(cmd.translation,cmd.rotation);
  }
}

// ######################################################################
void ND_Navigation::updateMotor(double tran,double rot)
{
    BeobotEvents::MotorRequestPtr msg = new BeobotEvents::MotorRequest;
    msg->transVel = tran;
    msg->rotVel   = rot;
    this->publish("MotorRequestTopic", msg);

}
// ######################################################################

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/*!@file Robots2/Beobot2/Hardware/SLAM_Navigation.C
 SLAM HNB basement navigation                                 */
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
// Primary maintainer for this file: Josh Villbrandt <josh.villbrandt@usc.edu>
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Navigation/SLAM_Navigation/SLAM_Navigation.C
// $ $Id: SLAM_Navigation.C 12962 2010-03-06 02:13:53Z irock $
//
//////////////////////////////////////////////////////////////////////////

#include "Robots/Beobot2/Navigation/SLAM_Navigation/SLAM_Navigation.H"
#include "Ice/BeobotEvents.ice.H"

#include "Raster/Raster.H"
#include "Util/sformat.H"
#include "Image/Image.H"
#include "Ice/IceImageUtils.H"

// ######################################################################
SLAM_Navigation::SLAM_Navigation(OptionManager& mgr,
               const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsTimer(1000000)
  //  itsOfs(new OutputFrameSeries(mgr))
{
  //  addSubComponent(itsOfs);

}

// ######################################################################
SLAM_Navigation::~SLAM_Navigation()
{ }

// ######################################################################
void SLAM_Navigation::start1()
{
}

// ######################################################################
void SLAM_Navigation::registerTopics()
{
  // subscribe to all sensor data
  this->registerSubscription("SLAMMessageTopic");
  this->registerPublisher("MotorRequestTopic");
}

// ######################################################################
void SLAM_Navigation::evolve()
{ }

// ######################################################################
Beobot2::MotorCommand SLAM_Navigation::computeSLAM_Navigation()
{
/*//   static float rotation = 0;
//   static bool dir = false;

//   if(dir)  rotation+=.01;
//   if(!dir) rotation-=.01;
//   if(rotation > 1 || rotation < -1) dir = !dir;

//   Beobot2::MotorCommand cmd;
//   cmd.rotation = rotation;
//   cmd.translation = 0;

  // check the front 50 degree devided into equal sized zones
  int  rangeStart = -50;  // NOTE: make sure range start is smaller
  int  rangeEnd   =  49;  //       than end
  uint numZones   =   10;
  LINFO("Range to check (%d, %d) in %d zones",
        rangeStart, rangeEnd, numZones);

  // regions much like the sonar
  std::vector<float> straightAheadAvg(numZones);
  int total = rangeEnd - rangeStart;
  int startIndex = rangeStart + 141;
  for(uint j = 0; j < numZones; j++)
  {
    int frontStart = startIndex + (total*float(j)/numZones);
    int frontEnd   = startIndex + (total*float(j+1)/numZones) - 1;
    LINFO("Range to check (%d, %d)", frontStart, frontEnd);
    for(int i = frontStart; i <= frontEnd; i++)
      {
        straightAheadAvg[j] += itsDistances[i];
      }
    straightAheadAvg[j] /= float(frontEnd - frontStart);
    LINFO("straight Ahead Average[%d]: %f mm",
          j, straightAheadAvg[j]);
  }

  // command to be executed by the BeoPilot
  Beobot2::MotorCommand cmd;

  // stop if the average distance in any region is less than 50cm
  bool stopNow = false;
  for(uint j = 0; j < numZones; j++)
    if(straightAheadAvg[j] < 500.0) stopNow = true;
  if(stopNow)
    {
      cmd.rotation = 0;
      cmd.translation = 0;
      LINFO(" BAM Stop");
    }
  else
    {
      // find the difference of distance
      // between the left and rightmost
      float diff = straightAheadAvg[numZones - 1] - straightAheadAvg[0];

      float rot = diff/3000.0;
      if(rot >  1.0) rot =  1.0;
      if(rot < -1.0) rot = -1.0;
      cmd.rotation =  rot;
      LINFO("diff: %f -> rot: %f ", diff, rot);

      cmd.translation = 1.0;
    }

  return cmd;*/
  Beobot2::MotorCommand cmd;
  return cmd;
}

// ######################################################################
void SLAM_Navigation::updateMessage
(const RobotSimEvents::EventMessagePtr& eMsg, const Ice::Current&)
{
  // SLAM message
  if(eMsg->ice_isA("::BeobotEvents::SLAMMessage"))
  {
    /*// we got LRF data
    BeobotEvents::LRFMessagePtr lrfMsg =
      BeobotEvents::LRFMessagePtr::dynamicCast(eMsg);

    itsDistances = lrfMsg->distances;
    itsAngles    = lrfMsg->angles;

    // compute navigation
    Beobot2::MotorCommand cmd = computeSLAM_Navigation();

    // send to BeoPilot
                updateMotor(cmd.translation,cmd.rotation);*/
  }
}

// ######################################################################
void SLAM_Navigation::updateMotor(double tran, double rot)
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

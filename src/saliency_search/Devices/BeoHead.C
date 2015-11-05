/*!@file Devices/BeoHead.C Interfaces to robot head */

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/BeoHead.C $
// $Id: BeoHead.C 15310 2012-06-01 02:29:24Z itti $
//

#include "Devices/BeoHead.H"
#include "Component/OptionManager.H"
#include "Util/MathFunctions.H"
#include "Util/Assert.H"
#include "rutz/compat_snprintf.h"
#include <unistd.h>

// ######################################################################
BeoHead::BeoHead(OptionManager& mgr, const std::string& descrName,
                   const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsLeftEyePanPos(0),
  itsRightEyeTiltPos(0),
  itsRightEyePanPos(0),
  itsLeftEyeTiltPos(0),
  itsHeadPanPos(0),
  itsHeadTiltPos(0.67),
  itsHeadYawPos(0),
  //pidLeftPan(0.6, 0.002, 0.2, -1.0, 1.0),
  //pidLeftTilt(0.4, 0.002, 0.2, -1.0, 1.0),
  //pidHeadPan(0.1, 0, 0, -1.0, 1.0)
  pidLeftPan(0.2, 0, 0.4, -1.0, 1.0),
  pidLeftTilt(0.2, 0, 0.3, -1.0, 1.0),
  pidHeadPan(0.05, 0, 0, -1.0, 1.0),
  isNeckRelaxed(true)

{
  itsPololu = nub::soft_ref<Pololu>(new Pololu(mgr));
  addSubComponent(itsPololu);

}


// ######################################################################
bool BeoHead::setLeftEyePan(float pos)
{
  if (pos > 1.0) pos = 1.0;
  if (pos < -1.0) pos = -1.0;
  itsLeftEyePanPos = pos;
  int rawPos = int(63.5*pos+63.5);
  return itsPololu->moveRaw(LEFT_EYE_PAN, rawPos);
}

// ######################################################################
bool BeoHead::setLeftEyeTilt(float pos)
{
  if (pos > 1.0) pos = 1.0;
  if (pos < -1.0) pos = -1.0;
  int rawPos = int(63.5*pos+63.5);
  itsLeftEyeTiltPos = pos;
  return itsPololu->moveRaw(LEFT_EYE_TILT, rawPos);
}

// ######################################################################
bool BeoHead::setRightEyePan(float pos)
{
  if (pos > 1.0) pos = 1.0;
  if (pos < -1.0) pos = -1.0;
  int rawPos = int(63.5*pos+63.5);
  itsRightEyePanPos = pos;
  return itsPololu->moveRaw(RIGHT_EYE_PAN, rawPos);
}

// ######################################################################
bool BeoHead::setRightEyeTilt(float pos)
{
  if (pos > 1.0) pos = 1.0;
  if (pos < -1.0) pos = -1.0;
  int rawPos = int(63.5*pos+63.5);
  itsRightEyeTiltPos = pos;
  return itsPololu->moveRaw(RIGHT_EYE_TILT, rawPos);
}

// ######################################################################
bool BeoHead::setHeadPan(float pos)
{
  if (isNeckRelaxed) return false;
  if (pos > 1.0) pos = 1.0;
  if (pos < -1.0) pos = -1.0;
  itsHeadPanPos = pos;
  int rawPos = int(63.5*pos+63.5);
  return itsPololu->moveRaw(HEAD_PAN, rawPos);
}

// ######################################################################
bool BeoHead::setHeadTilt(float pos)
{
  if (isNeckRelaxed) return false;
  if (pos > 1.0) pos = 1.0;
  if (pos < -1.0) pos = -1.0;
  int rawLeftPos =  int(63.5*(pos+itsHeadYawPos)+63.5);
  int rawRightPos =  int(63.5*(pos-itsHeadYawPos)+63.5);

  itsHeadTiltPos = pos;

  if (rawLeftPos > 0 && rawLeftPos < 127 &&
      rawRightPos > 0 && rawRightPos < 127)
  {
    itsPololu->moveRaw(HEAD_LEFT, rawLeftPos);
    itsPololu->moveRaw(HEAD_RIGHT, rawRightPos);
  }

  return true;
}

// ######################################################################
bool BeoHead::setHeadYaw(float pos)
{
  if (isNeckRelaxed) return false;

  if (pos > 1.0) pos = 1.0;
  if (pos < -1.0) pos = -1.0;
  int rawLeftPos =  int(63.5*(pos+itsHeadTiltPos)+63.5);
  int rawRightPos =  int(-63.5*(pos-itsHeadTiltPos)+63.5);

  itsHeadYawPos = pos;

  if (rawLeftPos > 0 && rawLeftPos < 127 &&
      rawRightPos > 0 && rawRightPos < 127)
  {
    itsPololu->moveRaw(HEAD_LEFT, rawLeftPos);
    itsPololu->moveRaw(HEAD_RIGHT, rawRightPos);
  }
  return true;
}

bool BeoHead::relaxNeck()
{
  if (!isNeckRelaxed)
  {
    setHeadPan(0);
    setHeadTilt(0.67);
    setHeadYaw(0);
    sleep(2);
  }

  itsPololu->setParam(HEAD_PAN, 0, 0, 10);
  itsPololu->setParam(HEAD_RIGHT, 0, 0, 3);
  itsPololu->setParam(HEAD_LEFT, 0, 1, 3);
  isNeckRelaxed = true;
  return true;
}

bool BeoHead::relaxHead()
{
  itsPololu->setParam(LEFT_EYE_TILT, 0, 0, 13);
  itsPololu->setParam(LEFT_EYE_PAN, 0, 0, 13);
  itsPololu->setParam(RIGHT_EYE_TILT, 0, 1, 13);
  itsPololu->setParam(RIGHT_EYE_PAN, 0, 0, 13);

  itsPololu->setParam(HEAD_PAN, 0, 0, 10);
  itsPololu->setParam(HEAD_RIGHT, 0, 0, 3);
  itsPololu->setParam(HEAD_LEFT, 0, 1, 3);

  return true;
}

bool BeoHead::moveRestPos()
{
  isNeckRelaxed = false;

  itsPololu->setParam(LEFT_EYE_TILT, 1, 0, 13);
  itsPololu->setNeutral(LEFT_EYE_TILT,3180);
  itsPololu->setParam(LEFT_EYE_PAN, 1, 0, 13);
  itsPololu->setNeutral(LEFT_EYE_PAN,2850);
  itsPololu->setParam(RIGHT_EYE_TILT, 1, 1, 13);
  itsPololu->setNeutral(RIGHT_EYE_TILT,3024);
  itsPololu->setParam(RIGHT_EYE_PAN, 1, 0, 13);
  itsPololu->setNeutral(RIGHT_EYE_PAN,3150);


  setLeftEyePan(0);
  setLeftEyeTilt(0);
  setRightEyePan(0);
  setRightEyeTilt(0);

  itsPololu->setSpeed(HEAD_PAN, 1);
  itsPololu->setParam(HEAD_PAN, 0, 0, 10);
  itsPololu->setNeutral(HEAD_PAN,3024);
  itsPololu->setSpeed(HEAD_RIGHT, 1);
  itsPololu->setParam(HEAD_RIGHT, 0, 0, 3);
  itsPololu->setNeutral(HEAD_RIGHT,3624);
  itsPololu->setSpeed(HEAD_LEFT, 1);
  itsPololu->setParam(HEAD_LEFT, 0, 1, 3);
  itsPololu->setNeutral(HEAD_LEFT,2334);

  setHeadPan(0);
  setHeadTilt(0.67);
  setHeadYaw(0);

  itsPololu->setSpeed(HEAD_PAN, 20);
  itsPololu->setSpeed(HEAD_RIGHT, 20);
  itsPololu->setSpeed(HEAD_LEFT, 20);

  isNeckRelaxed = false;
  return true;
}

//input should be normalized from 0 to 1
float BeoHead::trackTarget(float desiredVisualPosX, float desiredVisualPosY,
    float currentVisualPosX, float currentVisualPosY)
{

  float leftEyeMovePan = itsLeftEyePanPos +
      pidLeftPan.update(desiredVisualPosX, currentVisualPosX);

  float leftEyeMoveTilt = itsLeftEyeTiltPos +
      pidLeftTilt.update(desiredVisualPosY, currentVisualPosY);

  float moveHeadPan = itsHeadPanPos +
    pidHeadPan.update(0, leftEyeMovePan);

  //make the movement
  setLeftEyePan(leftEyeMovePan);
  setLeftEyeTilt(leftEyeMoveTilt);
  setRightEyePan(leftEyeMovePan);
  setRightEyeTilt(leftEyeMoveTilt);

  setHeadPan(moveHeadPan);

  //the total error in position
  float err = fabs(desiredVisualPosX-currentVisualPosX) +
              fabs(desiredVisualPosY-currentVisualPosY);
  return err;
}


// ######################################################################
void BeoHead::start2()
{
   initHead();
}

// ######################################################################
void BeoHead::initHead()
{
  LINFO("Init Servos...");
  itsPololu->setParam(LEFT_EYE_TILT, 1, 0, 13);
  itsPololu->setNeutral(LEFT_EYE_TILT,3180);
  //itsPololu->setSpeed(LEFT_EYE_TILT, 30);

  itsPololu->setParam(LEFT_EYE_PAN, 1, 0, 13);
  itsPololu->setNeutral(LEFT_EYE_PAN,2850);
  //itsPololu->setSpeed(LEFT_EYE_PAN, 30);

  itsPololu->setParam(RIGHT_EYE_TILT, 1, 1, 13);
  itsPololu->setNeutral(RIGHT_EYE_TILT,3024);

  itsPololu->setParam(RIGHT_EYE_PAN, 1, 0, 13);
  itsPololu->setNeutral(RIGHT_EYE_PAN,3150);

  //start with the neck off
  itsPololu->setSpeed(HEAD_PAN, 20);
  itsPololu->setParam(HEAD_PAN, 0, 0, 10);
  itsPololu->setNeutral(HEAD_PAN,3024);


  itsPololu->setSpeed(HEAD_RIGHT, 20);
  itsPololu->setParam(HEAD_RIGHT, 0, 0, 3);
  itsPololu->setNeutral(HEAD_RIGHT,3624);

  itsPololu->setSpeed(HEAD_LEFT, 20);
  itsPololu->setParam(HEAD_LEFT, 0, 1, 3);
  itsPololu->setNeutral(HEAD_LEFT,2334);

}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

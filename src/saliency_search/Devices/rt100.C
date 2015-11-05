/*!@file Devices/rt100.C Interface to a UMI rt-100 robot arm */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/rt100.C $
// $Id: rt100.C 7833 2007-02-01 23:01:39Z rjpeters $
//

#include "Devices/rt100.H"

#include "Component/OptionManager.H"
#include "Devices/Serial.H"

// ######################################################################
RT100::RT100(OptionManager& mgr, const std::string& descrName,
         const std::string& tagName, const char *defdev) :
  ModelComponent(mgr, descrName, tagName),
  itsPort(new Serial(mgr)),
  itsWristTiltVal(0),
  itsWristRollVal(0)
{
  // set a default config for our serial port:
  itsPort->configure(defdev, 9600, "8N1", false, true, 0); //use hardware flow

  // attach our port as a subcomponent:
  addSubComponent(itsPort);

}

// ######################################################################
RT100::~RT100()
{ }


// ######################################################################
int RT100::getNumJoints()
{
  return NUM_JOINTS;
}

// ######################################################################
int RT100::setJointParam(JOINTS joint, PARAM param, short int val)
{

  switch(joint)
  {
    case BASE1:
      break;
    case BASE2:
      break;
    case WRIST1:
      deferredWrite(IP0, IP0WRIST1, param, val);
      break;
    case WRIST2:
      deferredWrite(IP0, IP0WRIST2, param, val);
      break;
    case SENSOR:
      break;
    case ELBOW:
      deferredWrite(IP1, IP1ELBOW, param, val);
      break;
    case SHOLDER:
      deferredWrite(IP1, IP1SHOLDER, param, val);
      break;
    case YAW:
      deferredWrite(IP1, IP1YAW, param, val);
      break;
    case ZED:
      deferredWrite(IP1, IP1ZED, param, val);
      break;
    case GRIPPER:
      deferredWrite(IP1, IP1GRIPPER, param, val);
      break;

    case ROLL_WRIST:
      break;

    case TILT_WRIST:
      break;

    default:
      LFATAL("Invalid joint");
      break;

  }

  return RT100_OK;
}


// ######################################################################
int RT100::setJointPosition(JOINTS joint, short int position, bool immediate)
{
  unsigned char resCode;
  int wrist1Pos, wrist2Pos;

  switch(joint)
  {
    case BASE1:
      break;
    case BASE2:
      break;
    case WRIST1:
      deferredWrite(IP0, IP0WRIST1, NEW_POSITION, position);
      break;
    case WRIST2:
      deferredWrite(IP0, IP0WRIST2, NEW_POSITION, position);
      break;
    case SENSOR:
      break;
    case ELBOW:
      deferredWrite(IP1, IP1ELBOW, NEW_POSITION, position);
      break;
    case SHOLDER:
      deferredWrite(IP1, IP1SHOLDER, NEW_POSITION, position);
      break;
    case YAW:
      deferredWrite(IP1, IP1YAW, NEW_POSITION, position);
      break;
    case ZED:
      deferredWrite(IP1, IP1ZED, NEW_POSITION, position);
      break;
    case GRIPPER:
      deferredWrite(IP1, IP1GRIPPER, NEW_POSITION, position);
      break;

    case ROLL_WRIST:
      itsWristRollVal = position;
      wrist1Pos = itsWristTiltVal + position;
      wrist2Pos = itsWristTiltVal + -1*position;
      deferredWrite(IP0, IP0WRIST1, NEW_POSITION, wrist1Pos);
      deferredWrite(IP0, IP0WRIST2, NEW_POSITION, wrist2Pos);
      break;

    case TILT_WRIST:
      itsWristTiltVal = position;
      wrist1Pos = itsWristRollVal + position;
      wrist2Pos = -1*itsWristRollVal + position;
      deferredWrite(IP0, IP0WRIST1, NEW_POSITION, wrist1Pos);
      deferredWrite(IP0, IP0WRIST2, NEW_POSITION, wrist2Pos);
      break;

    default:
      LFATAL("Invalid joint");
      break;

  }

  if (immediate)
  {
    rawCommand(IP0, 0xAC, &resCode);
    rawCommand(IP1, 0xBF, &resCode);
  }


  return RT100_OK;
}

int RT100::getJointPosition(JOINTS joint, short int *position)
{
  ASSERT(position != NULL);

  switch (joint)
  {
    case BASE1:
      break;
    case BASE2:
      break;
    case WRIST1:
      immediateRead(IP0, IP0WRIST1, CURRENT_POSITION, position);
      break;
    case WRIST2:
      immediateRead(IP0, IP0WRIST2, CURRENT_POSITION, position);
      break;
    case ROLL_WRIST:
      break;
    case TILT_WRIST:
      break;
    case SENSOR:
      break;
    case ELBOW:
      immediateRead(IP1, IP1ELBOW, CURRENT_POSITION, position);
      break;
    case SHOLDER:
      immediateRead(IP1, IP1SHOLDER, CURRENT_POSITION, position);
      break;
    case YAW:
      immediateRead(IP1, IP1YAW, CURRENT_POSITION, position);
      break;
    case ZED:
      immediateRead(IP1, IP1ZED, CURRENT_POSITION, position);
      break;
    case GRIPPER:
      immediateRead(IP1, IP1GRIPPER, CURRENT_POSITION, position);
      break;

    default:
      LFATAL("Invalid joint");
      break;
  }

  return RT100_OK;
}

// ######################################################################
int RT100::moveArm(bool waitUntilComplete)
{

  unsigned char resCode, byte1, byte2;
  rawCommand(IP0, 0xAC, &resCode);
  rawCommand(IP1, 0xBF, &resCode);

  if (waitUntilComplete)
  {
    bool moveDone = false;
    for(int i=0; i<1000 && !moveDone; i++) //timeout
    {
      rawCommand(IP1, 0x17, &resCode, &byte1, &byte2); //general status
      if (!(byte1 & (1 << TASK_IN_PROGRESS))) //check if no movment in the last time
        moveDone = true;
    }
  }


  return RT100_OK;

}

bool RT100::moveComplete()
{
  unsigned char resCode, byte1, byte2;

  rawCommand(IP1, 0x17, &resCode, &byte1, &byte2); //general status
  if (!(byte1 & (1 << TASK_IN_PROGRESS))) //check if no movment in the last time
    return true;
  else
    return false;
}


// ######################################################################
int RT100::interpolationMove(std::vector<short int> &moveVals)
{
  unsigned char resCode;
  ASSERT(moveVals.size() == NUM_JOINTS);

  unsigned char ip0Byte1 = 0x00, ip0Byte2 = 0x00, ip0Byte3 = 0x00;
  unsigned char ip1Byte1 = 0x00, ip1Byte2 = 0x00, ip1Byte3 = 0x00;


  for(unsigned int joint = 0; joint<NUM_JOINTS; joint++)
  {
    short int jointVal = moveVals[joint];
    if (jointVal > 7 || jointVal < -8)
      LINFO("WARANING!!! joint %i is out of range (-8 < %i < 7)", joint, moveVals[joint]);

    jointVal = jointVal & 0x0F; //trancate to 4 bits


  enum IP0MOTORS {IP0BASE1, IP0BASE2, IP0WRIST1, IP0WRIST2, IP0SENSOR};
    switch (joint)
    {
      case BASE1:
        ip0Byte3 = 0x00;
        break;
      case BASE2:
        ip0Byte3 = 0x00;
        break;
      case WRIST1:
        ip0Byte3 += jointVal;
        break;
      case WRIST2:
        ip0Byte3 = ip0Byte3 + (jointVal << 4);
        break;
      case ROLL_WRIST:
        break;
      case TILT_WRIST:
        break;
      case SENSOR:
        ip0Byte1 = 0xC0 ;
        break;
      case ELBOW:
        ip1Byte2 += jointVal;
        break;
      case SHOLDER:
        ip1Byte2 = ip1Byte2 + (jointVal << 4);
        break;
      case YAW:
        ip1Byte3 = ip1Byte3 + (jointVal << 4);
        break;
      case ZED:
        ip1Byte3 += jointVal;
        break;
      case GRIPPER:
        ip1Byte1 = 0xC0 + jointVal;
        break;

      default:
        LFATAL("Invalid joint");
        break;
    }
  }

  //only send if we have a movement
  if (ip0Byte1 || ip0Byte2 || ip0Byte3)
  {
    LDEBUG("Sending interpolation command to IP0: %x %x %x", ip0Byte1, ip0Byte2, ip0Byte3);
    rawCommand(IP0, ip0Byte1, ip0Byte2, ip0Byte3, &resCode);
  }

  if (ip1Byte1 || ip1Byte2 || ip1Byte3)
  {
    LDEBUG("Sending interpolation command to IP1: %x %x %x", ip1Byte1, ip1Byte2, ip1Byte3);
    rawCommand(IP1, ip1Byte1, ip1Byte2, ip1Byte3, &resCode);
  }


  return RT100_OK;

}

// ######################################################################
int RT100::init()
{
  unsigned char resCode;
  unsigned cmd;
  LINFO("Initalizing arm to home position...");

  //init
  cmd = 0x00;
  itsPort->write(&cmd,1);
  usleep(10000); //TODO remove

  //reset IPs
  //rawCommand(IP0, 0x20, &resCode);
  //rawCommand(IP1, 0x20, &resCode);


  //free off (enable PWM output)
  rawCommand(IP0, EMERGENCY_STOP + FREE_OFF, &resCode);
  rawCommand(IP1, EMERGENCY_STOP + FREE_OFF, &resCode);


  initZed();
  initGripper();
  initWrist();
  initSholder();
  initElbowYaw();

  //set the current position of all joints
  immediateWrite(IP1, IP1ELBOW, CURRENT_POSITION, 62907);
  immediateWrite(IP1, IP1SHOLDER, CURRENT_POSITION, 2629);
  immediateWrite(IP1, IP1ZED, CURRENT_POSITION, 0);
  immediateWrite(IP1, IP1YAW, CURRENT_POSITION, 63588);
  immediateWrite(IP1, IP1GRIPPER, CURRENT_POSITION, 0);

  immediateWrite(IP0, IP0WRIST1, CURRENT_POSITION, 62435);
  immediateWrite(IP0, IP0WRIST2, CURRENT_POSITION, 459);

  gotoHomePosition();

  //define home position
 // rawCommand(IP0, 0x21, &resCode);
 // rawCommand(IP1, 0x21, &resCode);

  return RT100_OK;
}

void RT100::gotoHomePosition()
{

  unsigned char resCode;
  deferredWrite(IP1, IP1ELBOW, NEW_POSITION, 0);
  deferredWrite(IP1, IP1SHOLDER, NEW_POSITION, 0);
  deferredWrite(IP1, IP1ZED, NEW_POSITION, -400); //fffb
  deferredWrite(IP1, IP1YAW, NEW_POSITION, 0);
  deferredWrite(IP1, IP1GRIPPER, NEW_POSITION, 0);

  deferredWrite(IP0, IP0WRIST1, NEW_POSITION, 0);
  deferredWrite(IP0, IP0WRIST2, NEW_POSITION, 0);

  rawCommand(IP0, 0xAC, &resCode); //drive all motors home
  rawCommand(IP1, 0xBF, &resCode); //drive all motors home

  sleep(2);

}

void RT100::initZed()
{
  unsigned char resCode, byte1, byte2;
  deferredWrite(IP1, IP1ZED, SPEED, 30);
  deferredWrite(IP1, IP1ZED, MAX_FORCE, 30);

  rawCommand(IP0, EMERGENCY_STOP + DEAD_STOP, &resCode);
  rawCommand(IP1, EMERGENCY_STOP + DEAD_STOP, &resCode);

  rawCommand(IP0, (unsigned char)0x80, (unsigned char)0x00, (unsigned char)0x00, &resCode); //all off
  rawCommand(IP1, 0x80, 0x10, 0x00, &resCode); //drive zed up

  //check if we hit the end stop
  bool hitEndStop = false;
  for(int i=0; i<500 && !hitEndStop; i++)
  {
    rawCommand(IP1, 0x10 + IP1ZED, &resCode, &byte1, &byte2);
    if (byte1 & (1 << NO_MOVEMENT)) //check if no movment in the last time
     hitEndStop = true;
  }
  rawCommand(IP1, (unsigned char)0x80, (unsigned char)0x00, (unsigned char)0x00, &resCode); //all off

  if (!hitEndStop)
    LFATAL("ERROR: Can not init zed");

  //define home position
  rawCommand(IP0, 0x21, &resCode);
  rawCommand(IP1, 0x21, &resCode);

}

void RT100::initGripper()
{
  unsigned char resCode, byte1, byte2;

  deferredWrite(IP1, IP1GRIPPER, SPEED, 30);
  deferredWrite(IP1, IP1GRIPPER, MAX_FORCE, 30);

  rawCommand(IP0, EMERGENCY_STOP + DEAD_STOP, &resCode);
  rawCommand(IP1, EMERGENCY_STOP + DEAD_STOP, &resCode);

  rawCommand(IP0, (unsigned char)0x80, (unsigned char)0x00, (unsigned char)0x00, &resCode); //all off
  rawCommand(IP1, 0x80, 0x00, 0x02, &resCode); //gripper close

  //check if we hit the end stop
  bool hitEndStop = false;
  for(int i=0; i<500 && !hitEndStop; i++)
  {
    rawCommand(IP1, 0x10 + IP1GRIPPER, &resCode, &byte1, &byte2);
    if (byte1 & (1 << NO_MOVEMENT)) //check if no movment in the last time
     hitEndStop = true;
  }


  rawCommand(IP1, (unsigned char)0x80, (unsigned char)0x00, (unsigned char)0x00, &resCode); //all off

  if (!hitEndStop)
    LFATAL("ERROR: Can not init zed");

  //define home position
  rawCommand(IP0, 0x21, &resCode);
  rawCommand(IP1, 0x21, &resCode);

  //reload pids
  rawCommand(IP0, 0x22, &resCode);
  rawCommand(IP1, 0x22, &resCode);


  //open the gripper a bit and set that as the 0 position

  deferredWrite(IP1, IP1GRIPPER, NEW_POSITION, 30);

  rawCommand(IP1, EMERGENCY_STOP + DEAD_STOP, &resCode);
  rawCommand(IP1, 0xA0|(1<<IP1GRIPPER), &resCode); //numeric drive gripper

  //check if we hit the end stop
  bool taskComplete = false;
  for(int i=0; i<500 && !taskComplete; i++)
  {
    rawCommand(IP1, 0x10 + IP1GRIPPER, &resCode, &byte1, &byte2);
    if (byte1 & (1 << TASK_COMPLETE)) //check if no movment in the last time
      taskComplete = true;
  }

  //define home position
  rawCommand(IP0, 0x21, &resCode);
  rawCommand(IP1, 0x21, &resCode);

  //reload pids
  rawCommand(IP0, 0x22, &resCode);
  rawCommand(IP1, 0x22, &resCode);

}

void RT100::initWrist()
{
  unsigned char resCode, byte1, byte2;

  //TODO: move elbow a bit. Is this nessesary

  deferredWrite(IP0, IP0WRIST1, SPEED, 80);
  deferredWrite(IP0, IP0WRIST2, SPEED, 80);

  rawCommand(IP0, EMERGENCY_STOP + DEAD_STOP, &resCode);
  rawCommand(IP1, EMERGENCY_STOP + DEAD_STOP, &resCode);


  //Rotate the wrist
  rawCommand(IP1, (unsigned char)0x80, (unsigned char)0x00, (unsigned char)0x00, &resCode); //all off
  rawCommand(IP0, 0x80, 0x60, 0x00, &resCode); //rotate wrist all the way

  //check if we hit the end stop
  bool hitEndStop = false;
  for(int i=0; i<500 && !hitEndStop; i++)
  {
    rawCommand(IP0, 0x10 + IP0WRIST1, &resCode, &byte1, &byte2);
    if (byte1 & (1 << NO_MOVEMENT)) //check if no movment in the last time
     hitEndStop = true;
  }

  rawCommand(IP0, (unsigned char)0x80, (unsigned char)0x00, (unsigned char)0x00, &resCode); //all off

  if (!hitEndStop)
    LFATAL("ERROR: Can not init zed");

  //Move the wrist down
  rawCommand(IP0, 0x80, 0xa0, 0x00, &resCode); //rotate wrist all the way

  //check if we hit the end stop
   hitEndStop = false;
  for(int i=0; i<500 && !hitEndStop; i++)
  {
    rawCommand(IP0, 0x10 + IP0WRIST1, &resCode, &byte1, &byte2);
    if (byte1 & (1 << NO_MOVEMENT)) //check if no movment in the last time
     hitEndStop = true;
  }

  rawCommand(IP0, (unsigned char)0x80, (unsigned char)0x00, (unsigned char)0x00, &resCode); //all off

  if (!hitEndStop)
    LFATAL("ERROR: Can not init zed");

  rawCommand(IP0, EMERGENCY_STOP + DEAD_STOP, &resCode);
  rawCommand(IP1, EMERGENCY_STOP + DEAD_STOP, &resCode);


  //define home position
  rawCommand(IP0, 0x21, &resCode);
  rawCommand(IP1, 0x21, &resCode);

  //reload pids
  rawCommand(IP0, 0x22, &resCode);
  rawCommand(IP1, 0x22, &resCode);

}

void RT100::initSholder()
{
  unsigned char resCode, byte1, byte2;
  deferredWrite(IP1, IP1SHOLDER, SPEED, 30);
  deferredWrite(IP1, IP1SHOLDER, MAX_FORCE, 20);

  rawCommand(IP0, EMERGENCY_STOP + DEAD_STOP, &resCode);
  rawCommand(IP1, EMERGENCY_STOP + DEAD_STOP, &resCode);

  rawCommand(IP0, (unsigned char)0x80, (unsigned char)0x00, (unsigned char)0x00, &resCode); //all off
  rawCommand(IP1, 0x80, 0x04, 0x00, &resCode); //move sholder to end point

  //check if we hit the end stop
  bool hitEndStop = false;
  for(int i=0; i<500 && !hitEndStop; i++)
  {
    rawCommand(IP1, 0x10 + IP1SHOLDER, &resCode, &byte1, &byte2);
    if (byte1 & (1 << NO_MOVEMENT)) //check if no movment in the last time
     hitEndStop = true;
  }
  rawCommand(IP1, (unsigned char)0x80, (unsigned char)0x00, (unsigned char)0x00, &resCode); //all off

  if (!hitEndStop)
    LFATAL("ERROR: Can not init sholder");

  //define home position
  rawCommand(IP0, 0x21, &resCode);
  rawCommand(IP1, 0x21, &resCode);

  //reload pids
  rawCommand(IP0, 0x22, &resCode);
  rawCommand(IP1, 0x22, &resCode);

}

void RT100::initElbowYaw()
{
  unsigned char resCode, byte1, byte2;
  deferredWrite(IP1, IP1ELBOW, SPEED, 30);
  deferredWrite(IP1, IP1ELBOW, MAX_FORCE, 20);

  deferredWrite(IP1, IP1YAW, SPEED, 30);
  deferredWrite(IP1, IP1YAW, MAX_FORCE, 20);

  deferredWrite(IP1, IP1YAW, ERROR_LIMIT, 1500);

  rawCommand(IP0, EMERGENCY_STOP + DEAD_STOP, &resCode);
  rawCommand(IP1, EMERGENCY_STOP + DEAD_STOP, &resCode);

  rawCommand(IP0, (unsigned char)0x80, (unsigned char)0x00, (unsigned char)0x00, &resCode); //all off
  rawCommand(IP1, 0x80, 0x82, 0x00, &resCode); //move elbow and sholder to end point

  //check if we hit the end stop
  bool hitEndStop = false;
  for(int i=0; i<500 && !hitEndStop; i++)
  {
    rawCommand(IP1, 0x10 + IP1YAW, &resCode, &byte1, &byte2);
    if (byte1 & (1 << NO_MOVEMENT)) //check if no movment in the last time
     hitEndStop = true;
  }
  rawCommand(IP1, (unsigned char)0x80, (unsigned char)0x00, (unsigned char)0x00, &resCode); //all off

  if (!hitEndStop)
    LFATAL("ERROR: Can not init sholder");

  //define home position
  rawCommand(IP0, 0x21, &resCode);
  rawCommand(IP1, 0x21, &resCode);

  //reload pids
  rawCommand(IP0, 0x22, &resCode);
  rawCommand(IP1, 0x22, &resCode);

}

// ######################################################################
int RT100::shutdown()
{

  return RT100_OK;
}

int RT100::deferredRead(IP ipID, unsigned int ctrl, PARAM param, short int *val)
{

  unsigned char cmd[3];
  unsigned char cmdResults[3];


  switchIP(ipID);

  //first transaction, specify the ctrl to read from and the param
  cmd[0] = 0x06 << 4;
  cmd[0] += ctrl;
  cmd[1] = param;
  cmd[2] = 0xcc; //ignored

  LDEBUG("Sending T1: %x %x %x", cmd[0], cmd[1], cmd[2]);
  itsPort->write(&cmd,3);
  usleep(10000); //TODO remove
  unsigned char result = itsPort->read();
  if (result != 0xC0)
    LFATAL("Error in deferred read\n");

  //second transaction, specify the ctrl to read from and the param
  cmd[0] = 0x06 << 4;
  cmd[0] += 0x08 + ctrl;

  LDEBUG("Sending T2: %x", cmd[0]);
  itsPort->write(&cmd,1);
  usleep(10000); //TODO remove
  itsPort->read(cmdResults, 3);
  LDEBUG("Got: %x %x %x", cmdResults[0], cmdResults[1], cmdResults[2]);

  //todo check schecksum

  *val = cmdResults[2] << 8;
  *val += cmdResults[1];

  return RT100_OK;
}

int RT100::deferredWrite(IP ipID, unsigned int ctrl, PARAM param, short int val)
{

  unsigned char cmd[3];
  unsigned char cmdResults[3];
  switchIP(ipID);

  //first transaction, set the parameter to change
  cmd[0] = 0x07 << 4; //set the controller for first transaction
  cmd[0] += ctrl;
  cmd[1] = param; //the paramter to change
  cmd[2] = 0xcc; //ignored in first transaction

  LDEBUG("Sending T1: %x %x %x", cmd[0], cmd[1], cmd[2]);
  itsPort->write(&cmd,3);
  usleep(10000); //TODO remove
  unsigned char result = itsPort->read();
  if (result != 0xE0)
    LFATAL("Error in deferred Write (sent %x,%x,%x got %x)\n",
        cmd[0], cmd[1], cmd[2], result);

  //second transaction, set the parameter value
  cmd[0] = 0x07 << 4; //set the controller for second transaction
  cmd[0] += 0x08 + ctrl;
  cmd[1] = val;  //LSB
  cmd[2] = val >> 8; //MSB

  LDEBUG("Sending T2: %x %x %x", cmd[0], cmd[1], cmd[2]);
  itsPort->write(&cmd,3);
  usleep(10000); //TODO remove
  itsPort->read(cmdResults, 3);
  LDEBUG("Got: %x %x %x", cmdResults[0], cmdResults[1], cmdResults[2]);

  //TODO check checksum


  return RT100_OK;
}

int RT100::immediateRead(IP ipID, unsigned int ctrl, PARAM param, short int *val)
{
  unsigned char cmd[3];
  unsigned char cmdResults[3];

  switchIP(ipID);

  cmd[0] = 0x04 << 4;
  switch (param)
  {
    case CP_ERROR:
      cmd[0] += ctrl;
      break;
    case CURRENT_POSITION:
      cmd[0] += 0x08 + ctrl;
      break;
    default:
      LFATAL("Can not send param in immediate mode");
      break;
  }

  LDEBUG("Sending: %x", cmd[0]);
  itsPort->write(&cmd,1);
  usleep(10000); //TODO remove
  itsPort->read(cmdResults, 3);
  LDEBUG("Got: %x %x %x", cmdResults[0], cmdResults[1], cmdResults[2]);

  //todo check schecksum

  *val = cmdResults[2] << 8;
  *val += cmdResults[1];

  return RT100_OK;
}

int RT100::immediateWrite(IP ipID, unsigned int ctrl, PARAM param, short int val)
{

  unsigned char cmd[3];
  switchIP(ipID);

  if (param != CURRENT_POSITION) LFATAL("Can only write to CURRENT_POSITION");

  cmd[0] = 0x05 << 4; //set the controller for first transaction
  cmd[0] += 0x08 + ctrl;
  cmd[1] = val;  //LSB
  cmd[2] = val >> 8; //MSB

  LDEBUG("Sending: %x %x %x", cmd[0], cmd[1], cmd[2]);
  itsPort->write(&cmd,3);
  usleep(10000); //TODO remove
  itsPort->read();

  //TODO check checksum

  return RT100_OK;
}

int RT100::switchIP(IP ipID)
{
  unsigned char cmd;
  unsigned char cmdResults[3];


  //get the current IP number
  cmd = 0x01; //get IP identification
  if (itsPort->write(&cmd,1) != 1)
    LFATAL("Can not send command to arm");
  usleep(10000); //TODO remove

  if (itsPort->read(cmdResults, 1) != 1)
    LFATAL("Can not get IP id");
  usleep(10000); //TODO remove


  if ((cmdResults[0] == 0x20 && ipID == IP1) ||
      (cmdResults[0] == 0x21 && ipID == IP0) ) //if we are on a diffrent ip, switch
  {
    cmd = 0x29; //switch IPS
    if (itsPort->write(&cmd,1) != 1)
      LFATAL("Can not send command to arm");
    usleep(10000); //TODO remove
    if (itsPort->read(cmdResults, 1) != 1)
      LFATAL("Can not get results");
    if (cmdResults[0] != 0x00 && cmdResults[0] != 0x21)
      LFATAL("Can not switch IPS got: %x", cmdResults[0]);
  }


  usleep(10000);

  return RT100_OK;

}

// ######################################################################
int RT100::rawCommand(IP ipID, unsigned char cmdType,
    unsigned char *resCode, short int *results)
{

  unsigned char cmdResults[3];
  switchIP(ipID); //switch to a diffrent IP if nessesary

  LDEBUG("Sending: %x", cmdType);
  if (itsPort->write(&cmdType,1) != 1)
    LFATAL("Can not send command to arm");
  usleep(10000); //TODO remove

  cmdResults[0] = 0xFF;
  int n = itsPort->read(cmdResults, 3);
  *resCode = cmdResults[0];
  LDEBUG("Got: %x", cmdResults[0]);
  if (n > 1)
  {
    //TODO combin results
  }
  usleep(10000); //TODO remove

  return RT100_OK;
}

// ######################################################################
int RT100::rawCommand(IP ipID, unsigned char cmdType,
    unsigned char *resCode, unsigned char *byte1, unsigned char *byte2)
{

  unsigned char cmdResults[3];
  switchIP(ipID); //switch to a diffrent IP if nessesary

  LDEBUG("Sending: %x", cmdType);
  if (itsPort->write(&cmdType,1) != 1)
    LFATAL("Can not send command to arm");
  usleep(10000); //TODO remove

  cmdResults[0] = 0xFF;
  int n = itsPort->read(cmdResults, 3);
  LDEBUG("Got: %x %x %x", cmdResults[0], cmdResults[1], cmdResults[2]);

  *resCode = cmdResults[0];
  if (n > 1)
  {
    *byte1 = cmdResults[1];
    *byte2 = cmdResults[2];
  } else {
    *byte1 = 0xFF;
    *byte2 = 0xFF;
  }

  usleep(10000); //TODO remove

  return RT100_OK;
}

// ######################################################################
int RT100::rawCommand(IP ipID, unsigned char cmdType,
    unsigned char byte1, unsigned char byte2,
    unsigned char *resCode)
{
  unsigned char cmdResults;
  unsigned char cmd[3];

  switchIP(ipID); //switch to a diffrent IP if nessesary

  cmd[0] = cmdType;
  cmd[1] = byte1;
  cmd[2] = byte2;
  LDEBUG("Sending: %x %x %x", cmd[0], cmd[1], cmd[2]);
  if (itsPort->write(&cmd,3) != 3)
    LFATAL("Can not send command to arm");
  usleep(10000); //TODO remove

  cmdResults = 0xFF;
  itsPort->read(&cmdResults, 1);
  LDEBUG("Got: %x", cmdResults);
  *resCode = cmdResults;
  usleep(10000); //TODO remove

  return RT100_OK;
}



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/*!@file Devices/BeeStem3.C Simple interface to beestem */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/BeeStem3.C $
// $Id: BeeStem3.C 13312 2010-04-26 02:16:07Z beobot $
//

#include "Devices/BeeStem3.H"

#include "Component/OptionManager.H"
#include <string>

#define BS_CMD_DELAY 5000000

// ######################################################################
BeeStem3::BeeStem3(OptionManager& mgr, const std::string& descrName,
         const std::string& tagName, const char *defdev) :
  ModelComponent(mgr, descrName, tagName),
  itsPort(new Serial(mgr))
{
  // set a default config for our serial port:
  itsPort->configure(defdev, 57600, "8N1", false, false, 1);
  //  itsPort->setBlocking(true);

  // attach our port as a subcomponent:
  addSubComponent(itsPort);

  mMotorControllerState.resize(NUM_MOTOR_CONTROLLERS);

  for(int i = 0; i < mMotorControllerState.size(); i ++)
  {
	  mMotorControllerState[i] = 0;
  }

  pthread_mutex_init(&itsSerialLock, NULL);
}

// ######################################################################
BeeStem3::~BeeStem3()
{
  pthread_mutex_destroy(&itsSerialLock);
}

// ######################################################################
bool BeeStem3::getSensors(int &accelX,int &accelY,int &accelZ,
                          int &compassHeading, int &compassPitch, int &compassRoll,
                          int &internalPressure, int &externalPressure,
                          int &desiredHeading, int &desiredDepth, int &desiredSpeed,
                          int &headingK, int &headingP, int &headingD, int &headingI, int &headingOutput,
                          int &depthK, int &depthP, int &depthD, int &depthI, int &depthOutput, char &killSwitch)//,
                          //                          int &thruster1, int &thruster2,int &thruster3,
                          //int &thruster4,int &thruster5,int &thruster6)
{
  char readCmd = 0x00;
  char accel_data[3];
  unsigned char adc_data[32];
  char desired_heading[2];
  char desired_depth[2];
  char desired_speed;
  char marker_drop[2];
  char comp_accel[6];
  char comp_mag[6];
  char comp_heading[6];
  char comp_tilt[6];
  char battery[4];
  char pid[12];
  char kill_switch;
  char temp;


  //clear serial buffer
  while(itsPort->read(&temp,1));

  // send read command to Propeller
  itsPort->write(&readCmd, 1);
  usleep(40000);

  //read accelerometer
  int size = itsPort->read(&accel_data, 3);

  if(size <= 0)
    {
      LERROR("Couldn't read accel_data.");
      return false;
    }

  accelX = accel_data[0];
  accelY = accel_data[1];
  accelZ = accel_data[2];

  //read adc data
  size = itsPort->read(&adc_data, 32);

  if(size <= 0)
    {
      LERROR("Couldn't read adc_data.");
      return false;
    }

  internalPressure = adc_data[14];
  internalPressure += adc_data[15]<<8;

  externalPressure = adc_data[12];
  externalPressure += adc_data[13]<<8;

  /*  thruster1 = adc_data[16];
  thruster1 += adc_data[17]<<8;

  thruster2 = adc_data[18];
  thruster2 += adc_data[19]<<8;

  thruster3 = adc_data[20];
  thruster3 += adc_data[21]<<8;

  thruster4 = adc_data[22];
  thruster4 += adc_data[23]<<8;

  thruster5 = adc_data[24];
  thruster5 += adc_data[25]<<8;

  thruster6 = adc_data[26];
  thruster6 += adc_data[27]<<8;*/

  //read desired heading
  size = itsPort->read(&desired_heading, 2);

  if(size <= 0)
    {
      LERROR("Couldn't read desired_heading.");
      return false;
    }

  desiredHeading = desired_heading[0];
  desiredHeading += desired_heading[1]<<8;

  //read desired depth
  size = itsPort->read(&desired_depth, 2);

  if(size <= 0)
    {
      LERROR("Couldn't read desired_depth.");
      return false;
    }

  desiredDepth = (255 & desired_depth[0]);
  desiredDepth |= (255 & desired_depth[1]<<8) & 65280;

  //read desired speed
  size = itsPort->read(&desired_speed, 1);

  if(size <= 0)
    {
      LERROR("Couldn't read desired_speed.");
      return false;
    }

  desiredSpeed = desired_speed;

  //read marker droppers
  size = itsPort->read(&marker_drop, 2);

  if(size <= 0)
    {
      LERROR("Couldn't read marker_drop.");
      return false;
    }

  //read compass acceleration
  size = itsPort->read(&comp_accel, 6);

  if(size <= 0)
    {
      LERROR("Couldn't read comp_accel.");
      return false;
    }

 //read compass magnetic field
  size = itsPort->read(&comp_mag, 6);

  if(size <= 0)
    {
      LERROR("Couldn't read comp_mag.");
      return false;
    }

  //read compass heading
  size = itsPort->read(&comp_heading, 6);

  if(size <= 0)
    {
      LERROR("Couldn't read comp_heading.");
      return false;
    }


  compassHeading = (unsigned char) comp_heading[0];
  compassHeading += (unsigned char) (comp_heading[1])<<8;

  compassPitch = comp_heading[2];
  compassPitch += comp_heading[3]<<8;

  compassRoll = comp_heading[4];
  compassRoll += comp_heading[5]<<8;

  //read compass tilt
  size = itsPort->read(&comp_tilt, 6);

  if(size <= 0)
    {
      LERROR("Couldn't read comp_tilt.");
      return false;
    }

  //read battery values
  size = itsPort->read(&battery,4);

  if(size <= 0)
    {
      LERROR("Couldn't read battery.");
      return false;
    }

  //read pid values
  size = itsPort->read(&pid,12);

  if(size <= 0)
    {
      LERROR("Couldn't read pid.");
      return false;
    }

  headingK = pid[0];
  headingP = pid[1];
  headingD = pid[2];
  headingI = pid[3];

  headingOutput = (0x00ff & pid[4]);
  headingOutput |= pid[5] << 8;

  depthK = pid[6];
  depthP = pid[7];
  depthD = pid[8];
  depthI = pid[9];
  depthOutput = (0x00ff & pid[10]);
  depthOutput |= pid[11] << 8;

  //read killswitch value
  size = itsPort->read(&kill_switch,1);

  if(size <= 0)
    {
      LERROR("Couldn't read kill switch.");
      return false;
    }

  killSwitch = kill_switch;

  /*  LINFO("desired_depth[0] = %x, desired_depth[1] = %x, depthOutput= %x",
        (0x00ff & pid[10]),
        ((0x00ff & pid[11]) << 8) & 0x0ff00,
        depthOutput); */

  return true;
}

bool BeeStem3::setPID(int pidMode, float k, float p, float i, float d)
{
  LINFO("pidMode: %d, k %f, p %f, i %f, %f",pidMode,k,p,i,d);

  char pidCmdK;
  char pidCmdP;
  char pidCmdI;
  char pidCmdD;
  char pidDepthEn = 0x61;
  char pidHeadingEn = 0x60;
  char en = 0x01;
  char dis = 0x00;

  int16 kv, pv, iv, dv;
  kv = k*100;
  pv = p*100;
  iv = i*100;
  dv = d*100;

  char temp;

  //clear serial buffer
  while(itsPort->read(&temp,1));

  switch(pidMode)
    {
    case PID_DEPTH:
      pidCmdK = 0x20;
      pidCmdP = 0x21;
      pidCmdI = 0x22;
      pidCmdD = 0x23;
      break;
    case PID_HEADING:
      pidCmdK = 0x30;
      pidCmdP = 0x31;
      pidCmdI = 0x32;
      pidCmdD = 0x33;
      break;
   case PID_DISABLE:
     LINFO("Disable PID.");
      itsPort->write(&pidDepthEn, 1);
      itsPort->write(&dis, 1);
      itsPort->write(&pidHeadingEn, 1);
      itsPort->write(&dis, 1);
      return true;
      break;
   case PID_ENABLE:
     LINFO("Enable PID.");
      itsPort->write(&pidDepthEn, 1);
      itsPort->write(&en, 1);
      //itsPort->write(&pidHeadingEn, 1);
      //itsPort->write(&en, 1);
      return true;
      break;
    default:
      LERROR("Invalid PID mode specified.");
      return false;
    }


  // send set update K cmd upper lower
  itsPort->write(&pidCmdK, 1);

  temp =(kv & 0xff00) >> 8;
  itsPort->write(&temp,1);
  temp =(kv & 0x00ff);
  itsPort->write(&temp,1);

  // send set update P cmd
  itsPort->write(&pidCmdP, 1);
  temp =(pv & 0xff00) >> 8;
  itsPort->write(&temp,1);
  temp =(pv & 0x00ff);
  itsPort->write(&temp,1);

  // send set update K cmd
  itsPort->write(&pidCmdI, 1);
  temp =(iv & 0xff00) >> 8;
  itsPort->write(&temp,1);
  temp =(iv & 0x00ff);
  itsPort->write(&temp,1);

  // send set update K cmd
  itsPort->write(&pidCmdD, 1);
  temp =(dv & 0xff00) >> 8;
  itsPort->write(&temp,1);
  temp =(dv & 0x00ff);
  itsPort->write(&temp,1);

  return true;
}

void BeeStem3::setThruster(int num, int val)
{
	if(val == mMotorControllerState[num]) //don't bother setting it if it's already at this value
	{
		return;
	}
	mMotorControllerState[num] = val; //save the new state
	printf("Set thruster [%d]:%d\n", num, val);
  char thrusterCmd = 0xff;
  //char temp;

  //clear serial buffer
  //  while(itsPort->read(&temp,1));

  // send set thruster command to Propeller
  itsPort->write(&thrusterCmd, 1);

  //  while(itsPort->read(&temp, 1) != 0)
  // std::cout << temp;
  // std::cout << std::endl;

  // send set thruster command to Propeller
  itsPort->write(&num, 1);
  //while(itsPort->read(&temp, 1) != 0)
  //  printf("%c", temp);
  //std::cout << std::endl;

  // send set thruster command to Propeller
  itsPort->write(&val, 1);
}

/*bool BeeStem3::setDesiredValues(int16 heading, int16 depth, char speed,
                                char markerDropper)
{
  char setDesiredHeadingCmd = 0x0b;
  char setDesiredDepthCmd = 0x0c;
  char temp;

  //clear serial buffer
  while(itsPort->read(&temp,1));

  // send set desired values command to Propeller
  itsPort->write(&setDesiredHeadingCmd, 1);

  char headingUpper = ((0x00ff00 & heading) >> 8) & 0x00ff;
  char headingLower = (heading & 0x00ff);

  char depthUpper = ((0x00ff00 &depth) >> 8) & 0x00ff;
  char depthLower = (depth & 0x00ff);

  LINFO("Writing Heading Upper %x", headingUpper);
  itsPort->write(&headingUpper,1);
  LINFO("Writing Heading Lower %x", headingLower);
  itsPort->write(&headingLower,1);

  itsPort->write(&setDesiredDepthCmd, 1);

  itsPort->write(&depthUpper,1);
  itsPort->write(&depthLower,1);

  itsPort->write(&speed,1);
  itsPort->write(&markerDropper,1);
  itsPort->write(&markerDropper,1);

  return true;
}*/

bool BeeStem3::setDesiredHeading(int16 heading)
{
  char setDesiredHeadingCmd = 0x0b;
  char temp;

  //clear serial buffer
  while(itsPort->read(&temp,1));

  // send set desired values command to Propeller
  itsPort->write(&setDesiredHeadingCmd, 1);

  char headingUpper = ((0x00ff00 & heading) >> 8) & 0x00ff;
  char headingLower = (heading & 0x00ff);

  LINFO("Writing Heading Upper %x", headingUpper);
  itsPort->write(&headingUpper,1);
  LINFO("Writing Heading Lower %x", headingLower);
  itsPort->write(&headingLower,1);

  return true;
}

bool BeeStem3::setDesiredDepth(int16 depth)
{
  char setDesiredDepthCmd = 0x0c;
  char temp;

  //clear serial buffer
  while(itsPort->read(&temp,1));

  // send set desired values command to Propeller
  itsPort->write(&setDesiredDepthCmd, 1);

  char depthUpper = ((0x00ff00 & depth) >> 8) & 0x00ff;
  char depthLower = (depth & 0x00ff);

  LINFO("Writing Depth Upper %x", depthUpper);
  itsPort->write(&depthUpper,1);
  LINFO("Writing Depth Lower %x", depthLower);
  itsPort->write(&depthLower,1);

  return true;
}

bool BeeStem3::setDesiredSpeed(char speed)
{
  char setDesiredSpeedCmd = 0x0d;
  char temp;

  //clear serial buffer
  while(itsPort->read(&temp,1));

  // send set desired values command to Propeller
  itsPort->write(&setDesiredSpeedCmd, 1);

  LINFO("Setting speed: %d\n",speed);
  itsPort->write(&speed,1);

  return true;
}

void BeeStem3::startCompassCalibration()
{
  char startCalibCmd = 0xe0;
  char temp;

  //clear serial buffer
  while(itsPort->read(&temp,1));

  // send set desired values command to Propeller
  itsPort->write(&startCalibCmd, 1);
}

void BeeStem3::endCompassCalibration()
{
  char endCalibCmd = 0xe1;
  char temp;

  //clear serial buffer
  while(itsPort->read(&temp,1));

  // send set desired values command to Propeller
  itsPort->write(&endCalibCmd, 1);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

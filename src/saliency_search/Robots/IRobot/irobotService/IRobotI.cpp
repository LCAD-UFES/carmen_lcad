/*!@file IRobotI.cpp IRobot service implimantation  */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/IRobot/irobotService/IRobotI.cpp $
// $Id: IRobotI.cpp 10794 2009-02-08 06:21:09Z itti $
//

#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <IRobotI.h>
#include "capture.h"
#include "serial.h"


#include <IceE/IceE.h>

IRobotI::IRobotI(int debug) :
  itsCurrentSpeed(0),
  itsCurrentSteering(0),
  itsDebug(debug),
  itsVideoInit(-1)
{
  //start the serial device
  itsSerialFd = openPort("/dev/ttyS2");

//#ifdef USEMMJPEG
  printf("Using MJpeg\n");
  //Requested Camera Resolution
  int width = 320;
  int height = 240;
  //Requested framerate
  int fps = 10;
  //Video device file
  char* videodevice = "/dev/video0";

  int format = V4L2_PIX_FMT_MJPEG;
  int grabmethod = 1;
  char *avifilename = NULL;

  colorspace_init();
  //Allocate our video input structure
  itsVideoIn = (struct vdIn *) calloc(1, sizeof(struct vdIn));
  //Initialize our color lookup tables
  initLut();
  ////Initialize the video input data structure
  init_videoIn(itsVideoIn,
      (char *) videodevice,
      width, height, fps, format,
      grabmethod, avifilename);
  itsVideoInit = 1;
//#else
//  open_device ();
//  init_device (0);
//  start_capturing();
//  itsVideoInit = 1;
//#endif

  if (itsDebug)
    printf("IRobot initalized\n");
}

IRobotI::~IRobotI() {
  closePort(itsSerialFd);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float IRobotI::getSpeed(const Ice::Current&){
  return itsCurrentSpeed;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
short IRobotI::setSpeed(const float speed, const Ice::Current&){
  if (itsDebug)
    printf("Setting speed to %f\n", speed);
  itsCurrentSpeed = speed;


  sendDirectDriveCommand();

  return 0;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float IRobotI::getSteering(const Ice::Current&){
  return itsCurrentSteering;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
short IRobotI::setSteering(const float steeringPos, const Ice::Current&){
  if (itsDebug)
    printf("Setting steering to %f\n", steeringPos);
  itsCurrentSteering = steeringPos;
  sendDirectDriveCommand();
  return 0;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ImageIceMod::ImageIce IRobotI::getImageSensor(const short indx, const bool useColor, const Ice::Current&){

  //frame* f = get_frame(false);
  ImageIceMod::ImageIce imgRet;

  if (itsVideoInit == -1)
    return imgRet;

//#ifdef USEMJEPEG
  int jpeg_decode_stat;
  uvcGrab(itsVideoIn);
  imgRet.width = itsVideoIn->width;
  imgRet.height = itsVideoIn->height;

  int size = itsVideoIn->width*itsVideoIn->height;
  unsigned char frameData[size*3];
  Pyuv422torgb24(itsVideoIn->framebuffer,
      frameData,
      itsVideoIn->width,
      itsVideoIn->height);


  if (useColor)
  {
    imgRet.pixSize = 3;
    imgRet.data.resize(size*imgRet.pixSize);
    std::copy(frameData, frameData + (size*imgRet.pixSize), imgRet.data.begin());

  } else {
    imgRet.pixSize = 1;
    const unsigned char*sPtr = frameData;
    imgRet.data.resize(size*imgRet.pixSize);

    for(int i=0; i<size; i++)
    {
      imgRet.data[i] = (*sPtr + *(sPtr+1) + *(sPtr+2))/3;
      sPtr += 3;
    }

  }



//#else
//
//  frame* f = get_frame(useColor);
//
//  if (useColor)
//    imgRet.pixSize = 3;
//  else
//    imgRet.pixSize = 1;
//
//  int size = f->width*f->height*imgRet.pixSize;
//  imgRet.width = f->width;
//  imgRet.height = f->height;
//
//  imgRet.data.resize(size);
//  if (useColor)
//    std::copy(f->data, f->data + size, imgRet.data.begin());
//  else
//    std::copy(f->lumData, f->lumData + size, imgRet.data.begin());
//
//#endif
  return imgRet;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IRobotI::sendDriveCommand()
{
  unsigned char cmd[5];
  cmd[0] = 137; //drive command
  cmd[1] = ((short int)itsCurrentSpeed&0xFFFF)>>8; //velocity high byte
  cmd[2] = ((short int)itsCurrentSpeed&0xFF); //velocity low byte

  if (itsCurrentSteering == 0) //drive striaght
  {
    cmd[3] = 0x7F; //Radius high byte
    cmd[4] = 0xFF; //Radius low byte
  } else {
    cmd[3] = ((short int)itsCurrentSteering&0xFFFF)>>8; //Radius high byte
    cmd[4] = ((short int)itsCurrentSteering&0xFF); //Radius low byte
  }


  if (itsDebug)
  {
    printf("Sending: ");
    for(int i=0; i<5; i++)
      printf("%i ", cmd[i]);
    printf("\n");
  }
  sendData(itsSerialFd, cmd, 5);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IRobotI::sendDirectDriveCommand()
{

  int rightWheel = (int)((itsCurrentSpeed*500) + (itsCurrentSteering*500));
  int leftWheel = (int)((itsCurrentSpeed*500) - (itsCurrentSteering*500));

  if (rightWheel > 500) rightWheel = 500;
  if (rightWheel < -500) rightWheel = -500;

  if (leftWheel > 500) leftWheel = 500;
  if (leftWheel < -500) leftWheel = -500;

  unsigned char cmd[5];
  cmd[0] = 145; //Direct drive command
  cmd[1] = ((short int)rightWheel&0xFFFF)>>8; //Right Wheel high byte
  cmd[2] = ((short int)rightWheel&0xFF); //Right Wheel low byte

  cmd[3] = ((short int)leftWheel&0xFFFF)>>8; //Left Wheel high byte
  cmd[4] = ((short int)leftWheel&0xFF); //Left Wheel low byte


  if (itsDebug)
  {
    printf("Sending: ");
    for(int i=0; i<5; i++)
      printf("%i ", cmd[i]);
    printf("\n");
  }
  sendData(itsSerialFd, cmd, 5);


//  unsigned char data[255];
//  int len = read(itsSerialFd, data, 255);
//  printf("Got %i\n", len);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IRobotI::sendStart(const Ice::Current&)
{
  unsigned char cmd[1];
  cmd[0] = 128;
  sendData(itsSerialFd, cmd, 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IRobotI::setMode(const Robots::IRobotModes mode, const Ice::Current&)
{
  unsigned char cmd[1];
  cmd[0] = 0;

  switch(mode)
  {
    case Robots::SafeMode: cmd[0] = 131; break;
    case Robots::FullMode: cmd[0] = 132; break;
    case Robots::SpotMode: cmd[0] = 134; break;
    case Robots::CoverMode: cmd[0] = 135; break;
    case Robots::CoverAndDockMode: cmd[0] = 143; break;
  };

  sendData(itsSerialFd, cmd, 1);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IRobotI::setDemo(const short demo, const Ice::Current&)
{
  unsigned char cmd[2];
  cmd[0] = 136;
  cmd[1] = demo;

  if (itsDebug)
  {
    printf("Sending: ");
    for(int i=0; i<2; i++)
      printf("%i ", cmd[i]);
    printf("\n");
  }
  sendData(itsSerialFd, cmd, 2);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IRobotI::setLED(const short led, const short color, const short intensity, const Ice::Current&)
{
  unsigned char cmd[4];
  cmd[0] = 139;

  switch(led)
  {
    case 1: cmd[1] = 8; break; //Advance LED
    case 2: cmd[1] = 2; break; //Play Led
    case 3: cmd[1] = 10; break; //Both Led
    default: cmd[1] = 0; //default to turn off
  };

  cmd[2] = color;
  cmd[3] = intensity;

  if (itsDebug)
  {
    printf("Sending: ");
    for(int i=0; i<2; i++)
      printf("%i ", cmd[i]);
    printf("\n");
  }
  sendData(itsSerialFd, cmd, 4);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IRobotI::playSong(const short song, const Ice::Current&)
{

  if (itsDebug)
    printf("Play song %i\n", song);
  // The imperial March Song
  // G  G  G  E  Bb  G  E  Bb  G  2D  2D  2D  2Eb  Bb  F#  E  Bb  G
  // 55 55 55 52 58 55 52 58  55  62  62  62  63   58  54  51 58  55
  unsigned char s1[21] = {140, 0, 9, 55, 30, 55, 30, 55, 30, 51, 30, 58, 12, 55, 30, 51, 30, 58, 12, 55, 30};
  //unsigned char s2[21] = {140, 0, 9, 55, 30, 55, 30, 55, 30, 51, 30, 58, 12, 55, 30, 51, 30, 58, 12, 55, 30};
  unsigned char s2[21] = {140, 1, 9, 62, 30, 62, 30, 62, 30, 63, 30, 58, 12, 54, 30, 51, 30, 58, 12, 55, 30};

  unsigned char cmd[2] ;

  sendData(itsSerialFd, s1, 21);
  sendData(itsSerialFd, s2, 21);

  cmd[0] = 141; cmd[1] = song;
  sendData(itsSerialFd, cmd, 2);
  //sleep(4);
  //cmd[0] = 141; cmd[1] = 1;
  //sendData(itsSerialFd, cmd, 2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ImageIceMod::DimsIce IRobotI::getImageSensorDims(const short i, const Ice::Current&) {
  ImageIceMod::DimsIce dims;
  dims.w = -1;
  dims.h = -1;

  return dims;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float IRobotI::getSensorValue(const short i, const Ice::Current&) {

  //Query the distance and angle sensors
  unsigned char cmd[4];
  unsigned char data[255];
  int len;
  cmd[0] = 149; //Query list opcode
  cmd[1] = 1; //we need two sensors
  cmd[2] = i; //distance

  sendData(itsSerialFd, cmd, 3);

  //get the data

  len = read(itsSerialFd, data, 4);
  if (len == 2)
  {
    int16_t val = (int16_t)((data[0]<<8) | data[1]);
    return (float)val;
  } else {
    return (float)(data[0]);
  }

  return -1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool IRobotI::getDistanceAngle(float& dist, float& ang, const Ice::Current&) {

  //Query the distance and angle sensors
  unsigned char cmd[4];
  unsigned char data[255];
  int len;
  cmd[0] = 149; //Query list opcode
  cmd[1] = 2; //we need two sensors
  cmd[2] = 19; //distance
  cmd[3] = 20; //angle

  sendData(itsSerialFd, cmd, 4);

  //get the data

  len = read(itsSerialFd, data, 4);
  //printf("Read %i\n", len);
  short int rDist = 0, rAng = 0;
  //if (len == 4)
  {
   // printf("%i %i %i %i\n\n",
   //     data[0], data[1], data[2], data[3]);

    rDist = (int16_t)((data[0]<<8) | data[1]);
    rAng = (int16_t)((data[2]<<8) | data[3]);

  }

  dist = rDist;
  ang = rAng;


  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IRobotI::motorsOff(const short i, const Ice::Current&)
{
  itsCurrentSteering = 0;
  itsCurrentSpeed = 0;
  sendDirectDriveCommand();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IRobotI::setMotor(const short i, const float val, const Ice::Current&)
{

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
short IRobotI::sendRawCmd(const std::string& s, const Ice::Current&)
{
  return -1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IRobotI::shutdown(const Ice::Current& c)
{
  motorsOff(0, c);
  if (itsDebug)
    printf("Shutting down...\n");
  c.adapter->getCommunicator()->shutdown();
}


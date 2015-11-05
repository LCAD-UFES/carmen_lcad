/*!@file GumbotI.cpp Gumbot service implimantation  */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Gumbot/GumbotService/GumbotI.cpp $
// $Id: GumbotI.cpp 10794 2009-02-08 06:21:09Z itti $
//

#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <GumbotI.h>
#include "capture.h"
#include "serial.h"

#include <IceE/IceE.h>

GumbotI::GumbotI(int debug) :
  itsCurrentSpeed(0),
  itsCurrentSteering(0),
  itsDebug(debug)
{
  //start the serial device
  itsSerialFd = openPort("/dev/ttyS2");

  colorspace_init();
  open_device ();
  init_device (0);
  start_capturing();

  if (itsDebug)
    printf("Gumbot initalized\n");
}

GumbotI::~GumbotI() {
  closePort(itsSerialFd);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float GumbotI::getSpeed(const Ice::Current&){
  return itsCurrentSpeed;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
short GumbotI::setSpeed(const float speed, const Ice::Current&){
  if (itsDebug)
    printf("Setting speed to %f\n", speed);
  itsCurrentSpeed = speed;
  sendDriveCommand();

  return 0;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float GumbotI::getSteering(const Ice::Current&){
  return itsCurrentSteering;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
short GumbotI::setSteering(const float steeringPos, const Ice::Current&){
  if (itsDebug)
    printf("Setting steering to %f\n", steeringPos);
  itsCurrentSteering = steeringPos;
  sendDriveCommand();
  return 0;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ImageIceMod::ImageIce GumbotI::getImageSensor(const short i, const Ice::Current&){
  frame* f = get_frame();
  int size = f->width*f->height*3;

  ImageIceMod::ImageIce imgRet;
  imgRet.width = f->width;
  imgRet.height = f->height;
  imgRet.pixSize = 3;

  imgRet.data.resize(size);
  std::copy(f->data, f->data + size, imgRet.data.begin());

  return imgRet;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GumbotI::sendDriveCommand()
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
void GumbotI::sendStart(const Ice::Current&)
{
  unsigned char cmd[1];
  cmd[0] = 128;
  sendData(itsSerialFd, cmd, 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GumbotI::setMode(const Robots::GumbotModes mode, const Ice::Current&)
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
void GumbotI::setDemo(const short demo, const Ice::Current&)
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
void GumbotI::setLED(const short led, const short color, const short intensity, const Ice::Current&)
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
void GumbotI::playSong(const short song, const Ice::Current&)
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
ImageIceMod::DimsIce GumbotI::getImageSensorDims(const short i, const Ice::Current&) {
  ImageIceMod::DimsIce dims;
  dims.w = -1;
  dims.h = -1;

  return dims;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
short GumbotI::getSensorValue(const short i, const Ice::Current&) {
  return -1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GumbotI::motorsOff(const short i, const Ice::Current&)
{

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GumbotI::setMotor(const short i, const float val, const Ice::Current&)
{

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
short GumbotI::sendRawCmd(const std::string& s, const Ice::Current&)
{
  return -1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GumbotI::shutdown(const Ice::Current& c)
{
  if (itsDebug)
    printf("Shutting down...\n");
  c.adapter->getCommunicator()->shutdown();
}


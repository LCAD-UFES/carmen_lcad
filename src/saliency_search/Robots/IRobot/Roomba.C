/*!@file Roomba.C Simple roomba class to interact with irobot create  */

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
// Primary maintainer for this file: Farhan Baluch <fbaluch@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/IRobot/Roomba.C $
// $Id: Roomba.C 12962 2010-03-06 02:13:53Z irock $
//


#include "Robots/IRobot/Roomba.H"
#include "Devices/Serial.H"
#include "Component/OptionManager.H"
#include "Util/Timer.H"
#include <vector>


#define SERIAL_TIMEOUT 1.0 //The maximum number of seconds to wait for a serial response

// ######################################################################
Roomba::Roomba(OptionManager& mgr, const std::string& descrName,
                     const std::string& tagName, const char *defdev) :
  ModelComponent(mgr, descrName, tagName),itsPort(new Serial(mgr,"roomba-serial","roomba-serial"))
{

  // set a default config for our serial port:
  itsPort->configure(defdev, 57600, "8N1", false, false, 1);
  // attach our port as a subcomponent:
  addSubComponent(itsPort);
  itsSpeed = 0;
  itsRadius =0;
  itsAngle=0;
  itsDist = 0;
}


// ######################################################################
Roomba::~Roomba()
{}


// ######################################################################
int Roomba::getSpeed()
{
    return itsSpeed;
}

// ######################################################################
int Roomba::getRadius()
{
    return itsRadius;
}


// ######################################################################
int Roomba::getAngle()
{
    return itsAngle;
}

// ######################################################################
int Roomba::getDist()
{
    return itsDist;
}

// ######################################################################
void Roomba::setSpeed(const int speed)
{
    if(!robotStarted)
        LFATAL("You must send start command first");

    itsSpeed = speed;
    sendDriveCommand();
}

// ######################################################################
void Roomba::setRadius(const int radius)
{
    if(!robotStarted)
        LFATAL("You must send start command first");

    itsRadius = radius;
    sendDriveCommand();
}

// ######################################################################
void Roomba::sendDriveCommand()
{
    if(!robotStarted)
        LFATAL("You must send start command first");

    unsigned char cmd[5];
    cmd[0] = 137; //drive command
    cmd[1] = ((short int)itsSpeed&0x00FF00)>>8; //velocity high byte
    cmd[2] = ((short int)itsSpeed&0x0000FF); //velocity low byte

    if (itsRadius == 0) //drive striaght
    {
        cmd[3] = 0x7F; //Radius high byte
        cmd[4] = 0xFF; //Radius low byte
    }
    else {
        cmd[3] = ((short int)itsRadius&0x00FF00)>>8; //Radius high byte
        cmd[4] = ((short int)itsRadius&0x0000FF); //Radius low byte
  }


    itsPort->write(cmd, 5);
    usleep(15*1000);

}

// ######################################################################
void Roomba::sendStart()
{
     unsigned char cmd[1];
     cmd[0] = 128;
     itsPort->write(cmd,1);
     usleep(15*1000);
     robotStarted = 1;
}
// ######################################################################
void Roomba::setMode(const int mode)
{
    unsigned char cmd[1];
    cmd[0] = 0;

    switch(mode)
    {
        case 1: cmd[0] = 131; break;     //safemode
        case 2: cmd[0] = 132; break;     //fullmode
    default: LFATAL("bad mode bye");
    };

    itsPort->write(cmd, 1);
    usleep(15*1000);
}

// ######################################################################
void Roomba::setDemo(const short demo)
{
    unsigned char cmd[2];
    cmd[0] = 136;
    cmd[1] = demo;

    itsPort->write(cmd,1);

}

// ######################################################################
void Roomba::setLED(const short led, const short color, const short intensity)
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

    itsPort->write(cmd,4);

}

// ######################################################################
void Roomba::playSong(int song)
{
     // The imperial March Song
  // G  G  G  E  Bb  G  E  Bb  G  2D  2D  2D  2Eb  Bb  F#  E  Bb  G
  // 55 55 55 52 58 55 52 58  55  62  62  62  63   58  54  51 58  55
  unsigned char s1[21] = {140, 0, 9, 55, 30, 55, 30, 55, 30, 51, 30, 58, 12, 55, 30, 51, 30, 58, 12, 55, 30};
  //unsigned char s2[21] = {140, 0, 9, 55, 30, 55, 30, 55, 30, 51, 30, 58, 12, 55, 30, 51, 30, 58, 12, 55, 30};
  unsigned char s2[21] = {140, 1, 9, 62, 30, 62, 30, 62, 30, 63, 30, 58, 12, 54, 30, 51, 30, 58, 12, 55, 30};

  unsigned char cmd[2] ;

  itsPort->write(s1,21);
  itsPort->write(s2,21);

  cmd[0] = 141;
  cmd[1] = (short)song;

  itsPort->write(cmd,2);
}


// ######################################################################
void Roomba::getDistanceAngle(int& dist,int& ang)
{

  unsigned char buff[255];
  int len = itsPort->read(buff,255);
  printf("%d bytes cleared\n",len);

  //Query the distance and angle sensors
  unsigned char cmd[4];
  unsigned char data[255];
  cmd[0] = 149; //Query list opcode
  cmd[1] = 2; //we need two sensors
  cmd[2] = 19; //distance
  cmd[3] = 20; //angle

  itsPort->write(cmd,4);

  usleep(10*1000);

  //get the data
  len = itsPort->read(data, 4);
        std::vector<byte> data_rcvd;
        for(int i=0; i<len; i++) data_rcvd.push_back(data[i]);

        Timer serialTimeout;
        serialTimeout.reset();
        while(data_rcvd.size() < 4 && serialTimeout.getSecs() < SERIAL_TIMEOUT)
        {
                len = itsPort->read(data, 4);
                for(int i=0; i<len; i++) data_rcvd.push_back(data[i]);
        }
        double timeToRcv = serialTimeout.getSecs();



  printf("Read dist angle bytes %i\n", data_rcvd.size());
  short int rDist = 0, rAng = 0;
  if (data_rcvd.size() == 4)
  {
          //printf("%i %i %i %i\n\n",
          //data[0], data[1], data[2], data[3]);

    rDist = (int16_t)((data_rcvd[0]<<8) | data_rcvd[1]);
    rAng = (int16_t)((data_rcvd[2]<<8) | data_rcvd[3]);

  }
  else
      LERROR("Read return = %d bytes -- took %fs", data_rcvd.size(), timeToRcv);

  LINFO("Read Took %fs", timeToRcv);
  dist= rDist;
  ang = rAng;
  itsDist = dist;
  itsAngle = ang;
}

// ######################################################################
void Roomba::sendRawCmd(const std::string& data)
{
}

// ######################################################################
void Roomba::sendDirectDriveCommand(float itsCurrentSteering)
{

    int rightWheel = (int)((itsSpeed*500) + (itsCurrentSteering*500));
    int leftWheel = (int)((itsSpeed*500) - (itsCurrentSteering*500));

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

    itsPort->write(cmd,5);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */















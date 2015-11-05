/*!@file BeoBorg.C  Main class to control BeoBorg based neural data from locusts   */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/BeoBorg/BeoBorg.C $
// $Id: BeoBorg.C   irock $
//


#include "Robots/BeoBorg/BeoBorg.H"
#include "Devices/Serial.H"
#include "Component/OptionManager.H"
#include "Util/Timer.H"
#include <vector>


#define SERIAL_TIMEOUT 1.0 //The maximum number of seconds to wait for a serial response

// ######################################################################
BeoBorg::BeoBorg(OptionManager& mgr, const std::string& descrName,
                     const std::string& tagName, const char *defdev) :
  ModelComponent(mgr, descrName, tagName),
  itsPort(new Serial(mgr,"beoborg-serial","beoborg-serial"))
{
  // set a default config for our serial port:
  itsPort->configure(defdev, 115200, "8N1", false, false, 1);
  // attach our port as a subcomponent:
  addSubComponent(itsPort);
  itsSpeed = 0;
  itsRadius =0;
  itsDist = 0;
  itsLeftEncoder = 0;
  itsRightEncoder = 0;
  itsLeftPWM = 0;
  itsRightPWM = 0;
}


// ######################################################################
BeoBorg::~BeoBorg()
{}


// ######################################################################
int BeoBorg::getSpeed()
{
    return itsSpeed;
}

// ######################################################################
int BeoBorg::getRadius()
{
    return itsRadius;
}


// ######################################################################
int BeoBorg::getDist()
{
    return itsDist;
}

// ######################################################################
int BeoBorg::getLeftEncoder()
{
    unsigned char cmd[3];
    cmd[0] = 100;
    itsPort->write(cmd,1);
    
    unsigned char mark = {255};
    std::vector<unsigned char> frame = itsPort ->readFrame(mark,mark,4,1);
    itsLeftEncoder =0;
    if(frame.size()!=4)
        LERROR("busted frame on left encoder skipping");
    else
      {
        itsLeftEncoder = ((0x0FF & frame[0])  << 24) |
            ((0x0FF & frame[1])  << 16) |
            ((0x0FF & frame[2])  << 8)  |
                  ((0x0FF & frame[3])  << 0);
      }
   
    return itsLeftEncoder;
}
// ######################################################################
int BeoBorg::getRightEncoder()
{
  unsigned char cmd[3];
  cmd[0] = 101;
  itsPort->write(cmd,1);
 
  unsigned char mark = {255};
  std::vector<unsigned char> frame = itsPort ->readFrame(mark,mark,4,1);
  LINFO("got %d bytes",frame.size());
   if(frame.size()!=4)
        LERROR("busted frame on right encoder skipping");
    else
      {
  itsRightEncoder = ((0x0FF & frame[0])  << 24) |
            ((0x0FF & frame[1])  << 16) |
            ((0x0FF & frame[2])  << 8)  |
            ((0x0FF & frame[3])  << 0);
      }
    return itsRightEncoder;
}

// ######################################################################
void BeoBorg::setSpeed(const int speed)
{
    itsSpeed = speed;
}

// ######################################################################
void BeoBorg::setRadius(const int radius)
{
    itsRadius = radius;
}

// ######################################################################
void BeoBorg::calibrateESC()
{
  unsigned char cmd[5];
  //create esc calibrate code
  //send esc calibrate to port;
  itsPort->write(cmd,5);
  
}

// ######################################################################
void BeoBorg::setMode(const int mode)
{
    itsMode = mode;
}

// ######################################################################
void BeoBorg::setLeftMotor(const int PWM)
{
    itsLeftPWM = PWM;
    unsigned char cmd[2];
    cmd[0] = 106;
    cmd[1] = PWM;
    itsPort->write(cmd,2);
}

// ######################################################################
void BeoBorg::setRightMotor(const int PWM)
{
    itsRightPWM = PWM;
    unsigned char cmd[2];
    cmd[0] = 107;
    cmd[1] = PWM;
    itsPort->write(cmd,2);

    //receive the pwm value after setting to confirm its set
    unsigned char cmd2[1];
    cmd2[0] = 109;
    itsPort->write(cmd2,1);
    
    int itsRightPWMc;
    unsigned char mark = {255};
    std::vector<unsigned char> frame = itsPort ->readFrame(mark,mark,4,1);
    LINFO("got %d bytes",frame.size());
    if(frame.size()!=4)
      LERROR("busted frame on getting right PWM skipping");
    else
      {
        itsRightPWMc = ((0x0FF & frame[0])  << 24) |
          ((0x0FF & frame[1])  << 16) |
          ((0x0FF & frame[2])  << 8)  |
          ((0x0FF & frame[3])  << 0);
      }
    
    LINFO("from controller i got %d",itsRightPWMc);
}


// ######################################################################
int BeoBorg::getLeftPWM()
{

      //receive the pwm value after setting to confirm its set
    unsigned char cmd2[1];
    cmd2[0] = 108;
    itsPort->write(cmd2,1);
    
    int itsLeftPWMc;
    unsigned char mark = {255};
    std::vector<unsigned char> frame = itsPort ->readFrame(mark,mark,4,1);
    LINFO("got %d bytes",frame.size());
    if(frame.size()!=4)
      LERROR("busted frame on getting left PWM skipping");
    else
      {
        itsLeftPWMc = ((0x0FF & frame[0])  << 24) |
          ((0x0FF & frame[1])  << 16) |
          ((0x0FF & frame[2])  << 8)  |
          ((0x0FF & frame[3])  << 0);
      }
    
    return itsLeftPWMc;

  
}


// ######################################################################
int BeoBorg::getRightPWM()
{

      //receive the pwm value after setting to confirm its set
    unsigned char cmd2[1];
    cmd2[0] = 109;
    itsPort->write(cmd2,1);
    
    int itsRightPWMc;
    unsigned char mark = {255};
    std::vector<unsigned char> frame = itsPort ->readFrame(mark,mark,4,1);
    LINFO("got %d bytes",frame.size());
    if(frame.size()!=4)
      LERROR("busted frame on getting left PWM skipping");
    else
      {
        itsRightPWMc = ((0x0FF & frame[0])  << 24) |
          ((0x0FF & frame[1])  << 16) |
          ((0x0FF & frame[2])  << 8)  |
          ((0x0FF & frame[3])  << 0);
      }
    
    return itsRightPWMc;

  
}

// ######################################################################
void BeoBorg::sendDriveCommand()
{
 
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
void BeoBorg::sendRaw(const int raw)
{
  LINFO("sending raw shit");
  unsigned char craw[1];
  craw[0] = raw;
  itsPort->write(craw,1);
}


// ######################################################################
void BeoBorg::printRaw()
{
  unsigned char buff[15];
  
  int ret = itsPort->read(buff,15);
  LINFO("print %d shit",ret);
  for (int i=0; i<ret;i++)
    LINFO("[%3d]",buff[i]);

  for (int i=0; i<ret;i++)
    LINFO("[%3c]",buff[i]);

  
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */















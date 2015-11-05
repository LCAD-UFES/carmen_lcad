/*!@file Robots/NavBot/NavBot.C Interface to navbot robot */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/NavBot/NavBot.C $
// $Id: NavBot.C 10794 2009-02-08 06:21:09Z itti $
//

#include "Robots/NavBot/NavBot.H"
#include "Component/OptionManager.H"

// ######################################################################
NavBot::NavBot(OptionManager& mgr, const std::string& descrName,
                   const std::string& tagName,
                   const char *defdev) :
  ModelComponent(mgr, descrName, tagName),
  itsSerial(new Serial(mgr))

{
  itsSerial->configure (defdev, 115200, "8N1", false, false, 0);
  addSubComponent(itsSerial);
}

NavBot::~NavBot()
{

}

void NavBot::start2()
{
}

bool NavBot::setMotor(MOTOR m, int pwm)
{
  unsigned char cmd[6];
  unsigned char buf[2];

        if (pwm > 63) pwm = 63;
        if (pwm < -63) pwm = -63;

        cmd[0] = 255; //Start header
        cmd[1] = 10; //move motor

  if (m == 0 )
    cmd[2] = 64 + pwm;
  else if (m == 1)
    cmd[2] = 192 + pwm;

  itsSerial->write(cmd, 3);

  //check that the command worked
  usleep(10000);
  buf[0] = 0;
  int i = itsSerial->read(buf, 1);
  if (i < 1 && buf[0] != 128)
    return false;
  return true;

}

bool NavBot::stopAllMotors()
{
  unsigned char cmd[2];
  unsigned char buf[2];
  cmd[0] = 255; //stop all motors
  cmd[1] = 11; //stop all motors
  itsSerial->write(cmd, 2);

  //check that the command worked
  usleep(10000);
  buf[0] = 0;
  int i = itsSerial->read(buf, 1);
  if (i < 1 && buf[0] != 128)
    return false;
  return true;
}

float NavBot::getBatteryVoltage()
{
  unsigned char cmd[2];
  unsigned char buf[2];


  cmd[0] = 255; //start header
  cmd[1] = 20; //get batttery voltage

  itsSerial->write(cmd, 2);
  usleep(10000);
  buf[0] = 0; buf[1] = 0;
  int i = itsSerial->read(buf, 2);
  if (i < 2 && buf[1] != 128)
    return -1;

  return (float)buf[0]/10;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

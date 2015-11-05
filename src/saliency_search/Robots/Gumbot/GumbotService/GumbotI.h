/*!@file GumbotI.h Gumbot service implimantation  */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Gumbot/GumbotService/GumbotI.h $
// $Id: GumbotI.h 12281 2009-12-17 09:00:36Z itti $
//

#ifndef Gumbot_I_H
#define Gumbot_I_H

#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>

#include <Gumbot.ice.H>

//The cmap object implimentation
class GumbotI : public Robots::Gumbot
{
  public:
    GumbotI(int debug = 1);
    virtual ~GumbotI();

    virtual float getSpeed(const Ice::Current&);
    virtual short setSpeed(const float speed, const Ice::Current&);
    virtual float getSteering(const Ice::Current&);
    virtual short setSteering(const float steeringPos, const Ice::Current&);
    virtual ImageIceMod::ImageIce getImageSensor(const short i, const Ice::Current&);
    virtual ImageIceMod::DimsIce getImageSensorDims(const short i, const Ice::Current&);
    virtual short getSensorValue(const short i, const Ice::Current&);
    virtual void motorsOff(const short i, const Ice::Current&);
    virtual void setMotor(const short i, const float val, const Ice::Current&);
    virtual short sendRawCmd(const std::string& data, const Ice::Current&);
    virtual void playSong(const short song, const Ice::Current&);
    virtual void shutdown(const Ice::Current&);
    virtual void sendStart(const Ice::Current&);
    virtual void setMode(const Robots::GumbotModes demo, const Ice::Current&);
    virtual void setDemo(const short demo, const Ice::Current&);
    virtual void setLED(const short led, const short color, const short intensity, const Ice::Current&);

    //!Send the command to drive the robot
    void sendDriveCommand();

  private:
    float itsCurrentSpeed;
    float itsCurrentSteering;
    int itsSerialFd;
    int itsDebug;

};

#endif

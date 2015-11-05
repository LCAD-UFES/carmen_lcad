/*!@file Robots/LoBot/LoRoomba.C Test collision avoidance using ping sonar on
roomba */

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
// Primary maintainer for this file:  Farhan Baluch <fbaluch@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/LoRoomba.C $
// $Id: LoRoomba.C 12962 2010-03-06 02:13:53Z irock $
//

// model manager
#include "Component/ModelManager.H"
#include "Util/log.H"
#include "rutz/shared_ptr.h"

// for images and display
#include "Raster/Raster.H"
#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "Image/PixelsTypes.H"
#include "Transport/FrameIstream.H"
#include "Transport/FrameInfo.H"
#include "GUI/XWinManaged.H"
#include "GUI/XWindow.H"
#include "Media/FrameSeries.H"

// Frame grabber
#include "Devices/FrameGrabberConfigurator.H"
#include "Devices/DeviceOpts.H"
#include "Media/FrameSeries.H"

// Devices
#include "Devices/PingSonar.H"

// for color segmentation
#include "Util/Timer.H"
#include "Util/Types.H"
#include "Util/log.H"
#include <cstdio>
#include <cstdlib>
#include <signal.h>

// for image manipulation
#include "Image/CutPaste.H"     // for inplacePaste()
#include "Image/vec2.h"
#include "Image/DrawOps.H"

// number of frames over which framerate info is averaged:
#define NAVG 20

#if HAVE_LIBSERIAL && HAVE_LIBIROBOT_CREATE

static bool goforever = true;

// for Robot controller
#include <SerialStream.h>
#include <irobot-create.hh>

using namespace iRobot;
using namespace LibSerial;
using namespace geom;

#define MAXVEL 100
#define INPLACEVEL 100

typedef struct robotCmd
{
    int motor;
    int radius;

} robotCmd;

// ######################################################################
void terminate(int s)
{ LERROR("*** INTERRUPT ***"); goforever = false; exit(1); }


// ######################################################################
//! Visualize distance received from sensor

Image<PixRGB<byte> > vizDist(std::vector<int> dists,int divisions)
{

    Image<PixRGB<byte> > img(800,800,ZEROS);
    int startAng = 0;
    int increment = 180/dists.size();
    int beginAng=startAng, endAng=increment;

    for(uint s=0;s<dists.size();s++)
    {

        for(int i=1; i<=divisions;i++)
            for (int ang=beginAng;ang<=endAng;ang++)
            {
                int rad = i*5;
                Point2D<int> pt;
                pt.i = 200+100*s - (int) (rad*cos(ang*M_PI/180.0));
                pt.j = 400 - (int) (rad*sin(ang*M_PI/180.0));

                if(dists.at(s) <= i*250)
                    drawPoint(img,pt.i,pt.j,PixRGB<byte>(255,0,0));
                else
                    drawPoint(img,pt.i,pt.j,PixRGB<byte>(0,0,255));

                writeText(img,Point2D<int>(10+100*s,10),
                          sformat("%i--%d ",s,dists.at(s)).c_str(),
                          PixRGB<byte>(255),PixRGB<byte>(0));

            }
        beginAng = endAng;
        endAng = endAng + increment;
    }

     return img;

}



// ######################################################################
//! Convert vectors to motor command
robotCmd vec2motor(geom::vec2f a)
{
    //divide our space into 4 quadrants and deal with each one separately
    //right now just concerned with angle
    //---------//
    //| 1 | 2 |//
    //|-------|//
    //| 3 | 4 |//
    //---------//

    float ang = a.theta_deg();
    int tmpRad;
    robotCmd cmd1;

    cmd1.radius = 0;
    cmd1.motor = 0;


    if(ang == 90.0)
      {
        cmd1.radius = Create::DRIVE_STRAIGHT;
        cmd1.motor = MAXVEL;
      }
    //turn clockwise if in quadrant 2
    if(ang > 90 && ang <=180)
      {
        LINFO("quadrant--2");
        tmpRad=(int)((ang-181.0) * 2000/90.0);

        if(tmpRad >= 0) //i.e. angle is 0 or less we need to turn away
        {
            cmd1.radius = Create::DRIVE_INPLACE_COUNTERCLOCKWISE;
            cmd1.motor = INPLACEVEL;
        }

        else
        {
            cmd1.radius = tmpRad;
            cmd1.motor = (int)(MAXVEL/2000)*fabs(tmpRad); //speed propotional to radius
            cmd1.motor = 100;
        }

    }


    //turn anti-clockwise if in quadrant 1
    if(ang >= 0 && ang <90)
      {
        LINFO("quadrant--1");
        tmpRad=2000-(int)((89.0-ang) * 2000/90.0);

        if(tmpRad > 2000)  //i.e. we are at 90deg no need to turn
        {
            cmd1.radius = 0.0;
            cmd1.motor = MAXVEL;
        }

        if(tmpRad <= 0)  //i.e. we are at horizontal ang =179
        {
            cmd1.radius = Create::DRIVE_INPLACE_COUNTERCLOCKWISE;
            cmd1.motor = 100;
        }


        else
        {
            cmd1.radius = tmpRad;
            cmd1.motor = (int)(MAXVEL/2000)*fabs(tmpRad); //speed propotional to radius
            cmd1.motor = 100;
        }

    }

    //turn anti-clockwise in quadrant 3
    if(ang < 0 && ang >= -90)
    {
      LINFO("quadrant--3");
            cmd1.radius = Create::DRIVE_INPLACE_COUNTERCLOCKWISE;
            cmd1.motor = INPLACEVEL;
    }

    //turn clockwise if in quadrant 4
    if(ang < -90 && ang >= -180)
    {
      LINFO("quadrant--4 , clockwise val = %d",Create::DRIVE_INPLACE_CLOCKWISE);
            cmd1.radius = Create::DRIVE_INPLACE_CLOCKWISE;
            cmd1.motor = INPLACEVEL;
    }

    return cmd1;

}


// ######################################################################
//! Receive signals from master node and performs requested actions
int main(const int argc, const char **argv)
{


  MYLOGVERB = LOG_INFO;

  // instantiate a model manager
  ModelManager manager( "Avoid Collisions " );

      /*nub::ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);

  manager.setOptionValString(&OPT_FrameGrabberMode, "RGB24");
  manager.setOptionValString(&OPT_FrameGrabberDims, "320x240");
  manager.setOptionValString(&OPT_FrameGrabberFPS, "30");
      */
  // nub::soft_ref<PingSonar> pingSonar(new PingSonar(manager,"PingSonar",                                           "PingSonar","/dev/ttyUSB0",3));

   nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
   manager.addSubComponent(ofs);
   // manager.addSubComponent(pingSonar);
   manager.exportOptions(MC_RECURSE);

  // parse command-line
  if( manager.parseCommandLine( argc, argv, "", 0, 0 ) == false ) return(1);

  manager.start();

  // initialize the motor controller
    SerialStream stream ("/dev/rfcomm5");
    //SerialStream stream ("/dev/ttyUSB0");

  // catch signals and redirect them to terminate for clean exit:
  signal(SIGHUP, terminate); signal(SIGINT, terminate);
  signal(SIGQUIT, terminate); signal(SIGTERM, terminate);
  signal(SIGALRM, terminate);

  // instantiate a robot controller
  Create robot(stream);

  robot.sendBaudCommand(Create::BAUD_57600);
  // Swith to full mode.
  robot.sendFullCommand();

  // Let's stream some sensors.
  Create::sensorPackets_t sensors;
   sensors.push_back(Create::SENSOR_ANGLE);
  sensors.push_back(Create::SENSOR_DISTANCE);

  robot.sendStreamCommand(sensors);

  robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
  //int ledColor = Create::LED_COLOR_GREEN;
  robot.sendLedCommand(Create::LED_PLAY, 0, 0);
  usleep(20*1000);

  // timer initialization
  Timer tim;
  tim.reset();
  //std::vector<int> dists = pingSonar->getDists();

  Image<PixRGB<byte> > dispImg(500,500,ZEROS);
  Point2D<int> startPt(dispImg.getWidth()/2,dispImg.getHeight()-50), endPt;
  static int dist=0;
  static int ang=0;

  // get ready for main loop:
  while (!robot.playButton () && goforever)
    {
      //    std::vector<int> dists = pingSonar->getDists();

      // drawPoint(dispImg,endPt.i,endPt.j,PixRGB<byte>(255,0,0));

      // ofs->writeRGB(vizDist(dists,12),"Output",FrameInfo("output",SRC_POS));
      ofs->writeRGB(dispImg,"Output",FrameInfo("output",SRC_POS));
      geom::vec2f a;

      float timeInt =5.0F;
      if(tim.getSecs() < timeInt)
        {
          a.set_polar_rad(25.0F,geom::deg2rad(90.0F));
          LINFO("driving 90deg -- vec %f",a.theta_deg());
        }
      else if(tim.getSecs() > timeInt && tim.getSecs() <= 2*timeInt)
        {
          a.set_polar_rad(25.0F,geom::deg2rad(135.0F));
          LINFO("driving 135deg -- vec %f",a.theta_deg());
        }
      else if(tim.getSecs() > 2*timeInt && tim.getSecs() <= 3*timeInt)
        {
          a.set_polar_rad(25.0F,geom::deg2rad(45.0F));
          LINFO("driving 45deg -- vec %f",a.theta_deg());
        }
      else if(tim.getSecs() > 3*timeInt && tim.getSecs() < 4*timeInt)
        {
          LINFO("driving 190deg");
          a.set_polar_rad(25.0F,geom::deg2rad(190.0F));
          LINFO("driving 190deg -- vec %f",a.theta_deg());
        }

      robotCmd cmd = vec2motor(a);

      LINFO("motor %d,radius %d", cmd.motor,cmd.radius);
      int crntDist =robot.distance() ; int crntAng=robot.angle();
      dist += crntDist;
      ang += crntAng;
      LINFO("dist travelled %d, angle turned %d",dist,ang);
      try
        {
          if(cmd.radius == 0 || cmd.radius ==Create::DRIVE_STRAIGHT)
            {
              robot.sendDriveCommand (cmd.motor, Create::DRIVE_STRAIGHT);
              LINFO("the straight command has val %d", Create::DRIVE_STRAIGHT);
            }
          else if (cmd.radius == 1 || cmd.radius ==Create::DRIVE_INPLACE_CLOCKWISE)
            {
              robot.sendDriveCommand (cmd.motor, Create::DRIVE_INPLACE_CLOCKWISE);
              LINFO("the clockwise command has val %d", Create::DRIVE_INPLACE_CLOCKWISE);
            }
           else if (cmd.radius == 2 || cmd.radius ==Create::DRIVE_INPLACE_COUNTERCLOCKWISE)
            {
              robot.sendDriveCommand (cmd.motor, Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
              LINFO("the counter clockwise command has val %d", Create::DRIVE_INPLACE_COUNTERCLOCKWISE);
            }

          else
            robot.sendDriveCommand (cmd.motor, cmd.radius);

        }
        catch (...) {}

      // process robot input
      if (robot.bumpLeft () || robot.bumpRight ())
        {
          std::cout << "Bump !" << std::endl;
          robot.sendPlaySongCommand(2);
          robot.sendDriveCommand(50,Create::DRIVE_INPLACE_CLOCKWISE);
        }


      if (robot.advanceButton ())
        {

        }

      if(tim.getSecs() > 5*timeInt)
            goforever=false;

      endPt.i = dispImg.getWidth()/2 - dist*sin(geom::deg2rad(ang));
      endPt.j = (dispImg.getHeight() - 10) - dist*cos(geom::deg2rad(ang));


      // You can add more commands here.
      usleep(50 * 1000);

    }

  robot.sendDriveCommand (0, Create::DRIVE_STRAIGHT);
  // Swith to full mode.
  robot.sendFullCommand();
  stream.Close();
  manager.stop();

}

#else

int main(const int argc, const char **argv)
{
  //LINFO("I cannot work without LibSerial or libirobot-create");
  return 1;
}

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/*!@file Robots/LoBot/LoRoombaVecHist.C Test collision avoidance using ping sonar on roomba */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/LoRoombaVecHist.C $
// $Id: LoRoombaVecHist.C 12962 2010-03-06 02:13:53Z irock $
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
#include "Util/sformat.H"
#include <cstdio>
#include <cstdlib>
#include <signal.h>

// for image manipulation
#include "Image/CutPaste.H"     // for inplacePaste()
#include "Image/vec2.h"
#include "Image/DrawOps.H"

//for vector fields
#include "Robots/LoBot/control/VectorHistField.H"

#include "Robots/IRobot/Roomba.H"

// number of frames over which framerate info is averaged:
#define NAVG 20

static bool goforever = true;

using namespace geom;

#define MAXVEL 100
#define INPLACEVEL 50

typedef struct robotCmd
{
    int motor;
    int radius;

} robotCmd;

// ######################################################################
void terminate(int s)
{ LERROR("*** INTERRUPT ***"); goforever = false; }


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

  float ang = (int)a.theta_deg();
  LINFO("ang %f",ang);
  int tmpRad;
  robotCmd cmd1;

  cmd1.radius = 0;
  cmd1.motor = 0;


  if(ang == 90.0)
    {
      cmd1.radius = 0;
      cmd1.motor = MAXVEL;
    }
  //turn clockwise if in quadrant 2
  if(ang > 90 && ang <=180)
    {
      LINFO("quadrant--2");
      tmpRad=(int)((ang-181.0) * 2000/90.0);

      if(tmpRad >= 0) //i.e. angle is 0 or less we need to turn away
        {
          cmd1.radius = 23;
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
      LINFO("quadrant--1--ang %f",ang);
      tmpRad = 2000-(int)((89.0-ang) * 2000/90.0);

      if(tmpRad > 2000)  //i.e. we are at 90deg no need to turn
        {
          cmd1.radius = 0.0;
          cmd1.motor = MAXVEL;
        }

      if(tmpRad <= 0)  //i.e. we are at horizontal ang =179
        {
          cmd1.radius = 23;
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
      cmd1.radius = 250;
      cmd1.motor = INPLACEVEL;
    }

  //turn clockwise if in quadrant 4
  if(ang < -90 && ang >= -180)
    {
      LINFO("quadrant--4 , clockwise");
      cmd1.radius = -250;
      cmd1.motor = INPLACEVEL;
    }

  if (cmd1.radius > 2000)
    cmd1.radius =2000;
  if (cmd1.radius < -2000)
    cmd1.radius =-2000;

  return cmd1;
}


// ######################################################################
//! Receive signals from master node and performs requested actions
int main(const int argc, const char **argv)
{


  MYLOGVERB = LOG_INFO;
  // instantiate a model manager
  ModelManager manager( "Avoid Collisions " );

  int itsDim = 30;
  int itsSpacing = 15;

  //nub::ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  //manager.addSubComponent(ifs);

  manager.setOptionValString(&OPT_FrameGrabberMode, "RGB24");
  manager.setOptionValString(&OPT_FrameGrabberDims, "320x240");
  manager.setOptionValString(&OPT_FrameGrabberFPS, "30");

  nub::soft_ref<PingSonar> pingSonar(new PingSonar(manager,"PingSonar",                                           "PingSonar","/dev/ttyUSB0",3));

  nub::soft_ref<VectorHistField> vectorHistField(new VectorHistField
                                               (manager,"VectorHistField",
                                              "VectorHistField",itsDim,itsDim));


  nub::soft_ref<Roomba> roomba(new Roomba(manager,"Roomba",
                                                   "Roomba","/dev/ttyUSB1"));

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(roomba);
   manager.addSubComponent(ofs);
  manager.addSubComponent(pingSonar);
  manager.addSubComponent(vectorHistField);

  manager.exportOptions(MC_RECURSE);

  // parse command-line
  if( manager.parseCommandLine( argc, argv, "", 0, 0 ) == false ) return(1);

  // setup signal handling:
  signal(SIGHUP, terminate); signal(SIGINT, terminate);
  signal(SIGQUIT, terminate); signal(SIGTERM, terminate);

  manager.start();

  LINFO("starting roomba....");
  roomba->sendStart();
  LINFO("setting to full mode....");
  roomba->setMode(2);


  roomba->setSpeed(0);
  sleep(1);
  // timer initialization
  Timer tim;
  tim.reset();

  Image<PixRGB<byte> > dispImg(itsDim*itsSpacing, itsDim*itsSpacing,ZEROS);
  Point2D<int> itsFixedRobotPos(itsDim/2,itsDim-5);

  static int dist=0;
  static int ang=0;

  //create obstacle template field once
  float sigma = 5.5;
  float amp= 45.0;
  Image<geom::vec2f> obsTemplate = vectorHistField->obstacleTemplate(sigma,amp);

  //int cnt=0;
  // get ready for main loop:
  while (goforever)
    {

        std::vector<int> dists = pingSonar->getDists();
        std::vector<Point2D<float> > sensor;

        LINFO("getting dists %d,%d,%d",dists.at(0),dists.at(1),dists.at(2));

        sensor.push_back(Point2D<float> ((float)(dists.at(0)*itsDim/3000),150.0));
        sensor.push_back(Point2D<float> ((float)(dists.at(1)*itsDim/3000),90.0));
        sensor.push_back(Point2D<float> ((float)(dists.at(2)*itsDim/3000),30.0));

        roomba->getDistanceAngle(dist,ang);
        LINFO("dist travelled %d, angle turned %d",roomba->getDist(),roomba->getAngle());

        Image<PixRGB<byte> > blobs = vectorHistField->updateField(sensor,                             Point2D<int>(0,0), ang, dist, Point2D<int> (15,90),obsTemplate);

        dispImg = vectorHistField->plotGridField(itsSpacing);
        ofs->writeRGB(dispImg,"Output",FrameInfo("output",SRC_POS));
        //ofs->writeRGB(dispImg,sformat("Output%d",cnt++));


        geom::vec2f a;
        a = vectorHistField->getVectorAt(itsFixedRobotPos);

        robotCmd cmd = vec2motor(a);
        //LINFO("motor %d,radius %d", cmd.motor,cmd.radius);

        //cmd.motor = 0;
        //cmd.radius =0;
        try
        {
           roomba->setSpeed(cmd.motor);
           roomba->setRadius(cmd.radius);
        }
        catch (...) {}


        if(tim.getSecs() > 65.0)
            goforever=false;

            // You can add more commands here.
        usleep(100 * 1000);

    }


  roomba->setSpeed(0);

  manager.stop();

}



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */


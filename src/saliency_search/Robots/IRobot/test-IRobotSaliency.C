/*!@file test-IRobot.C a test the irobot service */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/IRobot/test-IRobotSaliency.C $
// $Id: test-IRobotSaliency.C 12962 2010-03-06 02:13:53Z irock $
//

#include <Ice/Ice.h>
#include "Ice/IRobot.ice.H"
#include "Ice/ImageIce.ice.H"
#include "Ice/IceImageUtils.H"

#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "Image/ColorOps.H"
#include "Image/CutPaste.H"
#include "Image/ShapeOps.H"
#include "Image/MathOps.H"
#include "GUI/XWinManaged.H"
#include "GUI/ImageDisplayStream.H"
#include "Util/Timer.H"
#include "Robots/RobotBrain/RobotCommon.H"
#include "Neuro/getSaliency.H"

using namespace std;
using namespace Robots;

#define KEY_UP 98
#define KEY_DOWN 104
#define KEY_LEFT 100
#define KEY_RIGHT 102

int getKey(nub::ref<OutputFrameSeries> &ofs)
{
  const nub::soft_ref<ImageDisplayStream> ids =
    ofs->findFrameDestType<ImageDisplayStream>();

  const rutz::shared_ptr<XWinManaged> uiwin =
    ids.is_valid()
    ? ids->getWindow("Output")
    : rutz::shared_ptr<XWinManaged>();
  if (uiwin.is_valid())
    return uiwin->getLastKeyPress();

  return -1;
}

//////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{

  MYLOGVERB = LOG_INFO;
  ModelManager manager("test-IRobot");

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  nub::ref<GetSaliency> saliency(new GetSaliency(manager));
  manager.addSubComponent(saliency);

  Timer localTimer;

  int status = 0;
  Ice::CommunicatorPtr ic;
  try {
    ic = Ice::initialize(argc, argv);
    Ice::ObjectPrx base = ic->stringToProxy(
        "IRobotService:default -p 10000 -h " ROBOT_IP);
    IRobotPrx iRobot = IRobotPrx::checkedCast(base);
    if(!iRobot)
      throw "Invalid proxy";

    manager.exportOptions(MC_RECURSE);

    if (manager.parseCommandLine((const int)argc, (const char**)argv, "", 0, 0) == false)
      return 1;
    manager.start();


    iRobot->sendStart();
    iRobot->setMode(Robots::SafeMode);
    localTimer.reset();


    float speed = 0, steering = 0;
    Timer localTimer;

    int id = 0;
    id=id;

    Image<PixRGB<byte> > iRobotImg(320,240, ZEROS);
    ofs->writeRGB(iRobotImg, "Output", FrameInfo("Output", SRC_POS));

    while(true)
    {
      localTimer.reset();
   //   LINFO("Get %i\n", id++);
      ImageIceMod::ImageIce imgIce = iRobot->getImageSensor(320*4 + id, false);
    //  LINFO("Rate: %f fps", 1.0F/localTimer.getSecs());

      if (imgIce.pixSize == 1)
       iRobotImg = toRGB(Ice2Image<byte>(imgIce));
      else
       iRobotImg = Ice2Image<PixRGB<byte> >(imgIce);

      ////Show the image from the robot camera

      //Get Saliency of image
      const int num_salient_spots = saliency->compute(iRobotImg, SimTime::SECS(1));
      Image<float> salmap(iRobotImg.getDims(),ZEROS);

      salmap = rescale(saliency->getSalmap(), iRobotImg.getDims());

      inplaceNormalize(salmap, 0.0F, 255.0F); //located in MathOps.H

      LINFO("%d value at loc = %f", num_salient_spots, salmap.getVal(150));
      LINFO("img size = %d,%d, salmap size = %d,%d",iRobotImg.getWidth(),
            iRobotImg.getHeight(),salmap.getWidth(),salmap.getHeight());

      Image<PixRGB<byte> > dispImage(642,240, ZEROS);

      //Display the image from camera next to the saliency map of the image
      inplacePaste(dispImage, iRobotImg, Point2D<int> (0,0));
      inplacePaste(dispImage, toRGB<byte>(salmap), Point2D<int> (321,0));


     //Get other sensors
      float dist, ang;
      iRobot->getDistanceAngle(dist, ang);
      printf("%f %f %f %f %f %i\n", speed, steering, dist, ang, localTimer.get()/1000.0F,
          ofs->frame());
      fflush(stdout);

    //  drawLine(iRobotImg, Point2D<int>(iRobotImg.getWidth()/2, 0),
    //      Point2D<int>(iRobotImg.getWidth()/2, iRobotImg.getHeight()),
    //      PixRGB<byte>(255,0,0));

      //ofs->writeRGB(iRobotImg, "Cross", FrameInfo("Cross", SRC_POS));
      ofs->updateNext();

     //Display image with saliency map
      ofs->writeRGB(dispImage, "Output", FrameInfo("Output", SRC_POS));

      //ofs->writeRGB(iRobotImg, "Output", FrameInfo("Output", SRC_POS));

      usleep(100000);
      int key = getKey(ofs);

      if (key != -1)
      {
        //LINFO("Got Key:%d", key);
        switch(key)
        {
          case KEY_UP:
            speed       =  0.05;
            steering = 0;
            iRobot->setSteering(steering);
            iRobot->setSpeed(speed);
            break;
          case KEY_DOWN:
            speed     = -0.05;
            steering = 0;
            iRobot->setSteering(steering);
            iRobot->setSpeed(speed);
            break;
          case KEY_LEFT:
            steering =  0.05;
            iRobot->setSteering(steering);
            iRobot->setSpeed(speed);
            break;
          case KEY_RIGHT:
            steering = -0.05;
            iRobot->setSteering(steering);
            iRobot->setSpeed(speed);
            break;
          case 65: //space
            speed = 0;
            steering = 0;
            iRobot->motorsOff(0);
            break;
          case 33: //p for playing the song
            iRobot->setMode(Robots::SafeMode);
            //LINFO("Play song");
            iRobot->playSong(0);
            break;
          case 40: //d for dock with base station
            //LINFO("Docking");
            iRobot->setMode(Robots::CoverAndDockMode);
            break;

          case 39: //s for status
            {
              float chargeState = iRobot->getSensorValue(21);
              float voltage = iRobot->getSensorValue(22);
              float current = iRobot->getSensorValue(23);
              float batteryTemp = iRobot->getSensorValue(24);
              float batteryCharge = iRobot->getSensorValue(25);
              float batteryCapacity = iRobot->getSensorValue(26);

              LINFO("ChargeState %i", (int)chargeState);
              LINFO("Voltage %i mV", (unsigned int)voltage);
              LINFO("Current %i mA", (int)current);
              LINFO("Battery Temperature %iC", (int)batteryTemp);
              LINFO("Battery Charge %i mAh", (int)batteryCharge);
              LINFO("Battery Capacity %i mAh", (int)batteryCapacity);
            }
            break;
          case 27: //r
            iRobot->sendStart();
            iRobot->setMode(Robots::SafeMode);
            break;


          default:
            LINFO("Unknown key %i\n", key);
            break;
        }
        localTimer.reset();
       // iRobot->setSteering(steering);
       // iRobot->setSpeed(speed);
       // usleep(100000);
      // LINFO("Rate2: %f fps", 1.0F/localTimer.getSecs());
      }
    }

  }
  catch (const Ice::Exception& ex) {
    cerr << ex << endl;
    status = 1;
  }
  catch(const char* msg) {
    cerr << msg << endl;
    status = 1;
  }
  if (ic)
    ic->destroy();
  return status;
}

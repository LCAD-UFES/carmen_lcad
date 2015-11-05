/*!@file VFAT/test-multiTrack.C Test IEEE1394 frame grabbing and X display */

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
// Primary maintainer for this file:  T. Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/test-multiTrack.C $
// $Id: test-multiTrack.C 14376 2011-01-11 02:44:34Z pez $
//


#include "Component/ModelManager.H"
#include "Component/ModelOptionDef.H"
#include "Devices/CameraControl.H"
#include "Devices/DeviceOpts.H"
#include "Devices/FrameGrabberFactory.H"
#include "GUI/XWindow.H"
#include "VFAT/segmentImageMerge.H"
#include "rutz/shared_ptr.h"

//#include <pthread.h>
#include <cstdio>
#include <cstdlib>
#include <iostream>

// number of frames over which framerate info is averaged:
#define NAVG 20

int main(const int argc, const char **argv)
{

  // instantiate a model manager:
  ModelManager manager("Multi Frame Grabber Tester");
  manager.allowOptions(OPTEXP_NONE);

  // Instantiate our various ModelComponents:

  nub::soft_ref<FrameIstream> gb1(makeIEEE1394grabber(manager));
  gb1->setModelParamVal("FrameGrabberSubChan", 0);
  //gb1->setModelParamVal("FrameGrabberGamma", 2);
  //gb1->setModelParamVal("framegrabber-whiteness", 1);

  nub::soft_ref<FrameIstream> gb2(makeIEEE1394grabber(manager));
  gb2->setModelParamVal("FrameGrabberSubChan", 1);
  //gb2->setModelParamVal("FrameGrabberGamma", 2);
  //gb2->setModelParamVal("framegrabber-whiteness", 100);

  nub::soft_ref<FrameIstream> gb3(makeIEEE1394grabber(manager));
  gb3->setModelParamVal("FrameGrabberSubChan", 2);
  //gb3->setModelParamVal("FrameGrabberGamma", 2);

  nub::soft_ref<FrameIstream> gb4(makeIEEE1394grabber(manager));
  gb4->setModelParamVal("FrameGrabberSubChan", 3);
  //gb3->setModelParamVal("framegrabber-whiteness", 1000);

  //gb3->setModelParamVal("FrameGrabberChannel", 1);

  manager.addSubComponent(gb1);
  manager.addSubComponent(gb2);
  manager.addSubComponent(gb3);
  manager.addSubComponent(gb4);


  // we don't want people messing around with some of our options; so
  // let's selectively export only those thay can play with:
  manager.allowOptions(OPTEXP_ALL);
  manager.doRequestOption(&OPT_FrameGrabberDims);
  manager.doRequestOption(&OPT_FrameGrabberMode);
  manager.doRequestOption(&OPT_FrameGrabberFPS);
  manager.doRequestOption(&OPT_FrameGrabberNbuf);
  manager.allowOptions(OPTEXP_NONE);

  nub::soft_ref<CameraControl>
    camera1(new CameraControl(manager, "Camera Controller", "CameraControl",
                             0, true, 0, 1, 1));
  nub::soft_ref<CameraControl>
    camera2(new CameraControl(manager, "Camera Controller", "CameraControl",
                             0, true, 2, 3, 1));
  nub::soft_ref<CameraControl>
    camera3(new CameraControl(manager, "Camera Controller", "CameraControl",
                             0, true, 4, 5, 1));
  nub::soft_ref<CameraControl>
    camera4(new CameraControl(manager, "Camera Controller", "CameraControl",
                             0, true, 6, 7, 1));

  manager.addSubComponent(camera1);
  manager.addSubComponent(camera2);
  manager.addSubComponent(camera3);
  manager.addSubComponent(camera4);

  // Parse command-le:
  if (manager.parseCommandLine(argc, argv, "", 0, 0) == false) return(1);

  // do post-command-line configs:
  //nub::soft_ref<FrameIstream> gb1 = gbc1->getFrameGrabber();
  //nub::soft_ref<FrameIstream> gb2 = gbc2->getFrameGrabber();
  /*if (gb1.isInvalid())
    LFATAL("You need to select a frame grabber type via the "
           "--fg-type=XX command-line option for this program "
           "to be useful");
  if (gb2.isInvalid())
    LFATAL("You need to select a frame grabber type via the "
           "--fg-type=XX command-line option for this program "
           "to be useful");*/
  int width = gb1->getWidth(), height = gb1->getHeight();
  float delay[4];
  delay[0] = 0; delay[1] = 0; delay[2] = 0; delay[3] = 0;

  // let's get all our ModelComponent instances started:
  manager.start();
  XWindow wini1(Dims(width, height), 0, 0, "test-input window 1");
  XWindow wini2(Dims(width, height), 0, 0, "test-input window 2");
  XWindow wini3(Dims(width, height), 0, 0, "test-input window 3");
  XWindow wini4(Dims(width, height), 0, 0, "test-input window 4");

  XWindow roomOver(Dims(400,400),0,0,"Room Overhead");
  XWindow roomFront(Dims(400,400),0,0,"Room Front");
  Image<PixRGB<byte> > overhead;
  Image<PixRGB<byte> > front;


  Timer tim;

  Image< PixRGB<float> > fima;

  std::vector< Image< PixRGB<byte> > > ima;
  ima.resize(4);

  std::vector< Image< PixRGB<byte> > > display;
  display.resize(4);

  Timer camPause[4];       // to pause the move command
  camPause[0].reset();
  camPause[1].reset();
  camPause[2].reset();
  camPause[3].reset();

  uint64 t[NAVG]; int frame = 0;

  // Create tracker and state how many trackers will be used
  segmentImageMerge segmenter(4);

  //****************************
  // set up tracking parameters
  //****************************

  // HSV mean color to start with and standard deviation
  //segmenter.setTrackColor(12,8,0.23,0.08,135,35,0,true,15);
  //segmenter.setTrackColor(12,8,0.23,0.08,135,35,1,true,15);
  //segmenter.setTrackColor(12,8,0.23,0.08,135,35,2,true,15);
  //segmenter.setTrackColor(12,8,0.23,0.08,135,35,3,true,15);
  segmenter.setTrackColor(10,10,0.15,0.20,150,150,0,true,15);
  segmenter.setTrackColor(10,10,0.15,0.20,150,150,1,true,15);
  segmenter.setTrackColor(10,10,0.15,0.20,150,150,2,true,15);
  segmenter.setTrackColor(10,10,0.15,0.20,150,150,3,true,15);
  //segmenter.setTrackColor(13,7,0.17,0.3,156,150,0,true,15);
  //segmenter.setTrackColor(13,7,0.17,0.3,156,150,1,true,15);
  //segmenter.setTrackColor(13,7,0.17,0.3,156,150,2,true,15);
  //segmenter.setTrackColor(13,7,0.17,0.3,156,150,3,true,15);

  // HSV Hard Boundaries (H.upper,H.lower,...,tracker)
  segmenter.setAdaptBound(20,5,.30,.15,170,100,0);
  segmenter.setAdaptBound(20,5,.30,.15,170,100,1);
  segmenter.setAdaptBound(20,5,.30,.15,170,100,2);
  segmenter.setAdaptBound(20,5,.30,.15,170,100,3);


  //segmenter.setAdaptBound(15,5,.30,.25,140,100,0);
  //segmenter.setAdaptBound(15,5,.30,.25,140,100,1);
  //segmenter.setAdaptBound(15,5,.30,.25,140,100,2);
  //segmenter.setAdaptBound(15,5,.30,.25,140,100,0);
  //segmenter.setAdaptBound(15,5,.30,.25,140,100,1);
  //segmenter.setAdaptBound(15,5,.30,.25,140,100,2);
  //segmenter.setAdaptBound(15,5,.30,.25,140,100,3);
  //segmenter.setAdaptBound(15,5,.30,.25,140,100,3);
  //segmenter.setAdaptBound(40,5,.40,.10,170,100,0);
  //segmenter.setAdaptBound(40,5,.40,.10,170,100,1);
  //segmenter.setAdaptBound(40,5,.40,.10,170,100,2);
  //segmenter.setAdaptBound(40,5,.40,.10,170,100,3);

  // Inspection box size for each tracker
  segmenter.setFrame(0,0,width/4,height/4,width/4,height/4,0);
  segmenter.setFrame(0,0,width/4,height/4,width/4,height/4,1);
  segmenter.setFrame(0,0,width/4,height/4,width/4,height/4,2);
  segmenter.setFrame(0,0,width/4,height/4,width/4,height/4,3);

  // RGB color of tracker circle for display
  segmenter.setCircleColor(0,255,0,0);
  segmenter.setCircleColor(0,255,0,1);
  segmenter.setCircleColor(0,255,0,2);
  segmenter.setCircleColor(0,255,0,3);

  // RGB color of tracker bounding box for display
  segmenter.setBoxColor(255,255,0,0);
  segmenter.setBoxColor(255,255,0,1);
  segmenter.setBoxColor(255,255,0,2);
  segmenter.setBoxColor(255,255,0,3);

  // set what type of color adaptation to use and if to use it
  segmenter.setAdapt(3,true,3,true,3,true,0);
  segmenter.setAdapt(3,true,3,true,3,true,1);
  segmenter.setAdapt(3,true,3,true,3,true,2);
  segmenter.setAdapt(3,true,3,true,3,true,3);

  CameraParams* params = (CameraParams*)calloc(4, sizeof(CameraParams));
  CameraParams* tempParams = (CameraParams*)calloc(2, sizeof(CameraParams));
  PixelPoint* points = (PixelPoint*)calloc(4, sizeof(PixelPoint));
  PixelPoint* tempPoints = (PixelPoint*)calloc(2, sizeof(PixelPoint));

  params[0] = CameraParams(15.0, 0.0, 3.0, 90.0, -90.0, 0.465/2.54, 2.5);
  params[1] = CameraParams(5.5, 0.0, 3.0, 90.0, -90.0, 0.465/2.54, 2.5);
  params[2] = CameraParams(-5.5, 0.0, 3.0, 90.0, -90.0, 0.465/2.54, 2.5);
  params[3] = CameraParams(-15.0, 0.0, 3.0, 90.0, -90.0, 0.465/2.54, 2.5);


  points[0] = PixelPoint(0.0, 0.0);
  points[1] = PixelPoint(0.0, 0.0);
  points[2] = PixelPoint(0.0, 0.0);
  points[3] = PixelPoint(0.0, 0.0);
  // iteratively grab video from source and feed it into tracker
  overhead.resize(288,384);
  front.resize(288,288);

  while(1) {
    tim.reset();


    ima[0] = gb1->readRGB();
    ima[1] = gb2->readRGB();
    ima[2] = gb3->readRGB();
    ima[3] = gb4->readRGB();

    uint64 t0 = tim.get();  // to measure display time

    //display[0] = ima[0];
    //display[1] = ima[1];
    //display[2] = ima[2];
    //display[3] = ima[3];

    // call tracker on images
    segmenter.trackImageMulti(&ima,4);

    int modi[4],modj[4];
    // get camera movement parameters from tracker
    for(int i = 0; i < 4; i++)
    {
      if(camPause[i].get() > delay[i])
      {
        float doPan, doTilt;
        if(segmenter.returnLOT(i) == false)
        {
          segmenter.getImageTrackXY2(&modi[i],&modj[i],i);
          modi[i] = modi[i]*8;
          modj[i] = 480-modj[i]*8;
          // stereo stuff
          points[i].x = (modi[i]-320)*(4.6/(2.54*659));
          points[i].y = (modj[i]-240)*(3.97/(2.54*494));
          if(modi[i] > 0 && modi[i] < 640 && modj[i] > 0 && modj[i] < 480)
          {
            //std::cout << "doing Camera " << i << "\n";
            if(i == 0)
            {
              delay[i] = camera1->moveCamXYFrame(modi[i],modj[i],i);
              if((delay[i] > 0) || (delay[i] == -5))
              {
                camPause[i].reset();
                doPan = camera1->getCurrentPan();
                doTilt = camera1->getCurrentTilt();
                segmenter.setCameraPosition(doPan,doTilt,i,true);
              }
            }
            if(i == 1)
            {
              delay[i] = camera2->moveCamXYFrame(modi[i],modj[i],i);
              if((delay[i] > 0) || (delay[i] == -5))
              {
                camPause[i].reset();
                doPan = camera2->getCurrentPan();
                doTilt= camera2->getCurrentTilt();
                segmenter.setCameraPosition(doPan,doTilt,i,true);
              }
            }
            if(i == 2)
            {
              delay[i] = camera3->moveCamXYFrame(modi[i],modj[i],i);
              if((delay[i] > 0) || (delay[i] == -5))
              {
                camPause[i].reset();
                doPan = camera3->getCurrentPan();
                doTilt = camera3->getCurrentTilt();
                segmenter.setCameraPosition(doPan,doTilt,i,true);
              }
            }
            if(i == 3)
            {

              delay[i] = camera4->moveCamXYFrame(modi[i],modj[i],i);
              if((delay[i] > 0) || (delay[i] == -5))
              {
                camPause[i].reset();
                doPan = camera4->getCurrentPan();
                doTilt = camera4->getCurrentTilt();
                segmenter.setCameraPosition(doPan,doTilt,i,true);
              }
            }
            // stereo stuff
            params[i].theta = 180.0-doTilt;
            params[i].phi = -180.0+doPan;
          }
        }
        else
        {
          if(segmenter.doMoveCamera(i,&doPan,&doTilt) == true)
          {
            //LINFO("MOVING LOT camera %d to %f %f",i,doPan,doTilt);

            if(i == 0)
            {
              delay[i] = camera1->moveCamTPFrame(doPan,doTilt,i);
              if(delay[i] > 0)
              {
                segmenter.setCameraPosition(doPan,doTilt,i,false);
                camPause[i].reset();
              }
            }
            if(i == 1)
            {
              delay[i] = camera2->moveCamTPFrame(doPan,doTilt);
              if(delay[i] > 0)
              {
                segmenter.setCameraPosition(doPan,doTilt,i,false);
                camPause[i].reset();
              }
            }
            if(i == 2)
            {
              delay[i] = camera3->moveCamTPFrame(doPan,doTilt);
              if(delay[i] > 0)
              {
                segmenter.setCameraPosition(doPan,doTilt,i,false);
                camPause[i].reset();
              }
            }
            if(i == 3)
            {
              delay[i] = camera4->moveCamTPFrame(doPan,doTilt);
              if(delay[i] > 0)
              {
                segmenter.setCameraPosition(doPan,doTilt,i,false);
                camPause[i].reset();
              }
            }
          }
        }
      }
    }

    // END

    // draw all our X window displays

    int highest = 0;
    int nextHighest = 1;
    int colorSetR[4];
    int colorSetB[4];
    for(int i = 0; i < 4; i++)
    {
      colorSetR[i] = 0;
      colorSetB[i] = 255;

      float p = segmenter.returnCameraProb(i);
      LINFO("CAMERA %d is P %f",i,p);
      bool high = true;
      bool nextHigh = false;
      bool stop = false;
      for(int j = 0; j < 4; j++)
      {
        if(j != i)
        {
          if(p < segmenter.returnCameraProb(j))
          {
            //LINFO("%f is LT %f",p, segmenter.returnCameraProb(j));
            high = false;
            if((nextHigh == false) && (stop == false))
            {
              nextHigh = true;
              stop = true;
            }
            else
            {
              //LINFO("NEXTHIGHEST FALSE");
              nextHigh = false;
            }
          }
        }
      }
      if(high == true)
      {
        highest = i;
      }
      if(nextHigh == true)
      {
        nextHighest = i;
      }
    }
    colorSetR[highest] = 255;
    colorSetB[highest] = 0;
    colorSetR[nextHighest] = 255;
    colorSetB[nextHighest] = 0;

    LINFO("HIGHEST %d NEXT HIGHEST %d",highest,nextHighest);
    //if((segmenter.returnLOT(highest) == false) && (segmenter.returnLOT(nextHighest) == false))
    //{
      Point3D retPoint = Point3D(0.0, 0.0, 0.0);
      tempParams[0] = params[0];
      tempParams[1] = params[1];
      tempPoints[0] = points[0];
      tempPoints[1] = points[1];
      bool retVal = segmenter.StereoMatch(tempPoints, tempParams, &retPoint);

      //printf("###############################################################\n");

      //        printf("%f %f\n", tempPoints[0].x, tempPoints[0].y);
      //        printf("%f %f %f %f %f %f %f\n", tempParams[0].x, tempParams[0].y, tempParams[0].z,
      //                        tempParams[0].theta, tempParams[0].phi, tempParams[0].f, tempParams[0].r);
      //
      //                printf("%f %f\n", tempPoints[1].x, tempPoints[1].y);
      //        printf("%f %f %f %f %f %f %f\n", tempParams[1].x, tempParams[1].y, tempParams[1].z,
      //                        tempParams[1].theta, tempParams[1].phi, tempParams[1].f, tempParams[1].r);
      if(retVal)
      {
        overhead.resize(400,400,true);
        front.resize(400,400,true);
        drawGrid(overhead, 96,96,1,1,PixRGB<byte>(150,150,150));
        drawGrid(front, 96,96,1,1,PixRGB<byte>(150,150,150));

        drawCircle(overhead, Point2D<int>(140,200)
                   ,2,PixRGB<byte>(colorSetR[0],0,colorSetB[0]),3);
        drawCircle(overhead, Point2D<int>(180,200)
                   ,2,PixRGB<byte>(colorSetR[1],0,colorSetB[1]),3);
        drawCircle(overhead, Point2D<int>(220,200)
                   ,2,PixRGB<byte>(colorSetR[2],0,colorSetB[2]),3);
        drawCircle(overhead, Point2D<int>(260,200)
                   ,2,PixRGB<byte>(colorSetR[3],0,colorSetB[3]),3);

        drawCircle(overhead, Point2D<int>(200,220)
                   ,2,PixRGB<byte>(0,255,255),3);
        float pointX = 200-(4*retPoint.x);
        float pointY = 200-(4*(-1*retPoint.y));

        //std::cout << "pointX " << pointX << " pointY " << pointY << "\n";
        if(((pointX < 400) && (pointX > 0)) && ((pointY < 400) && (pointY > 0)))
        {
          printf("x=%f y=%f z=%f\n", retPoint.x, retPoint.y, retPoint.z);
          drawCircle(overhead, Point2D<int>((int)pointX,(int)pointY)
                     ,2,PixRGB<byte>(255,0,0),2);
        }
      }
      else
        printf("Not Admissible\n");
      //}
    //printf("###############################################################\n");

    wini1.drawImage(ima[0]);
    wini2.drawImage(ima[1]);
    wini3.drawImage(ima[2]);
    wini4.drawImage(ima[3]);
    roomOver.drawImage(overhead);
    roomFront.drawImage(front);


    t[frame % NAVG] = tim.get();
    t0 = t[frame % NAVG] - t0;
    if (t0 > 28) LINFO("Display took %llums", t0);

    // compute and show framerate over the last NAVG frames:
    if (frame % NAVG == 0 && frame > 0)
    {
      uint64 avg = 0; for (int i = 0; i < NAVG; i ++) avg += t[i];
      float avg2 = 1000.0 / (float)avg * NAVG;
      printf("Framerate: %.1f fps\n", avg2);
    }
    frame ++;
  }

  manager.stop();
  return 0;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

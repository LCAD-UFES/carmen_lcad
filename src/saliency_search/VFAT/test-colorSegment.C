/*!@file VFAT/test-colorSegment.C Test IEEE1394 frame grabbing and X display */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/test-colorSegment.C $
// $Id: test-colorSegment.C 6988 2006-08-11 17:15:38Z rjpeters $
//


#include "Component/ModelManager.H"
#include "Devices/CameraControl.H"
#include "Devices/FrameGrabberConfigurator.H"
#include "GUI/XWindow.H"
#include "Transport/FrameIstream.H"
#include "VFAT/segmentImageMerge.H"
#include "rutz/shared_ptr.h"

#include <cstdio>
#include <cstdlib>

// number of frames over which framerate info is averaged:
#define NAVG 20

int main(const int argc, const char **argv)
{

  // instantiate a model manager:
  ModelManager manager("Frame Grabber Tester");

  // Instantiate our various ModelComponents:
  nub::soft_ref<FrameGrabberConfigurator>
    gbc(new FrameGrabberConfigurator(manager));
  manager.addSubComponent(gbc);

  nub::soft_ref<CameraControl>
    camera(new CameraControl(manager, "Camera Controller", "CameraControl",
                             0, true, 0, 1, 1));
  manager.addSubComponent(camera);

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv, "", 0, 0) == false) return(1);

  // do post-command-line configs:
  nub::soft_ref<FrameIstream> gb = gbc->getFrameGrabber();
  if (gb.isInvalid())
    LFATAL("You need to select a frame grabber type via the "
           "--fg-type=XX command-line option for this program "
           "to be useful");
  int width = gb->getWidth(), height = gb->getHeight();
  float delay = 0;

  // let's get all our ModelComponent instances started:
  manager.start();
  XWindow wini(Dims(width, height), 0, 0, "test-input window");
  XWindow wino1(Dims(width/4, height/4), 0, 0, "test-output window 1");
  XWindow wino2(Dims(width/4, height/4), 0, 0, "test-output window 2");
  XWindow winAux1(Dims(100, 450), 0, 0, "HSV levels 1");
  XWindow winAux2(Dims(100, 450), 0, 0, "HSV levels 2");
  Timer tim; Image< PixRGB<byte> > ima; Image< PixRGB<float> > fima;
  Image< PixRGB<byte> > display;
  Timer camPause;       // to pause the move command
  camPause.reset();
  uint64 t[NAVG]; int frame = 0;
  segmentImageMerge segmenter(2);
  // set up tracking parameters
  //segmenter.setTrackColor(10,10,0.15,0.20,150,150,0,true,15);
  segmenter.setTrackColor(13,7,0.17,0.3,156,30,0,true,15);
  //segmenter.setTrackColor(10,10,0.15,0.20,150,150,1,false,15);
  segmenter.setTrackColor(270,10,0.18,0.25,60,60,1,true,15);
  segmenter.setAdaptBound(20,5,.30,.15,170,100,0);
  //segmenter.setAdaptBound(15,5,.30,.25,140,100,0);
  segmenter.setAdaptBound(285,265,.25,.15,80,40,1);
  segmenter.setFrame(0,0,width/4,height/4,width/4,height/4,0);
  segmenter.setFrame(0,0,width/4,height/4,width/4,height/4,1);
  segmenter.setCircleColor(0,255,0,0);
  segmenter.setCircleColor(0,0,255,1);
  segmenter.setBoxColor(255,255,0,0);
  segmenter.setBoxColor(255,0,255,1);

  segmenter.setAdapt(3,true,3,true,3,true,0);
  segmenter.setAdapt(3,true,3,true,3,true,1);

  while(1) {
    tim.reset();
    ima = gb->readRGB();
    uint64 t0 = tim.get();  // to measure display time

    Image<PixRGB<byte> > Aux1;
    Image<PixRGB<byte> > Aux2;
    Aux1.resize(100,450,true);
    Aux2.resize(100,450,true);

    Image<byte> outputI1;
    Image<byte> outputI2;

    display = ima;
    segmenter.trackImage(ima,&display,0,&Aux1);
    segmenter.trackImage(ima,&display,1,&Aux2);
    segmenter.mergeImages(&display);

    if(camPause.get() > delay)
    {
      int modi,modj;
      segmenter.getImageTrackXY(&modi,&modj,0);
      //segmenter.getImageTrackXYMerge(&modi,&modj);
      modi = modi*8;
      modj = 480-modj*8;
      if(modi > 0 && modi < 640 && modj > 0 && modj < 480)
      {
        if(segmenter.returnLOT(0) == false)
        {
          camPause.reset();
          delay = camera->moveCamXYFrame(modi,modj);
        }
      }
    }

    Image<byte> temp1 = segmenter.returnCandidateImage(0);
    Image<byte> temp2 = segmenter.returnCandidateImage(1);
    wini.drawImage(display);
    //wino1.drawImage(outputI1);
    wino1.drawImage(temp1);
    wino2.drawImage(temp2);
    winAux1.drawImage(Aux1);
    winAux2.drawImage(Aux2);
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

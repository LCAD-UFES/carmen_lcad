 /*!@file Robots/BeoBorg/BeoBorgCtrl.C BeoBorg main control program  */

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
// Primary maintainer for this file: farhan baluch <fbaluch@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/BeoBorg/BeoBorgCtrl.C $
// $Id: BeoBorgCtrl.C  irock $
//

#include "Component/ModelManager.H"
#include "Util/log.H"
#include "Util/Types.H"
#include "Util/sformat.H"
#include "Util/Timer.H"
#include "Image/DrawOps.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Robots/BeoBorg/BeoBorg.H"
#include "Devices/AudioGrabber.H"
#include "Devices/AudioMixer.H"
#include "Audio/AudioWavFile.H"
#include "GUI/XWindow.H"

#include <stdio.h>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <pthread.h>
#include <queue>
#include <fstream>
#include <cmath>


//function prototype
long double filterSamp(float samp);

Timer tim;
static bool goforever = true;

std::queue<AudioBuffer<uint16> > rec;
std::queue<AudioBuffer<uint16> > dispQ;
pthread_mutex_t qmutex_data;
pthread_mutex_t qmutex_disp;

uint nsamples;
int numSamps=0;
int numBuffs=0;
int nChans =0;

int fl=300; //low cutoff for highpass
int fs=20000; //sampling frequency

long double A = exp(-2*M_PI*fl/fs);
long double B = 1-A;
long double state = 0.0;

//##############################################################

//! save data as it becomes available
static void* saveData(void*)
{
  
  bool hadStuff = false;
  AudioBuffer<uint16> buf;
  std::ofstream outFile("ephys.dat");

  while(goforever)
    {
      pthread_mutex_lock(&qmutex_data);
      if(rec.size())
        {
          buf = rec.front();
          rec.pop();
          hadStuff = true;       
        }
      
      pthread_mutex_unlock(&qmutex_data);
      
      
      if (hadStuff)
        {
                 
          numBuffs++;
          for (uint i=0; i < buf.nsamples()-1; i++)
          {
            numSamps++;
            for (int c=0; c < nChans;c++)
              // outFile << (buf.getVal(i,c))*(2.5F/65536) << "\t ";
              outFile << filterSamp(((buf.getVal(i,c))*(2.5F/65536))-1.225) << "\t ";
            
              
            outFile << std::endl;                          
          }
          hadStuff = false;
        }
    }
    outFile.close();
    return NULL;

}


//##############################################################
//! simple high pass filter on data

long double filterSamp(float samp)
{
  
  state = B*samp + A*state;
  return samp-state;
  
}

//##############################################################

//!plot the result
static void* plotData(void*)
{
    // draw the buffer into the image:
  int plotHeight = 128;
  XWindow win(Dims(nsamples, plotHeight * nChans),
                 -1, -1, "test-audioGrab window");
  Image<byte> ima(nsamples, plotHeight*nChans, NO_INIT);
  Image<byte> imaPrev(nsamples, plotHeight*nChans, NO_INIT);
    
  Point2D<int> p1, p2;
 
  bool hasStuff = false;
  AudioBuffer<uint16> buffer;
  
  while (goforever)
    {
      imaPrev = ima;
      ima.clear();
      pthread_mutex_lock(&qmutex_disp);
      if(dispQ.size())
        {       
          buffer = dispQ.front();
          dispQ.pop();
          hasStuff = true;
        }
      pthread_mutex_unlock(&qmutex_disp);
      if(hasStuff)   
      {
          for (int c=0; c < nChans; c++)
            for (uint i = 1; i < nsamples; i ++)
              {
                float val = (buffer.getVal(i,c))*(2.5F/65536);
                float val1 = (buffer.getVal(i-1,c))*(2.5F/65536);
                
                float factor=plotHeight/2.5F;
                p1.i = i - 1;
                p1.j = plotHeight*(c+1) - val1*factor;
                p2.i = i;
                p2.j = plotHeight*(c+1) - val*factor;
                
                writeText(ima,Point2D<int>(4,plotHeight*(c+1)-plotHeight-5),
                          sformat("%f %f",val,tim.getSecs()).c_str());
                drawLine(ima, p1, p2, byte(255));
              } 
          hasStuff = false;
        }
        else
          ima = imaPrev;

      win.drawImage(ima);
    }

  return NULL;
  }


// ######################################################################

int main(int argc, const char **argv)
{
  // Instantiate a ModelManager:
  ModelManager manager("BeoBorg Control");

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  // Instantiate our various ModelComponents:
  nub::soft_ref<BeoBorg> beoBorg(new BeoBorg(manager,"BeoBorg",
                                             "BeoBorg","/dev/ttyUSB0"));
  manager.addSubComponent(beoBorg);

  
  // Instantiate our various ModelComponents:
  nub::soft_ref<AudioMixer> mix(new AudioMixer(manager));
  manager.addSubComponent(mix);

  nub::soft_ref<AudioGrabber> agb(new AudioGrabber(manager));
  manager.addSubComponent(agb);

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv,"<recording time/s>", 1, 1) == false) return(1);
  
  int recTime = atoi(manager.getExtraArg(0).c_str());
  // Let's get some of the option values, to configure our window:
  nsamples = agb->getModelParamVal<uint>("AudioGrabberBufSamples");
  nChans = agb->getModelParamVal<uint>("AudioGrabberChans");
  
  // let's get all our ModelComponent instances started:
  manager.start();

  std::ofstream outFileOdo("odomet.dat");
  pthread_t acqThread;
  pthread_t dispThread;

  int rc;
  rc = pthread_create(&acqThread,NULL,&saveData,NULL);
  if(rc)			
    LFATAL("\n ERROR: return code from pthread_create is %d \n", rc);
  
  LINFO("\n Created new acquisition thread (%d) ... \n", (int)acqThread);
  
  //  int dt;
  //dt = pthread_create(&dispThread,NULL,&plotData,NULL);
  //if(dt)			
  //  LFATAL("\n ERROR: return code from pthread_create is %d \n", dt);
  
  LINFO("\n Created new display thread (%d) ... \n", (int)dispThread);

  Image<PixRGB<byte> > img(800,800,ZEROS);
  int divisions=12;
  LINFO("WillKommen");
  

  int val; 
  tim.reset();
  val = 45;
  
  AudioBuffer<uint16> buffer;
    
  LINFO("Setting MOTOR ON");
    
  while(goforever)
  {
    //grab a buffer of neural data
    agb->grab(buffer);
     
     //fill display queue
    //pthread_mutex_lock(&qmutex_disp);
    //dispQ.push(buffer);
    //pthread_mutex_unlock(&qmutex_disp);
 
    //fill data queue
    pthread_mutex_lock(&qmutex_data);
    rec.push(buffer);
    pthread_mutex_unlock(&qmutex_data);
   
    beoBorg->setLeftMotor(val);
    beoBorg->setRightMotor(val);
    
    outFileOdo << beoBorg->getLeftEncoder() << "\t" << beoBorg->getRightEncoder() << "\t" << tim.getSecs();
    //std::cin >> val;
  
    //    LINFO("Left encoder value: %d",beoBorg->getLeftEncoder());
    //LINFO("Right encoder value: %d", beoBorg->getRightEncoder());
    //LINFO("Left PWM value: %d",beoBorg->getLeftPWM());
    //LINFO("Right PWM value: %d", beoBorg->getRightPWM());
    
    //beoBorg->sendRaw(0);
    
    //beoBorg->printRaw();
    //usleep(1000); 
    
    //      ofs->writeRGB(img, "Output", FrameInfo("output", SRC_POS));
    if(tim.getSecs()>=recTime)
      goforever =false;


  }
  LINFO("Setting MOTOR OFF");
  val =0;
  beoBorg->setLeftMotor(val);
  beoBorg->setRightMotor(val);
  LINFO("Left encoder value: %d",beoBorg->getLeftEncoder());
  LINFO("Right encoder value: %d", beoBorg->getRightEncoder());
  LINFO("Left PWM value: %d",beoBorg->getLeftPWM());
  LINFO("Right PWM value: %d", beoBorg->getRightPWM());
   
  // stop all our ModelComponents
  manager.stop();

  // all done!
  return 0;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

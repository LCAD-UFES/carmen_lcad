/*!@file Psycho/EyeTrackerISCAN.C Abstraction for an ISCAN eye-tracker */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/EyeTrackerISCAN.C $
// $Id: EyeTrackerISCAN.C 14159 2010-10-22 04:04:17Z ilink $
//

#ifndef PSYCHO_EYETRACKERISCAN_C_DEFINED
#define PSYCHO_EYETRACKERISCAN_C_DEFINED

#include "Psycho/EyeTrackerISCAN.H"

#include "Component/OptionManager.H"
#include "Devices/ParPort.H"
#include "Devices/Serial.H"
#include "Psycho/PsychoOpts.H"
#include "Psycho/PsychoDisplay.H"
#include "Util/sformat.H"
#include "Util/stats.H"
#include <cmath>
#include "Image/Point2D.H"
#include "Image/DrawOps.H"
#include "Image/AffineTransform.H"
#include <fstream>

// ######################################################################
EyeTrackerISCAN::EyeTrackerISCAN(OptionManager& mgr,
                                 const std::string& descrName,
                                 const std::string& tagName) :
  EyeTracker(mgr, descrName, tagName),
  itsParTrig(&OPT_EyeTrackerParTrig, this),
  itsSerDev(&OPT_EyeTrackerSerDev, this),
  itsParDev(&OPT_EyeTrackerParDev, this),
  itsSerial(new Serial(mgr, "ISCAN Serial Port", "ISCANSerial")),
  itsParPort(new ParPort(mgr, "ISCAN Parallel Port", "ISCANParPort")),
  itsRecalibCount(0)
{
  addSubComponent(itsSerial);
  addSubComponent(itsParPort);
  itsRequestQuickEyeS = false;

}

// ######################################################################
EyeTrackerISCAN::~EyeTrackerISCAN()
{  }

// ######################################################################
void EyeTrackerISCAN::start1()
{
  // configure our serial and parallel ports:
  itsSerial->setModelParamVal("DevName", itsSerDev.getVal());
  itsParPort->setModelParamVal("ISCANParPortDevName", itsParDev.getVal());
  itsSerial->configure(itsSerDev.getVal().c_str(),
                       115200, "8N1", false, false, 0);

  EyeTracker::start1();
}

// ######################################################################
void EyeTrackerISCAN::start2()
{
  // if using parallel trigger, be sure to initialize in non-tracking mode:
  if (itsParTrig.getVal())
    itsParPort->WriteData(255, 255); // turn all data bits to on

  EyeTracker::start2();
}

// ######################################################################
void EyeTrackerISCAN::calibrate(nub::soft_ref<PsychoDisplay> d)
{
  // let's display an ISCAN calibration grid:
  d->clearScreen();
  d->displayISCANcalib();
  d->waitForKey(true);

  // now run a 9-point calibration:
  d->clearScreen();
  d->displayText("<SPACE> to calibrate; other key to skip");
  int c = d->waitForKey(true);
  if (c == ' ') d->displayEyeTrackerCalibration(3, 3);
}

// ######################################################################
void EyeTrackerISCAN::recalibrate(nub::soft_ref<PsychoDisplay> d, int repeats)
{
  ++itsRecalibCount;
  if (itsRecalibCount == repeats)
    {
      itsRecalibCount = 0;
      d->clearScreen();
      d->displayText("Ready for quick recalibration");
      d->waitForKey(true);
      d->clearScreen();
      d->displayISCANcalib();
      d->waitForKey(true);
      d->displayEyeTrackerCalibration(3, 3);
      d->clearScreen();
      d->displayText("Ready to continue");
      d->waitForKey(true);
    }
}

// ######################################################################
void EyeTrackerISCAN::calibrateOnline(nub::soft_ref<PsychoDisplay> d)
{


        CalibrationTransform::Data pts;
        pts = getCalibrationSet(d);
        LINFO("got calibration set computing transform");
        itsAffine.computeTransform(pts);
}


// ######################################################################
void EyeTrackerISCAN::startTracking()
{
  if (itsParTrig.getVal())
    {
        if(itsRequestQuickEyeS)
        {
          LINFO("\n creating new Thread now");
            //clear the eyePosEvents buffer and create the thread
            itsEyePosEvents.resize(0);
            itsEyePosEvents.reserve(240 * 60 * 60);

            if (0 != pthread_create(&itsEyePosPollThread, NULL,
                                    &EyeTrackerISCAN::eyePosPollThread,
                                    (void*)(this)))
                LFATAL("Cannot create thread");

            // start tracker using parallel port:
            itsParPort->WriteData(255, 0); // turn all data bits to off
        }
        else
        {         // start tracker using parallel port:
          LINFO("requestEyeS was false");
            itsParPort->WriteData(255, 0); // turn all data bits to off
        }

    }
  else
    {
      // start tracker using serial port:
      LINFO("using serial port so screwed");
      char cmd[1]; cmd[0] = 132; // ISCAN start command
      itsSerial->write(cmd, 1);
    }
}

// ######################################################################
void EyeTrackerISCAN::stopTracking()
{
  if (itsParTrig.getVal())
    {
         if(itsRequestQuickEyeS)
        {

                //stop thread recording live eyePos
            if (0 != pthread_cancel(itsEyePosPollThread))
                LFATAL("pthread_cancel failed");

            if (0 != pthread_join(itsEyePosPollThread, NULL))
                LFATAL("pthread_join failed");

                // stop tracker using parallel port:
            itsParPort->WriteData(255, 255); // turn all data bits to on

             //#####################theEyeSFiledump##################/

                //TODO:dumpdata to eyeS file
                //iterate through eyePosevents
                //format time write to eyeData class use IO for eyeS


            if (itsEyePosEvents.empty() == false)
            {
                const char *fname = getCurrentStimFile().c_str();
                std::ofstream ofs(fname);
                if (!ofs.is_open())
                    LERROR("Couldn't open file '%s' for writing.", fname);
                else
                {
                    std::vector<EyePosEvent>::const_iterator itr = itsEyePosEvents.begin();

                    while (itr != itsEyePosEvents.end())
                    {
                        const uint64 t = itr->tim;
                        const int usec = int(t % 1000ULL);
                        const int msec = int((t / 1000ULL) % 1000ULL);
                        const int sec  = int((t / 1000000ULL) % 60ULL);
                        const int minu = int((t / 60000000ULL) % 60ULL);
                        const int hour = int(t / 3600000000ULL);
                        ofs << sformat("%03d:%02d:%02d.%03d.%03d",
                                       hour, minu, sec, msec, usec)
                            << " " << itr->pt.i << " "  << itr->pt.j << std::endl;
                        ++ itr;
                    }
                    ofs.close();
                    LINFO("Saved log to '%s'", fname);
                }
                                                                                                         }



            //################endfiledump######################


        }
         else
         {
            // stop tracker using parallel port:
            itsParPort->WriteData(255, 255); // turn all data bits to on
         }



    }
  else
    {
      // stop tracker using serial port:
      char cmd[1]; cmd[0] = 136; // ISCAN stop command
      itsSerial->write(cmd, 1);
    }
}

// ######################################################################
bool EyeTrackerISCAN::isFixating()
{
  LFATAL("Unimplemented for now");
  return false;
}

// ######################################################################
bool EyeTrackerISCAN::isSaccade()
{
  LFATAL("Unimplemented for now");
  return false;
}

// ######################################################################
Point2D<int> EyeTrackerISCAN::getEyePos() const
{
  /*if (!itsParTrig.getVal())
        LFATAL("must use parallel-port triggering");

    int val = itsCurrentRawEyePos.atomic_get();
        //   return itsCurrentRawEyePos.set(val & 0xfff,
        //                       (val & 0xfff000) >> 12);

    return Point2D<int>(val & 0xfff,(val & 0xfff000 >> 12));
  */

  unsigned char buf[256];
  int n;
  Point2D<int> eyePos(-1,-1);
  //prefix byte 0 = prefix byte 1 =  hex value = 0x44

  n =   itsSerial->read(buf,256);
  bool gotHeader =false;

  for(int i=0;i<n-1;i++)
    {

      if(gotHeader && n-i>5) //ensure enough data to read
        {
          eyePos.i = buf[i+2] <<8;
          eyePos.i += buf[i+1];
          eyePos.j = buf[i+4] <<8;
          eyePos.j += buf[i+3];
          break;

        }

      if(buf[i] == 0x44 && buf[i+1] == 0x44)
        gotHeader = true;
    }

      return eyePos;
}

// ######################################################################
Point2D<int> EyeTrackerISCAN::getFixationPos() const
{
  LFATAL("Unavailable on DML tracker, sorry.");
  return Point2D<int>(0, 0);
}

// ######################################################################
Point2D<int> EyeTrackerISCAN::getCalibEyePos()
{
         if (!itsParTrig.getVal())
             LFATAL("must use parallel-port triggering");

         int val = itsCurrentCalibEyePos.atomic_get();
             //return itsCurrentCalibEyePos.set(val & 0xfff,
             //                    (val & 0xfff000) >> 12);
         int val2 = itsCurrentRawEyePos.atomic_get();
         LINFO("eye pos at getRawEyePos %d,%d",val2 & 0xfff,(val2 & 0xfff000 >> 12));
         LINFO("Eye Pos at thread calib = (%d,%d)",val & 0xfff,(val & 0xfff000 >> 12));
         //return Point2D<int>(val & 0xfff,(val & 0xfff000 >> 12));
         return Point2D<int>(itsCurrentCalibEyePosX.atomic_get(),itsCurrentCalibEyePosY.atomic_get());

        //Point2D<double> rawPos = Point2D<double>(getEyePos());
        //return Point2D<int>(itsAffine.getCalibrated(rawPos));
}

// ######################################################################
CalibrationTransform::Data EyeTrackerISCAN::getCalibrationSet(nub::soft_ref<PsychoDisplay> d) const
{
    /*get the calibration data for online calibration*/

    LINFO("\n getting calibration set...");


     //start with simple test to see if someone is fixating then
    //say fixating with location

    int fixWindow = 75;
    std::vector<Point2D<int> > temp(fixWindow);
    std::vector<float> tempX(fixWindow);
    std::vector<float> tempY(fixWindow);
    float xMean,yMean,xStd,yStd,xVar,yVar;

    //you need a minimum of 4 calibration points to ensure inverse matrix
    //computations are legal

    const int nptshoriz=3, nptsvertic=3;
    const int npts = nptshoriz*nptsvertic;
    Image<double> displayImage(1920,1080,ZEROS);
    Dims dispDims = displayImage.getDims();
    int w=dispDims.w(), h = dispDims.h();


    int deltax = w / (nptshoriz + 1), deltay = h / (nptsvertic + 1);

    // list all the points we want:
    std::vector<Point2D<int> > pts;
    for (int j = deltay-1; j < h - deltay; j += deltay)
      for (int i = deltax-1; i < w - deltax; i += deltax)
        pts.push_back(Point2D<int>(i, j));

    Point2D<int> tempPt,tempCalibPt;
    std::vector<Point2D<int> > eyeMeans(npts);
    stats<float> Stats;
    bool fixated;
    d->clearScreen();
    char tmp[50];
    bool happy=false; //happy with calib or not
    double diffX=0.0F,diffY=0.0F;
    AffineTransform tempAffine;
    CalibrationTransform::Data finalPts;


    while(!happy)
    {
      //startTracking();
      //event log code marked as EL-code eventually should be removed
      //EL-code
      d->pushEventBegin("EyeTrackerCalibration");
       //~EL-code
      CalibrationTransform::Data firstFix;

        for (int j=0;j<npts;j++)
        {
                d->clearScreen();
                tempPt = pts[j];
                //d->drawCalibPoint(tempPt);
                //rintf(tmp,"press key when ready");
                d->displayFixation();
                d->waitForKey();
                d->displayFixationBlink();
                d->clearScreen();
                fixated = false;
                d->drawCalibPoint(tempPt);
                //EL-code
                d->pushEventBegin(sformat("eyeTrackerCalibration at (%d, %d)", tempPt.i, tempPt.j));
                       while(fixated==false)
                {


                           for(int i=0; i < fixWindow; i++)
                            {
                                temp[i] = getEyePos();
                                if((temp[i].i < 0) || (temp[i].j<0))
                                  i--;
                                else
                                {
                                  LINFO("I got eyePos %d,%d",temp[i].i, temp[i].j);
                                  tempX[i] = temp[i].i;
                                  tempY[i] = temp[i].j;
                                }
                            }

                               xMean = Stats.mean(tempX);
                            yMean = Stats.mean(tempY);
                               xStd = Stats.findS(tempX,xMean);
                            xVar = Stats.S2;
                            yStd = Stats.findS(tempY,yMean);
                              yVar = Stats.S2;


                        if((xVar < 5.0) && (yVar < 5.0) && (xMean > 0) && (yMean >0))
                        {
                                //EL-code
                                d->pushEventEnd(sformat("eyeTrackerCalibration at (%d, %d)", tempPt.i, tempPt.j));
                                fixated = true;
                                 d->displayText("Found stable fixation");
                                LINFO("####Calibration Point %d #######",j+1);
                                LINFO("found fixation varX = %f, var Y =%f....meanX = %f meanY = %f ",xStd,yStd,xMean,yMean);
                                LINFO("raw (%f,%f) scr(%d,%d)",xMean,yMean,pts[j].i,pts[j].j);
                                eyeMeans[j] = Point2D<int>((int)xMean,(int)yMean);
                                firstFix.addData(Point2D<double>(xMean,yMean),Point2D<double>(tempPt));
                        }

                    }
                    }
        //lets display a grid show original screen points in red squares and calib pts
        //in blue
        //EL-Code
        d->pushEventEnd("EyeTrackerCalibration");
        //    stopTracking();
        tempAffine.computeTransform(firstFix);
        for (int j=0;j<npts;j++)
        {
                tempPt = pts[j];
                d->drawCalibPoint(tempPt);
                tempCalibPt = Point2D<int>(tempAffine.getCalibrated(Point2D<double>(eyeMeans[j])));
                d->drawPointColor(tempCalibPt,PixRGB<byte>(0,0,255));
                diffX += abs(tempPt.i - tempCalibPt.i);
                diffY += abs(tempPt.j - tempCalibPt.j);
                LINFO("raw(%d,%d),calib(%d,%d)",tempPt.i,tempPt.j,tempCalibPt.i,tempCalibPt.j);

        }
        d->waitForKey();
        sprintf(tmp,"avg X difference = %f, avg Y difference = %f \n--press 5  to repeat Calibration",diffX/npts,diffY/npts);
        d->displayText(tmp);
        int c=d->waitForKey(true);
        if(c != '5')
        {
                happy = true;
                finalPts= firstFix;
                LINFO("setfirstfix");

        }
     }

    d->clearScreen();
    d->displayText("Calibrated!");
    LINFO("calibration ended");
    return finalPts;
}


// ####################################################################
void EyeTrackerISCAN::requestQuickEyeS()
{
    itsRequestQuickEyeS=true;
}

//#####################################################################


void* EyeTrackerISCAN::eyePosPollThread(void* p)
{
  EyeTrackerISCAN* et = static_cast<EyeTrackerISCAN*>(p);

  Timer timer(1000000);

  timer.reset();

  while (true)
    {
      // read one sample from the serial port
      Point2D<int> raw;

          //###############serial reading code###################/
      unsigned char buf[256];
      int n;
      //prefix byte 0 = prefix byte 1 =  hex value = 0x44

      n =   et->itsSerial->read(buf,256);
      bool gotHeader =false;

      for(int i=0;i<n-1;i++)
      {

          if(gotHeader && n-i>5) //ensure enough data to read
          {
              raw.i = buf[i+2] <<8;
              raw.i += buf[i+1];
              raw.j = buf[i+4] <<8;
              raw.j += buf[i+3];
              break;
          }

          if(buf[i] == 0x44 && buf[i+1] == 0x44)
              gotHeader = true;
      }


      EyePosEvent ev;
      ev.tim = timer.get();
      ev.pt = Point2D<int16>(et->itsAffine.getCalibrated(Point2D<double>(raw)));
      //LINFO("Eye Pos at thread raw = (%d,%d) calib=(%d,%d)",raw.i,raw.j, ev.pt.i,ev.pt.j);
      et->itsEyePosEvents.push_back(ev);
      et->itsCurrentRawEyePos.atomic_set(raw.i + (raw.j << 12));
      et->itsCurrentCalibEyePos.atomic_set(ev.pt.i + (ev.pt.j << 12));
      et->itsCurrentCalibEyePosX.atomic_set(ev.pt.i);
      et->itsCurrentCalibEyePosY.atomic_set(ev.pt.j);
    }

  return NULL;
}

// #####################################################################


/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_EYETRACKERISCAN_C_DEFINED

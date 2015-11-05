/*!@file Robots2/Beobot2/Hardware/BeoLRF.C Ice Module for an LRF        */
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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Hardware/BeoLRF.C
// $ $Id: BeoLRF.C 15336 2012-07-19 21:26:31Z kai $
//
//////////////////////////////////////////////////////////////////////////

#include "Robots/Beobot2/Hardware/BeoLRF.H"
#define IMG_WIDTH  512  //1024
#define IMG_HEIGHT 512
#define REPORTING_INTERVAL 100

// filtering constant to combat sunlight interference
#define USE_FILTERING          true
#define MAXIMUM_FILTERED_RANGE 500 //mm
#define MAX_FILTERED_DIFF      100 //mm

const ModelOptionCateg MOC_BeoLRF = {
        MOC_SORTPRI_3, "Beobot LRF Related Options" };

const ModelOptionDef OPT_Serial=
{ MODOPT_ARG(std::string), "device", &MOC_BeoLRF, OPTEXP_CORE,
        "device path. default is /dev/ttyACM0",
        "device", '\0', "string", "/dev/ttyACM0"};

// identification label for the different LRF devices
const ModelOptionDef OPT_DeviceLabel=
{ MODOPT_ARG(int), "id", &MOC_BeoLRF, OPTEXP_CORE,
  "When using multiple LRF,we can label them differently "
  "such as id=0 for top laser,id=1 for cliff laser",
  "id", '\0', "int", "0"};

// turn on/off GUI
const ModelOptionDef OPT_GUI=
{ MODOPT_ARG(bool), "gui", &MOC_BeoLRF, OPTEXP_CORE,
        "Turn On/Off GUI ",
        "gui", '\0', "bool", "true"};

// ######################################################################
BeoLRF::BeoLRF(OptionManager& mgr,
               const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsOfs(new OutputFrameSeries(mgr)),
  itsTimer(1000000),
  itsDrawImage(IMG_WIDTH,IMG_HEIGHT,ZEROS),
  itsCurrMessageID(0),
  itsDevice(&OPT_Serial, this, 0),
  itsDeviceLabel(&OPT_DeviceLabel, this, 0),
  itsGUIstatus(&OPT_GUI, this, 0)
{
  addSubComponent(itsOfs);
  itsTimer.reset();

  itsPreviousTime = 0.0f;

  itsRecentTimes.clear();
  itsRecentTimes.resize(REPORTING_INTERVAL);

  itsPreviousRawLRFreading.clear();
}

// ######################################################################
void BeoLRF::setDrawImage(bool onoff)
{
  itsGUIstatus.setVal(onoff);
}

// ######################################################################
BeoLRF::~BeoLRF()
{ }

// ######################################################################
void BeoLRF::registerTopics()
{
  this->registerPublisher("LRFMessageTopic");
}

// ######################################################################
void BeoLRF::start3()
{
  itsLRF = new lobot::LaserRangeFinder(itsDevice.getVal(),115200);
}

// ######################################################################
void BeoLRF::evolve()
{
  Image<int>::iterator aptr, stop;
  int dist; int  min,max; int angle;

  itsLRF->update();
  itsDists = itsLRF->get_distances();
  aptr = itsDists.beginw();
  stop = itsDists.endw();
  //LDEBUG("dims= %d", itsDists.getDims().w());

  // some scaling
  getMinMax(itsDists, min, max);
  if (max == min) max = min + 1;

  // LRFMessage
  BeobotEvents::LRFMessagePtr msg = new BeobotEvents::LRFMessage;

  msg->RequestID = itsCurrMessageID;
  msg->distances.resize(itsDists.getDims().w());
  msg->angles.resize(itsDists.getDims().w());

  msg->LRFID = itsDeviceLabel.getVal();
  angle = -141;  int count = 0;

  // initial step
  bool init_step = false;
  if(itsPreviousRawLRFreading.size() == 0) init_step = true;



  //char buffer[255]; 
  itsDrawImage = Image<PixRGB<byte> >(IMG_WIDTH,IMG_HEIGHT,ZEROS);
  while(aptr!=stop)
    {
      dist = *aptr++;

      // check for initial step
      if(init_step) itsPreviousRawLRFreading.push_back(dist);

      // simple temporal filtering  
      double filtered_dist = dist;
      if(USE_FILTERING && !init_step && 
         dist < MAXIMUM_FILTERED_RANGE &&
         (abs(itsPreviousRawLRFreading[count] - dist) > MAX_FILTERED_DIFF)) 
        {
          filtered_dist = -1;
          // if(dist != -1 && (angle >= -90 && angle <= 90))
          //   LINFO("filtering[%3d]: c: %5d vs p: %5d", angle, dist, 
          //         itsPreviousRawLRFreading[count]);
        }

      itsPreviousRawLRFreading[count] = dist;

      msg->distances[count] = filtered_dist;
      msg->angles   [count] = angle;

      // note drawing takes up a lot of time
      if(itsGUIstatus.getVal() || itsCurrMessageID%REPORTING_INTERVAL == 0)
        {
          float rad = dist; 
          if(itsDeviceLabel.getVal()==1)
            rad = ((rad - min)/(2100-min))*IMG_HEIGHT/2;//fixed size
          else
            rad = ((rad - min)/(max-min))*IMG_HEIGHT/2;//float size
          if (rad < 0) rad = 1.0;

          Point2D<int> pt;
          pt.i = IMG_WIDTH/2  - int(rad*sin((double)angle*M_PI/180.0));
          pt.j = IMG_HEIGHT/2 - int(rad*cos((double)angle*M_PI/180.0));

          drawLine(itsDrawImage, Point2D<int>(IMG_WIDTH/2,IMG_HEIGHT/2),
                   pt,PixRGB<byte>(0,255,0));
          drawCircle(itsDrawImage, pt, 2, PixRGB<byte>(255,0,0));
        }

      //LINFO("[%4d] <%4d>: %13d mm", count, angle, dist);
      angle++; count++;
    }

  if(itsGUIstatus.getVal() || itsCurrMessageID%REPORTING_INTERVAL == 0)
    itsOfs->writeRGB(itsDrawImage,"Output",FrameInfo("output",SRC_POS));

  if(itsCurrMessageID%REPORTING_INTERVAL == 0)
    LINFO("[%6d]Publishing LRF[%d]Message  time: %f",
          itsCurrMessageID, itsDeviceLabel.getVal(),itsTimer.get()/1000.0F);
  publish("LRFMessageTopic", msg);

  float ctime = itsTimer.get()/1000.0f;
  float time  = ctime - itsPreviousTime;
  itsPreviousTime = ctime;
  itsRecentTimes[itsCurrMessageID%REPORTING_INTERVAL] = time;
  if(itsCurrMessageID > 0 && 
     itsCurrMessageID%REPORTING_INTERVAL == 0)
    {
      float sum = 0.0f;
      for(uint i = 0; i < REPORTING_INTERVAL; i++)
        sum += itsRecentTimes[i];
      float atime = sum/REPORTING_INTERVAL;
      itsFrameRate = 1000.0f/atime;

      LINFO("[%6d] Avg. time: %10.4f --> frate: %8.5f", 
            itsCurrMessageID, atime, itsFrameRate);
    }

  itsCurrMessageID++;
}

// ######################################################################
void BeoLRF::drawRobotBody(Point2D<int>pt,int count,int dist)
{
	
  //char buffer[10];
  int lrfToBodyOffset = 20;
  //From center of lrf to front panel of beobot 20pixel , 580mm = 70 pixel
  int beobotFront = IMG_HEIGHT/2 - lrfToBodyOffset;
  int leftWheel = 475;
  int rightWheel = 555;
  bool stopMotor = false;

  drawRect
    (itsDrawImage, 
     Rectangle(Point2D<int>(leftWheel,beobotFront), 
               Dims(rightWheel-leftWheel, 80)), PixRGB<byte>(255,128,0),3);//Beobot Body

  if((pt.i > leftWheel && pt.i < rightWheel) &&
     ((count >= (100)) && (count <= (180))))
    //((count >= (116)) && (count <= (163))))
    {
      drawLine(itsDrawImage, Point2D<int>(IMG_WIDTH/2,IMG_HEIGHT/2),
               pt,PixRGB<byte>(0,0,255));
      if(dist != -1)
        {
          //LINFO("[%4d] <%4d>: %13d mm (%3d,%3d)pixel", count, angle, dist,pt.i,pt.j);
          if(abs(pt.j-177) > 5)
            stopMotor = true;
        }    
      
    }
  else
    {
      drawLine(itsDrawImage, Point2D<int>(IMG_WIDTH/2,IMG_HEIGHT/2),
               pt,PixRGB<byte>(0,255,0));
    }

  if(stopMotor)
    {
      drawFilledRect
        (itsDrawImage, Rectangle(Point2D<int>(leftWheel,beobotFront), 
                                 Dims(rightWheel-leftWheel, 80)), 
         PixRGB<byte>(255,0,0));//Draw Stop Sign
      writeText
        (itsDrawImage, Point2D<int>(leftWheel+20,beobotFront+30), "STOP", 
         PixRGB<byte>(0,255,0), PixRGB<byte>(255,0,0),SimpleFont::FIXED(8));
    }
	
}
// ######################################################################
void BeoLRF::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{ }

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

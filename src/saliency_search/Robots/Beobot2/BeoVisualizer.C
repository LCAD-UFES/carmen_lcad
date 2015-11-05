/*!@file Robots2/Beobot2/Hardware/BeoVisualizer.C Diaply all module*/
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
// Primary maintainer for this file: Chin-Kai Chang<chinkaic@usc.edu>
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Service/BeoVisualizer.C
// $ $Id: BeoVisualizer.C 15242 2012-03-30 11:15:37Z kai $
//
//////////////////////////////////////////////////////////////////////////

#include "Robots/Beobot2/BeoVisualizer.H"
#include "Robots/Beobot2/BeoCommon.H"
#include "Ice/BeobotEvents.ice.H"

#include "Raster/Raster.H"
#include "Util/sformat.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "Ice/IceImageUtils.H"
#include "Image/JPEGUtil.H"
#include <sys/stat.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#define  LOG_FOLDER "../data/logs/"
#define  REPORTING_INTERVAL     100
#define IMG_WIDTH 1920
#define IMG_HEIGHT 1080
// ######################################################################
/*
The default layout is 1080p(1920x1080)

                              1280             1920
+-------------------------------+---------------+
|                               |               |
|                               |    Global     |
|                               |   Localizer   |
|                               |               |
|           HandyCam            |               |
|                               +---------------+ 440
|                               |               |
|                               |               |
|                               |               |
+----------------+--------------+   Navigator   | 720
|                |              |               |
|   RoadFinder   |   Saliency   |               |
|                |    Match     |               |
|                |              |               |
+----------------+--------------+---------------+ 1080
               720            1280
*/
BeoVisualizer::BeoVisualizer(OptionManager& mgr,
               const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsTimer(1000000),
  itsCurrMessageID(0),
  itsOfs(new OutputFrameSeries(mgr))
{
  addSubComponent(itsOfs);

  itsDispImage   = Image<PixRGB<byte> >(IMG_WIDTH,IMG_HEIGHT,ZEROS);
  itsHandycam  = Panel(Point2D<int>(0,0) ,Dims(1280,720), "HandyCam");
  itsRoadFinder= Panel(itsHandycam.bl()  ,Dims(1280,360), "RoadFinder");
  itsImageMatch= Panel(itsRoadFinder.tr(),Dims(560 ,360), "ImageMatch");
  itsLocalizer = Panel(itsHandycam.tr()  ,Dims(640 ,440), "Global Map");
  itsNavigator = Panel(itsLocalizer.bl() ,Dims(640 ,640), "Local Map");
  itsPilot= Panel(itsLocalizer.bl() ,Dims(640 ,640), "Pilot");
  itsSim = Panel(itsLocalizer.bl() ,Dims(640 ,400), "Pilot");
  

  //fake image just for display test FIXXXXXX
  //Image<PixRGB<byte> > fakeHandyCamImg = Raster::ReadRGB("HandyCam.png");
  //itsHandycam.updateImage(fakeHandyCamImg);
}

// ######################################################################
BeoVisualizer::~BeoVisualizer()
{ 
}


// ######################################################################
void BeoVisualizer::start1()
{
  //initLogFile();
}

// ######################################################################
void BeoVisualizer::start3()
{
  // set start time
  itsTimer.reset();
}

// ######################################################################
void BeoVisualizer::registerTopics()
{
  // subscribe to all sensor data
  this->registerSubscription("CameraMessageTopic");
  this->registerSubscription("BlankCameraMessageTopic");
  this->registerSubscription("IMUMessageTopic");
  this->registerSubscription("LRFMessageTopic");
  this->registerSubscription("GPSMessageTopic");
  this->registerSubscription("MotorMessageTopic");
  this->registerSubscription("CurrentLocationMessageTopic");
  this->registerSubscription("LogFolderNameRequestTopic");
  this->registerSubscription("VisualizerMessageTopic");

}
// ######################################################################
void BeoVisualizer::evolve()
{

  itsHandycam.draw(itsDispImage);  
  itsRoadFinder.draw(itsDispImage);
  //itsImageMatch.draw(itsDispImage);             
  itsLocalizer.draw(itsDispImage);
  //itsNavigator.draw(itsDispImage);
  itsSim.draw(itsDispImage,false);

  // display the image
  itsOfs->writeRGB(itsDispImage, "BeoVisualizer");
  itsOfs->updateNext();
  itsCurrMessageID++;
}
// ######################################################################
void BeoVisualizer::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{
  // record the time
  uint64 time = itsTimer.get();

  if(eMsg->ice_isA("::BeobotEvents::VisualizerMessage"))
  {
    // store the image
    BeobotEvents::VisualizerMessagePtr visualMsg =
      BeobotEvents::VisualizerMessagePtr::dynamicCast(eMsg);

    int currRequestID        = visualMsg->RequestID;
    int beoAppID             = visualMsg->BeoAppID;
    Image<PixRGB<byte> > ima = Ice2Image<PixRGB<byte> >(visualMsg->image);
		// write frame id on the image
		//char buffer[255];
		//sprintf(buffer, "[%5d]", currRequestID);
		//writeText(ima, Point2D<int>(0,0), buffer, 
		//		PixRGB<byte>(255,255,255), 
		//		PixRGB<byte>(0,0,0),
		//		SimpleFont::FIXED(6));

    LINFO("Got Image from %d",beoAppID);
    switch(beoAppID)
    {
      case BEO_LOCALIZER:
        itsLocalizer.updateImage(ima);
        break;
      case BEO_NAVIGATOR:
        itsNavigator.updateImage(ima);
        break;
      case BEO_ROADFINDER:
        itsRoadFinder.updateImage(ima);
        break;
      case BEO_SIM:
        itsSim.updateImage(ima);
        break;
    }


  }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

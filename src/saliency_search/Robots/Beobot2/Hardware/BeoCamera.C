/*!@file Robots2/Beobot2/Hardware/BeoCamera.C Ice Module for sending out
  images from a camera                                                  */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Hardware/BeoCamera.C
// $ $Id: BeoCamera.C 15431 2012-11-06 05:17:38Z kai $
//
//////////////////////////////////////////////////////////////////////////

#include "Robots/Beobot2/Hardware/BeoCamera.H"
#include "Ice/BeobotEvents.ice.H"
#include "Ice/IceImageUtils.H"
#include "Raster/Raster.H"
#include "Image/DrawOps.H"
#include "Image/Point3D.H"
#include "Image/ShapeOps.H"

#include "GUI/XWinManaged.H"
#include "GUI/ImageDisplayStream.H"
#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"

#include "Devices/FrameGrabberConfigurator.H"
#include "Raster/GenericFrame.H"
#include "Transport/FrameIstream.H"


#include "Util/sformat.H"
#include <sys/stat.h>


#define FRAMERATE_INTERVAL        100
#define FRAMERATE_CALC_INTERVAL   20
#define LOG_FOLDER                 "../data/logs/"
#define NONE_FOLDER_NAME           "NONE"

const ModelOptionCateg MOC_BeoCamera = {
        MOC_SORTPRI_3, "Beobot Camera Related Options" };

const ModelOptionDef OPT_DisplayFps =
{ MODOPT_ARG(float), "Display Refresh Rate", &MOC_BeoCamera, OPTEXP_CORE,
  "When network speed is not fast enough to display camera image "
  "in real time, lower the display fps to reduce display load "
  "This option does not change publish rate (30fps)",
  "fps", '\0', "float", "30.0"};

// identification label for the different camera devices
const ModelOptionDef OPT_DeviceLabel =
{ MODOPT_ARG(int), "id", &MOC_BeoCamera, OPTEXP_CORE,
  "When using multiple cameras, we can label them differently ",
  "id", '\0', "int", "0"};

// turn on/off GUI
const ModelOptionDef OPT_LogMode =
{ MODOPT_ARG(bool), "gui", &MOC_BeoCamera, OPTEXP_CORE,
  "Turn On/Off Log Mode ", "log-mode", '\0', "bool", "false"};

// foldername location: initialize to NONE
// it has to be specified to start saving
const ModelOptionDef OPT_LogFolderName =
{ MODOPT_ARG(string), "logfoldername", &MOC_BeoCamera, OPTEXP_CORE,
  "folder to save the file to ",
  "logfolder-name", '\0', "<string>", NONE_FOLDER_NAME};

// ######################################################################
BeoCamera::BeoCamera(OptionManager& mgr,
               const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsIfs(new InputFrameSeries(mgr)),
  itsOfs(new OutputFrameSeries(mgr)),
  itsTimer(1000000),
  itsDisplayTimer(1000000),
  itsCurrMessageID(0),
  itsDisplayFps(&OPT_DisplayFps,this,0),
  itsDeviceLabel(&OPT_DeviceLabel, this, 0),
  itsLogMode(&OPT_LogMode, this, 0),
  itsLogFolderName(&OPT_LogFolderName, this, 0)
{
  addSubComponent(itsIfs);
  addSubComponent(itsOfs);
  itsTimer.reset();
  itsDisplayTimer.reset();

  itsPrevDisplayMessageID = -1;
  itsPreviousTime = 0.0f;

  itsRecentTimes.clear();
  itsRecentTimes.resize(FRAMERATE_INTERVAL);
  
  // initial frame rate is invalid
  itsFrameRate = -1.0F;

  // prepare a gamma table for RGBD displays (e.g., Kinect grabber):
  itsGamma.clear(); itsGamma.resize(2048);
  for (int i = 0; i < 2048; ++i) {
    float v = i/2048.0;
    v = powf(v, 3)* 6;
    itsGamma[i] = v*6*256;
  }
}


// ######################################################################
bool BeoCamera::initLogFolder()
{
  // get the time of day
  time_t rawtime; struct tm * timeinfo;
  time ( &rawtime );
  timeinfo = localtime ( &rawtime );
  char buffer [80];
  strftime (buffer,80,
            "%Y_%m_%d__%H_%M_%S",timeinfo);
  std::string startTime(buffer);

  itsLogFolderName.setVal
    (std::string(sformat("%s%s_%d", LOG_FOLDER, startTime.c_str(),
                         itsDeviceLabel.getVal())));
  LINFO("logFolderName: %s", itsLogFolderName.getVal().c_str());

  // create a log directory
  if (mkdir(itsLogFolderName.getVal().c_str(), 0777) == -1)
    {
      LFATAL("Cannot create log folder: %s", 
             itsLogFolderName.getVal().c_str());
      return(EXIT_FAILURE);
    }

  return true;
}

// ######################################################################
BeoCamera::~BeoCamera()
{ }

// ######################################################################
void BeoCamera::registerTopics()
{
  this->registerPublisher("CameraMessageTopic");
  this->registerPublisher("BlankCameraMessageTopic");
  this->registerPublisher("ColorDepthCameraMessageTopic");

  this->registerPublisher("LogFolderNameRequestTopic");

  // send out the log name
  this->registerSubscription("LogFolderNameMessageTopic");  
}

// ######################################################################
void BeoCamera::start3()
{
  if(itsLogMode.getVal())
    {
      itsSaveImage  = true; itsSendImage  = false;
      itsDisplayFps.setVal(0.3); // once every 100 frames;

      BeobotEvents::LogFolderNameRequestPtr msg =
        new BeobotEvents::LogFolderNameRequest;
      
      // store the input image and related info
      msg->RequestID  = 0;
      
      // publish the message
      LINFO("[%6d] Publishing Log Folder Name Request", itsCurrMessageID);
      publish("LogFolderNameRequestTopic", msg);
    }
  else
    { itsSaveImage  = false; itsSendImage  = true; }
  itsDisplayPeriod = 1000.0f / itsDisplayFps.getVal();
}

// ######################################################################
void BeoCamera::evolve()
{
  usleep(20000);

  itsIfs->updateNext(); 
  GenericFrame fr = itsIfs->readFrame();
  GenericFrameSpec fspec = itsIfs->peekFrameSpec();
  bool isRGBD = fspec.nativeType == GenericFrame::RGBD;

  Image<PixRGB<byte> > ima = fr.asRgbU8();

  Image<uint16> depthIma;
  if (isRGBD) depthIma = fr.asGrayU16(); // get the depth image

  if(itsLogMode.getVal())
    {
      itsSaveImage  = true; itsSendImage  = false;
      if (!isRGBD) itsDisplayFps.setVal(0.3); // once every 100 frames;
    }
  else
    { itsSaveImage  = false; itsSendImage  = true; }
  itsDisplayPeriod = 1000.0f / itsDisplayFps.getVal();

  if(ima.initialized())
    {
      // if we are sending images
      if(itsSendImage)
        {
          // prepare the camera message

          // color+depth camera
          if(isRGBD)
            {
              BeobotEvents::ColorDepthCameraMessagePtr msg =
                new BeobotEvents::ColorDepthCameraMessage;
     
              // store the input image and related info
              msg->cameraID   = itsDeviceLabel.getVal();
              msg->RequestID  = itsCurrMessageID;
              msg->colorImage = Image2Ice(ima);
              msg->depthImage = Image2Ice(depthIma); 
     
              // publish the message
              LDEBUG("[%6d] Publishing Color+Depth Camera Message", 
                     itsCurrMessageID);
              publish("ColorDepthCameraMessageTopic", msg);
            }
          else // regular camera
            {
              BeobotEvents::CameraMessagePtr msg =
                new BeobotEvents::CameraMessage;
     
              // store the input image and related info
              msg->cameraID  = itsDeviceLabel.getVal();
              msg->RequestID = itsCurrMessageID;
              msg->image     = Image2Ice(ima);
     
              // publish the message
              LDEBUG("[%6d] Publishing Camera Message", itsCurrMessageID);
              publish("CameraMessageTopic", msg);
            }
        }
      else
        {
          // store only the image info
          BeobotEvents::BlankCameraMessagePtr bmsg =
            new BeobotEvents::BlankCameraMessage;
          bmsg->cameraID  = itsDeviceLabel.getVal();
          bmsg->RequestID = itsCurrMessageID;
          bmsg->cameraType= isRGBD? RGBD : RGB;

          // publish the message
          LDEBUG("[%6d] Publishing Blank Camera Message", itsCurrMessageID);
          publish("BlankCameraMessageTopic", bmsg);
        }

      // if we are saving images
      if(itsSaveImage)
        {
          bool logFolderSet = true;
          if(!itsLogFolderName.getVal().compare(std::string(NONE_FOLDER_NAME)))
            {
              logFolderSet = false;
              LDEBUG("Log Folder name not yet specified; "
                     "not currently saving");
            }

          its_logFolderName_mutex.lock();
          std::string logFolderName = itsLogFolderName.getVal();          
          its_logFolderName_mutex.unlock();

          // saved file name
          std::string saveFName
            (sformat("%s/image_%03d_%06d.ppm",
                     logFolderName.c_str(), 
                     itsDeviceLabel.getVal(), itsCurrMessageID));

          if(logFolderSet)
            {
              if(itsCurrMessageID%FRAMERATE_CALC_INTERVAL == 0)
                LINFO("Saving: %s",saveFName.c_str());
              else
                LDEBUG("Saving: %s",saveFName.c_str());
              Raster::WriteRGB(ima, saveFName);          

              // save the depth file as well
              if(isRGBD)
                {
                  // saved file name
                  std::string saveDFName
                    (sformat("%s/depth_image_%03d_%06d.pgm",
                             logFolderName.c_str(), 
                             itsDeviceLabel.getVal(), itsCurrMessageID));
                  Raster::WriteFrame(GenericFrame(depthIma), saveDFName); //save as 16-bit pgm         
                }
            }
        }

      float displayDuration = itsDisplayTimer.get()/ 1000.0f;       

      // display protocol
      if((displayDuration) > itsDisplayPeriod)
        {
          itsPrevDisplayMessageID = itsCurrMessageID;
          itsDisplayTimer.reset();

          Image<PixRGB<byte> > dispIma = ima; 
          if (isRGBD) 
            {    
              Image<PixRGB<byte> > d(depthIma.getDims(), NO_INIT);
              const int sz = depthIma.size();
              for (int i = 0; i < sz; ++i) {
                uint v = depthIma.getVal(i); if (v > 2047) v = 2047;
                int pval = itsGamma[v];
                int lb = pval & 0xff;
                switch (pval>>8) {
                case 0: d.setVal(i, PixRGB<byte>(255, 255-lb, 255-lb)); break;
                case 1: d.setVal(i, PixRGB<byte>(255, lb, 0)); break;
                case 2: d.setVal(i, PixRGB<byte>(255-lb, 255, 0)); break;
                case 3: d.setVal(i, PixRGB<byte>(0, 255, lb)); break;
                case 4: d.setVal(i, PixRGB<byte>(0, 255-lb, 255)); break;
                case 5: d.setVal(i, PixRGB<byte>(0, 0, 255-lb)); break;
                default: d.setVal(i, PixRGB<byte>(0, 0, 0)); break;
                }
              }
              dispIma = concatX(dispIma, d);
            }

          // write frame id on the image
          char buffer[255];
          sprintf(buffer, "[%5d]", itsCurrMessageID);
          writeText(dispIma, Point2D<int>(0,0), buffer, 
                    PixRGB<byte>(255,255,255), 
                    PixRGB<byte>(0,0,0),
                    SimpleFont::FIXED(6));

          // write display fps on the image
          sprintf(buffer, "[%5.2ffps]", itsFrameRate);
          writeText(dispIma, Point2D<int>(ima.getWidth()-strlen(buffer)*6,0), 
                    buffer, 
                    PixRGB<byte>(255,255,255), 
                    PixRGB<byte>(0,0,0),
                    SimpleFont::FIXED(6));

          // display the image
          itsOfs->writeRGB(dispIma, "display");
          itsOfs->updateNext();
        }

      float ctime = itsTimer.get()/1000.0f;
      float time  = ctime - itsPreviousTime;
      itsPreviousTime = ctime;
      itsRecentTimes[itsCurrMessageID%FRAMERATE_INTERVAL] = time;
      if(itsCurrMessageID >= FRAMERATE_INTERVAL && 
         itsCurrMessageID%FRAMERATE_CALC_INTERVAL == 0)
        {
          float sum = 0.0f;
          for(uint i = 0; i < FRAMERATE_INTERVAL; i++)
            sum += itsRecentTimes[i];
          float atime = sum/FRAMERATE_INTERVAL;
          itsFrameRate = 1000.0f/atime;

          LINFO("[%6d] Avg. time: %10.4f --> frate: %8.5f", 
                itsCurrMessageID, atime, itsFrameRate);
        }
      
      itsCurrMessageID++;  

      //Raster::waitForKey();
    }
}

// ######################################################################
void BeoCamera::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{ 
  // camera message 
  if(eMsg->ice_isA("::BeobotEvents::LogFolderNameMessage"))
  {
    // store the image
    BeobotEvents::LogFolderNameMessagePtr logfnMsg =
      BeobotEvents::LogFolderNameMessagePtr::dynamicCast(eMsg);

    int currRequestID         = logfnMsg->RequestID;
    std::string logFolderName = logfnMsg->logFolderName;

    its_logFolderName_mutex.lock();
    itsLogFolderName.setVal(logFolderName);
    its_logFolderName_mutex.unlock();

    LINFO("Got a LogFolderNameMessage '%s' with Request ID = %d", 
          logFolderName.c_str(), currRequestID);

    // if we are not saving images just return
    if(!itsLogMode.getVal() ||
       (!logFolderName.compare(std::string(NONE_FOLDER_NAME)))) 
      return;

    // create a log directory
    if (mkdir(logFolderName.c_str(), 0777) == -1)
      {
        LINFO("Cannot create log folder: %s", logFolderName.c_str());
      }
    else
      LINFO("Create log folder: %s", logFolderName.c_str());
  }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

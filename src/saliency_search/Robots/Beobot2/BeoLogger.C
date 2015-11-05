/*!@file Robots2/Beobot2/Hardware/BeoLogger.C Ice Module to log data    */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/BeoLogger.C
// $ $Id: BeoLogger.C 15441 2012-11-14 21:28:03Z kai $
//
//////////////////////////////////////////////////////////////////////////

#include "Robots/Beobot2/BeoLogger.H"
#include "Ice/BeobotEvents.ice.H"

#include "Raster/GenericFrame.H"
#include "Raster/Raster.H"
#include "Util/sformat.H"
#include "Image/Image.H"
#include "Ice/IceImageUtils.H"

#include <sys/stat.h>

#define  LOG_FOLDER "../data/logs/"
#define  REPORTING_INTERVAL     100

// ######################################################################
BeoLogger::BeoLogger(OptionManager& mgr,
               const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsTimer(1000000)
  //  itsOfs(new OutputFrameSeries(mgr))
{
  //  addSubComponent(itsOfs);

}

// ######################################################################
BeoLogger::~BeoLogger()
{ 
  // send the log folder of where to save images
  BeobotEvents::LogFolderNameMessagePtr msg =
    new BeobotEvents::LogFolderNameMessage;
  
  // store the input image and related info
  msg->RequestID     = 0;
  msg->logFolderName = std::string("NONE");
  
  // publish the message
  LINFO("[%6d] Publishing Log Folder Name Message: NONE", 0);
  publish("LogFolderNameMessageTopic", msg);
}


// ######################################################################
void BeoLogger::start1()
{
  initLogFile();
}

// ######################################################################
void BeoLogger::start3()
{
  broadcastLogFileName();

  // set start time
  itsTimer.reset();
}

// ######################################################################
bool BeoLogger::initLogFile()
{
  // get the time of day
  time_t rawtime; struct tm * timeinfo;
  time ( &rawtime );
  timeinfo = localtime ( &rawtime );
  char buffer [80];
  strftime (buffer,80,
            "%Y_%m_%d__%H_%M_%S",timeinfo);
  std::string startTime(buffer);

  itsLogFolderName =
    std::string(sformat("%s%s", LOG_FOLDER, startTime.c_str()));
  LINFO("logFoldername: %s", itsLogFolderName.c_str());

  // create a log directory
  if (mkdir(itsLogFolderName.c_str(), 0777) == -1)
    {
      LFATAL("Cannot create log folder: %s", itsLogFolderName.c_str());
      return(EXIT_FAILURE);
    }

  std::string logFilename
    (sformat("%s/Log_%s.txt", itsLogFolderName.c_str(), startTime.c_str()));
  LINFO("logFilename: %s", logFilename.c_str());

  std::string cTime = std::string("Time of day: ") + startTime;
  LINFO("%s", cTime.c_str());
  cTime += std::string("\n");

  // save  in a file by appending to the file
  itsLogFilename = logFilename;
  FILE *rFile = fopen(itsLogFilename.c_str(), "at");
  if (rFile != NULL)
    {
      LDEBUG("saving result to %s", logFilename.c_str());
      fputs(cTime.c_str(), rFile);
      fclose (rFile);
    }
  else LFATAL("can't create file: %s", itsLogFilename.c_str());

  return true;
}

// ######################################################################
bool BeoLogger::broadcastLogFileName()
{
  // send the log folder of where to save images
  BeobotEvents::LogFolderNameMessagePtr msg =
    new BeobotEvents::LogFolderNameMessage;
  
  // store the input image and related info
  msg->RequestID     = 0;
  msg->logFolderName = itsLogFolderName;
  
  // publish the message
  LINFO("[%6d] Publishing Log Folder Name Message: %s", 
        0, itsLogFolderName.c_str());
  publish("LogFolderNameMessageTopic", msg);
  
  return true;
}

// ######################################################################
void BeoLogger::registerTopics()
{
  // send out the log folder name
  this->registerPublisher("LogFolderNameMessageTopic");  

  // subscribe to all sensor data
  this->registerSubscription("CameraMessageTopic");
  this->registerSubscription("ColorDepthCameraMessageTopic");
  this->registerSubscription("BlankCameraMessageTopic");
  this->registerSubscription("IMUMessageTopic");
  this->registerSubscription("LRFMessageTopic");
  this->registerSubscription("GPSMessageTopic");
  this->registerSubscription("MotorMessageTopic");
  this->registerSubscription("CurrentLocationMessageTopic");

  this->registerSubscription("LogFolderNameRequestTopic");
}


// ######################################################################
void BeoLogger::evolve()
{

}

// ######################################################################
void BeoLogger::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{
  // record the time
  uint64 time = itsTimer.get();
  //LDEBUG("Get EventMessage");
  // camera message 
  if(eMsg->ice_isA("::BeobotEvents::LogFolderNameRequest"))
  {
    // doesn't matter where it comes from
    LINFO("Got a LogFolderNameRequest");

    BeobotEvents::LogFolderNameMessagePtr msg =
      new BeobotEvents::LogFolderNameMessage;
    
    // store the input image and related info
    msg->RequestID     = 0;
    msg->logFolderName = itsLogFolderName;
  
    // publish the message
    LINFO("[%6d] Publishing Log Folder Name Message: %s", 
          itsCurrMessageID, itsLogFolderName.c_str());
    publish("LogFolderNameMessageTopic", msg);
  }

  // camera message 
  else if(eMsg->ice_isA("::BeobotEvents::CameraMessage"))
  {
    // store the image
    BeobotEvents::CameraMessagePtr cameraMsg =
      BeobotEvents::CameraMessagePtr::dynamicCast(eMsg);

    int currRequestID        = cameraMsg->RequestID;
    int cameraID             = cameraMsg->cameraID;
    Image<PixRGB<byte> > ima = Ice2Image<PixRGB<byte> >(cameraMsg->image);

    if(currRequestID%REPORTING_INTERVAL == 0)
      LINFO("Got a CameraMessage [%3d] with Request ID = %d", 
            cameraID, currRequestID);
    //
    std::string saveFName
      (sformat("%s/image_%03d_%06d.ppm",
               itsLogFolderName.c_str(), cameraID, currRequestID));
    LDEBUG("Saving: %s",saveFName.c_str());
    Raster::WriteRGB(ima, saveFName);

    std::string line =
      sformat("[%15.3f] CAM[%3d] filename: %s", 
              time/1000.0, cameraID, saveFName.c_str());
    line += std::string("\n");

    writeToLogFile(line);
  }
  else if(eMsg->ice_isA("::BeobotEvents::ColorDepthCameraMessage"))
  {
    // store the RGBD image
    BeobotEvents::ColorDepthCameraMessagePtr cameraMsg =
      BeobotEvents::ColorDepthCameraMessagePtr::dynamicCast(eMsg);

    int currRequestID        = cameraMsg->RequestID;
    int cameraID             = cameraMsg->cameraID;
    Image<PixRGB<byte> > ima = Ice2Image<PixRGB<byte> >(cameraMsg->colorImage);
    Image<uint16> depth       = Ice2Image<uint16>(cameraMsg->depthImage);

    if(currRequestID%REPORTING_INTERVAL == 0)
      LINFO("Got a ColorDepthCameraMessage [%3d] with Request ID = %d", 
            cameraID, currRequestID);
    //
    std::string baseName
      (sformat("%s/image_%03d_%06d",
               itsLogFolderName.c_str(), cameraID, currRequestID));
    std::string saveFName
      (sformat("%s/image_%03d_%06d_color.ppm",
               itsLogFolderName.c_str(), cameraID, currRequestID));
    LDEBUG("Saving: %s",saveFName.c_str());
    Raster::WriteRGB(ima, saveFName);
    std::string saveDName
      (sformat("%s/image_%03d_%06d_depth.png",
               itsLogFolderName.c_str(), cameraID, currRequestID));
    LDEBUG("Saving: %s",saveDName.c_str());
    Raster::WriteFrame(GenericFrame(depth), saveDName);//saving 16-bit png

    std::string line =
      sformat("[%15.3f] RGBD[%3d] filename: %s (color/dpeth)", 
              time/1000.0, cameraID, baseName.c_str());
    line += std::string("\n");

    writeToLogFile(line);
  }

  // blank camera message 
  else if(eMsg->ice_isA("::BeobotEvents::BlankCameraMessage"))
  {
    // store the image
    BeobotEvents::BlankCameraMessagePtr bcameraMsg =
      BeobotEvents::BlankCameraMessagePtr::dynamicCast(eMsg);

    int currRequestID        = bcameraMsg->RequestID;
    int cameraID             = bcameraMsg->cameraID;
    int cameraType           = bcameraMsg->cameraType;

    if(currRequestID%REPORTING_INTERVAL == 0){
      LINFO("Got a BlankCameraMessage Type[%4s] ID:[%3d] with Request ID = %d", 
            cameraType==RGBD?"RGBD":"RGB",cameraID, currRequestID);
    }
    //
    std::string saveFName
      (sformat("%s/image_%03d_%06d.ppm",
               itsLogFolderName.c_str(), cameraID, currRequestID));

    std::string line =
      sformat("[%15.3f] %s[%3d] filename: %s", 
              time/1000.0,cameraType==RGBD?"RGBD":"RGB", cameraID, saveFName.c_str());
    line += std::string("\n");

    writeToLogFile(line);
  }


  // IMU message
  else if(eMsg->ice_isA("::BeobotEvents::IMUMessage"))
  {
    // store the IMU data
    BeobotEvents::IMUMessagePtr imuMsg =
      BeobotEvents::IMUMessagePtr::dynamicCast(eMsg);

    int currRequestID = imuMsg->RequestID;
    if(currRequestID%REPORTING_INTERVAL == 0)          
      LINFO("Got a IMUMessage with Request ID = %d", currRequestID);

    // get roll, pitch, and yaw
    if(imuMsg->validRollPitchYaw)
      {
        float roll  = imuMsg->roll;
        float pitch = imuMsg->pitch;
        float yaw   = imuMsg->yaw;

        std::string line =
         sformat("[%15.3f] IMU Euler Angle  r:%15.6f p:%15.6f y:%15.6f",
                 time/1000.0, roll, pitch, yaw);
        line += std::string("\n");

        writeToLogFile(line);
      }
    // get magnetometer
    else if(imuMsg->validMagnetometer)
      {
        float magX  = imuMsg->magX;
        float magY  = imuMsg->magY;
        float magZ  = imuMsg->magZ;

        std::string line =
         sformat("[%15.3f] IMU Magnetometer  X:%15.6f Y:%15.6f Z:%15.6f",
                 time/1000.0, magX, magY, magZ);
        line += std::string("\n");

        writeToLogFile(line);
      }
  }

  // LRF message
  else if(eMsg->ice_isA("::BeobotEvents::LRFMessage"))
  {
    // store the LRF data
    BeobotEvents::LRFMessagePtr lrfMsg =
      BeobotEvents::LRFMessagePtr::dynamicCast(eMsg);

    int currRequestID = lrfMsg->RequestID;
    int lrfID         = lrfMsg->LRFID;
    if(currRequestID%REPORTING_INTERVAL == 0)
      LINFO("Got a LRFMessage [%3d] with Request ID = %d", 
            lrfID, currRequestID);

    std::string line = sformat("[%15.3f] LRF[%3d] ", time/1000.0, lrfID);
    uint size = lrfMsg->distances.size();
    float lrfMax = 0.0,lrfMin = 100000.0;
    for(uint i = 0; i < size; i++)
      {
        if(i%10 == 0) line += std::string("\n");
        line+= sformat("%9.2f ", lrfMsg->distances[i]);
        //msg->angles   [i];
        if(lrfMsg->distances[i] > lrfMax)
          lrfMax = lrfMsg->distances[i];
				//skip -1 value in lrfMin	
        if(lrfMsg->distances[i]!= -1 && lrfMsg->distances[i] < lrfMin)
          lrfMin = lrfMsg->distances[i];
      }
    line += std::string("\n");
    line += sformat("[%15.3f] LRF MIN:%9.2f MAX:%9.2f",
                    time/1000.0, lrfMin, lrfMax);
    line += std::string("\n");

    writeToLogFile(line);
  }

  // GPS message
  else if(eMsg->ice_isA("::BeobotEvents::GPSMessage"))
  {
    // store the GPS data
    BeobotEvents::GPSMessagePtr gpsMsg =
      BeobotEvents::GPSMessagePtr::dynamicCast(eMsg);

    int currRequestID = gpsMsg->RequestID;
    LDEBUG("Got a GPSMessage with Request ID = %d", currRequestID);

    double lat = gpsMsg->latitude;
    double lon = gpsMsg->longitude;

    std::string line =
      sformat("[%15.3f] GPS lat:%15.6f lon:%15.6f",
              time/1000.0, lat, lon);
    line += std::string("\n");

    writeToLogFile(line);
  }

  //  message
  else if(eMsg->ice_isA("::BeobotEvents::MotorMessage"))
  {
    // store the Motor data
    BeobotEvents::MotorMessagePtr mtrMsg =
      BeobotEvents::MotorMessagePtr::dynamicCast(eMsg);

    int currRequestID = mtrMsg->RequestID;
    LDEBUG("Got a MotorMessage with Request ID = %d", currRequestID);

    int rcMode = mtrMsg->rcMode;

    //int motor1 = mtrMsg->motor1;
    //int motor2 = mtrMsg->motor2;

    double transVel = mtrMsg->transVel;
    double rotVel   = mtrMsg->rotVel;

    double encoderX   = mtrMsg->encoderX;
    double encoderY   = mtrMsg->encoderY;
    double encoderOri = mtrMsg->encoderOri;

    double rcTransVel = mtrMsg->rcTransVel;
    double rcRotVel   = mtrMsg->rcRotVel;

    double rcTransCap = mtrMsg->rcTransCap;
    double rcRotCap   = mtrMsg->rcRotCap;

		double robotTransVel = mtrMsg->robotTransVel;
		double robotRotVel   = mtrMsg->robotRotVel;

  
    double rawEncoderX   = mtrMsg->rawEncoderX;
    double rawEncoderY   = mtrMsg->rawEncoderY;
    double rawEncoderOri = mtrMsg->rawEncoderOri;

    double imuHeading    = mtrMsg->imuHeading;

    double trajLen = mtrMsg->trajLen;
    double dist    = mtrMsg->dist;

    double battery = mtrMsg->battery;

    std::string line =
      sformat("[%15.3f] MTR rcMode: %d, transVel: %f  rotVel: %f "
              "encoderX: %f encoderY: %f encoderOri: %f "
              "rcTransVel: %f rcRotVel: %f rcTransCap: %f rcRotCap: %f"
							"robotTransVel: %f m/s robotRotVel: %f rad/s "
              "rawEncoderX: %f rawEncoderY: %f rawEncoderOri: %f "
              "imuHeading %f "
              "trajLen: %f m dist %f m "
              "battery %f V",


              time/1000.0, rcMode, transVel, rotVel,
              encoderX,  encoderY, encoderOri,
              rcTransVel, rcRotVel, rcTransCap, rcRotCap,
							robotTransVel,robotRotVel,
              rawEncoderX,rawEncoderY,rawEncoderOri,
              imuHeading,
              trajLen,dist,
              battery                            
              );
    line += std::string("\n");

    writeToLogFile(line);
  }

  // current location message
  else if(eMsg->ice_isA("::BeobotEvents::CurrentLocationMessage"))
    {
      BeobotEvents::CurrentLocationMessagePtr clMsg =
        BeobotEvents::CurrentLocationMessagePtr::dynamicCast(eMsg);
      LINFO("Got a CurrentLocationMessage with Request ID = %d "
            "loc: (%d, %f)",
             clMsg->RequestID, clMsg->segNum, clMsg->lenTrav);

      std::string line =
        sformat("[%15.3f] LOC seg:%5d ltrav:%15.6f",
                time/1000.0, clMsg->segNum, clMsg->lenTrav);
      line += std::string("\n");

      writeToLogFile(line);
    }

  // accumulated odometry information
  else if(eMsg->ice_isA("::BeobotEvents::AccumulatedOdometryMessage"))
    {
      // we got new goal location request                     
      BeobotEvents::AccumulatedOdometryMessagePtr aoMsg =
        BeobotEvents::AccumulatedOdometryMessagePtr::dynamicCast(eMsg);      

      int currRequestID  = aoMsg->RequestID;
      float acc_odo_dist = aoMsg->AccOdo;
      LINFO("AO[%3d] distance %f", currRequestID, acc_odo_dist);

      std::string line =
        sformat("[%15.3f] AO dist: %f", time/1000.0,  acc_odo_dist);
      line += std::string("\n");

      writeToLogFile(line);
    }
}

// ######################################################################
void BeoLogger::writeToLogFile(std::string line)
{
  its_logFilename_mutex.lock();
  FILE *rFile = fopen(itsLogFilename.c_str(), "at");
  if (rFile != NULL)
    {
      fputs(line.c_str(), rFile);
      fclose (rFile);
    }
  else LFATAL("can't append to file: %s", itsLogFilename.c_str());

  its_logFilename_mutex.unlock();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

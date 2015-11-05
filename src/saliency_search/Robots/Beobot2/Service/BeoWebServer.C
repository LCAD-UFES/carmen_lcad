/*!@file Robots2/Beobot2/Hardware/BeoLogger.C Ice Module to Webpage*/
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Service/BeoWebServer.C
// $ $Id: BeoWebServer.C 15242 2012-03-30 11:15:37Z kai $
//
//////////////////////////////////////////////////////////////////////////

#include "Robots/Beobot2/Service/BeoWebServer.H"
#include "Robots/Beobot2/BeoCommon.H"
#include "Ice/BeobotEvents.ice.H"

#include "Raster/Raster.H"
#include "Util/sformat.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "Ice/IceImageUtils.H"
#include "Image/JPEGUtil.H"
#include <sys/stat.h>

#define  LOG_FOLDER "../data/logs/"
#define  REPORTING_INTERVAL     100

static BeoWebServer* webserverStatic;
// ######################################################################
static void *callback(enum mg_event event,
                      struct mg_connection *conn,
                      const struct mg_request_info *request_info) {
return static_cast<BeoWebServer*>(request_info->user_data)->HTTPRequestCallback(event, conn, request_info);
}
// ######################################################################
static void get_qsvar(const struct mg_request_info *request_info,
                      const char *name, char *dst, size_t dst_len) {
  const char *qs = request_info->query_string;
  mg_get_var(qs, strlen(qs == NULL ? "" : qs), name, dst, dst_len);
}

// ######################################################################
void *BeoWebServer::HTTPRequestCallback(enum mg_event event,
    struct mg_connection *conn,
    const struct mg_request_info *request_info)
{
  if (event == MG_NEW_REQUEST) {
    // Echo requested URI back to the client
		std::string uri(request_info->uri);
    std::string method(request_info->request_method);
		if (uri == "/image.jpg")
		{
				std::vector<byte> imgData = webserverStatic->getJpgImg();
				mg_write(conn, &imgData[0], imgData.size());
				LINFO("JPG Image Size %d",(int)imgData.size());
    }else if(uri == "/cmd"){

      int tran = 0,rot = 0;

      char trans[5],rots[5];
      get_qsvar(request_info, "trans", trans, sizeof(trans));
      get_qsvar(request_info, "rot", rots, sizeof(rots));

      tran = trans[0] == '\0' ? 0 : atoi(trans);
      rot  = rots[0] == '\0' ? 0 : atoi(rots);

      LINFO("Trans speed is %d rot %d",tran,rot);
      webserverStatic->updateMotor(tran/100.0,rot/100.0);
      mg_printf(conn,"Trans speed %d Rot speed %d",tran,rot);

    }else if(uri == "/imu"){
        mg_printf(conn,"%s",webserverStatic->itsImuString.c_str());
    }else if(uri == "/motor"){
        mg_printf(conn,"%s",webserverStatic->itsMotorString.c_str());
    }else if(uri == "/lrf"){
        mg_printf(conn,"%s",webserverStatic->itsLrfString.c_str());
    }else if(uri == "/gps"){
        mg_printf(conn,"%s",webserverStatic->itsGpsString.c_str());
		}else{
			return NULL;	
		}
    return (void *)"";  // Mark as processed
  } else {
    return NULL;
  }
}

// ######################################################################
BeoWebServer::BeoWebServer(OptionManager& mgr,
               const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsTimer(1000000)
  //  itsOfs(new OutputFrameSeries(mgr))
{
	webserverStatic = this;
	test = 1;
	itsJpgImg.resize(0);
  //  addSubComponent(itsOfs);

}

// ######################################################################
BeoWebServer::~BeoWebServer()
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
void BeoWebServer::start1()
{
  //initLogFile();
	// Webserver Code
  const char *options[] = {
		"listening_ports", "8080", 
		"document_root","src/Robots/Beobot2/Service/html",
	NULL};

	struct mg_context *ctx = mg_start(&callback, this, options);
  //getchar();  // Wait until user hits "enter"
  //mg_stop(ctx);
}

// ######################################################################
void BeoWebServer::start3()
{
  broadcastLogFileName();

  // set start time
  itsTimer.reset();
}

// ######################################################################
bool BeoWebServer::initLogFile()
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
bool BeoWebServer::broadcastLogFileName()
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
void BeoWebServer::registerTopics()
{
  // send out the log folder name
  this->registerPublisher("LogFolderNameMessageTopic");  

  // subscribe to all sensor data
  this->registerSubscription("CameraMessageTopic");
  this->registerSubscription("BlankCameraMessageTopic");
  this->registerSubscription("IMUMessageTopic");
  this->registerSubscription("LRFMessageTopic");
  this->registerSubscription("GPSMessageTopic");
  this->registerSubscription("MotorMessageTopic");
  this->registerSubscription("CurrentLocationMessageTopic");

  this->registerSubscription("LogFolderNameRequestTopic");

  this->registerPublisher("MotorRequestTopic");
}


// ######################################################################
void BeoWebServer::evolve()
{

}
// ######################################################################
void BeoWebServer::updateMotor(double tran, double rot)
{
  BeobotEvents::MotorRequestPtr msg = new BeobotEvents::MotorRequest;
  msg->RequestAppID = BEO_WEB;
  msg->transVel = tran;
  msg->rotVel   = rot;
  this->publish("MotorRequestTopic", msg);
  // LINFO("[%d] Publish motor request Trans %f Rotation %f",
  //       itsPrevProcImgID,tran,rot);
}
// ######################################################################
std::vector<byte> BeoWebServer::getJpgImg()
{
		std::vector<byte> loc_jpg;
		its_JpgImg_mutex.lock();
			loc_jpg = itsJpgImg ;
		its_JpgImg_mutex.unlock();

		return loc_jpg;
		
}
// ######################################################################
void BeoWebServer::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{
  // record the time
  uint64 time = itsTimer.get();

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
		// write frame id on the image
		char buffer[255];
		sprintf(buffer, "[%5d]", currRequestID);
		writeText(ima, Point2D<int>(0,0), buffer, 
				PixRGB<byte>(255,255,255), 
				PixRGB<byte>(0,0,0),
				SimpleFont::FIXED(6));

		// write display fps on the image
		sprintf(buffer, "[%2d]", cameraID);
		writeText(ima, Point2D<int>(ima.getWidth()-strlen(buffer)*6,0), 
				buffer, 
				PixRGB<byte>(255,255,255), 
				PixRGB<byte>(0,0,0),
				SimpleFont::FIXED(6));
		JPEGCompressor jcomp;
		its_JpgImg_mutex.lock();
			itsJpgImg = jcomp.compressImage(ima);
		its_JpgImg_mutex.unlock();

    if(currRequestID%REPORTING_INTERVAL == 0)
      LINFO("Got a CameraMessage [%3d] with Request ID = %d", 
            cameraID, currRequestID);
    //
    std::string saveFName
      (sformat("%s/image_%03d_%06d.ppm",
               itsLogFolderName.c_str(), cameraID, currRequestID));
    LDEBUG("Saving: %s",saveFName.c_str());

    std::string line =
      sformat("[%15.3f] CAM[%3d] filename: %s", 
              time/1000.0, cameraID, saveFName.c_str());
    line += std::string("\n");

  }

  // blank camera message 
  else if(eMsg->ice_isA("::BeobotEvents::BlankCameraMessage"))
  {
    // store the image
    BeobotEvents::BlankCameraMessagePtr bcameraMsg =
      BeobotEvents::BlankCameraMessagePtr::dynamicCast(eMsg);

    int currRequestID        = bcameraMsg->RequestID;
    int cameraID             = bcameraMsg->cameraID;

    if(currRequestID%REPORTING_INTERVAL == 0)
      LINFO("Got a BlankCameraMessage [%3d] with Request ID = %d", 
            cameraID, currRequestID);
    //
    std::string saveFName
      (sformat("%s/image_%03d_%06d.ppm",
               itsLogFolderName.c_str(), cameraID, currRequestID));

    std::string line =
      sformat("[%15.3f] CAM[%3d] filename: %s", 
              time/1000.0, cameraID, saveFName.c_str());
    line += std::string("\n");

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
        itsImuString = line;
        line += std::string("\n");

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
    itsLrfString = line;
    line += std::string("\n");
    line += sformat("[%15.3f] LRF MIN:%9.2f MAX:%9.2f",
                    time/1000.0, lrfMin, lrfMax);
    line += std::string("\n");

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
    itsGpsString = line;
    line += std::string("\n");

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

    std::string line =
      sformat("[%15.3f] MTR rcMode: %d, transVel: %f  rotVel: %f "
              "encoderX: %f encoderY: %f encoderOri: %f "
              "rcTransVel: %f rcRotVel: %f",
              time/1000.0, rcMode, transVel, rotVel,
              encoderX,  encoderY, encoderOri,
              rcTransVel, rcRotVel);
    itsMotorString = line;
    line += std::string("\n");

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

    }
}

// ######################################################################
void BeoWebServer::writeToLogFile(std::string line)
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

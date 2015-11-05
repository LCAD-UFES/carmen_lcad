/*!@file Robots2/Beobot2/Hardware/BeoSim.C Beobot2.0 simulator    */
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
// Primary maintainer for this file: Chin-Kai Chang <chinkaic@usc.edu>
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Hardware/BeoSim.C
// $ $Id: BeoSim.C 15190 2012-02-29 04:00:13Z kai $
//
//////////////////////////////////////////////////////////////////////////
#include "Raster/Raster.H"
#include "Robots/Beobot2/Hardware/BeoSim.H"
#include "Ice/BeobotEvents.ice.H"
#include "Ice/IceImageUtils.H" // for Image2Ice
#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"
#include "Util/sformat.H"
#include "Util/MathFunctions.H"
#include "Image/ShapeOps.H" // for rescale()
#include "Image/MatrixOps.H"
#include <iostream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#define LOG_FOLDER "../data/logs"
//HD
//#define WINDOW_W 1600 
//#define WINDOW_H 1080 

//XGA
#define WINDOW_W 1024 
#define WINDOW_H 768 

#define MAPWINDOW_H WINDOW_H-266 
#define MAPWINDOW_W WINDOW_W 

const ModelOptionCateg MOC_BeoSim = {
  MOC_SORTPRI_3, "Beobot Pilot Related Options" };

const ModelOptionDef OPT_MinimumSafeDistance =
{ MODOPT_ARG(float), "MinimumSafeDistance", &MOC_BeoSim, OPTEXP_CORE,
  "The minimum distance (in meters) to objects we can allow before cutting off the motors.",
  "min-distance", '\0', "float", ".5"};

const ModelOptionDef OPT_MaxSpeed =
{ MODOPT_ARG(float), "MaxSpeed", &MOC_BeoSim, OPTEXP_CORE,
  "The maximum speed of the robot (from [0 .. 1]).",
  "max-speed", '\0', "float", "1.0"};

const ModelOptionDef OPT_IgnoreSonar =
{ MODOPT_ARG(bool), "IgnoreSonar", &MOC_BeoSim, OPTEXP_CORE,
  "Do we want to ignore the sonar for proximity warnings. Use this with caution, as someone could get hurt!?",
  "ignore-sonar", '\0', "true | false", "true"};

const ModelOptionDef OPT_LogFile =
{ MODOPT_ARG(std::string), "LogFile", &MOC_BeoSim, OPTEXP_CORE,
  "There are 3 mode for input log file name \n"
    "you can enter\n"
    "1)  ./bin/app-BeoSim --log-file=<Full Log File Path>\n"
    "    it will open the file directly\n"
    "2)  ./bin/app-BeoSim --log-file=<date>\n"
    "    it will look for ../data/logs/<date>/Log_<date>.txt\n"
    "    The date formate can be flexible.\n"
    "    You can enter full date such as 2009_11_03__14_03_49\n"
    "    or 20091103 or 091103 or 1403 or 140349. \n"
    "    If there are more than one match, \n"
    "    it will list all options for choosen.\n"
    "3)  ./bin/app-BeoSim --log-file=search\n"
    "    it will look for all directory under ../data/logs/\n"
    "    and list all files for choosen\n",
  "log-file", '\0', "<string>", "search"};


const ModelOptionDef OPT_ReplaySpeed=
{ MODOPT_ARG(int), "ReplaySpeed", &MOC_BeoSim, OPTEXP_CORE,
  "The replay speed from the Log file,default is 1x. To speed up the speed, using --speed=2 or faster",
  "speed", '\0', "int", "1"};
#define ORI_OFFSET (230.0/180.0)*M_PI
#define QUE_SIZE 150
//const double cosOri = cos(ORI_OFFSET);
//const double sinOri = sin(ORI_OFFSET);
#define RC2VEL 3.0 // convert from -1.0~1.0 rc value to velocity m/s
// ######################################################################
BeoSim::BeoSim(int id, OptionManager& mgr,
    const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsCurrMessageID(0),
  itsCurrentTimeIndex(0),
  itsLogCameraTimeIndex(0),
  itsLogIMUTimeIndex(0),
  itsLogFrontLRFTimeIndex(0),
  itsLogDownLRFTimeIndex(0),
  itsLogTimer(1000000),
  itsLoadFromLogFile(true),
  itsLogLeftMotorVelocity(0.0),
  itsLogRightMotorVelocity(0.0),
  itsLogPosition(0.0,0.0,0.0),
  itsLogDiffPosition(0.0,0.0,0.0),
  itsLogCameraFrameID(0),
  itsRcTransSpeed(0.0),
  itsRcRotSpeed(0.0),
  itsRcTransSpeedCapped(0.0),
  itsRcRotSpeedCapped(0.0),
  itsTransCap(0.0),
  itsRotCap(0.0),
  itsRCChannels(7, -1),
  itsEmergencyMode(-1),
  itsPosition(0.0,0.0,0.0),
  itsDiffPosition(0.0,0.0,0.0),
  itsIMUDiffPosition(0.0,0.0,0.0),
  itsMapImage(MAPWINDOW_W,MAPWINDOW_H,ZEROS),
  itsInfoImage(640,400,ZEROS),
  itsTrajImage(640,240,ZEROS),
  itsMapCenter(WINDOW_W/20,(MAPWINDOW_H)/2),
  itsEncoderTimer(1000000),
  itsLeftMotorVelocity(0.0),
  itsRightMotorVelocity(0.0),
  itsTravelDistance(0.0),
  itsTravelDistanceAuto(0.0),
  itsTravelDistanceManual(0.0),
  itsStraightDistance(0.0),
  itsBattery(0.0),
  itsPidRot(0.1,0.04,0.05,0.0,0.0,0,0,0,1.0,-1.0),
  itsPidTrans(0.1,0.10,0.05,//pid gain
      0.0,1.0,//imin,imax
      0,//errthresh,not in use
      0,0//pos,neg thresh,not in use
      ,1.0,-1.0),//max,min motor

  //    nomovethresh,
  //    runpid,
  //    speed,
  //    posstaticerrthresh,
  //    negstaticerrthresh
  itsSonarTimeout(.5),
  itsIMUheading(10.0),
  itsIMUheadingInit(10.0),
  itsIMURoll(0.0),
  itsIMUPitch(0.0),
  itsIMUon(false),
  itsIMUPosition(0.0,0.0,0.0),
  itsControlImage(512,256,ZEROS),
  itsDispImage(WINDOW_W,WINDOW_H,ZEROS),
  itsDispCameraImage(320,240,ZEROS),
  itsWaitScreen(false),
  itsMeterXwin(itsDispImage,"BeoBot Sim"),
  //itsMapXwin(itsMapImage,"Map Control"),
  itsDisplayUpdateRate(.05),
  itsMapScale(100.0),
  itsChannelMeters(7, SimpleMeter(20, 60, 0,100)),
  itsVelocityTransQue(QUE_SIZE),
  itsVelocityRotQue(QUE_SIZE),
  itsVelocityTransCmdQue(QUE_SIZE),
  itsVelocityRotCmdQue(QUE_SIZE),
  itsVelocityTransRequestQue(QUE_SIZE),
  itsVelocityRotRequestQue(QUE_SIZE),
  itsMinimumSafeDistance(&OPT_MinimumSafeDistance, this, 0),
  itsMaxSpeed(&OPT_MaxSpeed, this, 0),
  itsIgnoreSonar(&OPT_IgnoreSonar, this, 0),
  itsLogFile(&OPT_LogFile, this, 0),
  itsReplaySpeedOpt(&OPT_ReplaySpeed, this, 0),
  itsReplaySpeed(0)
{
}

// ######################################################################
void BeoSim::checkLogFile()
{

  std::string logfile = itsLogFile.getVal();
  if(itsLogFile.getVal() =="NULL"){
    LINFO("Start BeoSim with real device");
  }else if(itsLogFile.getVal() =="search" || itsLogFile.getVal().length() > 0){
    LINFO("Search for all log file");
    DIR *dir_p;
    struct dirent *entry_p;
    dir_p = ::opendir(LOG_FOLDER);
    if(dir_p == NULL)
      LFATAL("Can Not Open %s directory",LOG_FOLDER);
    //uint i = 1;

    std::vector<std::string> subdir;
    while((entry_p = ::readdir(dir_p)))
    {
      std::string subdirname(entry_p->d_name);
      if(subdirname != "." && subdirname !="..")
      {
        //LINFO("File[%d]:[%s]",i++,subdirname.c_str());
        if(subdirname.find(logfile) != std::string::npos || logfile == "search")
        {
          subdir.push_back(subdirname);
          //LINFO("Found %s match file query %s",subdirname.c_str(),logfile.c_str());
        }
      }
    }
    (void)::closedir(dir_p);

    //sort filename
    std::sort(subdir.begin(),subdir.end());
    for(int i = 0;i < (int)subdir.size();i++)
    {
      LINFO("File[%3d]:[%s]  \t[%3d]",i+1,subdir[i].c_str(),i+1);
    }




    int option = -1;
    //Don't ask user if only one option
    if(subdir.size()==0){LFATAL("No log file");}
    if(subdir.size()==1){
      option = 1;
    }else{  
      do{
        LINFO("Please Choose a File Number:");
        int ret = scanf("%d",&option);//FIXX Any better idea?
        if(option <= 0 || ret == 0) LFATAL("Exit");
        if(option >int(subdir.size()))
          LINFO("Option Invalid, please try again");
      }while(option < 1 || option > int(subdir.size()));
      LINFO("Your Choose is [%d] filename[%s] ",option,subdir[option-1].c_str());
    }
    //                int run,segt;
    //                char buf[255];
    //sscanf(subdir[option-1].c_str(),"S%d_R%d_%s",&run,&segt,buf);
    std::string logFileName(
        sformat("%s/%s/Log_%s.txt",
          LOG_FOLDER,
          subdir[option-1].c_str(),
          subdir[option-1].c_str()
          //buf
               ));
    std::string outputFileName(
        sformat("%s/BeoCoord_%s",
          LOG_FOLDER,
          subdir[option-1].c_str()));

    itsLogFolderName = subdir[option-1];
    itsLogFile.setVal(logFileName);
    itsOutputFile = outputFileName;
    loadFromLogFile();
  }else{
    LINFO("LogFileName:[%s]",itsLogFile.getVal().c_str());
    loadFromLogFile();
  }


}

// ######################################################################
void BeoSim::loadFromLogFile()
{


  FILE *logFile = fopen(itsLogFile.getVal().c_str(),"r");
  if(logFile == NULL)
  {
    LFATAL("can't not open file: %s",itsLogFile.getVal().c_str());
  }else
  {
    char line[512];
    bool useCustomStartFrameNumber = false;
    bool startLoad=!useCustomStartFrameNumber;
    int idCounter = 0;
    while(fgets(line,sizeof line,logFile)!= NULL)
    {
      //Read Motor Data
      float time,transVel,rotVel,encoderX,encoderY,encoderOri,rcTransVel,rcRotVel,rcTransCap,rcRotCap;
      float robotTransVel,robotRotVel;
      float rawEncoderX,rawEncoderY,rawEncoderOri;
      float imuHeading;
      float trajLen,dist;
      float battery;
      int rcMode;
      int ret = sscanf (line,"[%f] MTR rcMode: %d, transVel: %f  rotVel: %f encoderX: %f encoderY: %f encoderOri: %f "
                             "rcTransVel: %f rcRotVel: %f rcTransCap: %f rcRotCap: %f "
                             "robotTransVel: %f m/s robotRotVel: %f rad/s "
                             "rawEncoderX: %f rawEncoderY: %f rawEncoderOri: %f "
                             "imuHeading %f "
                             "trajLen: %f m dist %f m "
                             "battery %f V",
          &time, &rcMode, &transVel, &rotVel, &encoderX,  &encoderY, &encoderOri, 
          &rcTransVel, &rcRotVel, &rcTransCap, &rcRotCap, 
          &robotTransVel, &robotRotVel,
          &rawEncoderX,&rawEncoderY,&rawEncoderOri,
          &imuHeading,
          &trajLen,&dist,
          &battery                            
          );
      if(startLoad && (ret == 9 || ret==13 ||ret == 20)){
        Beobot2::motorData tmp;
        tmp.time = time;
        tmp.rcMode = rcMode;
        tmp.transVel = transVel;
        tmp.rotVel = rotVel;
        tmp.encoderX = encoderX;
        tmp.encoderY = encoderY;
        tmp.encoderOri = encoderOri;
        tmp.rcTransVel = rcTransVel;
        tmp.rcRotVel = rcRotVel;

        if(ret==20){
         tmp.rawEncoderX   = rawEncoderX; 
         tmp.rawEncoderY   = rawEncoderY; 
         tmp.rawEncoderOri = rawEncoderOri; 
         tmp.imuHeading    = imuHeading;
         tmp.trajLen       = trajLen;
         tmp.dist          = dist;
         tmp.battery       = battery; 

        }else{          
          LINFO("Ret is %d",ret);
          tmp.rawEncoderX   = encoderX; 
          tmp.rawEncoderY   = encoderY; 
          tmp.rawEncoderOri = encoderOri; 
          tmp.imuHeading    = -M_PI*2;
          tmp.trajLen       = -1.0;
          tmp.dist          = -1.0;
          tmp.battery       = -1.0; 
        }
          
        if(ret==13||ret==20){
          tmp.rcTransCap = rcTransCap;
          tmp.rcRotCap = rcRotCap;
          tmp.robotTransVel = robotTransVel;
          tmp.robotRotVel = robotRotVel;
        }else{
          tmp.rcTransCap = 0.0;
          tmp.rcRotCap = 0.0;
          int logDataSize = (int) itsLogData.size();
          if(logDataSize != 0){
            Beobot2::motorData lastData = itsLogData[logDataSize -1];
            double dt = time - lastData.time;
            double vx = encoderX / dt;
            double vy = encoderY / dt;
            double dtheta = encoderOri - lastData.encoderOri;
            double w = dtheta/dt;
            double v = sqrt(vx*vx+vy*vy);
            tmp.robotTransVel = v;
            tmp.robotRotVel = w;
          }else{
            tmp.robotTransVel = 0.0;
            tmp.robotRotVel = 0.0;
          }

        }
        itsLogData.push_back(tmp);
        LDEBUG("Got Motor Data %f %d",time,rcMode);

      }

      //Read Camera Data
      std::string imagePath;
      char imgPath[256];
      int camid;
      ret = sscanf(line,"[%f] CAM[%d] filename: %s", &time, &camid, imgPath);
      if(ret == 3)
      {
        Beobot2::imageData imgData;
        imgData.time = time;
        imgData.imagePath = imgPath;
        //FIXXXXX
        //../data/logs/2012_04_05__14_20_26/image_000_003242.ppm
        //char folder[80];
        //char imgName[256];
        if(useCustomStartFrameNumber)
        {
          int cameraID,frameID;
          int rret = sscanf(imgPath,"../data/logs/2012_04_05__14_20_26/image_%d_%d.ppm",&cameraID,&frameID);
                                                  
          if(rret ==2){
            LINFO("Got Image id [%d] frame[%d]",cameraID,frameID);
            if(frameID >= 15000) startLoad = true;
            if(frameID > 24000) startLoad = false;
            //LINFO("imgName [%s]",imgName);
            imgData.frameID = frameID;
            imgData.cameraID = cameraID;
          }else{
            LINFO("RRET is %d",rret);
          }
        }else{
          char logdir[50],imgfilename[255];
          int cameraID = 0;
          int rret = sscanf(imgPath,"../data/logs/%20s/%s",logdir,imgfilename);
          if(rret ==2){
            //LINFO("Got Image dir [%s] frame[%s]",logdir,imgfilename);
            imgData.frameID = idCounter++;
            imgData.cameraID = 0;
            imgData.fileName = std::string(imgfilename);
          }else{
            LINFO("no match rret is %d %s",rret,logdir);
            imgData.frameID = idCounter++;
            imgData.cameraID = cameraID;
          }
        }
        if(startLoad){
          itsLogImageData.push_back(imgData);
          LINFO("Got Image at %f path=[%s] from camera %d",time,imgPath,camid);
        }
      }
      //Read IMU Data
      float roll,pitch,yaw;
      ret = sscanf(line,"[%f] IMU Euler Angle  r:%f p:%f y:%f",
          &time, &roll, &pitch, &yaw);
      if(ret == 4 && startLoad){
        LDEBUG("Got IMU at %5.2f roll %4.2f pitch %4.2f yaw %f4.2",time,roll,pitch,yaw);
        Beobot2::imuData imudata;
        imudata.time = time;
        imudata.roll = roll;
        imudata.pitch = pitch;
        imudata.yaw = yaw;
        itsLogIMUData.push_back(imudata);

      }
      //Read LRFData
      float lrfMax,lrfMin;
      int lrfID;//0 is front LRF,1 is down LRF
      ret = sscanf(line,"[%f] LRF[%d] ", &time, &lrfID);
      if(ret == 2 && startLoad){
        LDEBUG("Got LRF[%d] at %5.2f",lrfID,time);
        bool endLRF = false;
        int linecount = 0;
        std::vector<double> dist;
        do{
          if(fgets(line,sizeof line,logFile)!= NULL){

            //check is end of lrf line
            ret = sscanf(line,"[%f] LRF MIN:%f MAX:%f", &time, &lrfMin, &lrfMax);
            if(ret == 3){
              LDEBUG("Got LRF[%d] min %f max %f",lrfID,lrfMin,lrfMax);
              endLRF = true;            
            }else{
              //normal laser data
              std::stringstream buffer(line);
              std::vector<std::string> ret;

              std::copy( std::istream_iterator<std::string>(buffer),
                  std::istream_iterator<std::string>(),
                  std::back_inserter(ret));
              for(uint i=0;i<ret.size();i++)
                dist.push_back(atof(ret[i].c_str())/1000.0);//convert mm to meter
            }

          }else{
            endLRF = true;  
          }
        }while(!endLRF && linecount++ < 30);
        LDEBUG("Got LRF[%d] with %d number eading",lrfID,(int)dist.size());
        Beobot2::lrfData lrfdata;
        lrfdata.time = time;
        lrfdata.lrfMax = lrfMax;
        lrfdata.lrfMin = lrfMin;
        lrfdata.dist = dist;
        lrfdata.deviceID = lrfID;
        if(lrfID == 0){
          itsLogFrontLRFData.push_back(lrfdata);          
        }else{
          itsLogDownLRFData.push_back(lrfdata);                   
        }

      }//end read lrf



    }
    LINFO("Load Log file with %d motor command lines",(int)itsLogData.size());
    LINFO("Load Log file with %d Images",(int)itsLogImageData.size());
    itsLoadFromLogFile = true;
    itsLogTimer.reset();
    //drawAllMap();
  }
}


void BeoSim::start3()
{
  checkLogFile();
  itsReplaySpeed = itsReplaySpeedOpt.getVal();
}

// ######################################################################
// ######################################################################
BeoSim::~BeoSim()
{
  //Kill the motors on exit
  SetMotors(0,0);
}


// ######################################################################
void BeoSim::registerTopics()
{
  //registerSubscription("MotorRequestTopic");
  //registerSubscription("SonarMessageTopic");
  //registerSubscription("IMUMessageTopic");
  
  registerPublisher("LRFMessageTopic");
  registerPublisher("MotorMessageTopic");
  registerPublisher("CameraMessageTopic");
  registerPublisher("IMUMessageTopic");
  // sends out the screen shot to BeoVisualizer
  registerPublisher("VisualizerMessageTopic");
}


// ######################################################################
void BeoSim::evolve()
{

  //Recieve telemetry from the propeller

  itsDebugWin.reset();
  LDEBUG("Evolve Start");


  itsRotVelocityTarget = itsRcRotSpeedCapped;
  itsTransVelocityTarget = itsRcTransSpeedCapped;
  //itsVelocityQue.add(randomDouble()*2 -1.0);
  //itsVelocityTargetQue.add(randomDouble());
  //int lastRcMode = itsRemoteMode;
  //Check if it is in log file replay mode and also make sure emergency mode is off
  if(itsLoadFromLogFile){
    if(itsWaitScreen){
      itsLogTimer.pause();
    } 
    else{
      itsLogTimer.resume();
      //No RC(Emergency = -1): getCurrentLogData and timer resume but no set Motor
      //With RC:
      // Emergency is off (Emergency = 0)
      //        1) Mode 1,2 :timer pause
      //        2) Mode 3 timer resume and run log file,set motor
      // Emergency is on (Emergency = 255)
      //         1) Timer pause
      if(itsEmergencyMode == -1||itsEmergencyMode == 255)
      {
        itsLogTimer.resume();
        getCurrentLogData();
      }
      else if(itsEmergencyMode == 0 && itsRemoteMode == 3)
      {
        itsLogTimer.resume();
        getCurrentLogData();
        SetMotors(itsRotationalSpeed,itsForwardSpeed);
      }
      else{
        itsLogTimer.pause();
        LINFO("Wait for RC mode 3");
      }
      uint64 time = itsLogTimer.get()/1000.0;
      LDEBUG("Current Log Timer is %f",(double)time);
    }
  }
  //Draw the GUI to the screen
  updateGUI();
  createVisualGUI();
  handleUserEvent();

}
// ######################################################################
void BeoSim::saveScreenShot()
{

  //save screen shot to file
  std::string saveTName(sformat("../vss2012/image_%05d_%05d_T.png",itsCurrentTimeIndex,itsLogCameraFrameID));
  std::string saveCName(sformat("../vss2012/image_%05d_%05d_C.png",itsCurrentTimeIndex,itsLogCameraFrameID));
  if(bfile_exists(saveTName.c_str())){
    LINFO("File %s Exist....Skip",saveTName.c_str());
  }else{
    Raster::WriteRGB(itsDrawMapImage, saveTName);
    Raster::WriteRGB(itsDispCameraImage, saveCName);
    LINFO("Save %s and %s",saveTName.c_str(),"_C");
  }
}
// ######################################################################
void BeoSim::reset()
{
  itsLogCameraFrameID   = 0;
  itsCurrMessageID      = 0;
  itsCurrentTimeIndex   = 0;
  itsLogCameraTimeIndex = 0;
  itsLogIMUTimeIndex    = 0;
  itsLogFrontLRFTimeIndex = 0;
  itsLogDownLRFTimeIndex= 0;
  itsLogPositionMap.clear();
  itsIMUPositionMap.clear();
  itsLRFPositionMap.clear();
  itsLRFCurrentMap.clear();
  itsTravelDistance = 0;
  itsTravelDistanceAuto = 0;
  itsTravelDistanceManual = 0;
  itsBattery = 0.0;
  itsMapScale =100.0;
  itsIMUPosition = Point3DwithColor<double>(0.0,0.0,0.0);
  itsIMUDiffPosition = Point3DwithColor<double>(0.0,0.0,0.0);
  itsPosition = Point3DwithColor<double>(0.0,0.0,0.0);
  itsDiffPosition = Point3DwithColor<double>(0.0,0.0,0.0);
  itsLogPosition = Point3DwithColor<double>(0.0,0.0,0.0);
  reDrawMap(itsMapScale);
  itsLogTimer.reset();
}
// ######################################################################
void BeoSim::handleUserEvent()
{
  //handle clicks
  //  const nub::soft_ref<ImageDisplayStream> ids =
  //    itsOfs->findFrameDestType<ImageDisplayStream>();
  //

  int key = itsMeterXwin.getLastKeyPress();

  switch(key)
  {
    case -1:
      break;
    case 27: //r reset
      reset();
      break;
    case 13: //g (MAC)
    case 42: //g label ground
      break;
    case 11: //f (mac)
    case 41: //f forward faster
      itsReplaySpeed++;
      break;
    case 10: //d (mac) 
    case 40: //d forward slower 
      itsReplaySpeed--;
      if(itsReplaySpeed < 1) itsReplaySpeed = 1;
      break;
    case 9:  //s (mac) 
    case 39: //s save screenshot 
      saveScreenShot();
      break;
    case  8: //a (mac) 
    case 38: //a forward normal
      itsReplaySpeed = 1;
      break;
    case 12: //h (mac) 
    case 43: //h switch labeling ground truth mode 
      break;
      //case 12: //j (mac) 
    case 44: //j play only ground truth image
      break;
    case 57: //space (mac)(n on PC) 
    case 65: //space pause/resume
      itsWaitScreen =! itsWaitScreen;
      break;
    case 14: //z (mac)
    case 52: //z switch to Anf
      outputMap();
      break;
    case 15: //x switch to SSL 
    case 53: //x switch to SSL 
      break;
    case 16: //c (mac) 
    case 54: //c switch to Equad
      break;
    case 17: //v (mac)
    case 55: //v switch to HNB 
      break;
    case 19: //b (mac) 
    case 56: //b switch to RoadGenerator 
      break;
    case 20: //q (mac) 
    case 24: //q Turn on/off VPD 
      break;
    case 21: //w (mac) 
    case 25: //w Turn on/off CSH 
      break;
    case 111://up key
      itsMapCenter.j -=10;
      break;
    case 116://down key
      itsMapCenter.j +=10;
      break;
    case 113://left key
      itsMapCenter.i -=10;
      break;
    case 114://right key
      itsMapCenter.i +=10;
      break;
    default:    
      LINFO("key %i", key);
      break;
  }

  Point2D<int> pos = itsMeterXwin.getLastMouseClick();
  if (pos.isValid())
  {
    PixRGB<byte> pixColor = itsDispImage.getVal(pos);
    LINFO("Mouse Click (%d %d) Color (%d,%d,%d)", 
        pos.i,pos.j,pixColor.red(),pixColor.green(),pixColor.blue());
  } 
} 

// ######################################################################
//####################################################################
void BeoSim::updateIMUPosition(double dt)
{
  double loc_IMUheading = 10.0;
  itsIMUMutex.lock();
  {
    //  LINFO("locIMU %f, itsIMU %f",loc_IMUheading,itsIMUheading);
    loc_IMUheading = itsIMUheading;
  }
  itsIMUMutex.unlock();
  if(loc_IMUheading <= M_PI && loc_IMUheading >= -M_PI && loc_IMUheading < 10.0)
  {
    //LINFO("I have locIMU %f, itsIMU %f",loc_IMUheading,itsIMUheading);
    //initialized first imu heading
    if(itsIMUheadingInit == 10.0)
    {
      itsIMUheadingInit = loc_IMUheading; 
      itsIMUon = true;
      LDEBUG("initial imu is %f",itsIMUheadingInit);
    }
    //LINFO("Got IMU Reading, Current Heading is %f",loc_IMUheading); 
    double imu_ot = (loc_IMUheading - itsIMUheadingInit); 
    //double dt = (dr + dl)/2;
    double imu_diff_x = dt * cos(imu_ot);
    double imu_diff_y = dt * sin(imu_ot);
    double imu_xt = itsIMUPosition.x + imu_diff_x;
    double imu_yt = itsIMUPosition.y + imu_diff_y;
    //LDEBUG("ENC ot %3.2f,xt %3.2f,yt %3.2f,IMU ot %3.2f,xt %3.2f,yt %3.2f",ot,xt,yt,imu_ot,imu_xt,imu_yt);
    itsIMUPosition.x = imu_xt;
    itsIMUPosition.y = imu_yt;
    itsIMUPosition.z = imu_ot;//use z as orientation data
    if(itsLogRemoteMode == 1)
      itsIMUPosition.color = PixRGB<byte>(255,0,0);
    else
      itsIMUPosition.color = PixRGB<byte>(0,0,255);//IMU use blue

    itsIMUDiffPosition.x = imu_diff_x; 
    itsIMUDiffPosition.y = imu_diff_y; 
    itsIMUDiffPosition.z = imu_ot; 

    itsTravelDistance += sqrt(imu_diff_x*imu_diff_x + imu_diff_y*imu_diff_y);
    itsStraightDistance = sqrt(imu_xt*imu_xt +imu_yt*imu_yt); 

  }else{
    //itsIMUheading = itsPosition.z;//when imu is not available, juse use wheel heading
    itsIMUPosition.z = itsPosition.z;//
    double dx = itsDiffPosition.x;
    double dy = itsDiffPosition.y;
    double xt = itsPosition.x;
    double yt = itsPosition.y;
    itsTravelDistance += sqrt(dx*dx +dy*dy); 
    itsStraightDistance = sqrt(xt*xt + yt*yt);

  }
}   
// ######################################################################
void BeoSim::resetEncoder()
{
  itsPosition.x = 0;
  itsPosition.y = 0;
  itsPosition.z = 0;
  itsDiffPosition.x = 0;
  itsDiffPosition.y = 0;
  itsDiffPosition.z = 0;
}
// ######################################################################
void BeoSim::SetMotors(float rotationalSpeed, float translationalSpeed) { }
// ######################################################################
void BeoSim::createVisualGUI()
{
    double loc_forwardSpeed = itsForwardSpeed;
    double loc_rotationalSpeed = itsRotationalSpeed;
    //Update the display
    itsInfoImage = Image<PixRGB<byte> >(640,400,ZEROS);
    char buffer[128];
    int speedHeight = 0;
    sprintf(buffer, "FWR: %6.3f:%6.3f", loc_forwardSpeed,loc_forwardSpeed*itsTransCap);

    writeText(itsInfoImage, Point2D<int>(0,speedHeight), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(12));

    speedHeight+=20;
    sprintf(buffer, "ROT: %6.3f:%6.3f", loc_rotationalSpeed, loc_rotationalSpeed *itsRotCap/100.0);
    writeText(itsInfoImage, Point2D<int>(0,speedHeight), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(12));

    speedHeight+=20;
    sprintf(buffer, "TransCap: %3d  RotatCap %3d", (int)itsTransCap,(int)itsRotCap);
    writeText(itsInfoImage, Point2D<int>(0,speedHeight), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

    speedHeight+=20;
    sprintf(buffer, "Forward    Speed:%5.2f m/s", itsTransVelocityCurrent);
    writeText(itsInfoImage, Point2D<int>(0,speedHeight), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(12));

    speedHeight+=20;
    sprintf(buffer, "Rotational Speed:%5.2f rad/s ", itsRotVelocityCurrent);
    writeText(itsInfoImage, Point2D<int>(0,speedHeight), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(12));



    int encHeightCurrent = 105;
    int encHeightSize = 20;
    int encFontSize = 12;
    sprintf(buffer, "IMU X: %f m",itsIMUPosition.x );
    writeText(itsInfoImage, Point2D<int>(0,encHeightCurrent), buffer, 
      PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(encFontSize));

    encHeightCurrent+= encHeightSize;
    sprintf(buffer, "IMU Y: %f m",itsIMUPosition.y );
    writeText(itsInfoImage, Point2D<int>(0,encHeightCurrent), buffer, 
      PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(encFontSize));

      int travelPosH = 0;
      int travelPosW = 350;
      int travelHeight = 20;
      int travelFont = 12;

      sprintf(buffer, "Travel Manual: %8.4fm",itsTravelDistanceManual );
      writeText(itsInfoImage, Point2D<int>(travelPosW,travelPosH), buffer, 
        PixRGB<byte>(255,0,0), PixRGB<byte>(0,0,0),SimpleFont::FIXED(travelFont));

      travelPosH += travelHeight;
      sprintf(buffer, "Travel Auto  : %8.4fm",itsTravelDistanceAuto );
      writeText(itsInfoImage, Point2D<int>(travelPosW,travelPosH), buffer, 
        PixRGB<byte>(0,255,0), PixRGB<byte>(0,0,0),SimpleFont::FIXED(travelFont));

      travelPosH += travelHeight;
      itsTravelDistance = itsTravelDistanceAuto+itsTravelDistanceManual;
      double autoRate = 0.0;
      if(itsTravelDistance!=0.0) autoRate = itsTravelDistanceAuto/itsTravelDistance;
      sprintf(buffer, "Travel Total : %8.4fm ",itsTravelDistance);
      writeText(itsInfoImage, Point2D<int>(travelPosW,travelPosH), buffer, 
        PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(travelFont));

      travelPosH += travelHeight;
      sprintf(buffer, "Auto Drive %5.2f%%",autoRate*100.0);
      writeText(itsInfoImage, Point2D<int>(travelPosW,travelPosH), buffer, 
        PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(travelFont));

    std::vector<std::vector<float> > lines;
    lines.push_back(itsVelocityTransQue.getVector());
    lines.push_back(itsVelocityTransCmdQue.getVector());
    lines.push_back(itsVelocityTransRequestQue.getVector());

    std::vector<PixRGB<byte> > linesColor;
    linesColor.push_back(PixRGB<byte>(255,0,0));
    linesColor.push_back(PixRGB<byte>(255,165,0));
    linesColor.push_back(PixRGB<byte>(0,0,255));

    int plotPosW = 0;  
    int plotPosH = 400-256;  
    int plotWidth = 640;
    int plotHeight= 128;
    float plotRange = 0.5f;
    Image<PixRGB<byte> > plotImage = multilinePlot(
        lines,
        plotWidth,plotHeight,
        0.0f,plotRange*2,
        "Trans velocity(v)","m/s","",
        linesColor,
        PixRGB<byte>(255,255,255),
        PixRGB<byte>(0,0,0)

        );
    inplacePaste(itsInfoImage,plotImage,Point2D<int>(plotPosW,plotPosH));

    plotPosH += plotHeight;
    std::vector<std::vector<float> > rotlines;
    rotlines.push_back(itsVelocityRotQue.getVector());
    rotlines.push_back(itsVelocityRotCmdQue.getVector());
    rotlines.push_back(itsVelocityRotRequestQue.getVector());

    Image<PixRGB<byte> > rotWimage = multilinePlot(
        rotlines,
        plotWidth,plotHeight,
        -plotRange,plotRange,
        "Rot velocity(w)","rad/s","",
        linesColor,
        PixRGB<byte>(255,255,255),
        PixRGB<byte>(0,0,0)
        );

    inplacePaste(itsInfoImage,rotWimage,Point2D<int>(plotPosW,plotPosH));

    //draw IMU meter
    //==========================================================
    int imuPosW = itsInfoImage.getWidth()-85;
    int imuPosH = itsInfoImage.getHeight()-260;
    //draw compass from imu yaw reading
    PixRGB<byte> imuRingColor(0,128,255);
    drawCircle(itsInfoImage, Point2D<int>(imuPosW,imuPosH), 45,    imuRingColor);
    drawHalfDisk(itsInfoImage, Point2D<int>(imuPosW,imuPosH), 43,  PixRGB<byte>(0,255,0),itsIMURoll);
    drawLine(itsInfoImage, Point2D<int>(imuPosW,imuPosH),itsIMUPosition.z, 90, PixRGB<byte>(255,0,0),3);
    drawLine(itsInfoImage, Point2D<int>(imuPosW-55,imuPosH),0, 20, imuRingColor);
    drawLine(itsInfoImage, Point2D<int>(imuPosW+55,imuPosH),0, 20, imuRingColor);
    drawLine(itsInfoImage, Point2D<int>(imuPosW,imuPosH-55),M_PI/2, 20, imuRingColor);
    drawLine(itsInfoImage, Point2D<int>(imuPosW,imuPosH+55),M_PI/2, 20, imuRingColor);
    sprintf(buffer, "%6.2f deg",(itsIMUPosition.z/M_PI)*180.0f);
    writeText(itsInfoImage, Point2D<int>(imuPosW-40,imuPosH-5), buffer, 
      PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8),true);
    if(itsIMUon){
      sprintf(buffer, "IMU:ON");
      writeText(itsInfoImage, Point2D<int>(imuPosW+35,imuPosH+40), buffer, 
        PixRGB<byte>(0,255,0), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
    }else{
      sprintf(buffer, "IMU:Off");
      writeText(itsInfoImage, Point2D<int>(imuPosW+35,imuPosW+40), buffer,
       PixRGB<byte>(255,0,0), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
    }
    //==========================================================
    itsDebugWin.show(itsInfoImage ,"Visualizer Map");



}
// ######################################################################
void BeoSim::publishScreenShot(Image<PixRGB<byte> > img)
{
  BeobotEvents::VisualizerMessagePtr msg = 
    new BeobotEvents::VisualizerMessage;
  msg->image     = Image2Ice(img); 
  msg->BeoAppID  = BEO_SIM;
  msg->RequestID = itsCurrentTimeIndex;
  this->publish("VisualizerMessageTopic", msg);
}
// ######################################################################
void BeoSim::updateGUI()
{
  if(itsDisplayTimer.getSecs() > itsDisplayUpdateRate)
  {
    itsDisplayTimer.reset();
    //Grab the last key pressed by the user
    // int key = itsMeterXwin.getLastKeyPress();

    double loc_forwardSpeed = itsForwardSpeed;
    double loc_rotationalSpeed = itsRotationalSpeed;
    //Update the display
    itsControlImage = Image<PixRGB<byte> >(512,256,ZEROS);
    char buffer[128];
    int speedHeight = 0;
    sprintf(buffer, "Fwr: %6.3f:%6.3f", loc_forwardSpeed,loc_forwardSpeed*itsTransCap);
    writeText(itsControlImage, Point2D<int>(0,speedHeight), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(12));

    speedHeight+=20;
    sprintf(buffer, "Rot: %6.3f:%6.3f", loc_rotationalSpeed, loc_rotationalSpeed *itsRotCap/100.0);
    writeText(itsControlImage, Point2D<int>(0,speedHeight), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(12));

    speedHeight+=20;
    sprintf(buffer, "TransCap: %3d  RotatCap %3d", (int)itsTransCap,(int)itsRotCap);
    writeText(itsControlImage, Point2D<int>(0,speedHeight), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

    speedHeight+=20;
    sprintf(buffer, "Motor T: %5.2f m/s  R:%5.2f rad/s ", itsTransVelocityCurrent,itsRotVelocityCurrent);
    writeText(itsControlImage, Point2D<int>(0,speedHeight), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(12));

    sprintf(buffer, "Right Encoder: %d Left Encoder: %d", itsRightEncoder,itsLeftEncoder);
    writeText(itsControlImage, Point2D<int>(0,90), buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

    int encHeightStart = 98;
    int encHeightSize = 12;
    int encHeightCurrent = encHeightStart;
    int encFontSize = 8;

    sprintf(buffer, "Enc X: %f m",itsPosition.x );
    writeText(itsControlImage, Point2D<int>(0,encHeightCurrent), buffer, 
      PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(encFontSize));

    encHeightCurrent+= encHeightSize;
    sprintf(buffer, "Enc Y: %f m",itsPosition.y );
    writeText(itsControlImage, Point2D<int>(0,encHeightCurrent), buffer, 
      PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(encFontSize+4));


    encHeightCurrent+= encHeightSize+8;
    sprintf(buffer, "Enc R: %f deg",(itsPosition.z/M_PI)*180.0f );
    writeText(itsControlImage, Point2D<int>(0,encHeightCurrent), buffer, 
      PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(encFontSize));

    encHeightCurrent+= encHeightSize;
    sprintf(buffer, "IMU X: %f m",itsIMUPosition.x );
    writeText(itsControlImage, Point2D<int>(0,encHeightCurrent), buffer, 
      PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(encFontSize));

    encHeightCurrent+= encHeightSize;
    sprintf(buffer, "IMU Y: %f m",itsIMUPosition.y );
    writeText(itsControlImage, Point2D<int>(0,encHeightCurrent), buffer, 
      PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(12));

    //draw compass from imu yaw reading
    PixRGB<byte> imuRingColor(0,128,255);
    drawCircle(itsControlImage, Point2D<int>(340,200), 45,    imuRingColor);
    drawHalfDisk(itsControlImage, Point2D<int>(340,200), 43,  PixRGB<byte>(0,255,0),itsIMURoll);
    drawLine(itsControlImage, Point2D<int>(340,200),itsIMUPosition.z, 90, PixRGB<byte>(255,0,0),3);
    drawLine(itsControlImage, Point2D<int>(340-55,200),0, 20, imuRingColor);
    drawLine(itsControlImage, Point2D<int>(340+55,200),0, 20, imuRingColor);
    drawLine(itsControlImage, Point2D<int>(340,200-55),M_PI/2, 20, imuRingColor);
    drawLine(itsControlImage, Point2D<int>(340,200+55),M_PI/2, 20, imuRingColor);
    sprintf(buffer, "%6.2f deg",(itsIMUPosition.z/M_PI)*180.0f);
    writeText(itsControlImage, Point2D<int>(300,195), buffer, 
      PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
    if(itsIMUon){
      sprintf(buffer, "IMU:ON");
      writeText(itsControlImage, Point2D<int>(375,240), buffer, 
        PixRGB<byte>(0,255,0), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
    }else{
      sprintf(buffer, "IMU:Off");
      writeText(itsControlImage, Point2D<int>(375,240), buffer,
       PixRGB<byte>(255,0,0), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

    }



    for(size_t i=0; i<itsChannelMeters.size(); i++)
    {

      sprintf(buffer, "Ch%lu: %d", i+1, itsRCChannels[i]);

      Point2D<int> pastePoint = Point2D<int>(
          int(itsChannelMeters[0].getWidth()*itsChannelMeters.size()*1.05+20),
          itsControlImage.getHeight() - itsChannelMeters[i].getHeight() - 10 + i*8
          );

      writeText(itsControlImage, pastePoint, buffer, 
        PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

      pastePoint = Point2D<int>(
          int(10+itsChannelMeters[i].getWidth()*i*1.05),
          itsControlImage.getHeight() - itsChannelMeters[i].getHeight() - 10
          );
      inplacePaste(itsControlImage, itsChannelMeters[i].render(itsRCChannels[i]), pastePoint );
    }
    SimpleMeter battMeter = SimpleMeter(20, 60, 22.0,24.5);
    Point2D<int> battPt = Point2D<int>( int(battMeter.getWidth()*1.05+200), 
                                          itsControlImage.getHeight() - battMeter.getHeight() - 10);
    inplacePaste(itsControlImage, battMeter.render(itsBattery), battPt);
      sprintf(buffer, "Volt:%4.1fV",  itsBattery);
      writeText(itsControlImage, Point2D<int>(210,170), buffer, 
        PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));


    sprintf(buffer, "Emergency Mode: %d", itsEmergencyMode);
    writeText(itsControlImage, Point2D<int>(256,88), buffer, 
      PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

    sprintf(buffer, "RemoteMode: %d", itsRemoteMode);
    writeText(itsControlImage, Point2D<int>(256,96), buffer, 
      PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

    if(itsLoadFromLogFile){
      sprintf(buffer, "Log Mode: %d", itsLogRemoteMode);
      writeText(itsControlImage, Point2D<int>(340,96), buffer, 
        PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));
    }
    sprintf(buffer, "Motor 1 Speed: %0.2f", itsMotor1Speed);
    writeText(itsControlImage, Point2D<int>(256,104), buffer, 
      PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

    sprintf(buffer, "Motor 2 Speed: %0.2f", itsMotor2Speed);
    writeText(itsControlImage, Point2D<int>(256,112), buffer, 
      PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

    sprintf(buffer, "RC Speed   : %0.2f -> %0.2f", itsRcTransSpeed,itsRcTransSpeedCapped);
    writeText(itsControlImage, Point2D<int>(256,120), buffer, 
      PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

    sprintf(buffer, "RC Rotation: %0.2f -> %0.2f", itsRcRotSpeed,itsRcRotSpeedCapped);
    writeText(itsControlImage, Point2D<int>(256,128), buffer, 
      PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

    //Draw the encoder-based trajectory map in red and green
    //float replayspeed = itsReplaySpeed.getVal();
    int w = itsMapImage.getWidth();
    //int h = itsMapImage.getHeight();
    double scaleRate = 0.9;
    //Point2D<int> drawPos(
    //    int(itsPosition.x*itsMapScale + itsMapCenter.i),
    //    int(itsPosition.y*itsMapScale +itsMapCenter.j)
    //    );
    //if(itsMapImage.coordsOk(drawPos))
    //{
    //  itsPositionMap.push_back(itsPosition);
    //}else
    //{
    //  //Find New Scale
    //  do{
    //    itsMapScale *= scaleRate;
    //    drawPos = Point2D<int>(
    //        int(itsPosition.x*itsMapScale + itsMapCenter.i),
    //        int(itsPosition.y*itsMapScale + itsMapCenter.j)
    //        );
    //  }while(!itsMapImage.coordsOk(drawPos) );
    //  reDrawMap(itsMapScale);                               
    //}
    //if(itsMapImage.coordsOk(drawPos))
    //  itsMapImage.setVal(drawPos, itsPosition.color);


    itsDrawMapImage = Image<PixRGB<byte> >(itsMapImage);
    //drawCircle(itsDrawMapImage, drawPos, 9, PixRGB<byte>(255,255,255));
    //drawLine(itsDrawMapImage, drawPos,-itsPosition.z, 9, PixRGB<byte>(255,255,255));


    //draw IMU odometry     
    Point2D<int> drawPosIMU(
        int(itsIMUPosition.x*itsMapScale + itsMapCenter.i),
        int(itsIMUPosition.y*itsMapScale +itsMapCenter.j)
        );
    if(itsMapImage.coordsOk(drawPosIMU))
    {
      //itsIMUPositionMap.push_back(itsIMUPosition);
    }else
    {
      //Find New Scale
      do{
        itsMapScale *= scaleRate;
        drawPosIMU = Point2D<int>(
            int(itsIMUPosition.x*itsMapScale + itsMapCenter.i),
            int(itsIMUPosition.y*itsMapScale + itsMapCenter.j)
            );
      }while(!itsMapImage.coordsOk(drawPosIMU) );
      reDrawMap(itsMapScale);                               
    }
    if(itsMapImage.coordsOk(drawPosIMU))
    {
      if(itsLoadFromLogFile)
        drawCircle(itsMapImage, drawPosIMU, 2, itsIMUPosition.color);
      itsMapImage.setVal(drawPosIMU, itsIMUPosition.color);
    }
    for(int i = 0;i< (int)itsLRFCurrentMap.size();i++)
    {
      Point2D<int>drawPosLRF = Point2D<int>(
          int(itsLRFCurrentMap[i].x*itsMapScale + itsMapCenter.i),
          int(itsLRFCurrentMap[i].y*itsMapScale + itsMapCenter.j)
          );
      if(itsMapImage.coordsOk(drawPosLRF))
      {
        itsMapImage.setVal(drawPosLRF, PixRGB<byte>(0,255,0));

      }
    }

    drawCircle(itsDrawMapImage, drawPosIMU, 9, PixRGB<byte>(255,0,255));
    drawLine(itsDrawMapImage, drawPosIMU,-itsIMUPosition.z, 9, PixRGB<byte>(255,0,255));

    if(itsLoadFromLogFile ){

      //boundingBox();
      if(!itsIMUon)
      {
        Point2D<int> drawPos2(
            int(itsLogPosition.x*itsMapScale + itsMapCenter.i),
            int(itsLogPosition.y*itsMapScale + itsMapCenter.j)
            );
        if(!itsMapImage.coordsOk(drawPos2))
        {
          //Find New Scale
          do{
            itsMapScale *= scaleRate;
            drawPos2 = Point2D<int>(
                int(itsLogPosition.x*itsMapScale + itsMapCenter.i),
                int(itsLogPosition.y*itsMapScale + itsMapCenter.j)
                );
          }while(!itsMapImage.coordsOk(drawPos2) );
          LINFO("Curren MapScale %f",itsMapScale);
          reDrawMap(itsMapScale);
        }
        if(itsMapImage.coordsOk(drawPos2)){
          itsMapImage.setVal(drawPos2, itsLogPosition.color);
          drawCircle(itsMapImage, drawPos2, 2, itsLogPosition.color);//make line thicker
          drawCircle(itsDrawMapImage, drawPos2, 9, PixRGB<byte>(0,255,0));
          drawLine(itsDrawMapImage, drawPos2,-itsLogPosition.z, 9, PixRGB<byte>(128,128,255));
        }
      }
      //redraw it every time since we fastward the speed
      if(itsReplaySpeed != 1)
        reDrawMap(itsMapScale);

    }
    if(itsLoadFromLogFile){
      int offset = 45;//MAPWINDOW_H;
      itsTravelDistance = itsTravelDistanceAuto+itsTravelDistanceManual;
      double autoRate = 0.0;
      if(itsTravelDistance!=0.0) autoRate = itsTravelDistanceAuto/itsTravelDistance;
      sprintf(buffer, "Travel       : %fm Auto %5.2f%%",itsTravelDistance,autoRate*100.0 );
      writeText(itsDrawMapImage, Point2D<int>(0,offset-15), buffer, 
        PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
      sprintf(buffer, "Travel Auto  : %fm",itsTravelDistanceAuto );
      writeText(itsDrawMapImage, Point2D<int>(0,offset-30), buffer, 
        PixRGB<byte>(0,255,0), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
      sprintf(buffer, "Travel Manual: %fm",itsTravelDistanceManual );
      writeText(itsDrawMapImage, Point2D<int>(0,offset-45), buffer, 
        PixRGB<byte>(255,0,0), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

      sprintf(buffer, "LEn X: %fm",itsLogPosition.x );
      writeText(itsDrawMapImage, Point2D<int>(320,offset-45), buffer, 
        PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

      sprintf(buffer, "LEn Y: %fm",itsLogPosition.y );
      writeText(itsDrawMapImage, Point2D<int>(320,offset-30), buffer, 
        PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

      sprintf(buffer, "LEn R: %f",itsLogPosition.z );
      writeText(itsDrawMapImage, Point2D<int>(320,offset-15), buffer, 
        PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

      sprintf(buffer, "Play Speed:%3dX",itsReplaySpeed );
      writeText(itsDrawMapImage, Point2D<int>(w-120,offset-45), buffer, 
        PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

      //sprintf(buffer, "IMU R: %f",itsIMUheading );
      //writeText(itsDrawMapImage, Point2D<int>(320,offset-15), buffer, 
      //  PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
    }
    inplacePaste(itsDispImage,itsControlImage,Point2D<int>(0,0));

    inplacePaste(itsDispImage,itsDrawMapImage,Point2D<int>(0,256));

    std::vector<std::vector<float> > lines;
    lines.push_back(itsVelocityTransQue.getVector());
    lines.push_back(itsVelocityTransCmdQue.getVector());
    lines.push_back(itsVelocityTransRequestQue.getVector());

    std::vector<PixRGB<byte> > linesColor;
    linesColor.push_back(PixRGB<byte>(255,0,0));
    linesColor.push_back(PixRGB<byte>(255,165,0));
    linesColor.push_back(PixRGB<byte>(0,0,255));

    int plotPosW = 512-70;  
    int plotWidth = itsDispImage.getWidth()-itsControlImage.getWidth()-itsDispCameraImage.getWidth()+110;
    int plotHeight= 128;
    float plotRange = 0.5f;
    Image<PixRGB<byte> > plotImage = multilinePlot(
        lines,
        plotWidth,plotHeight,
        -plotRange*4,plotRange*4,
        "Trans velocity","m/s","",
        linesColor,
        PixRGB<byte>(255,255,255),
        PixRGB<byte>(0,0,0)

        );
    inplacePaste(itsDispImage,plotImage,Point2D<int>(plotPosW,0));

    std::vector<std::vector<float> > rotlines;
    rotlines.push_back(itsVelocityRotQue.getVector());
    rotlines.push_back(itsVelocityRotCmdQue.getVector());
    rotlines.push_back(itsVelocityRotRequestQue.getVector());

    Image<PixRGB<byte> > rotWimage = multilinePlot(
        rotlines,
        plotWidth,plotHeight,
        -plotRange,plotRange,
        "Rot velocity(w)","rad/s","",
        linesColor,
        PixRGB<byte>(255,255,255),
        PixRGB<byte>(0,0,0)
        );

    inplacePaste(itsDispImage,rotWimage,Point2D<int>(plotPosW,plotHeight));

    sprintf(buffer, "%5d",itsLogCameraFrameID );
    writeText(itsDispCameraImage, Point2D<int>(0,0), buffer, 
      PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

    inplacePaste(itsDispImage,itsDispCameraImage,Point2D<int>(itsDispImage.getWidth()-itsDispCameraImage.getWidth(),0));


    //Draw Divide Line
    drawLine(itsDispImage,Point2D<int>(0,255),Point2D<int>(itsDispImage.getWidth(),255),PixRGB<byte>(255,255,255));//HR
    drawLine(itsDispImage,Point2D<int>(512-70,0),Point2D<int>(512-70,255),PixRGB<byte>(255,255,255));//VT

    int diw = itsDispImage.getWidth();
    int dih = itsDispImage.getHeight();
    //Draw progress bar 
    double progress = (double)itsCurrentTimeIndex / (double)itsLogData.size();

    drawLine(itsDispImage,Point2D<int>(0,dih-8),Point2D<int>(diw,dih-8),PixRGB<byte>(255,255,255),5);
    drawLine(itsDispImage,Point2D<int>(0,dih-8),Point2D<int>(diw*progress,dih-8),PixRGB<byte>(0,0,255),5);
    sprintf(buffer, "%5d/%5d",itsCurrentTimeIndex,(int)itsLogData.size());    

    //make text float until the end
    Point2D<int> textPos(diw-90,dih-13);
    if(progress <= 0.9)      
      textPos = Point2D<int>(diw*progress-10,dih-13);
    writeText(itsDispImage, textPos, buffer, PixRGB<byte>(255,0,0), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8),true);
  

    itsMeterXwin.drawImage(itsDispImage);

  }
}
// ######################################################################
void BeoSim::reDrawMap(double scale)
{

  //Clean up the old images
  itsMapImage = Image<PixRGB<byte> >(MAPWINDOW_W,MAPWINDOW_H,ZEROS);

  //int w = itsMapImage.getWidth();
  //int h = itsMapImage.getHeight();

  //Redraw all points with new scale
  if(!itsIMUon)
  {
    for(int i = 0;i < (int)itsLogPositionMap.size();i++)
    {
      Point2D<int> tmp(

          int(itsLogPositionMap[i].x*scale + itsMapCenter.i),
          int(itsLogPositionMap[i].y*scale + itsMapCenter.j)
          );
      if(itsMapImage.coordsOk(tmp)){
        drawCircle(itsMapImage, tmp, 2, PixRGB<byte>(itsLogPositionMap[i].color ));
        itsMapImage.setVal(tmp, itsLogPositionMap[i].color);
      }
    }
  }
  if(!itsIMUon)
  {
    LINFO("itsPositionMap.size is %d",(int)itsPositionMap.size());
    for(int i = 0;i < (int)itsPositionMap.size();i++)
    {

      Point2D<int> tmp(

          int(itsPositionMap[i].x*scale + itsMapCenter.i),
          int(itsPositionMap[i].y*scale + itsMapCenter.j)
          );
      if(itsMapImage.coordsOk(tmp))
        itsMapImage.setVal(tmp, itsPositionMap[i].color);
    }
  }
  for(int i = 0;i < (int)itsIMUPositionMap.size();i++)
  {
    Point2D<int> tmp(

        int(itsIMUPositionMap[i].x*scale + itsMapCenter.i),
        int(itsIMUPositionMap[i].y*scale + itsMapCenter.j)
        );
    if(itsMapImage.coordsOk(tmp))
      if(itsLoadFromLogFile)
        drawCircle(itsMapImage, tmp, 2, itsIMUPositionMap[i].color);
    itsMapImage.setVal(tmp, itsIMUPositionMap[i].color);
  }
  for(int i = 0;i < (int)itsLRFPositionMap.size();i++)
  {
    Point2D<int> tmp(

        int(itsLRFPositionMap[i].x*scale + itsMapCenter.i),
        int(itsLRFPositionMap[i].y*scale + itsMapCenter.j)
        );
    if(itsMapImage.coordsOk(tmp))
      itsMapImage.setVal(tmp, itsLRFPositionMap[i].color);

  }

}
// ######################################################################
void BeoSim::updateLRFMap(Point3DwithColor<double>loc,std::vector<double> dist)
{
  int size = (int)dist.size();
  int center = size/2;//this is front angle 
  itsLRFCurrentMap.clear();
  for(int i = size-90; i< size+90 ;i++)
  {
    double lrfDist = dist[i];
    if(lrfDist != -1)
    {
      double lrfAng = ((i-center)/180.0)*M_PI;
      double robotAng = loc.z;
      double angle = robotAng+lrfAng + (M_PI/4)+ M_PI;
      double dy = lrfDist * sin(angle); 
      double dx = lrfDist * cos(angle);
      Point3DwithColor<double> p(loc.x+dx,loc.y+dy,angle);
      p.color = PixRGB<byte>(128,128,0);//set lrf dot yellow;
      itsLRFPositionMap.push_back(p); 
      itsLRFCurrentMap.push_back(p);
    }
  }


}
// ######################################################################
void BeoSim::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{
}

// ######################################################################
void BeoSim::outputMap()
{
  // get the time of day
  char buf[255];
  printf("Please Enter Tag Name:");
  int ret = scanf("%s",buf);
  std::string file( sformat("%s_%s.txt", itsOutputFile.c_str(), buf));
  ret = 0;

  // save  in a file by appending to the file
  FILE *rFile = fopen(file.c_str(), "at");
  if (rFile != NULL)
  {
    //fputs(sformat("############# start #############\n").c_str(), rFile);
    for(int i = 0;i < (int)itsIMUPositionMap.size();i++)
    {
      Point3DwithColor<double> p = itsIMUPositionMap[i];
      int remoteMode = (p.color == PixRGB<byte>(0,0,255)? 1:0);
      std::string line( 
          sformat("%4d,%15.10f,%15.10f,%15.10f,%d\n",
            i,p.x,p.y,p.z,remoteMode).c_str() );
      fputs(line.c_str(), rFile);
      printf("%4d,%15.10f,%15.10f,%15.10f,%d\n",i,p.x,p.y,p.z,remoteMode);
    }
    //fputs(sformat("############ end ##############").c_str(), rFile);
    fclose (rFile);
    LINFO("Save File: %s",file.c_str());
  }
  else LFATAL("can't create file: %s", file.c_str());
}
// ######################################################################
void BeoSim::publishMotor(Beobot2::motorData motor)
{

    //Report the current motor speeds to IceStorm
    BeobotEvents::MotorMessagePtr msg = new BeobotEvents::MotorMessage;

    msg->RequestID = itsCurrMessageID++;


    msg->transVel = motor.transVel;
    msg->rotVel   = motor.rotVel;

    msg->encoderX   = motor.encoderX;
    msg->encoderY   = motor.encoderY;
    msg->encoderOri = motor.encoderOri;
    if(itsIMUon)
      msg->imuHeading = itsIMUDiffPosition.z;
    else
      msg->imuHeading = -2*M_PI;// imu not available, sending invalid value 

    msg->rcTransVel = motor.rcTransVel;
    msg->rcRotVel   = motor.rcRotVel;
    msg->rcTransCap = motor.rcTransCap;
    msg->rcRotCap   = motor.rcRotCap;
    msg->rcMode     = motor.rcMode;

    msg->robotTransVel = motor.robotTransVel;
    msg->robotRotVel   = motor.robotRotVel; 

    msg->trajLen    = itsTravelDistanceAuto + itsTravelDistanceManual ;//total move length
    msg->dist       = itsStraightDistance;//distance between orgin and current
    this->publish("MotorMessageTopic", msg);

}   

// ######################################################################
void BeoSim::publishLRF(Beobot2::lrfData lrf)
{

  // LRFMessage
  BeobotEvents::LRFMessagePtr msg = new BeobotEvents::LRFMessage;

  msg->RequestID = itsCurrMessageID++;

  msg->distances.resize(lrf.dist.size());
  msg->angles.resize(lrf.dist.size());

  msg->LRFID = lrf.deviceID;

  //LINFO("ID %d LRF size %d",lrf.deviceID,(int)lrf.dist.size());
  for(uint i = 0; i < lrf.dist.size(); i++)
  {
    msg->distances[i] = lrf.dist[i]*1000.0;//convert from meter to mm
    msg->angles   [i] = i;
  }

  publish("LRFMessageTopic", msg);
}

// ######################################################################
void BeoSim::publishCamera(Beobot2::imageData cam,Image<PixRGB<byte> > img)
{
  BeobotEvents::CameraMessagePtr msg =
    new BeobotEvents::CameraMessage;

  // store the input image and related info
  msg->cameraID  = cam.cameraID;
  msg->RequestID = cam.frameID;
  msg->image     = Image2Ice(img);

  // publish the message
  publish("CameraMessageTopic", msg);
}
// ######################################################################
void BeoSim::publishIMU(Beobot2::imuData imu)
{
      BeobotEvents::IMUMessagePtr msg =
        new BeobotEvents::IMUMessage;

      msg->RequestID = itsCurrMessageID++;
      msg->roll  = imu.roll;
      msg->pitch = imu.pitch;
      msg->yaw   = imu.yaw;

      msg->validRollPitchYaw = true;
      msg->validAccAndAng    = false;
      msg->validMagnetometer = false;

      publish("IMUMessageTopic", msg);
}


// ######################################################################
void BeoSim::getCurrentLogData()
{
  int logSize = (int)itsLogData.size();

  if(logSize == 0 ||itsCurrentTimeIndex > logSize) {
    return;
  }

  double time = itsLogTimer.get()/1000.0 + itsLogData[0].time;//FIXXX
  double logTime = itsLogData[itsCurrentTimeIndex].time;

  //LINFO("Load next log data logSize %d cti %d time %7.2f ltime %7.2f",logSize,itsCurrentTimeIndex,time,logTime);


  bool realtime = false;
  //LINFO("Replay at speed %d",itsReplaySpeed);
  for(int i = 0;i < itsReplaySpeed;i++)
  {
    if(itsReplaySpeed==1 && realtime)
    {
      LINFO("Speed is 1, Sync With Current Timer");
      do{
        logTime = itsLogData[itsCurrentTimeIndex].time;
        LINFO("Current cti is [%d]current time[%f],log time[%f]\n",
            itsCurrentTimeIndex, time, logTime);  

        if(logTime < time && itsCurrentTimeIndex < logSize-1)
        {
          itsCurrentTimeIndex++;
          logTime = itsLogData[itsCurrentTimeIndex].time;
        }

      }while(logTime < time && itsCurrentTimeIndex < logSize-1);
    }else{
          itsCurrentTimeIndex++;
          logTime = itsLogData[itsCurrentTimeIndex].time;
    }



  //LINFO("Replay at speed %d i = %d",itsReplaySpeed,i);
   // none real-time render
   // if(itsReplaySpeed !=0 && itsCurrentTimeIndex < logSize)
   //   itsCurrentTimeIndex++;

    //sync imu data time
    int logImuSize = (int)itsLogIMUData.size();
    if(logImuSize != 0)
    {
      double logImuTime = itsLogIMUData[itsLogIMUTimeIndex].time;
      //LINFO("Sync IMU size %d it %f",logImuSize,logImuTime);
      do
      {
        if(logImuTime < logTime && itsLogIMUTimeIndex < logImuSize -1)
        {
          itsLogIMUTimeIndex++;
          logImuTime = itsLogIMUData[itsLogIMUTimeIndex].time;
        }
      }while(logImuTime < logTime && itsLogIMUTimeIndex < logImuSize-1);

      Beobot2::imuData imudata = itsLogIMUData[itsLogIMUTimeIndex];
      itsIMURoll = imudata.roll;
      itsIMUPitch = imudata.pitch;
      itsIMUheading = imudata.yaw;
      LDEBUG("Found Index[%d] Time: %5.2f with Yaw[%4.2f]",itsLogIMUTimeIndex,imudata.time,imudata.yaw);
      publishIMU(imudata);
    }
    //sync LRF data
    int logLrfSize = (int)itsLogFrontLRFData.size();
    if(logLrfSize != 0)
    {
      double logLrfTime = itsLogFrontLRFData[itsLogFrontLRFTimeIndex].time;
      LINFO("Sync LRF size %d it %f",logLrfSize,logLrfTime);
      do
      {
        if(logLrfTime < logTime && itsLogFrontLRFTimeIndex < logLrfSize -1)
        {
                itsLogFrontLRFTimeIndex++;
                logLrfTime = itsLogFrontLRFData[itsLogFrontLRFTimeIndex].time;
        }
      }while(logLrfTime < logTime && itsLogFrontLRFTimeIndex < logLrfSize-1);

      Beobot2::lrfData lrfdata = itsLogFrontLRFData[itsLogFrontLRFTimeIndex];
      LINFO("Found Index[%d] Time: %5.2f with front lrf readings[%3d]",
          itsLogFrontLRFTimeIndex,lrfdata.time, (int)lrfdata.dist.size());
      publishLRF(lrfdata);
    }

    //synic MOTOR data
    Beobot2::motorData tmp = itsLogData[itsCurrentTimeIndex];
    //LINFO("%5d Sync motor cti %d time %f",i,itsCurrentTimeIndex,tmp.time);

    if(tmp.rcMode == 1)
    {
      itsForwardSpeed = tmp.rcTransVel;
      itsRotationalSpeed = tmp.rcRotVel;
    }else 
    {
      itsForwardSpeed = tmp.transVel;
      itsRotationalSpeed = tmp.rotVel;
    }
    //Point2D<double> rotPt = rotZ(tmp.encoderX,tmp.encoderY,ORI_OFFSET);
    itsLogDiffPosition.x = tmp.encoderX;
    itsLogDiffPosition.y = tmp.encoderY;
    itsLogDiffPosition.z = tmp.encoderOri;

    //itsLogDiffPosition.x = tmp.encoderX* cosOri - tmp.encoderY* sinOri;
    //itsLogDiffPosition.y = tmp.encoderX* sinOri + tmp.encoderY* cosOri;
    //itsLogDiffPosition.z = tmp.encoderOri;

    itsLogPosition.x += itsLogDiffPosition.x;
    itsLogPosition.y += itsLogDiffPosition.y;
    itsLogPosition.z = itsLogDiffPosition.z;

    itsRcTransSpeed = tmp.rcTransVel;
    itsRcRotSpeed = tmp.rcRotVel  ;
    itsTransCap = tmp.rcTransCap;
    itsRotCap = tmp.rcRotCap;
    itsRcTransSpeedCapped = itsRcTransSpeed * itsTransCap /100.0;
    itsRcRotSpeedCapped= itsRcRotSpeed* itsRotCap /100.0;
    itsVelocityTransCmdQue.add(itsRcTransSpeedCapped * RC2VEL);
    itsVelocityRotCmdQue.add(itsRcRotSpeedCapped * RC2VEL);
    itsVelocityTransRequestQue.add(itsForwardSpeed*itsTransCap/100.0 * RC2VEL );
    itsVelocityRotRequestQue.add(itsRotationalSpeed*itsRotCap/100.0 * RC2VEL );

    itsTransVelocityCurrent = tmp.robotTransVel;
    itsRotVelocityCurrent = tmp.robotRotVel;


    itsVelocityTransQue.add(itsTransVelocityCurrent);
    itsVelocityRotQue.add(itsRotVelocityCurrent);

    itsLogRemoteMode = tmp.rcMode  ;


    itsRCChannels[0] = itsRcRotSpeed*100;
    itsRCChannels[1] = itsRcTransSpeed*100;
    itsRCChannels[2] = itsTransCap;
    itsRCChannels[5] = itsRotCap;
    itsRCChannels[6] = itsLogRemoteMode*30;
    itsBattery = tmp.battery;

    double dx = itsLogDiffPosition.x;
    double dy = itsLogDiffPosition.y;
    double dt = sqrt(dx*dx +dy*dy); 
    updateIMUPosition(dt);
    if(itsLogRemoteMode == 1){
      itsTravelDistanceManual +=dt; 
    }
    if(itsLogRemoteMode == 3){
      itsTravelDistanceAuto+=dt;  
    }
    //push current location into map
    itsIMUPositionMap.push_back(itsIMUPosition);

    itsMotor1Speed = itsForwardSpeed - itsRotationalSpeed;
    itsMotor2Speed = itsForwardSpeed + itsRotationalSpeed;
    if(itsLogRemoteMode == 1)
      itsLogPosition.color = PixRGB<byte>(255,0,0);
    else
      itsLogPosition.color = PixRGB<byte>(0,255,0);

    if(itsCurrentTimeIndex <(int)itsLogData.size())
      itsLogPositionMap.push_back(itsLogPosition);
    publishMotor(tmp);
  }

  //sync camera data time
  int logImageSize = (int) itsLogImageData.size();
  if(logImageSize != 0){
    double logImageTime = itsLogImageData[itsLogCameraTimeIndex].time;
    do{
      if(logImageTime < logTime && itsLogCameraTimeIndex< logImageSize-1)
      {
        itsLogCameraTimeIndex++;
        logImageTime = itsLogImageData[itsLogCameraTimeIndex].time;
      }

    }while(logImageTime < logTime && itsLogCameraTimeIndex< logImageSize-1);

    Beobot2::imageData img = itsLogImageData[itsLogCameraTimeIndex];
    itsLogCameraFrameID = img.frameID;
    if(file_exists(img.imagePath.c_str()))
    {
      Image<PixRGB<byte> > rawImg = Raster::ReadRGB(img.imagePath);
      itsDispCameraImage = rescale(rawImg ,320,240);
      char buffer[255];
      sprintf(buffer, "%s",img.fileName.c_str());
      writeText(itsDispCameraImage, Point2D<int>(0,220), buffer, 
          PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
      publishCamera(img,rawImg);
    }
  }
  if(itsLogFrontLRFData.size() != 0){
    Beobot2::lrfData lrfdata = itsLogFrontLRFData[itsLogFrontLRFTimeIndex];
    updateLRFMap(itsIMUPosition,lrfdata.dist);
  }
  if(itsCurrentTimeIndex > logSize)
    outputMap();//FIXXX will output coord file at end of loading
}
// ######################################################################
Point3D<double> BeoSim::tf(Point3D<double> p,double x,double y,double z,double roll,double pitch,double yaw)
{

  double a = pitch,b = yaw,c = roll;

  Image<double> R = Image<double>(3,3,ZEROS);
  R.setVal(0,0, cos(a)*cos(b)); R.setVal(1,0,cos(a)*sin(b)*c-sin(a)*cos(c));  R.setVal(2,0,cos(a)*sin(b)*cos(c)+sin(a)*sin(c));
  R.setVal(0,1, sin(a)*cos(b)); R.setVal(1,1,sin(a)*sin(b)*c+cos(a)*cos(c));  R.setVal(2,1,sin(a)*sin(b)*cos(c)-cos(a)*sin(c));
  R.setVal(0,2,-sin(b));        R.setVal(1,2,cos(b)*sin(c));                  R.setVal(2,2,cos(b)*cos(c));

  Image<double> pt  = Image<double>(1,3,ZEROS);
  pt.setVal(0,0,x);
  pt.setVal(0,1,y);
  pt.setVal(0,2,z);

  //Image<double> pt = pt2matrix(p);//convert Point3D to Matrix format
  Image<double> ptr = matrixMult(R,pt);
  Point3D<double> T = Point3D<double>(x,y,z);
  Point3D<double> result = Point3D<double>(
      ptr.getVal(0,0),
      ptr.getVal(0,1),
      ptr.getVal(0,2))+T;

  return result;
}

// ######################################################################
Point2D<double> BeoSim::rotZ(double x,double y,double ori)
{
  Point3D<double> result = tf(Point3D<double>(x,y,0),0,0,0,0.0,ori,0.0);
  LINFO("%f,%f rot %f become +> %f,%f",x,y,ori,result.x,result.y);

  return Point2D<double>(result.x,result.y);

}



/*!@file Robots2/Beobot2/Hardware/BeoSim.H Beobot2.0 simulator    */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Hardware/BeoSim.H
// $ $Id: BeoSim.H 15190 2012-02-29 04:00:13Z kai $
//
//////////////////////////////////////////////////////////////////////////
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "Image/Point3D.H"
#include "GUI/XWinManaged.H"
#include "GUI/ImageDisplayStream.H"
#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Util/Timer.H"
#include "GUI/SimpleMeter.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include <IceUtil/Thread.h>

#include <vector>
#include <deque>
#include <list>

#include "Devices/Serial.H"
#include "Controllers/PID.H"
#include "Robots/Beobot2/BeoCommon.H"


#ifndef BEOSIM_H
#define BEOSIM_H
//! The BeoSim module is responsible for actually driving the BeoBot2.  All motor
//! requests are serviced to hardware by this module through MotorRequest messages.
class BeoSim : public RobotBrainComponent
{
  public:

    BeoSim(int id, OptionManager& mgr,
        const std::string& descrName = "BeoSim",
        const std::string& tagName = "BeoSim");

    ~BeoSim();

    //! Evolve keeps the motor driver propeller up to date on the latest motor speed requests.
    //! If the evolve loop ever hangs for more than 1/4 second, then the propeller will shut down
    //! the motors until it recieves a new message.
    virtual void evolve();

    virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
        const Ice::Current&);

    virtual void registerTopics();

    void publishMotor(Beobot2::motorData motor);
    void publishLRF(Beobot2::lrfData lrf);
    void publishCamera(Beobot2::imageData cam,Image<PixRGB<byte> > img);
    void publishIMU(Beobot2::imuData imu);
    void publishScreenShot(Image<PixRGB<byte> > img);
    
    void checkLogFile();
    //! Check the serial port to ensure that we are connected to a 'motorboard' propeller,
    //! and also check the sanity of our command line options

    void loadFromLogFile();
    void start3();

    //Draw the updated local debugging GUI
    void updateGUI();
    void createVisualGUI();

    //Send the motor speeds ([-1 .. 1]) to the motordriver/propeller
    void SetMotors(float motor1speed, float motor2speed);

    //Reset the encoder value
    void resetEncoder();

    //Update position after get odometry data
    void UpdatePosition(void);
    void updateIMUPosition(double dt);
    void updateLRFMap(Point3DwithColor<double>loc,std::vector<double> dist);

    //Get Log Data from current time
    void getCurrentLogData(void);

    //Redraw map base on the current scale
    void reDrawMap(double scale);

    //Find the odometry boundary box
    void boundingBox();

    //Draw all point at map
    void drawAllMap(void);
    int file_exists(const char * fileName)
    {
      struct stat buf;
      int i = stat(fileName,&buf);
      if(i == 0)
        return 1;
      return 0;


    }
    void reset();
    void handleUserEvent();
    void saveScreenShot();
    Point3D<double> tf(Point3D<double> p,double x,double y,double z,double roll,double pitch,double yaw);
    Point2D<double> rotZ(double x,double y,double ori);

    int bfile_exists(const char * fileName)
    {
      struct stat buf;
      int i = stat(fileName,&buf);
      if(i == 0)
        return 1;
      return 0;

    }
    void outputMap();
  private:
    DebugWin itsDebugWin;

    int itsCurrMessageID;

    //////////////////////////////////////////
    //Log Data Structures
    //////////////////////////////////////////
    std::vector<Beobot2::motorData> itsLogData;
    std::vector<Beobot2::imageData> itsLogImageData;

    //for multiple cameras
    std::vector<std::vector<Beobot2::imageData> >itsLogCameras;

    std::vector<Beobot2::imuData> itsLogIMUData;
    std::vector<Beobot2::lrfData> itsLogFrontLRFData;
    std::vector<Beobot2::lrfData> itsLogDownLRFData;
    int itsCurrentTimeIndex;
    int itsLogCameraTimeIndex;
    int itsLogIMUTimeIndex;
    int itsLogFrontLRFTimeIndex;
    int itsLogDownLRFTimeIndex;
    Timer itsLogTimer;
    bool itsLoadFromLogFile;
    int itsLogRemoteMode;
    double itsLogLeftMotorVelocity;
    double itsLogRightMotorVelocity;
    //Point3D<double> itsLogPosition;
    Point3DwithColor<double> itsLogPosition;
    Point3DwithColor<double> itsLogDiffPosition;
    int itsLogCameraFrameID;
    std::string itsLogFolderName;


    //////////////////////////////////////////
    //Motor Data Structures
    //////////////////////////////////////////
    IceUtil::Mutex itsSpeedMutex;
    double itsForwardSpeedReq;
    double itsRotationalSpeedReq;
    double itsForwardSpeed;
    double itsRotationalSpeed;

    double itsRcTransSpeed;
    double itsRcRotSpeed;
    double itsRcTransSpeedCapped;
    double itsRcRotSpeedCapped;

    double itsTransCap;
    double itsRotCap;

    std::vector<int> itsRCChannels;
    int itsEmergencyMode;
    int itsRemoteMode;
    float itsMotor1Speed;
    float itsMotor2Speed;
    int itsLeftEncoder;
    int itsRightEncoder;
    int itsLastLeftEncoder;
    int itsLastRightEncoder;
    Point3DwithColor<double> itsPosition;
    Point3DwithColor<double> itsDiffPosition;
    Point3DwithColor<double> itsIMUDiffPosition;

    Image<PixRGB<byte> > itsMapImage;//Raw Trj
    Image<PixRGB<byte> > itsDrawMapImage;//With circle and text
    Image<PixRGB<byte> > itsInfoImage;//Info Panel for visualizer
    Image<PixRGB<byte> > itsTrajImage;//trajectory for visualizer

    Point2D<int> itsMapCenter;
    Timer itsEncoderTimer;
    double itsLeftMotorVelocity;
    double itsRightMotorVelocity;
    double itsRotVelocityCurrent;//for pid
    double itsTransVelocityCurrent;//for pid
    double itsRotVelocityTarget;//for pid
    double itsTransVelocityTarget;//for pid
    double itsTravelDistance;//trajectory length in meter
    double itsTravelDistanceAuto;
    double itsTravelDistanceManual;
    double itsStraightDistance;//straight distance from origin to current location
    double itsBattery;

    PID<float> itsPidRot;
    PID<float> itsPidTrans;

    //////////////////////////////////////////
    //Sonar Data Structures
    //////////////////////////////////////////
    IceUtil::Mutex itsSonarMutex;
    std::vector<double> itsSonarReadings;
    std::vector<double> itsSonarAngles;
    Timer itsSonarTimer;
    bool  itsSonarProxAlert;                  //  Are we in a sonar proximity alert state?
    float itsSonarTimeout;                    //  The number of seconds allowed to elapse between
    //sonar readings before we kill the motors

    //////////////////////////////////////////
    //IMU Data Structures
    //////////////////////////////////////////
    IceUtil::Mutex itsIMUMutex;
    double itsIMUheading;//imu yaw
    double itsIMUheadingInit;//initial imu yaw
    double itsIMURoll;//imu roll
    double itsIMUPitch;//imu pitch
    bool itsIMUon;
    Point3DwithColor<double> itsIMUPosition;



    //////////////////////////////////////////
    //Display Data Structures
    //////////////////////////////////////////
    Image<PixRGB<byte> > itsControlImage;
    Image<PixRGB<byte> > itsDispImage;
    Image<PixRGB<byte> > itsDispCameraImage;
    bool itsWaitScreen;
    XWinManaged itsMeterXwin;
    //        XWinManaged itsMapXwin;
    Timer itsDisplayTimer;
    float itsDisplayUpdateRate;
    float itsMapScale;
    std::vector<SimpleMeter> itsChannelMeters;
    velocityQue<float>        itsVelocityTransQue;//current robot trans speed
    velocityQue<float>        itsVelocityRotQue;//current robot rot speed
    velocityQue<float>        itsVelocityTransCmdQue;//current rc/cmd trans speed(capped)
    velocityQue<float>        itsVelocityRotCmdQue;//current rc/cmd rot speed(capped)
    velocityQue<float>    itsVelocityTransRequestQue;
    velocityQue<float>    itsVelocityRotRequestQue;

    std::vector<Point3DwithColor<double> >itsLogPositionMap;
    std::vector<Point3DwithColor<double> >itsPositionMap;
    std::vector<Point3DwithColor<double> >itsIMUPositionMap;
    std::vector<Point3DwithColor<double> >itsLRFPositionMap;
    std::vector<Point3DwithColor<double> >itsLRFCurrentMap;

    std::string       itsOutputFile;
    /////////////////////////////////////////
    //System Parameters
    /////////////////////////////////////////
    OModelParam<float>       itsMinimumSafeDistance;
    OModelParam<float>       itsMaxSpeed;
    OModelParam<bool>        itsIgnoreSonar;
    OModelParam<std::string> itsLogFile;
    OModelParam<int>       itsReplaySpeedOpt;
    int itsReplaySpeed;

};
#endif

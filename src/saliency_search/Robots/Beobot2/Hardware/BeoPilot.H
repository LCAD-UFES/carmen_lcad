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

#ifndef BEOPILOT_H
#define BEOPILOT_H

//! The BeoPilot module is responsible for actually driving the BeoBot2.  All motor
//! requests are serviced to hardware by this module through MotorRequest messages.
class BeoPilot : public RobotBrainComponent
{
  public:

    BeoPilot(int id, OptionManager& mgr,
        const std::string& descrName = "BeoPilot",
        const std::string& tagName = "BeoPilot");

    ~BeoPilot();

    //! Evolve keeps the motor driver propeller up to date on the latest motor speed requests.
    //! If the evolve loop ever hangs for more than 1/4 second, then the propeller will shut down
    //! the motors until it recieves a new message.
    virtual void evolve();

    virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
        const Ice::Current&);

    virtual void registerTopics();

    //! Check the serial port to ensure that we are connected to a 'motorboard' propeller,
    //! and also check the sanity of our command line options
    void start3();

    //Draw the updated local debugging GUI
    void updateGUI();

    //Send the motor speeds ([-1 .. 1]) to the motordriver/propeller
    void SetMotors(float motor1speed, float motor2speed);

    //Get the RC transmitter status
    unsigned char getRCStatus();

    //Get the status of an RC channel
    unsigned int getRCChannel(int channel);

    //Get the Enabled status from RC
    unsigned char getRCEnabled();

    //Get the Mix speed status from MotorBoard
    unsigned int getRCSpeed();

    //Get all of the RC status data in one frame
    //return false is bad frame received
    bool UpdateRCStatus();

    //Reset the encoder value
    void resetEncoder();

    //Reset the whole pilot
    void reset();

    //Get Battery Level in Volt
    double getBatteryVoltage();
    double adc2volt(unsigned char adc);

    //Update position after get odometry data
    void UpdatePosition(void);
    void updateIMUPosition(double dt);
    double getMaxTransAcceleration();
    double getMaxRotAcceleration();
    double getMaxRotVelocity();
    double getMaxTransVelocity();
    double getAcceleration(double motorPower);
    double getVelocity(double motorPower);

    //Get Log Data from current time
    void getCurrentLogData(void);

    //Redraw map base on the current scale
    void reDrawMap(double scale);

    void handleUserEvent();
    Point3D<double> tf(Point3D<double> p,double x,double y,double z,double roll,double pitch,double yaw);
    Point2D<double> rotZ(double x,double y,double ori);

  private:
    nub::soft_ref<Serial> itsSerial;
    Timer itsTimer;
    Timer itsSerialTimer;
    IceUtil::Mutex itsSerialMutex;
    int itsSerialTimeOutCounter;

    int itsCurrMessageID;
    int itsLastMessageID;

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

    double itsBatteryVotage;

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
    Point2D<int> itsMapCenter;

    Timer itsEncoderTimer;
    double itsLastDt;
    double itsLeftMotorVelocity;
    double itsRightMotorVelocity;
    double itsTransVelocityCurrent;//for pid
    double itsRotVelocityCurrent;//for pid
    double itsTransAccelCurrent;//for pid
    double itsRotAccelCurrent;//for pid
    double itsTravelDistance;//trajectory length in meter
    double itsTravelDistanceAuto;
    double itsTravelDistanceManual;
    double itsStraightDistance;//straight distance from origin to current location

    PID<float> itsPidRot;
    PID<float> itsPidTrans;

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
    std::vector<Point3DwithColor<double> >itsIMUPositionMap;
    std::vector<Point3DwithColor<double> >itsPositionMap;

    //////////////////////////////////////////
    //Display Data Structures
    //////////////////////////////////////////
    Image<PixRGB<byte> > itsControlImage;
    Image<PixRGB<byte> > itsDispImage;
    Image<PixRGB<byte> > itsDispCameraImage;
    bool itsWaitScreen;
    XWinManaged itsMeterXwin;
    Timer itsDisplayTimer;
    float itsDisplayUpdateRate;
    float itsMapScale;
    std::vector<SimpleMeter> itsChannelMeters;
    SimpleMeter itsBatteryMeter;
    velocityQue<float>        itsVelocityTransQue;//current robot trans speed
    velocityQue<float>        itsVelocityRotQue;//current robot rot speed
    velocityQue<float>        itsVelocityTransCmdQue;//current rc/cmd trans speed(capped)
    velocityQue<float>        itsVelocityRotCmdQue;//current rc/cmd rot speed(capped)
    velocityQue<float>        itsAccelTransQue;//current robot trans acceleration 
    velocityQue<float>        itsAccelRotQue;//current robot rot acceleration
    /////////////////////////////////////////
    //System Parameters
    /////////////////////////////////////////
    OModelParam<float>       itsMinimumSafeDistance;
    OModelParam<float>       itsMaxSpeed;

};
#endif

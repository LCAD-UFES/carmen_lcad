/*!@file Devices/Scorbot.H Interface to robot arm */

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/Scorbot.H $
// $Id: Scorbot.H 13955 2010-09-17 00:59:56Z lior $
//

#ifndef Scorbot_H_DEFINED
#define Scorbot_H_DEFINED

#include "Image/Layout.H"
#include "Image/MatrixOps.H"
#include "ArmControl/RobotArm.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Devices/Serial.H"
#include "Controllers/PID.H"
#include "Util/Types.H"
#include "Util/Timer.H"
#include "Image/Point3D.H"
#include <deque>
#include <pthread.h>
#include <stdarg.h>

#define MAX_PWM 100

#define BASE_POS_THRESH 12
#define BASE_NEG_THRESH -12

#define SHOLDER_POS_THRESH 12
#define SHOLDER_NEG_THRESH -12

#define ELBOW_POS_THRESH 12
#define ELBOW_NEG_THRESH -12

#define GRIPPER_POS_THRESH 12
#define GRIPPER_NEG_THRESH -12

#define WRIST_PITCH_POS_THRESH 12
#define WRIST_PITCH_NEG_THRESH -12

#define WRIST_ROLL_POS_THRESH 12
#define WRIST_ROLL_NEG_THRESH -12

#define EX1_POS_THRESH 12
#define EX1_NEG_THRESH -12

class Scorbot : public RobotArm
{
public:
  //Constructor
  Scorbot(OptionManager& mgr,
          const std::string& descrName = "Scorbot",
          const std::string& tagName = "Scorbot",
          const char *defdev = "/dev/ttyUSB0");

  ~Scorbot();

  struct ArmPos
  {
    int base;
    int sholder;
    int elbow;
    int gripper;
    int wristRoll;
    int wristPitch;
    int wrist1;
    int wrist2;
    int ex1;
    int ex2;
    int duration;

    bool baseMS;
    bool sholderMS;
    bool elbowMS;
    bool wrist1MS;
    bool wrist2MS;
    bool gripperMS;

    ArmPos() :
	    base(0), sholder(0), elbow(0), gripper(0),
	    wristRoll(0), wristPitch(0), wrist1(0), wrist2(0),
	    ex1(0), ex2(0), duration(0)
	{}
  };

  struct VelParam
  {
    double a;
    double b;
    double c;
    double d;
  };

  struct ArmVelParam
  {
    VelParam base;
    VelParam sholder;
    VelParam elbow;
    VelParam gripper;
    VelParam wristRoll;
    VelParam wristPitch;
    VelParam wrist1;
    VelParam wrist2;
    VelParam ex1;
    VelParam ex2;
  };

  //###########################################################
  void start2();
  void stop1();

  //servo definition

  //enum MOTOR {BASE, SHOLDER, ELBOW, WRIST1, WRIST2, GRIPPER, WRIST_ROLL, WRIST_PITCH, EX1, EX2};

  //enum COMMANDS {SAFE_MODE=9, SET_MOTORS=10, MOTORS_OFF=11,
  //        GET_ENCODER=12, RESET_ENCODER=13, GET_PWM=14, GET_MS=15};

  bool setMotor(MOTOR m, int pwm);
  void setMotorPos(MOTOR m, int pos);
  bool setJointPos(MOTOR joint, int position, bool relative = false, int duration = 2000);


  int getEncoder(MOTOR m);
  float getEncoderAng(MOTOR m);
  int getPWM(MOTOR m);

  bool resetEncoders();
  void resetMotorPos(void);

  bool stopAllMotors();

//  Image<PixRGB<unsigned char> > getFrame(int camera){return NULL;}
//  Image<PixRGB<byte> > getFrame(int camera){return NULL;}
  void simLoop(){};

  void setSafety(bool val);

  void getEF_pos(float &x, float &y, float &z);

  void homeMotors();
  void homeMotor(RobotArm::MOTOR axis, int LimitSeekSpeed, int MSJumpSpeed, float MSJumpDelay, int MSSeekSpeed, bool MSStopCondition, bool checkMS);

  bool motorsOff();
  bool motorsOn();

  int getMicroSwitch();
  char *getMicroSwitchByte();
  bool getMicroSwitchMotor(RobotArm::MOTOR m);

  void tunePID(MOTOR m, float p, float i, float d, bool relative);

  void stopControl() { itsStopControl = true; }
  void startControl() { itsStopControl = false; }
  bool controlStoped() { return itsStopControl; }

  double enc2ang(int encoderTicks);
  int ang2enc(double degrees);
  double enc2mm(int encoderTicks);
  int mm2enc(double mm);

  void controlLoop(); //the main control loop

  void setArmPos(ArmPos &armPos);
  ArmPos getArmPos();

  //Read the entire arm state using only one serial write,
  //and (hopefully) one serial read
  ArmPos readArmState();

  //! Get the End Effector pos
  Point3D<float> getEFPos(const ArmPos &armPos);

  //! Get the arm position from desired end effector posision
  ArmPos getIKArmPos(const Point3D<float>& efPos);

  bool set3DPos(bool relative = false, int duration = 2000);
  

  //! gravity compensator
  double gravityComp(int sholderPos, int elbowPos);


  ArmPos getDesiredArmPos();

  int getJointPos(MOTOR m);



  bool moveDone() { return itsMoveDone; }

  //Propeller PID commands
  bool setInternalPIDGain(MOTOR m, int P, int I, int D);
  bool setInternalPos(ArmPos pos);
  bool internalPidEn() { return itsInternalPIDen.getVal(); }


  //Read and write to raw serial data
  int write(const void* buffer, const int nbytes);
  int read(void* buffer, const int nbytes);

  //Set the motors of at shutdown
  void shutdownMotorsOff(bool val) { itsShutdownMotorsOff = val; }

private:

  bool enableInternalPID();
  bool disableInternalPID();

  virtual void paramChanged(ModelParamBase* const param,
      const bool valueChanged,
      ParamClient::ChangeStatus* status);

  char ms[9];//for micro switch binary display
  int msi[5];//for micro switch binary display
  nub::soft_ref<Serial> itsSerial; //our serial controller
  Timer itsTimer;

  PID<float>  pidBase;
  PID<float>  pidSholder;
  PID<float>  pidElbow;
  PID<float>  pidWrist1;
  PID<float>  pidWrist2;
  PID<float>  pidGripper;
  PID<float>  pidEx1;
  PID<float>  pidEx2;

  //ArmParameters
  float itsUpperarmLength;
  float itsForearmLength;
  float itsUpperarmCOM;
  float itsUpperarmMass;
  float itsForearmCOM;
  float itsForearmMass;
  float itsRadPerSholderEncoder;
  float itsRadPerElbowEncoder;
  float itsMMPerSlideEncoder;
  float itsBaseXOffset;
  float itsBaseZOffset;

  int itsHomeBaseOffset; //the oiffset from the home location to 0
  int itsHomeSholderOffset; //the oiffset from the home location to 0
  int itsHomeElbowOffset; //the oiffset from the home location to 0
  int itsHomeWristRollBaseOffset; //the oiffset from the home location to 0
  int itsHomeWristPitchBaseOffset; //the oiffset from the home location to 0

  NModelParam<int> itsBasePos;
  NModelParam<int> itsSholderPos;
  NModelParam<int> itsElbowPos;
  NModelParam<int> itsWrist1Pos;
  NModelParam<int> itsWrist2Pos;
  NModelParam<int> itsGripperPos;
  NModelParam<int> itsEX1Pos;
  NModelParam<int> itsEX2Pos;
  NModelParam<float> itsArmXPos;
  NModelParam<float> itsArmYPos;
  NModelParam<float> itsArmZPos;
  NModelParam<bool> itsMoveTo3D;
  NModelParam<int> itsDuration;
  NModelParam<bool> itsInternalPIDen;


  pthread_t itsControlThread;
  pthread_mutex_t itsLock;
  pthread_mutex_t itsSerialLock;
  pthread_mutex_t itsPosLock;

  mutable bool  itsRunControl; //the control loop thread
  mutable bool  itsStopControl; //start/stop control
  mutable bool  itsReadEncoders; //start/stop control
  mutable bool  itsMoveDone;
  mutable bool  itsShutdownMotorsOff;


  mutable bool  itsTemp;

  ArmPos itsCurrentPos;
  ArmPos itsDesiredPos;
  ArmVelParam itsArmVelParams;


  //! Bring motor to a predefined position
  char *int2byte(int x)
  {
   int n;
   for(n=0; n<8; n++)
   {
      if((x & 0x80) !=0)
      {
        ms[n]='1';
      }
      else
      {
        ms[n]='0';
      }
      x = x<<1;
   }
   ms[8]='\0';
   return ms;
  }
  int int2array(int x,int index)
  {
   int n;
   x = x<<3;
   for(n=0; n<5; n++)
   {
      if((x & 0x80) !=0)
      {
        msi[n]=1;
      }
      else
      {
        msi[n]=0;
      }
      x = x<<1;
   }
   return msi[index];
  }


};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

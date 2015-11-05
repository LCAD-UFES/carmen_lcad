/*!@file Devices/Scorbot.C Interfaces to the robot arm */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/Scorbot.C $
// $Id: Scorbot.C 13960 2010-09-17 17:15:03Z lior $
//

#include "Devices/Scorbot.H"
#include "Component/OptionManager.H"
#include "Util/MathFunctions.H"
#include "Util/Assert.H"
#include "Util/Timer.H"
#include "rutz/compat_snprintf.h"

//void* Scorbot_Control(void *scorbot);

void* Scorbot_Control(void *obj)
{
  Scorbot *scorbot = (Scorbot*)obj;
  scorbot->controlLoop();
  return NULL;
}



// ######################################################################
Scorbot::Scorbot(OptionManager& mgr, const std::string& descrName,
    const std::string& tagName,
    const char *defdev) :
  RobotArm(mgr, descrName, tagName),
  itsSerial(new Serial(mgr)),
  pidBase(0.05, 0.001, 0.02, -10, 10, 1, BASE_POS_THRESH, BASE_NEG_THRESH),
  pidSholder(0.05, 0.001, 0.01,-50, 50, 1, SHOLDER_POS_THRESH, SHOLDER_NEG_THRESH),
  pidElbow(0.05, 0.001, 0.03, -50, 50, 1, ELBOW_POS_THRESH, ELBOW_NEG_THRESH),
  pidWrist1(0.055, 0.01, 0.01,-50, 50, 1, WRIST_PITCH_POS_THRESH, WRIST_PITCH_NEG_THRESH),
  pidWrist2(0.055, 0.01, 0.01,-50, 50, 1, WRIST_ROLL_POS_THRESH, WRIST_ROLL_NEG_THRESH),
  pidGripper(0.025, 0.01, 0.01,-50, 50, 1, GRIPPER_POS_THRESH, GRIPPER_NEG_THRESH),
  pidEx1(0.010, 0, 0.009,-50, 50, 1, EX1_POS_THRESH, EX1_NEG_THRESH),
  pidEx2(0.0, 0, 0.0,-40, 40),
  itsUpperarmLength(222.0),
  itsForearmLength(222.0),
  itsUpperarmCOM(110),
  itsUpperarmMass(80),
  itsForearmCOM(200),
  itsForearmMass(200),
  itsRadPerSholderEncoder(M_PI/(10128.0*2.0)), //10234 ticks per 90 deg
  itsRadPerElbowEncoder(M_PI/(10128.0*2.0)), //10234 ticks per 90 deg
  itsMMPerSlideEncoder(500.0/139520.0),
  itsBaseXOffset(0),
  itsBaseZOffset(0),
  itsHomeBaseOffset(0),
  itsHomeSholderOffset(0),
  itsHomeElbowOffset(0),
  itsHomeWristRollBaseOffset(-1320),
  itsHomeWristPitchBaseOffset(-578),
  itsBasePos("BasePos", this, 0, ALLOW_ONLINE_CHANGES),
  itsSholderPos("SholderPos", this, 0, ALLOW_ONLINE_CHANGES),
  itsElbowPos("ElbowPos", this, 0, ALLOW_ONLINE_CHANGES),
  itsWrist1Pos("Wrist1Pos", this, 0, ALLOW_ONLINE_CHANGES),
  itsWrist2Pos("Wrist2Pos", this, 0, ALLOW_ONLINE_CHANGES),
  itsGripperPos("GripperPos", this, 0, ALLOW_ONLINE_CHANGES),
  itsEX1Pos("EX1Pos", this, 0, ALLOW_ONLINE_CHANGES),
  itsEX2Pos("EX2Pos", this, 0, ALLOW_ONLINE_CHANGES),
  itsArmXPos("ArmXPos", this, 0, ALLOW_ONLINE_CHANGES),
  itsArmYPos("ArmYPos", this, 0, ALLOW_ONLINE_CHANGES),
  itsArmZPos("ArmZPos", this, 0, ALLOW_ONLINE_CHANGES),
  itsMoveTo3D("MoveTo3D", this, 0, ALLOW_ONLINE_CHANGES),
  itsDuration("Duration", this, 2000, ALLOW_ONLINE_CHANGES),
  itsInternalPIDen("InternalPID", this, false, ALLOW_ONLINE_CHANGES),
  itsRunControl(false),
  itsStopControl(true),
  itsReadEncoders(false),
  itsShutdownMotorsOff(true) //Shutdown the motors during shutdown by default
{
  itsSerial->configure (defdev, 115200, "8N1", false, false, 10000);

  addSubComponent(itsSerial);

  itsRunControl = false;
  pthread_mutex_init(&itsLock, NULL);
  pthread_mutex_init(&itsSerialLock, NULL);
  pthread_mutex_init(&itsPosLock, NULL);

  itsDesiredPos.base = (int)itsHomeBaseOffset;
  itsDesiredPos.sholder = -7036;
  itsDesiredPos.elbow = 10958;
  itsDesiredPos.wristRoll = -1930;
  itsDesiredPos.wristPitch = 4748;
  itsDesiredPos.ex1 = 54000;

  itsArmVelParams.ex1.a = itsDesiredPos.ex1;
  itsArmVelParams.ex1.b = 0;
  itsArmVelParams.ex1.c = 0;
  itsArmVelParams.ex1.d = 0;


}

// ######################################################################
void Scorbot::start2()
{
//  itsSerial->setBlocking(false);

  //Start the controll thread
  //pthread_create(&itsControlThread, NULL, &Scorbot_Control, (void *)this);
}

// ######################################################################
void Scorbot::stop1()
{
  // stop our thread:
  while(itsRunControl == true)
    usleep(10000);
  itsRunControl = false;

  if (itsShutdownMotorsOff)
          motorsOff();
}


Scorbot::~Scorbot()
{
  pthread_mutex_destroy(&itsLock);
}


double Scorbot::gravityComp(int sholderPos, int elbowPos)
{
  //sholderPos =getEncoder(SHOLDER);
  //elbowPos = getEncoder(ELBOW);

  double sholderAng = (double)sholderPos*M_PI/20000.0;
  double elbowAng = -1*(double)elbowPos*M_PI/20000.0;

  double c1x = cos(sholderAng)*itsUpperarmCOM;
  double c1y = sin(sholderAng)*itsUpperarmCOM;

  double c2x = cos(sholderAng)*itsUpperarmLength + cos(elbowAng)*itsForearmCOM;
  double c2y = sin(sholderAng)*itsUpperarmLength + sin(elbowAng)*itsForearmCOM;

  double MvCv_x = (itsUpperarmMass*c1x + itsForearmMass*c2x)/2.0;
  double MvCv_y = (itsUpperarmMass*c1y + itsForearmMass*c2y)/2.0;

  double torque = sqrt(squareOf(MvCv_x) + squareOf(MvCv_y))*9.81*1000*sin(atan(MvCv_y/MvCv_x)+M_PI/2);

  return torque;
}


int Scorbot::mm2enc(double mm)
{
  return (int)(mm*69.39);
}

double Scorbot::enc2mm(int encoderTicks)
{
  return double(encoderTicks) / 69.39;
}

double Scorbot::enc2ang(int encoderTicks)
{
  return 90.0/10234.0*double(encoderTicks);
}

int Scorbot::ang2enc(double degrees)
{
  return int(10234.0 / 90.0 * degrees);
}

void Scorbot::controlLoop()
{
  itsRunControl = true;

  pidBase.setSpeed(50);
  pidSholder.setSpeed(40);
  pidElbow.setSpeed(40);
  pidGripper.setSpeed(50);
  pidWrist1.setSpeed(35);
  pidWrist2.setSpeed(35);
  pidEx1.setSpeed(30);
  pidEx2.setSpeed(30);

  int moveDoneCounter = 0;
  int thresh = 250;



  while(itsRunControl)
  {
    if (!itsStopControl)
    {
      pthread_mutex_lock(&itsPosLock);
      double time = itsTimer.getSecs();
      printf("%f %i\n", time, itsTemp);
      itsTemp = false;
      if (time > itsDesiredPos.duration) //Ensure that we do not go over the time limit
        time = itsDesiredPos.duration;


      int basePos = getEncoder(BASE);

      int basePwm = (int)pidBase.update(itsDesiredPos.base, basePos);
      int baseErr = abs(itsDesiredPos.base - basePos);

      int sholderPos =getEncoder(SHOLDER);
      int sholderPwm =  (int)pidSholder.update(itsDesiredPos.sholder, sholderPos);
      int sholderErr = abs(itsDesiredPos.sholder - sholderPos);

      int elbowPos = getEncoder(ELBOW);
      int elbowPwm = (int)pidElbow.update(itsDesiredPos.elbow, elbowPos);
      int elbowErr = abs(itsDesiredPos.elbow - elbowPos);


      int desiredWrist1Pos = (itsDesiredPos.wristRoll + itsDesiredPos.wristPitch)/2;
      int desiredWrist2Pos = (itsDesiredPos.wristRoll - itsDesiredPos.wristPitch)/2;

      int wrist1Pos = getEncoder(WRIST1);
      int wrist1Pwm = (int)pidWrist1.update(desiredWrist1Pos, wrist1Pos);
      int wrist1Err = abs(desiredWrist1Pos - wrist1Pos);

      int wrist2Pos = getEncoder(WRIST2);
      int wrist2Pwm = (int)pidWrist2.update(desiredWrist2Pos, wrist2Pos);
      int wrist2Err = abs(desiredWrist2Pos - wrist2Pos);

      int gripperPos = getEncoder(GRIPPER);
      int gripperPwm = (int)pidGripper.update(itsDesiredPos.gripper, gripperPos);
      int gripperErr = abs(itsDesiredPos.gripper - gripperPos);

      int ex1Pos = getEncoder(EX1);
      float ex1DesiredPos = itsArmVelParams.ex1.a + itsArmVelParams.ex1.c*(time*time) + itsArmVelParams.ex1.d*(time*time*time);
      float ex1DesiredVel =  2*itsArmVelParams.ex1.c*(time) + 3*itsArmVelParams.ex1.d*(time*time);
      int ex1Pwm = (int)pidEx1.update(ex1DesiredPos, ex1DesiredVel, ex1Pos);
      int ex1Err = abs(itsDesiredPos.ex1 - ex1Pos);

      int ex2Pos = getEncoder(EX2);
      int ex2Pwm = (int)pidEx2.update(itsDesiredPos.ex2, ex2Pos);
      //int ex2Err = abs(itsDesiredPos.ex2 - ex2Pos);

      //double gc = gravityComp(sholderPos, elbowPos)/10000000.0;
      //sholderPwm -= (int)gc;

      //LINFO("B:%i:%i S:%i:%i E:%i:%i wr:%i:%i wp:%i:%i gr(%i):%i:%i Ex:%i:%i",
      //    basePwm, baseErr,
      //    sholderPwm, sholderErr, // gc,
      //    elbowPwm, elbowErr,
      //    wrist1Pwm, wrist1Err,
      //    wrist2Pwm, wrist2Err,
      //    itsCurrentPos.gripper, gripperPwm, gripperErr,
      //    ex1Pwm, ex1Err);

     // printf("%i %i %i %i %i %i %i \n",
     //     basePos, sholderPos, elbowPos, wrist1Pos, wrist2Pos, gripperPos, ex1Pos);
      printf("%i %f %f %i\n", ex1Pos, ex1DesiredPos, ex1DesiredVel, ex1Pwm);
      fflush(stdout);
      pthread_mutex_unlock(&itsPosLock);

      //Update the itsCurrentPos structure

      pthread_mutex_lock(&itsLock);
      itsCurrentPos.base = basePos;
      itsCurrentPos.sholder = sholderPos;
      itsCurrentPos.elbow = elbowPos;
      itsCurrentPos.gripper = gripperPos;
      itsCurrentPos.wrist1 = wrist1Pos;
      itsCurrentPos.wrist2 = wrist2Pos;
      itsCurrentPos.wristRoll = wrist1Pos + wrist2Pos;
      itsCurrentPos.wristPitch = wrist1Pos - wrist2Pos;
      itsCurrentPos.ex1 = ex1Pos;
      itsCurrentPos.ex2 = ex2Pos;
      pthread_mutex_unlock(&itsLock);

      if (moveDoneCounter > 3)
        itsMoveDone = true;
      else
        itsMoveDone = false;

      if (baseErr < thresh &&
          sholderErr < thresh &&
          elbowErr < thresh &&
          gripperErr < thresh &&
          wrist1Err < thresh &&
          wrist2Err < thresh &&
          ex1Err < 1500)
        moveDoneCounter++;
      else
        moveDoneCounter = 0;


      //LINFO("moveDone %i %i", itsMoveDone, moveDoneCounter);

      setMotor(BASE, (int)basePwm);
      setMotor(SHOLDER, (int)sholderPwm);
      setMotor(ELBOW, (int)elbowPwm);
      setMotor(GRIPPER, (int)gripperPwm);
      setMotor(WRIST1, wrist1Pwm);
      setMotor(WRIST2, wrist2Pwm);
      setMotor(EX1, (int)ex1Pwm);
      setMotor(EX2, (int)ex2Pwm);


    } else {
      if (itsReadEncoders)
      {
        pthread_mutex_lock(&itsLock);
        itsCurrentPos = readArmState();

        //itsCurrentPos.base = getEncoder(BASE);
        //itsCurrentPos.sholder =getEncoder(SHOLDER);
        //itsCurrentPos.elbow = getEncoder(ELBOW);
        //itsCurrentPos.gripper = getEncoder(GRIPPER);
        //itsCurrentPos.wrist1 = getEncoder(WRIST1);
        //itsCurrentPos.wrist2 = getEncoder(WRIST2);
        //itsCurrentPos.wristRoll = itsCurrentPos.wrist1 + itsCurrentPos.wrist2;
        //itsCurrentPos.wristPitch = itsCurrentPos.wrist1 - itsCurrentPos.wrist2;
        //itsCurrentPos.ex1 = getEncoder(EX1);
        //itsCurrentPos.ex2 = getEncoder(EX2);
        pthread_mutex_unlock(&itsLock);
      }
    }

    usleep(10000);
  }

  pthread_exit(0);

}


void Scorbot::setArmPos(ArmPos &armPos)
{
  LINFO("Setting arm to: b:%i s:%i e:%i g:%i e1:%i e2:%i duration: %i",
      armPos.base, armPos.sholder, armPos.elbow, armPos.gripper,
      armPos.ex1, armPos.ex2, armPos.duration);

  unsigned char cmd[100];

  cmd[0] = 20;
  
  cmd[1] = (armPos.base >> 24) & 0xFF;
  cmd[2] = (armPos.base >> 16) & 0xFF;
  cmd[3] = (armPos.base >>  8) & 0xFF;
  cmd[4] = (armPos.base >>  0) & 0xFF;

  cmd[5] = (armPos.sholder >> 24) & 0xFF;
  cmd[6] = (armPos.sholder >> 16) & 0xFF;
  cmd[7] = (armPos.sholder >>  8) & 0xFF;
  cmd[8] = (armPos.sholder >>  0) & 0xFF;

  cmd[9] = (armPos.elbow >> 24) & 0xFF;
  cmd[10] = (armPos.elbow >> 16) & 0xFF;
  cmd[11] = (armPos.elbow >>  8) & 0xFF;
  cmd[12] = (armPos.elbow >>  0) & 0xFF;

  cmd[13] = (armPos.wrist1 >> 24) & 0xFF;
  cmd[14] = (armPos.wrist1 >> 16) & 0xFF;
  cmd[15] = (armPos.wrist1 >>  8) & 0xFF;
  cmd[16] = (armPos.wrist1 >>  0) & 0xFF;

  cmd[17] = (armPos.wrist2 >> 24) & 0xFF;
  cmd[18] = (armPos.wrist2 >> 16) & 0xFF;
  cmd[19] = (armPos.wrist2 >>  8) & 0xFF;
  cmd[20] = (armPos.wrist2 >>  0) & 0xFF;


  cmd[21] = (armPos.gripper >> 24) & 0xFF;
  cmd[22] = (armPos.gripper >> 16) & 0xFF;
  cmd[23] = (armPos.gripper >>  8) & 0xFF;
  cmd[24] = (armPos.gripper >>  0) & 0xFF;


  cmd[25] = (armPos.ex1 >> 24) & 0xFF;
  cmd[26] = (armPos.ex1 >> 16) & 0xFF;
  cmd[27] = (armPos.ex1 >>  8) & 0xFF;
  cmd[28] = (armPos.ex1 >>  0) & 0xFF;

  cmd[29] = (armPos.ex2 >> 24) & 0xFF;
  cmd[30] = (armPos.ex2 >> 16) & 0xFF;
  cmd[31] = (armPos.ex2 >>  8) & 0xFF;
  cmd[32] = (armPos.ex2 >>  0) & 0xFF;


  cmd[33] = (armPos.duration >> 24) & 0xFF;
  cmd[34] = (armPos.duration >> 16) & 0xFF;
  cmd[35] = (armPos.duration >>  8) & 0xFF;
  cmd[36] = (armPos.duration >>  0) & 0xFF;

  cmd[37] = 0xFF;
  cmd[38] = 0x00;
  cmd[39] = 0xFF;
  cmd[40] = 0x00;

  printf("Sending: ");
  for(int i=0; i<41; i++)
    printf("%u ", (unsigned char)cmd[i]);
  printf("\n");

  itsSerial->write(cmd, 41);

  usleep(50000);
  unsigned char buff[1024];

  int ret= itsSerial->read(buff, 1024);
  printf("Got %i: ", ret);
  for(int i=0; i<ret; i++)
    printf("%3d ", buff[i]);
  printf("\n");


}


Scorbot::ArmPos Scorbot::getArmPos()
{
  return Scorbot::readArmState();
}

Scorbot::ArmPos Scorbot::getDesiredArmPos()
{
  return itsDesiredPos;
}


void Scorbot::getEF_pos(float &x, float &y, float &z)
{

  float th1 = -1*getEncoderAng(RobotArm::SHOLDER) + (M_PI/2);
  float th2 = getEncoderAng(RobotArm::ELBOW) - th1;

  x = 0.220*cos(th1) + 0.220*cos(th1+th2);
  y = 0;
  z = 0.220*sin(th1) + 0.220*sin(th1+th2);
}

Point3D<float> Scorbot::getEFPos(const ArmPos& armPos)
{
  Point3D<float> efPos;


  float sholderAng = -1*(armPos.sholder-itsHomeSholderOffset)*itsRadPerSholderEncoder;
  float elbowAng = -1*(armPos.elbow-itsHomeElbowOffset)*itsRadPerElbowEncoder;

  //LINFO("Ang:: %i==>%f %f", armPos.sholder, sholderAng*180/M_PI, elbowAng*180/M_PI);
  efPos.x = armPos.ex1*itsMMPerSlideEncoder;
  efPos.y = itsBaseXOffset +
            itsUpperarmLength*sin(sholderAng+M_PI/2) +
            itsForearmLength*sin(-elbowAng+M_PI/2);
  efPos.z = itsBaseZOffset +
            itsUpperarmLength*cos(sholderAng+M_PI/2) +
            itsForearmLength*cos(-elbowAng+M_PI/2);


  return efPos;

}

Scorbot::ArmPos Scorbot::getIKArmPos(const Point3D<float>& efPos)
{

  double y = efPos.y + itsUpperarmLength+itsForearmLength;
  double z = efPos.z;


  double c2 = ( squareOf(y) +
                squareOf(z) -
                squareOf(itsUpperarmLength) -
                squareOf(itsForearmLength) ) /
              ( 2 * itsUpperarmLength * itsForearmLength );
  double s2 = sqrt(1 - squareOf(c2)); //+/- for the diffrent solutions

  //check valid disance to goal sqrt(x^2+y^2) < L1+L2
  //Not satafy when point is out of reach
  //Also, check valid (-1 < c2 < 1)

  double hatElbowAng = 1*atan2(s2, c2);

  double beta = M_PI/2 - atan2(z,y);

  double k1 = itsUpperarmLength+(itsForearmLength*c2);
  double k2 = itsForearmLength*s2;

  double sholderAng = 0;
  if (hatElbowAng > 0)
  {
    sholderAng = M_PI/2 - beta + atan2(k2,k1);
  } else {
    sholderAng = M_PI/2 - beta - atan2(k2,k1);
  }

  double elbowAng = -1*sholderAng + hatElbowAng;

  LINFO("Ang Shoulder: %f Elbow: %f %f", sholderAng*180/M_PI, elbowAng*180/M_PI, itsRadPerElbowEncoder);

  ArmPos armPos;
  armPos.base    =(int) itsHomeBaseOffset;
  armPos.sholder = ((int)(sholderAng/itsRadPerSholderEncoder)); 
  armPos.elbow   = ((int)(elbowAng/itsRadPerElbowEncoder));
  armPos.wristRoll = (int)itsCurrentPos.wristRoll;
  armPos.wristPitch = (int)itsCurrentPos.wristPitch;
  armPos.gripper = (int)itsCurrentPos.gripper;
  armPos.ex1 = (int)(((float)efPos.x)/itsMMPerSlideEncoder);
  armPos.ex2 = itsCurrentPos.ex2;

  return armPos;

}

bool Scorbot::set3DPos(bool relative, int duration )
{

  Point3D<float> efPos(itsArmXPos.getVal(), itsArmYPos.getVal(), itsArmZPos.getVal());
	Scorbot::ArmPos armPos = getIKArmPos(efPos);
	armPos.duration = duration;

  LINFO("3D Pos %f %f %f", efPos.x, efPos.y, efPos.z);

	LINFO("Encoders: B:%d S:%d E:%d WR:%d WP:%d W1:%i W2:%i G:%d E1:%d E2:%d",
			armPos.base,
			armPos.sholder,
			armPos.elbow, 
			armPos.wristRoll,
			armPos.wristPitch,
			armPos.wrist1,
			armPos.wrist2,
			armPos.gripper,
			armPos.ex1,
			armPos.ex2);

  setArmPos(armPos);
  return true;
}

Scorbot::ArmPos Scorbot::readArmState()
{

  unsigned char command = 22;
  itsSerial->write(&command, 1);
  std::vector<unsigned char> serialData = itsSerial->readFrame(command);

  if(serialData.size() != 16)
  {
    LERROR("Bad Serial Size!");
    return itsCurrentPos;
  }

  ArmPos currPos;

  currPos.base       = (short int)      ((int(0x00FF & serialData[0])  << 8) | (0x00FF & serialData[1]));
  currPos.sholder    = (short int)      ((int(0x00FF & serialData[2])  << 8) | (0x00FF & serialData[3]));
  currPos.elbow      = (short int)      ((int(0x00FF & serialData[4])  << 8) | (0x00FF & serialData[5]));
  currPos.wrist1     = (short int)      ((int(0x00FF & serialData[6])  << 8) | (0x00FF & serialData[7]));
  currPos.wrist2     = (short int)      ((int(0x00FF & serialData[8])  << 8) | (0x00FF & serialData[9]));
  currPos.gripper    = (short int)      ((int(0x00FF & serialData[10]) << 8) | (0x00FF & serialData[11]));
  currPos.ex1        = (unsigned int) 	((int(0x00FF & serialData[12]) << 8) | (0x00FF & serialData[13]))*4; //WE define the encoders by 4 for positions
  currPos.ex2        = (unsigned int) 	((int(0x00FF & serialData[14]) << 8) | (0x00FF & serialData[15]))*4;

  currPos.wristRoll  = currPos.wrist1 + currPos.wrist2;
  currPos.wristPitch = currPos.wrist1 - currPos.wrist2;

  currPos.baseMS = 0;
  currPos.sholderMS = 0;
  currPos.elbowMS = 0;
  currPos.wrist1MS = 0;
  currPos.wrist2MS = 0;
  currPos.gripperMS = 0;
  currPos.duration=0;


  return currPos;
}

bool Scorbot::enableInternalPID()
{
  LINFO("Enable PID");
  unsigned char cmd = 16;
  itsSerial->write(&cmd, 1);
  return true;
}

bool Scorbot::disableInternalPID()
{
  LINFO("Disable PID");
  unsigned char cmd = 17;
  itsSerial->write(&cmd, 1);
  return true;
}

bool Scorbot::setJointPos(MOTOR joint, int position, bool relative, int duration)
{
  unsigned char cmd[14];

  LINFO("Joint %i position %i duration %i",
      joint, position, duration);
  cmd[0] = 19;
  cmd[1] = joint;

  cmd[2] = (position >> 24) & 0xFF;
  cmd[3] = (position >> 16) & 0xFF;
  cmd[4] = (position >>  8) & 0xFF;
  cmd[5] = (position >>  0) & 0xFF;

  cmd[6] = (duration >> 24) & 0xFF;
  cmd[7] = (duration >> 16) & 0xFF;
  cmd[8] = (duration >>  8) & 0xFF;
  cmd[9] = (duration >>  0) & 0xFF;

  cmd[10] = 0xFF;
  cmd[11] = 0x00;
  cmd[12] = 0xFF;
  cmd[13] = 0x00;

  printf("Sending: ");
  for(int i=0; i<14; i++)
    printf("%u ", (unsigned char)cmd[i]);
  printf("\n");

  itsSerial->write(&cmd, 14);

  sleep(1);
  char buff[1024];

  int ret= itsSerial->read(buff, 1024);
  printf("Got %i: ", ret);
  for(int i=0; i<ret; i++)
    printf("%u ", buff[i]);
  printf("\n");


  return true;
}

void Scorbot::paramChanged(ModelParamBase* const param,
                                   const bool valueChanged,
                                   ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);

  if (param == &itsBasePos ) { setJointPos(BASE, itsBasePos.getVal(), false, itsDuration.getVal()); }
  else if (param == &itsSholderPos ) { setJointPos(SHOLDER, itsSholderPos.getVal(), false, itsDuration.getVal()); }
  else if (param == &itsElbowPos ) { setJointPos(ELBOW, itsElbowPos.getVal(), false, itsDuration.getVal()); }
  else if (param == &itsWrist1Pos ) { setJointPos(WRIST1, itsWrist1Pos.getVal(), false, itsDuration.getVal()); }
  else if (param == &itsWrist2Pos ) { setJointPos(WRIST2, itsWrist2Pos.getVal(), false, itsDuration.getVal()); }
  else if (param == &itsGripperPos ) { setJointPos(GRIPPER, itsGripperPos.getVal(), false, itsDuration.getVal()); }
  else if (param == &itsEX1Pos ) { setJointPos(EX1, itsEX1Pos.getVal(), false, itsDuration.getVal()); }
  else if (param == &itsEX2Pos ) { setJointPos(EX2, itsEX2Pos.getVal(), false, itsDuration.getVal()); }
  else if (param == &itsMoveTo3D ) { set3DPos(false, itsDuration.getVal()); }
  else if (param == &itsInternalPIDen ) {
          if (itsInternalPIDen.getVal())
                enableInternalPID();
          else
                disableInternalPID();
  }

}


bool Scorbot::setInternalPIDGain(MOTOR m, int P, int I, int D)
{
  unsigned char cmd[14];

  cmd[0]  = 16;
  cmd[1]  = m;

  cmd[2]  = 0x00FF & (P >> 24);
  cmd[3]  = 0x00FF & (P >> 16);
  cmd[4]  = 0x00FF & (P >>  8);
  cmd[5]  = 0x00FF & (P >>  0);

  cmd[6]  = 0x00FF & (I >> 24);
  cmd[7]  = 0x00FF & (I >> 16);
  cmd[8]  = 0x00FF & (I >>  8);
  cmd[9]  = 0x00FF & (I >>  0);

  cmd[10] = 0x00FF & (D >> 24);
  cmd[11] = 0x00FF & (D >> 16);
  cmd[12] = 0x00FF & (D >>  8);
  cmd[13] = 0x00FF & (D >>  0);

  itsSerial->write(&cmd, 14);
  return true;
}

bool Scorbot::setInternalPos(ArmPos pos)
{
  unsigned char cmd[15];
  LINFO("Setting Pos: %d", pos.ex1);

  cmd[0] = 19;
  cmd[1] = 6;
  cmd[2] = 0x00FF & (pos.ex1 >> 24);
  cmd[3] = 0x00FF & (pos.ex1 >> 16);
  cmd[4] = 0x00FF & (pos.ex1 >>  8);
  cmd[5] = 0x00FF & (pos.ex1 >>  0);

  unsigned char buf[256];
  itsSerial->write(&cmd, 14);
  usleep(1000000);

  int bytes_read = itsSerial->read(buf, 4);

  if(bytes_read == 4)
  {
    int d = ((0x00FF & buf[3]) << 24) |
      ((0x00FF & buf[2]) << 16) |
      ((0x00FF & buf[1]) << 8)  |
      ((0x00FF & buf[0]) << 0);

    int us = 80000000/1000000;
    int p = (-200*d)/(64*us) - 100;


    LINFO("MOTOR: %d", p);
  }
  else
    LINFO("ERROR! Read %d Bytes", bytes_read);



  return true;
}

bool Scorbot::setMotor(MOTOR m, int pwm)
{
  unsigned char cmd[6];
  //unsigned char buf[255];

  LINFO("Set motor %i pwm %i", m, pwm);
  if (pwm > MAX_PWM) pwm = MAX_PWM;
  if (pwm < -MAX_PWM) pwm = -MAX_PWM;

  cmd[0] = 10; //move motor

  if (m == WRIST_ROLL || m == WRIST_PITCH)
  {
    cmd[1] = WRIST1; //move motor
    cmd[3] = 10; //move motor
    cmd[4] = WRIST2;


    if (pwm >= 0)
    {
      cmd[2] = (short int)pwm&0x7F;
    }
    else
    {
      cmd[2] = (abs((short int)pwm)&0x7F) | 0x80;
    }

    if (m == WRIST_ROLL)
    {
      if (pwm >= 0)
      {
        cmd[5] = (short int)pwm&0x7F;
      }
      else
      {
        cmd[5] = (abs((short int)pwm)&0x7F) | 0x80;
      }
    } else {
      if (pwm <= 0)
      {
        cmd[5] = abs((short int)pwm)&0x7F;
      }
      else
      {
        cmd[5] = (abs((short int)pwm)&0x7F) | 0x80;
      }
    }

    itsSerial->write(cmd, 6);
    usleep(10000);

    //int bytes_read = 0;
    //while(bytes_read != 2)
    //{
    //  bytes_read += itsSerial->read(buf+bytes_read, 1);
    //  usleep(10000);
    //}

    //if (buf[0]  != 255 && buf[1] != 255)
    //  return false;
    //else
      return true;

  } else {
    cmd[1] = m; //motor
    if (pwm >= 0)
      cmd[2] = (short int)pwm&0x7F;
    else
      cmd[2] = (abs((short int)pwm)&0x7F) | 0x80;

    itsSerial->write(cmd, 3);
    usleep(10000);

    char buf[100];
    int bytes_read = itsSerial->read(buf, 1);
    while(bytes_read != 1)
    {
      bytes_read = itsSerial->read(buf, 1);
      usleep(10000);
    }
    LINFO("Got %i", bytes_read);
    for(int i=0; i<bytes_read; i++)
        LINFO("Bytes %i %u", i, buf[i]);

    //if (buf[0]  != 255)
    //  return false;
    //else
      return true;
  }

  return false;


}

bool Scorbot::motorsOff()
{
  unsigned char cmd[1];
  //unsigned char buf[1];

  LINFO("Turning motors off");
  cmd[0] = 15;
  itsSerial->write(cmd, 1);

  //itsSerial->read(buf, 1);
  //if (buf[0]  != 255)
  //  return false;
  //else
   return true;
}

bool Scorbot::motorsOn()
{
  unsigned char cmd[1];
 // unsigned char buf[1];

  LINFO("Turning motors on");
  cmd[0] = 14;
  itsSerial->write(cmd, 1);
  usleep(50000);

  itsSerial->read(cmd, 1);
  if (cmd[0]  != 255)
    return false;
  else
    return true;
}



bool Scorbot::stopAllMotors()
{
  unsigned char cmd[1];
  //unsigned char buf[1];

  cmd[0] = 11; //stop all motors
  itsSerial->write(cmd, 1);

  //itsSerial->read(buf, 1);
  //if (buf[0]  != 255)
  //  return false;
  //else
    return true;
}



int Scorbot::getMicroSwitch()
{
  unsigned char cmd;
  unsigned char buf = 11;
 // unsigned char check_byte;
 // unsigned char buf;
  int bytes_read;

  std::vector<unsigned char> serialIn;

  cmd = 21; //get all ms

  itsSerial->write(&cmd, 1);

  while(1)
  {
    bytes_read = itsSerial->read(&buf, 1);

    if(bytes_read > 0)
    {
      serialIn.push_back(buf);

      if(serialIn.size() > 3)
        serialIn.erase(serialIn.begin());

      if(serialIn.size() == 3)
      {
        if(serialIn.at(1) == 255 && serialIn.at(2) == 99)
          return serialIn.at(0);
      }
    }
  }
  return buf;
}

char* Scorbot::getMicroSwitchByte()
{
  return int2byte(getMicroSwitch());
}
bool Scorbot::getMicroSwitchMotor(RobotArm::MOTOR m)
{
  int axis = -1;
  if(m == BASE)
    axis = 1;
  if(m == SHOLDER)
    axis = 2;
  if(m == ELBOW)
    axis = 3;
  if(m == WRIST_ROLL)
    axis = 5;
  if(m == WRIST_PITCH)
    axis = 4;

  return !((getMicroSwitch() >> (8-axis)) & 0x01);
//  return int2array(getMicroSwitch(),m);
}

int Scorbot::getEncoder(MOTOR m)
{
  std::vector<char> serialIn;
  unsigned char cmd[2];
  unsigned char buf[255];

  //Clear the serial port
  while(itsSerial->read(&buf, 255) > 0);

  if (m == WRIST_ROLL || m == WRIST_PITCH)
  {
    int wrist1 = getEncoder(WRIST1);
    int wrist2 = getEncoder(WRIST2);

    if (m==WRIST_PITCH)
      return wrist1-wrist2;
    else
      return wrist1+wrist2;

  } else {

    unsigned char axis = 0;
    switch(m)
    {
      case BASE:
        axis = 0;
        break;
      case SHOLDER:
        axis = 1;
        break;
      case ELBOW:
        axis = 2;
        break;
      case WRIST1:
        axis = 4;
        break;
      case WRIST2:
        axis = 3;
        break;
      case GRIPPER:
        axis = 5;
        break;
      case EX1:
        axis = 6;
        break;
      case EX2:
        axis = 7;
        break;
      default:
        LFATAL("UNKOWN axis: %d", axis);
        break;
    }

    cmd[0] = 12; //get encoder

    cmd[1] = axis; //motor

    short val = 0;
    itsSerial->write(cmd, 2);
    //LINFO("REad: %i %i", cmd[0], cmd[1]);


    usleep(10000);
    int bytes_read = itsSerial->read(buf, 3);

    while (bytes_read < 3)
    {
      bytes_read += itsSerial->read(buf+bytes_read, 3-bytes_read);
      usleep(10000);
    }
    //printf("R %i: ", bytes_read);
    //for(int i=0; i<bytes_read; i++)
    //  printf("%i ", buf[i]);
    //printf("\n");

    //while(1)
    //{
    //  bytes_read = itsSerial->read(buf, 1);
    //  if(bytes_read > 0 && buf[0] == 255 && serialIn.size() >= 2)
    //  {
    //    break;
    //  }
    //  else
    //  {
    //    serialIn.push_back(buf[0]);

    //    if(serialIn.size() > 2)
    //      serialIn.erase(serialIn.begin());
    //  }
    //}

    //val = (int(0x00FF & serialIn.at(0)) << 8) | (0x00FF & serialIn.at(1));
    val = (int(0x00FF & buf[0]) << 8) | (0x00FF & buf[1]);

    if (m == EX1)
      return (unsigned short)val;
    else
      return (short int)val;

    //if (i == 1) //Only read one byte, wait for another
    //{
    //  usleep(10000);
    //  i += itsSerial->read(buf+1, 2);
    //}
    //if (i == 2) //Only read one byte, wait for another
    //{
    //  usleep(10000);
    //  i += itsSerial->read(buf+1, 1);
    //}



    //if (i < 3 || buf[2] != 255)
    //{
    //  LINFO("Error reading encoder value read(%i %x %x %x)", i,
    //      buf[0], buf[1], buf[2]);
    //} else {
    //  val = (buf[0] << 8) + buf[1];
    //  return val;
    //}
  }
  //pthread_mutex_lock(&itsSerialLock);
  return 0;
}

float Scorbot::getEncoderAng(MOTOR m)
{

  return 0; //(float)e*M_PI/(2975*2);

}

void Scorbot::setSafety(bool val)
{
  unsigned char cmd[2];
  cmd[0] = SAFE_MODE;
  if (val)
    cmd[1] = 1;
  else
    cmd[1] = 0;

  itsSerial->write(cmd, 2);
}

bool Scorbot::resetEncoders()
{
  unsigned char cmd[1];
  //unsigned char buf[1];
  cmd[0] = 13; //Reset encoders
  itsSerial->write(cmd, 1);
  usleep(50000);

  itsSerial->read(cmd, 1);
  if (cmd[0]  != 255)
    return false;
  else
    return true;

}

int Scorbot::getPWM(MOTOR m)
{
  unsigned char cmd[2];
  unsigned char buf[2];
  cmd[0] = GET_PWM;
  cmd[1] = m; //motor
  itsSerial->write(cmd, 2);

  int i = itsSerial->read(buf, 1);
  if (i < 1) return -1;
  return buf[0];
}

void Scorbot::setMotorPos(MOTOR m, int pos)
{


}
void Scorbot::resetMotorPos(void)
{
}

void Scorbot::tunePID(MOTOR m, float p, float i, float d, bool relative)
{

  PID<float> *pid = 0;

  switch(m)
  {
    case BASE:
      pid = &pidBase;
      break;
    case SHOLDER:
      pid = &pidSholder;
      break;
    case ELBOW:
      pid = &pidElbow;
      break;
    case GRIPPER:
      pid = &pidGripper;
      break;
    case EX1:
      pid = &pidEx1;
      break;
    case EX2:
      pid = &pidEx2;
      break;
    default:
      LERROR("INVALID TUNEPID MOTOR (%d)", m);
      return;
  }

  if (relative)
  {
    pid->setPIDPgain(pid->getPIDPgain() + p);
    pid->setPIDIgain(pid->getPIDIgain() + i);
    pid->setPIDDgain(pid->getPIDDgain() + d);
  } else {
    pid->setPIDPgain(p);
    pid->setPIDIgain(i);
    pid->setPIDDgain(d);
  }
  LINFO("Setting PID p=%f i=%f d=%f",
      pid->getPIDPgain(),
      pid->getPIDIgain(),
      pid->getPIDDgain());
}


//void Scorbot::homeMotors()
//{
//
//  //First, move all joints to their limits
//  int   Base_LimitSeekSpeed       = -50;
////  int   Base_MSJumpSpeed          = 100;
////  float Base_MSJumpDelay          = 3.5;
////  int   Base_MSSeekSpeed          = 15;
//
//  int   Shoulder_LimitSeekSpeed   = -40;
////  int   Sholder_MSJumpSpeed       = 0;
////  float Sholder_MSJumpDelay       = 1.0;
////  int   Sholder_MSSeekSpeed       = -20;
//
//  int   Elbow_LimitSeekSpeed      = -40;
////  int   Elbow_MSJumpSpeed         = 0;
////  float Elbow_MSJumpDelay         = 1.0;
////  int   Elbow_MSSeekSpeed         = -25;
//
//  int   WristPitch_LimitSeekSpeed = -40;
////  int   WristPitch_MSJumpSpeed    = 0;
////  float WristPitch_MSJumpDelay    = 1.0;
////  int   WristPitch_MSSeekSpeed    = -25;
//
////  int   WristRoll_MSJumpSpeed     = 10;
////  float WristRoll_MSJumpDelay     = .1;
////  int   WristRoll_MSSeekSpeed     = -25;
//
//  int   Gripper_LimitSeekSpeed    = -40;
//
//  int   EX1_LimitSeekSpeed        = 50;
//
//
//  unsigned int numLimitsHit = 0;
//  std::map<RobotArm::MOTOR, int> limitHit;
//  limitHit[BASE]        = 0;
//  limitHit[SHOLDER]     = 0;
//  limitHit[ELBOW]       = 0;
//  limitHit[WRIST_PITCH] = 0;
//  limitHit[GRIPPER]     = 0;
//  limitHit[EX1]         = 0;
//
//  std::map<RobotArm::MOTOR, int> lastEnc;
//  lastEnc[BASE]        = getEncoder(BASE);
//  lastEnc[SHOLDER]     = getEncoder(SHOLDER);
//  lastEnc[ELBOW]       = getEncoder(ELBOW);
//  lastEnc[WRIST_PITCH] = getEncoder(WRIST_PITCH);
//  lastEnc[GRIPPER]     = getEncoder(GRIPPER);
//  lastEnc[EX1]         = getEncoder(EX1);
//
//
//  setMotor(BASE,         Base_LimitSeekSpeed);
//  setMotor(SHOLDER,      Shoulder_LimitSeekSpeed);
//  setMotor(ELBOW,        Elbow_LimitSeekSpeed);
//  setMotor(WRIST_PITCH,  WristPitch_LimitSeekSpeed);
//  setMotor(GRIPPER,      Gripper_LimitSeekSpeed);
//  setMotor(EX1,          EX1_LimitSeekSpeed);
//  usleep(100000);
//
//  std::map<RobotArm::MOTOR, int>::iterator lastEncIt;
//  while(numLimitsHit < limitHit.size())
//  {
//    for(lastEncIt = lastEnc.begin(); lastEncIt != lastEnc.end(); lastEncIt++)
//    {
//      LINFO("NumLimitsHit: %d/%lu", numLimitsHit, limitHit.size());
//      usleep(10000);
//      int currEncoder = getEncoder((*lastEncIt).first);
//      int lastEncoder = (*lastEncIt).second;
//      RobotArm::MOTOR axis = (*lastEncIt).first;
//
//
//      if(abs(currEncoder - lastEncoder) == 0)
//      {
//        limitHit[axis] = limitHit[axis] + 1;
//        if(limitHit[axis] > 20)
//        {
//          //Stop The Motor
//          setMotor(axis, 0);
//          //Record the limit as hit
//          limitHit[axis] = true;
//          //Increment the hit counter
//          numLimitsHit++;
//        }
//      }
//      else
//      {
//        limitHit[axis] = std::max(0, limitHit[axis]-1);
//
//      }
//      lastEnc[axis] = currEncoder;
//    }
//  }
//
//  stopAllMotors();
//  LINFO("DONE! NumLimitsHit: %d/%lu", numLimitsHit, limitHit.size());
////
////  LINFO("Homing Base");
////  homeMotor(BASE, Base_MSJumpSpeed, Base_MSJumpDelay, Base_MSSeekSpeed, true);
////
////  LINFO("Homing Shoulder");
////  homeMotor(SHOLDER, Sholder_MSJumpSpeed, Sholder_MSJumpDelay, Sholder_MSSeekSpeed, false);
////
////  LINFO("Homing Elbow");
////  homeMotor(ELBOW, Elbow_MSJumpSpeed, Elbow_MSJumpDelay, Elbow_MSSeekSpeed, true);
////
////  LINFO("Homing Wrist Pitch");
////  homeMotor(WRIST_PITCH,  WristPitch_MSJumpSpeed, WristPitch_MSJumpDelay, WristPitch_MSSeekSpeed, true);
////
////  LINFO("Homing Wrist Roll");
////  homeMotor(WRIST_ROLL, WristRoll_MSJumpSpeed, WristRoll_MSJumpDelay, WristRoll_MSSeekSpeed, true);
////  homeMotor(WRIST_ROLL, WristRoll_MSJumpSpeed, WristRoll_MSJumpDelay, WristRoll_MSSeekSpeed, false);
//
//
//
//
//
//}

/*
 Encoder Offsets:
  Base:        -3860
  Shoulder:    -13736
  Elbow:        6764
  Wrist Roll:  -1320
  Wrist Pitch: -578


 */


void Scorbot::homeMotors()
{
  int   EX1_LimitSeekSpeed        = 40;

  int   Base_LimitSeekSpeed       = -50;
  int   Base_MSJumpSpeed          = 100;
  float Base_MSJumpDelay          = 3.5;
  int   Base_MSSeekSpeed          = 15;

  int   Sholder_LimitSeekSpeed    = -25;
  int   Sholder_MSJumpSpeed       = 70;
  float Sholder_MSJumpDelay       = 3.6;
  int   Sholder_MSSeekSpeed       = 10;

  int   Elbow_LimitSeekSpeed      = -35;
  int   Elbow_MSJumpSpeed         = 90;
  float Elbow_MSJumpDelay         = 1.8;
  int   Elbow_MSSeekSpeed         = 25;

  int   WristPitch_LimitSeekSpeed = 20;
  int   WristPitch_MSJumpSpeed    = 0;
  float WristPitch_MSJumpDelay    = 0.0;
  int   WristPitch_MSSeekSpeed    = 20;

  int   WristRoll1_MSSeekSpeed     = 25;

  int   WristRoll2_MSJumpSpeed     = 15;
  float WristRoll2_MSJumpDelay     = .1;
  int   WristRoll2_MSSeekSpeed     = 20;

  int   Gripper_LimitSeekSpeed     = 40;

  itsReadEncoders = false;

  LINFO("Homing Slide");
  homeMotor(EX1, EX1_LimitSeekSpeed, 0, 0, 0, true, false);

  LINFO("Homing Base");
  homeMotor(BASE, Base_LimitSeekSpeed, Base_MSJumpSpeed, Base_MSJumpDelay, Base_MSSeekSpeed, true, true);

  LINFO("Homing Shoulder");
  homeMotor(SHOLDER, Sholder_LimitSeekSpeed, Sholder_MSJumpSpeed, Sholder_MSJumpDelay, Sholder_MSSeekSpeed, true, true);

  LINFO("Homing Elbow");
  homeMotor(ELBOW, Elbow_LimitSeekSpeed, Elbow_MSJumpSpeed, Elbow_MSJumpDelay, Elbow_MSSeekSpeed, true, true);

  LINFO("Homing Wrist Pitch");
  homeMotor(WRIST_PITCH, WristPitch_LimitSeekSpeed, WristPitch_MSJumpSpeed, WristPitch_MSJumpDelay, WristPitch_MSSeekSpeed, true, true);

  LINFO("Homing Wrist Roll");
  homeMotor(WRIST_ROLL, 0, 0, 0, WristRoll1_MSSeekSpeed, true, true);
  homeMotor(WRIST_ROLL, 0, WristRoll2_MSJumpSpeed, WristRoll2_MSJumpDelay, WristRoll2_MSSeekSpeed, false, true);

  LINFO("Homing Gripper");
  homeMotor(GRIPPER, Gripper_LimitSeekSpeed, 0, 0, 0, true, false);

  stopAllMotors();
  resetEncoders();
  itsReadEncoders = true;
  LINFO("Finished Homing");
}



void Scorbot::homeMotor(RobotArm::MOTOR axis, int LimitSeekSpeed, int MSJumpSpeed, float MSJumpDelay, int MSSeekSpeed, bool MSStopCondition, bool checkMS)
{
  int lastPos = -1;
  int currPos = -1;


//  //First move the joint all the way to one side to find the mechanical limit
//  LINFO("Finding Limit");
//  lastPos = getEncoder(axis);
//  setMotor(axis, LimitSeekSpeed);
//  usleep(500000);
//  currPos = getEncoder(axis);
//  while(abs(currPos - lastPos) > 0) //As long as the joint is still moving, then don't stop
//  {
//    lastPos = currPos;
//    currPos = getEncoder(axis);
//    usleep(70000);
//  }

  //Should we look for a microswitch?
  if(checkMS)
  {
//    //Now, kick the joint over to get it almost to the microswitch
//    LINFO("Jumping to MS");
//    setMotor(axis, MSJumpSpeed);
//    usleep(float(MSJumpDelay) * 1000000);

    //And now move until we hit the microswitch
    setMotor(axis, MSSeekSpeed);
    LINFO("Looking for MS");
    while(getMicroSwitchMotor(axis) != MSStopCondition);
  } else {
    LINFO("Finding Limit");
    lastPos = getEncoder(axis);
    setMotor(axis, LimitSeekSpeed);
    usleep(500000);
    currPos = getEncoder(axis);
    while(abs(currPos - lastPos) > 0) //As long as the joint is still moving, then don't stop
    {
      lastPos = currPos;
      currPos = getEncoder(axis);
      LINFO("Speed: %i", abs(currPos - lastPos));
      usleep(90000);
    }
  }
  //Stop the motor
  setMotor(axis,0);
}

int Scorbot::getJointPos(MOTOR m)
{
  switch(m)
  {
    case BASE: return itsCurrentPos.base;
    case SHOLDER: return itsCurrentPos.sholder;
    case ELBOW: return itsCurrentPos.elbow;
    case GRIPPER: return itsCurrentPos.gripper;
    case WRIST_ROLL: return itsCurrentPos.wristRoll;
    case WRIST_PITCH: return itsCurrentPos.wristPitch;
    case EX1: return itsCurrentPos.ex1;
    case EX2: return itsCurrentPos.ex2;
    default:
              LFATAL("UNKOWN axis: %d", m);
              break;
  }

  return 999999;
}

int Scorbot::write(const void* buffer, const int nbytes)
{
        return itsSerial->write(buffer, nbytes);
}

int Scorbot::read(void* buffer, const int nbytes)
{
        return itsSerial->read(buffer, nbytes);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

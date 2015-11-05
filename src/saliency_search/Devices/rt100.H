/*!@file Devices/rt100.H Interface to a UMI rt-100 robot arm */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/rt100.H $
// $Id: rt100.H 7833 2007-02-01 23:01:39Z rjpeters $
//

#ifndef RT100_H_DEFINED
#define RT100_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Util/Types.H"      // for byte
#include "Util/Assert.H"
#include <vector>

class Serial;

//! Interface to a serial-servo-controller
class RT100 : public ModelComponent
{
public:

  static const int RT100_OK = 0;
  static const int RT100_FAIL = 1;
  static const unsigned char EMERGENCY_STOP = 0x24;

  enum MOTOR_STATUS {TASK_IN_PROGRESS = 0, //task still in progress?
                     TASK_COMPLETE = 1, //task complete?
                     ERROR_LIMIT_EXCEEDED = 2,
                     NO_MOVEMENT = 3,
                     MODE_ABSOLUTE_RELATIVE = 4,
                     MODE_POSITION_FORCE = 5
  };


  enum IP {IP0=0, IP1=1};
  enum PARAM {CP_ERROR, CURRENT_POSITION, ERROR_LIMIT, NEW_POSITION, SPEED, KP, KI, KD,
             DEAD_BAND, OFFSET, MAX_FORCE, CURRENT_FORCE, ACCELERATION_TIME, USER_RAM,
             USER_IO, ACTUAL_POSITION };
  enum IP0MOTORS {IP0BASE1, IP0BASE2, IP0WRIST1, IP0WRIST2, IP0SENSOR};
  enum IP1MOTORS {IP1ELBOW, IP1SHOLDER, IP1ZED, IP1YAW, IP1GRIPPER};
  enum JOINTS {BASE1, BASE2, WRIST1, WRIST2, ROLL_WRIST, TILT_WRIST,
               SENSOR, ELBOW, SHOLDER, ZED, YAW, GRIPPER};
  static const unsigned int NUM_JOINTS = 12; //increment if JOINTS are changed

  enum MOTORS_TOGGLES {FORCE_POSITION, ABSOLUTE_RELATIVE, USER_INPUTOUTPUT};
  enum MOTORES_STOP {DEAD_STOP, RAMP_STOP,
    FREE_STOP, //disables PWM output
    FREE_OFF}; //enable PWM output

  //! Default constructor; see ModelComponent.H
  RT100(OptionManager& mgr,
      const std::string& descrName = "UMI RT100 Robot Arm Controller",
      const std::string& tagName = "RT100Controller",
      const char *defdev = "/dev/ttyS0");

  //! Destructor
  ~RT100();

  //! initalize the arm and set default values;
  int init();

  //! shutdown the arm and switch off all motors
  int shutdown();

  //! get the number of joints
  int getNumJoints();

  //! set the joint position, if immediate is true, then initiate a move command
  int setJointPosition(JOINTS joint, short int position, bool immediate = false);

  //! get the current joints position
  int getJointPosition(JOINTS joint, short int *position);

  //! set the joint parameter
  int setJointParam(JOINTS joint, PARAM param, short int val);

  //! move the arm to the new position
  int moveArm(bool waitUntilComplete = false);

  //! return if the move arm completed
  bool moveComplete();

  //! move the arm in interpolation mode
  int interpolationMove(std::vector<short int> &moveVals);

  //! deferred write param to a particuler ctrl (done in 2 transactions)
  int deferredWrite(IP ipID, unsigned int ctrl, PARAM param,short int val);

  //! deferred read param from a particuler ctrl (done in 2 transactions)
  int deferredRead(IP ipID, unsigned int ctrl, PARAM param,short int *val);

  //! immediate read param from a particuler ctrl (done in 1 transactions)
  int immediateRead(IP ipID, unsigned int ctrl, PARAM param, short int *val);

  //! immediate write param to a particuler ctrl (done in 1 transactions)
  int immediateWrite(IP ipID, unsigned int ctrl, PARAM param,short int val);

  //! send raw commands to the arm (one byte command)
  int rawCommand(IP ip, unsigned char cmdType,
      unsigned char *resCode, short int *results = NULL);

  //! send raw commands to the arm (one byte command)
  int rawCommand(IP ipID, unsigned char cmdType,
      unsigned char *resCode,
      unsigned char *byte1,
      unsigned char *byte2);

  //! send raw commands to the arm (three byte command)
  int rawCommand(IP ip, unsigned char cmdType,
      unsigned char byte1, unsigned char byte2,
      unsigned char *resCode);


  //! initalize zed to home position and reset counters
  void initZed();

  //! initalize gripper to home position and reset counters
  void initGripper();

  //! initalize wrist to home position and reset counters
  void initWrist();

  //! initalize sholder to home position and reset counters
  void initSholder();

  //! initalize elbow and yaw to home position and reset counters
  void initElbowYaw();

  //! move the arm to home position
  void gotoHomePosition();


protected:
  nub::soft_ref<Serial> itsPort;   //!< Serial port to use
  int switchIP(IP ipID); //switch to a diffrent ip

private:
  int itsWristTiltVal; //keep the tilt and rool value for calculating wrist position
  int itsWristRollVal;

};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

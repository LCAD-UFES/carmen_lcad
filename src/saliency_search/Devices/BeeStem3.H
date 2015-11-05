
/*!@file Devices/BeeStem3.H Simple interface to beestem */


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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/BeeStem3.H $
// $Id: BeeStem3.H 13241 2010-04-15 10:23:02Z beobot $
//

#ifndef BeeStem3_H_DEFINED
#define BeeStem3_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Devices/Serial.H"
#include "Util/Types.H"      // for byte
#include <string>
#include <vector>

#define PID_DEPTH       0
#define PID_HEADING     1
#define PID_ENABLE      3
#define PID_DISABLE     2

#define DESIRED_DEPTH   0
#define DESIRED_HEADING 1

class Serial;

class BeeStem3 : public ModelComponent
{
public:
	const static int NUM_MOTOR_CONTROLLERS = 9;
	struct MotorControllerIDs
	{
		const static int FWD_RIGHT_THRUSTER = 3;
		const static int FWD_LEFT_THRUSTER = 1;
		const static int DEPTH_RIGHT_THRUSTER = 4;
		const static int DEPTH_LEFT_THRUSTER = 2;
		const static int STRAFE_FRONT_THRUSTER = 0;
		const static int STRAFE_BACK_THRUSTER = 5;
		const static int SHOOTER = 6;
		const static int DROPPER_STAGE1 = 7;
		const static int DROPPER_STAGE2 = 8;
	};
  //! Default constructor; see ModelComponent.H
  BeeStem3(OptionManager& mgr,
      const std::string& descrName = "BeeStem3 Driver",
      const std::string& tagName = "BeeStem3",
      const char *defdev = "/dev/ttyUSB0");

  //! Destructor
  ~BeeStem3();

  bool getSensors(int &accelX, int &accelY, int &accelZ,
                  int &compassHeading, int &compassPitch, int &compassRoll,
                  int &internalPressure, int &externalPressure,
                  int &desiredHeading, int &desiredDepth, int &desiredSpeed,
                  int &headingK, int &headingP, int &headingD, int &headingI, int &headingOutput,
                  int &depthK, int &depthP, int &depthD, int &depthI, int &depthOutput, char &killSwitch);


  bool setPID(int pidMode, float k, float p, float i, float d);

  // bool setDesiredValues(int16 heading, int16 depth, char speed, char markerDropper);
  bool setDesiredHeading(int16 heading);
  bool setDesiredDepth(int16 depth);
  bool setDesiredSpeed(char speed);
  void setThruster(int num, int val);
  void startCompassCalibration();
  void endCompassCalibration();

  std::vector<int> mMotorControllerState;

protected:
  nub::soft_ref<Serial> itsPort;   //!< Serial port to use
  pthread_mutex_t itsSerialLock;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/*!@file Devices/IMU_MicroStrain_3DM_GX2.H class
 for interfacing with the MicroStrain 3DM_GX2 IMU */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
// by the University of Southern California (USC) and the iLab at USC.  //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/IMU_MicroStrain_3DM_GX2.H $
// $Id: IMU_MicroStrain_3DM_GX2.H 14981 2011-10-14 00:07:57Z beobot $
//

#ifndef IMU_MICROSTRAIN_3DM_GX2_H_DEFINED
#define IMU_MICROSTRAIN_3DM_GX2_H_DEFINED

#include "Component/OptionManager.H"
#include "Devices/Serial.H"
#include "Util/Angle.H"

enum DataRequested { ACCEL_AND_ANG_RATE,
                     DELTA_ANG_AND_VEL,
                     MAGNETOMETER,
                     ORIENTATION_MATRIX,
                     ROLL_PITCH_YAW,
                     TEMPERATURE };

typedef struct _AccelAndAngRateRecord
{
  float accelX;
  float accelY;
  float accelZ;
  float angRateX;
  float angRateY;
  float angRateZ;
}
AccelAndAngRateRecord;

typedef struct _MagnetometerRecord
{
  float magX;
  float magY;
  float magZ;
}
MagnetometerRecord;

typedef struct _RollPitchYawRecord
{
  float roll;
  float pitch;
  float yaw;
}
RollPitchYawRecord;

//! Class for interfacing with the IMU
//! It sends X,Y,Z acceleration and X,Y,Z angular velocity
class IMU_MicroStrain_3DM_GX2: public ModelComponent
{
public:
  //! Initialize
  IMU_MicroStrain_3DM_GX2
  (OptionManager& mgr,
   const std::string& descrName = "IMU_MicroStrain_3DM_GX2",
   const std::string& tagName   = "IMU_MicroStrain_3DM_GX2",
   const std::string& dev       = "/dev/ttyUSB0");

  //! Destructor
  ~IMU_MicroStrain_3DM_GX2();

  //! to select which serial port to use
  void configureSerial(std::string dev);

  //! get acceleration and angular rate
  void getAccelerationAndAngularRate
  (AccelAndAngRateRecord &record);

  //! get magnetometer dircetion and magnitude
  void getMagnetometer(MagnetometerRecord &record);

  //! get roll, pitch, and yaw
  void getRollPitchYaw(RollPitchYawRecord &record);

  //! set the data requested from the IMU
  bool setDataRequested(DataRequested dataRequested);

  DataRequested getDataRequested();

  //! a thread that keeps updating the requested data
  void run();

  //! is there a new data since it is last accessed
  bool newData();

  //! set the flag to run display data thread
  //! NOTE: can only go to effect before ::start1 is called
  void setRunDataDisplayThread(bool run);

protected:
  void start1(); //!< get started
  void stop1();  //!< get stopped

private:

  //! actual get acceleration and angular rate to IMU
  void getAccelerationAndAngularRate();

  //! actual get magnetometer dircetion and magnitude to IMU
  void getMagnetometer();

  //! actual get roll, pitch, and yaw to IMU
  void getRollPitchYaw();

  std::string itsSerialDev;
  nub::soft_ref<Serial> itsSerial;

  DataRequested itsDataRequested;
  pthread_mutex_t itsDataRequestedLock;

  //! results
  AccelAndAngRateRecord itsAccelAndAngRateRecord;
  MagnetometerRecord itsMagnetometerRecord;
  RollPitchYawRecord itsRollPitchYawRecord;
  bool itsNewData;

  bool itsRunDataUpdateThread;
  bool itsRunDataDisplayThread;
  pthread_t itsDataUpdateThread;
  pthread_t itsDataDisplayThread;
  pthread_mutex_t itsResLock;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/*!@file Robots2/Beobot2/Hardware/BeoIMU.C Ice Module for IMU           */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Hardware/BeoIMU.C
// $ $Id: BeoIMU.C 15335 2012-07-19 21:19:36Z beobot $
//
//////////////////////////////////////////////////////////////////////////

#include "Robots/Beobot2/Hardware/BeoIMU.H"
#include "Ice/BeobotEvents.ice.H"

// ######################################################################
BeoIMU::BeoIMU(OptionManager& mgr,
               const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsIMU(new IMU_MicroStrain_3DM_GX2(mgr)),
  //  itsOfs(new OutputFrameSeries(mgr))
  itsTimer(1000000),
  itsCurrMessageID(0)
{
  addSubComponent(itsIMU);
  //  addSubComponent(itsOfs);
  itsTimer.reset();
}

// ######################################################################
void BeoIMU::configureSerial(std::string dev)
{
  itsIMU->configureSerial(dev);
}

// ######################################################################
bool BeoIMU::setDataRequested(DataRequested dataRequested)
{
  return itsIMU->setDataRequested(dataRequested);
}

// ######################################################################
BeoIMU::~BeoIMU()
{ }

// ######################################################################
void BeoIMU::registerTopics()
{
  this->registerPublisher("IMUMessageTopic");
}

// ######################################################################
void BeoIMU::evolve()
{
  // get data from IMU
  if(itsIMU->newData())
    {
      DataRequested dr = itsIMU->getDataRequested();

      // IMUMessage to be sent out
      BeobotEvents::IMUMessagePtr msg =
        new BeobotEvents::IMUMessage;
      msg->RequestID = itsCurrMessageID;

      // data requested for polling mode
      switch(dr)
        {
        case ACCEL_AND_ANG_RATE:
          {
            // get acceleration and angular rate
            AccelAndAngRateRecord record;
            itsIMU->getAccelerationAndAngularRate(record);
            LINFO("Acceleration x:%15.6f y:%15.6f z:%15.6f",
                  record.accelX, record.accelY, record.accelZ);
            LINFO("Angular Rate x:%15.6f y:%15.6f z:%15.6f",
                  record.angRateX , record.angRateY, record.angRateZ);

            msg->validAccAndAng = true;
            msg->accelX = record.accelX;
            msg->accelY = record.accelY;
            msg->accelZ = record.accelZ;

            msg->angRateX = record.angRateX;
            msg->angRateY = record.angRateY;
            msg->angRateZ = record.angRateZ;

            msg->validMagnetometer = false;
            msg->validRollPitchYaw = false;
	    setDataRequested(ROLL_PITCH_YAW);
          }
          break;

        case DELTA_ANG_AND_VEL:
          // FIX: itsIMU->getDeltaAngleAndVelocity();
          break;

        case MAGNETOMETER:
          {
            // get instantaneous magnetometer direction and magnitude
            MagnetometerRecord record;
            itsIMU->getMagnetometer(record);
            LINFO("Magnetometer x:%15.6f y:%15.6f z:%15.6f",
                  record.magX, record.magY, record.magZ);

            msg->validMagnetometer = true;
            msg->magX = record.magX;
            msg->magY = record.magY;
            msg->magZ = record.magZ;

            msg->validAccAndAng    = false;
            msg->validRollPitchYaw = false;
          }
          break;

        case ORIENTATION_MATRIX:
          // FIX: itsIMU->getOrientationMatrix();
          break;

        case ROLL_PITCH_YAW:
          {
            // get roll, pitch, and yaw
            RollPitchYawRecord rpyRecord;
            itsIMU->getRollPitchYaw(rpyRecord);

            msg->validRollPitchYaw = true;

            // YAW has to be reversed
            // because of the IMU configuration in Beobot2.0
            msg->roll  = rpyRecord.roll;
            msg->pitch = rpyRecord.pitch;
            if(rpyRecord.yaw < 0)
              msg->yaw   =  rpyRecord.yaw + M_PI;
            else
              msg->yaw   =  rpyRecord.yaw - M_PI;

            msg->validAccAndAng    = false;
            msg->validMagnetometer = false;

            LINFO("Euler Angle  r:%8.3fdeg   p:%8.3fdeg   y:%8.3fdeg",
                  msg->roll /M_PI*180.0, 
                  msg->pitch/M_PI*180.0, 
                  msg->yaw  /M_PI*180.0 );

	    setDataRequested(ACCEL_AND_ANG_RATE);
          }
          break;

        case TEMPERATURE:
          // FIX: itsIMU->getTemperature();
          break;

        default: LERROR("Unknown data requested: %d", dr);
        }

      //LINFO("[%6d] Publishing IMUMessage", itsCurrMessageID);
      publish("IMUMessageTopic", msg);

      //LINFO("IMU Time: %f", itsTimer.get()/1000.0F);
      itsTimer.reset();
      itsCurrMessageID++;      
    }
}

// ######################################################################
void BeoIMU::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{ }

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

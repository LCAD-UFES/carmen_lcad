/*!@file Devices/GPS_USGlobalSat_.C class for interfacing with the
  USGlobalSat EM-408 GPS */
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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/GPS_USGlobalSat_EM_408.C $
// $Id: GPS_USGlobalSat_EM_408.C 12962 2010-03-06 02:13:53Z irock $
//

#include "Devices/GPS_USGlobalSat_EM_408.H"
#include "Util/BinaryConversion.H"
#include "Util/Timer.H"
#include <GL/glut.h>

// A static pointer to the IMU
static GPS_USGlobalSat_EM_408* theIMU = NULL;

void *GPS_USGlobalSat_EM_408_run(void *c);

// ######################################################################
void *GPS_USGlobalSat_EM_408_run(void *c)
{
  GPS_USGlobalSat_EM_408 *d = (GPS_USGlobalSat_EM_408 *) c;
  d ->run();
  return NULL;
}

// ######################################################################
GPS_USGlobalSat_EM_408::GPS_USGlobalSat_EM_408
(OptionManager& mgr,
 const std::string& descrName,
 const std::string& tagName,
 const std::string& dev) :
  ModelComponent(mgr, descrName, tagName),
  itsSerial(new Serial(mgr))
{
  addSubComponent(itsSerial);
  pthread_mutex_init(&itsResLock, NULL);
  itsRunDataUpdateThread = true;
  itsNewData = false;
}

// ######################################################################
void GPS_USGlobalSat_EM_408::configureSerial(std::string dev)
{
  itsSerialDev = dev;
  itsSerial->configure(itsSerialDev.c_str(), 115200);
}

// ######################################################################
void GPS_USGlobalSat_EM_408::start1()
{
  // thread to update the IMU data
  pthread_create(&itsDataUpdateThread, NULL,
                 &GPS_USGlobalSat_EM_408_run, (void *) this);

  if(theIMU != NULL)
    {
    LINFO("Trying to create a duplicate Gumbot!!!");
    exit(-1);
  }
  theIMU = this;

  // thread to make OpenGL rendering
  pthread_create(&itsDataDisplayThread, NULL,
                 display_thread_function, NULL);
}

// ######################################################################
void GPS_USGlobalSat_EM_408::stop1()
{
  itsRunDataUpdateThread = false;
  usleep(300000); // make sure thread has exited
}

// ######################################################################
GPS_USGlobalSat_EM_408::~GPS_USGlobalSat_EM_408()
{
  pthread_mutex_destroy(&itsResLock);
}

// ######################################################################
bool GPS_USGlobalSat_EM_408::newData()
{
  bool ret;
  pthread_mutex_lock(&itsResLock);
  ret = itsNewData;
  pthread_mutex_unlock(&itsResLock);
  return ret;
}

// ######################################################################
void GPS_USGlobalSat_EM_408::getAccelerationAndAngularRate()
{
  // 0xC2 for acceleration and angular rate
  unsigned char cmd = char(0xC2);
  itsSerial->write(&cmd, 1);

  unsigned char buffer[256];
  itsSerial->read(&buffer,31);

  std::vector<unsigned char> response(buffer, buffer+31);

  //if(dwBytesRead != 31){ return SERIAL_READ_ERROR; }
  if(response[0] == 0xC2) LDEBUG("Angular ACC");
  else                    LINFO("NOT Angular ACC");

  pthread_mutex_lock(&itsResLock);

  // acceleration
  itsAccelAndAngRateRecord.accelX =
    binaryconversion::FloatFromBytes(&response[1], true); //bytes 1.. 4
  itsAccelAndAngRateRecord.accelY =
    binaryconversion::FloatFromBytes(&response[5], true); //bytes 5.. 8
  itsAccelAndAngRateRecord.accelZ =
    binaryconversion::FloatFromBytes(&response[9], true); //bytes 9..12


  // Angular Rate
  itsAccelAndAngRateRecord.angRateX =
    binaryconversion::FloatFromBytes(&response[13], true); //bytes 13.. 16
  itsAccelAndAngRateRecord.angRateY =
    binaryconversion::FloatFromBytes(&response[17], true); //bytes 17.. 20
  itsAccelAndAngRateRecord.angRateZ =
    binaryconversion::FloatFromBytes(&response[21], true); //bytes 21.. 24

  // Timer
  // record->timer = response[25, 26, 27, 28];

  //wChecksum = MAKEWORD(response[30], response[29]);
  //calculate the checkusm, 29 = 31-2 don't include the checksum bytes
  //wCalculatedCheckSum = Checksum(&response[0], 29);
  //if(wChecksum != wCalculatedCheckSum)  CHECKSUM_ERROR;

  itsNewData = true;
  pthread_mutex_unlock(&itsResLock);
}

// ######################################################################
void GPS_USGlobalSat_EM_408::
getAccelerationAndAngularRate(AccelAndAngRateRecord &record)
{
  pthread_mutex_lock(&itsResLock);

  // acceleration
  record.accelX = itsAccelAndAngRateRecord.accelX;
  record.accelY = itsAccelAndAngRateRecord.accelY;
  record.accelZ = itsAccelAndAngRateRecord.accelZ;

  // Angular Rate
  record.angRateX = itsAccelAndAngRateRecord.angRateX;
  record.angRateY = itsAccelAndAngRateRecord.angRateY;
  record.angRateZ = itsAccelAndAngRateRecord.angRateZ;

  itsNewData = false;
  pthread_mutex_unlock(&itsResLock);
}

// ######################################################################
void GPS_USGlobalSat_EM_408::getMagnetometer()
{
  // 0xC7 for magnetometer direction and magnitude
  unsigned char cmd = char(0xC7);
  itsSerial->write(&cmd, 1);

  unsigned char buffer[256];
  itsSerial->read(&buffer,19);

  std::vector<unsigned char> response(buffer, buffer+31);

  if(response[0] == 0xC7) LDEBUG("Magnetometer");
  else                    LINFO("NOT Magnetometer");

  pthread_mutex_lock(&itsResLock);

  // magnetometer
  itsMagnetometerRecord.magX =
    binaryconversion::FloatFromBytes(&response[1], true); //bytes 1.. 4
  itsMagnetometerRecord.magY =
    binaryconversion::FloatFromBytes(&response[5], true); //bytes 5.. 8
  itsMagnetometerRecord.magZ =
    binaryconversion::FloatFromBytes(&response[9], true); //bytes 9..12

  itsNewData = true;
  pthread_mutex_unlock(&itsResLock);
}

// ######################################################################
void GPS_USGlobalSat_EM_408::getMagnetometer(MagnetometerRecord &record)
{
  pthread_mutex_lock(&itsResLock);

  record.magX = itsMagnetometerRecord.magX;
  record.magY = itsMagnetometerRecord.magY;
  record.magZ = itsMagnetometerRecord.magZ;

  itsNewData = false;
  pthread_mutex_unlock(&itsResLock);
}

// ######################################################################
void GPS_USGlobalSat_EM_408::getRollPitchYaw()
{
  // 0xCE for roll, pitch, yaw
  unsigned char cmd = char(0xCE);
  itsSerial->write(&cmd, 1);

  unsigned char buffer[256];
  itsSerial->read(&buffer,19);

  std::vector<unsigned char> response(buffer, buffer+31);

  if(response[0] == 0xCE) LDEBUG("Roll, pitch, yaw");
  else                    LINFO("NOT roll, pitch, yaw");

  pthread_mutex_lock(&itsResLock);

  // acceleration
  itsRollPitchYawRecord.roll  =
    binaryconversion::FloatFromBytes(&response[1], true); //bytes 1.. 4
  itsRollPitchYawRecord.pitch =
    binaryconversion::FloatFromBytes(&response[5], true); //bytes 5.. 8
  itsRollPitchYawRecord.yaw   =
    binaryconversion::FloatFromBytes(&response[9], true); //bytes 9..12

  itsNewData = true;
  pthread_mutex_unlock(&itsResLock);
}

// ######################################################################
void GPS_USGlobalSat_EM_408::getRollPitchYaw(RollPitchYawRecord &record)
{
  pthread_mutex_lock(&itsResLock);

  record.roll  = itsRollPitchYawRecord.roll;
  record.pitch = itsRollPitchYawRecord.pitch;
  record.yaw   = itsRollPitchYawRecord.yaw;

  itsNewData = false;
  pthread_mutex_unlock(&itsResLock);
}

// ######################################################################
void GPS_USGlobalSat_EM_408::run()
{
  Timer t(1000000);
  t.reset();

  while(itsRunDataUpdateThread)
    {
//       do {
//       } while ((checksum & 0x0000FFFF)); // exit iff checksum correct

      // get Acceleration and Angular Rate
      //getAccelerationAndAngularRate();
      //getMagnetometer();
      getRollPitchYaw();

      usleep(1000);
      LDEBUG("time %11.5f", t.get()/1000.0F);
      t.reset();
    }

  pthread_exit(0);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

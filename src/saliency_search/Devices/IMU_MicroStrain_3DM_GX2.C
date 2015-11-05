/*!@file Devices/IMU_MicroStrain_3DM-GX2.C class for interfacing with the
  MicroStrain 3DM-GX2 IMU */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/IMU_MicroStrain_3DM_GX2.C $
// $Id: IMU_MicroStrain_3DM_GX2.C 15293 2012-05-19 03:48:08Z beobot $
//
// NOTE: This IMU has 2 reporting mode. One is a continuous mode and another
//       is a polled mode. Current default mode is polled mode.
//
//       Continuous mode is setting a continuous command address to
//       the desired command (this is done using test-serial) and it
//       will do it continuously with a specified interval (can be
//       changed by writing to the EEPROM address OxFCA2. Read pg. 29
//       of the manual). Default 100Hz.
//
//       Polled mode allow the user to inquire a one time data.
//       If both modes are running, there will be two data being sent, the
//       first one from the continuous mode, the the polled command.

// TODO:
//  Get the orientation Matrix M and change matrix C
//    (Euler Angle can be calculated with this)
//          Magnetometer
//          ACC bias/ gyro bias <- to zero out stuff
//  OPENGL setup for other information?
//  temperature?

#include "Devices/IMU_MicroStrain_3DM_GX2.H"
#include "Util/BinaryConversion.H"
#include "Util/Timer.H"
#include <GL/gl.h>
#undef APIENTRY // otherwise it gets redefined between gl.h and glut.h???
#include <GL/glut.h>

// A static pointer to the IMU
static IMU_MicroStrain_3DM_GX2* theIMU = NULL;

void *IMU_MicroStrain_3DM_GX2_run(void *c);

// ######################################################################
void *IMU_MicroStrain_3DM_GX2_run(void *c)
{
  IMU_MicroStrain_3DM_GX2 *d = (IMU_MicroStrain_3DM_GX2 *) c;
  d ->run();
  return NULL;
}

// ######################################################################
void renderScene(void)
{
  // get IMU attitude
  RollPitchYawRecord rpyRecord;
  theIMU->getRollPitchYaw(rpyRecord);
  LDEBUG("Euler Angle  r:%15.6f p:%15.6f y:%15.6f",
         rpyRecord.roll, rpyRecord.pitch, rpyRecord.yaw);

  // visualize the IMU attitude

  // notice that we're now clearing the depth buffer
  // as well this is required, otherwise the depth buffer
  // gets filled and nothing gets rendered.
  // Try it out, remove the depth buffer part.
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Reset The Current Modelview Matrix
  glLoadIdentity();

  // save the previous settings, in this case save
  // we're refering to the camera settings.
  //glPushMatrix();

  float sz = 0.25;
  glRotatef( rpyRecord.pitch/M_PI * 180.0,  1.0f,0.0f,0.0f);
  glRotatef( rpyRecord.yaw  /M_PI * 180.0,  0.0f,1.0f,0.0f);
  glRotatef( rpyRecord.roll /M_PI * 180.0,  0.0f,0.0f,1.0f);

  glBegin(GL_QUADS);
  glColor3f(0.0f, 0.0f, 1.0f);        // Color Blue
  glVertex3f(-sz, -sz, -sz);
  glVertex3f(-sz,  sz, -sz);
  glVertex3f( sz,  sz, -sz);
  glVertex3f( sz, -sz, -sz);

  glColor3f(1.0f, 1.0f, 0.0f);        // Color Yellow
  glVertex3f(-sz, -sz,  sz);
  glVertex3f(-sz,  sz,  sz);
  glVertex3f( sz,  sz,  sz);
  glVertex3f( sz, -sz,  sz);

  glColor3f(1.0f, 0.0f, 0.0f);        // Color Red
  glVertex3f(-sz, -sz, -sz);
  glVertex3f(-sz, -sz,  sz);
  glVertex3f(-sz,  sz,  sz);
  glVertex3f(-sz,  sz, -sz);

  glColor3f(0.0f, 1.0f,1.0f);        // Color Cyan
  glVertex3f( sz, -sz, -sz);
  glVertex3f( sz, -sz,  sz);
  glVertex3f( sz,  sz,  sz);
  glVertex3f( sz,  sz, -sz);

  glColor3f(0.0f, 1.0f, 0.0f);        // Color Green
  glVertex3f(-sz, -sz, -sz);
  glVertex3f(-sz, -sz,  sz);
  glVertex3f( sz, -sz,  sz);
  glVertex3f( sz, -sz, -sz);

  glColor3f(1.0f,0.0f,1.0f);        // Color Magenta
  glVertex3f(-sz,  sz, -sz);
  glVertex3f(-sz,  sz,  sz);
  glVertex3f( sz,  sz,  sz);
  glVertex3f( sz,  sz, -sz);

  glEnd();

  //  glPopMatrix();

  // swapping the buffers causes the rendering above to be shown
  glutSwapBuffers();
}

// ######################################################################
void *display_thread_function(void *ptr)
{
  //Initialize the GLUT runtime with some bogus command line args
  int argc = 1;
  char** argv = new char*[1];
  argv[0] = new char[4];
  glutInit(&argc, argv);

  // This is where we say that we want a double buffer
  glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);

  glutInitWindowPosition(100,100);
  glutInitWindowSize(320,320);
  glutCreateWindow("IMU Visualization");
  glutDisplayFunc(renderScene);

  // here is the setting of the idle function
  glutIdleFunc(renderScene);

  //glutReshapeFunc(changeSize);

  // enable depth testing
  glEnable(GL_DEPTH_TEST);
  glutMainLoop();
  return NULL;
}

// ######################################################################
IMU_MicroStrain_3DM_GX2::IMU_MicroStrain_3DM_GX2
(OptionManager& mgr,
 const std::string& descrName,
 const std::string& tagName,
 const std::string& dev) :
  ModelComponent(mgr, descrName, tagName),
  itsSerial(new Serial(mgr))
{
  // Default Data requested is roll, pitch, yaw
  //itsDataRequested = MAGNETOMETER;
  itsDataRequested = ROLL_PITCH_YAW;

  addSubComponent(itsSerial);
  pthread_mutex_init(&itsResLock, NULL);
  pthread_mutex_init(&itsDataRequestedLock, NULL);
  itsRunDataDisplayThread = false;
  itsRunDataUpdateThread  = true;
  itsNewData = false;
}

// ######################################################################
void IMU_MicroStrain_3DM_GX2::setRunDataDisplayThread(bool run)
{
  // NOTE: can only go to effect before ::start1 is called
  itsRunDataDisplayThread = run;
}

// ######################################################################
void IMU_MicroStrain_3DM_GX2::configureSerial(std::string dev)
{
  itsSerialDev = dev;
  itsSerial->configure(itsSerialDev.c_str(), 115200);
}

// ######################################################################
void IMU_MicroStrain_3DM_GX2::start1()
{
  // thread to update the IMU data
  pthread_create(&itsDataUpdateThread, NULL,
                 &IMU_MicroStrain_3DM_GX2_run, (void *) this);

  if(theIMU != NULL)
    {
      LINFO("Trying to create a duplicate IMU!!!");
      exit(-1);
    }
  theIMU = this;

  // thread to make OpenGL rendering
  if(itsRunDataDisplayThread)
    pthread_create(&itsDataDisplayThread, NULL,
                   display_thread_function, NULL);

  itsRollPitchYawRecord.roll  = 0.0;
  itsRollPitchYawRecord.pitch = 0.0;
  itsRollPitchYawRecord.yaw   = 0.0;
}

// ######################################################################
void IMU_MicroStrain_3DM_GX2::stop1()
{
  itsRunDataUpdateThread = false;
  usleep(300000); // make sure thread has exited
}

// ######################################################################
IMU_MicroStrain_3DM_GX2::~IMU_MicroStrain_3DM_GX2()
{
  pthread_mutex_destroy(&itsResLock);
  pthread_mutex_destroy(&itsDataRequestedLock);
}

// ######################################################################
bool IMU_MicroStrain_3DM_GX2::setDataRequested(DataRequested dataRequested)
{
  // double check if it's a valid dataRequested
  switch(dataRequested)
    {
    case ACCEL_AND_ANG_RATE: break;
    case DELTA_ANG_AND_VEL:  break;
    case MAGNETOMETER:       break;
    case ORIENTATION_MATRIX: break;
    case ROLL_PITCH_YAW:     break;
    case TEMPERATURE:        break;

    default: LERROR("Unknown data requested: %d", dataRequested);
      return false;
    }

  pthread_mutex_lock(&itsDataRequestedLock);
  itsDataRequested = dataRequested;
  pthread_mutex_unlock(&itsDataRequestedLock);

  return true;
}

// ######################################################################
DataRequested IMU_MicroStrain_3DM_GX2::getDataRequested()
{
  pthread_mutex_lock(&itsDataRequestedLock);
  DataRequested dataRequested = itsDataRequested;
  pthread_mutex_unlock(&itsDataRequestedLock);

  return dataRequested;
}

// ######################################################################
bool IMU_MicroStrain_3DM_GX2::newData()
{
  bool ret;
  pthread_mutex_lock(&itsResLock);
  ret = itsNewData;
  pthread_mutex_unlock(&itsResLock);
  return ret;
}

// ######################################################################
void IMU_MicroStrain_3DM_GX2::getAccelerationAndAngularRate()
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
void IMU_MicroStrain_3DM_GX2::
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
void IMU_MicroStrain_3DM_GX2::getMagnetometer()
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
void IMU_MicroStrain_3DM_GX2::getMagnetometer(MagnetometerRecord &record)
{
  pthread_mutex_lock(&itsResLock);

  record.magX = itsMagnetometerRecord.magX;
  record.magY = itsMagnetometerRecord.magY;
  record.magZ = itsMagnetometerRecord.magZ;

  itsNewData = false;
  pthread_mutex_unlock(&itsResLock);
}

// ######################################################################
void IMU_MicroStrain_3DM_GX2::getRollPitchYaw()
{
  // 0xCE for roll, pitch, yaw
  unsigned char cmd = char(0xCE);
  itsSerial->write(&cmd, 1);

  unsigned char buffer[256];
  itsSerial->read(&buffer,19);

  std::vector<unsigned char> response(buffer, buffer+31);

  if(response[0] == 0xCE) LDEBUG("Roll, pitch, yaw");
  else                    LDEBUG("NOT roll, pitch, yaw: %d", response[0]);

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
void IMU_MicroStrain_3DM_GX2::getRollPitchYaw(RollPitchYawRecord &record)
{
  pthread_mutex_lock(&itsResLock);

  // NOTES: we flipped the direction of angle for the YAW 
  //        to conform with convention that turning left increases angle
  //        while turning right decreases angle magnitude

  record.roll  = itsRollPitchYawRecord.roll;
  record.pitch = itsRollPitchYawRecord.pitch;
  record.yaw   = -itsRollPitchYawRecord.yaw;

  itsNewData = false;
  pthread_mutex_unlock(&itsResLock);
}

// ######################################################################
void IMU_MicroStrain_3DM_GX2::run()
{
  Timer t(1000000);
  t.reset();

  while(itsRunDataUpdateThread)
    {
//       do {
//       } while ((checksum & 0x0000FFFF)); // exit iff checksum correct

      // if continuous mode is also set
      // then we have to read those data first before processing
      // polling mode data

      pthread_mutex_lock(&itsDataRequestedLock);
      DataRequested dr = itsDataRequested;
      pthread_mutex_unlock(&itsDataRequestedLock);

      // data requested for polling mode
      switch(dr)
        {
        case ACCEL_AND_ANG_RATE:
          getAccelerationAndAngularRate();
          break;

        case DELTA_ANG_AND_VEL:
          // FIX: getDeltaAngleAndVelocity();
          break;

        case MAGNETOMETER:
          getMagnetometer();
          break;

        case ORIENTATION_MATRIX:
          // FIX: getOrientationMatrix();
          break;

        case ROLL_PITCH_YAW:
          getRollPitchYaw();
          break;

        case TEMPERATURE:
          // FIX: getTemperature();
          break;

        default: LERROR("Unknown data requested: %d",
                        itsDataRequested);
        }

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

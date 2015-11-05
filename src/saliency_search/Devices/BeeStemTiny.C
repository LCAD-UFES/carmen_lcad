/*!@file Devices/BeeStemTiny.C Simple interface to beestem */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/BeeStemTiny.C $
// $Id: BeeStemTiny.C 10794 2009-02-08 06:21:09Z itti $
//

#include "Devices/BeeStemTiny.H"

#include "Component/OptionManager.H"
#include "Devices/Serial.H"
#include <string>

#define BS_CMD_DELAY 5000000

// ######################################################################
BeeStemTiny::BeeStemTiny(OptionManager& mgr, const std::string& descrName,
         const std::string& tagName, const char *defdev, const char *defdev2) :
  ModelComponent(mgr, descrName, tagName),
  itsPort(new Serial(mgr)),
  itsCompassPort(new Serial(mgr))
{


  // set a default config for our serial port:
  itsPort->configure(defdev, 38400, "8N1", false, false, 1);

  itsCompassPort->configure(defdev2, 2400, "8N1", false, false, 1);

  // attach our port as a subcomponent:
  addSubComponent(itsPort);
  addSubComponent(itsCompassPort);

        for (int i=0; i<5; i++)
                itsLastMotorCmd[i] = 0;


  pthread_mutex_init(&itsSerialLock, NULL);
}

// ######################################################################
BeeStemTiny::~BeeStemTiny()
{
  pthread_mutex_destroy(&itsSerialLock);
}

// ######################################################################
//The crappyCompass functionality can map crappy compass values to real headings as long as the crappy input is consistent.
//  It does this by running a calibration routined which reads in as many value as possible and stores them in a
//  vector. Then, when a user asks the crappyCompass class for a real heading, they can pass in a crappy value, and
//  the BeeStem will map it to a point in its crappy vector, returning the index.
//Just make sure to start the calibration at a value that doesn't get ever get crappy.
//A better way to do this is to prompts for user input
bool BeeStemTiny::calibrateCrappyCompass() {


  int starting_point;
  char sensor_reports[3];
  char cmd = 102;

  //char* values = NULL;
  pthread_mutex_lock(&itsSerialLock);
  itsCompassPort->write(&cmd, 1);
  pthread_mutex_unlock(&itsSerialLock);

  if( itsCompassPort->read(sensor_reports, 3) != 3 ) {
    return false;
  }

  //Get our starting point
  starting_point = sensor_reports[0];

  return true;

}


// ######################################################################
bool BeeStemTiny::setThrusters(int &m1, int &m2, int &m3,
                int &m4, int &m5)
{
  // Command Buffer:
  // [0] is start character
  // [1..5] is m1 m5

  pthread_mutex_lock(&itsSerialLock);

  if (m1 > MOTOR_MAX) m1 = MOTOR_MAX; if (m1 < -MOTOR_MAX) m1 = -MOTOR_MAX;
  if (m2 > MOTOR_MAX) m2 = MOTOR_MAX; if (m2 < -MOTOR_MAX) m2 = -MOTOR_MAX;
  if (m3 > MOTOR_MAX) m3 = MOTOR_MAX; if (m3 < -MOTOR_MAX) m3 = -MOTOR_MAX;
  if (m4 > MOTOR_MAX) m4 = MOTOR_MAX; if (m4 < -MOTOR_MAX) m4 = -MOTOR_MAX;
  if (m5 > MOTOR_MAX) m5 = MOTOR_MAX; if (m5 < -MOTOR_MAX) m5 = -MOTOR_MAX;



  if (abs(itsLastMotorCmd[0] - m1) > 60 && itsLastMotorCmd[0]*m1 < 0) m1 = 0;
  if (abs(itsLastMotorCmd[1] - m2) > 60 && itsLastMotorCmd[1]*m2 < 0) m2 = 0;
  if (abs(itsLastMotorCmd[2] - m3) > 60 && itsLastMotorCmd[2]*m3 < 0) m3 = 0;
  if (abs(itsLastMotorCmd[3] - m4) > 60 && itsLastMotorCmd[3]*m4 < 0) m4 = 0;
  if (abs(itsLastMotorCmd[4] - m5) > 60 && itsLastMotorCmd[4]*m5 < 0) m5 = 0;

  //LINFO("%i %i %i %i %i", m1, m2, m3, m4, m5);

  itsLastMotorCmd[0] = m1;
  itsLastMotorCmd[1] = m2;
  itsLastMotorCmd[2] = m3;
  itsLastMotorCmd[3] = m4;
  itsLastMotorCmd[4] = m5;



  char command[6];
  command[0] = 101;
  command[1] = (signed char)m1;
  command[2] = (signed char)m2;
  command[3] = (signed char)m3;
  command[4] = (signed char)m4;
  command[5] = (signed char)m5;

//        LINFO("%i %i %i %i %i",
//                                                                        command[0], command[1], command[2], command[3], command[4], command[5]);

  for(int i = 0; i<6; i++) {
    itsPort->write(&command[i], 1);
    //usleep(BS_CMD_DELAY);
 }

  pthread_mutex_unlock(&itsSerialLock);

  return true;


  /*
  // write command buffer
  if (itsPort->write(command, 6) == 6)
          return true;
  else
  return false;*/
}

// ######################################################################
bool BeeStemTiny::getSensors(int &heading, int &pitch, int &roll, int &ext_pressure, int &int_pressure)
{


  //First let's handle incoming compass data (not actually related to the BeeStem
  //at this point, but whatever.
  float h,p,r;
  static std::string compassValues;
  char compassBuffer[50];

  //Read as many bytes from the serial buffer as are available
  int size = itsCompassPort->read(&compassBuffer, 50);

  if (size > 0) //only update if we have any values
  {
      compassBuffer[size] = 0;
      compassValues+=std::string(compassBuffer);
  }

  //Once we have at least 30 bytes we know that we have a frame, so we can parse it.
  if(compassValues.size() > 30) {
//    LINFO("COMPASS VALUES: %s", compassValues.c_str());
    int frameBegin = compassValues.find('\n');
    int frameEnd = compassValues.find('\n', frameBegin+1);
    std::string compassFrame = compassValues.substr(frameBegin, frameEnd-frameBegin);
    sscanf(compassValues.substr(frameBegin).c_str(), "\r%f,%f,%f", &h, &p, &r);

    heading = (int)h;
    pitch = (int)p;
    roll = (int)r;

    compassValues.clear();
  }


  //Now we deal with the actual BeeStem
  // Command Buffer:
  // [0] is start character
  // [1..5] is the data
  pthread_mutex_lock(&itsSerialLock);
  char cmd = 102;
  unsigned char presBuff[10];
  itsPort->write(&cmd, 1);
  int presSize = itsPort->read(presBuff, 10);

  if(presSize == 4) {
    itsAvgDepth.push_back(((unsigned int)presBuff[2] | ((unsigned int)presBuff[3] << 8)));

    if (itsAvgDepth.size() > 40)
      {
        long avg = 0;
        for(std::list<int>::iterator itr=itsAvgDepth.begin();
            itr != itsAvgDepth.end(); ++itr)
          avg += *itr;
        itsAvgDepth.pop_front();

        ext_pressure = (unsigned int)(avg/itsAvgDepth.size());
      }

    //ext_pressure = ((unsigned int)presBuff[2] | ((unsigned int)presBuff[3] << 8));
  }

  int_pressure = ((unsigned int)presBuff[0] | ((unsigned int)presBuff[1] << 8));


  // for(int i=0; i<presSize; i++)
  //  LINFO("presBuff[%d] = %d", i, presBuff[i]);

  /*if (presSize > 4)
    {
      unsigned char val = ((unsigned char)presBuff[3])-227;
      int_pressure = val;

      itsAvgDepth.push_back((unsigned char)presBuff[4]);
      if (itsAvgDepth.size() > 40)
        {
          long avg = 0;
          for(std::list<int>::iterator itr=itsAvgDepth.begin();
              itr != itsAvgDepth.end(); ++itr)
            avg += *itr;
          itsAvgDepth.pop_front();

          ext_pressure = (int)(avg/itsAvgDepth.size());
        }
        }*/
  //LINFO("Ext Pressure: %d, Int Pressure: %d, presSize %d\n", ext_pressure, int_pressure, presSize);
  pthread_mutex_unlock(&itsSerialLock);



  return true;
}

bool BeeStemTiny::setHeartBeat()
{
  pthread_mutex_lock(&itsSerialLock);
  char cmd = 103;

  // write command buffer
  bool ret = (itsPort->write(&cmd, 1) == 1);

  pthread_mutex_unlock(&itsSerialLock);

  return ret;

}



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/*!@file Devices/IMU_IMU_SFE_Atomic.C class
 for interfacing with the Spark Fun Electronics Atomic IMU */

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


#ifndef IMU_SFE_ATOMIC_C
#define IMU_SFE_ATOMIC_C

#include "Devices/IMU_SFE_Atomic.H"
#include "Util/Assert.H"

IMU_SFE_Atomic::IMU_SFE_Atomic(OptionManager& mgr,
    const std::string& descrName,
    const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsSerial(new Serial(mgr, "IMU", "IMU"))
{
  addSubComponent(itsSerial);

  itsSerial->configure("/dev/ttyUSB0", 115200);
}

void IMU_SFE_Atomic::start2()
{
  //Tell the Atomic IMU to start sending data
  unsigned char hash = '#';
  itsSerial->write(&hash, 1);
}

IMU_SFE_Atomic::IMUData IMU_SFE_Atomic::readIMUData()
{

  IMUData data;
  std::vector<unsigned char> raw_data = itsSerial->readFrame('A', 'Z', 14);

  ASSERT(raw_data.size() == 14);

  data.count  = (raw_data[0]  << 8) | raw_data[1];
  data.accelX = (raw_data[2]  << 8) | raw_data[3];
  data.accelY = (raw_data[4]  << 8) | raw_data[5];
  data.accelZ = (raw_data[6]  << 8) | raw_data[7];
  data.pitch  = (raw_data[8]  << 8) | raw_data[9];
  data.roll   = (raw_data[10] << 8) | raw_data[11];
  data.yaw    = (raw_data[12] << 8) | raw_data[13];

  return data;
}

#endif

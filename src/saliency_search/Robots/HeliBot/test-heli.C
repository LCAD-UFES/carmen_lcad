/*!@file Robots/HeliBot/test-heli.C Test the heli */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/HeliBot/test-heli.C $
// $Id: test-heli.C 13901 2010-09-09 15:12:26Z lior $
//

#include "Robots/HeliBot/HeliPose.H"
#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "Image/ColorOps.H"
#include "Image/FilterOps.H"
#include "GUI/ImageDisplayStream.H"
#include "GUI/XWinManaged.H"
#include "Devices/Serial.H"
#include "Devices/WiiMote.H"
#include "Util/CpuTimer.H"
#include "Util/Timer.H"
#include "Image/MathOps.H"
#include "Image/Transforms.H"
#include "Image/Layout.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Raster/Raster.H"
#include "GUI/XWinManaged.H"
#include "GUI/DebugWin.H"
#include "Controllers/PID.H"
#include "Devices/IMU_SFE_Atomic.H"

#include <pthread.h>

#define KEY_UP 98
#define KEY_DOWN 104
#define KEY_LEFT 100
#define KEY_RIGHT 102

int getKey(nub::ref<OutputFrameSeries> &ofs)
{
  const nub::soft_ref<ImageDisplayStream> ids =
    ofs->findFrameDestType<ImageDisplayStream>();

  const rutz::shared_ptr<XWinManaged> uiwin =
    ids.is_valid()
    ? ids->getWindow("Output")
    : rutz::shared_ptr<XWinManaged>();
  return uiwin->getLastKeyPress();
}

Point2D<int> getMouseClick(nub::ref<OutputFrameSeries> &ofs, const char* wname)
{
  const nub::soft_ref<ImageDisplayStream> ids =
    ofs->findFrameDestType<ImageDisplayStream>();

  const rutz::shared_ptr<XWinManaged> uiwin =
    ids.is_valid()
    ? ids->getWindow(wname)
    : rutz::shared_ptr<XWinManaged>();

  if (uiwin.is_valid())
    return uiwin->getLastMouseClick();
  else
    return Point2D<int>(-1,-1);
}

struct RadioStatus
{
  int thr;
  int elevator;
  int aileron;
  int yaw;

  bool pushButton;
  bool rightSwitch;
  bool leftSwitch;
};


void drawWorld(float x, float y, float z, float rot, nub::ref<OutputFrameSeries>& ofs)
{

  Image<PixRGB<byte> > world(255, 255, ZEROS);
  Point2D<int> loc(128+(int)(x/4.0), 128+(int)(y/4.0));

  float heliLength = 30;
  int headX = int(cos(rot)*heliLength/2);
  int headY = int(sin(rot)*heliLength/2);

  drawCircle(world, Point2D<int>(loc.i + headX, loc.j - headY), 6, PixRGB<byte>(0,255,0), 3);
  drawLine(world, loc, rot, heliLength, PixRGB<byte>(0,255,0), 3);

  ofs->writeRGB(world, "world", FrameInfo("world", SRC_POS));
}

RadioStatus getRadioStatus(nub::ref<Serial>& serial)
{
  RadioStatus radioStatus;

  std::vector<unsigned char> data = serial->readFrame(0, 255); //start frame 0 end frame 255

  if(data.size() == 9)
  {

    radioStatus.thr        = (data[0]) | (data[1] << 8);
    radioStatus.elevator   = (data[2]) | (data[3] << 8);
    radioStatus.aileron    = (data[4]) | (data[5] << 8);
    radioStatus.yaw        = (data[6]) | (data[7] << 8);
    unsigned char switches = data[8];

    radioStatus.pushButton = switches & 0x04;
    radioStatus.rightSwitch = switches & 0x08;
    radioStatus.leftSwitch = switches & 0x10;
  } else {
    LERROR("BAD RADIO FRAME SIZE!");
    radioStatus.thr = -1;
    radioStatus.elevator = -1;
    radioStatus.aileron = -1;
    radioStatus.yaw = -1;
  }


  return radioStatus;

}

void sendRadioStatus(const RadioStatus& radioStatus, nub::ref<Serial>& serial)
{
  unsigned char data[10];

  data[0] = 255;
  data[1] = 255;

  data[2] = (radioStatus.thr >> 8);
  data[3] = radioStatus.thr;

  data[4] = (radioStatus.elevator >> 8);
  data[5] = radioStatus.elevator;

  data[6] = (radioStatus.aileron >> 8);
  data[7] = radioStatus.aileron;

  data[8] = (radioStatus.yaw >> 8);
  data[9] = radioStatus.yaw;

  serial->write(data, 10);
}

void tunePID(const RadioStatus& radioStatus, PID<float>& pid)
{
    ///Set the pid gains according to the switches
    if (!radioStatus.pushButton)
    {
      if (radioStatus.leftSwitch)
        pid.setPIDPgain(pid.getPIDPgain() + 0.0001);
      else
      {
        float gain = pid.getPIDPgain() - 0.0001;
        if (gain > 0)
          pid.setPIDPgain(gain);
      }
      LINFO("PID p gain %f", pid.getPIDPgain());
    }
}

int main(int argc, const char **argv)
{
  // Instantiate a ModelManager:
  ModelManager manager("Test wiimote");

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  nub::ref<Serial> serial(new Serial(manager));
  manager.addSubComponent(serial);

  nub::ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  //manager.addSubComponent(ifs);

  nub::ref<HeliPose> heliPose(new HeliPose(manager, ifs, ofs));
  manager.addSubComponent(heliPose);

  PID<float> yawPid(0.0010, 0.01, 0.0, -1, 1, 0, 0, 0, 1.0, -1.0);
  PID<float> posXPid(0.0010, 0.0, 0.0, -1, 1, 0, 0, 0, 1.0, -1.0);
  PID<float> posYPid(0.00270, 0.0, 0.0, -1, 1, 0, 0, 0, 1.0, -1.0);

  serial->configure("/dev/ttyUSB1", 115200, "8N1", false, false, 0);

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv, "", 0, 0) == false) return(1);

  manager.start();

  Timer timer;
  timer.reset();

  LINFO("Press enter to calculate the mean/std of the IMU");
  getchar();
  heliPose->getIMUBias();
  LINFO("Done");

  float frame = 0;
  float currentYaw = 0;
  while(1)
  {

    //draw world
    HeliPose::Pose pose = heliPose->getPose();

    drawWorld(pose.translation.x,pose.translation.y,pose.translation.z,pose.rotation.z, ofs);

    RadioStatus radioStatus; // = getRadioStatus(serial);

    //Raw data out
    printf("%f %i %i %i %i %f %f %f %f %f %f\n",
        timer.getSecs(),
        radioStatus.thr, radioStatus.elevator, radioStatus.aileron, radioStatus.yaw,
        pose.accelX, pose.accelY, pose.accelZ,
        pose.roll, pose.pitch, pose.yaw);


    //printf("acc %f velocity = %f Translation.x = %f\n",
    //    pose.accelX, pose.velocity.x, pose.translation.x);
    fflush(stdout);

    tunePID(radioStatus, yawPid);

    //float elevatorInput = posYPid.update(0, -1*pose.translation.y);
    //float aileronInput = posXPid.update(0, pose.translation.x);
    if (frame > 500)
    {
      frame = 0;
      currentYaw = pose.rotation.z*180/M_PI;
    } else {
      frame++;
    }

    if (pose.valid)
    {
      float yawInput = -1*yawPid.update(currentYaw, pose.rotation.z*180/M_PI);

    //  radioStatus.elevator += (int)(elevatorInput*(1523-510));
    //  radioStatus.aileron += (int)(aileronInput*(1523-510));
        radioStatus.yaw += (int)(yawInput*(1523-510));

       // LINFO("YawInput(d%0.f) %f,%f,%i",currentYaw, pose.rotation.z*180/M_PI,
       //     yawInput, radioStatus.yaw);
    }

  //  LINFO("ElevatorInput %f,%f,%i",pose.translation.y, elevatorInput, radioStatus.elevator);
  //  LINFO("AileronInput %f,%f,%i",pose.translation.x, aileronInput, radioStatus.aileron);

    sendRadioStatus(radioStatus, serial);

    ofs->updateNext();
    frame++;
  }

  // stop all our ModelComponents
  manager.stop();

  // all done!
  return 0;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

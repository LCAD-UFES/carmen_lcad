/*!@file AppDevices/test-armSim.C Test the sub simulator */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/BeoHawk/test-BeoHawkSim.C $
// $Id: test-BeoHawkSim.C 12962 2010-03-06 02:13:53Z irock $
//



#include "Component/ModelManager.H"
#include "Raster/GenericFrame.H"
#include "Image/Layout.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Image/MatrixOps.H"
#include "GUI/ImageDisplayStream.H"
#include "GUI/XWinManaged.H"
#include "Devices/Serial.H"
#include "Robots/BeoHawk/BeoHawkSim.H"
#include <stdio.h>
#include <stdlib.h>

void handle_keys(nub::soft_ref<OutputFrameSeries> ofs, nub::soft_ref<BeoHawkSim> harrierSim)
{
  //handle keyboard input
  const nub::soft_ref<ImageDisplayStream> ids =
    ofs->findFrameDestType<ImageDisplayStream>();

  const rutz::shared_ptr<XWinManaged> uiwin =
    ids.is_valid()
    ? ids->getWindow("harrierSim")
    : rutz::shared_ptr<XWinManaged>();

  int key = uiwin->getLastKeyPress();
  if (key != -1)
  {
    float panThruster = 0;
    float tiltThruster = 0;
    float forwardThruster = 0;
    float upThruster = 0;
    switch(key)
    {
      case 38: upThruster = -3.0; break; //a
      case 52: upThruster = 3.0; break; //z
      case 33: panThruster = 1.0; break; //p
      case 32: panThruster = -1.0; break; //o
      case 40: forwardThruster = 1.0; break; //d
      case 54: forwardThruster = -1.0; break; //c
      case 39: tiltThruster = 1.0; break; //s
      case 53: tiltThruster = -1.0; break; //x
      case 65: //stop
               panThruster = 0;
               tiltThruster = 0;
               forwardThruster = 0;
               upThruster = 0;
               break; // space

    }
    harrierSim->setThrusters(panThruster, tiltThruster, forwardThruster, upThruster);

    LINFO("Key is %i\n", key);
  }
}

struct RadioStatus
{
  int thr;
  int elevator;
  int aileron;
  int yaw;

  int ch1;
  int ch2;
  int ch3;
  int ch4;

};

RadioStatus getRadioStatus(nub::ref<Serial> serial)
{
  RadioStatus radioStatus;

  std::vector<unsigned char> data = serial->readFrame(252, 85, 16); //start frame 0 end frame 255

  if(data.size() == 16)
  {

    radioStatus.aileron    = (data[0] <<8) | (data[1] );
    radioStatus.elevator   = (data[2] <<8) | (data[3] );
    radioStatus.thr         = (data[4] <<8) | (data[5] );
    radioStatus.yaw        = (data[6] <<8) | (data[7] );

    radioStatus.ch1   = (data[8]  << 8) | (data[9] );
    radioStatus.ch2   = (data[10] << 8) | (data[11] );
    radioStatus.ch3   = (data[12] << 8) | (data[13] );
    radioStatus.ch4   = (data[14] << 8) | (data[15] );

  } else {
    LERROR("BAD RADIO FRAME SIZE!");
    radioStatus.thr = -1;
    radioStatus.elevator = -1;
    radioStatus.aileron = -1;
    radioStatus.yaw = -1;
  }


  return radioStatus;

}


int main(int argc, char *argv[])
{
  // Instantiate a ModelManager:
  ModelManager manager("Sub Simulator");

  //nub::ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  //manager.addSubComponent(ifs);

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  // Instantiate our various ModelComponents:
  nub::soft_ref<BeoHawkSim> harrierSim(new BeoHawkSim(manager));
  manager.addSubComponent(harrierSim);

  nub::soft_ref<Serial> serial(new Serial(manager));
  manager.addSubComponent(serial);

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv, "", 0, 0) == false) return(1);

  serial->configure("/dev/ttyUSB0", 115200, "8N1", false, false, 0);


  // let's get all our ModelComponent instances started:
  manager.start();


  while(1){
    Layout<PixRGB<byte> > outDisp;

    harrierSim->simLoop();
    Image<PixRGB<byte> > downwardCam = flipVertic(harrierSim->getFrame(-1));
    ofs->writeRGB(downwardCam, "harrierSim", FrameInfo("harrierSim", SRC_POS));

    //RadioStatus radioStatus = getRadioStatus(serial);
    ////printf("%i\t\t%i\t\t%i\t\t%i\t\t%i\t\t%i\t\t%i\t\t %i\n",
    ////    radioStatus.thr, radioStatus.elevator, radioStatus.aileron, radioStatus.yaw,
    ////    radioStatus.ch1, radioStatus.ch2, radioStatus.ch3, radioStatus.ch4 );
    //

  handle_keys(ofs, harrierSim);
    //float upThruster = -25*((float)radioStatus.thr* (1.0/(1890.0-1130.0)) - 1.498684);
    //float panThruster = -4.0*((float)radioStatus.yaw* (1.0/(1845.0-1070.0)) - 1.917419);
    //float pitchThruster = -4.0*((float)radioStatus.elevator* (1.0/(1890.0-1060.0)) - 1.709639);
    //float rollThruster = 4.0*((float)radioStatus.aileron* (1.0/(1860.0-1050.0)) - 1.919753);
    ////LINFO("%f %f %f %f",
    ////    upThruster,
    ////    panThruster,
    ////    pitchThruster,
    ////    rollThruster);
    //harrierSim->setThrusters(panThruster, pitchThruster, rollThruster, upThruster);

  }
  return 0;

}

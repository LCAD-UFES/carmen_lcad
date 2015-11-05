/*!@file Util/LEDcontroller.C LED controller class */

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
// Primary maintainer for this file: Dicky Nauli Sihite <sihite@usc.edu>
// $HeadURL:
// $Id:
//

#include "Util/LEDcontroller.H"

LEDcontroller::LEDcontroller(OptionManager& mgr,
			     const std::string& descrName,
			     const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsSerial(new Serial(mgr))
{
  addSubComponent(itsSerial);
  itsSerial->configureSearch("ledboard",115200);
}

LEDcontroller::~LEDcontroller()
{ }

void LEDcontroller::setLED(int ledNumber, bool onoff, unsigned char flickerRate)
{
  if (ledNumber > 9 || ledNumber < 1)
    LFATAL("LED number out of range : %d", ledNumber);
  unsigned char cmd = 100 + ledNumber + (onoff?10:0);
  //LINFO("Command : %i",cmd);
  itsSerial->write(cmd);
  unsigned char cmdar[2] = {200,flickerRate};
  itsSerial->write(cmdar,2);
}

void LEDcontroller::start3()
{
  if (itsSerial->isSerialOk())
    LINFO("%s found on %s",
	  itsSerial->getDeviceDescriptor().c_str(),
	  itsSerial->getDevName().c_str());
}

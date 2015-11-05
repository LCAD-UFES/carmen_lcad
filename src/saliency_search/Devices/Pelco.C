/*!@file Devices/Pelco.C Interface to ptz cameras via the Pelco protocol */


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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/Pelco.C $
// $Id: Pelco.C 12962 2010-03-06 02:13:53Z irock $
//

#include "Devices/Pelco.H"

#include "Component/OptionManager.H"
#include "Devices/Serial.H"

///*** Protol P for now ***//

// ######################################################################
Pelco::Pelco(OptionManager& mgr, const std::string& descrName,
         const std::string& tagName, const char *defdev, const int cameraAddr) :
  ModelComponent(mgr, descrName, tagName),
  itsPort(new Serial(mgr)),
  itsCameraAddr(cameraAddr)
{
  // set a default config for our serial port:
  itsPort->configure(defdev, 9600 , "8N1", false, false, 1);

  // attach our port as a subcomponent:
  addSubComponent(itsPort);

}

// ######################################################################
Pelco::~Pelco()
{
}

int Pelco::sendRawCmd(const unsigned char byte1, const unsigned char byte2, const unsigned char byte3, const unsigned char byte4)
{

  int size = 8;
  unsigned char cmd[size];

  cmd[0] = 0xA0; //STX start transmission
  cmd[1] = itsCameraAddr-1; //zero based address (i.e. addr 1 is 0)
  cmd[2] = byte1;
  cmd[3] = byte2;
  cmd[4] = byte3;
  cmd[5] = byte4;
  cmd[6] = 0xAF; //ETX (end transmission)
  cmd[7] = cmd[1]^cmd[2]^cmd[3]^cmd[4]^cmd[5]; //Check sum

  // write command buffer
  itsPort->write(cmd, size);

  if (true)
  {
    LINFO("Sending: ");
    for(int i=0; i<size; i++)
      printf("%x ", cmd[i]);
    printf("\n");
  }

  return 0;
}

void Pelco::start2()
{
}

// ######################################################################
bool Pelco::movePanTilt(const int pan, const int tilt,
    const bool relative,
    const int panSpeed, const int tiltSpeed)
{
  return true;
}

// ######################################################################
bool Pelco::resetPanTilt()
{
  return true;

}

// ######################################################################
bool Pelco::zoom(const int val, const bool relative)
{

  return false;

}

// ######################################################################
bool Pelco::setFocus(const int val, const bool relative)
{

  return false;
}


bool Pelco::stop()
{
  return false;

}



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/*!@file Devices/Visca.C Interface to ptz cameras via the VISCA protocol */


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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/Visca.C $
// $Id: Visca.C 12962 2010-03-06 02:13:53Z irock $
//

#include "Devices/Visca.H"

#include "Component/OptionManager.H"
#include "Devices/Serial.H"

// ######################################################################
Visca::Visca(OptionManager& mgr, const std::string& descrName,
         const std::string& tagName, const char *defdev, const int cameraAddr) :
  ModelComponent(mgr, descrName, tagName),
  itsPort(new Serial(mgr)),
  itsCameraAddr(cameraAddr),
  itsCurrentZoom(500)
{
  // set a default config for our serial port:
  itsPort->configure(defdev, 9600 , "8N1", false, false, 1);

  // attach our port as a subcomponent:
  addSubComponent(itsPort);

}

// ######################################################################
Visca::~Visca()
{
}

int Visca::sendRawCmd(const unsigned char* cmd, const int length)
{
  // write command buffer
  itsPort->write(cmd, length);

  if (false)
  {
    LINFO("Sending: ");
    for(int i=0; i<length; i++)
      printf("%x ", cmd[i]);
    printf("\n");
  }

  bool found = false;
        int size=10;
  unsigned char cmdResults[size];
  int r = itsPort->read(cmdResults, size);
  while (r<size)
  {
    for(int i=0; i<r; i++)
    {
      if (cmdResults[i] == 0xff)
        found = true;
    }
    if (found) break;
          r += itsPort->read(&cmdResults[r], size-r);
          usleep(10000);
        }

  if (true)
  {
    LINFO("Got %i:", r);
    for(int i=0; i<r; i++)
      printf("%x ", cmdResults[i]);
    printf("\n");
  }

  return r;

}

void Visca::start2()
{
}

// ######################################################################
bool Visca::movePanTilt(const int pan, const int tilt,
    const bool relative,
    const int panSpeed, const int tiltSpeed)
{
  unsigned char command[15];
  command[0] = 0x80 + itsCameraAddr;
  command[1] = 0x01;
  command[2] = 0x06;

  LINFO("Pan %i tilt %i", pan, tilt);
  if (relative)
    command[3] = 0x03;
  else
    command[3] = 0x02;

  command[4] = panSpeed;
  command[5] = tiltSpeed;

  command[6] = (pan >> 4*3) & 0x0F;
  command[7] = (pan >> 4*2) & 0x0F;
  command[8] = (pan >> 4*1) & 0x0F;
  command[9] = (pan >> 4*0) & 0x0F;

  command[10] = (tilt >> 4*3) & 0x0F;
  command[11] = (tilt >> 4*2) & 0x0F;
  command[12] = (tilt >> 4*1) & 0x0F;
  command[13] = (tilt >> 4*0) & 0x0F;

  command[14] = 0xFF;

  return sendRawCmd(command, 15);
}

// ######################################################################
bool Visca::getPanTilt(short int &pan, short int &tilt)
{
  unsigned char command[15];
  command[0] = 0x80 + itsCameraAddr;
  command[1] = 0x09;
  command[2] = 0x06;
  command[3] = 0x12;
  command[4] = 0xFF;

  // write command buffer
  itsPort->flush();
  itsPort->write(command, 5);

        int size=255;
  bool found = false;
  unsigned char cmdResults[size];
  int r = itsPort->read(cmdResults, size);
  while (r<size)
  {
    for(int i=0; i<r; i++) //look for termination
      if (cmdResults[i] == 0xff)
        found = true;

    if (found) break;
          r += itsPort->read(&cmdResults[r], size-r);
          usleep(10000);
        }

  if (true)
  {
    LINFO("Got %i:", r);
    for(int i=0; i<r; i++)
      printf("%x ", cmdResults[i]);
    printf("\n");
  }

        if(cmdResults[r-1] != 0xff)
        {
                                        LINFO("Bad pan tilt");
                                        return false;
        } else {
                                        pan = (cmdResults[2] << 4*3);
                                        pan += (cmdResults[3] << 4*2);
                                        pan += (cmdResults[4] << 4*1);
                                        pan += (cmdResults[5] << 4*0);

                                        tilt = (cmdResults[6] << 4*3);
                                        tilt += (cmdResults[7] << 4*2);
                                        tilt += (cmdResults[8] << 4*1);
                                        tilt += (cmdResults[9] << 4*0);

        }



  return true;
}


// ######################################################################
bool Visca::resetPanTilt()
{
  unsigned char command[5];
  command[0] = 0x80 + itsCameraAddr;
  command[1] = 0x01;
  command[2] = 0x06;
  command[3] = 0x05;
  command[4] = 0xFF;

  return sendRawCmd(command, 5);

}

// ######################################################################
bool Visca::zoom(const int val, const bool relative)
{

  int zoomVal = itsCurrentZoom;

  if (relative)
    zoomVal += val;
  else
    zoomVal = val;

  if (zoomVal > 1023) return false;
  LINFO("Zoom to %i", zoomVal);

  unsigned char command[9];
  command[0] = 0x80 + itsCameraAddr;
  command[1] = 0x01;
  command[2] = 0x04;
  command[3] = 0x47;

  command[4] = (zoomVal >> 4*3) & 0x0F;
  command[5] = (zoomVal >> 4*2) & 0x0F;
  command[6] = (zoomVal >> 4*1) & 0x0F;
  command[7] = (zoomVal >> 4*0) & 0x0F;

  command[8] = 0xFF;

  int ret =  sendRawCmd(command, 9);

  if (!relative)
    itsCurrentZoom = zoomVal;

  if (ret == 6 || ret == 7)
  {
    itsCurrentZoom = zoomVal;
    return true;
  }

  return false;

}

// ######################################################################
bool Visca::setFocus(const int val, const bool relative)
{

  LINFO("focus to %i", val);
  unsigned char command[9];
  command[0] = 0x80 + itsCameraAddr;
  command[1] = 0x01;
  command[2] = 0x04;
  command[3] = 0x48;

  command[4] = (val >> 4*3) & 0x0F;
  command[5] = (val >> 4*2) & 0x0F;
  command[6] = (val >> 4*1) & 0x0F;
  command[7] = (val >> 4*0) & 0x0F;

  command[8] = 0xFF;

  if( sendRawCmd(command, 9))
  {
    return true;
  }

  return false;
}

bool Visca::setAutoFocus(const bool val)
{

  LINFO("autoFocus to %i", val);
  unsigned char command[9];
  command[0] = 0x80 + itsCameraAddr;
  command[1] = 0x01;
  command[2] = 0x04;
  command[3] = 0x38;

  if (val)
    command[4] = 0x02;
  else
    command[4] = 0x03;

  command[5] = 0xFF;

  if( sendRawCmd(command, 6))
  {
    return true;
  }

  return false;
}


// ######################################################################
bool Visca::setTargetTracking()
{
  unsigned char command[6];
  command[0] = 0x80 + itsCameraAddr;
  command[1] = 0x01;
  command[2] = 0x06;
  command[3] = 0x06;
  command[4] = 0x02;
  command[5] = 0xFF;

  sendRawCmd(command, 6);

  command[0] = 0x80 + itsCameraAddr;
  command[1] = 0x01;
  command[2] = 0x07;
  command[3] = 0x07;
  command[4] = 0x03;
  command[5] = 0xFF;

  return sendRawCmd(command, 6);

}



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/*!@file Robots/Rovio/Rovio.C Interface to Rovio robot */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Rovio/Rovio.C $
// $Id: Rovio.C 12962 2010-03-06 02:13:53Z irock $
//

#include "Robots/Rovio/Rovio.H"
#include "Component/OptionManager.H"

// ######################################################################
Rovio::Rovio(OptionManager& mgr, const std::string& descrName,
                   const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsHttpClient(new HTTPClient(mgr))

{
  addSubComponent(itsHttpClient);
}

// ######################################################################
Rovio::~Rovio()
{

}

// ######################################################################
void Rovio::start2()
{

}
// ######################################################################
void Rovio::stop2()
{
}

// ######################################################################
bool Rovio::stop()
{
  return sendDriveRequest(STOP, 0);
}

// ######################################################################
bool Rovio::moveForward(float speed)
{
  return sendDriveRequest(FORWARD, speed);
}

// ######################################################################
bool Rovio::moveBackward(float speed)
{
  return sendDriveRequest(BACKWARD, speed);
}

// ######################################################################
bool Rovio::straightLeft(float speed)
{
  return sendDriveRequest(STRAIGHT_LEFT, speed);
}

// ######################################################################
bool Rovio::straightRight(float speed)
{
  return sendDriveRequest(STRAIGHT_RIGHT, speed);
}

// ######################################################################
bool Rovio::rotateLeft(float speed)
{
  return sendDriveRequest(ROTATE_LEFT, speed);
}

// ######################################################################
bool Rovio::rotateRight(float speed)
{
  return sendDriveRequest(ROTATE_RIGHT, speed);
}


// ######################################################################
bool Rovio::sendDriveRequest(DRIVE_PARAMS dValue, float speed)
{

  char params[255];
  sprintf(params, "%i&speed=%i",  dValue, 10 - (int)(speed*10.0F));

  std::string request = "/rev.cgi?Cmd=nav&action=18&drive=" ;
  request += params;
  std::string ret = itsHttpClient->sendGetRequest(request);

  if (ret.size() > 0 )
    return true;
  else
    return false;
}


// ######################################################################
bool Rovio::getStatus()
{
  std::string request = "/rev.cgi?Cmd=nav&action=1" ;
  std::string ret = itsHttpClient->sendGetRequest(request);

  LINFO("Status %s", ret.c_str());

  return true;
}

// ######################################################################
bool Rovio::setCameraPos(int pos)
{

  std::string request;

  switch (pos)
  {
    case 0:
      request = "/rev.cgi?Cmd=nav&action=18&drive=12&speed=1" ;
      break;
    case 1:
      request = "/rev.cgi?Cmd=nav&action=18&drive=13&speed=1" ;
      break;
    case 2:
      request = "/rev.cgi?Cmd=nav&action=18&drive=11&speed=1" ;
      break;
  }

  std::string ret = itsHttpClient->sendGetRequest(request);

  if (ret.size() > 0 )
    return true;
  else
    return false;
}

bool Rovio::playSound()
{
  //Read the daisy file and send to robot
  //
  std::ifstream file ("etc/daisy.wav", std::ios::in|std::ios::binary|std::ios::ate);
  if (file.is_open())
  {
    file.seekg(0, std::ios::end);
    long size = file.tellg();
    file.seekg(0, std::ios::beg);

    LINFO("File open reading %lu into mem", size);

    char *buffer = new char[size];

    file.read(buffer, size);

    itsHttpClient->sendPostRequest("/GetAudio.cgi", buffer, size);

    file.close();
  }


  return true;


}








// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

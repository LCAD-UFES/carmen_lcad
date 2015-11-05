/*!@file Psycho/EyeTrackerUDP.C Eye tracker in Doug Munoz' lab using
   UDP packets for communication */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
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
// Primary maintainer for this file: David J. Bert <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/EyeTrackerUDP.C $

#ifndef PSYCHO_EYETRACKERUDP_C_DEFINED
#define PSYCHO_EYETRACKERUDP_C_DEFINED

#include "Psycho/EyeTrackerUDP.H"

#include "Component/ModelOptionDef.H"
#include "Psycho/PsychoOpts.H"
#include "Component/OptionManager.H"
#include "Util/Timer.H"
#include "Util/sformat.H"

#include <sys/time.h>

// ######################################################################
EyeTrackerUDP::EyeTrackerUDP(OptionManager& mgr,
                             const std::string& descrName,
                             const std::string& tagName) :
  EyeTracker(mgr, descrName, tagName),           //base init
  startMsg(NULL), stopMsg(NULL),                 //msg init
  isConnected(false), isFix(false), isSac(false),//state init
  sockfd(0), myInfo(NULL), hostInfo(NULL)        //socket init
{
  startMsg = new char[2];
  startMsg[0] = (char)1;
  startMsg[1] = '\0';
  
  stopMsg = new char[2];
  stopMsg[0] = (char)2;
  stopMsg[1] = '\0';
}

// ######################################################################
EyeTrackerUDP::~EyeTrackerUDP()
{
  delete startMsg;
  delete stopMsg;
}

// ######################################################################
void EyeTrackerUDP::start1()
{
  // start our udp listner
  FD_ZERO(&master); // clear the master and temp sets
  FD_ZERO(&read_fs); // clear the master and temp sets

  //setup the type of socket to request
  struct addrinfo hints;
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_UNSPEC;//ipv4 or ipv6
  hints.ai_socktype = SOCK_DGRAM;//UDP
  hints.ai_flags = AI_PASSIVE; // use my IP

  //get some info about our own adress
  int rv;
  if ((rv = getaddrinfo(NULL, "9999",
                        &hints, &myInfo)) < 0)
    {
      std::string ts = sformat("getaddrinfo: %s", gai_strerror(rv));
      LINFO("%s", ts.c_str());
      freeaddrinfo(myInfo);
      LFATAL("Fatal Error");
    }

  // loop through all the results and bind to the first we can
  for(; myInfo != NULL; myInfo = myInfo->ai_next)
    {
      if ((sockfd = socket(myInfo->ai_family, myInfo->ai_socktype,
                           myInfo->ai_protocol)) == -1)
        {
          LINFO("Error::Socket");
          continue;
        }

      //bind our socket
      int bindr;
      if ((bindr = bind(sockfd, myInfo->ai_addr, myInfo->ai_addrlen)) < 0)
        {
          close(sockfd);
          LINFO("Error::Listener bind");
          continue;
        }
      else break;
    }

  if (myInfo == NULL)
    {
      LINFO("Error: Failed to bind socket");
      freeaddrinfo(myInfo);
      LFATAL("Fatal Error");
    }

  if (hostInfo == NULL)
    LINFO("No host. Getting host from packets");

  // add the socket to the master set
  FD_SET(sockfd, &master);
  isConnected = true;
  LINFO("Socket creation succesful");

  //call our base start
  EyeTracker::start1();
}

// ######################################################################
void EyeTrackerUDP::start2()
{
  Timer itsTimer; //our timout period
  
  //wait for a code from host before we move on
  LINFO("Waiting for a UDP packet to complete start "
        "(timeout after 60 seconds)");
  
  while(true)
    {
      if (checkForData()) 
        {
          LINFO("Recieved start signal from host");
          break;
        }
      
      if (itsTimer.getSecs() > 60.0) 
        {
          LINFO("Timed out after 60 seconds. Finishing start without start "
                "signal from host");
          break;
        }
    }
  clearEyeStatus();
}

 // ######################################################################
void EyeTrackerUDP::stop1()
{
  usleep(50000);
  FD_CLR(sockfd, &master); // remove from master set
  close(sockfd);
  //free our server info
  if (hostInfo!=myInfo)
    freeaddrinfo(hostInfo);
  freeaddrinfo(myInfo);
  hostInfo = NULL;
  myInfo = NULL;
}

// ######################################################################
void EyeTrackerUDP::startTracking()
{
  sendMessage(startMsg);
}

// ######################################################################
void EyeTrackerUDP::stopTracking()
{
  sendMessage(stopMsg);
}

// ######################################################################
bool EyeTrackerUDP::isFixating()
{
//if true, a call by another state checking function set the isFix
//state and we don't need to check again. Just flip the state and
//return true. If false, check to see if any data is available and
//then check our state again.
  if (isFix)
    {
      isFix = false;
      return true;
    }
  else
    {
      checkForData();
      if (isFix)
        {
          isFix = false;
          return true;
        } 
    }
  return false;
}

// ######################################################################
bool EyeTrackerUDP::isSaccade()
{
//if true, a call by another state checking function set the isSac
//state and we don't need to check again. Just flip the state and
//return true. If false, check to see if any data is available and
//then check our state again.
  if (isSac)
    {
      isSac = false;
      return true;
    }
  else
    {
      checkForData();
      if (isSac)
        {
          isSac = false;
          return true;
        } 
    }
  return false;
}

// ######################################################################
void EyeTrackerUDP::clearEyeStatus()
{
  while (checkForData()) ; //gobble up any data that has been sent
  resetEyeFlags(); //set all of our eye status flags to false
}

// ######################################################################
bool EyeTrackerUDP::checkForData()
{
  int numbytes = 0;
  if (isConnected)
    {
      struct timeval tm;
      tm.tv_sec = 0;
      tm.tv_usec = 0;

      read_fs = master; // copy master
      select(sockfd+1, &read_fs, NULL, NULL, &tm);

      if (FD_ISSET(sockfd, &read_fs))
        {
          //ready to read
          lasthost_addrlen = sizeof lasthost_addr;
          unsigned char msgBuffer[BUFSIZE];

          if ( (numbytes = recvfrom(sockfd, msgBuffer, BUFSIZE-1 , 0,
                                    &lasthost_addr,
                                    &lasthost_addrlen)) <= 0)
            {
              // got error or connection closed by client
              if (numbytes == 0)
                {
                  // connection closed
                  pushEvent("Error::socket closed");
                  LFATAL("The connection closed");
                }
              else
                pushEvent("Error::recieve");
            }
          else
            {
              pushEvent("Received Trigger");
              setEyeFlags(msgBuffer[0]);
            }
        }//end FS_SET == true
    }
  else
    {
      pushEvent("Must be conncted to listen");
      LFATAL("Must be connected to listen");
    }

  return (bool)numbytes;
}

// ######################################################################
Point2D<int> EyeTrackerUDP::getEyePos() const
{
  LFATAL("Unavailable on UDP tracker, sorry.");
  return Point2D<int>(0, 0);
}

// ######################################################################
Point2D<int> EyeTrackerUDP::getFixationPos() const
{
  LFATAL("Unavailable on TIL tracker, sorry.");
  return Point2D<int>(0, 0);
}

//#######################################################################
//!Get the calibration set
CalibrationTransform::Data EyeTrackerUDP::getCalibrationSet(nub::soft_ref<PsychoDisplay> d) const
{
    CalibrationTransform::Data dummy;
    dummy.addData(Point2D<double>(-1.0,-1.0),Point2D<double>(-1.0,-1.0));
    return dummy;
}


// ######################################################################
void EyeTrackerUDP::sendMessage(const char* msg)
{
  //send some packets
    if (isConnected)
    {
      //return a status code to sender
      int check = 0;
      (hostInfo != NULL)?
        check = sendto(sockfd, msg, strlen(msg), 0,
                       hostInfo->ai_addr, hostInfo->ai_addrlen)
        :check = sendto(sockfd, msg, strlen(msg), 0,
                        &lasthost_addr, lasthost_addrlen);

      (check < 0)?
        pushEvent("Error::Sending"):
        pushEvent( std::string("Listener::Sent '" +
                               std::string(msg) + "'"));
    }
  else
    {
      pushEvent("Must be Connected to send a message");
      LFATAL("Must be conncted to send a message");
    }
}

// ######################################################################
void EyeTrackerUDP::pushEvent(const std::string& ev)
{
  if (itsEventLog.isValid())
    itsEventLog->pushEvent(ev);
}

// ######################################################################
void EyeTrackerUDP::setEyeFlags(const unsigned char msg)
{
  switch (msg)
    {
    case FIXCODE :
      isFix = true;
      break;
    case SACCODE :
      isSac = true;
      break;
    }
}

// ######################################################################
void EyeTrackerUDP::resetEyeFlags()
{
  isFix = false;
  isSac = false;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_EYETRACKERUDP_C_DEFINED

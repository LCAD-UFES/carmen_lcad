/*!@file Psycho/StimListener.C A thread to grab commands from a
   StimListener through a shared buffer and render them using SDL. */

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
// Primary maintainer for this file: David Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/StimListener.C $
#include "Psycho/StimListener.H"
#include "Util/sformat.H"

#define BUFSIZE 50
#define PORT "9999"

// ######################################################################
//implementation of StimListener
// ######################################################################
StimListener::StimListener() :
  itsEventLog(), itsBuffer() { };

// ######################################################################
StimListener::~StimListener() { };

// ######################################################################
void StimListener::setBuffer(const rutz::shared_ptr<SharedBuffer<StimData> >& buffer)
{
  itsBuffer = buffer;
}

// ######################################################################
void StimListener::setEventLog(const nub::soft_ref<EventLog>& elog)
{
  itsEventLog = elog;
}

// ######################################################################
void StimListener::pushEvent(const std::string& msg, const bool& useLinfo)
{
  if (useLinfo)
    LINFO(msg.c_str());

  if (itsEventLog.isValid())
    itsEventLog->pushEvent(msg);
}

// ######################################################################
void StimListener::addData(const StimData&, const data)
{
  itsBuffer.push(data);
  pushEvent(sformat("Recieved::%d bytes",d->buflen));
}

// ######################################################################
void StimListener::start()
{
  if ( !itsBuffer.is_valid() )
    LFATAL("A call to setBuffer(rutz::shared_ptr<SharedBuffer<StimData> >) is"
           " needed to start");
}

// ######################################################################
void StimListener::stop()
{

}

// ######################################################################
void StimListener::run()
{
  listen();
}

// ######################################################################
const char* StimListener::jobType() const
{
  return "Stimulus Listener";
}

// ######################################################################
int StimListener::priority() const
{
  return 0;
}

// ######################################################################
//implementation of StimListenerDML
// ######################################################################
StimListenerDML::StimListenerDML(unsigned char exitcode, const unsigned char[] msg) :
  StimListener(), sockfd(0), myInfo(NULL),
  hostInfo(NULL), keepGoing(true),itsExitCode(exitcode), itsConnected(false)
{
  //setup our default messages
  itsMsg = new unsigned char[sizeof(msg)];
  strcpy(itsMsg, msg);
};

// ######################################################################
StimListenerDML::~StimListenerDML()
{
  delete itsMsg;
  delete myInfo;
  delete hostInfo;
};

// ######################################################################
void StimListenerDML::start()
{

  StimListener::start(); //call our parents start2

  FD_ZERO(&master); // clear the master and temp sets

  //setup the type of socket to request
  struct addrinfo hints;
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_UNSPEC;//ipv4 or ipv6
  hints.ai_socktype = SOCK_DGRAM;//UDP
  hints.ai_flags = AI_PASSIVE; // use my IP

  //get some info about our own adress
  int rv;
  if ((rv = getaddrinfo(NULL, PORT, &hints, &myInfo)) < 0)
    {
      pushEvent(sformat("getaddrinfo: %s", gai_strerror(rv)),true);
      freeaddrinfo(myInfo);
      LFATAL("Fatal Error");
    }

  // loop through all the results and bind to the first we can
  for(; myInfo != NULL; myInfo = myInfo->ai_next)
    {
      if ((sockfd = socket(myInfo->ai_family, myInfo->ai_socktype,
                           myInfo->ai_protocol)) == -1)
        {
          pushEvent("Error::Socket",true);
          continue;
        }

      //bind our socket
      int bindr;
      if ((bindr = bind(sockfd, myInfo->ai_addr, myInfo->ai_addrlen)) < 0)
        {
          close(sockfd);
          pushEvent("Error::Listener bind",true);
          continue;
        }
      else break;
    }

  if (myInfo == NULL)
    {
      pushEvent("Error: Failed to bind socket",true);
      freeaddrinfo(myInfo);
      LFATAL("Fatal Error");
    }
  if (hostInfo == NULL)
      pushEvent("No host. Getting host from packets");

  // add the socket to the master set
  FD_SET(sockfd, &master);
  itsConnected = true;
  pushEvent("Socket creation succesful",true);
  itsConnected = true;
}

// ######################################################################
void StimListenerDML::stop()
{
  StimListener::stop(); // call our parent stop
  keepGoing = false;
  usleep(50000);
  FD_CLR(sockfd, &master); // remove from master set
  close(sockfd);
  //free our server info
  if (hostInfo!=myInfo)
    freeaddrinfo(hostInfo);
  freeaddrinfo(myInfo);
  hostInfo = NULL;
  myInfo = NULL;
  itsConnected = false;
}

// ######################################################################
bool StimListenerDML::sendMessage(const unsigned char* msg)
{
    if (itsConnected)
    {
      //return a status code to sender
      int check = 0;
      (hostInfo != NULL)?
        check = sendto(sockfd, msg, strlen(msg), 0,
                       hostInfo->ai_addr, hostInfo->ai_addrlen)
        :check = sendto(sockfd, msg, strlen(msg), 0,
                        &lasthost_addr, lasthost_addrlen);

      if (check < 0)
        {
          pushEvent("Error::Sending",true);
          return false;
        }
      else
        {
          pushEvent( std::string("Listener::Sent '" + std::string(msg) + "'"),
                     true);
          return true;
        }
    }
  else
    {
      pushEvent("Must be Connected to send a message",true);
      return false;
    }
}

// ######################################################################
bool StimListenerDML::listen()
{
  if (itsConnected)
    {
      int numbytes = 0, retval=0;
      unsigned char status_code = '\0';
      fd_set read_fds;  // temp file descriptor list for select()
      FD_ZERO(&read_fds);

      //run until we get an exit code
      pushEvent("Listener::Waiting",true);
      while( (status_code != itsExitCode) && keepGoing)
        {
          do
            {//repeat select until we catch a signal
              read_fds = master; // copy it
              retval = select(sockfd+1, &read_fds, NULL, NULL, NULL);
            }
          while( (retval ==-1) && (errno==EINTR) && (keepGoing) );

          if (retval < 0)
            {
              pushEvent("Error::select",true);
              status_code = itsExitCode;
              continue;
            }

          if (FD_ISSET(sockfd, &read_fds))
            {//ready to read
              lasthost_addrlen = sizeof lasthost_addr;
              unsigned char recvbuffer[BUFSIZE];
              if ( (numbytes = recvfrom(sockfd, recvbuffer, BUFSIZE-1 , 0,
                                        &lasthost_addr,
                                        &lasthost_addrlen)) <= 0)
                {
                  // got error or connection closed by client
                  if (numbytes == 0)
                    {
                      // connection closed
                      pushEvent("Error::socket closed",true);
                    }
                  else
                    {
                      pushEvent("Error::recieve",true);
                    }
                  status_code = itsExitCode;
                }
              else
                {
                  addData(StimData(recvbuffer,(uint)numbytes));
                  sendOK();
                }
            }
        }
      if ((retval < 0) || (numbytes==0))
        return false;
      else
        return true;
    }
  else
    {
      pushEvent("Must be Conncted to listen",true);
      return false;
    }
}

// ######################################################################
void StimListenerDML::setHostName(const std::string hostname)
{

  if (!itsConnected)
    {
      struct addrinfo hints;
      memset(&hints, 0, sizeof hints);
      hints.ai_family = AF_UNSPEC;//ipv4 or ipv6
      hints.ai_socktype = SOCK_DGRAM;//UDP
      int rv;
      if ( (rv = getaddrinfo(hostname.c_str(),PORT,&hints,&hostInfo)) == -1)
        {
          pushEvent(sformat("error host connection setup: %s",
                            gai_strerror(rv)),true);
          freeaddrinfo(hostInfo);
          hostInfo = NULL;
        }
      pushEvent("Got info for " + hostname,true);
    }
  else
    {
      pushEvent("Cannot setHostName() while connected, please call stop(),"
                "setHostName(), and start()",true);
      LFATAL("Fatal Error");
    }
}

// ######################################################################
const char* StimListenerDML::jobType() const
{
  return "Stimulus Listener DML";
}


// ######################################################################
void StimListener::sendOK()
{
  sendMessage(itsMsg);
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

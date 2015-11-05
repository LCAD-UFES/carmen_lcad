/*!@file Psycho/StimListener.H an abstraction that recieves stimulus
   commands from a host and pushes them into a buffer that can be
   shared by other compononents. A specific implementation using UDP
   packets for the Rex recording system is included. Rex used in the
   Doug Munoz laboratory in Kinston, ON. This system is a standard
   electrophysiology tool.  */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/StimListener.H $

#ifndef PSYCHO_STIMLISTENER_H_FEFINED
#define PSYCHO_STIMLISTENER_H_FEFINED

#include <sys/socket.h>
#include <netdb.h>

#include "Component/EventLog.H"
#include "Util/JobServer.H"
#include "Util/SharedBuffer.H"
#include "rutz/shared_ptr.h"
#include "Psycho/StimData.H"

//!Interface for a stimulus listener
/*!A StimListener can listen for stimulus commands from connected
   hosts and places them in a shared buffer. This buffer can then be
   read by other components to actually render the requested stimuli
   on the screen. This class is a JobServer::Job, which should be used
   with a JobServer derivitive. When queued this job will listen for
   commands in its own thread.*/
//######################################################################
class StimListener : public JobServer::Job
{
public:
  //!constructor
  StimListener();

  //!destructor
  virtual ~StimListener();

  //!set the buffer from a shared pointer
  void setBuffer(const rutz::shared_ptr<SharedBuffer<StimData> >& buffer);

  //! set the log file from a shared pointer
  void setEventLog(const nub::soft_ref<EventLog>& log);

  //! any pre listening initialization should go here
  virtual void start();

  //! clean things up after listening is over
  virtual void stop();

  /*! send a message, usually back to the sender. Derived classes need
    to implement this.*/
  virtual bool sendMessage(const unsigned char* msg) = 0;

  //! listen for stimulus codes, Default is Empty.
  virtual bool listen() = 0;

//!From Jobserver::Job
//######################################################################

  //!the actual work, called by JobServer after being queued
  virtual void run();

  //!return the job name
  virtual const char* jobType() const;

  //!return the priority, 0 for highest
  virtual int priority() const;

protected:
  //! push an event to the log, if uselinfo is true also display to the screen
  void pushEvent(const std::string& msg, const bool& uselinfo = false);

  //!push data onto the shared buffer
  void addData(const StimData& const data);

private:
  //prohibit copy and assignment
  StimListener& operator=(const StimListener& rhs);
  StimListener(const StimListener& rhs);

  // log stuff if desired:
  nub::soft_ref<EventLog> itsEventLog;

  //shared buffer to store incoming commands
  rutz::shared_ptr<SharedBuffer<StimData> > itsBuffer;
};

// ######################################################################
//!a class to control communication with the Rex experimental control system
// ######################################################################
/*! A derived class of StimulusListener to support comunication with
  the Rex experimental control system, which is a standard tool in
  electrophysiology labs. This system is designed to recieve UPD
  packets from Rex that indicate the stimulus to be drawn. The
  packets contain 1 byte for the stimulus primitive type (e.g. circle),
  and several bytes for the stimulus parameters (e.g. size, color). */

class StimListenerDML: public StimListener
{
public:
  //!constructor
  StimListenerDML(unsigned char exitcode = '0', const unsigned char[] = "1\0");

  //!destructor
  virtual ~StimListenerDML();

  //!send a message back to the host using a UDP packet
  bool sendMessage(const unsigned char* msg);

  //! listen for codes, which are UDP packets, usually 2-4 bytes long
  bool listen();

  //! set the host name
  /*! If set the listener will only accept packets from a specific
    host. Additionally, messages will be sent specifically to this
    host. If no host is set packets will be accepted by anyone, and
    status messages will be sent back to the last sender (Not
    gaurnteed on all systems). In most cases the host should be set
    to avoid problems with some systems not supplying the packet
    senders address and erroring on sendMessage() and sendOK() */
  void setHostName(const std::string hostname);

  //!get our component going and do pre-listening stuff
  void start();

  //!clean things up
  void stop();

  //!return the job name
  const char* jobType() const;


private:
  //!Send an confirmation message to the host
  void sendOK();

  int sockfd;   //our socket

  struct addrinfo *myInfo, *hostInfo; //adress info

  //hold info about the last socket
  struct sockaddr lasthost_addr;
  socklen_t lasthost_addrlen;

  fd_set master;    // master file descriptor list

  volatile bool keepGoing; // to sop all listening loops without a command,
  // maybe this needs to be atomic, but I think OK.

  //!defaults status code to stop listening
  unsigned char itsExitCode;

  //!are we ready to listen?
  bool  itsConnected;

  //for our status message - used during sendOK()
  unsigned char* itsMsg;
 };

#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */


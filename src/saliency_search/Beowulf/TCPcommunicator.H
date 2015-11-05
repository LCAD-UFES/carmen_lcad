/*!@file Beowulf/TCPcommunicator.H A class to handle multiple TCPmessage
  communications */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Beowulf/TCPcommunicator.H $
// $Id: TCPcommunicator.H 6986 2006-08-10 23:51:03Z rjpeters $
//

#ifndef TCPCOMMUNICATOR_H_DEFINED
#define TCPCOMMUNICATOR_H_DEFINED

#include "Beowulf/SockServ.H"
#include "Beowulf/TCPcliServ.H"
#include "Beowulf/TCPdefs.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "rutz/atomic.h"

#include <pthread.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <unistd.h>

//! delay in us of sleep while polling for running thread:
#define POLL_SLEEP 100

//! This class handles passing TCPmessages in and out
/*! This class handles passing TCPmessages in and out; CAUTION: it is a
  multi-threaded class.  Users can establish a connection with
  another host also running a TCPcommunicator by using
  contact(). Then, they can send out TCPmessages to that host using
  send().  Similarly, other hosts may contact us and send us
  messages, which we can then retrieve using receive(). */
class TCPcommunicator : public ModelComponent {
public:
  //! default constructor
  TCPcommunicator(OptionManager& mgr,
                  const std::string& descrName = "TCP Communicator",
                  const std::string& tagName = "TCPcommunicator");

  //! TCPcommunicator destructor
  virtual ~TCPcommunicator();

  //! contact a peer (return fd). hostname contains name:port
  int contact(const char *hostname, const bool blocking = true);

  //! contact a peer using its IP and port (returns fd)
  int contact(const in_addr_t ip, const short int port,
              const bool blocking = true);

  //! send a message to a known peer:
  void send(const int sfd, TCPmessage& smsg);

  //! receive a message
  /*! receive a message (returns false if no new message); use rfd =
    -1 to receive from any currently connected fd (fd received from
    will then be put in rfd). If timeout is zero, the call returns
    immediately, otherwise may wait up to timeout ms. If err is
    non-null, then (*err) will be set to non-zero if an error
    occurs. */
  bool receive(int& rfd, TCPmessage& rmsg, const int timeout = 0,
               int* err = 0);

  //! Do we have any received messages?
  /*! Returns the total number of messages in the incoming queues of
    our various TCPcliServ members. If rfd != -1, only consider a
    given client (fd) rather than any client. */
  int nbReceived(const int rfd = -1);

  //! Wait for availability of an incoming message
  /*!  use rfd = -1 to wait for message on any currently connected fd;
    timeout is in milliseconds. */
  void waitFor(const int rfd, const int timeout);

  //! terminate all open connections (except master server for new incoming)
  void terminateAll();

  //! terminate all open connections but one (and master server)
  void terminateAllButOne(const int fd);

  //! thread for data transfers
  /*! this will run in a different thread and take care of the actual
    data transfers, off-loading the main thread from them. Do not call
    this directly! */
  void run();

protected:
  OModelParam<in_addr> myIP;      //!< My IP address to listen on
  OModelParam<int> inqlen;        //!< incoming queue lengths
  OModelParam<int> ouqlen;        //!< outgoing queue lengths
  OModelParam<bool> indroplast;   //!< incoming queue drop policy
  OModelParam<bool> oudroplast;   //!< outgoing queue drop policy
  OModelParam<bool> disableShm;   //!< disbale shm transfers if true

  //! manages incoming connections and transfers
  nub::soft_ref<SockServ> server;

  //! get started and initialize ourselves (before SockServ start)
  void start1();
  //! get started and initialize ourselves (after SockServ start)
  void start2();
  //! get stopped (before SockServ stop)
  void stop1();

  //! poll var until it becomes false
  void pollvar(volatile bool *var);

  //! internal non-blocking receive check
  bool receiveIt(int& rfd, TCPmessage& rmsg, int* err);

  //! terminate a TCPcliServ
  void terminateCli(const int fd);

  //! array of monitored TCPcliServ's
  TCPcliServ** cli;

  //! all transfers are done via a separate thread
  pthread_t runner;

  //! these are for communication between main and running threads:
  mutable bool to_add, to_stop;
  int addfd;
  in_addr_t addip;
  short int addport;
  rutz::atomic_int_t threadRunning;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

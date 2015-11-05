/*!@file Beowulf/TCPcliServ.H A client/server to receive/send TCPmessage */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Beowulf/TCPcliServ.H $
// $Id: TCPcliServ.H 4663 2005-06-23 17:47:28Z rjpeters $
//

#ifndef TCPCLISERV_H_DEFINED
#define TCPCLISERV_H_DEFINED

#include "Beowulf/TCPmessage.H"
#include "Util/Timer.H"
#include <deque>
#include <map>
#include <netinet/in.h>
#include <pthread.h>
#include <unistd.h>

//! A client/server to receive/send TCPmessage
/*! This class defines a simple client/server that handles one TCP
  connection and can send/receive TCPmessages over it.  Messages to
  be sent or being received are queued; the actual transmission
  happens in check() which should be called often, typically under
  the control of a TCPcommunicator object that uses a SockServ object
  to know when our TCPcliServ needs attention.

  this class uses a mutex to allow check() to be called from one thread
  while send() and receive() may simultaneously be called from another
  thread.
*/
class TCPcliServ {
public:
  //! Base constructor. Must call init() later
  TCPcliServ();

  //! (re)-initialization from a connected socket
  /*! my_ipaddr and my_port should be my IP address and
    port. cli_ipaddr and cli_port should be the IP address and port of
    the client on the other end of connected_fd. For now, they are
    only used to determine whether we can optimize transfers by using
    shared memory in case the client's IP address matches ours. inqlen
    and outqlen are the maximum incoming and outgoing queue length, or
    zero for unlimited. indlast and outdlast specify the drop policies
    for ths input and output queues; if true, the last message (most
    recent) coming into the queue will be dropped if the queue is
    full, otherwise the oldest message already in the queue will be
    dropped. */
  void init(const int connected_fd, const in_addr_t my_ipaddr,
            const short int my_port, const in_addr_t cli_ipaddr,
            const short int cli_port, const int inqlen = 0,
            const int outqlen = 0, const bool indlast = false,
            const bool outdlast = false, const bool disableShm = false);

  //! Constructor and initialization from a connected socket
  TCPcliServ(const int connected_fd, const in_addr_t my_ipaddr,
             const short int my_port, const in_addr_t cli_ipaddr,
             const short int cli_port, const int inqlen = 0,
             const int outqlen = 0, const bool indlast = false,
             const bool outdlast = false, const bool disableShm = false);

  //! Destructor
  ~TCPcliServ();

  //! Send a message
  void send(const TCPmessage& msg);

  //! See if we have received a message (returns false if no new message)
  bool receive(TCPmessage& msg);

  //! Do we have any received messages in our queue
  /*! Returns the number of messages that have been fully received
    (not those in progress) and are waiting in the queue. Use
    receive() to get the messages out of the incoming queue. */
  int nbReceived();

  //! Check for new incoming message and do transfers
  /*! Check for new incoming message, continue reading/writing current
    messages, or start writing out a new message from the write queue
    (thread-safe). This must be called often for data to flow. */
  int check();

  //! Terminate the connection
  int disconnect();

private:
  void storeReceivedMessage(); // Store a received incoming message
  void queueIncomingMessage(TCPmessage& imsg);
  int reset();                 // reset everything

  int state;                // Internal state
  int fd;                   // Socket used to talk to peer
  std::deque<TCPmessage> inmsg;  // Incoming messages
  std::deque<TCPmessage> oumsg;  // Outgoing messages
  TCPmessage im;            // Currently incoming message
  TCPmessage om;            // Currently outgoing message
  uint inmsgqlen, outmsgqlen; // in/out message queue length or zero
  bool indroplast, outdroplast; // queue drop policies
  pthread_mutex_t mutin;    // Mutex for access to incoming message queue
  pthread_mutex_t mutou;    // Mutex for access to outgoing message queue

  in_addr_t myIP, cliIP;
  short int myPort, cliPort;
  bool useShm;
  std::map<int, void *> shmmap; //!< map associating shmid to address
  int shmcount;             //!< counter used to generate unique shm keys
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

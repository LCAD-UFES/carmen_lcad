/*!@file Beowulf/Beowulf.H Simple interfacing to a Beowulf cluster */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Beowulf/Beowulf.H $
// $Id: Beowulf.H 8160 2007-03-21 21:34:16Z rjpeters $
//

#ifndef BEOWULF_H_DEFINED
#define BEOWULF_H_DEFINED

#include "Beowulf/TCPcommunicator.H"
#include "Beowulf/TCPdefs.H"
#include "Beowulf/TCPmessage.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Util/Timer.H"

#include <deque>
#include <pthread.h>
#include <vector>

// ##### 32-bit action codes:
// the following are reserved for init of the beowulf communications:
#define BEO_NONE       0
#define BEO_INIT      -1
#define BEO_INIT2     -2
#define BEO_INIT3     -3

//! Simple interfacing to a Beowulf cluster
/*! The idea of this class is to hide all of the low-level
  communication setup and transfer details from the user, and to
  provide a simple interface for passing messages between nodes on a
  Beowulf cluster. Each slave node should instantiate a Beowulf object
  and initialize it with slaveInit(). This will block the slave until
  it is contacted by the Beowulf master node. The master node
  instantiates a Beowulf object, and initializes it using
  masterInit(), passing along the hostnames of the slave nodes. During
  initialization, the Beowulf master contacts all the slaves and
  instructs them to fully interconnect with each other.  Once
  initialization is complete, any node can send() and receive()
  TCPmessages to and from any other node. Both send() and receive()
  are non-blocking methods. Actual queueing and transfer of messages
  is done in a thread that runs in parallel with the main program
  thread. */

class Beowulf : public ModelComponent {
public:

  // ######################################################################
  /*! @name Constructors and destructors */
  //@{

  //! Constructor
  /*! @param isMaster true if we are the master of the Beowulf. The
    master is the one that gets the list of slaves and then
    initializes all the slaves at start() time. */
  Beowulf(OptionManager& mgr,
          const std::string& descrName = "Beowulf",
          const std::string& tagName = "Beowulf",
          const bool ismaster = false);

  //! Reset and kill all connections except possibly one (keepfd)
  /*! Resets the Beowulf to uninitialized state. Kills all connections,
     except possibly one (typically towards the master) that may be
     specified as argument.
     @param keepfd the fd to keep (or -1 to keep no fd) */
  void resetConnections(const int keepfd = -1);

  //! Destructor
  /*! Will properly terminate all connections. */
  virtual ~Beowulf();
  //@}

  // ######################################################################
  /*! @name Access functions */
  //@{

  //! get number of slave nodes
  int getNbSlaves() const;

  //! get our node number (-1 is the master)
  int getNodeNumber() const;

  //! Get hostname:port of node with given node number
  /*! This is whatever the user gave at configuration, so it could be
    just a short hostname, a fully-qualified hostname, or a
    hostname:port. If nb is -1, we return "BeoMaster" */
  const char* nodeName(const int nb) const;

  //! Request a so-far unallocated node
  /*! This will return the next node number that has not yet been
    requested. This only works if we are Beowulf master. It will
    generate an error message and return -2 if we have no more
    unallocated nodes. We need to be start()'ed for this to work. */
  int requestNode();

  //! De-allocate a currently allocated node
  void releaseNode(int nodenum);

  //@}

  // ######################################################################
  /*! @name Message passing functions */
  //@{

  //! Send message to another node
  /*! This method is non-blocking (returns immediately). A copy of msg
    is taken, so you can destroy it immediately after send.
    @param node_nb is the destination node number. A value of -1 on a
    slave Beowulf means that msg should be sent to the Beowulf master.  */
  void send(const int node_nb, TCPmessage& msg);

  //! Send message to the least-loaded of our slave nodes
  /*! This method is non-blocking (returns immediately). A copy of msg
    is taken, so you can destroy it immediately after send.  Only
    works if we are the Beowulf master, fatal error otherwise. This
    implements load balancing. The ETI (estimated time to idle) fields
    in TCPmessage are used to determine which of our slave nodes has
    the shortest pending work queue (i.e., shortest ETI) and the
    message will be sent to that node. Thus, this functionality
    assumes that every slave node can process every message that you
    might send them (as opposed to more constrained architectures
    where a given node is only capable of doing a given type of
    processing corresponding to a given type of received message).
    For this load balancing to work, the slaves should try to put
    good-faith estimates of their time to idle (in seconds) each time
    they send us (the master) a message back. The master relies on
    those good-faith estimates to decide which node is the least
    loaded. This approach has severe limitations if your overall
    message traffic is low, as your ETI estimates at the master will
    not be refreshed regularly and may become grossly
    inaccurate. Thus, this approach is mostly intended for streaming
    applications, where every node will usually send several messages
    back to the master every 30ms or so, so that the ETI estimates
    collected at the master will be reasonably fresh and accurate. If
    several slave nodes have the lowest ETI, one will be picked at
    random. */
  void send(TCPmessage& msg);

  //! Receive message from a given node (or from any node)
  /*! Check whether a message has been received; returns false
    otherwise.  If a message was received, the node it came from will
    be in node_nb, and its frame and action fields will be pre-decoded
    for convenience (they still are in the message itself too). This
    method is always non-blocking, i.e., it returns immediately and
    does not wait for messages to come in.
    @param node_nb node number to receive from, or -1 to receive from
    any node.  If a message is received, node_nb will be updated to
    the node number from which the message was received, or -1 if it
    was received from the master.
    @param msg received message
    @param frame frame number from received message
    @param action action field from received message
    @param timeout if non-zero, max time (in ms) this call may block
    @param err if non-null, then (*err) will be set to non-zero if an error occurs
  */
  bool receive(int& node_nb, TCPmessage& msg, int32& frame,
               int32& action, const int timeout = 0,
               int* err = 0);

  //! Do we have any received messages?
  /*! Returns the total number of messages in the incoming queues of
    our various connected nodes. If node_nb == -1, only consider
    messages from the Beowulf master. If node_nb == -2, consider any
    node, otherwise only consider the specified node. */
  int nbReceived(const int node_nb = -2);

  //@}

protected:
  //! names of our slaves as a space-separated list of hostname:port
  /*! port is optional and we will use the default SockServ port if
    unspecified. This parameter is only used if we are Beowulf master
    (see constructor) */
  OModelParam<std::string> itsSlaveNames;

  OModelParam<bool> isMaster;       //!< true if we are the master
  OModelParam<int> selfqlen;        //!< self-message queue length
  OModelParam<bool> selfdroplast;   //!< self-message queue drop policy
  OModelParam<double> initTimeout;  //!< max time to wait for initialization

  //! Intercept people changing our ModelParam
  /*! See ModelComponent.H; as parsing the command-line or reading a
    config file sets our name, we'll also here instantiate a
    controller of the proper type (and export its options) */
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

private:
  nub::soft_ref<TCPcommunicator> com; // Handles all communications

  struct NodeInfo
  {
    NodeInfo() : fd(-1), name(), ETI(-1.0f), ETIreceived(-1.0f),
                 isAvailable(true) {}

    int fd; // Translate node number into fd (socket)
    std::string name; // Hostname of the slave node
    float ETI; // ETIs (in seconds) as sent to us by our slaves
    float ETIreceived; // time at which an ETI was last received
    bool isAvailable; // true if this node is not currently in use
  };

  bool initialized;    // True if all communications ok
  std::vector<NodeInfo> itsNodes; // Table of per-node info
  int *fd2node;        // Table to translate fd into node number
  int master;          // fd of my master if I am a slave
  int me;              // My node number if I am a slave

  Timer tim;           // to record message arrival times
  std::deque<TCPmessage> selfmsg; // messages to myself
  pthread_mutex_t mutselfmsg;     // Mutex for access to self message queue

  // get started (after our TCPcommunicator has started)
  void start2();

  // get stopped (before our TCPcommunicator has stopped)
  void stop1();

  //! Initialize as master node, using array of slave node hostnames
  /*! Master node initialization will contact all nodes specified and
     initialize them. Once this is done, everything will be ready to
     send() and receive() TCPmessages.
     @param nb_nodes number of slave nodes
     @param node_names hostnames of the slave nodes (format name:port) */
  void masterInit(const int nb_nodes, char **node_names);

  //! Initialize as master node using string of slave node hostnames
  /*! Master node initialization will contact all nodes specified and
     initialize them. Once this is done, everything will be ready to
     send() and receive() TCPmessages.
     @param node_names space-separated or comma-separated names of slave
     nodes as name:port, or a single absolute path (starting with '/') of
     a text file that contains the node names, one name per line. */
  void masterInit(const char *node_names);

  //! Initialize as a slave node
  /*! This method will block until a Beowulf master contacts us and
    initializes us. Once this is done, everything will be ready to
    send() and receive() TCPmessages. */
  void slaveInit();

  // In case we receive a message of type BEO_INIT while processing
  // stuff, receive() will call this function with the message; this
  // will start reinitializing us completely. Then receive will also
  // call slaveInit() to finish the re-initialization:
  void slaveReInit(TCPmessage& rmsg);
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

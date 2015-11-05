/*!@file Beowulf/Beowulf.C Simple interfacing to a Beowulf cluster */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Beowulf/Beowulf.C $
// $Id: Beowulf.C 15310 2012-06-01 02:29:24Z itti $
//

#include "Beowulf/Beowulf.H"

#include "Beowulf/BeowulfOpts.H"
#include "Component/OptionManager.H"
#include "Util/Assert.H"
#include "Util/MathFunctions.H"
#include "Util/sformat.H"

#include <cmath>    // for floor()
#include <cstdio>   // for fseek()
#include <limits>

// ######################################################################
Beowulf::Beowulf(OptionManager& mgr, const std::string& descrName,
                 const std::string& tagName, const bool ismaster) :
  ModelComponent(mgr, descrName, tagName),
  itsSlaveNames(&OPT_BeowulfSlaveNames, this),
  isMaster(&OPT_BeowulfMaster, this, ismaster, USE_MY_VAL),
  selfqlen(&OPT_BeowulfSelfQlen, this),
  selfdroplast(&OPT_BeowulfSelfDropLast, this),
  initTimeout(&OPT_BeowulfInitTimeout, this),
  com(new TCPcommunicator(mgr)), initialized(false),
  itsNodes(), fd2node(NULL), master(0), me(0),
  tim(1000000),  // microsecond accuracy
  selfmsg()
{
  // initialize our self-addressed message queue mutex:
  pthread_mutex_init(&mutselfmsg, NULL);

  // make our TCPcommunicator a subcomponent of us:
  addSubComponent(com);

  if (isMaster.getVal() == false)
    this->unregisterParam(&itsSlaveNames);
}

// ######################################################################
Beowulf::~Beowulf()
{ pthread_mutex_destroy(&mutselfmsg); }

// ######################################################################
void Beowulf::paramChanged(ModelParamBase* const param,
                           const bool valueChanged,
                           ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);

  // was that a change of masterhood?
  if (param == &isMaster)
    {
      if (isMaster.getVal())
        { me = -1; this->registerOptionedParam(&itsSlaveNames, 0); }
      else
        { me = 0; this->unregisterParam(&itsSlaveNames); }
    }
}

// ######################################################################
void Beowulf::start2()
{
  if (isMaster.getVal()) masterInit(itsSlaveNames.getVal().c_str());
  else slaveInit();
}

// ######################################################################
void Beowulf::stop1()
{
  resetConnections(-1);
}

// ######################################################################
int Beowulf::getNbSlaves() const
{
  // we're converting from size_t->int; let's make sure that
  // conversion is going to be safe:
  ASSERT(itsNodes.size() <= size_t(std::numeric_limits<int>::max()));

  return int(itsNodes.size());
}

// ######################################################################
int Beowulf::getNodeNumber() const
{ return me; }

// ######################################################################
const char* Beowulf::nodeName(const int nb) const
{
  // is it the master?
  if (nb == -1) return "BeoMaster";
  if (nb < 0 || size_t(nb) >= itsNodes.size())
    LFATAL("Node number %d exceeds number of nodes: %" ZU,
           nb, itsNodes.size());
  return itsNodes[nb].name.c_str();
}

// ######################################################################
int Beowulf::requestNode()
{
  if (started() == false) LFATAL("I am not started");

  for (size_t i = 0; i < itsNodes.size(); ++i)
    {
      if (itsNodes[i].isAvailable)
        {
          itsNodes[i].isAvailable = false;
          return int(i);
        }
    }

  LERROR("I am out of nodes -- RETURNING -2");
  return -2;
}

// ######################################################################
void Beowulf::releaseNode(int nodenum)
{
  if (nodenum < 0 || size_t(nodenum) >= itsNodes.size())
    LERROR("Request to release an invalid node number (%d) -- IGNORING",
           nodenum);
  else if (itsNodes[nodenum].isAvailable)
    LERROR("Request to release an unallocated node (%d) -- IGNORING",
           nodenum);
  else
    itsNodes[nodenum].isAvailable = true;
}

// ######################################################################
void Beowulf::resetConnections(const int masterfd)
{
  itsNodes.resize(0);
  if (fd2node) { delete [] fd2node; fd2node = NULL; }
  if (masterfd == -1) com->terminateAll();
  else com->terminateAllButOne(masterfd);
  initialized = false; tim.reset(); selfmsg.clear();
}

// ######################################################################
void Beowulf::masterInit(const int nb_nodes, char** node_names)
{
  resetConnections(-1);  // reset possible existing communications
  master = -1;  // I have no master, so I am the master
  me = -1;      // yes, I am the master

  // setup table of all node nostnames:
  itsNodes.resize(nb_nodes);

  // setup initial communication with all nodes:
  fd2node = new int[FD_SETSIZE];
  for (int i = 0; i < FD_SETSIZE; i ++) fd2node[i] = -1;

  for (size_t i = 0; i < itsNodes.size(); ++i)
    {
      itsNodes[i].name = node_names[i];
      itsNodes[i].fd = com->contact(itsNodes[i].name.c_str());
      LDEBUG("Linked node %" ZU " to fd %d", i, itsNodes[i].fd);
      if (itsNodes[i].fd == -1)
        LFATAL("Failed contacting %s", itsNodes[i].name.c_str());
      fd2node[itsNodes[i].fd] = i;

      // setup initial ETIs. We assume that all nodes are idle:
      itsNodes[i].ETI = 0.0F;
      itsNodes[i].ETIreceived = 0.0F;
    }

  tim.reset();

  // now, send an initial message to each node that tells them what the
  // array of slaves is, so that they can allocate memory for it and
  // initialize it:
  for (size_t i = 0; i < itsNodes.size(); ++i)
    {
      // send a message with the names of all nodes and the index of the
      // node of interest:
      TCPmessage msg(0, BEO_INIT);
      std::string buf = sformat("%" ZU " %" ZU " ", itsNodes.size(), i);
      for (size_t j = 0; j < itsNodes.size(); ++j)
        { buf += itsNodes[j].name; buf += " "; }
      msg.addString(buf.c_str());
      com->send(itsNodes[i].fd, msg);
    }

  // wait for a message of type INIT from each node, acknowledging that they
  // have initialized their array of peers:
  size_t nodeok = 0;
  LDEBUG("Waiting for INIT from each node");
  Timer initTimer;
  while (nodeok < itsNodes.size() &&
         (initTimeout.getVal() <= 0.0 ||
          initTimer.getSecs() <= initTimeout.getVal()))
    {
      TCPmessage msg; int rfd = -1;  // receive from any node
      if (com->receive(rfd, msg, 5))  // block up to 5ms
        {
          if (msg.getAction() != BEO_INIT)
            LERROR("Bogus message [%d, %d] ignored!",
                   msg.getID(), msg.getAction());
          else
            nodeok ++;
        }
    }
  if (nodeok != itsNodes.size())
    LFATAL("Timeout while waiting for INIT from each node");

  // now, send an order to each node to contact its peers:
  TCPmessage msg(0, BEO_INIT2);
  for (size_t i = 0; i < itsNodes.size(); ++i)
    com->send(itsNodes[i].fd, msg);

  // wait for a message of type INIT3 from each node, acknowledging that they
  // have contacted their peers:
  nodeok = 0;
  LDEBUG("Waiting for INIT3 from each node");
  while (nodeok < itsNodes.size() &&
         (initTimeout.getVal() <= 0.0 ||
          initTimer.getSecs() <= initTimeout.getVal()))
    {
      TCPmessage msg; int rfd = -1;  // receive from any node
      if (com->receive(rfd, msg, 5))  // block up to 5ms
        {
          if (msg.getAction() != BEO_INIT3)
            LERROR("Bogus message [%d, %d] ignored!",
                   msg.getID(), msg.getAction());
          else
            {
              nodeok ++;
              LINFO("Slave node %d [%s] is configured.", msg.getID(),
                    itsNodes[msg.getID()].name.c_str());
            }
        }
    }

  if (nodeok != itsNodes.size())
    LFATAL("Timeout while waiting for INIT3 from each node");

  // ready to transmit/receive messages:
  initialized = true;
  LINFO("Initialization complete -- all slaves ready!");
}

// ######################################################################
void Beowulf::masterInit(const char* node_names)
{
  ASSERT(node_names);
  int nbn = 0;
  typedef char* charptr; char **nn;

  if (node_names[0] == '/') // file that contains the node list
    {
      // count how many nodes:
      LINFO("Using Beowulf slaves from file %s", node_names);
      char line[1024];
      FILE *fil = fopen(node_names, "r");
      if (fil == NULL) LFATAL("Cannot open slave node file %s", node_names);
      while(fgets(line, 1024, fil)) nbn ++;

      // create list of node names:
      fseek(fil, 0, SEEK_SET); // back to start of file
      int no = 0;
      nn = new charptr[nbn];
      while(fgets(line, 1024, fil))
        {
          line[strlen(line) - 1] = '\0';  // eliminate LF
          nn[no] = new char[strlen(line) + 1];
          strcpy(nn[no], line);
          no ++;
        }
      fclose(fil);
    }
  else
    {                       // node list as n01:9567,n02:9567, ...
      LINFO("Using Beowulf slaves %s", node_names);
      // count how many nodes:
      int idx = 0, slen = strlen(node_names);
      while(idx < slen) {
        if (node_names[idx] == ' ' || node_names[idx] == ',' ||
            idx == slen - 1) nbn ++;
        idx ++;
      }

      // create list of node names:
      nn = new charptr[nbn];
      idx = 0; int idx2 = 0, no = 0;
      while(idx <= slen) {
        if (idx == slen || node_names[idx] == ' ' || node_names[idx] == ',') {
          nn[no] = new char[idx - idx2 + 1];
          strncpy(nn[no], node_names + idx2, idx - idx2);
          nn[no][idx - idx2] = '\0';
          no ++; idx2 = idx + 1;
        }
        idx ++;
      }
    }

  // do the initialization:
  masterInit(nbn, nn);

  // delete local vars:
  for (int i = 0; i < nbn; i ++) delete [] nn[i];
  delete [] nn;
}

// ######################################################################
void Beowulf::slaveInit()
{
  size_t nodeok = 1;
  LINFO("waiting for master...");

  // wait for my master to contact me, then contact other nodes and tell
  // master when I am ready:
  while(initialized == false)
    {
      TCPmessage msg; int rfd = -1;  // receive from any open socket
      if (com->receive(rfd, msg, 5))  // block up to 5ms
        {
          int32 id = msg.getID();
          int32 action = msg.getAction();
          //LDEBUG("Received message [%d, %d] from fd=%d", id, action, rfd);
          switch(action)
            {
            case BEO_INIT: // ##############################
              {
                // this message comes from our master:
                master = rfd;

                // start the (re)-initialization:
                slaveReInit(msg);
              }
              break;
            case BEO_INIT2:
              {
                // ok, now all the slaves should know what the array
                // of slaves is.  Let's contact our peers: send a
                // message of type INIT3 to all nodes with number
                // above mine:
                TCPmessage msg2(me, BEO_INIT3); nodeok = 1;
                for (size_t i = me + 1; i < itsNodes.size(); ++i)
                  {
                    itsNodes[i].fd =
                      com->contact(itsNodes[i].name.c_str());
                    if (itsNodes[i].fd == -1)
                      LFATAL("Failed contacting %s",
                             itsNodes[i].name.c_str());
                    fd2node[itsNodes[i].fd] = i;
                    com->send(itsNodes[i].fd, msg2);
                    nodeok ++; // node[i] is initialized and ok
                  }
                // have we contacted everybody we needed to?
                if (nodeok == itsNodes.size()) initialized = true;
              }
              break;
            case BEO_INIT3: // ##############################
              {
                // we received a message that will allow us to fill up
                // blanks in our node[] array:
                ASSERT(id >= 0 && size_t(id) < itsNodes.size());
                itsNodes[id].fd = rfd; fd2node[rfd] = id;
                nodeok ++;

                // have we contacted everybody we needed to?
                if (nodeok == itsNodes.size()) initialized = true;
              }
              break;
            default: // ##############################
              LERROR("Bogus action %d -- IGNORING.", action);
              break;
            }
        }
    }

  // ok, we are ready; tell master:
  TCPmessage msg(me, BEO_INIT3);
  com->send(master, msg);
  LINFO("Initialization complete -- all connections ready!");

  ASSERT(me >= 0); // because we are a slave

  std::string xx;

  for (size_t i = 0; i < itsNodes.size(); ++i)
    if (i == size_t(me)) xx += "me ";
    else { xx += sformat("%02d ", itsNodes[i].fd); }
  LINFO("NODES = [ %s]", xx.c_str());
}

// ######################################################################
void Beowulf::slaveReInit(TCPmessage& rmsg)
{
  // kill all existing communications except towards master:
  resetConnections(master);

  // decode the message:
  char* buf2;
  char* buf3;
  const std::string buf = rmsg.getElementString();
  const long nbnode = strtol(buf.c_str(), &buf2, 10); buf2 ++;
  me = strtol(buf2, &buf3, 10); buf3 ++;
  itsNodes.resize(nbnode);
  fd2node = new int[FD_SETSIZE];
  for (int i = 0; i < FD_SETSIZE; i ++) fd2node[i] = -1;

  for (size_t i = 0; i < itsNodes.size(); ++i)
    {
      itsNodes[i].fd = -1;  // no known node here yet...
      buf2 = buf3; while(*buf3 != ' ' && *buf3 != '\0') buf3 ++;
      *buf3++ = '\0';
      itsNodes[i].name = buf2;
    }

  // send an acknowledgment to master:
  LINFO("INIT with %" ZU " nodes, me = %d [%s]",
        itsNodes.size(), me, itsNodes[me].name.c_str());
  TCPmessage msg(me, BEO_INIT);
  com->send(master, msg);
}

// ######################################################################
void Beowulf::send(const int node_nb, TCPmessage& msg)
{
  ASSERT(initialized);
  ASSERT(node_nb == -1 ||
         (node_nb >= 0 && size_t(node_nb) < itsNodes.size()));
  if (node_nb == me)       // send to ourselves?
    {
      LDEBUG("Sending msg [%d, %d] to myself", msg.getID(), msg.getAction());
      if (selfqlen.getVal() > 0 && int(selfmsg.size()) >= selfqlen.getVal())
        {
          if (selfdroplast.getVal())
            LERROR("Self-message queue full -- DROPPING MOST RECENT MESSAGE");
          else
            {
              LERROR("Self-message queue full -- DROPPING LEAST RECENT "
                     "MESSAGE");
              pthread_mutex_lock(&mutselfmsg);
              selfmsg.pop_front();
              selfmsg.push_back(msg);
              pthread_mutex_unlock(&mutselfmsg);
            }
        }
      else
        {
          pthread_mutex_lock(&mutselfmsg);
          selfmsg.push_back(msg);
          pthread_mutex_unlock(&mutselfmsg);
        }
    }
  else if (node_nb == -1)  // want to send to my master
    {
      LDEBUG("Sending msg [%d, %d] to master", msg.getID(), msg.getAction());
      com->send(master, msg);
    }
  else
    {
      ASSERT(node_nb >= 0 && size_t(node_nb) < itsNodes.size());
      LDEBUG("Sending msg [%d, %d] to node %d [%s] [%d]", msg.getID(),
             msg.getAction(), node_nb,
             itsNodes[node_nb].name.c_str(), itsNodes[node_nb].fd);
      com->send(itsNodes[node_nb].fd, msg);
    }
}

// ######################################################################
void Beowulf::send(TCPmessage& msg)
{
  ASSERT(initialized);
  // this is the load-balanced send. Only works if we are master:
  ASSERT(me == -1);

  // find out least ETI delta; what counts here is ETIreceived + ETI:
  int minnode[itsNodes.size()]; // nodes that have the smallest ETI
  int nmin = 0;  // how many nodes have the smallest ETI?
  float mindiff = 1.0e30F;
  // std::string load;
  for (size_t i = 0; i < itsNodes.size(); ++i)
    {
      if (itsNodes[i].ETIreceived < 0.0F)
        { /* load += "- ";*/ continue; } // don't know about this node
      float diff = itsNodes[i].ETI + itsNodes[i].ETIreceived;
      if (diff < mindiff) { mindiff = diff; nmin = 1; minnode[0] = i; }
      else if (diff == mindiff) minnode[nmin++] = i;
      // load += sformat("%.1f ", diff * 1000.0F);
    }
  // LINFO("LOAD = [ %s]", load.c_str());

  int node_nb = -1;
  if (nmin == 0)  // don't know about load of any node -> random pick
    node_nb = int(floor(itsNodes.size() * randomDouble()));
  else            // pick one of our mindiff nodes and send to it
    node_nb = minnode[int(floor(nmin * randomDouble()))];

  ASSERT(node_nb >= 0 && size_t(node_nb) < itsNodes.size());
  LDEBUG("Sending msg [%d, %d] to least-loaded node %d [%s] [%d]",
         msg.getID(), msg.getAction(), node_nb,
         itsNodes[node_nb].name.c_str(), itsNodes[node_nb].fd);
  com->send(itsNodes[node_nb].fd, msg);
  itsNodes[node_nb].ETIreceived = -1.0F; // don't know about this node anymore
}

// ######################################################################
bool Beowulf::receive(int& node_nb, TCPmessage& msg, int32& frame,
                      int32& action, const int timeout, int* err)
{
  ASSERT(initialized);

  // our self-sent messages have priority, let's deal with them first:
  if (node_nb == me || node_nb == -1) // receive from myself or anybody
    {
      if (selfmsg.size() > 0)
        {
          pthread_mutex_lock(&mutselfmsg);
          msg = selfmsg.front(); selfmsg.pop_front();
          pthread_mutex_unlock(&mutselfmsg);
          frame = msg.getID(); action = msg.getAction(); node_nb = me;
          LDEBUG("Received msg [%d, %d] from myself", frame, action);
          return true;
        }
      else if (node_nb == me && me != -1)
        return false; // nothing received from myself and I am not master
    }

  ASSERT(node_nb == -1 ||
         (node_nb >= 0 && size_t(node_nb) < itsNodes.size()));

  int rfd;
  if (node_nb == -1) rfd = -1;  // receive from any node
  else rfd = itsNodes[node_nb].fd; // receive from given node
  bool rec = com->receive(rfd, msg, timeout, err);
  if (rec)
    {
      frame = msg.getID(); action = msg.getAction(); node_nb = fd2node[rfd];
      if (node_nb == -1)
        LDEBUG("Received msg [%d, %d] from master", frame, action);
      else
        LDEBUG("Received msg [%d, %d] from node %d [%s]", frame, action,
               node_nb, itsNodes[node_nb].name.c_str());

      // is this message an order for re-initialization?
      if (action == BEO_INIT && initialized)
        {
          LINFO("RE-INIT order received -- Starting re-initialization...");
          // whoever re-inits me is my new master:
          master = rfd;
          // start a re-init based on message we just received:
          slaveReInit(msg);
          // finish up re-init:
          slaveInit();
          // we still return the message to let our user know we got re-inited
        }

      // if we are the master, update that node's ETI:
      if (me == -1)
        {
          if (node_nb < 0 || size_t(node_nb) >= itsNodes.size())
            LERROR("Bogus node number %d - IGNORED", node_nb);
          else
            {
              itsNodes[node_nb].ETI = msg.getETI();
              itsNodes[node_nb].ETIreceived = tim.getSecs();
            }
        }
    }
  else
    { frame = -1; action = -1; }

  return rec;
}

// ######################################################################
int Beowulf::nbReceived(const int node_nb)
{
  if (node_nb == -2)  // any node
    return com->nbReceived(-1);
  if (node_nb == -1)  // only master
    {
      if (me == -1) LFATAL("Hey, I am the master!");
      return com->nbReceived(master);
    }
  if (node_nb < 0 || size_t(node_nb) >= itsNodes.size())
    LFATAL("Node number %d out of range [-2 .. %" ZU "]",
           node_nb, itsNodes.size());
  return com->nbReceived(itsNodes[node_nb].fd);
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

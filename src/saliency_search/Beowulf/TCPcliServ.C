/*!@file Beowulf/TCPcliServ.C A client/server to receive/send TCPmessage */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Beowulf/TCPcliServ.C $
// $Id: TCPcliServ.C 11538 2009-07-30 06:23:37Z itti $
//

#include "Beowulf/TCPcliServ.H"

#include "Beowulf/TCPdefs.H"
#include "Util/log.H"
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#ifdef HAVE_NET_ETHERNET_H
#include <net/ethernet.h>
#else
// workaround if we don't have <net/ethernet.h>
#define ETHERMTU 1500
#endif
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdlib.h>
#include <string.h>
#include <sys/shm.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

//! for ID-logging. See log.H:
#define MYLOGID fd

//! internal states:
#define TCPCS_UNKNOWN   0
#define TCPCS_READING   1
#define TCPCS_WRITING   2
#define TCPCS_BOGUS     4

// ######################################################################
TCPcliServ::TCPcliServ()
{ state = TCPCS_BOGUS; }

// ######################################################################
void TCPcliServ::init(const int connected_fd,
                      const in_addr_t my_ipaddr,
                      const short int my_port,
                      const in_addr_t cli_ipaddr,
                      const short int cli_port,
                      const int inqlen, const int outqlen,
                      const bool indlast, const bool outdlast,
                      const bool disableShm)
{
  shmcount = (getpid() << 24) + (size_t(this) << 16);
  fd = connected_fd; myIP = my_ipaddr; myPort = my_port;
  cliIP = cli_ipaddr; cliPort = cli_port;
  inmsgqlen = inqlen; outmsgqlen = outqlen;
  indroplast = indlast; outdroplast = outdlast;

  // switch fd to non-blocking and TCP_NODELAY modes:
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags == -1)
    { IDPLERROR("Cannot get socket flags"); state = TCPCS_BOGUS; }
  if (fcntl(fd, F_SETFL, O_NONBLOCK | flags) == -1)
    { IDPLERROR("Cannot set socket flags"); state = TCPCS_BOGUS; }
  int w = 1;  // watermark size
  if (setsockopt(fd, 6, TCP_NODELAY, (void *)&w, sizeof(int)))
    { IDPLERROR("Cannot set socket TCP_NODELAY"); state = TCPCS_BOGUS; }

  // init mutexes: FIXME: bogus if init called several times...
  pthread_mutex_init(&mutin, NULL);
  pthread_mutex_init(&mutou, NULL);

  // do other standard initializations:
  reset();

  // can we use shared memory?
  if (myIP == cliIP && disableShm == false) useShm = true; else useShm = false;

  // show some debug info:
  struct in_addr me; me.s_addr = htonl(myIP);
  struct in_addr him; him.s_addr = htonl(cliIP);
  char buf[32]; strncpy(buf, inet_ntoa(me), 32);
  if (useShm)
    IDLDEBUG("Ready: %s:%hu <-SHM-> %s:%hu", buf, myPort,
             inet_ntoa(him), cliPort);
  else
    IDLDEBUG("Ready: %s:%hu <-TCP-> %s:%hu", buf, myPort,
             inet_ntoa(him), cliPort);
}

// ######################################################################
TCPcliServ::TCPcliServ(const int connected_fd,
                       const in_addr_t my_ipaddr,
                       const short int my_port,
                       const in_addr_t cli_ipaddr,
                       const short int cli_port,
                       const int inqlen, const int outqlen,
                       const bool indlast, const bool outdlast,
                       const bool disableShm)
{
  state = TCPCS_BOGUS;
  init(connected_fd, my_ipaddr, my_port, cli_ipaddr, cli_port,
       inqlen, outqlen, indlast, outdlast, disableShm);
}

// ######################################################################
TCPcliServ::~TCPcliServ()
{
  disconnect(); pthread_mutex_destroy(&mutin); pthread_mutex_destroy(&mutou);

  // release all our shared memory segments:
  if (shmmap.empty() == false)
    for (std::map<int, void *>::const_iterator
           m = shmmap.begin(); m != shmmap.end(); m ++)
      if (shmdt(m->second) == -1) IDPLERROR("Error detaching Shm %d",m->first);
}

// ######################################################################
void TCPcliServ::send(const TCPmessage& msg)
{
  pthread_mutex_lock(&mutou);

  // check if our outgoing queue is full, and take action. Be careful
  // not to drop Shm acknowledge messages, though:
  int xx;
  if (outmsgqlen > 0 && oumsg.size() >= outmsgqlen &&
      msg.checkShmDone(xx) == false)
    {
      if (outdroplast)
      {
       // IDLERROR("Outgoing queue full -- DROPPING MOST RECENT MESSAGE");
      }
      else
        {
         // IDLERROR("Outgoing queue full -- DROPPING LEAST RECENT MESSAGE");
          if (oumsg.front().checkShmDone(xx) == false) oumsg.pop_front();
          oumsg.push_back(msg);  // keep local copy of message
        }
    }
  else
    oumsg.push_back(msg);  // keep local copy of message

  pthread_mutex_unlock(&mutou);
  // message will be sent out whenever its turn comes in check()...
}

// ######################################################################
bool TCPcliServ::receive(TCPmessage& msg)
{
  bool ret = false;
  pthread_mutex_lock(&mutin);
  if (inmsg.size() && inmsg.front().isBusy() == false)
    { msg = inmsg.front(); inmsg.pop_front(); ret = true; }
  pthread_mutex_unlock(&mutin);
  return ret;
}

// ######################################################################
int TCPcliServ::nbReceived()
{
  pthread_mutex_lock(&mutin);
  int ret = inmsg.size();
  if (ret > 0 && inmsg.front().isBusy()) ret = 0; // one msg, still receiving
  pthread_mutex_unlock(&mutin);
  return ret;
}

// ######################################################################
int TCPcliServ::check()
{
  if (state == TCPCS_BOGUS) return TCPBUG;  // I am bogus... leave me alone!

  // are we reading in a message?
  if (state & TCPCS_READING)
    {
      int r = im.readFrom(fd);  // read more; returns TCPWAITREAD, TCPDONE, or TCPBUG
      switch(r)
        {
        case TCPWAITREAD:  // need more data for current message
          break;
        case TCPDONE:      // current message done; save it and wait for new one
          storeReceivedMessage();  // store what we have received
          state &= ~TCPCS_READING;  // not reading a message any more...
          break;
        case TCPBUG:       // something went wrong; suicide
          state = TCPCS_BOGUS;
          break;
        default:
          LFATAL("Unknown return from TCPmessage::readFrom()");
        }
    }

  // if we are not reading anything, is there a new incoming message?
  if ((state & TCPCS_READING) == 0)
    {
      // attempt a read and see if there is anything coming in:
      int r = im.readHeaderFrom(fd);
      switch(r)
        {
        case TCPDONE:     // got a new message and we have its header ready
          // does the message have a non-empty body? if so, let's get
          // ready to read it. Otherwise we are done with this message.
          if (im.getSize() > 0) state |= TCPCS_READING;  // will start reading
          else storeReceivedMessage(); // store what we have received
          break;
        case TCPWAITREAD: // nothing new
          break;
        default:       // something bogus happened
          state = TCPCS_BOGUS;
        }
    }

  // are we writing out a message?
  if (state & TCPCS_WRITING)
    {
      int r = om.writeTo(fd);  // returns WAITWRITE, DONE, or BUG
      switch(r)
        {
        case TCPWAITWRITE: // send more data for current message
          break;
        case TCPDONE:      // current message done
          state &= ~TCPCS_WRITING;
          break;
        case TCPBUG:       // something went wrong; suicide
          state = TCPCS_BOGUS;
          break;
        default:
          LFATAL("Unknown return from TCPmessage::writeTo()");
        }
    }

  // if not writing, do we have pending messages in out queue?
  if ((state & TCPCS_WRITING) == 0)
    {
      // we are not writing any message; do we have one in the queue?
      pthread_mutex_lock(&mutou);
      if (oumsg.size())
        {
          int xx; bool shmdone = oumsg.front().checkShmDone(xx);
          int siz = oumsg.front().getSize();

          // are we using shared memory, and this message is not an shm
          // message acknowledge (which must go via TCP), and the
          // message is not very small (otherwise, TCP will be faster)?
          if (useShm &&
              shmdone == false &&
              siz > int(ETHERMTU - sizeof(TCPmessage::TCPmessageHeader)))
            {
              key_t key = shmcount ++;

              // create new shared memory segment:
              int shmid = shmget(key, siz, 0666 | IPC_CREAT | IPC_EXCL);
              if (shmid == -1)
                IDPLFATAL("Cannot create shared memory segment");

              // attach segment to our address space:
              char *shmbuf = (char *)shmat(shmid, NULL, 0);
              if (shmbuf == (char *)(-1))
                IDPLFATAL("Cannot attach shared memory segment");

              // keep track of this new segment:
              shmmap[shmid] = shmbuf;

              // copy message into shared memory zone:
              memcpy(shmbuf, (void *)(oumsg.front().getMsg()), siz);

              // create a TCPmessage of type TCPMSG_SHMINFO
              om.reset(oumsg.front().getID(), oumsg.front().getAction());
              om.addShmInfo(shmid, siz);

              // pop message to be sent from send queue:
              oumsg.pop_front();
            }
          else
            {
              // regular TCP transfer:
              om = oumsg.front(); // get next message to write out
              oumsg.pop_front();  // not in the queue any more...
            }
          if (shmdone == false)
            LDEBUG("Sending off [%d, %d] to %d",om.getID(),om.getAction(),fd);

          // write out the message header:
          if (om.writeHeaderTo(fd) != TCPDONE)
            { state = TCPCS_BOGUS; return TCPBUG; }

          // does the message have a non-empty body? if so, we will
          // write it out at the next check(). Otherwise we are done.
          if (om.getSize() > 0)
            state |= TCPCS_WRITING;  // ok, we are writing this guy out
        }
      pthread_mutex_unlock(&mutou);
    }

  // ok, now what do we return to SockServ? we always want to monitor for read,
  // and also for write if we are writing something out.
  if (state == TCPCS_BOGUS) return TCPBUG;
  else if (state & TCPCS_WRITING) return TCPWAITRW;
  else return TCPWAITREAD;
}

// ######################################################################
int TCPcliServ::disconnect()
{ reset(); return TCPFINISH; }

// ######################################################################
int TCPcliServ::reset()
{
  state = TCPCS_UNKNOWN;
  return TCPDONE;
}

// ######################################################################
void TCPcliServ::storeReceivedMessage()
{
  int shmid, siz;

  // is it a shared memory message?
  if (im.checkShmInfo(shmid, siz))
    {
      // attach to shared memory segment:
      char *msgbuf = (char *)shmat(shmid, NULL, 0);
      if (msgbuf == (char *)(-1))
        IDPLERROR("Cannot attach to shared memory segment");
      else
        {
          // make a deep copy of the shared memory segment into our
          // private memory, so that we can immediately release the
          // shared memory:
          TCPmessage insm(im.getID(), im.getAction(), im.getETI(),
                          msgbuf, siz);

          // push the TCPmessage into our received queue:
          queueIncomingMessage(insm);

          // build an acknowledgement message:
          TCPmessage ack(insm.getID(), insm.getAction());
          ack.addShmDone(shmid);

          // release shared memory:
          if (shmdt(msgbuf) == -1)
            IDPLERROR("Error detaching Shm segment %d", shmid);

          // send acknowledgement (will be forced to go via TCP):
          send(ack);
        }
    }
  // is it a shared memory segment release?
  else if (im.checkShmDone(shmid))
    {
      std::map<int, void *>::const_iterator m = shmmap.find(shmid);
      if (m != shmmap.end())
        {
          // ok, we do have this segment; let's release it:
          if (shmdt(m->second) == -1)
            IDPLERROR("Error detaching Shm segment %d", m->first);

          // nobody needs it anymore; let's destroy it:
          if (shmctl(shmid, IPC_RMID, 0) == -1)
            IDPLERROR("Error deleting Shm segment %d", m->first);

          // let's forget about this segment:
          shmmap.erase(shmid);
        }
      else
        IDLERROR("Attempt to release unknown Shm %d -- IGNORED", shmid);
    }
  // then it's a regular TCP message
  else
    queueIncomingMessage(im);

  // we are done with the contents of this message:
  im.freeMem();
}

// ######################################################################
void TCPcliServ::queueIncomingMessage(TCPmessage& imsg)
{
  pthread_mutex_lock(&mutin);

  // check if our incoming queue is full, and take action. Be careful
  // not to drop Shm acknowledge messages, though:
  int xx;
  if (inmsgqlen > 0 && inmsg.size() >= inmsgqlen &&
      imsg.checkShmDone(xx) == false)
    {
      if (indroplast)
      {
       // IDLERROR("Incoming queue full -- DROPPING MOST RECENT MESSAGE");
      }
      else
        {
         // IDLERROR("Incoming queue full -- DROPPING LEAST RECENT MESSAGE");
          if (inmsg.front().checkShmDone(xx) == false) inmsg.pop_front();
          inmsg.push_back(imsg);
        }
    }
  else
    inmsg.push_back(imsg);

  pthread_mutex_unlock(&mutin);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

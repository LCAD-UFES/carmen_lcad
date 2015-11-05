/*!@file Beowulf/TCPcommunicator.C A class to handle multiple TCPmessage
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Beowulf/TCPcommunicator.C $
// $Id: TCPcommunicator.C 11538 2009-07-30 06:23:37Z itti $
//

#include "Beowulf/TCPcommunicator.H"

#include "Beowulf/BeowulfOpts.H"
#include "Beowulf/SockServ.H"
#include "Beowulf/TCPdefs.H"
#include "Component/OptionManager.H"
#include "Util/Assert.H"
#include "Util/log.H"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>

void* TCPcommunicator_run(void *c0);  // will live in a separate thread

//! for id-logging; see log.H:
#define MYLOGID fd

namespace
{
  bool operator==(const in_addr& addr1, const in_addr& addr2)
  {
    return addr1.s_addr == addr2.s_addr;
  }
}

// ######################################################################
void* TCPcommunicator_run(void *c0)
{
  // this is just a wrapper so that we call the member function run() on
  // the TCPcommunicator object passed as argument:
  TCPcommunicator *c = (TCPcommunicator *)c0;
  c->run(); return NULL;
}

// ######################################################################
TCPcommunicator::TCPcommunicator(OptionManager& mgr,
                                 const std::string& descrName,
                                 const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  myIP(&OPT_TCPcommunicatorIPaddr, this),
  inqlen(&OPT_TCPcommunicatorInQlen, this),
  ouqlen(&OPT_TCPcommunicatorOuQlen, this),
  indroplast(&OPT_TCPcommunicatorInDropLast, this),
  oudroplast(&OPT_TCPcommunicatorOuDropLast, this),
  disableShm(&OPT_TCPcommunicatorDisableShm, this),
  server(new SockServ(mgr))
{
  // make our server a subcomponent of us:
  addSubComponent(server);
  threadRunning.atomic_set(0);
}

// ######################################################################
void TCPcommunicator::start1()
{
  // determine my IP if not given:
  if (myIP.getVal().s_addr == 0)
    {
      char hostname[32]; gethostname(hostname, 32);
      struct hostent *he = gethostbyname(hostname);
      if (he == NULL) LFATAL("Cannot determine my IP address");

      in_addr myip;
      myip.s_addr = ntohl( ((in_addr *)(he->h_addr_list[0]))->s_addr );
      myIP.setVal(myip);
      LINFO("Using IP address %s", myIP.getValString().c_str());
    }

  // block signals:
  sigset_t sset; sigemptyset(&sset);
  sigaddset(&sset, SIGHUP); sigaddset(&sset, SIGPIPE);
  int s = sigprocmask(SIG_BLOCK, &sset, NULL);
  if (s != 0) PLERROR("Sigprocmask failed");

  // let now our SockServ start, and we'll be back in start2()
}

// ######################################################################
void TCPcommunicator::start2()
{
  // setup client array:
  typedef TCPcliServ* TCPcliServPtr;
  cli = new TCPcliServPtr[FD_SETSIZE];
  for (int i = 0; i < FD_SETSIZE; i ++) cli[i] = NULL; // no client
  to_add = false; to_stop = false;

  // start thread for run():
  threadRunning.atomic_set(1);
  pthread_create(&runner, NULL, &TCPcommunicator_run, (void *)this);
}

// ######################################################################
void TCPcommunicator::stop1()
{
  // we are going to signal to the worker thread that we want it to
  // stop by setting to_stop=true, then we wait for it to respond by
  // setting threadRunning=0; however this will also work if the
  // worker thread has already quit before we even begin stop1(), in
  // which case threadRunning will be already 0

  struct timespec ts, ts2;
  while (threadRunning.atomic_get() != 0)
    {
      to_stop = true;
      ts.tv_sec = 0; ts.tv_nsec = 1000 * POLL_SLEEP;
      nanosleep(&ts, &ts2);
    }

  ASSERT(threadRunning.atomic_get() == 0);

  delete [] cli; cli = 0;
}

// ######################################################################
TCPcommunicator::~TCPcommunicator()
{ }

// ######################################################################
int TCPcommunicator::contact(const char *hostname, const bool blocking)
{
  // extract port information from name:port format in hostname:
  char hn2[strlen(hostname) + 1]; strcpy(hn2, hostname);
  int port = DEFPORT;
  for (int i = strlen(hn2) - 1; i > 0; i --)
    if (hn2[i] == ':') // ok, we have port information
      { hn2[i] = '\0'; port = atoi(&(hn2[i + 1])); }

  // get IP address:
  struct hostent *he = gethostbyname(hn2);
  if (he == NULL)
    { LERROR("Cannot gethostbyname(%s): %d", hn2, h_errno); return -1; }

  // ready to contact:
  return contact(ntohl( ((in_addr *)(he->h_addr_list[0]))->s_addr ),
                 port, blocking);
}

// ######################################################################
int TCPcommunicator::contact(const in_addr_t ip, const short int port,
                             const bool blocking)
{
  ASSERT(started());

  int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd == -1) { PLERROR("Cannot create socket"); return -1; }

  // attempt a connect to server:
  struct sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(ip);
  addr.sin_port = htons(port);

  struct in_addr him; him.s_addr = htonl(ip);
  if (connect(fd, (struct sockaddr*)(&addr), sizeof(addr)) == -1) {
    IDPLERROR("Error connecting to %s:%hu", inet_ntoa(him), port);
    return -1;
  }
  IDLDEBUG("Connected to %s:%hu", inet_ntoa(him), port);

  // switch to non-blocking mode:
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags == -1)
    { IDPLERROR("Cannot get socket flags"); close(fd); return -1; }
  if (fcntl(fd, F_SETFL, O_NONBLOCK | flags) == -1)
    { IDPLERROR("Cannot set socket flags"); close(fd); return -1; }

  // get ready to add new TCPcliServ (will be done in run thread):
  addfd = fd; addip = ip; addport = port;
  pollvar(&to_add); to_add = true; if (blocking) pollvar(&to_add);

  return fd;
}

// ######################################################################
void TCPcommunicator::run()
{
  bool running = true;
  const short myPort = server->getModelParamVal<short>("SockServPort");

  while(running) {
    // were we instructed to stop?
    if (to_stop) { running = false; break; }

    // check if we have a new client added through contact():
    if (to_add) {
      if (cli[addfd]) {
        LERROR("Already have client %d! Killing.", addfd);
        terminateCli(addfd);
      }
      cli[addfd] = new
        TCPcliServ(addfd, myIP.getVal().s_addr, myPort, addip, addport,
                   inqlen.getVal(), ouqlen.getVal(),
                   indroplast.getVal(), oudroplast.getVal(),
                   disableShm.getVal());
      server->addClient(addfd); server->monitorRead(addfd);
      server->monitorError(addfd); to_add = false;
    }

    // check what's going on with all our clients:
    int client;
    int result = server->check(TCP_TIMEOUT_SEC, TCP_TIMEOUT_USEC);

    switch(result) {

    case SOCKSERV_BUG:                        // server got bogus. The end...
      LERROR("Server messed-up. ABORT."); running = false;
      break;
    case SOCKSERV_IDLE:                                // nothing happened...
      break;
    case SOCKSERV_ACTIV:                 // some activity! let's check it out

      // ########## first, check for new clients and configure them:
      while((client = server->getNewClient())) {
        if (cli[client]) {
          LDEBUG("Already have TCPcliServ %d? Killing", client);
          terminateCli(client);
        }
        cli[client] = new TCPcliServ(client, myIP.getVal().s_addr, myPort,
                                     server->getClientIP(client),
                                     server->getClientPort(client),
                                     inqlen.getVal(), ouqlen.getVal(),
                                     indroplast.getVal(), oudroplast.getVal(),
                                     disableShm.getVal());
        server->monitorRead(client); server->monitorWrite(client, false);
        server->monitorError(client);
        // wait until this client has some data for us to read
      }

      // ########## check for clients that got errored:
      while((client = server->getErrorClient())) {
        if (cli[client] == NULL) continue;  // already gone
        LDEBUG("Connection to client %d messed-up. Terminating.", client);
        terminateCli(client);
      }

      // ########## check for clients ready to be read from / written to:
      while((client = server->getRWClient()))
        {
          if (cli[client] == NULL) continue;  // already gone
          switch(cli[client]->check())
            {
            case TCPDONE:                 // transaction done -> wait for next one
            case TCPWAITREAD:                          // this guy needs more data
              server->monitorRead(client);
              server->monitorWrite(client, false);
              break;
            case TCPWAITWRITE:                 // this guy wants to send more data
              server->monitorWrite(client);
              server->monitorRead(client);  // always be open to incoming stuff
              break;
            case TCPWAITRW:             // this guy wants to send & read more data
              server->monitorWrite(client);
              server->monitorRead(client);
              break;
            case TCPBUG:                                  // this guy is messed-up
              LDEBUG("TCPcliServ %d messed-up. Terminating.", client);
              terminateCli(client);
              break;
            case TCPFINISH:                                // this guy is finished
              terminateCli(client);
              break;
            }
        }
      break;
    default:
      LERROR("Server gave unknown result %d. Ignored.", result);
    }
  }
  threadRunning.atomic_set(0);
  pthread_exit(0);
}

// ######################################################################
void TCPcommunicator::terminateAll()
{ for (int i = 0; i < FD_SETSIZE; i ++) if (cli[i]) terminateCli(i); }

// ######################################################################
void TCPcommunicator::terminateAllButOne(const int fd)
{ for (int i = 0; i < FD_SETSIZE; i ++) if (cli[i] && i!=fd) terminateCli(i); }

// ######################################################################
void TCPcommunicator::terminateCli(const int fd)
{
  ASSERT(fd >= 0 && fd < FD_SETSIZE);
  if (cli[fd])
    {
      server->disconnect(fd);
      if (cli[fd]) delete cli[fd];
      cli[fd] = NULL;
    }
}

// ######################################################################
void TCPcommunicator::send(const int sfd, TCPmessage& smsg)
{
  ASSERT(sfd >= 0 && sfd < FD_SETSIZE);
  if (cli[sfd] == NULL)
    LERROR("No client %d to send to!", sfd);
  else
    {
      cli[sfd]->send(smsg);  // thread-safe
      server->monitorWrite(sfd); server->monitorRead(sfd);
      server->monitorError(sfd);
    }
}

// ######################################################################
bool TCPcommunicator::receive(int& rfd, TCPmessage& rmsg,
                              const int timeout, int* err)
{
  // is there any incoming message already?
  bool got_one = receiveIt(rfd, rmsg, err);
  if (got_one) return true;  // got a message!

  // if we have a non-zero timeout, let's do a select() and try again:
  if (timeout) {
    // wait for the fds to have something ready
    waitFor(rfd, timeout);

    // if something came in, it should have also been picked up by the
    // receiver thread, and it may be available now.  Try once more
    // and return status:
    return receiveIt(rfd, rmsg, err);
  }

  // nothing received:
  return false;
}

// ######################################################################
int TCPcommunicator::nbReceived(const int rfd)
{
  int nb = 0;

  if (rfd != -1)  // only check specific fd
    {
      if (cli[rfd] == NULL) { LERROR("No client %d!", rfd); nb = 0; }
      else nb = cli[rfd]->nbReceived();
    }
  else  // rfd = -1, then check all fds
    for (int i = 0; i < FD_SETSIZE; i ++)
      if (cli[i] != NULL)
        nb += cli[i]->nbReceived();
  return nb;
}

// ######################################################################
void TCPcommunicator::waitFor(const int rfd, const int timeout)
{
  // we'll do a blocking select on those fds we are interested in:
  fd_set rfds, wfds, efds; FD_ZERO(&rfds); FD_ZERO(&wfds); FD_ZERO(&efds);
  if (rfd == -1) {  // monitor all our fds
    for (int i = 0; i < FD_SETSIZE; i ++)
      if (cli[i] != NULL) FD_SET(i, &rfds);
  } else FD_SET(rfd, &rfds);
  struct timeval to; to.tv_sec = 0; to.tv_usec = timeout * 1000;

  // don't care about the result; we just want to wait:
  select(FD_SETSIZE, &rfds, &wfds, &efds, &to);
}

// ######################################################################
void TCPcommunicator::pollvar(volatile bool *var)
{
  struct timespec ts, ts2;
  while (*var == true)
    {
       ts.tv_sec = 0; ts.tv_nsec = 1000 * POLL_SLEEP;
       nanosleep(&ts, &ts2);
    }
}

// ######################################################################
bool TCPcommunicator::receiveIt(int& rfd, TCPmessage& rmsg, int* err)
{
  if (threadRunning.atomic_get() != 1)
    LFATAL("Oops! The communicator is not running anymore");

  ASSERT(rfd >= -1 && rfd < FD_SETSIZE); bool got_one = false;

  if (rfd != -1)  // receive from specific fd:
    {
      if (cli[rfd] == NULL)
        {
          LERROR("No client %d to receive from!", rfd);
          if (err != 0) *err = 1;
          got_one = false;
        }
      else
        got_one = cli[rfd]->receive(rmsg);
    }
  else  // rfd = -1, then receive from any fd:
    {
      for (int i = 0; i < FD_SETSIZE; i ++)
        if (cli[i] != NULL)
          {
            got_one = cli[i]->receive(rmsg);
            if (got_one) { rfd = i; break; }
          }
    }
  return got_one;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

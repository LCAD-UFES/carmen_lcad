/*!@file Beowulf/SockServ.C A simple multi-client socket server */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Beowulf/SockServ.C $
// $Id: SockServ.C 12451 2010-01-02 23:51:24Z lior $
//

#include "Beowulf/SockServ.H"

#include "Beowulf/BeowulfOpts.H"
#include "Beowulf/TCPdefs.H"
#include "Component/OptionManager.H"
#include "Util/log.H"

#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define MYLOGID sock

//! internal states (in 'state' member):
#define SOCKSERV_UNCONFIGURED 0
#define SOCKSERV_RUNNING      1
#define SOCKSERV_BOGUS        2

//! timeout for termination of idle clients, in seconds:
#define CLITIMEOUT 3000

// ######################################################################
SockServ::SockServ(OptionManager& mgr, const std::string& descrName,
                   const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsPort(&OPT_SockServPort, this),
  itsServ("SockServServiceName", this, "uscbeo"),
  itsServType("SockServServiceType", this, "tcp"),
  itsCliTout("SockServCliTout", this, 3000),
  itsQlen("SockServQlen", this, 256)
{ state = SOCKSERV_UNCONFIGURED; }

// ######################################################################
SockServ::~SockServ()
{ IDLDEBUG("Terminating."); }

// ######################################################################
void SockServ::start1()
{
  state = SOCKSERV_BOGUS;  // in case we terminate early
  nbclients = 0; nbnewcli = 0; nbrcli = 0; nbecli = 0; nbwcli = 0; nbnucli = 0;

  // fds which we monitor: nothing
  FD_ZERO(&readfds); FD_ZERO(&writefds); FD_ZERO(&exceptfds);

  // find out our port from /etc/services if it is unset:
  if (itsPort.getVal() == 0) {
    struct servent *sv =
      getservbyname(itsServ.getVal().c_str(), itsServType.getVal().c_str());
    if (sv == NULL)
      {
        LERROR("%s/%s not set in /etc/services -- USING DEFAULT PORT",
               itsServ.getVal().c_str(), itsServType.getVal().c_str());
        itsPort.setVal(DEFPORT);
      }
    else
      itsPort.setVal(ntohs(sv->s_port));
    LINFO("Listening to port %d...", itsPort.getVal());
  }

  // ########## setup server:
  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock == -1) { PLERROR("Cannot create server socket"); return; }

  struct sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);  // accept from any host
  addr.sin_port = htons(itsPort.getVal());
  if (bind(sock, (struct sockaddr*)(&addr), sizeof(addr)) == -1)
    {
      IDPLFATAL("Cannot bind server socket to port %d",
                int(itsPort.getVal()));
      return;
    }
  if (listen(sock, itsQlen.getVal()) == -1)
    { IDPLFATAL("Cannot setup server listen queue"); return; }
  IDLDEBUG("Listen queue (len=%d) configured", itsQlen.getVal());

  // monitor server socket for read (=new client) and error
  FD_SET(sock, &readfds); FD_SET(sock, &exceptfds);
  state = SOCKSERV_RUNNING;
}

// ######################################################################
void SockServ::stop2()
{
  while(nbnewcli)
    { IDLDEBUG("Terminating new client %d", newcli[nbnewcli-1]);
    close(newcli[--nbnewcli]); }
  while(nbclients)
    { IDLDEBUG("Terminating client %d", cli[nbclients-1]);
    close(cli[--nbclients]); }
  if (sock != -1)
    { IDLDEBUG("Terminating server"); close(sock); sock = -1; }

  // fds which we monitor: nothing
  FD_ZERO(&readfds); FD_ZERO(&writefds); FD_ZERO(&exceptfds);

  // get ready for a restart:
  nbclients = 0; nbnewcli = 0; nbrcli = 0; nbecli = 0; nbwcli = 0; nbnucli = 0;
  state = SOCKSERV_UNCONFIGURED;
}

// ######################################################################
int SockServ::check(const int stimeout, const int utimeout)
{
  if (state != SOCKSERV_RUNNING) return SOCKSERV_BUG;
  struct timeval timeout; timeout.tv_sec=stimeout; timeout.tv_usec=utimeout;

  // did we take care of all new clients since last check?
  if (nbnewcli) {
    IDLERROR("Terminating %d new clients from previous check", nbnewcli);
    for (int i = 0; i < nbnewcli; i ++) disconnect(newcli[i]);
    nbnewcli = 0;
  }

  // wait for activity on some of our fds:
  memcpy(&rfds, &readfds, sizeof(fd_set));
  memcpy(&wfds, &writefds, sizeof(fd_set));
  memcpy(&efds, &exceptfds, sizeof(fd_set));
  nbrcli = 0; nbwcli = 0; nbecli = 0;

  //  if (LOG_FLAGS & LOG_FULLTRACE) debug("Monitoring: ");

  int result = select(FD_SETSIZE, &rfds, &wfds, &efds, &timeout);

  //  if (LOG_FLAGS & LOG_FULLTRACE) debug("Activated:  "); printf("\n");

  if (result == -1) { IDPLERROR("Error in select"); return SOCKSERV_BUG; }
  else if (result > 0) {
    int tim = time(NULL);  // for timout of idle fds

    // ##### first, check activity on the server socket:
    if (FD_ISSET(sock, &rfds)) {                               // a new client!
      FD_CLR(sock, &rfds); // clear so we don't interpret it in client loop
      // establish connection with new client:
      struct sockaddr_in clientaddr;
      socklen_t client_addrlen = sizeof(clientaddr);
      int clientfd = accept(sock, (struct sockaddr *)(&clientaddr),
                            &client_addrlen);
      if (clientfd == -1)
        IDPLDEBUG("Accept failed. Bogus client ignored.");
      else {
        cliIP[clientfd] = ntohl(clientaddr.sin_addr.s_addr);
        cliPort[clientfd] = ntohs(clientaddr.sin_port);

        IDLDEBUG("Connect -- Adding %s:%hu as client %d",
                 inet_ntoa(clientaddr.sin_addr), cliPort[clientfd], clientfd);
        newcli[nbnewcli++] = clientfd;
      }
    }
    if (FD_ISSET(sock, &efds)) {
      FD_CLR(sock, &efds); IDLFATAL("I got messed-up"); state = SOCKSERV_BOGUS;
      return SOCKSERV_BUG;
    }

    // ##### now, check what's going on with the clients:
    for (int i = 0; i < nbclients; i ++) {
      int fd = cli[i];
      // check for clients ready to read from:
      if (FD_ISSET(fd, &rfds))
        { rcli[nbrcli++] = fd; if (clitimeout[fd] != -1) clitimeout[fd]=tim; }
      // check for clients ready for writing to them:
      if (FD_ISSET(fd, &wfds))
        { wcli[nbwcli++] = fd; if (clitimeout[fd] != -1) clitimeout[fd]=tim; }
      // check for errored clients:
      if (FD_ISSET(fd, &efds)) ecli[nbecli++] = fd;
      // check for timed-out clients:
      else if (clitimeout[fd] != -1 && tim - clitimeout[fd] > CLITIMEOUT)
        { IDLDEBUG("Timeout on client %d", fd); ecli[nbecli++] = fd; }
    }
  } else {  // select returned 0 (timeout); just check for timed-out clients
    int tim = time(NULL);  // for timout of idle fds
    for (int i = 0; i < nbclients; i ++)
      if (clitimeout[cli[i]] != -1 && tim - clitimeout[cli[i]] > CLITIMEOUT)
        { IDLDEBUG("Timeout on client %d", cli[i]); ecli[nbecli++] = cli[i]; }
  }

  if (nbnewcli == 0 && nbrcli == 0 && nbwcli == 0 && nbecli == 0)
    return SOCKSERV_IDLE;  // nothing interesting happened...

  // if we got some new clients, register them:
  if (nbnewcli) addNewClients();

  return SOCKSERV_ACTIV;
}

// ######################################################################
int SockServ::getNewClient()
{ if (nbnewcli) return newcli[--nbnewcli]; else return 0; }

// ######################################################################
int SockServ::getReadClient()
{ if (nbrcli) return rcli[--nbrcli]; else return 0; }

// ######################################################################
int SockServ::getWriteClient()
{ if (nbwcli) return wcli[--nbwcli]; else return 0; }

// ######################################################################
int SockServ::getErrorClient()
{ if (nbecli) return ecli[--nbecli]; else return 0; }

// ######################################################################
in_addr_t SockServ::getClientIP(const int clifd) const
{
  for (int i = 0; i < nbclients; i ++)
    if (cli[i] == clifd) return cliIP[clifd];
  IDLERROR("Unknown client %d", clifd);
  return 0;
}

// ######################################################################
short int SockServ::getClientPort(const int clifd) const
{
  for (int i = 0; i < nbclients; i ++)
    if (cli[i] == clifd) return cliPort[clifd];
  IDLERROR("Unknown client %d", clifd);
  return 0;
}

// ######################################################################
int SockServ::getRWClient()
{
  if (nbwcli) return wcli[--nbwcli];
  if (nbrcli) return rcli[--nbrcli];
  return 0;
}

// ######################################################################
void SockServ::disconnect(const int client)
{
  if (hasClient(client))
    {
      IDLDEBUG("Disconnecting client %d", client);
      FD_CLR(client, &readfds); FD_CLR(client, &writefds);
      FD_CLR(client, &exceptfds);
      close(client);  // terminate the connection
      deleteClient(client);
    }
  else
    IDLDEBUG("No client %d. Ignored.", client);
}

// ######################################################################
bool SockServ::addUserClient(void* data)
{
  if (nbnucli == MAXNBNUCLI) return false;
  nuclidata[nbnucli++] = data; return true;
}

// ######################################################################
void* SockServ::getNewUserClient()
{
  if (nbnucli) return nuclidata[--nbnucli];
  return NULL;
}

// ######################################################################
void SockServ::addNewClients()
{ for (int i = 0; i < nbnewcli; i ++) addClient(newcli[i]); }

// ######################################################################
void SockServ::deleteClient(const int client)
{
  for (int i = 0; i < nbclients; i ++)
    if (cli[i] == client) {  // found it
      for (int j = i+1; j < nbclients; j ++) cli[j-1] = cli[j];
      nbclients --; break;
    }
}

// ######################################################################
bool SockServ::hasClient(const int client)
{
  if (client == sock) // this is the server...
    { IDLDEBUG("Called on server socket!"); return false; }
  for (int i = 0; i < nbclients; i ++) if (cli[i] == client) return true;
  return false;
}

// ######################################################################
bool SockServ::monitorRead(const int client, const bool startstop)
{
  if (hasClient(client)) {
    if (startstop) FD_SET(client, &readfds); else FD_CLR(client, &readfds);
    return true;
  }
  return false;
}

// ######################################################################
bool SockServ::monitorWrite(const int client, const bool startstop)
{
  if (hasClient(client)) {
    if (startstop) FD_SET(client, &writefds); else FD_CLR(client, &writefds);
    return true;
  }
  return false;
}

// ######################################################################
bool SockServ::monitorError(const int client, const bool startstop)
{
  if (hasClient(client)) {
    if (startstop) FD_SET(client, &exceptfds); else FD_CLR(client, &exceptfds);
    return true;
  }
  return false;

}

// ######################################################################
bool SockServ::noTimeOut(const int client)
{
  if (hasClient(client)) { clitimeout[client] = -1; return true; }
  return false;
}

// ######################################################################
void SockServ::resetTimeOut(const int client)
{ if (hasClient(client)) clitimeout[client] = time(NULL); }

// ######################################################################
void SockServ::addClient(const int client)
{
  if (client < 0 || client >= FD_SETSIZE)
    { IDLDEBUG("Invalid client number %d. Ignored.", client); return; }
  if (hasClient(client)) IDLDEBUG("Already have client %d. Ignored.", client);
  else { cli[nbclients++] = client; clitimeout[client] = time(NULL); }
}

// ######################################################################
void SockServ::debug(const char *label)
{
  printf("%s",label);
  for (int i = 0; i < FD_SETSIZE; i ++)
    if (FD_ISSET(i, &rfds) || FD_ISSET(i, &wfds) || FD_ISSET(i, &efds)) {
      printf("%d(", i);
      if (FD_ISSET(i, &rfds)) printf("R");
      if (FD_ISSET(i, &wfds)) printf("W");
      if (FD_ISSET(i, &efds)) printf("E");
      printf(") ");
    }
  printf("\n");
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/*!@file Beowulf/SockServ.H A simple multi-client socket server */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Beowulf/SockServ.H $
// $Id: SockServ.H 5335 2005-08-17 14:27:01Z rjpeters $
//

#ifndef SOCKSERV_H_DEFINED
#define SOCKSERV_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>

//! max mumber of new clients that can come between two check():
#define MAXNBNUCLI 128

//! return values for check():
#define SOCKSERV_IDLE   0
#define SOCKSERV_BUG    1
#define SOCKSERV_ACTIV  2

//! This is a multi-socket connection server
/*! This is a multi-socket connection server; it listens onto a master
  socket and accepts incoming connections; it then can monitor all
  open connections for read, write and/or error, all in a single call
  to check(), using a non-blocking select().

  Classes using SockServ should call check() to determine which
  connections require attention. Subsequently, they should take
  appropriate action on the connections that are active, using
  getXXXclient().  In adition to the connections that are
  automatically open by SockServ on accept() on the master socket,
  users may tell SockServ to monitor additional (already open)
  connections, by adding them to SockServ's monitor list using
  addUserClient().  By default, connections time out after CLITIMEOUT
  time of inactivity. This can be disabled using noTimeOut(). */
class SockServ : public ModelComponent {
public:
  //! SockServ constructor
  SockServ(OptionManager& mgr,
           const std::string& descrName = "Socket Server",
           const std::string& tagName = "SockServ");

  //! SockServ destructor
  ~SockServ();

  //! check what's going on
  int check(const int stimeout, const int utimeout);

  //! zero if no more new clients, else client fd
  int getNewClient();

  //! zero if no more new clients, else client fd
  int getReadClient();

  //! zero if no more new clients, else client fd
  int getWriteClient();

  //! zero if no more new clients, else client fd
  int getErrorClient();

  //! get IP address of a client
  in_addr_t getClientIP(const int clifd) const;

  //! get port of a client
  short int getClientPort(const int clifd) const;

  //! all wclients first, then all rclients
  int getRWClient();

  //! disconnect a client
  void disconnect(const int client);

  //! external users can also add their own new clients:
  bool addUserClient(void *data);
  void* getNewUserClient();

  //! what should we monitor for, for a given connection:
  bool monitorRead(const int client, const bool startstop = true);

  //! what should we monitor for, for a given connection:
  bool monitorWrite(const int client, const bool startstop = true);

  //! what should we monitor for, for a given connection:
  bool monitorError(const int client, const bool startstop = true);

  bool noTimeOut(const int client);

  void resetTimeOut(const int client);

  void addClient(const int client);
  void deleteClient(const int client);

protected:
  OModelParam<short int> itsPort;       //!< My port to listen to
  NModelParam<std::string> itsServ;     //!< Name of service in /etc/services
  NModelParam<std::string> itsServType; //!< Type of service in /etc/services
  NModelParam<int> itsCliTout;          //!< Timeout on idel clients (in s)
  NModelParam<int> itsQlen;             //!< Length of listen queue

private:
  void start1();  // get started
  void stop2();   // stop ourselves

  void init(const int servport, const int qlen);
  void closedown();

  void addNewClients();
  bool hasClient(const int client);
  void debug(const char *label);

  int state;
  int sock, port;
  fd_set readfds, writefds, exceptfds, rfds, wfds, efds;
  int newcli[FD_SETSIZE], rcli[FD_SETSIZE], wcli[FD_SETSIZE], ecli[FD_SETSIZE];
  void* nuclidata[MAXNBNUCLI];
  int nbnewcli, nbrcli, nbwcli, nbecli, nbnucli;

  int clitimeout[FD_SETSIZE];
  int cli[FD_SETSIZE];
  unsigned long int cliIP[FD_SETSIZE];
  short int cliPort[FD_SETSIZE];
  int nbclients;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

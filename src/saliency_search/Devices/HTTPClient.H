/*!@file Devices/HTTPClient.H  HTTP client for interfacing with http type devices */

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
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/HTTPClient.H $
// $Id: HTTPClient.H 12962 2010-03-06 02:13:53Z irock $
//

#ifndef __HTTPClient_H__
#define __HTTPClient_H__

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <netdb.h>

//! Interface to a HTTPClient port
/*! The socket will be open at start() time; see ModelComponent.H for
  details. */
class HTTPClient : public ModelComponent {
public:
  //! Constructor
  HTTPClient(OptionManager& mgr, const std::string& descrName = "HTTPClient Port",
         const std::string& tagName = "HTTPClientPort");

  //! destructor
  ~HTTPClient(void);

  //! write bytes from the port
  int read(void* buffer, const int nbytes);

  //! write bytes to the port
  int write(const void* buffer, const int nbytes);

  //! send request to the http server
  std::string sendGetRequest(const std::string& request);

  //! send Post data to the http server
  std::string sendPostRequest(const std::string& request, const char* data, long size);


  //! get data back
  std::string readData();

  //! Connect to the server
  void openConnection();

  //! Close the connection
  void closeConnection();

protected:

  //! Resolve hostname to ip address
  std::string  getIPAddress(const std::string& hostName);

  OModelParam<std::string> itsHostName; //! The host to contact
  OModelParam<unsigned int> itsPort;      //!< The port to contact on


  //! open the port and get started
  void start1();

  //! close the port and get stopped
  void stop2();

private:
  int itsSocket; // the socket to send/recv data on
  struct sockaddr_in * itsRemote; //the remove sockaddr
  bool itsConnected; //Are we connected to the server


};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/*!@file Devices/HttpServer.H  HTTP server for interfacing with http type devices */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/HTTPServer.H $
// $Id: HTTPServer.H 12962 2010-03-06 02:13:53Z irock $
//

#ifndef __HTTPServer_H__
#define __HTTPServer_H__

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

//! Interface to a HttpServer port
/*! The socket will be open at start() time; see ModelComponent.H for
  details. */
class HttpServer : public ModelComponent {
public:
  //! Constructor
  HttpServer(OptionManager& mgr, const std::string& descrName = "HttpServer Port",
         const std::string& tagName = "HttpServerPort");

  //! destructor
  ~HttpServer(void);

        //! Accept a connection and return the connection fd
        int acceptConn();

        //! write a string to the client fd
        int writeData(int clientFd, std::string& msg);

protected:

  //! Resolve hostname to ip address
  OModelParam<unsigned int> itsPort;      //!< The port we listen on

  //! open the port and get started
  void start1();

  //! close the port and get stopped
  void stop2();


private:
  int itsSocket; // the socket to send/recv data on
  struct sockaddr_in * itsRemote; //the remove sockaddr

};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

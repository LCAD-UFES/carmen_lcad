/*!@file Devices/HttpServer.C  HTTP server for interfacing with http type devices */


// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
// by the University of Southern California (USC) and the iLab at USC.  //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/HTTPServer.C $
// $Id: HTTPServer.C 12962 2010-03-06 02:13:53Z irock $
//

#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>

#include "Devices/HTTPServer.H"

#include "Component/OptionManager.H"
#include "Component/ModelOptionDef.H"
#include "Devices/DeviceOpts.H"
#include "Util/log.H"
#include "rutz/unixcall.h" // for rutz::unixcall::get_file_user_pid()

extern const ModelOptionCateg MOC_HTTP;

const ModelOptionCateg MOC_HTTP = {
  MOC_SORTPRI_2,   "HttpServer-Related Options" };

static const ModelOptionDef OPT_HttpServerPort =
  { MODOPT_ARG(uint), "ServerPort", &MOC_HTTP, OPTEXP_CORE,
    "Port number of the server to use ",
    "HttpServer-port", '\0', "<portnum>", "80" };


// ######################################################################
HttpServer::HttpServer(OptionManager& mgr,
               const std::string& descrName,
               const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsPort(&OPT_HttpServerPort, this),
  itsSocket(-1)
{

}

// ######################################################################
HttpServer::~HttpServer(void)
{
        LINFO("Clossing connections");
        close(itsSocket);

}


// ######################################################################
void HttpServer::start1()
{
        //Start the http server

   struct sockaddr_in servaddr;

         /* Avoid possible SIGPIPE when sending data */
         signal(SIGPIPE, SIG_IGN);

   itsSocket=socket(AF_INET,SOCK_STREAM,0);
         if (itsSocket < 0)
                 LFATAL("Can not create socket\n");

   bzero(&servaddr,sizeof(servaddr));
   servaddr.sin_family = AF_INET;
   servaddr.sin_addr.s_addr=htonl(INADDR_ANY);
   servaddr.sin_port=htons(itsPort.getVal());

         //Set non blocking
         long arg;
         if( (arg = fcntl(itsSocket, F_GETFL, NULL)) < 0)
                 LFATAL("Error fcntl(..., F_GETFL)\n");
         arg |= O_NONBLOCK;

         if( fcntl(itsSocket, F_SETFL, arg) < 0)
                 LFATAL("Error fcntl(..., F_SETFL)\n");


   if (bind(itsSocket,(struct sockaddr *)&servaddr,sizeof(servaddr)) < 0)
                 LFATAL("Can not bind to port: %i\n", itsPort.getVal());

   if (listen(itsSocket,1024) < 0)
                 LFATAL("Can not listen to port\n");
}

// ######################################################################
void HttpServer::stop2()
{
        close(itsSocket);
}

int HttpServer::acceptConn()
{
        struct sockaddr_in cliaddr;
        socklen_t clilen=sizeof(cliaddr);

        return accept(itsSocket,(struct sockaddr *)&cliaddr,&clilen);
}

int HttpServer::writeData(int clientFd, std::string& msg)
{
        return write(clientFd, msg.c_str(), msg.size());
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

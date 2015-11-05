/*!@file Devices/HTTPClient.C  HTTP client for interfacing with http type devices */


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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/HTTPClient.C $
// $Id: HTTPClient.C 12962 2010-03-06 02:13:53Z irock $
//

#include "Devices/HTTPClient.H"

#include "Component/OptionManager.H"
#include "Component/ModelOptionDef.H"
#include "Devices/DeviceOpts.H"
#include "Util/log.H"
#include "rutz/unixcall.h" // for rutz::unixcall::get_file_user_pid()

extern const ModelOptionCateg MOC_HTTP;

const ModelOptionCateg MOC_HTTP = {
  MOC_SORTPRI_2,   "HTTPClient-Related Options" };

static const ModelOptionDef OPT_HTTPClientHost =
  { MODOPT_ARG_STRING, "HostName", &MOC_HTTP, OPTEXP_CORE,
    "IP address of the server",
    "httpClient-hostname", '\0', "<name>", "localhost" };

static const ModelOptionDef OPT_HTTPClientPort =
  { MODOPT_ARG(uint), "ServerPort", &MOC_HTTP, OPTEXP_CORE,
    "Port number of the server to use ",
    "httpClient-port", '\0', "<portnum>", "80" };

// ######################################################################
HTTPClient::HTTPClient(OptionManager& mgr,
               const std::string& descrName,
               const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsHostName(&OPT_HTTPClientHost, this),
  itsPort(&OPT_HTTPClientPort, this),
  itsSocket(-1),
  itsRemote(NULL),
  itsConnected(false)
{  }

// ######################################################################
void HTTPClient::start1()
{

}

// ######################################################################
void HTTPClient::openConnection()
{
  if((itsSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
    LFATAL("Can't create TCP socket");

  std::string ipAddress = getIPAddress(itsHostName.getVal());

  LDEBUG("Connecting to %s", ipAddress.c_str());

  struct sockaddr_in * itsRemote = (struct sockaddr_in *)malloc(sizeof(struct sockaddr_in *));
  itsRemote->sin_family = AF_INET;
  int res = inet_pton(AF_INET, ipAddress.c_str(), (void *)(&(itsRemote->sin_addr.s_addr)));
  if( res < 0)
    LFATAL("Can't set remote->sin_addr.s_addr");
  else if(res == 0)
    LFATAL("%s is invalid.", ipAddress.c_str());
  itsRemote->sin_port = htons(itsPort.getVal());

  //Connect to the host
  if(connect(itsSocket, (struct sockaddr *)itsRemote, sizeof(struct sockaddr)) < 0)
  {
    LINFO("Can not connect to %s", itsHostName.getVal().c_str());
    itsConnected = false;
  } else {
    itsConnected = true;
  }


}


// ######################################################################
void HTTPClient::closeConnection()
{
  if (itsSocket >= 0)
    close(itsSocket);
}



// ######################################################################
std::string  HTTPClient::getIPAddress(const std::string& hostName)
{

  int iplen = 15; //XXX.XXX.XXX.XXX
  char ip[15];

  struct hostent *hent;
  if((hent = gethostbyname(hostName.c_str())) == NULL)
    LFATAL("Can not resolve ip for %s", hostName.c_str());

  if(inet_ntop(AF_INET, (void *)hent->h_addr_list[0], ip, iplen) == NULL)
    LFATAL("Can not resolve ip for %s", hostName.c_str());

  std::string ipAddress = ip;
  return ipAddress;
}


// ######################################################################
void HTTPClient::stop2()
{
  if (itsSocket >= 0)
    close(itsSocket);
  free(itsRemote);
}

// ######################################################################
int HTTPClient::read(void* buffer, const int nbytes)
{
  int n = 0;
  return n;
}


// ######################################################################
int HTTPClient::write(const void* buffer, const int nbytes)
{
  int n =  0;
  return n;
}

// ######################################################################
std::string HTTPClient::sendGetRequest(const std::string& request)
{
  //Send a full http request
  std::string queryString = "GET " + request + " HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\nUser-Agent: httpClient\r\n\r\n";

  openConnection();

  std::string retData;
  if (itsConnected)
  {
    unsigned int sent = 0;
    while(sent < queryString.size())
    {
      int ret = send(itsSocket, queryString.c_str()+sent, queryString.size()-sent, 0);
      if(ret == -1)
        LINFO("Can not send query");
      sent += ret;
    }

    retData = readData();
  }

  closeConnection();


  return retData;
}

// ######################################################################
std::string HTTPClient::sendPostRequest(const std::string& request, const char* data, long size)
{
  //Send a full http request
  char dataSize[255];
  sprintf(dataSize, "%lu", size);
  std::string queryString = "POST " + request + " HTTP/1.1\r\nHost: localhost\r\nConnection: Close\r\nUser-Agent: httpClient\r\nContent-Length: " + dataSize + "\r\n\r\n";

  openConnection();

  std::string retData;
  if (itsConnected)
  {
    unsigned int sent = 0;
    while(sent < queryString.size())
    {
      int ret = send(itsSocket, queryString.c_str()+sent, queryString.size()-sent, 0);
      if(ret == -1)
        LINFO("Can not send query");
      sent += ret;
    }

    //Send Post Data
    sent = 0;
    while(sent < (unsigned int)(size))
    {
      int ret = send(itsSocket, data+sent, size-sent, 0);
      if(ret == -1)
        LINFO("Can not send query");
      sent += ret;
    }

    //retData = readData();
  }

  closeConnection();


  return retData;
}



// ######################################################################
std::string HTTPClient::readData()
{
  std::string data;

  char buf[1024];

  int ret = 0;
  int htmlstart = 0;
  char * htmlcontent;
  while((ret = recv(itsSocket, buf, 1024, 0)) > 0)
  {
    buf[ret] = 0;
    if(htmlstart == 0)
    {
      htmlcontent = strstr(buf, "\r\n\r\n");
      if(htmlcontent != NULL){
        htmlstart = 1;
        data = htmlcontent + 4;
      }
    }else{
      data = data + buf;
    }
  }


  return data;
}



// ######################################################################
HTTPClient::~HTTPClient(void)
{  }



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

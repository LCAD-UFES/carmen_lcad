/*!@file Robots2/Beobot2/Hardware/app-BeoSim.C Beobot2.0 simulator	*/
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
// Primary maintainer for this file: Chin-Kai Chang <chinkaic@usc.edu>
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Hardware/app-BeoSim.C
// $ $Id: app-BeoSim.C 15190 2012-02-29 04:00:13Z kai $
//
//////////////////////////////////////////////////////////////////////////
#include "Component/ModelManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelOptionDef.H"
#include "Robots/Beobot2/Hardware/BeoSim.H"
#include <Ice/Ice.h>
#include <Ice/Service.h>
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/SimEventsUtils.H"
#include "Ice/IceImageUtils.H"
#include "Ice/RobotBrainObjects.ice.H"


// ######################################################################
// ######################################################################
class RobotBrainServiceService : public Ice::Service {
  protected:
#if ICE_INT_VERSION >= 30402
    virtual bool start(int, char* argv[],int &);
#else
    virtual bool start(int, char* argv[]);
#endif	
    virtual bool stop() {
      if (itsMgr)
        delete itsMgr;
      return true;
    }

  private:
    Ice::ObjectAdapterPtr itsAdapter;
    ModelManager *itsMgr;
};

// ######################################################################
#if ICE_INT_VERSION >= 30402
bool RobotBrainServiceService::start(int argc, char* argv[],int& status)
#else
bool RobotBrainServiceService::start(int argc, char* argv[])
#endif	
{
  char adapterStr[255];

  //Create the topics
//  SimEventsUtils::createTopic(communicator(), "BeoStemMessageTopic");

  //Create the adapter
  int port = RobotBrainObjects::RobotBrainPort;
  bool connected = false;

  while(!connected)
  {
    try
    {
      LINFO("Trying Port:%d", port);
      sprintf(adapterStr, "default -p %i", port);
      itsAdapter = communicator()->createObjectAdapterWithEndpoints("BeoSim",
          adapterStr);
      connected = true;
    }
    catch(Ice::SocketException)
    {
      port++;
    }
  }

  //Create the manager and its objects
  itsMgr = new ModelManager("BeoStemService");

  LINFO("Starting BeoSim System");
  nub::ref<BeoSim> motor(new BeoSim(0, *itsMgr, "BeoSim", "BeoSim"));
  LINFO("BeoSim Created");
  itsMgr->addSubComponent(motor);
  LINFO("BeoSim Added As Sub Component");
  motor->init(communicator(), itsAdapter);
  LINFO("BeoSim Inited");

  itsMgr->parseCommandLine((const int)argc, (const char**)argv, "", 0, 0);

  itsAdapter->activate();

  itsMgr->start();

  return true;
}

// ######################################################################
int main(int argc, char** argv) {

  RobotBrainServiceService svc;
  return svc.main(argc, argv);
}



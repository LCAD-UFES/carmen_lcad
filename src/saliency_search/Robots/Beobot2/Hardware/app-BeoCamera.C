/*!@file Robots/Beobot2/Hardware/app-BeoCamera.C ICE module for sending out
  images from a camera */
// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2002   //
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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Beobot2/Hardware/app-BeoCamera.C $
// $Id: app-BeoCamera.C 15391 2012-08-13 20:25:01Z kai $
//

#include "Component/ModelManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelOptionDef.H"

#include "Robots/Beobot2/Hardware/BeoCamera.H"

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
  MYLOGVERB = LOG_INFO;

  char adapterStr[255];

  //Create the topics
//  SimEventsUtils::createTopic(communicator(), "BeoCameraMessageTopic");

  //Create the adapter
  int port = RobotBrainObjects::RobotBrainPort;
  bool connected = false;

  // try to connect to ports until successful
  LDEBUG("Opening Connection");
  while(!connected)
  {
    try
    {
      LINFO("Trying Port:%d", port);
      sprintf(adapterStr, "default -p %i", port);
      itsAdapter = communicator()->createObjectAdapterWithEndpoints
        ("BeoCamera", adapterStr);
      connected = true;
    }
    catch(Ice::SocketException)
    {
      port++;
    }
  }

  //Create the manager and its objects
  itsMgr = new ModelManager("BeoCameraService");

  LINFO("Starting BeoCamera System");
  nub::ref<BeoCamera> cam
    (new BeoCamera(*itsMgr, "BeoCamera", "BeoCamera"));
  LINFO("BeoCamera created");
  itsMgr->addSubComponent(cam);
  LINFO("BeoCamera Added As a subcomponent");
  cam->init(communicator(), itsAdapter);
  LINFO("BeoCamera initiated");

  // check command line inputs/options
  if (itsMgr->parseCommandLine (argc, argv, "", 0, 0) == false)
    return(1);

  // activate manager and adapter
  itsAdapter->activate();
  itsMgr->start();

  return true;
}

// ######################################################################
int main(int argc, char** argv) {

  RobotBrainServiceService svc;
  
  Ice::InitializationData id;
  id.properties = Ice::createProperties();

  // Set the max message size to 100mb
  // usually for sending/receiving large-sized images
  id.properties->setProperty("Ice.MessageSizeMax", "102400"); 

  return svc.main(argc, argv, id);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */


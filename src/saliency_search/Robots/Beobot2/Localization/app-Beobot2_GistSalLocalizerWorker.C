/*!@file  Robots/Beobot2/Localization/app-Beobot2_GistSalLocalizerMaster.C
 localization using saliency and gist (worker node). 
 See Siagian_Itti09tr                                                   */
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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Beobot2/Localization/app-Beobot2_GistSalLocalizerWorker.C $
// $Id: app-Beobot2_GistSalLocalizerWorker.C 15441 2012-11-14 21:28:03Z kai $
//

#include "Component/ModelManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelOptionDef.H"

#include <Ice/Ice.h>
#include <Ice/Service.h>
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/SimEventsUtils.H"
#include "Ice/IceImageUtils.H"

#include "Robots/Beobot2/Localization/Beobot2_GistSalLocalizerWorker.H"


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

  LINFO("Creating Topic!");

  // Create the topics
  // SimEventsUtils::createTopic(communicator(),
  //                            "Beobot2_GistSalLocalizerWorkerMessageTopic");

  //Create the adapter
  int port = RobotBrainObjects::RobotBrainPort;
  bool connected = false;
  LDEBUG("Opening Connection");

  while(!connected)
  {
    try
    {
      LINFO("Trying Port:%d", port);
      sprintf(adapterStr, "default -p %i", port);
      itsAdapter = communicator()->createObjectAdapterWithEndpoints
        ("Beobot2_GistSalLocalizerWorker", adapterStr);
      connected = true;
    }
    catch(Ice::SocketException)
    {
      port++;
    }
  }

  // Create the manager and its objects
  itsMgr = new ModelManager("Beobot2_GistSalLocalizerWorkerService");

  LINFO("Starting Beobot2_GistSalLocalizerWorker");
  nub::ref<Beobot2_GistSalLocalizerWorkerI> gslw
    (new Beobot2_GistSalLocalizerWorkerI
     (*itsMgr, "Beobot2_GisSaltLocalizerWorker",
      "Beobot2_GistSalLocalizerWorker"));
  LINFO("Beobot2_GistSalLocalizerWorker Created");
  itsMgr->addSubComponent(gslw);
  LINFO("Beobot2_GistSalLocalizerWorker Added As Sub Component");
  gslw->init(communicator(), itsAdapter);
  LINFO("Beobot2_GistSalLocalizerWorker Initiated");

  if (itsMgr->parseCommandLine((const int)argc, (const char**)argv,
                               "<environment file> "
                               "<worker index> "
                               "<number of workers> ",
                               1, 3)
      == false) return(1);

  // initialize an environment - even if the .env file does not exist
  // automatically create a blank environment, ready to be built
  std::string envFName = itsMgr->getExtraArg(0);
  LINFO("Loading Env file: %s", envFName.c_str());
  rutz::shared_ptr<Environment> env(new Environment(envFName));
  //env->setWindow(matchWin);

  // link the environment to the localizer
  gslw->setEnvironment(env);
  if(itsMgr->numExtraArgs() >  2)
    gslw->setWorkerInformation
      (itsMgr->getExtraArgAs<uint>(1), itsMgr->getExtraArgAs<uint>(2));

  itsAdapter->activate();

  itsMgr->start();

  return true;
}

// ######################################################################
int main(int argc, char** argv)
{
  LINFO("Creating Service...");
  RobotBrainServiceService svc;
  LINFO("Service Created...");
  return svc.main(argc, argv);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

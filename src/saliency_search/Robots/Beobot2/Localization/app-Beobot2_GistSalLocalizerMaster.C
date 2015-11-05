/*!@file  Robots/Beobot2/Localization/app-Beobot2_GistSalLocalizerMaster.C
 localization using saliency and gist (master node). 
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Beobot2/Localization/app-Beobot2_GistSalLocalizerMaster.C $
// $Id: app-Beobot2_GistSalLocalizerMaster.C 15441 2012-11-14 21:28:03Z kai $
//

// ######################################################################
// ######################################################################
#include "Component/ModelManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelOptionDef.H"
#include "Robots/Beobot2/Localization/Beobot2_GistSalLocalizerMaster.H"
#include <Ice/Ice.h>
#include <Ice/Service.h>
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/SimEventsUtils.H"
#include "Ice/IceImageUtils.H"

#include "Beobot/beobot-GSnav-def.H"
#include "Beobot/GSnavResult.H"

#include <sys/stat.h>
#include <errno.h>
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

  LINFO("Creating Topic!");
  //Create the topics
//  SimEventsUtils::createTopic(communicator(), "LandmarkDBWorkerMessageTopic");

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
        ("LandmarkDBWorker", adapterStr);
      connected = true;
    }
    catch(Ice::SocketException)
    {
      port++;
    }
  }

  //Create the manager and its objects
  itsMgr = new ModelManager("Beobot2_GistSalLocalizerMasterService");

  LINFO("Starting Beobot2_GistSalLocalizerMaster");
  nub::ref<Beobot2_GistSalLocalizerMasterI> gslm
    (new Beobot2_GistSalLocalizerMasterI
     (*itsMgr, "Beobot2_GistSalLocalizerMaster",
      "Beobot2_GistSalLocalizerMaster"));
  LINFO("Beobot2_GistSalLocalizerMaster Created");
  itsMgr->addSubComponent(gslm);
  LINFO("Beobot2_GistSalLocalizerMaster Added As Sub Component");
  gslm->init(communicator(), itsAdapter);
  LINFO("Beobot2_GistSalLocalizerMaster Initiated");

  itsMgr->exportOptions(MC_RECURSE);
  if (itsMgr->parseCommandLine((const int)argc, (const char**)argv,
                               "<environment file> "
                               "<number of workers> "
                               "[test run file prefix] ",
                               2, 3)
      == false) return(1);


  // initialize an environment - even if the .env file does not exist
  // automatically create a blank environment, ready to be built
  std::string envFName = itsMgr->getExtraArg(0);

  // get the environment file and folder to save
  std::string saveFilePath = "";
  std::string::size_type lspos = envFName.find_last_of('/');
  int ldpos = envFName.find_last_of('.');
  std::string envPrefix;
  if(lspos != std::string::npos)
    {
      saveFilePath = envFName.substr(0, lspos+1);
      envPrefix =  envFName.substr(lspos+1, ldpos - lspos - 1);
    }
  else
    envPrefix =  envFName.substr(0, ldpos - 1);
  LINFO("Env file: %s", envFName.c_str());

  // get the time of day
  time_t rawtime; struct tm * timeinfo;
  time ( &rawtime );
  timeinfo = localtime ( &rawtime );
  char buffer [80];
  strftime (buffer,80, "results_%Y_%m_%d__%H_%M_%S",timeinfo);
  std::string testRunFPrefix(buffer);
  if(itsMgr->numExtraArgs() >  2)
    testRunFPrefix = itsMgr->getExtraArgAs<std::string>(2);

  std::string testRunFolder =
    saveFilePath + testRunFPrefix + std::string("/");

  std::string resultPrefix = testRunFolder + envPrefix;
  LINFO("result prefix: %s", resultPrefix.c_str());

  // create the session result folder
  if (mkdir(testRunFolder.c_str(), 0777) == -1 && errno != EEXIST)
    {
      LFATAL("Cannot create log folder: %s", testRunFolder.c_str());
    }

//   // print the results summary
//   GSnavResult r1;
//   r1.read(resultPrefix, 9);
//   r1.createSummaryResult();
//   Raster::waitForKey();
//   // HACK: results for paper
//   reportResults(resultPrefix, 9);
//   Raster::waitForKey();

  // link the environment to the localizer
  rutz::shared_ptr<Environment> env(new Environment(envFName));
  gslm->setEnvironment(env);
  gslm->setNumWorkers(itsMgr->getExtraArgAs<uint>(1));

  gslm->setSavePrefix(resultPrefix);

  // initialize the particles
  // in case we have a starting belief
  uint fstart = 0;

  std::string sBelFName = resultPrefix +
    sformat("_bel_%07d.txt", fstart-1);
  LINFO("Starting belief file: %s", sBelFName.c_str());
  gslm->initParticles(sBelFName);

  itsAdapter->activate();

  itsMgr->start();

  return true;
}

// ######################################################################
int main(int argc, char** argv) {

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


/*! @file Simulations/SimEventOS.C Stab to run a simulation modules configures with an xml file */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: $
// $Id: $
//

#include "Component/JobServerConfigurator.H"
#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/Point2D.H"
#include "Media/SimFrameSeries.H"
#include "Neuro/NeuroOpts.H"
#include "Raster/Raster.H"
#include "Simulation/SimEventQueueConfigurator.H"
#include "Simulation/SimEventQueue.H"
#include "Util/AllocAux.H"
#include "Util/Pause.H"
#include "Util/Types.H"
#include "Util/csignals.H"
#include "Util/log.H"
#include "Util/sformat.H"
#include "Util/StringUtil.H"
#include "Util/Timer.H"
#include "rutz/trace.h"

#include <signal.h>
#include <stdio.h>
#include <sys/types.h>
#include <dlfcn.h>

#ifdef HAVE_LIBXML
#include <libxml/parser.h>
#include <libxml/tree.h>
#endif

typedef int mainLoopFunc(const int argc, const char** argv);

mainLoopFunc* loadObject(ModelManager& manager,
 const std::string& libFile,
 const std::string& name, 
 const std::string& mainOverrideName = "")
{
	void* libPtr = dlopen(libFile.c_str(), RTLD_NOW  | RTLD_GLOBAL);
	if (!libPtr)
		LFATAL("Can not load library: %s (%s)", libFile.c_str(), dlerror());

  std::string modInstFuncName = "createModule" + name;
	dlerror(); //reset any errors
	createSimEventModule* createObj = (createSimEventModule*) dlsym(libPtr, modInstFuncName.c_str());

	if (!createObj)
		LFATAL("Can not find the %s symbol: %s. \nCheck the module Name", modInstFuncName.c_str(), dlerror());

  mainLoopFunc* mainLoop = NULL;
  if(mainOverrideName != "")
  {
    dlerror(); //reset any errors
     mainLoop = (mainLoopFunc*) dlsym(libPtr, mainOverrideName.c_str());
     if(!mainLoop)
       LFATAL("Can not find mainLoop symbol (%s): %s. \nCheck the module Name", mainOverrideName.c_str(), dlerror());
    
  }

  nub::ref<SimModule> obj = createObj(manager);
  manager.addSubComponent(obj);

  //dlclose(libPtr);

  return mainLoop;
}

void getNodeMatchText(xmlDocPtr doc, xmlNodePtr nodePtr,
                                  const char* nodeName, std::string &result)
{
  xmlChar *tmp = NULL;
  if (!xmlStrcmp(nodePtr->name, (const xmlChar *)nodeName))
    tmp = xmlNodeListGetString(doc, nodePtr->xmlChildrenNode, 1);

  if (tmp != NULL) {
    result = std::string((const char*)tmp);
    xmlFree(tmp);
  }
}

mainLoopFunc* configModules(const char* xmlFile, ModelManager& manager,
 const int argc, const char **argv, bool skipFirstArgs)
{
	std::vector<const char*> argvals;

  int argcnt=0;

  for(int i=0; i<argc; i++)
	{
		if (skipFirstArgs && (i==1) ) continue;
		argvals.push_back(argv[i]);
		argcnt++;
	}


  // check if the file exists:
  xmlDocPtr doc = xmlReadFile(xmlFile, NULL, 0);
  if (doc == NULL) {
    LFATAL("Failed to parse %s", xmlFile);
  }

  /* Get the root element node */
  xmlNode *root_element = xmlDocGetRootElement(doc);

  // look for object annotations
  xmlNodePtr cur = root_element; //->xmlChildrenNode; //dont care about top level

  //Skip the top level if scenes is in the name
  if ((!xmlStrcmp(cur->name, (const xmlChar *)"SimModules")))
    cur = root_element->xmlChildrenNode; //dont care about top level
	
  mainLoopFunc* mainLoop = NULL;

	while (cur != NULL) {
		if ((!xmlStrcmp(cur->name, (const xmlChar *)"Module"))) {
			// get the  scene data:
			xmlNodePtr modulePtr = cur->xmlChildrenNode;

			std::string name, location, mainLoopFuncName;

      while(modulePtr != NULL) { // read the attributes and polygons
        getNodeMatchText(doc, modulePtr, "Name", name);
        getNodeMatchText(doc, modulePtr, "Location", location);
        getNodeMatchText(doc, modulePtr, "MainLoop", mainLoopFuncName);

        //Process the args
        if ((!xmlStrcmp(modulePtr->name, (const xmlChar *)"Args"))) {
          xmlNodePtr argPtr = modulePtr->xmlChildrenNode;

          while(argPtr != NULL)
          {

            if ((!xmlStrcmp(argPtr->name, (const xmlChar *)"arg"))) {
              xmlChar* name = xmlGetProp(argPtr, (const xmlChar*)"name");
              xmlChar* value = xmlGetProp(argPtr, (const xmlChar*)"value");

              std::string arg = "--" + std::string((const char*)name) +
                                "=" + std::string((const char*)value);
              argvals.push_back(strdup(arg.c_str()));
              argcnt++;

              xmlFree(name);
              xmlFree(value);
            }
            argPtr = argPtr->next;
          }
          
        }
        
				modulePtr = modulePtr->next;
			}

			mainLoopFunc* tmpMainLoop = loadObject(manager, location, name, mainLoopFuncName);
      if(tmpMainLoop)
      {
        if(!mainLoop)
          mainLoop = tmpMainLoop;
        else
          LFATAL("ERROR! Two Main Loops Defined In XML!");
      }

		}
		cur = cur->next;
	}
  xmlFreeDoc(doc);
  xmlCleanupParser();

	if (manager.parseCommandLine(
				(const int)argcnt, &argvals[0], "", 0, 0) == false)
    LFATAL("Bad Arguments!");

  return mainLoop;
}

volatile int signum = 0;

void* RunSimOS(void* seq_p)
{
  nub::ref<SimEventQueue> seq = *((nub::ref<SimEventQueue>*)seq_p);

  PauseWaiter p;
  SimStatus status = SIM_CONTINUE;

  Timer timer;
	//main loop:
  while(status == SIM_CONTINUE)
  {
    // Are we in pause mode, if so, hold execution:
    if (p.checkPause()) continue;

    if (signum != 0) {
      char msg[255];
      snprintf(msg, 255, "quitting because %s was caught", signame(signum));
      LINFO("%s", msg);
      seq->post(rutz::make_shared(new SimEventBreak(seq.get(), msg)));
      status = seq->evolve();
      break;
    }
    // Evolve for one time step and switch to the next one:
    status = seq->evolve();
  }
  pthread_exit(NULL);
}

int main(const int argc, const char **argv)
{
  // 'volatile' because we will modify this from signal handlers

  // catch signals and redirect them for a clean exit (in particular,
  // this gives us a chance to do useful things like flush and close
  // output files that would otherwise be left in a bogus state, like
  // mpeg output files):
  catchsignals(&signum);

  MYLOGVERB = LOG_INFO;
  ModelManager manager("SimEvent Kernel");

  nub::ref<SimEventQueueConfigurator>
    seqc(new SimEventQueueConfigurator(manager));
  manager.addSubComponent(seqc);

  // Request a bunch of option aliases (shortcuts to lists of options):
  REQUEST_OPTIONALIAS_NEURO(manager);

  std::string xmlFile = std::string(getenv("HOME")) + "/.SimEventOS.xml";

  mainLoopFunc* mainLoop = NULL;
	if (argc > 1)
	{
			if (argv[1][0] != '-') //This is an xml file, use it
					mainLoop = configModules(argv[1], manager, argc, argv, true);
		  else
					mainLoop = configModules(xmlFile.c_str(), manager, argc, argv, false);
	} else {
			mainLoop = configModules(xmlFile.c_str(), manager, argc, argv, false);
	}

  nub::ref<SimEventQueue> seq = seqc->getQ();

  manager.start();

  // temporary, for debugging...
  seq->printCallbacks();


  //Start up our SimEventOS runtime thread
  pthread_t SimOSThread;
  int rc = pthread_create(&SimOSThread, NULL, RunSimOS, (void*)(&seq));

  //Execute an overridden mainloop if it exists
  if(mainLoop)
    rc = mainLoop(argc, argv);

  //Wait for the SimOSThread loop to finish
  void* status;
  rc = pthread_join(SimOSThread, &status);
    
  // print final memory allocation stats
  LINFO("Simulation terminated.");

  // stop all our ModelComponents
  manager.stop();

  return rc;
}

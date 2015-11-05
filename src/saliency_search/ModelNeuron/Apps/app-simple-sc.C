/*!@file AppNeuro/app-simple-sc.C */

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
// Primary maintainer for this file: David Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/AppNeuro/app-simple-sc.C $

#include "Component/ModelManager.H"
#include "Media/SimFrameSeries.H"
#include "Util/Pause.H"
#include "Util/csignals.H"
#include "Simulation/SimEventQueueConfigurator.H"
#include "Simulation/SimEventQueue.H"

#include "Neuro/Retina.H"
#include "Neuro/EyeHeadControllerConfigurator.H"
#include "ModelNeuron/FreeViewingModel.H"

int submain(const int argc, const char **argv)
{
  volatile int signum = 0;
  catchsignals(&signum);

  MYLOGVERB = LOG_INFO;  // suppress debug messages

  //model manager
  ModelManager manager("Superior colliculus visual processing demo");

  //get some model components going
  nub::ref<SimEventQueueConfigurator>
    seqc(new SimEventQueueConfigurator(manager));
  manager.addSubComponent(seqc);

  nub::ref<SimOutputFrameSeries> ofs(new SimOutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  nub::ref<SimInputFrameSeries> ifs(new SimInputFrameSeries(manager));
  manager.addSubComponent(ifs);  

  nub::ref<RetinaStd> retina(new RetinaStd(manager));
  manager.addSubComponent(retina);
  
  nub::ref<EyeHeadControllerConfigurator>  scc(new EyeHeadControllerConfigurator(manager));
  manager.addSubComponent(scc);

  nub::ref<FreeViewingModel> fvm(new FreeViewingModel(manager));
  manager.addSubComponent(fvm);

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv, "", 0, 0) == false)
    return(1);

  nub::ref<SimEventQueue> seq = seqc->getQ();

  // let's get all our ModelComponent instances started:
  manager.start();

  // temporary, for debugging...
  seq->printCallbacks();

  PauseWaiter p;
  int retval = 0;
  SimStatus status = SIM_CONTINUE;

  // main loop:
  while(status == SIM_CONTINUE)
  {
    // Abort if we received a kill or similar signal:
    if (signum != 0) 
    {
      LINFO("quitting because %s was caught", signame(signum));
      retval = -1; break;
    }
    
    // Are we in pause mode, if so, hold execution:
    if (p.checkPause()) continue;
    
    // Evolve for one time step and switch to the next one:
    status = seq->evolve();
  }
  
  // print final memory allocation stats
  LINFO("Simulation terminated.");

  // stop all our ModelComponents
  manager.stop();
  
  // all done!
  return retval;
}

//the actual main
int main(const int argc, const char **argv)
{
  try
    {
      return submain(argc, argv);
    }
  catch (...)
    {
      REPORT_CURRENT_EXCEPTION;
    }

  return 1;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

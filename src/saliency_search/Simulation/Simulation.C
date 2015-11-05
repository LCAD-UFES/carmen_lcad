/*!@file Simulation/Simulation.C Main loop of a simulation */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Simulation/Simulation.C $
// $Id: Simulation.C 7444 2006-11-15 19:16:40Z rjpeters $
//

#ifndef SIMULATION_SIMULATION_C_DEFINED
#define SIMULATION_SIMULATION_C_DEFINED

#include "Simulation/Simulation.H"

#include "Simulation/SimEventQueue.H"
#include "Util/Pause.H"
#include "Util/csignals.H"


// ######################################################################
Simulation::Simulation(OptionManager& mgr)
  :
  ModelComponent(mgr, "Simulation", "Simulation"),
  itsQC(new SimEventQueueConfigurator(mgr))
{
  this->addSubComponent(itsQC);
}

// ######################################################################
void Simulation::addModule(nub::ref<SimModule> mod)
{
  itsModules.push_back(mod);
  this->addSubComponent(mod);
}

// ######################################################################
int Simulation::run()
{
  // 'volatile' because we will modify this from signal handlers
  volatile int signum = 0;

  // catch signals and redirect them for a clean exit (in particular,
  // this gives us a chance to do useful things like flush and close
  // output files that would otherwise be left in a bogus state, like
  // mpeg output files):
  catchsignals(&signum);

  nub::ref<SimEventQueue> seq = itsQC->getQ();

  PauseWaiter p;
  int retval = 0;

  // main loop:
  do {
    if (signum != 0) {
      LINFO("quitting because %s was caught", signame(signum));
      retval = -1; break;
    }

    if (p.checkPause()) continue;

    for (size_t i = 0; i < itsModules.size(); ++i)
      itsModules[i]->evolve(*seq);

  } while(seq->evolve() == SIM_CONTINUE);

  LINFO("Simulation terminated.");
  return retval;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // SIMULATION_SIMULATION_C_DEFINED

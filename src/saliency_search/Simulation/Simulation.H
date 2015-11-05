/*!@file Simulation/Simulation.H Main loop of a simulation */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Simulation/Simulation.H $
// $Id: Simulation.H 7279 2006-10-19 04:48:22Z itti $
//

#ifndef SIMULATION_SIMULATION_H_DEFINED
#define SIMULATION_SIMULATION_H_DEFINED

#include "Component/ModelComponent.H"
#include "Simulation/SimEventQueueConfigurator.H"
#include "Simulation/SimModule.H"

//! Encapsulation of the main loop of a simulation
/*! This class encapsulates the main loop of a typical
  simulation. Basically it holds a SimEventQueue and a bunch of
  SimModules, and will keep evolving the modules and the queue until
  some simulation break event terminates the simulation. Modules are
  evolved in the order in which they have been added using
  addModule(), and the queue is evolved last. */
class Simulation : public ModelComponent
{
public:
  //! Constructor
  Simulation(OptionManager& mgr);

  //! Destructor
  virtual ~Simulation();

  //! Add a module to our evolve list
  /*! At each time step, SimModule::evolve() will be called on each
    added module, then on our internal SimEventQueue. */
  void addModule(nub::ref<SimModule> mod);

  //! Run the simulation until it breaks
  /*! Keep evolve()'ing the simulation until some module posts a
    SimBreak event. Returns 0 if the simulation ended normally through
    catching a SimBreak event, or -1 if a Unix signal was caught
    (e.g., CTRL-C). */
  int run();

private:
  nub::ref<SimEventQueueConfigurator> itsQC;
  std::vector<nub::ref<SimModule> > itsModules;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // SIMULATION_SIMULATION_H_DEFINED

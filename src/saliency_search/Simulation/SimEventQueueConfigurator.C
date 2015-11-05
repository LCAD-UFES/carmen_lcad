/*!@file Simulation/SimEventQueueConfigurator.C Pick a SimEventQueue */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Simulation/SimEventQueueConfigurator.C $
// $Id: SimEventQueueConfigurator.C 8160 2007-03-21 21:34:16Z rjpeters $
//

#include "Simulation/SimEventQueueConfigurator.H"
#include "Simulation/SimEventQueue.H"
#include "Simulation/SimEventQueueDebug.H"
#include "Component/ModelOptionDef.H"
#include "Simulation/SimulationOpts.H"

const ModelOptionDef OPT_SimEventQueueType =
  { MODOPT_ARG_STRING, "SimEventQueueType", &MOC_SIM, OPTEXP_CORE,
    "Type of event queue used to control simulations. Usually you would want "
    "to use 'Std', for standard clocking and dispatching of events that "
    "arise in various modules being simulated. 'Debug' provides more verbose "
    "tracing of events, which is useful to debug simulation flow issues.",
    "seq-type", '\0',
    "<Std|Debug>", "Std" };

// ######################################################################
SimEventQueueConfigurator::
SimEventQueueConfigurator(OptionManager& mgr,
                          const std::string& descrName,
                          const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsSEQname(&OPT_SimEventQueueType, this),
  itsQ(new SimEventQueue(mgr))
{
  addSubComponent(itsQ);
}

// ######################################################################
SimEventQueueConfigurator::~SimEventQueueConfigurator()
{  }

// ######################################################################
nub::ref<SimEventQueue> SimEventQueueConfigurator::getQ() const
{ return itsQ; }

// ######################################################################
void SimEventQueueConfigurator::
paramChanged(ModelParamBase* const param,
             const bool valueChanged,
             ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);

  // was that a change of our baby's name?
  if (param == &itsSEQname) {
    // if we had one, let's unregister it (when we later reset() the
    // nub::ref, the current SaccadeController will unexport its
    // command-line options):
    removeSubComponent(*itsQ);

    // instantiate a controller of the appropriate type:
    if (itsSEQname.getVal().compare("Std") == 0)
      itsQ.reset(new SimEventQueue(getManager()));
    else if (itsSEQname.getVal().compare("Debug") == 0)
      itsQ.reset(new SimEventQueueDebug(getManager()));
    else
      LFATAL("Unknown SimEvent Queue type %s",
             itsSEQname.getVal().c_str());

    // add our baby as a subcomponent of us so that it will become
    // linked to the manager through us (hopefully we are registered
    // with the manager), which in turn will allow it to export its
    // command-line options and get configured:
    addSubComponent(itsQ);

    // tell the controller to export its options:
    itsQ->exportOptions(MC_RECURSE);

    // some info message:
    LINFO("Selected SimEventQueue of type %s", itsSEQname.getVal().c_str());
  }
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

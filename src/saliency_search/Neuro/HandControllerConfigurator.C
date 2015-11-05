/*!@file Neuro/HandControllerConfigurator.C Configure hand controller */

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
// Primary maintainer for this file: Dicky Nauli Sihite <sihite@usc.edu>
// $HeadURL:
// $Id:
//

#include "Neuro/HandControllerConfigurator.H"
#include "Neuro/HandControllers.H"
#include "Component/ModelOptionDef.H" // What is this anyway?
#include "Neuro/NeuroOpts.H"

// ######################################################################
// ######################################################################
// ######################################################################
HandControllerConfigurator::
HandControllerConfigurator(OptionManager& mgr,
                           const std::string& descrName,
                           const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsHandctrlType(&OPT_HandControllerType, this),
  itsHand(new StubHandController(mgr))
{
  addSubComponent(itsHand);
}

// ######################################################################
HandControllerConfigurator::~HandControllerConfigurator()
{  }

// ######################################################################
nub::ref<HandController> HandControllerConfigurator::getHand() const {
  return itsHand; }

// ######################################################################
void HandControllerConfigurator::
paramChanged(ModelParamBase* const param,
             const bool valueChanged,
             ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);
  
  // was that a change of our baby's name?
  if (param == &itsHandctrlType) {
    // if we had one, let's unregister it (when we later reset() the
    // nub::ref, the current HandController will unexport its
    // command-line options):
    removeSubComponent(*itsHand);

    // instantiate a controller of the appropriate type:
    if (itsHandctrlType.getVal().compare("None") == 0 ||
        itsHandctrlType.getVal().compare("Stub") == 0)
      itsHand.reset(new StubHandController(getManager()));
    else if (itsHandctrlType.getVal().compare("HandTrack") == 0)
      itsHand.reset(new TrackerHandController(getManager()));
    else
      LFATAL("Unknown HandController type %s, known type: None|Stub|HandTrack",
             itsHandctrlType.getVal().c_str());

    // add our baby as a subcomponent of us so that it will become
    // linked to the manager through us (hopefully we are registered
    // with the manager), which in turn will allow it to export its
    // command-line options and get configured:
    addSubComponent(itsHand);
    
    // tell the controller to export its options:
    itsHand->exportOptions(MC_RECURSE);
    
    // some info message:
    LINFO("Selected Hand Controller of type %s",
          itsHandctrlType.getVal().c_str());
  }
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

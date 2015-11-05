/*!@file Neuro/SaccadeControllerConfigurator.C Pick a SaccadeController */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SaccadeControllerConfigurator.C $
// $Id: SaccadeControllerConfigurator.C 8160 2007-03-21 21:34:16Z rjpeters $
//

#include "Neuro/SaccadeControllerConfigurator.H"
#include "Neuro/SaccadeControllers.H"
#include "Component/ModelOptionDef.H"
#include "Neuro/NeuroOpts.H"

// ######################################################################
// ######################################################################
// ######################################################################
SaccadeControllerEyeConfigurator::
SaccadeControllerEyeConfigurator(OptionManager& mgr,
                                 const std::string& descrName,
                                 const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsSacCtrlType(&OPT_SaccadeControllerEyeType, this),
  itsSC(new StubSaccadeController(mgr, SaccadeBodyPartEye))
{
  addSubComponent(itsSC);
}

// ######################################################################
SaccadeControllerEyeConfigurator::~SaccadeControllerEyeConfigurator()
{  }

// ######################################################################
nub::ref<SaccadeController> SaccadeControllerEyeConfigurator::getSC() const
{ return itsSC; }

// ######################################################################
void SaccadeControllerEyeConfigurator::
paramChanged(ModelParamBase* const param,
             const bool valueChanged,
             ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);

  // was that a change of our baby's name?
  if (param == &itsSacCtrlType) {
    // if we had one, let's unregister it (when we later reset() the
    // nub::ref, the current SaccadeController will unexport its
    // command-line options):
    removeSubComponent(*itsSC);

    // instantiate a controller of the appropriate type:
    if (itsSacCtrlType.getVal().compare("None") == 0 ||
        itsSacCtrlType.getVal().compare("Stub") == 0)
      itsSC.reset(new StubSaccadeController(getManager(),
                                            SaccadeBodyPartEye));
    else if (itsSacCtrlType.getVal().compare("Trivial") == 0)
      itsSC.reset(new TrivialSaccadeController(getManager(),
                                               SaccadeBodyPartEye));
    else if (itsSacCtrlType.getVal().compare("Fixed") == 0)
      itsSC.reset(new FixedSaccadeController(getManager(),
                                             SaccadeBodyPartEye));
    else if (itsSacCtrlType.getVal().compare("Friction") == 0)
      itsSC.reset(new FrictionSaccadeController(getManager(),
                                                SaccadeBodyPartEye));
    else if (itsSacCtrlType.getVal().compare("Threshold") == 0)
      itsSC.reset(new ThresholdSaccadeController(getManager(),
                                                 SaccadeBodyPartEye));
    else if (itsSacCtrlType.getVal().compare("Threshfric") == 0)
      itsSC.reset(new ThresholdFrictionSaccadeController(getManager(),
                                                         SaccadeBodyPartEye));
    else
      LFATAL("Unknown eye SaccadeController type %s",
             itsSacCtrlType.getVal().c_str());

    // add our baby as a subcomponent of us so that it will become
    // linked to the manager through us (hopefully we are registered
    // with the manager), which in turn will allow it to export its
    // command-line options and get configured:

    addSubComponent(itsSC);

    // tell the controller to export its options:
    itsSC->exportOptions(MC_RECURSE);

    // some info message:
    LINFO("Selected Eye SC of type %s", itsSacCtrlType.getVal().c_str());
  }
}


// ######################################################################
// ######################################################################
// ######################################################################
SaccadeControllerHeadConfigurator::
SaccadeControllerHeadConfigurator(OptionManager& mgr,
                                 const std::string& descrName,
                                 const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsSacCtrlType(&OPT_SaccadeControllerHeadType, this),
  itsSC(new StubSaccadeController(mgr, SaccadeBodyPartHead))
{
  addSubComponent(itsSC);
}

// ######################################################################
SaccadeControllerHeadConfigurator::~SaccadeControllerHeadConfigurator()
{  }

// ######################################################################
nub::ref<SaccadeController> SaccadeControllerHeadConfigurator::getSC() const
{ return itsSC; }

// ######################################################################
void SaccadeControllerHeadConfigurator::
paramChanged(ModelParamBase* const param,
             const bool valueChanged,
             ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);

  // was that a change of our baby's name?
  if (param == &itsSacCtrlType) {
    // if we had one, let's unregister it (when we later reset() the
    // nub::ref, the current SaccadeController will unexport its
    // command-line options):
    removeSubComponent(*itsSC);

    // instantiate a controller of the appropriate type:
    if (itsSacCtrlType.getVal().compare("None") == 0 ||
        itsSacCtrlType.getVal().compare("Stub") == 0)
      itsSC.reset(new StubSaccadeController(getManager(),
                                            SaccadeBodyPartHead));
    else if (itsSacCtrlType.getVal().compare("Trivial") == 0)
      itsSC.reset(new TrivialSaccadeController(getManager(),
                                               SaccadeBodyPartHead));
    else if (itsSacCtrlType.getVal().compare("Fixed") == 0)
      itsSC.reset(new FixedSaccadeController(getManager(),
                                             SaccadeBodyPartHead));
    else if (itsSacCtrlType.getVal().compare("Friction") == 0)
      itsSC.reset(new FrictionSaccadeController(getManager(),
                                                SaccadeBodyPartHead));
    else if (itsSacCtrlType.getVal().compare("Threshold") == 0)
      itsSC.reset(new ThresholdSaccadeController(getManager(),
                                                 SaccadeBodyPartHead));
    else if (itsSacCtrlType.getVal().compare("Threshfric") == 0)
      itsSC.reset(new ThresholdFrictionSaccadeController(getManager(),
                                                         SaccadeBodyPartHead));
    else
      LFATAL("Unknown head SaccadeController type %s",
             itsSacCtrlType.getVal().c_str());

    // add our baby as a subcomponent of us so that it will become
    // linked to the manager through us (hopefully we are registered
    // with the manager), which in turn will allow it to export its
    // command-line options and get configured:

    addSubComponent(itsSC);

    // tell the controller to export its options:
    itsSC->exportOptions(MC_RECURSE);

    // some info message:
    LINFO("Selected Head SC of type %s", itsSacCtrlType.getVal().c_str());
  }
}



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

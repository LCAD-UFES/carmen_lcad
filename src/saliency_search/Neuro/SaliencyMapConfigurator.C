/*!@file Neuro/SaliencyMapConfigurator.C SaliencyMap configurator (factory) for the --sm-type command-line option */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SaliencyMapConfigurator.C $
// $Id: SaliencyMapConfigurator.C 8160 2007-03-21 21:34:16Z rjpeters $
//

#ifndef NEURO_SALIENCYMAPCONFIGURATOR_C_DEFINED
#define NEURO_SALIENCYMAPCONFIGURATOR_C_DEFINED

#include "Neuro/SaliencyMapConfigurator.H"

#include "Neuro/NeuroOpts.H"
#include "Neuro/SaliencyMap.H"
#include "Neuro/SaliencyMapStdOptim.H"
#include "rutz/trace.h"

// ######################################################################
// ######################################################################
// ########## SaliencyMapConfigurator implementation
// ######################################################################
// ######################################################################
SaliencyMapConfigurator::
SaliencyMapConfigurator(OptionManager& mgr,
                        const std::string& descrName,
                        const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsSMtype(&OPT_SaliencyMapType, this),
  itsSM(new SaliencyMapStub(mgr))
{
GVX_TRACE(__PRETTY_FUNCTION__);
  addSubComponent(itsSM);
}

// ######################################################################
SaliencyMapConfigurator::~SaliencyMapConfigurator()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
nub::ref<SaliencyMap> SaliencyMapConfigurator::getSM() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsSM;
}

// ######################################################################
void SaliencyMapConfigurator::paramChanged(ModelParamBase* const param,
                                           const bool valueChanged,
                                           ParamClient::ChangeStatus* status)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ModelComponent::paramChanged(param, valueChanged, status);

  // was that a change of our baby's name?
  if (param == &itsSMtype) {
    // if we had one, let's unregister it (when we later reset() the
    // nub::ref, the current SaliencyMap will unexport its
    // command-line options):
    removeSubComponent(*itsSM);

    // instantiate a SM of the appropriate type:
    if (itsSMtype.getVal().compare("None") == 0 ||
        itsSMtype.getVal().compare("Stub") == 0) // no SM
      itsSM.reset(new SaliencyMapStub(getManager()));
    else if (itsSMtype.getVal().compare("Std") == 0)      // standard
      itsSM.reset(new SaliencyMapStd(getManager()));
    else if (itsSMtype.getVal().compare("StdOptim") == 0) // optimized standard
      itsSM.reset(new SaliencyMapStdOptim(getManager()));
    else if (itsSMtype.getVal().compare("Trivial") == 0)     // trivial
      itsSM.reset(new SaliencyMapTrivial(getManager()));
    else if (itsSMtype.getVal().compare("Fast") == 0)     // fast
      itsSM.reset(new SaliencyMapFast(getManager()));
    else
      LFATAL("Unknown SM type %s", itsSMtype.getVal().c_str());

    // add our baby as a subcomponent of us so that it will become
    // linked to the manager through us (hopefully we are registered
    // with the manager), which in turn will allow it to export its
    // command-line options and get configured:

    addSubComponent(itsSM);

    // tell the controller to export its options:
    itsSM->exportOptions(MC_RECURSE);

    // some info message:
    LINFO("Selected SM of type %s", itsSMtype.getVal().c_str());
  }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_SALIENCYMAPCONFIGURATOR_C_DEFINED

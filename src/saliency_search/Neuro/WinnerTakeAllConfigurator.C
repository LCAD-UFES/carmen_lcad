/*!@file Neuro/WinnerTakeAllConfigurator.C WinnerTakeAll configurator (factory) for the --wta-type command-line option */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/WinnerTakeAllConfigurator.C $
// $Id: WinnerTakeAllConfigurator.C 10794 2009-02-08 06:21:09Z itti $
//

#ifndef NEURO_WINNERTAKEALLCONFIGURATOR_C_DEFINED
#define NEURO_WINNERTAKEALLCONFIGURATOR_C_DEFINED

#include "Neuro/WinnerTakeAllConfigurator.H"

#include "Neuro/NeuroOpts.H"
#include "Neuro/WinnerTakeAll.H"
#include "Neuro/WinnerTakeAllStdOptim.H"
#include "rutz/trace.h"

// ######################################################################
// ######################################################################
// ########## WinnerTakeAllConfigurator implementation
// ######################################################################
// ######################################################################
WinnerTakeAllConfigurator::
WinnerTakeAllConfigurator(OptionManager& mgr,
                          const std::string& descrName,
                          const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsWTAtype(&OPT_WinnerTakeAllType, this),
  itsWTA(new WinnerTakeAllStub(mgr))
{
GVX_TRACE(__PRETTY_FUNCTION__);
  addSubComponent(itsWTA);
}

// ######################################################################
WinnerTakeAllConfigurator::~WinnerTakeAllConfigurator()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
nub::ref<WinnerTakeAll> WinnerTakeAllConfigurator::getWTA() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsWTA;
}

// ######################################################################
void WinnerTakeAllConfigurator::paramChanged(ModelParamBase* const param,
                                             const bool valueChanged,
                                             ParamClient::ChangeStatus* status)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ModelComponent::paramChanged(param, valueChanged, status);

  // was that a change of our baby's name?
  if (param == &itsWTAtype) {
    // if we had one, let's unregister it (when we later reset() the
    // nub::ref, the current WinnerTakeAll will unexport its
    // command-line options):
    removeSubComponent(*itsWTA);

    // instantiate a WTA of the appropriate type:
    if (itsWTAtype.getVal().compare("None") == 0 ||
        itsWTAtype.getVal().compare("Stub") == 0)
      itsWTA.reset(new WinnerTakeAllStub(getManager()));
    else if (itsWTAtype.getVal().compare("Std") == 0)      // standard
      itsWTA.reset(new WinnerTakeAllStd(getManager()));
    else if (itsWTAtype.getVal().compare("StdOptim") == 0) // optimized std
      itsWTA.reset(new WinnerTakeAllStdOptim(getManager()));
    else if (itsWTAtype.getVal().compare("Fast") == 0)     // fast
      itsWTA.reset(new WinnerTakeAllFast(getManager()));
    else if (itsWTAtype.getVal().compare("Greedy") == 0)    // greedy
      itsWTA.reset(new WinnerTakeAllGreedy(getManager()));
    else if (itsWTAtype.getVal().compare("Notice") == 0)    // temporal Notice
      itsWTA.reset(new WinnerTakeAllTempNote(getManager()));
    else
      LFATAL("Unknown WTA type %s", itsWTAtype.getVal().c_str());

    // add our baby as a subcomponent of us so that it will become
    // linked to the manager through us (hopefully we are registered
    // with the manager), which in turn will allow it to export its
    // command-line options and get configured:

    addSubComponent(itsWTA);

    // tell the controller to export its options:
    itsWTA->exportOptions(MC_RECURSE);

    // some info message:
    LINFO("Selected WTA of type %s", itsWTAtype.getVal().c_str());
  }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_WINNERTAKEALLCONFIGURATOR_C_DEFINED

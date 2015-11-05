/*!@file Neuro/SimulationViewerConfigurator.C pick a SimulationViewer */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerConfigurator.C $
// $Id: SimulationViewerConfigurator.C 13657 2010-07-12 22:43:55Z beobot $
//

#include "Neuro/SimulationViewerConfigurator.H"
#include "Component/OptionManager.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/SimulationViewer.H"
#include "Neuro/SimulationViewerCompress.H"
#include "Neuro/SimulationViewerEyeMvt.H"
#include "Neuro/SimulationViewerEyeMvt2.H"
#include "Neuro/SimulationViewerEyeMvtNeuro.H"
#include "Neuro/SimulationViewerEyeRegion.H"
#include "Neuro/SimulationViewerEyeSim.H"
#include "Neuro/SimulationViewerStd.H"
#include "Neuro/SimulationViewerStub.H"
#include "Neuro/SimulationViewerSurpCont.H"
#include "Neuro/SimulationViewerNerdCam.H"
#include "Neuro/SimulationViewerStats.H"
#include "Neuro/SimulationViewerRecStats.H"
#include "Neuro/SimulationViewerHand.H"
#include "Neuro/SimulationViewerEyeHand.H"

// ######################################################################
// ######################################################################
// ######################################################################
SimulationViewerConfigurator::
SimulationViewerConfigurator(OptionManager& mgr,
                             const std::string& descrName,
                             const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsSVtype(&OPT_SimulationViewerType, this),
  itsSV(new SimulationViewerStub(mgr))
{
  addSubComponent(itsSV);
}

// ######################################################################
SimulationViewerConfigurator::~SimulationViewerConfigurator()
{  }

// ######################################################################
nub::ref<SimulationViewer> SimulationViewerConfigurator::getSV() const
{ return itsSV; }

// ######################################################################
void SimulationViewerConfigurator::
paramChanged(ModelParamBase* const param,
             const bool valueChanged,
             ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);

  // was that a change of our baby's name?
  if (param == &itsSVtype) {
    // if we had one, let's unregister it (when we later reset() the
    // nub::ref, the current SimulationViewer will unexport its
    // command-line options):
    removeSubComponent(*itsSV);

    // instantiate a SV of the appropriate type:
    if (itsSVtype.getVal().compare("None") == 0
        || itsSVtype.getVal().compare("Stub") == 0) // "stub" SimulationViewer
      itsSV.reset(new SimulationViewerStub(getManager()));
    else if (itsSVtype.getVal().compare("Std") == 0)      // standard
      itsSV.reset(new SimulationViewerStd(getManager()));
    else if (itsSVtype.getVal().compare("Compress") == 0) // compression
      itsSV.reset(new SimulationViewerCompress(getManager()));
    else if (itsSVtype.getVal().compare("EyeMvt") == 0) // eye movements
      itsSV.reset(new SimulationViewerEyeMvt(getManager()));
    else if (itsSVtype.getVal().compare("EyeMvt2") == 0) // eye movements v 2.0
      itsSV.reset(new SimulationViewerEyeMvt2(getManager()));
    else if (itsSVtype.getVal().compare("EyeMvtNeuro") == 0) // eye mvt neuro
      itsSV.reset(new SimulationViewerEyeMvtNeuro(getManager()));
    else if (itsSVtype.getVal().compare("EyeRegion") == 0) // eye mvt region
      itsSV.reset(new SimulationViewerEyeRegion(getManager()));
    else if (itsSVtype.getVal().compare("EyeSim") == 0) // eye mvt simulation
      itsSV.reset(new SimulationViewerEyeSim(getManager()));
    else if (itsSVtype.getVal().compare("ASAC") == 0) // Surprise Control
      itsSV.reset(new SimulationViewerSurpCont(getManager()));
    else if (itsSVtype.getVal().compare("NerdCam") == 0) // Surprise Control
      itsSV.reset(new SimulationViewerNerdCam(getManager()));
    else if (itsSVtype.getVal().compare("Stats") == 0) // Stats
      itsSV.reset(new SimulationViewerStats(getManager()));
    else if (itsSVtype.getVal().compare("RecStats") == 0) // Stats
      itsSV.reset(new SimulationViewerRecStats(getManager()));
    else if (itsSVtype.getVal().compare("Hand") == 0) // Hand
      itsSV.reset(new SimulationViewerHand(getManager()));
    else if (itsSVtype.getVal().compare("EyeHand") == 0) // Hand
      itsSV.reset(new SimulationViewerEyeHand(getManager()));
    else
      LFATAL("Unknown SimulationViewer type %s", itsSVtype.getVal().c_str());

    // add our baby as a subcomponent of us so that it will become
    // linked to the manager through us (hopefully we are registered
    // with the manager), which in turn will allow it to export its
    // command-line options and get configured:

    addSubComponent(itsSV);

    // tell the SV to export its options:
    itsSV->exportOptions(MC_RECURSE);

    // some info message:
    LINFO("Selected SV of type %s", itsSVtype.getVal().c_str());
  }
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/*!@file Neuro/InferoTemporalConfigurator.C Class to select a InferoTemporal at
  runtime */

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
// Primary maintainer for this file: James Bonaiuto <bonaiuto@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/InferoTemporalConfigurator.C $
// $Id: InferoTemporalConfigurator.C 14286 2010-12-01 17:46:34Z sophie $
//

#include "Neuro/InferoTemporalConfigurator.H"
#include "Component/OptionManager.H"
#include "Neuro/InferoTemporal.H"
#include "Neuro/InferoTemporalSalBayes.H"
#include "Neuro/InferoTemporalHmax.H"
#include "Neuro/InferoTemporalSIFT.H"
#ifdef INVT_USE_CUDA
#include "Neuro/InferoTemporalCudaHmax.H"
#endif
#include "Neuro/NeuroOpts.H"


// ######################################################################
InferoTemporalConfigurator::
InferoTemporalConfigurator(OptionManager& mgr,
                           const std::string& descrName,
                           const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsITtype(&OPT_InferoTemporalType, this),
  itsIT(new InferoTemporalStub(mgr))
{
  addSubComponent(itsIT);
}

// ######################################################################
InferoTemporalConfigurator::~InferoTemporalConfigurator()
{  }

// ######################################################################
nub::ref<InferoTemporal> InferoTemporalConfigurator::getIT() const
{ return itsIT; }

// ######################################################################
void InferoTemporalConfigurator::
paramChanged(ModelParamBase* const param,
             const bool valueChanged,
             ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);

  // was that a change of our baby's name?
  if (param == &itsITtype) {
    LINFO("Configuring IT of type %s", itsITtype.getVal().c_str());

    // if we had one, let's unregister it:
    LINFO("Resetting InferoTemporal...");
    removeSubComponent(*itsIT);

    // instantiate a IT of the appropriate type:

    // #################### No inferior temporal cortex:
    if (itsITtype.getVal().compare("None") == 0 ||
        itsITtype.getVal().compare("Stub") == 0)
      itsIT.reset(new InferoTemporalStub(getManager()));

    // #################### Standard inferior temporal cortex:
    else if (itsITtype.getVal().compare("Std") == 0)
      itsIT.reset(new InferoTemporalStd(getManager(),
                                        "Inferior Temporal Cortex",
                                        "InferoTemporal"));
    // #################### SalBayes inferior temporal cortex:
    else if (itsITtype.getVal().compare("SalBayes") == 0)
      itsIT.reset(new InferoTemporalSalBayes(getManager(),
                                        "Inferior Temporal Cortex",
                                        "InferoTemporal"));
    // #################### SIFT inferior temporal cortex:
    else if (itsITtype.getVal().compare("SIFT") == 0)
      itsIT.reset(new InferoTemporalSIFT(getManager(),
                                        "Inferior Temporal Cortex",
                                        "InferoTemporal"));
    // #################### Hmax inferior temporal cortex:
    else if (itsITtype.getVal().compare("HMAX") == 0)
      itsIT.reset(new InferoTemporalHmax(getManager(),
                                        "Inferior Temporal Cortex",
                                        "InferoTemporal"));
#ifdef INVT_USE_CUDA
    // #################### CUDA Hmax inferior temporal cortex:
    else if (itsITtype.getVal().compare("CUDAHMAX") == 0)
      itsIT.reset(new InferoTemporalCudaHmax(getManager(),
                                        "Inferior Temporal Cortex",
                                        "InferoTemporal"));
#endif
    // #################### Custom-defined:
    else
      itsIT.reset(new InferoTemporalStd(getManager(),
                                        "Inferior Temporal Cortex",
                                        "InferoTemporal"));

    // add our babies as a subcomponents of us so that they will
    // become linked to the manager through us (hopefully we are
    // registered with the manager), which in turn will allow them to
    // export their command-line options and get configured.

    addSubComponent(itsIT);

    // tell the controller to export its options:
    itsIT->exportOptions(MC_RECURSE);
  }
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

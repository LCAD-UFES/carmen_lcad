/*!@file Neuro/GistEstimatorConfigurator.C
  future expansions for run-time options                                */

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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/GistEstimatorConfigurator.C $
// $Id: GistEstimatorConfigurator.C 10535 2008-12-17 03:37:06Z mviswana $
//

#include "Neuro/GistEstimatorConfigurator.H"
#include "Component/OptionManager.H"
#include "Neuro/GistEstimatorStd.H"
#include "Neuro/GistEstimatorGen.H"
#include "Neuro/GistEstimatorFFT.H"
#include "Neuro/GistEstimatorTexton.H"
#include "Neuro/GistEstimatorBeyondBoF.H"
#include "Neuro/GistEstimatorContextBased.H"
#include "Neuro/GistEstimatorSurfPMK.H"
#include "Neuro/GistEstimatorStub.H"
#include "Neuro/NeuroOpts.H"

// ######################################################################
GistEstimatorConfigurator::
GistEstimatorConfigurator(OptionManager& mgr,
                          const std::string& descrName,
                          const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsGEtype(&OPT_GistEstimatorType, this),
  itsGE(new GistEstimatorStub(mgr))
{
  addSubComponent(itsGE);
}

// ######################################################################
GistEstimatorConfigurator::~GistEstimatorConfigurator()
{  }

// ######################################################################
nub::ref<GistEstimator> GistEstimatorConfigurator::getGE() const
{ return itsGE; }

// ######################################################################
void GistEstimatorConfigurator::paramChanged(ModelParamBase* const param,
                                             const bool valueChanged,
                                             ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);

  // was that a change of our baby's name?
  if (param == &itsGEtype) {
    // if we had one, let's unregister it (when we later reset() the
    // nub::ref, the current GistEstimator will unexport its
    // command-line options):
    removeSubComponent(*itsGE);

    // instantiate a controller of the appropriate type:
    if (itsGEtype.getVal().compare("None") == 0 ||
        itsGEtype.getVal().compare("Stub") == 0)
      itsGE.reset(new GistEstimatorStub(getManager()));
    else if (itsGEtype.getVal().compare("Std") == 0)
      itsGE.reset(new GistEstimatorStd(getManager()));
#ifdef HAVE_FFTW3_H
    else if (itsGEtype.getVal().compare("FFT") == 0)
      itsGE.reset(new GistEstimatorFFT(getManager()));
#endif
    else if (itsGEtype.getVal().compare("Texton") == 0)
      itsGE.reset(new GistEstimatorTexton(getManager()));
    else if (itsGEtype.getVal().compare("BeyondBoF") == 0 ||
             itsGEtype.getVal().compare("BBoF") == 0 ||
             itsGEtype.getVal().compare("bbof") == 0)
      itsGE.reset(new GistEstimatorBeyondBoF(getManager()));
    else if (itsGEtype.getVal().compare("SurfPMK") == 0 ||
             itsGEtype.getVal().compare("Surf") == 0 ||
             itsGEtype.getVal().compare("surf") == 0)
      itsGE.reset(new GistEstimatorSurfPMK(getManager()));
    else if (itsGEtype.getVal().compare("ContextBased") == 0 ||
             itsGEtype.getVal().compare("CB") == 0)
      itsGE.reset(new GistEstimatorContextBased(getManager()));
    else if (itsGEtype.getVal().compare("Gen") == 0)
      itsGE.reset(new GistEstimatorGen(getManager()));
    else
      LFATAL("Unknown GistEstimator type %s", itsGEtype.getVal().c_str());

    // add our baby as a subcomponent of us so that it will become
    // linked to the manager through us (hopefully we are registered
    // with the manager), which in turn will allow it to export its
    // command-line options and get configured:

    addSubComponent(itsGE);

    // tell it to export its options:
    itsGE->exportOptions(MC_RECURSE);

    // some info message:
    LINFO("Selected GE of type %s", itsGEtype.getVal().c_str());
  }
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

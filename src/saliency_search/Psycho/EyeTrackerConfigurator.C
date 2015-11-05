/*!@file Psycho/EyeTrackerConfigurator.C Select an EyeTracker type */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/EyeTrackerConfigurator.C $
// $Id: EyeTrackerConfigurator.C 12686 2010-01-27 21:48:44Z beobot $
//

#ifndef PSYCHO_EYETRACKERCONFIGURATOR_C_DEFINED
#define PSYCHO_EYETRACKERCONFIGURATOR_C_DEFINED

#include "Psycho/EyeTrackerConfigurator.H"
#include "Psycho/EyeTracker.H"
#include "Psycho/EyeTrackerISCAN.H"
#include "Psycho/EyeTrackerDML.H"
#include "Psycho/EyeTrackerUDP.H"
#include "Psycho/EyeTrackerStub.H"
#include "Psycho/EyeTrackerTIL.H"
#include "Psycho/EyeTrackerEyeLink.H"
#include "Component/OptionManager.H"
#include "Psycho/PsychoOpts.H"

// ######################################################################
EyeTrackerConfigurator::
EyeTrackerConfigurator(OptionManager& mgr,
                       const std::string& descrName,
                       const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsETtype(&OPT_EyeTrackerType, this),
  itsET(new EyeTrackerStub(mgr))
{
  this->addSubComponent(itsET);
}

// ######################################################################
EyeTrackerConfigurator::~EyeTrackerConfigurator()
{  }

// ######################################################################
nub::ref<EyeTracker> EyeTrackerConfigurator::getET() const
{ return itsET; }

// ######################################################################
void EyeTrackerConfigurator::paramChanged(ModelParamBase* const param,
                                          const bool valueChanged,
                                          ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);

  // was that a change of our baby's name?
  if (param == &itsETtype) {
    // some info message:
    LINFO("Configuring ET of type %s", itsETtype.getVal().c_str());

    // let's unregister our current EyeTracker (when we later reset()
    // the nub::ref, the current EyeTracker will unexport its
    // command-line options):
    LINFO("Resetting EyeTracker...");
    removeSubComponent(*itsET);

    // instantiate an EyeTracker of the appropriate type:

    // #################### No eye-tracker:
    if (itsETtype.getVal().compare("None") == 0 ||
        itsETtype.getVal().compare("Stub") == 0)
      itsET.reset(new EyeTrackerStub(getManager()));

    // #################### ISCAN:
    else if (itsETtype.getVal().compare("ISCAN") == 0)
      itsET.reset(new EyeTrackerISCAN(getManager()));

    // #################### DML (Doug Munoz Lab):
    else if (itsETtype.getVal().compare("DML") == 0)
      itsET.reset(new EyeTrackerDML(getManager()));

    // #################### DML (Doug Munoz Lab):
    else if (itsETtype.getVal().compare("UDP") == 0)
      itsET.reset(new EyeTrackerUDP(getManager()));

    // #################### TIL (Tadashi Isa Lab):
    else if (itsETtype.getVal().compare("TIL") == 0)
      itsET.reset(new EyeTrackerTIL(getManager()));

    // #################### EyeLink-II:
    else if (itsETtype.getVal().compare("EL") == 0)
      itsET.reset(new EyeTrackerEyeLink(getManager()));

    // #################### Unknown:
    else LFATAL("Unknown EyeTracker type: %s", itsETtype.getVal().c_str());

    // add our baby as a subcomponents of us so that it will
    // become linked to the manager through us (hopefully we are
    // registered with the manager), which in turn will allow it to
    // export command-line options and get configured:
    addSubComponent(itsET);

    // export options:
    itsET->exportOptions(MC_RECURSE);
  }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_EYETRACKERCONFIGURATOR_C_DEFINED

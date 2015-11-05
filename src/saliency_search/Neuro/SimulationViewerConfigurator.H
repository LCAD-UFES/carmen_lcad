/*!@file Neuro/SimulationViewerConfigurator.H pick a SimulationViewer */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerConfigurator.H $
// $Id: SimulationViewerConfigurator.H 8160 2007-03-21 21:34:16Z rjpeters $
//

#ifndef SIMULATIONVIEWERCONFIGURATOR_H_DEFINED
#define SIMULATIONVIEWERCONFIGURATOR_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

class SimulationViewer;

//! SimulationViewer configurator
/*! This will export the --sv-type=XX command-line option and will
  instantiate a SimulationViewer of the desired type as the option gets
  assigned a value. As this happens, new options may become available
  in the command-line. To see them, use --help AFTER you have chosen
  the type to use. The current SimulationViewer may be retrieved using
  getSV(). */
class SimulationViewerConfigurator : public ModelComponent
{
public:
  //! Constructor
  SimulationViewerConfigurator(OptionManager& mgr,
                           const std::string& descrName =
                           "Simulation Viewer Configurator",
                           const std::string& tagName =
                           "SimulationViewerConfigurator");

  //! destructor
  virtual ~SimulationViewerConfigurator();

  //! Get the chosen SimulationViewer
  /*! You should call this during start() of the ModelComponent that
      needs the SimulationViewer. The result will never be null (since
      it's a nub::ref, not a nub::soft_ref), but it may be a "stub"
      type (e.g. SimulationViewerStub). */
  nub::ref<SimulationViewer> getSV() const;

protected:
  OModelParam<std::string> itsSVtype; //!< type of viewer

  //! Intercept people changing our ModelParam
  /*! See ModelComponent.H; as parsing the command-line or reading a
    config file sets our name, we'll also here instantiate a
    viewer of the proper type (and export its options) */
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

private:
  nub::ref<SimulationViewer> itsSV; // the viewer
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

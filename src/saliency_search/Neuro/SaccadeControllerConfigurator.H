/*!@file Neuro/SaccadeControllerConfigurator.H Pick a saccade controller */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SaccadeControllerConfigurator.H $
// $Id: SaccadeControllerConfigurator.H 8160 2007-03-21 21:34:16Z rjpeters $
//

#ifndef NEURO_SACCADECONTROLLERCONFIGURATOR_H_DEFINED
#define NEURO_SACCADECONTROLLERCONFIGURATOR_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
class SaccadeController;

// ######################################################################
//! SaccadeController configurator for eye
/*! This will export the --esc-type=XX command-line option and will
  instantiate a SaccadeController of the desired type as the option
  gets assigned a value. As this happens, new options may become
  available in the command-line. To see them, use --help AFTER you
  have chosen the name of the controller to use. The current
  eye SaccadeController may be retrieved using getSC(). */
class SaccadeControllerEyeConfigurator : public ModelComponent
{
public:
  //! Constructor
  SaccadeControllerEyeConfigurator(OptionManager& mgr,
                                   const std::string& descrName =
                                   "Saccade Controller Eye Configurator",
                                   const std::string& tagName =
                                   "SaccadeControllerEyeConfigurator");

  //! destructor
  virtual ~SaccadeControllerEyeConfigurator();

  //! Get the chosen SaccadeController
  /*! You should call this during start() of the ModelComponent that
    needs the SaccadeController. CAUTION: this may be a
    rutz::shared_ptr(NULL) if no controller has been chosen. */
  nub::ref<SaccadeController> getSC() const;

  //! Intercept people changing our itsSacCtrlName ModelParam
  /*! See ModelComponent.H; as parsing the command-line or reading a
    config file sets our name, we'll also here instantiate a
    controller of the proper type (and export its options) */
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

protected:
  OModelParam<std::string> itsSacCtrlType; //!< Type of the controller

private:
  nub::ref<SaccadeController> itsSC; // the controller
};

// ######################################################################
//! SaccadeController configurator for head
/*! This will export the --hsc-type=XX command-line option and will
  instantiate a SaccadeController of the desired type as the option
  gets assigned a value. As this happens, new options may become
  available in the command-line. To see them, use --help AFTER you
  have chosen the name of the controller to use. The current
  head SaccadeController may be retrieved using getSC(). */
class SaccadeControllerHeadConfigurator : public ModelComponent
{
public:
  //! Constructor
  SaccadeControllerHeadConfigurator(OptionManager& mgr,
                                    const std::string& descrName =
                                    "Saccade Controller Head Configurator",
                                    const std::string& tagName =
                                    "SaccadeControllerHeadConfigurator");

  //! destructor
  virtual ~SaccadeControllerHeadConfigurator();

  //! Get the chosen SaccadeController
  /*! You should call this during start() of the ModelComponent that
    needs the SaccadeController. CAUTION: this may be a
    rutz::shared_ptr(NULL) if no controller has been chosen. */
  nub::ref<SaccadeController> getSC() const;

  //! Intercept people changing our itsSacCtrlName ModelParam
  /*! See ModelComponent.H; as parsing the command-line or reading a
    config file sets our name, we'll also here instantiate a
    controller of the proper type (and export its options) */
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

protected:
  OModelParam<std::string> itsSacCtrlType; //!< Type of the controller

private:
  nub::ref<SaccadeController> itsSC; // the controller
};



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_SACCADECONTROLLERCONFIGURATOR_H_DEFINED

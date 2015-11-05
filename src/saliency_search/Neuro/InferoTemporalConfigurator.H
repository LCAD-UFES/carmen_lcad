/*!@file Neuro/InferoTemporalConfigurator.H run-time selects which
  VisualCoxter to use */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/InferoTemporalConfigurator.H $
// $Id: InferoTemporalConfigurator.H 8160 2007-03-21 21:34:16Z rjpeters $
//

#ifndef INFEROTEMPORALCONFIGURATOR_H_DEFINED
#define INFEROTEMPORALCONFIGURATOR_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

class InferoTemporal;

//! InferoTemporal configurator
/*! This will export the --vc-type=XX command-line option and will
  instantiate a InferoTemporal of the desired type as the option gets
  assigned a value. As this happens, new options may become available
  in the command-line. To see them, use --help AFTER you have chosen
  the type to use. The current InferoTemporal may be retrieved using
  getIT(). */
class InferoTemporalConfigurator : public ModelComponent
{
public:
  //! Constructor
  InferoTemporalConfigurator(OptionManager& mgr,
                           const std::string& descrName =
                           "Inferior Temporal Cortex Configurator",
                           const std::string& tagName =
                           "InferoTemporalConfigurator");

  //! destructor
  virtual ~InferoTemporalConfigurator();

  //! Get the chosen InferoTemporal
  /*! You should call this during start() of the ModelComponent that
    needs the InferoTemporal. */
  nub::ref<InferoTemporal> getIT() const;

protected:
  OModelParam<std::string> itsITtype; //!< type of inferior temporal cortex

  //! Intercept people changing our ModelParam
  /*! See ModelComponent.H; as parsing the command-line or reading a
    config file sets our name, we'll also here instantiate a
    controller of the proper type (and export its options) */
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

private:
  nub::ref<InferoTemporal> itsIT; // the inferior temporal cortex
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

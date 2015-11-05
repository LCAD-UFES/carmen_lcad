/*!@file Neuro/VisualCortexConfigurator.H run-time selects which
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/VisualCortexConfigurator.H $
// $Id: VisualCortexConfigurator.H 8160 2007-03-21 21:34:16Z rjpeters $
//

#ifndef VISUALCORTEXCONFIGURATOR_H_DEFINED
#define VISUALCORTEXCONFIGURATOR_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

class VisualCortex;
class Beowulf;

//! VisualCortex configurator
/*! This will export the --vc-type=XX command-line option and will
  instantiate a VisualCortex of the desired type as the option gets
  assigned a value. As this happens, new options may become available
  in the command-line. To see them, use --help AFTER you have chosen
  the type to use. The current VisualCortex may be retrieved using
  getVC(). */
class VisualCortexConfigurator : public ModelComponent
{
public:
  //! Constructor
  VisualCortexConfigurator(OptionManager& mgr,
                           const std::string& descrName =
                           "Visual Cortex Configurator",
                           const std::string& tagName =
                           "VisualCortexConfigurator");

  //! destructor
  virtual ~VisualCortexConfigurator();

  //! Intercept people changing our ModelParam
  /*! See ModelComponent.H; as parsing the command-line or reading a
    config file sets our name, we'll also here instantiate a
    controller of the proper type (and export its options) */
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

  //! Get the chosen VisualCortex
  /*! You should call this during start() of the ModelComponent that
    needs the VisualCortex. CAUTION: this may be a
    rutz::shared_ptr(NULL) if no controller has been chosen. */
  nub::ref<VisualCortex> getVC() const;

  //! If the chosen VisualCortex uses a Beowulf, get it here
  /*! You should call this during start() of the ModelComponent that
    needs the VisualCortex. CAUTION: this may be a
    rutz::shared_ptr(NULL) if no controller has been chosen. */
  nub::soft_ref<Beowulf> getBeo() const;

protected:
  OModelParam<std::string> itsVCtype; //!< type of visual cortex

private:
  nub::ref<VisualCortex> itsVC; // the visual cortex
  nub::soft_ref<Beowulf> itsBeo;     // we may also need a Beowulf
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

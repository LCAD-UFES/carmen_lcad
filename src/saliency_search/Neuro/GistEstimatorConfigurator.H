/*!@file Neuro/GistEstimatorConfigurator.H
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/GistEstimatorConfigurator.H $
// $Id: GistEstimatorConfigurator.H 8160 2007-03-21 21:34:16Z rjpeters $
//

#ifndef GISTESTIMATORCONFIGURATOR_H_DEFINED
#define GISTESTIMATORCONFIGURATOR_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

class GistEstimator;

//! GistEstimator configurator
/*! This will export the --ge-type=XX command-line option and will
  instantiate a GistEstimator of the desired type as the option gets
  assigned a value. As this happens, new options may become available
  in the command-line. To see them, use --help AFTER you have chosen
  the type to use. The current Gist Estimator may be retrieved using
  getGE(). */
class GistEstimatorConfigurator : public ModelComponent
{
public:
  //! Constructor
  GistEstimatorConfigurator(OptionManager& mgr,
                            const std::string& descrName =
                            "Gist Estimator Configurator",
                            const std::string& tagName =
                            "GistEstimatorConfigurator");

  //! destructor
  virtual ~GistEstimatorConfigurator();

  //! Get the chosen GistEstimator
  /*! You should call this during start() of the ModelComponent that
      needs the GistEstimator. NOTE: the result will never be null
      (since it's a nub::ref rather than a nub::soft_ref), but the
      result might be a "stub" object (e.g. GistEstimatorStub)
      designed to gracefully do nothing. */
  nub::ref<GistEstimator> getGE() const;

protected:
  OModelParam<std::string> itsGEtype; //!< type of Gist Estimator

  //! Intercept people changing our ModelParam
  /*! See ModelComponent.H; as parsing the command-line or reading a
    config file sets our name, we'll also here instantiate a
    controller of the proper type (and export its options) */
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

private:
  nub::ref<GistEstimator> itsGE; // the Gist Estimator
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/*!@file Neuro/PrefrontalCortex.H simulation of a human PrefrontalCortex */

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
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/PrefrontalCortex.H $
// $Id: PrefrontalCortex.H 11086 2009-04-03 03:35:14Z lior $
//


#ifndef PrefrontalCortex_H_DEFINED
#define PrefrontalCortex_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Component/ParamMap.H"
#include "Media/MediaSimEvents.H"
#include "Neuro/NeuroSimEvents.H"
#include "Simulation/SimModule.H"

class ModelManager;
class Bayes;
class BayesianBiaser;

// ######################################################################
//! The PrefrontalCortex base class
/*! The PrefrontalCortex is the entry point to the Brain and VisualCortex,
  for now it is only used to load the task-relevant data for biasing. */
class PrefrontalCortex : public SimModule
{
public:
  //! Constructor
  /*! See ModelComponent.H for details */
  PrefrontalCortex(OptionManager& mgr,
                   const std::string& descrName = "PrefrontalCortex",
                   const std::string& tagName = "PrefrontalCortex");

  //! Destructor
  virtual ~PrefrontalCortex();
};


// ######################################################################
//! PrefrontalCortex configurator
// ######################################################################
/*! This will export the --PrefrontalCortex-type=XX command-line
  option and will instantiate a PrefrontalCortex of the desired type
  as the option gets assigned a value. As this happens, new options
  may become available in the command-line. To see them, use --help
  AFTER you have chosen the type to use. The current PrefrontalCortex
  may be retrieved using getPFC(). */
class PrefrontalCortexConfigurator : public ModelComponent
{
public:
  //! Constructor
  PrefrontalCortexConfigurator(OptionManager& mgr,
                     const std::string& descrName =
                               "PrefrontalCortex Configurator",
                     const std::string& tagName =
                               "PrefrontalCortexConfigurator");

  //! destructor
  virtual ~PrefrontalCortexConfigurator();

  //! Get the chosen PrefrontalCortex
  /*! You should call this during start() of the ModelComponent that
      needs the PrefrontalCortex. This is guaranteed to be a non-null object
      (though it may be "stub" type, e.g. PrefrontalCortexStub). */
  nub::ref<PrefrontalCortex> getPFC() const;

protected:
  OModelParam<std::string> itsType; //!< type of PrefrontalCortex

  //! Intercept people changing our ModelParam
  /*! See ModelComponent.H; as parsing the command-line or reading a
    config file sets our name, we'll also here instantiate a
    PrefrontalCortex of the proper type (and export its options) */
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

private:
  nub::ref<PrefrontalCortex> itsPFC; // the PrefrontalCortex
};

// ######################################################################
//! "Stub" PrefrontalCortex implementation
// ######################################################################
/*! Does nothing but copy its input to its output. */
class PrefrontalCortexStub : public PrefrontalCortex
{
public:
  //! Constructor
  PrefrontalCortexStub(OptionManager& mgr,
             const std::string& descrName = "PrefrontalCortexStub",
             const std::string& tagName = "PrefrontalCortexStub");

  //! Destructor
  virtual ~PrefrontalCortexStub();
};

// ######################################################################
//! OptimalGains PrefrontalCortex implementation
// ######################################################################
/*! This PFC computes the salience of targets and distractors and
  saves that info to a text file. Those text files can later be
  combined and used to compute optimal gains, per Navalpakkam & Itti,
  IEEE CVPR, 2006. The resulting optimal gains can finally be used to
  bias saliency computations, using a PrefrontalCortexGS. */
class PrefrontalCortexOG : public PrefrontalCortex
{
public:
  //! Constructor
  /*! See ModelComponent.H for details */
  PrefrontalCortexOG(OptionManager& mgr,
                     const std::string& descrName = "PrefrontalCortexOG",
                     const std::string& tagName = "PrefrontalCortexOG");

  //! Destructor
  virtual ~PrefrontalCortexOG();

protected:
  //! Callback for when a new input frame is available
  SIMCALLBACK_DECLARE(PrefrontalCortexOG, SimEventInputFrame);

  //! Callback for when a new vcx output is ready
  SIMCALLBACK_DECLARE(PrefrontalCortexOG, SimEventVisualCortexOutput);

  OModelParam<std::string> itsFilename; //!< file to save sT and sD values
  OModelParam<std::string> itsTargetMaskObjName; //!< target mask obj name
  OModelParam<std::string> itsDistractorMaskObjName; //!< distractor mask obj name
  OModelParam<bool> itsDoMax;
  Image<byte> itsTargetMask, itsDistractorMask;
};

// ######################################################################
//! GuidedSearch PrefrontalCortex implementation
// ######################################################################
/*! This PFC loads up a set of gains and applies them to the submaps
  and channels, thereby implementing Jeremy Wolfe's Guided Search
  (Wolfe, 1994). */
class PrefrontalCortexGS : public PrefrontalCortex
{
public:
  //! Constructor
  /*! See ModelComponent.H for details */
  PrefrontalCortexGS(OptionManager& mgr,
                     const std::string& descrName = "PrefrontalCortexGS",
                     const std::string& tagName = "PrefrontalCortexGS");

  //! Destructor
  virtual ~PrefrontalCortexGS();

protected:
  //! Callback for when a new input frame is available
  SIMCALLBACK_DECLARE(PrefrontalCortexGS, SimEventInputFrame);

  OModelParam<std::string> itsFilename; //!< file to save sT and sD values
};

// ######################################################################
//! SalBayes PrefrontalCortex implementation
// ######################################################################
class PrefrontalCortexSB : public PrefrontalCortex
{
public:
  //! Constructor
  /*! See ModelComponent.H for details */
  PrefrontalCortexSB(OptionManager& mgr,
                           const std::string& descrName =
                           "PrefrontalCortexSB",
                           const std::string& tagName =
                           "PrefrontalCortexSB");

  //! Destructor
  virtual ~PrefrontalCortexSB();

protected:
  //! Callback for when a new input frame is available
  SIMCALLBACK_DECLARE(PrefrontalCortexSB, SimEventInputFrame);

  OModelParam<std::string> itsBayesNetFilename; //!< The object database
  OModelParam<int> itsObjToBias; //!< the object to bias for from the database

  //! get started
  virtual void start1();

private:
  rutz::shared_ptr<Bayes> itsBayesNet;
};


#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

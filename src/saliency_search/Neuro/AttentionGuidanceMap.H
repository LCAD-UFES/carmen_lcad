/*!@file Neuro/AttentionGuidanceMap.H Class declarations for attentional guidance map class */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/AttentionGuidanceMap.H $
// $Id: AttentionGuidanceMap.H 15310 2012-06-01 02:29:24Z itti $
//

#ifndef ATTENTIONGUIDANCEMAP_H_DEFINED
#define ATTENTIONGUIDANCEMAP_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Image.H"
#include "Image/Range.H"
#include "Neuro/NeuroSimEvents.H"
#include "Simulation/SimModule.H"
#include "Simulation/SimEvents.H"

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work
#include "ModelNeuron/SupColliculusModule.H"
#include "ModelNeuron/NeuralFieldModule.H"
#endif

class FrameOstream;
class ModelManager;

// ######################################################################
//! The attentional guidance  map base class
// ######################################################################
/*! This is a 2D attentional guidance, or priority map. It is just a
  base class with virtual function definitions. Various
  implementations are available below.  The AGM is a topographic map
  that represents the priority of spatial locations. Priority is the
  combination of bottom-up or stimulus driven processing with top-down
  or goal directed processing. Top-down might also include
  higher-order elements of visual processing such as object
  recognition. Smaller values indicate that a location is of low
  priority while higher values indicate higher priority.*/
class AttentionGuidanceMap : public SimModule
{
public:
  // ######################################################################
  //! @name Constructor, destructor, and reset
  //@{

  //! Ininitialized constructor
  /*! The map will be resized and initialized the first time input() is
    called */
  AttentionGuidanceMap(OptionManager& mgr,
                   const std::string& descrName = "Attention Guidance Map",
                   const std::string& tagName = "AttentionGuidanceMap");

  //! Destructor
  virtual ~AttentionGuidanceMap();

  //! Reset to initial state just after construction
  virtual void reset() = 0;

  //@}

protected:
  //! Callback for when a new saliency map is available
  SIMCALLBACK_DECLARE(AttentionGuidanceMap, SimEventSaliencyMapOutput);

  //! Callback for when a new task relevance map is available
  SIMCALLBACK_DECLARE(AttentionGuidanceMap, SimEventTaskRelevanceMapOutput);

  //! Callback on every clock tick
  SIMCALLBACK_DECLARE(AttentionGuidanceMap, SimEventClockTick);

  //! Callback for every time we should save our outputs
  SIMCALLBACK_DECLARE(AttentionGuidanceMap, SimEventSaveOutput);

  //! Save our internals when saveResults() is called?
  OModelParam<bool> itsSaveResults;

  //! Set new bottom-up input
  /*! This should initialize and resize the map if it is currently
      uninitialized (e.g., just after construction or reset()). */
  virtual void inputBU(const Image<float>& in) = 0;

  //! Set new top-down input
  /*! This should initialize and resize the map if it is currently
      uninitialized (e.g., just after construction or reset()). */
  virtual void inputTD(const Image<float>& in) = 0;

  //! Return all our relevance values as an Image<float>
  virtual Image<float> getV() const = 0;

  //!run on overy time step
  virtual void doClockTick(SimEventQueue& q);

  //! save results
  /*! A default implementation is provided, which just calls
      getV() and saves that map with a file name prefix of "AGM".
      @param ofs will figure out file name/format and save our
      results. */
  virtual void save1(const ModelComponentSaveInfo& sinfo);

private:
  Image<float> itsOutputCache;

  // forbid assignment and copy-construction:
  AttentionGuidanceMap& operator=(const AttentionGuidanceMap& sm);
  AttentionGuidanceMap(const AttentionGuidanceMap& sm);
};

// ######################################################################
//! AttentionGuidanceMap configurator
// ######################################################################
/*! This will export the --agm-type=XX command-line option and will
  instantiate an AGM of the desired type as the option gets assigned a
  value. As this happens, new options may become available in the
  command-line. To see them, use --help AFTER you have chosen the type
  to use. The current AGM may be retrieved using getAGM(). */
class AttentionGuidanceMapConfigurator : public ModelComponent
{
public:
  //! Constructor
  AttentionGuidanceMapConfigurator(OptionManager& mgr,
                               const std::string& descrName =
                               "Attention Guidance Map Configurator",
                               const std::string& tagName =
                               "AttentionGuidanceMapConfigurator");

  //! destructor
  virtual ~AttentionGuidanceMapConfigurator();

  //! Get the chosen AGM
  /*! You should call this during start() of the ModelComponent that
      needs the AGM.  */
  nub::ref<AttentionGuidanceMap> getAGM() const;

protected:
  OModelParam<std::string> itsAGMtype; //!< type of map

  //! Intercept people changing our ModelParam
  /*! See ModelComponent.H; as parsing the command-line or reading a
    config file sets our name, we'll also here instantiate a
    controller of the proper type (and export its options) */
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

private:
  nub::ref<AttentionGuidanceMap> itsAGM; // the map
};


// ######################################################################
//! The standard attentional guidance map
// ######################################################################
/*! This is our current standard AGM implementation. It just takes the
  pointwise product between bottom-up and top-down inputs. */
class AttentionGuidanceMapStd : public AttentionGuidanceMap
{
public:
  //! Uninitialized constructor
  AttentionGuidanceMapStd(OptionManager& mgr, const std::string& descrName =
                      "Task-Relevance Map Std",
                      const std::string& tagName =
                      "AttentionGuidanceMapStd");

  //! Destructor
  virtual ~AttentionGuidanceMapStd();

  //! Reset to initial state just after construction
  virtual void reset();

protected:
  //! Set new bottom-up input
  virtual void inputBU(const Image<float>& in);

  //! Set new top-down input
  virtual void inputTD(const Image<float>& in);

  //! Return all our values as an Image<float>
  virtual Image<float> getV() const;

private:
  Image<float> itsBUmap;
  Image<float> itsTDmap;
};

// ######################################################################
//! The Optimized Attention Guidance Map
// ######################################################################
/*! This is our current optimized standard AGM implementation. It just takes the
  a*BU + b*TD + BU*TD mode, and (a,b) are optimized coefficients */
class AttentionGuidanceMapOpt : public AttentionGuidanceMap
{
public:
  //! Uninitialized constructor
  AttentionGuidanceMapOpt(OptionManager& mgr, const std::string& descrName =
                          "Attention Guidance Map Optimized",
                          const std::string& tagName =
                          "AttentionGuidanceMapOpt");
  
  //! Destructor
  virtual ~AttentionGuidanceMapOpt();

  //! Reset to initial state just after construction
  virtual void reset();

protected:
  //! Set new bottom-up input
  virtual void inputBU(const Image<float>& in);

  //! Set new top-down input
  virtual void inputTD(const Image<float>& in);

  //! Return all our values as an Image<float>
  virtual Image<float> getV() const;

private:
  Image<float> itsBUmap;
  Image<float> itsTDmap;
};

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work
// ######################################################################
//! An Attentional Guidance Map to interface the ModelNeuron classes.
// This is just an adaptor class, see below for full implementations
// ######################################################################
class AttentionGuidanceMapNeuralSim : public AttentionGuidanceMap
{
public:
  AttentionGuidanceMapNeuralSim(OptionManager& mgr, const std::string& descrName, const std::string& tagName);
  virtual ~AttentionGuidanceMapNeuralSim();

protected:
  //!run on overy time step
  void doClockTick(SimEventQueue& q);
  
  //! post our output
  virtual void postMessage(SimEventQueue& q) = 0;

  //! update our internals
  virtual void update(const SimTime& time) = 0;
  
  //! our output rate
  OModelParam<SimTime> itsOutRate;
  
private:
  SimTime itsTime;//the current time
};

// ######################################################################
//! An Attentional Guidance Map based on the mammalian superior colliculus
// ######################################################################
class AttentionGuidanceMapSC : public AttentionGuidanceMapNeuralSim
{
public:
  //! Uninitialized constructor
  AttentionGuidanceMapSC(OptionManager& mgr, 
                         const std::string& descrName = "AttentionGuidance Map SC",
                         const std::string& tagName = "AttentionGuidanceMapSC");
  //! Destructor
  virtual ~AttentionGuidanceMapSC();

  //! Reset to initial state just after construction
  virtual void reset();
  
protected:

  //! Set new bottom-up input
  virtual void inputBU(const Image<float>& in);

  //! Set new top-down input
  virtual void inputTD(const Image<float>& in);

  //! Return all our relevance values as an Image<float>
  virtual Image<float> getV() const;

  //! update our internals
  virtual void update(const SimTime& time);

  //!run on overy time step
  virtual void postMessage(SimEventQueue& q);

  //! save results
  virtual void save1(const ModelComponentSaveInfo& sinfo);

  nub::ref<nsu::SupColliculusModule> itsSC;//our actual component
};

// ######################################################################
//! An Attentional Guidance Map based on neural fields
// ######################################################################
class AttentionGuidanceMapNF : public AttentionGuidanceMapNeuralSim
{
public:
  //! Uninitialized constructor
  AttentionGuidanceMapNF(OptionManager& mgr, 
                         const std::string& descrName = "AttentionGuidance Map SC",
                         const std::string& tagName = "AttentionGuidanceMapSC");
  //! Destructor
  virtual ~AttentionGuidanceMapNF();

  //! Reset to initial state just after construction
  virtual void reset();
  
protected:
  //! Set new bottom-up input
  virtual void inputBU(const Image<float>& in);

  //! Set new top-down input
  virtual void inputTD(const Image<float>& in);

  //! Return all our relevance values as an Image<float>
  virtual Image<float> getV() const;

  //! update our internals
  virtual void update(const SimTime& time);

  //!run on overy time step
  virtual void postMessage(SimEventQueue& q);

  //! save results
  virtual void save1(const ModelComponentSaveInfo& sinfo);

  nub::ref<NeuralFieldModule> itsNF;//our actual component
};

#endif//use cpp0x features
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

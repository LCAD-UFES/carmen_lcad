/*!@file Neuro/SaliencyMap.H Class declarations for saliency map class */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SaliencyMap.H $
// $Id: SaliencyMap.H 11113 2009-04-14 07:23:37Z itti $
//

#ifndef SALIENCYMAP_H_DEFINED
#define SALIENCYMAP_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Image.H"
#include "Image/LevelSpec.H"
#include "Neuro/IORtypes.H"
#include "Neuro/LeakyIntegrator.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/SpatialMetrics.H"
#include "Simulation/SimModule.H"
#include "Simulation/SimEvents.H"
#include "Util/SimTime.H"

class FrameOstream;
class ModelManager;
class SpatialMetrics;
class WTAwinner;

// ######################################################################
//! The saliency map base class
// ######################################################################
/*! This is a 2D salience map. It is just a base class with virtual
  function definitions. Various saliency maps are available below,
  which provide implementations for these virtual functions. The
  saliency map receives input through the input() function, and its
  current state (membrane potential of neurons) can be retrieved using
  getV(). Various inplementations will typically derive from class
  Image and inherit all the standard methods of class Image. */
class SaliencyMap : public SimModule
{
public:
  // ######################################################################
  //! @name Constructor, destructor, and reset
  //@{

  //! Ininitialized constructor
  SaliencyMap(OptionManager& mgr,
              const std::string& descrName = "Saliency Map",
              const std::string& tagName = "SaliencyMap");

  //! Destructor
  virtual ~SaliencyMap();

  //@}

private:
  // forbid assignment and copy-construction:
  SaliencyMap& operator=(const SaliencyMap& sm);
  SaliencyMap(const SaliencyMap& sm);
};

// ######################################################################
//! A do-nothing stub implementation of SaliencyMap
// ######################################################################
class SaliencyMapStub : public SaliencyMap
{
public:
  //! Constructor
  SaliencyMapStub(OptionManager& mgr,
                  const std::string& descrName = "Saliency Map Stub",
                  const std::string& tagName = "SaliencyMapStub");

  //! Destructor
  virtual ~SaliencyMapStub();
};

// ######################################################################
//! A partial implementation of SaliencyMap for further subclasses
// ######################################################################
class SaliencyMapAdapter : public SaliencyMap
{
public:
  //! Constructor
  SaliencyMapAdapter(OptionManager& mgr,
                     const std::string& descrName,
                     const std::string& tagName);

  //! Destructor
  virtual ~SaliencyMapAdapter();

protected:
  //! Callback for when a new WTA winner is available
  SIMCALLBACK_DECLARE(SaliencyMapAdapter, SimEventWTAwinner);

  //! Callback for when a new data from VisualCortex is available
  SIMCALLBACK_DECLARE(SaliencyMapAdapter, SimEventVisualCortexOutput);

  //! Callback for when a new eye activity is going on
  SIMCALLBACK_DECLARE(SaliencyMapAdapter, SimEventSaccadeStatusEye);

  //! Callback for every time we should save our outputs
  SIMCALLBACK_DECLARE(SaliencyMapAdapter, SimEventSaveOutput);

  //! metrics that depend on the input size:
  nub::ref<SpatialMetrics> itsMetrics;

  //! type of IOR to use. See IORtypes.H
  OModelParam<IORtype> itsIORtype;

  //! Specification of our image processing center-surround levels
  /*! The only thing that we care about here is the saliency map level */
  OModelParam<LevelSpec> itsLevelSpec;

  //! Save our internals?
  OModelParam<bool> itsSaveResults;

  //! Save our cumulative internals when saveResults() is called?
  OModelParam<bool> itsSaveCumResults;

  //! use saccadic suppression?
  OModelParam<bool> itsUseSaccadicSuppression;

  //! use blink suppression?
  OModelParam<bool> itsUseBlinkSuppression;

  /*! If the saliency map voltage at winning location gets higher than
    this (in mV), trigger some global inhibition */
  OModelParam<float> itsMaxWinMv;

  //! Cumulative map so far. Cumulates each time saveResults() is called
  /*! This is computed only if itsSaveCumResults() is true */
  Image<float> itsCumMap;

  //! Set new input currents for all neurons
  /*! This should initialize and resize the saliency map if the
    saliency map is currently uninitialized (e.g., just after
    construction or reset()). */
  virtual void input(SimEventQueue& q, const Image<float>& in) = 0;

  //! Difference-of-Gaussians inhibition of return
  /*! The positive Gaussian is defined by its amplitute pampl and
    width psdev; the negative Gaussian by mampl and msdev.  Both
    centered at winner.  */
  virtual void depress(SimEventQueue& q, const Point2D<int>& winner,
                       const double& pampl, const double& mampl,
                       const double& psdev, const double& msdev) = 0;

  //! Object-based inhibition of return
  /*! The object mask defines where in the image the inhibition will
    be applied. The winner coordinates and ampl are used to calibrate
    the inhibition strength */
  virtual void depress(SimEventQueue& q, const Point2D<int>& winner, const double& ampl,
                       const Image<byte>& objectMask) = 0;

  //! Turn saccadic suppression on/off
  virtual void saccadicSuppression(SimEventQueue& q, const bool on) = 0;

  //! Turn blink suppression on/off
  virtual void blinkSuppression(SimEventQueue& q, const bool on) = 0;

  //! Return all our membrane potential voltages as an Image<float>
  virtual Image<float> getV() const = 0;

  //! Access function using Point2D<int>
  virtual float getV(const Point2D<int>& p) const = 0;
};

// ######################################################################
//! The standard saliency map: a 2D array of LeakyIntegrator neurons
// ######################################################################
/*! This is a 2D salience map. It is an Image<LeakyIntegrator> and
  inherits all the standard methods of class Image. To avoid
  confusion, we here add explicit input() and getV() methods (rather
  than providing conversion functions between LeakyIntegrator and
  float, which could make the process more transparent but also
  possibly more confusing). */
class SaliencyMapStd : public SaliencyMapAdapter
{
public:
  //! Ininitialized constructor
  SaliencyMapStd(OptionManager& mgr,
                 const std::string& descrName = "Saliency Map Std",
                 const std::string& tagName = "SaliencyMapStd");

  //! Destructor
  virtual ~SaliencyMapStd();

protected:
  //! Callback for every clock tick, to run our diff equations in integrate()
  SIMCALLBACK_DECLARE(SaliencyMapStd, SimEventClockTick);

  OModelParam<float> itsGinhDecay; //!< decay for IOR inhibition

  //! Set new input currents for all neurons
  virtual void input(SimEventQueue& q, const Image<float>& in);

  //! Difference-of-Gaussians inhibition of return
  virtual void depress(SimEventQueue& q, const Point2D<int>& winner,
                       const double& pampl, const double& mampl,
                       const double& psdev, const double& msdev);

  //! Object-based inhibition of return
  virtual void depress(SimEventQueue& q, const Point2D<int>& winner, const double& ampl,
                       const Image<byte>& objectMask);

  //! Turn saccadic suppression on/off
  virtual void saccadicSuppression(SimEventQueue& q, const bool on);

  //! Turn blink suppression on/off
  virtual void blinkSuppression(SimEventQueue& q, const bool on);

  //! Return all our membrane potential voltages as an Image<float>
  virtual Image<float> getV() const;

  //! Access function using Point2D<int>
  virtual float getV(const Point2D<int>& p) const;

  //! Reset to initial state just after construction
  virtual void reset1();

private:
  Image<LeakyIntegrator> itsNeurons;
};

// ######################################################################
//! The trivial saliency map: a 2D array of floats
// ######################################################################
/*! This is a 2D salience map. It is an Image<float> and inherits all
  the standard methods of class Image. To avoid confusion, we here add
  explicit input() and getV() methods. */
class SaliencyMapTrivial : public SaliencyMapAdapter
{
public:
  //! Ininitialized constructor
  /*! The SM will be resized and initialized the first time input() is
    called */
  SaliencyMapTrivial(OptionManager& mgr,
                     const std::string& descrName = "Saliency Map Trivial",
                     const std::string& tagName = "SaliencyMapTrivial");

  //! Destructor
  virtual ~SaliencyMapTrivial();

protected:
  //! Coeff to go from current inputs to voltage outputs
  OModelParam<float> itsItoVcoeff;

  //! Set new input currents for all neurons
  virtual void input(SimEventQueue& q, const Image<float>& in);

  //! Difference-of-Gaussians inhibition of return
  virtual void depress(SimEventQueue& q, const Point2D<int>& winner,
                       const double& pampl, const double& mampl,
                       const double& psdev, const double& msdev);

  //! Object-based inhibition of return
  virtual void depress(SimEventQueue& q, const Point2D<int>& winner, const double& ampl,
                       const Image<byte>& objectMask);

  //! Turn saccadic suppression on/off
  virtual void saccadicSuppression(SimEventQueue& q, const bool on);

  //! Turn blink suppression on/off
  virtual void blinkSuppression(SimEventQueue& q, const bool on);

  //! Return all our membrane potential voltages as an Image<float>
  virtual Image<float> getV() const;

  //! Access function using Point2D<int>
  virtual float getV(const Point2D<int>& p) const;

  //! Reset to initial state just after construction
  virtual void reset1();

private:
  Image<float> itsNeurons;
};

// ######################################################################
//! The fast saliency map: a 2D array of floats
// ######################################################################
/*! This is a 2D salience map. It is an Image<float> and inherits all
  the standard methods of class Image. To avoid confusion, we here add
  explicit input() and getV() methods. */
class SaliencyMapFast : public SaliencyMapAdapter
{
public:
  //! Ininitialized constructor
  SaliencyMapFast(OptionManager& mgr,
                  const std::string& descrName = "Saliency Map Fast",
                  const std::string& tagName = "SaliencyMapFast");

  //! Destructor
  virtual ~SaliencyMapFast();

protected:
  //! Simulation time step, in seconds
  OModelParam<SimTime> itsTimeStep;

  //! Coeff by which new inputs replace the current SM at each time step (0..1)
  OModelParam<float> itsInputCoeff;

  //! Coeff to go from current inputs to voltage outputs
  OModelParam<float> itsItoVcoeff;

  //! Set new input currents for all neurons
  virtual void input(SimEventQueue& q, const Image<float>& in);

  //! Difference-of-Gaussians inhibition of return
  virtual void depress(SimEventQueue& q, const Point2D<int>& winner,
                       const double& pampl, const double& mampl,
                       const double& psdev, const double& msdev);

  //! Object-based inhibition of return
  virtual void depress(SimEventQueue& q, const Point2D<int>& winner, const double& ampl,
                       const Image<byte>& objectMask);

  //! Integrate our neurons for one time step
  virtual void integrate(SimEventQueue& q);

  //! Turn saccadic suppression on/off
  virtual void saccadicSuppression(SimEventQueue& q, const bool on);

  //! Turn blink suppression on/off
  virtual void blinkSuppression(SimEventQueue& q, const bool on);

  //! Return all our membrane potential voltages as an Image<float>
  virtual Image<float> getV() const;

  //! Access function using Point2D<int>
  virtual float getV(const Point2D<int>& p) const;

  //! Reset to initial state just after construction
  virtual void reset1();

private:
  Image<float> itsNeurons;
  Image<float> itsInputCopy;  // copy of inputs
  SimTime itsT;               // keep track of time
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

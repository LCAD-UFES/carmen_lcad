/*!@file Neuro/WinnerTakeAll.H Header file for winner-take-all network */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/WinnerTakeAll.H $
// $Id: WinnerTakeAll.H 10818 2009-02-11 06:05:24Z itti $
//

#ifndef WINNERTAKEALL_H_DEFINED
#define WINNERTAKEALL_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Image.H"
#include "Image/LevelSpec.H"
#include "Image/Point2DT.H"
#include "Media/MediaSimEvents.H"
#include "Neuro/LeakyIntFire.H"
#include "Neuro/LeakyIntFireAdp.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/WTAwinner.H"
#include "Simulation/SimModule.H"
#include "Simulation/SimEvents.H"
#include "VFAT/segmentImageMC2.H"
#include "Util/SimTime.H"

class FrameOstream;
class ModelManager;

// ######################################################################
//! The winner-take-all base class
// ######################################################################
/*! This is a 2D winner-take-all, a.k.a., spatial maximum detector. It
  is just a base class with virtual function definitions. Various
  winner-take-all variants are available below, which provide
  implementations for these virtual functions. The WTA receives input
  through the input() function, and its current state (membrane
  potential of neurons) can be retrieved using getV(). Various
  inplementations will typically derive from class Image and inherit
  all the standard methods of class Image.

  See also WinnerTakeAllAdapter for an intermediate base class that
  provides the getV() output functions. Why are those functions not in
  this base class? The reason is that Brain only requires the input
  interface of WinnerTakeAll, and separating the output interface
  allows us to make a stub class that has just no-op input functions.
*/
class WinnerTakeAll : public SimModule
{
public:
  // ######################################################################
  //! @name Constructor, destructor, and reset
  //@{

  //! Ininitialized constructor
  /*! The SM will be resized and initialized the first time input() is
    called */
  WinnerTakeAll(OptionManager& mgr,
                const std::string& descrName,
                const std::string& tagName);

  //! Destructor
  virtual ~WinnerTakeAll();

  //@}

private:
  // forbid assignment and copy-construction:
  WinnerTakeAll& operator=(const WinnerTakeAll& wta);
  WinnerTakeAll(const WinnerTakeAll& wta);
};


// ######################################################################
//! A do-nothing stub implementation of WinnerTakeAll
// ######################################################################
class WinnerTakeAllStub : public WinnerTakeAll
{
public:
  //! Constructor
  WinnerTakeAllStub(OptionManager& mgr,
                    const std::string& descrName = "Winner-Take-All Stub",
                    const std::string& tagName = "WinnerTakeAllStub");

  //! Destructor
  virtual ~WinnerTakeAllStub();
};

// ######################################################################
//! A partial winner-take-all implementation with a few common member variables
// ######################################################################
class WinnerTakeAllAdapter : public WinnerTakeAll
{
public:
  //! Constructor
  WinnerTakeAllAdapter(OptionManager& mgr,
                       const std::string& descrName,
                       const std::string& tagName);

  //! Destructor
  virtual ~WinnerTakeAllAdapter();

  //! Reset our number of shifts
  virtual void reset1();

protected:
  //! Callback for when a new data from the AGM is available
  SIMCALLBACK_DECLARE(WinnerTakeAllAdapter, SimEventAttentionGuidanceMapOutput);

  //! Callback for when a new eye activity is going on
  SIMCALLBACK_DECLARE(WinnerTakeAllAdapter, SimEventSaccadeStatusEye);

  //! Callback for every clock tick, to run our diff equations in integrate()
  SIMCALLBACK_DECLARE(WinnerTakeAllAdapter, SimEventInputFrame);

  //! Callback for every time we should save our outputs
  SIMCALLBACK_DECLARE(WinnerTakeAllAdapter, SimEventSaveOutput);

 //! Set new input currents for all neurons
  /*! This will initialize and resize the network if the network is
    currently uninitialized (e.g., just after construction or reset()). */
  virtual void input(const Image<float>& in) = 0;

  //! Update with new eye position
  virtual void eyeMovement(const Point2D<int>& curreye);

  //! Return all our membrane potential voltages as an Image<float>
  virtual Image<float> getV() const = 0;

  //! Turn saccadic suppression on/off
  virtual void saccadicSuppression(const bool on) = 0;

  //! Turn blink suppression on/off
  virtual void blinkSuppression(const bool on) = 0;

  //! evolve and check if we got a winner
  virtual void integrate(const SimTime& t, Point2D<int>& winner) = 0;

  //! Text log file name
  OModelParam<std::string> itsLogFile;

  //! Save our internals when saveResults() is called?
  OModelParam<bool> itsSaveResults;

  //! use saccadic suppression?
  OModelParam<bool> itsUseSaccadicSuppression;

  //! use blink suppression?
  OModelParam<bool> itsUseBlinkSuppression;

  //! Specification of our image processing center-surround levels
  /*! The only thing that we care about here is the saliency map level */
  OModelParam<LevelSpec> itsLevelSpec;

  //! add jitter to winning location?
  OModelParam<bool> itsUseRandom;

  //! Exit after a given number of attention shifts (if not zero)
  OModelParam<int> itsTooManyShifts;

  //! Exit after a given number of attention shifts per frame (if not zero)
  OModelParam<int> itsTooManyShiftsPerFrame;

  //! Inter-covert-shift delay that makes a shift of attention boring
  OModelParam<SimTime> itsBoringDelay;

  /*! Minimum saliency map depolarization (in mV) to make a covert
    attention shift not boring */
  OModelParam<float> itsBoringSMmv;

  //! attention shift number (to decide whether we have done too many)
  int itsNumShifts;

  //! copy of our last input
  Image<float> itsInputCopy;

  //! last eye position
  Point2D<int> itsEyePos;

  //! its most recent winner
  WTAwinner itsLastWinner;
};

// ######################################################################
//! The standard winner-take-all
// ######################################################################
/*! This is a trivial winner-take-all implementation, based on a 2D
  layer of LeakyIntFire neurons and a unique global inhibitory
  neuron. All neurons in the layer charge up in parallel; whenever one
  reaches threshold, it activates the inhibitory interneuron (which is
  connected to all neurons in the layer), which in turn resets all
  neurons in the layer.  WinnerTakeAllStd is an Image<LeakyIntFire>
  and inherits all the standard methods of class Image. To avoid
  confusion, we here add explicit input() and getV() methods (rather
  than providing conversion functions between LeakyIntFire and float,
  which could make the process more transparent but also possibly more
  confusing). */
class WinnerTakeAllStd : public WinnerTakeAllAdapter
{
public:
  //! Constructor
  WinnerTakeAllStd(OptionManager& mgr,
                   const std::string& descrName = "Winner-Take-All Std",
                   const std::string& tagName = "WinnerTakeAllStd");

  //! Destructor
  virtual ~WinnerTakeAllStd();

  //! Reset to initial state just after construction
  virtual void reset1();

protected:
  //! Set new input currents for all neurons
  /*! This will initialize and resize the network if the network is
    currently uninitialized (e.g., just after construction or reset()). */
  virtual void input(const Image<float>& in);

  //! Return all our membrane potential voltages as an Image<float>
  virtual Image<float> getV() const;

  //! Turn saccadic suppression on/off
  virtual void saccadicSuppression(const bool on);

  //! Turn blink suppression on/off
  virtual void blinkSuppression(const bool on);

  //! Integrate inputs until time t (in s) and update membrane potentials
  virtual void integrate(const SimTime& t, Point2D<int>& winner);

  Image<LeakyIntFire> itsNeurons;
  LeakyIntFire itsGIN;               // global inhibition neuron
  SimTime itsT;                      // keep track of time
  float itsGleak, itsGinh;           // in Siemens
  float itsGinput;                   // in Siemens
  void inhibit();                    // inhibit the whole layer
};

// ######################################################################
//! A fast winner-take-all
// ######################################################################
/*! This just does an Image<T>::findMax() on the last input array
  received. It hence returns a winner at each time step. */
class WinnerTakeAllFast : public WinnerTakeAllAdapter
{
public:
  //! Ininitialized constructor
  /*! The WTA will be resized and initialized the first time input() is
    called */
  WinnerTakeAllFast(OptionManager& mgr,
                    const std::string& descrName = "Winner-Take-All Fast",
                    const std::string& tagName = "WinnerTakeAllFast");

  //! Destructor
  virtual ~WinnerTakeAllFast();

protected:
  //! Set new input currents for all neurons
  /*! This will initialize and resize the network if the network is
    currently uninitialized (e.g., just after construction or reset()). */
  virtual void input(const Image<float>& in);

  //! Return all our membrane potential voltages as an Image<float>
  virtual Image<float> getV() const;

  //! Turn saccadic suppression on/off
  virtual void saccadicSuppression(const bool on);

  //! Turn blink suppression on/off
  virtual void blinkSuppression(const bool on);

  //! Integrate inputs until time t (in s) and update membrane potentials
  virtual void integrate(const SimTime& t, Point2D<int>& winner);
};

// ######################################################################
//! A greedy attention selection mechanism, not exactly winner-take-all
// ######################################################################
/*! This will select, among a set of possible candidate targets that
are above a threshold, the one that is closest to current eye
position. It hence relies on being passed a valid current eye position
in the queue. Note that the returned winner could be exactly at our
current eye position if that is above threshold; we here do not
attempt to force it to be some distance away from current fixation,
and instead rely on IOR in the saliency map to enforce that if
desired. */
class WinnerTakeAllGreedy : public WinnerTakeAllStd
{
public:
  //! Ininitialized constructor
  WinnerTakeAllGreedy(OptionManager& mgr,
                      const std::string& descrName = "Winner-Take-All Greedy",
                      const std::string& tagName = "WinnerTakeAllGreedy");

  //! Destructor
  virtual ~WinnerTakeAllGreedy();

protected:
  //! Integrate inputs until time t (in s) and update membrane potentials
  /*! We operate like a WinnerTakeAllStd, except that when a winner is
    found, we look around for locations that are above threshold, and
    pick the closest one to our current eye position. */
  virtual void integrate(const SimTime& t, Point2D<int>& winner);

  OModelParam<float> itsThresholdFac; //!< Threshold factor, in [0.0 .. 1.0]
};

// ######################################################################
//! Winner take all adapted for temporal noticing
// ######################################################################
/*! This is a trivial winner-take-all implementation, based on a 2D
  layer of LeakyIntFire neurons and a unique global inhibitory
  neuron. All neurons in the layer charge up in parallel; whenever one
  reaches threshold, it activates the inhibitory interneuron (which is
  connected to all neurons in the layer), which in turn resets all
  neurons in the layer.  WinnerTakeAllStd is an Image<LeakyIntFire>
  and inherits all the standard methods of class Image. To avoid
  confusion, we here add explicit input() and getV() methods (rather
  than providing conversion functions between LeakyIntFire and float,
  which could make the process more transparent but also possibly more
  confusing). */
class WinnerTakeAllTempNote : public WinnerTakeAllStd
{
public:
  //! Ininitialized constructor
  /*! The WTA will be resized and initialized the first time input() is
    called */
  WinnerTakeAllTempNote(OptionManager& mgr,
            const std::string& descrName = "Winner-Take-All Temporal Noticing",
            const std::string& tagName = "WinnerTakeAllTempNote");

  //! Destructor
  virtual ~WinnerTakeAllTempNote();

  //! Reset to initial state just after construction
  virtual void reset1();

protected:
  //! Set new input currents for all neurons
  /*! This will initialize and resize the network if the network is
    currently uninitialized (e.g., just after construction or reset()). */
  virtual void input(const Image<float>& in);

  //! Return all our membrane potential voltages as an Image<float>
  /*! The curent values are possibly normalized to 0..255. The
    normalization will use itsSalmapFactor if it is non-null */
  virtual Image<float> getV() const;

  //! Turn saccadic suppression on/off
  virtual void saccadicSuppression(const bool on);

  //! Turn blink suppression on/off
  virtual void blinkSuppression(const bool on);

  Image<float> getVth(const bool normalize = false) const;

  //! Integrate inputs until time t (in s) and update membrane potentials
  virtual void integrate(const SimTime& t, Point2D<int>& winner);

  Image<LeakyIntFireAdp> itsNeurons;
  //! Use the color segmenter to find a good mask region around winner
  segmentImageMC2<float, unsigned int, 1> itsMaskSegment;
  std::vector<float> itsLowMaskBound;
  std::vector<float> itsHighMaskBound;
  Image<float>       itsInitMask;
  LeakyIntFire itsGIN;               // global inhibition neuron
  SimTime itsT;                      // keep track of time
  float itsGleak, itsGinh;           // in Siemens
  float itsGinput;                   // in Siemens
  void inhibit();                    // inhibit the whole layer

  //! Save our various results
  virtual void save1(const ModelComponentSaveInfo& sinfo);
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

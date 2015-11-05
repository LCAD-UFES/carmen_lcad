/*!@file Neuro/SaccadeControllers.H Derived classes for saccade generation */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SaccadeControllers.H $
// $Id: SaccadeControllers.H 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef SACCADECONTROLLERS_H_DEFINED
#define SACCADECONTROLLERS_H_DEFINED

#include "Component/ModelParam.H"
#include "Image/ArrayData.H" // for class Dims
#include "Neuro/SaccadeController.H"
#include "Util/SimTime.H"

#include <deque>
#include <string>

class SpatialMetrics;

// ######################################################################
//! Stub saccade controller
/*! No-op implementation -- never makes a decision (eye movement). */

class StubSaccadeController : public SaccadeController
{
public:
  //! Constructor
  StubSaccadeController(OptionManager& mgr, const SaccadeBodyPart bodypart);

  //! Destructor
  virtual ~StubSaccadeController();

protected:
  //! evolve one time step
  virtual void doEvolve(SimEventQueue& q);

  //! This method is called each time a new percept has arrived
  virtual void computeWhenNewPercept(SimEventQueue& q);

  //! This method is called each time a reset is made
  virtual void computeWhenResetPos(SimEventQueue& q);

  //! This method is called each time a new decision is requested
  virtual Point2D<int> computeWhenNewDecision(SaccadeState& sacstate,
                                         bool& blinkstate, SimEventQueue& q);
};

// ######################################################################
//! Trivial saccade controller
/*! TrivialSaccadeController is a direct mapping from percept to decision. */

class TrivialSaccadeController : public SaccadeController
{
public:
  //! Constructor
  TrivialSaccadeController(OptionManager& mgr, const SaccadeBodyPart bodypart);

  //! Destructor
  virtual ~TrivialSaccadeController();

protected:
  OModelParam<float> itsMinSacLen; //!< Minimum saccade len

  //! evolve one time step
  virtual void doEvolve(SimEventQueue& q);

  //! This method is called each time a new percept has arrived
  virtual void computeWhenNewPercept(SimEventQueue& q);

  //! This method is called each time a reset is made
  virtual void computeWhenResetPos(SimEventQueue& q);

  //! This method is called each time a new decision is requested
  virtual Point2D<int> computeWhenNewDecision(SaccadeState& sacstate,
                                         bool& blinkstate, SimEventQueue& q);
};

// ######################################################################
//! Fixed saccade controller
/*! FixedSaccadeController holds fixation at a fixed point, ignoring
its percepts. Useful to render the difference between covert and overt
attention evident in the model. Also useful for creating foveated
displays with fixed viewpoint. Like with all saccade controllers, the
fixation point may externally be changed using resetPos(). */

class FixedSaccadeController : public SaccadeController
{
public:
  //! Constructor
  FixedSaccadeController(OptionManager& mgr, const SaccadeBodyPart bodypart);

  //! Destructor
  virtual ~FixedSaccadeController();

protected:
  //! evolve one time step
  virtual void doEvolve(SimEventQueue& q);

  //! This method is called each time a new percept has arrived
  virtual void computeWhenNewPercept(SimEventQueue& q);

  //! This method is called each time a reset is made
  virtual void computeWhenResetPos(SimEventQueue& q);

  //! This method is called each time a new decision is requested
  virtual Point2D<int> computeWhenNewDecision(SaccadeState& sacstate,
                                         bool& blinkstate, SimEventQueue& q);

private:
  Point2D<int> itsPos;
};

// ######################################################################
//! Friction-damped mass-spring saccade controller
/*! FrictionSaccadeController acts like a mass (at saccade center)
connected to the covert focus of attention through a spring of zero
rest length, and sliding on a plane in the presence of friction. */

class FrictionSaccadeController : public SaccadeController
{
public:
  //! Constructor
  /*! @param part As this controller may be used either for the eye or
    for the head (as part of bigger controllers that include both eye
    and head), this should be either SaccadeBodyPartEye or
    SaccadeBodyPartHead and will determine which model options we use
    for our internal parameters (eye or head) */
  FrictionSaccadeController(OptionManager& mgr,
                            const SaccadeBodyPart bodypart);

  //! Destructor
  virtual ~FrictionSaccadeController();

protected:
  friend class ThresholdFrictionSaccadeController;
  friend class MonkeySaccadeController;

  OModelParam<double> itsSpringK;    //!< spring stiffness
  OModelParam<double> itsFrictionMu; //!< friction coeff
  OModelParam<Dims> itsDims;         //!< Image size so that we stay inside it
  OModelParam<SimTime> itsTimeStep;  //!< Simulation time step

  //! evolve one time step
  virtual void doEvolve(SimEventQueue& q);

  //! This method is called each time a new percept has arrived
  virtual void computeWhenNewPercept(SimEventQueue& q);

  //! This method is called each time a reset is made
  virtual void computeWhenResetPos(SimEventQueue& q);

  //! This method is called each time a new decision is requested
  virtual Point2D<int> computeWhenNewDecision(SaccadeState& sacstate,
                                         bool& blinkstate, SimEventQueue& q);

private:
  double x, y, oldx, oldy;
  SimTime oldt;
};

// ######################################################################
//! Threshold-limited controller that eliminates small or unreliable shifts
/*! ThresholdSaccadeController will issue an overt control whenever
  the received covert shifts are large and go to a same target for
  several successive covert shifts. It is hence to be used with IOR
  turned off, so that the covert system will keep attending to a same
  location for a while. Once a minimum number of covert shifts have
  been made that are sufficiently far from the current overt location
  and that are sufficiently close to each other, an overt shift will
  be issued, to the average location of the accumulated covert
  locations. */

class ThresholdSaccadeController : public SaccadeController
{
public:
  //! Constructor
  /*! @param minovertdist minimum distance required to warrant an overt
  shift (in pixels at the original image's scale).
  @param maxcovertdist maximum distance to consider that two covert
  shifts point to the "same" location.
  @param minfix minimum number of covert shifts within maxcovertdist
  of each other and farther than minovertdist from the curent overt
  fixation, such as to warrant an overt shift.
  @param sweigh if true, average covert location is a weighted average
  with saliency values at the covert locations used as weights. Otherwise,
  the weight of each covert location is unity. */

  /*! @param part As this controller may be used either for the eye or
      for the head (as part of bigger controllers that include both
      eye and head), this should be either SaccadeBodyPartEye or
      SaccadeBodyPartHead and will determine which model options we
      use for our internal parameters (eye or head) */
  ThresholdSaccadeController(OptionManager& mgr,
                             const SaccadeBodyPart bodypart);

  //! Destructor
  virtual ~ThresholdSaccadeController();

  //! Reset ThresholdSaccadeController
  /*! See the base function in ModelComponent.H for info. */
  virtual void reset1();

  //! Check queued-up percepts and return average
  /*! check whether queued-up percepts are close to each other, and
    also return average position of percept queue */
  virtual void checkPercepts(bool& areclose, Point2D<int>& avgp);

protected:
  friend class ThresholdFrictionSaccadeController;

  //! evolve one time step
  virtual void doEvolve(SimEventQueue& q);

  //! This method is called each time a new percept has arrived
  virtual void computeWhenNewPercept(SimEventQueue& q);

  //! This method is called each time a reset is made
  virtual void computeWhenResetPos(SimEventQueue& q);

  //! This method is called each time a new decision is requested
  virtual Point2D<int> computeWhenNewDecision(SaccadeState& sacstate,
                                         bool& blinkstate, SimEventQueue& q);

  //! Get us started
  virtual void start1();

  OModelParam<float> itsOdist;    //!< min overt distance
  OModelParam<float> itsCdist;    //!< max covert distance
  OModelParam<int> itsMinNum;     //!< min number of close covert shifts
  OModelParam<bool> itsSalWeight; //!< weight average covert pos by saliency

  nub::ref<SpatialMetrics> itsMetrics; //!< metrics that depend on input size

  float odistsq;  // square of min overt distance
  float cdistsq;  // square of max covert distance
  bool sweight;   // use saliency as weight for average covert location
  bool didresetpos;  // just did a resetPos
  Point2D<int> resetloc; // location of reset
};

// ######################################################################
//! Threshold-limited friction-damped controller
/*! ThresholdFrictionSaccadeController combines the features of
  ThresholdSaccadeController and FrictionSaccadeController.

  The ThresholdSaccadeController determines when a new saccade is
  warranted. The FrictionSaccadeController is fed with the average
  covert location over the duration of the percept queue of the
  Threshold controller. It then is responsible for inducing slow
  drifts of overt attention towards that average, in between two
  saccades. In addition, this controller will return to the center of
  the image if it does not move in some time. */
class ThresholdFrictionSaccadeController : public SaccadeController
{
public:
  //! Constructor
  /*! See the two base classes for parameters.

      @param maxidle max time (in seconds) before we naturally return
      to the center of the image. This will happen if the covert
      shifts are so chaotic that we never decide to follow them
      overtly.

      @param part As this controller may be used either for the eye or
      for the head (as part of bigger controllers that include both
      eye and head), this should be either SaccadeBodyPartEye or
      SaccadeBodyPartHead and will determine which model options we
      use for our internal parameters (eye or head) */
  ThresholdFrictionSaccadeController(OptionManager& mgr,
                                     const SaccadeBodyPart bodypart);

  //! Destructor
  virtual ~ThresholdFrictionSaccadeController();

protected:
  //! evolve one time step
  virtual void doEvolve(SimEventQueue& q);

  //! This method is called each time a new percept has arrived
  virtual void computeWhenNewPercept(SimEventQueue& q);

  //! This method is called each time a reset is made
  virtual void computeWhenResetPos(SimEventQueue& q);

  //! This method is called each time a new decision is requested
  virtual Point2D<int> computeWhenNewDecision(SaccadeState& sacstate,
                                         bool& blinkstate, SimEventQueue& q);

  OModelParam<SimTime> itsMaxIdle; //!< max time (in s) before we return to IEP

private:
  nub::ref<SpatialMetrics> itsMetrics;
  nub::ref<ThresholdSaccadeController> tsc;
  nub::ref<FrictionSaccadeController> fsc;
};


// ######################################################################
//! Realistic Monkey Eye or Head controller
/*! This is part of an attempt at a simplified approach to the
  MonkeySaccadeCopntroller, which turned out quite complex because of
  the interdependency between head and eye movements. Here, we fake it
  by having independent controllers for the eyes and the head, such
  that we don't need to even deal with gaze stabilization during head
  movements and VOR. This controller will do either the head or the
  eye. For a controller that does both, see
  Monkey2SaccadeController. */
class MonkeySaccadeController : public SaccadeController
{
public:
  //! Constructor
  /*! Parameters are similar to the FrictionSaccadeController, plus
      @param part As this controller may be used either for the eye or
      for the head (as part of bigger controllers that include both
      eye and head), this should be either SaccadeBodyPartEye or
      SaccadeBodyPartHead and will determine which model options we
      use for our internal parameters (eye or head) */
  MonkeySaccadeController(OptionManager& mgr,
                          const SaccadeBodyPart bodypart);

  //! Destructor
  ~MonkeySaccadeController();

protected:
  //! evolve one time step
  virtual void doEvolve(SimEventQueue& q);

  //! This method is called each time a new percept has arrived
  virtual void computeWhenNewPercept(SimEventQueue& q);

  //! This method is called each time a reset is made
  virtual void computeWhenResetPos(SimEventQueue& q);

  //! This method is called each time a new decision is requested
  virtual Point2D<int> computeWhenNewDecision(SaccadeState& sacstate,
                                         bool& blinkstate, SimEventQueue& q);

private:
  nub::ref<FrictionSaccadeController> fsc;
  nub::ref<SpatialMetrics> itsMetrics; // metrics that depend on input size
  double duration; // duration of realistic saccade
  SimTime oldt;    // time of last call to evolve()
  SimTime startt;  // time of start of realistic saccade
  double currx, curry; // current eye position
  double vmax;   // max velocity
  double accel;  // acceleration
  double theta; // angle of saccade
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

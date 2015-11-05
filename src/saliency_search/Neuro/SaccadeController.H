/*!@file Neuro/SaccadeController.H Base class for saccade generation */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SaccadeController.H $
// $Id: SaccadeController.H 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef SACCADECONTROLLER_H_DEFINED
#define SACCADECONTROLLER_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Point2DT.H"
#include "Neuro/WTAwinner.H"
#include "Neuro/SaccadeBodyPart.H"
#include "Psycho/SaccadeState.H"
#include "Simulation/SimModule.H"
#include "Util/TransientStatus.H"
#include <deque>

//! A base class to implement saccade controllers
/*! This base class handles the basic queuing of percepts and
fixations. Typically, new percepts (e.g., winner-take-all winners over
the saliency map) should be passed to the saccade controller using
setPercept(). During normal evolution of a simulation, evolve() should
be called at every time step, in a manner similar to that of Brain,
WTA and SM. In order to obtain coordinates of saccade targets computed
by this class, getDecision() should be called. It will return (-1, -1)
if no new saccade target is decided (either because it cannot be
decided, or because it is the same as the last valid returned
one). So, saccading to and fixating a new target will yield one
initial valid set of returned coordinates, followed by subsequent
returns of (-1, -1) as long as fixation is to be held
stationary. Derived classes should overload computeWhenNewPercept(),
called each time a new percept has arrived, and
computeWhenNewDecision(), called each time a new decision is
requested. In addition, derived classes should usually overload
doEvolve(), typically called at each time step during a simulation.
See SaccadeControllers.H for examples of derived classes which
actually implement saccadic control strategies. */

class SaccadeController : public SimModule
{
public:
  //! Constructor
  /*! @param bodypart should be SaccadeBodyPartEye or SaccadeBodyPartHead
    @param percept_qlen length of the queue of accumulated percepts
    @param decision_qlen length of the queue of generated decisions */
  SaccadeController(OptionManager& mgr,
                    const std::string& descrName,
                    const std::string& tagName,
                    const SaccadeBodyPart bodypart,
                    const int percept_qlen,
                    const int decision_qlen);

  //! Destructor (virtual to ensure that derived classes correctly destroyed)
  virtual ~SaccadeController();

  //! evolve one time step (called, e.g., by Brain::evolve())
  /*! evolve() should be invoked by external callers to allow the
      internals of the controller to evolve as time passes by but no
      new percept is available. Derived classes should not override
      this function, but should instead override doEvolve(), which is
      called from inside evolve(). */
  void evolve(SimEventQueue& q);

  //! Receive a new percept (e.g., winner from saliency map)
  /*! This method should be invoked by external callers each time a
    new percept is available. Do not overload this method, overload
    computeWhenNewPercept() instead if you need to perform some
    computation for each new percept. */
  virtual void setPercept(const WTAwinner& fix, SimEventQueue& q);

  //! Reset to an externally-given position
  /*! This will force the current position to the position given as
    argument, disregarding any internal physical model. So this may be
    used to externally drive the SaccadeController to given
    locations. A percept will be queued up, which will be built from
    the passed position/time plus default values for the WTAwinner
    fields. Do not overload this method, overload computeWhenReset()
    instead. NOTE: this will abort any current blink and will reset
    our internal state to fixation (if p is valid) or unknown (if p is
    invalid). */
  virtual void resetPos(const Point2D<int>& p, SimEventQueue& q);

  //! Get the fixation decided by the controller
  /*! @param t should be the current time (in seconds) and should be
    greater than or equal to the time of the last percept passed to
    setPercept(). The coordinates of the decided fixation are returned
    (or (-1, -1) of no decision has been made). If a decision is made
    but is the same as the previous one, the behavior here is to
    return (-1, -1), and the caller has the responsibility of holding
    fixation until a new, valid decision can be reached. Do not
    overload this method, overload computeWhenNewDecision() instead if
    you need to do some computation before returning a fixation. If
    dopoststatus is true, we will post a SimEventStatus[Eye|Head] with
    our current status to the SimEventQueue. */
  virtual Point2D<int> getDecision(SimEventQueue& q,
                              const bool dopoststatus = false);

  //! Get a previous percept
  /*! This method allows one to access a previous percept passed
    through setPercept(), or (-1, -1) if no valid previous percept
    exists.
    @param index indicates which previous percept; a value of zero
    will return the last percept that was passed to setPercept(), and
    increasing values will return increasingly older percepts. If
    index is more than the queue length, (-1, -1) will be returned. */
  WTAwinner getPreviousPercept(const unsigned int index = 0) const;

  //! Do we already have any percepts queued up?
  bool havePercepts() const;

  //! Returns length of percept queue
  /*! Note that the queue may not be full to that length yet */
  int getPqlen() const;

  //! Reset length of percept queue (contents will be lost)
  void resetPqlen(const int len);

  //! Kill all previous percepts (empty the percept queue)
  void killPercepts();

  //! Get a previous decision
  /*! This method allows one to access a previous valid decision. It
    will not attempt to determine whether a new decision can be made
    now, like getDecision() does. It will simply return the result of
    what was previously decided (or (-1, -1) if no valid previous
    decision exists).
    @param index indicates which previous decision; a value of zero
    will return the last decision that was made and returned by
    getDecision(), and increasing values will return increasingly
    older decisions. if index is more than the queue length, (-1, -1)
    will be returned. */
  Point2DT getPreviousDecision(const unsigned int index = 0) const;

  //! Do we already have any decisions queued up?
  bool haveDecisions() const;

  //! Returns length of decision queue
  /*! Note that the queue may not be full to that length yet */
  int getDqlen() const;

  //! Reset length of decision queue (contents will be lost)
  void resetDqlen(const int len);

  //! Kill all previous decisions (empty the decision queue)
  void killDecisions();

  //! get our body part
  SaccadeBodyPart bodyPart() const;

   //! Show contents of the queues, for debugging
   void dumpQueues() const;

protected:
  OModelParam<Point2D<int> > itsInitialPosition; //!< start position

  //! Reset SaccadeController
  virtual void reset1();

  //! Called from within evolve()
  /*! Classes deriving from SaccadeController should implement this. */
  virtual void doEvolve(SimEventQueue& q) = 0;

  //! This method is called each time a new percept has arrived
  /*! The latest percept is at the front of the percept queue. Derived
    classes must provide some implementation for this method, as it is
    called from within setPercept(). */
  virtual void computeWhenNewPercept(SimEventQueue& q) = 0;

  //! This method is called each time a reset() occurs
  /*! The reset location/time is at the front of the percept
    queue. Derived classes must provide some implementation for this
    method, as it is called from within resetPos().  NOTE: a
    resetPos(), even if it is to a location far away from the current
    one, should not create a saccade, because it is a way to break a
    simulation and reset it, not a way to drive saccades
    externally. Saccades should arise internally in
    computeWhenNewDecision(). You should not change the state here as
    resetPos() itself will change if to fixation or unknown anyway. */
  virtual void computeWhenResetPos(SimEventQueue& q) = 0;

  //! This method is called each time a new decision is requested
  /*! The latest percept is at the front of the percept queue, and the
    latest previous decision at the front of the decision queue. The
    method should return the decision to be taken given the time
    passed as argument. This method should return the current decision
    for fixation, as well as current saccade and blink states. The
    wrapping getDecision() call will take care of eliminating
    decisions that are the same as the previous ones.  Derived classes
    must provide some implementation for this method, as it is called
    from within getDecision(). */
  virtual Point2D<int> computeWhenNewDecision(SaccadeState& sacstate,
                                         bool& blinkstate,
                                         SimEventQueue& q) = 0;

  //! Helper for derived classes to get our internal saccade/fix/smooth state
  SaccadeState getState() const;

  //! Helper for derived classes to get our internal blink state
  bool getBlinkState() const;

private:
  std::deque<WTAwinner> percept; // queue of accumulated percepts; front=latest
  std::deque<Point2DT> decision; // queue of accumulated decisions;front=latest
  unsigned int pqlen, dqlen;     // size of percept & decisioon queues
  SaccadeState itsState;         // our saccade/fix/smooth/etc state
  SaccadeState itsPrevState;     // state before last evolve()
  bool itsBlinkState;            // blink state
  bool itsPrevBlinkState;        // previous blink state
  SaccadeBodyPart itsBodyPart;   // are we eye or head?
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
#endif

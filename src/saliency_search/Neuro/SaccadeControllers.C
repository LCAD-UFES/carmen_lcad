/*!@file Neuro/SaccadeControllers.C Derived classes for saccade generation */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SaccadeControllers.C $
// $Id: SaccadeControllers.C 10710 2009-02-01 03:33:35Z itti $
//

#include "Neuro/SaccadeControllers.H"

#include "Component/GlobalOpts.H"
#include "Component/OptionManager.H"
#include "Image/Pixels.H"
#include "Media/MediaOpts.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/Retina.H"
#include "Neuro/SpatialMetrics.H"
#include "Simulation/SimulationOpts.H"
#include "Simulation/SimEventQueue.H"
#include "Util/MathFunctions.H"
#include "Util/log.H"
#include "rutz/compat_cmath.h"

#include <fstream>

// ######################################################################
// ######################################################################
// ######################################################################
StubSaccadeController::StubSaccadeController(OptionManager& mgr,
                                             const SaccadeBodyPart bodypart) :
  SaccadeController(mgr, "Stub Saccade Controller",
                    "StubSaccadeController", bodypart, 2, 2)
{  }

// ######################################################################
StubSaccadeController::~StubSaccadeController()
{  }

// ######################################################################
void StubSaccadeController::doEvolve(SimEventQueue& q)
{  }

// ######################################################################
void StubSaccadeController::computeWhenNewPercept(SimEventQueue& q)
{  }

// ######################################################################
void StubSaccadeController::computeWhenResetPos(SimEventQueue& q)
{  }

// ######################################################################
Point2D<int> StubSaccadeController::
computeWhenNewDecision(SaccadeState& sacstate, bool& blinkstate,
                       SimEventQueue& q)
{
  sacstate = SACSTATE_UNK;
  blinkstate = false;
  return Point2D<int>(-1,-1);
}

// ######################################################################
// ######################################################################
// ######################################################################
TrivialSaccadeController::
TrivialSaccadeController(OptionManager& mgr, const SaccadeBodyPart bodypart) :
  SaccadeController(mgr, "Trivial Saccade Controller",
                    "TrivialSaccadeController", bodypart, 2, 2),
  itsMinSacLen(bodypart == SaccadeBodyPartEye
               ? &OPT_SCeyeMinSacLen
               : &OPT_SCheadMinSacLen,
               this)
{  }

// ######################################################################
TrivialSaccadeController::~TrivialSaccadeController()
{  }

// ######################################################################
void TrivialSaccadeController::doEvolve(SimEventQueue& q)
{  }

// ######################################################################
void TrivialSaccadeController::computeWhenNewPercept(SimEventQueue& q)
{  }

// ######################################################################
void TrivialSaccadeController::computeWhenResetPos(SimEventQueue& q)
{  }

// ######################################################################
Point2D<int> TrivialSaccadeController::
computeWhenNewDecision(SaccadeState& sacstate, bool& blinkstate,
                       SimEventQueue& q)
{
  // this controller never blinks:
  blinkstate = false;

  // just return last percept received (or (-1,-1) if no percept yet):
  Point2D<int> p = getPreviousPercept(0).p;

  if (p.isValid())
    {
      // if this is far from our last decision, let's put ourselve in saccade:
      Point2D<int> pp = getPreviousDecision(0).p;
      if (pp.isValid() && p.distance(pp) >= itsMinSacLen.getVal())
        sacstate = SACSTATE_SAC;
      else
        sacstate = SACSTATE_FIX;
    }
  else sacstate = SACSTATE_UNK;

  return p;
}

// ######################################################################
// ######################################################################
// ######################################################################
FixedSaccadeController::
FixedSaccadeController(OptionManager& mgr, const SaccadeBodyPart bodypart) :
  SaccadeController(mgr, "Fixed Saccade Controller",
                    "FixedSaccadeController", bodypart, 2, 2),
  itsPos(-1, -1)
{  }

// ######################################################################
FixedSaccadeController::~FixedSaccadeController()
{  }

// ######################################################################
void FixedSaccadeController::doEvolve(SimEventQueue& q)
{  }

// ######################################################################
void FixedSaccadeController::computeWhenNewPercept(SimEventQueue& q)
{  }

// ######################################################################
void FixedSaccadeController::computeWhenResetPos(SimEventQueue& q)
{
  // during start/reset, an eye position should have been set
  // depending on command-line options for initial position:
  itsPos = getPreviousPercept(0).p;
}

// ######################################################################
Point2D<int> FixedSaccadeController::
computeWhenNewDecision(SaccadeState& sacstate, bool& blinkstate,
                       SimEventQueue& q)
{
  if (itsPos.isValid()) sacstate = SACSTATE_FIX;
  else sacstate = SACSTATE_UNK;

  // this controller never blinks:
  blinkstate = false;

  // just return our current fixed position:
  return itsPos;
}

// ######################################################################
// ######################################################################
// ######################################################################
FrictionSaccadeController::
FrictionSaccadeController(OptionManager& mgr, const SaccadeBodyPart part)
  :
  SaccadeController(mgr, "Friction Saccade Controller",
                    "FrictionSaccadeController",
                    part, 2, 2),  // queue length 2 for current and previous
  itsSpringK(part == SaccadeBodyPartEye
             ? &OPT_SCeyeSpringK
             : &OPT_SCheadSpringK,
             this),
  itsFrictionMu(part == SaccadeBodyPartEye
                ? &OPT_SCeyeFrictionMu
                : &OPT_SCheadFrictionMu,
                this),
  itsDims(&OPT_InputFrameDims, this),
  itsTimeStep(&OPT_SimulationTimeStep, this),
  x(0.0), y(0.0), oldx(0.0), oldy(0.0),
  oldt(SimTime::SECS(-1.0))
{  }

// ######################################################################
FrictionSaccadeController::~FrictionSaccadeController()
{  }

// ######################################################################
void FrictionSaccadeController::doEvolve(SimEventQueue& q)
{
  const SimTime t = q.now();

  if (oldt.secs() < 0.0) return;  // not simulating yet
  if (havePercepts() == false) { oldt = t; return; } // no percept yet

  // get FoA coordinates:
  WTAwinner last = getPreviousPercept(0);   // get latest percept
  double foax = last.p.i + 0.49999;
  double foay = last.p.j + 0.49999;

  // For stability of our difference equations, we need to run them at
  // the given simulation time step. Here, we however support for us
  // being called at much coarser time intervals.
  const SimTime dt =
    SimTime::computeDeltaT((t - oldt), itsTimeStep.getVal());
  double ox = oldx, oy = oldy;
  oldx = x; oldy = y; // keep track of how much we move this step

  for (SimTime tt = oldt; tt < t; tt += dt)
    {
      // compute force:
      double k = itsSpringK.getVal(), mu = itsFrictionMu.getVal();
      double fx = k * (foax - x);
      double fy = k * (foay - y);

      // update position:
      double dts = dt.secs();
      double newx = 2.0 * x - ox + dts * (dts * fx - mu * (x - ox));
      double newy = 2.0 * y - oy + dts * (dts * fy - mu * (y - oy));
      //LINFO("curr=(%.2f, %.2f) foa=(%.2f, %.2f) new=(%.2f, %.2f) "
      //"f=(%.2f, %.2f)", x, y, foax, foay, newx, newy, fx, fy);

      // keep track of the time step and current/old position:
      ox = x; oy = y; x = newx; y = newy;
    }

  oldt = t;
}

// ######################################################################
void FrictionSaccadeController::computeWhenNewPercept(SimEventQueue& q)
{
  // if this is our first percept, let's start the mass/spring now and here:
  if (oldt.secs() < 0.0)
    {
      WTAwinner last = getPreviousPercept(0);
      x = oldx = last.p.i + 0.49999;
      y = oldy = last.p.j + 0.49999;
      oldt = last.t;
    }
  // otherwise, the percept will simply be used as anchor point in our
  // next evolve()
}

// ######################################################################
void FrictionSaccadeController::computeWhenResetPos(SimEventQueue& q)
{
  // get desired new fixation location:
  Point2D<int> newloc = getPreviousPercept(0).p;

  // just shift overt attention coordinates to the new location,
  // disregarding physics of friction and the spring:
  x = newloc.i + 0.49999;
  y = newloc.j + 0.49999;
  oldx = x; oldy = y; oldt = getPreviousPercept(0).t;
}

// ######################################################################
Point2D<int> FrictionSaccadeController::
computeWhenNewDecision(SaccadeState& sacstate, bool& blinkstate,
                       SimEventQueue& q)
{
  // this controller never blinks:
  blinkstate = false;

  if (oldt.secs() < 0.0 || havePercepts() == false)
    { sacstate = SACSTATE_UNK; return Point2D<int>(-1, -1); } // not started yet

  // if we did move during last evolve(), then we are in smooth
  // pursuit, otherwise fixation. This controller never saccades:
  if (sqrt((x-oldx)*(x-oldx) + (y-oldy)*(y-oldy)) > 0.001)
    sacstate = SACSTATE_SMO; else sacstate = SACSTATE_FIX;

  // round current coordinates to pixels:
  Point2D<int> fi = Point2D<int>(int(x), int(y));

  // possibly force fixation to inside image:
  fi.clampToDims(itsDims.getVal());

  // return our current rounded & clamped coordinates:
  return fi;
}

// ######################################################################
// ######################################################################
// ######################################################################
ThresholdSaccadeController::
ThresholdSaccadeController(OptionManager& mgr, SaccadeBodyPart part)
  :
  SaccadeController(mgr, "Threshold Saccade Controller",
                    "ThresholdSaccadeController", part, 2, 2),
  itsOdist(part == SaccadeBodyPartEye
           ? &OPT_SCeyeThreshMinOvert
           : &OPT_SCheadThreshMinOvert,
           this), // see ModelOptionDefs.C
  itsCdist(part == SaccadeBodyPartEye
           ? &OPT_SCeyeThreshMaxCovert
           : &OPT_SCheadThreshMaxCovert,
           this),  // see ModelOptionDefs.C
  itsMinNum(part == SaccadeBodyPartEye
            ? &OPT_SCeyeThreshMinNum
            : &OPT_SCheadThreshMinNum,
            this),  // see ModelOptionDefs.C
  itsSalWeight(part == SaccadeBodyPartEye
               ? &OPT_SCeyeThreshSalWeigh
               : &OPT_SCheadThreshSalWeigh,
               this),  // see ModelOptionDefs.C
  itsMetrics(new SpatialMetrics(mgr))
{
  this->addSubComponent(itsMetrics);
}

// ######################################################################
void ThresholdSaccadeController::start1()
{
  resetPqlen(itsMinNum.getVal());
  odistsq = float(itsMetrics->getFoveaRadius()) * itsOdist.getVal();
  odistsq *= odistsq;
  cdistsq = float(itsMetrics->getFoveaRadius()) * itsCdist.getVal();
  cdistsq *= cdistsq;
  didresetpos = false;

  SaccadeController::start1();
}

// ######################################################################
ThresholdSaccadeController::~ThresholdSaccadeController()
{  }

// ######################################################################
void ThresholdSaccadeController::doEvolve(SimEventQueue& q)
{  }

// ######################################################################
void ThresholdSaccadeController::reset1()
{
  // reset some stuff for ThresholdSaccadeController
  didresetpos = false;

  // propagate to our base class:
  SaccadeController::reset1();
}

// ######################################################################
void ThresholdSaccadeController::checkPercepts(bool& areclose, Point2D<int>& avgp)
{
  // compute average location of queued percepts:
  float ii = 0.0f, jj = 0.0f, w = 0.0f; int idx = 0;
  while(1) {
    WTAwinner win = getPreviousPercept(idx);
    if (win.isValid() == false) break; // reached end of percept queue
    float weight = itsSalWeight.getVal() ? win.sv : 1.0f;
    ii += float(win.p.i) * weight;
    jj += float(win.p.j) * weight;
    w += weight; ++ idx;
  }
  if (w) { avgp.i = int(ii / w + 0.4999f); avgp.j = int(jj / w + 0.4999f); }
  else { avgp.i = -1; avgp.j = -1; }

  // test whether all of our percepts are within cdistsq of the average:
  idx = 0; areclose = true;
  while(1) {
    WTAwinner win = getPreviousPercept(idx);
    if (win.isValid() == false) break; // reached end of percept queue
    if (avgp.squdist(win.p) > cdistsq) { areclose = false; break; }
    ++ idx;
  }
}

// ######################################################################
void ThresholdSaccadeController::computeWhenNewPercept(SimEventQueue& q)
{  }

// ######################################################################
void ThresholdSaccadeController::computeWhenResetPos(SimEventQueue& q)
{
  // get desired new fixation location:
  resetloc = getPreviousPercept(0).p;
  didresetpos = true;

  // NOTE that this is not a legit saccade, hence we do not switch our
  // state to SACSTATE_SAC. Our resetPos() wrapper will set our state
  // to fixation (or unknown if resetloc is invalid).
}

// ######################################################################
Point2D<int> ThresholdSaccadeController::
computeWhenNewDecision(SaccadeState& sacstate, bool& blinkstate,
                       SimEventQueue& q)
{
  // this controller never blinks:
  blinkstate = false;

  // if we just did a reset, return the reset location:
  if (didresetpos)
    { didresetpos = false; sacstate = SACSTATE_FIX; return resetloc; }

  // if no percept yet, return invalid position:
  if (havePercepts() == false)
    { sacstate = SACSTATE_UNK; return Point2D<int>(-1, -1); }

  // are our queued-up percepts close to each other and get their average:
  bool areclose; Point2D<int> avgp;
  checkPercepts(areclose, avgp);
  const Point2D<int> prev = getPreviousDecision(0).p;

  // if the covert shifts are nicely clustered, check that the average
  // if sufficiently far from our current overt location (if we have
  // made an overt shift already; otherwise make our first overt
  // shift if areclose is true):
  if (areclose && haveDecisions() == false)
    { sacstate = SACSTATE_SAC; return avgp; } // first overt shift
  if (areclose && avgp.squdist(prev) >= odistsq)
    { sacstate = SACSTATE_SAC; return avgp; } // overt shift to avg covert loc

  // either the covert shifts are not close to each other, or they are
  // too close to our current overt fixation. Don't move overt
  // attention yet and return whatever previous decision we have made
  // (if any):
  if (prev.isValid()) sacstate = SACSTATE_FIX; else sacstate = SACSTATE_UNK;
  return prev;
}

// ######################################################################
// ######################################################################
// ######################################################################
ThresholdFrictionSaccadeController::
ThresholdFrictionSaccadeController(OptionManager& mgr,
                                   SaccadeBodyPart part)
  :
  SaccadeController(mgr,
                    "Threshold-Friction Saccade Controller",
                    "ThresholdFrictionSaccadeController",
                    part, 2, 2),  // queue length 2 for current and previous
  itsMaxIdle(part == SaccadeBodyPartEye
             ? &OPT_SCeyeMaxIdleSecs
             : &OPT_SCheadMaxIdleSecs,
             this), // see ModelOptionDefs.C
  itsMetrics(new SpatialMetrics(mgr)),
  tsc(new ThresholdSaccadeController(mgr, part)),
  fsc(new FrictionSaccadeController(mgr, part))
{
  this->addSubComponent(itsMetrics);
  addSubComponent(tsc);
  addSubComponent(fsc);
}

// ######################################################################
ThresholdFrictionSaccadeController::~ThresholdFrictionSaccadeController()
{  }

// ######################################################################
void ThresholdFrictionSaccadeController::doEvolve(SimEventQueue& q)
{
  // evolve the friction part of our business:
  fsc->evolve(q);

  // evolve the threshold part of our business:
  tsc->evolve(q);
}

// ######################################################################
void ThresholdFrictionSaccadeController::
computeWhenNewPercept(SimEventQueue& q)
{
  // get the percept we just received:
  WTAwinner win = getPreviousPercept(0);

  // Feed that new percept directly to the ThresholdSaccadeController:
  tsc->setPercept(win, q);

  // feed the average covert location as new percept to the friction
  // controller, only if the queued-up percepts are close to each
  // other (so that the average represents a fairly stable covert
  // fixation), and if that new percept is different from the last one
  // we gave to the FrictionController:
  bool areclose; Point2D<int> avgp; tsc->checkPercepts(areclose, avgp);
  if (areclose)
    {
      const Point2D<int> flast = fsc->getPreviousPercept(0).p;
      if (avgp.isValid() && avgp != flast)
        {
          win.p = avgp;  // replace covert location by average covert location
          fsc->setPercept(win, q);
        }
    }
}

// ######################################################################
void ThresholdFrictionSaccadeController::computeWhenResetPos(SimEventQueue& q)
{
  // get the percept we just received:
  const WTAwinner win = getPreviousPercept(0);

  // reset friction:
  fsc->resetPos(win.p, q);

  // reset threhold:
  tsc->resetPos(win.p, q);
}

// ######################################################################
Point2D<int> ThresholdFrictionSaccadeController::
computeWhenNewDecision(SaccadeState& sacstate, bool& blinkstate,
                       SimEventQueue& q)
{
  const SimTime t = q.now();

  // this controller never blinks:
  blinkstate = false;

  // what does the FrictionSaccadeController recommend?
  Point2D<int> pf = fsc->getDecision(q);

  // what does the ThresholdSaccadeController recommend?
  Point2D<int> pt = tsc->getDecision(q);

  // if we have a saccade recommendation from the threshold
  // controller, override the slow drift recommended by the friction
  // controller. Otherwise, if the friction controller is not
  // recommending a move either, see if we have been idle for too
  // long, in which case let's home to the center of the image:
  if (pt.isValid() ||            // thresh controller has a new move, -- OR --
      (pt.isValid() == false &&  // no new threshold move
       pf.isValid() == false &&  // no new friction move
       haveDecisions() &&        // but we have moved in the past
       t - getPreviousDecision(0).t >
       itsMaxIdle.getVal()))    // and it's been a long time
    {
      // if homing, let's get the coords:
      if (pt.isValid() == false)
        {
          if (SeC<SimEventRetinaImage> e =
              q.check<SimEventRetinaImage>(this, SEQ_ANY))
            pt = e->center();
          else CLFATAL("Cannot home to center without a retina image!");

          // reset tsc to this homing position:
          tsc->resetPos(pt, q);
        }

      // reset the mass/spring simulation of the friction controller
      // to new overt fixation:
      fsc->resetPos(pt, q);

      // we are saccading:
      sacstate = SACSTATE_SAC;

      // do overt shift:
      return pt;
    }

  // otherwise, follow the recommendation of the friction controller,
  // if any (damped drifting of overt attention in between two
  // saccades). This will not yield chaotic damped movements, since
  // the friction controller is only fed with stable percepts
  // (queued-up percepts that are close to each other):
  if (pf.isValid())
    {
      // are we fixating or in smooth pursuit? we'll just do whatever fsc says:
      sacstate = fsc->getState();
      return pf;
    }

  // if friction is not moving, we could be unknown, in saccade, or in
  // fixation; if tsc is in saccade, then that's what we are;
  // otherwise, we are just like fsc:
  if (tsc->getState() == SACSTATE_SAC) sacstate = SACSTATE_SAC;
  else sacstate = fsc->getState();
  return getPreviousDecision(0).p;
}

// ######################################################################
// ######################################################################
// ######################################################################
MonkeySaccadeController::MonkeySaccadeController(OptionManager& mgr,
                                                 SaccadeBodyPart part)
  :
  SaccadeController(mgr, "Monkey Saccade Controller",
                    "MonkeySaccadeController", part, 2, 2),
  fsc(new FrictionSaccadeController(mgr, part)),
  itsMetrics(new SpatialMetrics(mgr))
{
  oldt = SimTime::ZERO();
  addSubComponent(fsc);
  addSubComponent(itsMetrics);
}

// ######################################################################
MonkeySaccadeController::~MonkeySaccadeController()
{  }

// ######################################################################
/*
void MonkeySaccadeController::saccade(const Point2DT& target)
{
  // our starting point is whatever fsc is currently at:
  Point2D<int> currfsc = fsc->getPreviousDecision(0).p;
  itsMetrics->pix2deg(currfsc, currx, curry);

  // initialize simulation:
  startt = oldt = target.t;
  double ampX, ampY; itsMetrics->pix2deg(target.p, ampX, ampY);
  ampX -= currx; ampY -= curry;  // amplitude of displacement from current

  double amplitude = sqrt(ampX * ampX + ampY * ampY);
  theta = atan(ampY / ampX); if (ampX < 0.0) theta += M_PI;

  // If amplitude too small, don't even bother starting the saccade:
  if (amplitude < 0.0001) return;

  if (SaccadeBodyPartEye == bodyPart())
    {
      // set us up for a realistic eye saccade:
      vmax = 473.0 * (1.0 - exp(-amplitude / 7.8)); // degrees/ms
      duration = 2.0 * amplitude / vmax;
      accel = 2.0 * vmax / duration; // degrees/s^2
    }
  else
    {
      // set us up for a realistic head saccade:
      vmax = (3.9 * amplitude + 16.0); // degrees/ms
      duration = 2.0 * amplitude / vmax;
      accel = 2.0 * vmax / duration; // degrees/ms^2
    }

  LINFO("%s: Ampl=%.2fdeg vMax=%.2f dur=%.2fms theta=%.2fdeg start=%.2fms",
        (SaccadeBodyPartEye == bodyPart()) ? "Eye " : "Head",
        amplitude, vmax, duration * 1000.0,
        theta * 180.0 / M_PI, startt.msecs());
}
*/
// ######################################################################
void MonkeySaccadeController::computeWhenNewPercept(SimEventQueue& q)
{
  // get our new percept:
  WTAwinner win = getPreviousPercept(0);

  // just pass it on to fsc; that's our new spring anchor point:
  fsc->setPercept(win, q);
}

// ######################################################################
void MonkeySaccadeController::doEvolve(SimEventQueue& q)
{
  const SimTime t = q.now();
  /*
  // are we executing a realistic saccade?
  if (getState() == SACSTATE_SAC)
    {
      // do the realistic simulation:
      double deltaT = (t - oldt).secs(), elapsedT = (t - startt).secs();

      // Get the average velocity during this interval, that's the
      // velocity at mid-point of the interval:
      double v;
      if (elapsedT > 0.5 * duration)
        v = vmax - accel * (elapsedT + 0.5 * deltaT - 0.5 * duration);
      else
        v = accel * (elapsedT + 0.5 * deltaT);

      // update current position:
      currx += v * cos(theta) * deltaT;
      curry += v * sin(theta) * deltaT;

      // drive the friction controller with our current coordinates:
      Point2D<int> currpt; itsMetrics->deg2pix(currx, curry, currpt);
      fsc->resetPos(currpt, q);

      // if reached the end, finish the saccade:
      if (elapsedT >= duration) {
        LINFO("=== %s: end of saccade at (%d, %d) at %.2fms ===",
              (SaccadeBodyPartEye == bodyPart()) ? "Eye " : "Head",
              currpt.i, currpt.j, t.msecs());
        sacstate = SACSTATE_FIX;
      }
    }
  else
    {
      // let the frictioncontroller evolve and inherit its state:
      fsc->evolve(q);
      sacstate = fsc->getState();
    }
  */
  // get ready for next time step:
  oldt = t;
}

// ######################################################################
void MonkeySaccadeController::computeWhenResetPos(SimEventQueue& q)
{
  // get our reset target:
  WTAwinner win = getPreviousPercept(0);

  // feed it to fsc:
  fsc->resetPos(win.p, q);

  // abort any saccade we may be executing:
  //////////setState(SACSTATE_FIX);
}

// ######################################################################
Point2D<int> MonkeySaccadeController::
computeWhenNewDecision(SaccadeState& sacstate, bool& blinkstate,
                       SimEventQueue& q)
{
  // since our realistic simulation directly drives the
  // FrictionController, it is always up to date, so return its
  // current position, or our current if fsc has nothing new for us:
  Point2D<int> p = fsc->getDecision(q);
  if (p.isValid()) return p; else return getPreviousDecision(0).p;
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

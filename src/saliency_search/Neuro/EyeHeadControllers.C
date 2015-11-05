/*!@file Neuro/EyeHeadControllers.C Eye/Head controllers */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/EyeHeadControllers.C $
// $Id: EyeHeadControllers.C 15310 2012-06-01 02:29:24Z itti $
//

#include "Neuro/EyeHeadControllers.H"
#include "Neuro/SaccadeControllers.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/SpatialMetrics.H"
#include "Component/OptionManager.H"
#include "Psycho/EyeTrace.H"
#include "Simulation/SimEventQueue.H"
#include "Util/StringConversions.H"
#include "Util/StringUtil.H"
#include "Util/sformat.H"

#include <vector>

// ######################################################################
// ######################################################################
// ######################################################################
StubEyeHeadController::StubEyeHeadController(OptionManager& mgr) :
  EyeHeadController(mgr, "Stub Eye/Head Controller",
                    "StubEyeHeadController")
{  }

// ######################################################################
StubEyeHeadController::~StubEyeHeadController()
{  }

// ######################################################################
// ######################################################################
// ######################################################################
SimpleEyeHeadController::SimpleEyeHeadController(OptionManager& mgr) :
  EyeHeadController(mgr, "Simple Eye/Head Controller",
                    "SimpleEyeHeadController"),
  SIMCALLBACK_INIT(SimEventClockTick),
  itsSCCeye(new SaccadeControllerEyeConfigurator(mgr)),
  itsSCChead(new SaccadeControllerHeadConfigurator(mgr)),
  itsSCeye(), itsSChead()
{
  addSubComponent(itsSCCeye);
  addSubComponent(itsSCChead);
}

// ######################################################################
SimpleEyeHeadController::~SimpleEyeHeadController()
{  }

// ######################################################################
void SimpleEyeHeadController::start1()
{
  // install our shortcuts:
  itsSCeye = itsSCCeye->getSC();
  itsSChead = itsSCChead->getSC();

  if (itsSCeye.is_invalid() || itsSChead.is_invalid())
    LFATAL("I need valid eye and head SaccadeControllers");

  EyeHeadController::start1();
}

// ######################################################################
void SimpleEyeHeadController::
onSimEventClockTick(SimEventQueue& q, rutz::shared_ptr<SimEventClockTick>& ect)
{
  // catch any WTAwinner and feed to eye:
  if (SeC<SimEventWTAwinner> e = q.check<SimEventWTAwinner>(this))
    itsSCeye->setPercept(e->winner(), q);

  // catch any SaccadeController eye status event:
  if (SeC<SimEventSaccadeStatusEye> e = q.check<SimEventSaccadeStatusEye>(this))
    {
      WTAwinner win(e->position(), q.now(), 0.0, false);
      itsSChead->setPercept(win, q);
    }

  // evolve our controllers; they will post events to let everyone
  // know what they are up to:
  itsSCeye->evolve(q);
  itsSChead->evolve(q);

  // force a computation of the new decisions and instruct our
  // controllers to post a status event to let everyone know what we
  // are up to:
  itsSCeye->getDecision(q, true);
  itsSChead->getDecision(q, true);
}

// ######################################################################
// ######################################################################
// ######################################################################
EyeTrackerEyeHeadController::EyeTrackerEyeHeadController(OptionManager& mgr) :
  EyeHeadController(mgr, "EyeTracker Eye/Head Controller",
                    "EyeTrackerEyeHeadController"),
  SIMCALLBACK_INIT(SimEventClockTick),
  itsConfig(&OPT_EHCeyeTrackConfig, this),
  itsEyeTrace(), itsEyeSample()
{  }

// ######################################################################
EyeTrackerEyeHeadController::~EyeTrackerEyeHeadController()
{  }

// ######################################################################
void EyeTrackerEyeHeadController::start1()
{
  // parse our config string and instantiate all our trackers:
  std::vector<std::string> tok;
  split(itsConfig.getVal(), ",", std::back_inserter(tok));
  if (tok.empty()) LFATAL("I cannot run without at least one eyetrace.");

  for (uint i = 0; i < tok.size(); i ++)
    {
      std::vector<std::string> tt;
      split(tok[i], ":", std::back_inserter(tt));
      if (tt.empty()) LFATAL("Invalid empty eye-tracker filename");

      std::string fname = tt[0];
      std::string extras = join(tt.begin() + 1, tt.end(), ":");

      LINFO("Instantiating Tracker %03d with file '%s', extras '%s'",
            i, fname.c_str(), extras.c_str());

      // the only extra we support for now is a color:
      PixRGB<byte> color(128, 255, 255);
      if (tt.size() > 1) convertFromString(tt[1], color);

      // instantiate a new EyeTrace object:
      rutz::shared_ptr<EyeTrace> et(new EyeTrace(fname, color));

      itsEyeTrace.push_back(et);
      itsEyeSample.push_back(0);
    }

  EyeHeadController::start1();
}

// ######################################################################
void EyeTrackerEyeHeadController::
onSimEventClockTick(SimEventQueue& q, rutz::shared_ptr<SimEventClockTick>& ect)
{
  const SimTime t = q.now();

  // evolve all our babies and post their data:
  bool keepgoing = true;
  while(keepgoing) // will go on until no tracker has any new data
    {
      keepgoing = false;

      // loop over our trackers:
      for (uint i = 0; i < itsEyeTrace.size(); i ++)
        if (itsEyeTrace[i]->hasData(itsEyeSample[i], t))
          {
            // ok, this warrants that we continue our while() loop:
            keepgoing = true;

            // get the next data sample:
            rutz::shared_ptr<EyeData> data = itsEyeTrace[i]->data(itsEyeSample[i]);

            CLDEBUG("Human eye %03u [%07" ZU "] (%d, %d) at %.1fms",
                    i, itsEyeSample[i], data->position().i, data->position().j,
                    itsEyeSample[i] * itsEyeTrace[i]->period().msecs());

            // post an event with it:
            q.post(rutz::make_shared(new SimEventEyeTrackerData(this, data, i, itsEyeTrace[i]->filename(), itsEyeTrace[i]->color(), itsEyeTrace[i]->ppd(),itsEyeTrace[i]->period())));
            
            // get the previous status info, if possible:
            SaccadeState prevSacState = SACSTATE_UNK;
            bool prevBlinkState = false;
            if (itsEyeSample[i] > 0) {
              rutz::shared_ptr<EyeData> prevEyeData = itsEyeTrace[i]->data(itsEyeSample[i] - 1);
              prevSacState = prevEyeData->saccadeState();
              prevBlinkState = prevEyeData->isInBlink();
            }

            // also send eye event; this one we will shift by any retinal input shift if available:
            Point2D<int> retinaleyepos = data->position();
            if (SeC<SimEventRetinaImage> e = q.check<SimEventRetinaImage>(this, SEQ_ANY))
              retinaleyepos = e->rawToRetinal(retinaleyepos);

            q.post(rutz::make_shared(new SimEventSaccadeStatusEye(this, retinaleyepos, data->saccadeState(),
                                                                  prevSacState, data->isInBlink(), prevBlinkState)));

            // ready for next eye movement sample:
            ++ itsEyeSample[i];
          }
    }
}

// ######################################################################
// ######################################################################
// ######################################################################
MonkeyEyeHeadController::
MonkeyEyeHeadController(OptionManager& mgr) :
  EyeHeadController(mgr, "Monkey Eye/Head Controller", "MonkeyEyeHeadController"),
  SIMCALLBACK_INIT(SimEventClockTick),
  itsBlinkWait(&OPT_SCeyeBlinkWaitTime, this), // see Neuro/NeuroOpts.{H,C}
  itsBlinkDur(&OPT_SCeyeBlinkDuration, this), // see Neuro/NeuroOpts.{H,C}
  itsOdist(&OPT_SCeyeThreshMinOvert, this), // see Neuro/NeuroOpts.{H,C}
  itsMetrics(new SpatialMetrics(mgr)),
  itsTSC(new ThresholdSaccadeController(mgr, SaccadeBodyPartEye)),
  lastsbt(), blinkt()
{
  this->addSubComponent(itsMetrics);

  // set default eye and head saccade controllers:
  /////  itsSCCeye->setModelParamString("SaccadeControllerEyeType", "Monkey");
  //////  itsSCChead->setModelParamString("SaccadeControllerHeadType", "Monkey");

  this->addSubComponent(itsTSC);

  lastsbt = SimTime::ZERO(); blinkt = SimTime::ZERO();
}

// ######################################################################
MonkeyEyeHeadController::~MonkeyEyeHeadController()
{  }

// ######################################################################
void MonkeyEyeHeadController::
onSimEventClockTick(SimEventQueue& q, rutz::shared_ptr<SimEventClockTick>& ect)
{
  //const SimTime t = q.now();

  /*  // catch any WTAwinner and feed to eye and TSC:
  if (SeC<SimEventWTAwinner> e = q.check<SimEventWTAwinner>(this))
    this->setPercept(e->winner(), q);

  // evolve the thresholdfriction part of our business:
  itsTSC->evolve(q);
  Point2D<int> tsc = itsTSC->getDecision(q);

  // evolve the eye part of our business:
  itsSCeye->evolve(q);
  Point2D<int> eye = itsSCeye->getDecision(q);


  // did we just end our eye saccade?
  if (itsSCeye->saccadeStatus() == EVENT_END) lastsbt = t;

  // feed current eye position to head if it is different from the
  // last percept we have fed to the head:
  Point2D<int> headlast = itsSChead->getPreviousPercept(0).p;
  if (eye.isValid() && (eye.i != headlast.i || eye.j != headlast.j)) {
    WTAwinner win(eye, t, 0.0, false);
    itsSChead->setPercept(win, q);
  }

  // evolve the head part of our business:
  itsSChead->evolve(q);
  Point2D<int> head = itsSChead->getDecision(q);

  // are we blinking but time is up?
  if (itsSCeye->getBlinkState() == true && t - blinkt > itsBlinkDur.getVal())
    {
      LINFO("===== Ending eye blink at t=%.2fms =====", t.msecs());
      ////////////////      itsSCeye->endBlink(); lastsbt = t;
    }

  // do we want to blink?
  if (itsSCeye->getBlinkState() == false && itsSCeye->isInSaccade() == false &&
      t - lastsbt > itsBlinkWait.getVal())
    {
      // check whether our percepts are nicely clustered (no action going on)
      bool areclose; Point2D<int> avgp;
      itsTSC->checkPercepts(areclose, avgp);
      if (areclose)
        {
          // start a blink with very small probability, or with a higher
          // probability if the percepts are clustered, or for sure if it
          // has been a long time even though the percepts may not be
          // clustered:
          double r = randomDouble();
          if (r > 0.99995 ||
              (r > 0.9999 && areclose == true) ||
              (r > 0.999 && t - lastsbt > itsBlinkWait.getVal() * 2.0) ||
              (r > 0.9 && t - lastsbt > itsBlinkWait.getVal() * 3.0))
            {
              LINFO("===== Starting blink at t=%.2fms (last at %.2fms)=====",
                    t.msecs(), lastsbt.msecs());
              //////////////////////              itsSCeye->beginBlink(); blinkt = t;
            }
        }
    }
  */
}

// ######################################################################
void MonkeyEyeHeadController::setPercept(const WTAwinner& fix,
                                         SimEventQueue& q)
{
  // do the thresholdfriction part of our business:
  itsTSC->setPercept(fix, q);
  /*
  // get our current eye and head positions:
  Point2D<int> curreye = itsSCeye->getPreviousDecision(0).p;
  Point2D<int> currhead = itsSChead->getPreviousDecision(0).p;

  // if itsTSC recommends a saccade, let's execute it:
  Point2D<int> target = itsTSC->getDecision(q);
  if (target.i != -1)
    {
      const float sacthresh =
        float(itsMetrics->getFoveaRadius()) * itsOdist.getVal();

      // if the move recommended by itsTSC is large enough, and we are
      // not blinking, initiate a realistic saccade:
      if (target.distance(curreye) >= sacthresh &&
          itsSCeye->isInBlink() == false)
        {
          LINFO("=== Initiating saccade to (%d, %d) at %.2fms ===",
                target.i, target.j, fix.t.msecs());

          // find the eye and head contributions:
          double eyecx, eyecy, headcx, headcy, tpx, tpy;
          itsMetrics->pix2deg(curreye, eyecx, eyecy);
          itsMetrics->pix2deg(currhead, headcx, headcy);
          itsMetrics->pix2deg(target, tpx, tpy);

          // get horizontal head displacement:
          const double hampx = headAmplitude(eyecx, headcx, tpx);

          // get vertical head displacement:
          const double hampy = headAmplitude(eyecy, headcy, tpy);

          // now, we'll send the eyes directly to the target and we'll
          // send the head to an angular displacement of (hampx, hampy):
          ///////////////itsSCeye->saccade(Point2DT(target, fix.t));

          Point2D<int> htarget;
          itsMetrics->deg2pix(hampx + headcx, hampy + headcy, htarget);
          ///////////////itsSChead->saccade(Point2DT(htarget, fix.t));
        }
      else
        {
          LINFO("=== Smooth pursuit to (%d, %d) at %.2fms ===",
                target.i, target.j, fix.t.msecs());

          // we have a recommendation from itsTSC, but it's too small for
          // a realistic saccade. Just use it as friction targets:
          WTAwinner win(fix); // new target for our eye controller
          win.p = target;     // use target location from itsTSC as percept loc
          itsSCeye->setPercept(win, q);

          // the head follows the eye:
          win.p = itsSCeye->getPreviousDecision(0).p;
          itsSChead->setPercept(win, q);
        }
    }
  else
    {
      // we have no recommendation from itsTSC

      // set friction targets: the eye follows the moving average if
      // the recent percepts are nicely clustered, otherwise the eyes
      // just don't move:
      bool areclose; Point2D<int> avgp;
      WTAwinner win(fix);
      itsTSC->checkPercepts(areclose, avgp);
      if (areclose)
        {
          LINFO("=== Smooth pursuit to moving avg (%d, %d) at %.2fms ===",
                avgp.i, avgp.j, win.t.msecs());
          win.p = avgp;     // use target location from itsTSC as percept loc
          itsSCeye->setPercept(win, q);
        }

      // the head follows the eye:
      win.p = itsSCeye->getPreviousDecision(0).p;
      itsSChead->setPercept(win, q);
    }
  */
}
/*
// ######################################################################
Point2D<int> MonkeyEyeHeadController::getEyeDecision(const SimTime& t)
{

  // get current eye position:
  Point2D<int> eye = itsSCeye->getDecision(q);  // run decision on itsSCeye

  // feed the current eye position as new percept to the head
  // controller, if it is differemt from what the head controller
  // already has as its last percept:
  Point2D<int> headlast = itsSChead->getPreviousPercept(0).p;
  if (eye.i != -1 && (eye.i != headlast.i || eye.j != headlast.j)) {
    WTAwinner win(eye, t, 0.0, false);
    itsSChead->setPercept(win, q);
  }

  // we only return eye movements here, so it's whatever itsSCeye says:
  return eye;
}

// ######################################################################
Point2D<int> MonkeyEyeHeadController::getHeadDecision(const SimTime& t)
{
  // get current head position:
  Point2D<int> head = itsSChead->getDecision(q);   // run decision on itsSChead

  if (head.i != -1)
    LINFO("### head at (%d, %d) target (%d, %d) ###", head.i, head.j,
          itsSChead->getPreviousPercept(0).p.i,
          itsSChead->getPreviousPercept(0).p.j);

  return head;
}
*/
// ######################################################################
double MonkeyEyeHeadController::headAmplitude(const double curreye,
                                               const double currhead,
                                               const double target)
{
  // This implementation follows Freedman, Biol Cybern, 2001.

  // initial eye position with respect to the head. Sign is positive
  // if the eyes begin deviated in the direction of the subsequent
  // movement, negative otherwise:
  double iep = (curreye - currhead) * signOf(target - curreye);

  // compute desired gaze displacement:
  double hgazedd = target - curreye;

  // compute head displacement dead zone max boundary (note: we here
  // deviate from Freedman's formulation by clamping out negative
  // values):
  double hhdz = 0.56 * (20.0 - 0.5 * iep); if (hhdz < 0.0) hhdz = 0.0;

  // if desired gaze displacement in dead zone, do not move the head,
  // otherwise compute slope hslh of tradeoff between dead zone
  // boundary and actual gaze displacement amplitude, and derive head
  // displacement:
  double hheaddd = 0.0, hslh = 0.0;

  if (fabs(hgazedd) >= hhdz)
    {
      // I think Freedman forgot to take the absolute value of iep, so
      // we do that here. Also we'll make sure the slope is smaller
      // than 1, as otherwise Freeman's equation for hheaddd yields
      // weird results. Finally, let's also make sure both terms of
      // the tradeoff have the sign of hgazedd:
      hslh = 0.65 * (fabs(iep) / 35.0 + 1.1); if (hslh > 1.0) hslh = 1.0;
      hheaddd = hgazedd * hslh + hhdz * (1.0 - hslh) * signOf(hgazedd);
    }

  LINFO("curreye=%f currhead=%f target=%f iep=%f hgazedd=%f hhdz=%f "
        "hslh=%f hheaddd=%f", curreye, currhead, target, iep, hgazedd, hhdz,
        hslh, hheaddd);
  return hheaddd;
}



// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

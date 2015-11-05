/*!@file Neuro/SaccadeController.C Base class for saccade generation */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SaccadeController.C $
// $Id: SaccadeController.C 9412 2008-03-10 23:10:15Z farhan $
//

#include "Neuro/SaccadeController.H"

#include "Simulation/SimEventQueue.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Util/log.H"
#include "Util/sformat.H"

// ######################################################################
SaccadeController::SaccadeController(OptionManager& mgr,
                                     const std::string& descrName,
                                     const std::string& tagName,
                                     const SaccadeBodyPart bodypart,
                                     const int percept_qlen,
                                     const int decision_qlen) :
  SimModule(mgr, std::string(saccadeBodyPartName(bodypart)) +
            std::string(" ") + descrName,
            std::string(saccadeBodyPartName(bodypart)) + tagName),
  itsInitialPosition(bodypart == SaccadeBodyPartEye
                     ? &OPT_SCeyeInitialPosition
                     : &OPT_SCheadInitialPosition, this),
  percept(), decision(), pqlen(percept_qlen), dqlen(decision_qlen),
  itsState(SACSTATE_UNK), itsPrevState(SACSTATE_UNK),
  itsBlinkState(false), itsPrevBlinkState(false),
  itsBodyPart(bodypart)
{
  if (pqlen < 2 || dqlen < 2)
    LFATAL("Queue lengths must be at least 2 for proper operation.");
}

// ######################################################################
SaccadeController::~SaccadeController()
{  }

// ######################################################################
void SaccadeController::reset1()
{
  // reset some stuff for SaccadeController
  percept.clear();          decision.clear();
  itsState = SACSTATE_UNK;  itsPrevState = SACSTATE_UNK;
  itsBlinkState = false;    itsPrevBlinkState = false;

  // propagate to our base class:
  SimModule::reset1();
}

// ######################################################################
void SaccadeController::setPercept(const WTAwinner& fix, SimEventQueue& q)
{
  // push new percept to front of percept queue:
  percept.push_front(fix);

  // truncate percept queue if necessary:
  while(percept.size() > pqlen) percept.pop_back();

  // do some processing:
  computeWhenNewPercept(q);
}

// ######################################################################
void SaccadeController::resetPos(const Point2D<int>& p, SimEventQueue& q)
{
  // push new percept to front of percept queue:
  const WTAwinner win(p, q.now(), 0.0, false);
  percept.push_front(win);

  // truncate percept queue if necessary:
  while(percept.size() > pqlen) percept.pop_back();

  // do some processing:
  computeWhenResetPos(q);

  // If the reset location is valid, then we are in fixation,
  // otherwise we are in unknown state:
  if (p.isValid()) itsState = SACSTATE_FIX;
  else itsState = SACSTATE_UNK;

  // also always abort any blink:
  itsBlinkState = false;

  CLDEBUG("### State reset to [%s] (%d,%d) %.1fms ###",
          saccadeStateName(itsState), p.i, p.j, q.now().msecs());
  CLDEBUG("## Blink state reset to [Not-Blinking] (%d,%d) %.1fms ##",
          p.i, p.j, q.now().msecs());
}

// ######################################################################
Point2D<int> SaccadeController::getDecision(SimEventQueue& q,
                                       const bool dopoststatus)
{
  // if we have no percepts yet, we may want to push an initial
  // position in depending on command-line option:
  if (percept.empty() && q.now() == SimTime::ZERO())
    {
      const Point2D<int> p = itsInitialPosition.getVal();
      if (p.i == -2 && p.j == -2)
        {
          // we want to start at center; get the latest input frame to
          // figure oput where the center is:
          if (SeC<SimEventRetinaImage> e =
              q.check<SimEventRetinaImage>(this, SEQ_ANY))
            {
              const Point2D<int> c = e->center();
              resetPos(c, q);
              CLINFO("Starting %s at centered initial position (%d, %d).",
                     saccadeBodyPartName(itsBodyPart), c.i, c.j);
            }
          else
            CLFATAL("ooops, cannot find a retina image to get its center?");
        }
      else if (p.isValid())
        {
          // we want to start at a specified position:
          resetPos(p, q);
          CLINFO("Starting %s at specified initial position (%d, %d).",
                 saccadeBodyPartName(itsBodyPart), p.i, p.j);
        }
      else
        CLINFO("Starting %s at unspecified initial position.",
               saccadeBodyPartName(itsBodyPart));
    }

  // do some computation and push a new decision into decision queue,
  // if the new decision is different from the last one pushed into
  // the queue. This is also where we update our saccade and blink
  // state information:
  Point2D<int> last(-1, -1);
  if (decision.size()) last = decision.front().p;
  Point2D<int> loc = computeWhenNewDecision(itsState, itsBlinkState, q);
  bool nochange = (loc == last);
  if (loc.isValid() && nochange == false)
    decision.push_front(Point2DT(loc, q.now()));

  // truncate decision queue as necessary:
  while(decision.size() > dqlen) decision.pop_back();

  // display a few messages whenever we have a state change:
  if (itsState != itsPrevState)
    CLDEBUG("### State change [%s -> %s] at (%d,%d) %.1fms ###",
            saccadeStateName(itsPrevState), saccadeStateName(itsState),
            loc.i, loc.j, q.now().msecs());
  if (itsBlinkState != itsPrevBlinkState)
    CLDEBUG("## Blink state change [%s -> %s] at (%d,%d) %.1fms ##",
            itsPrevBlinkState ? "Blinking" : "Not-Blinking",
            itsBlinkState ? "Blinking" : "Not-Blinking",
            loc.i, loc.j, q.now().msecs());

  // post our status to the SimEventQueue?
  if (dopoststatus)
    {
      if (itsBodyPart == SaccadeBodyPartEye)
        {
          rutz::shared_ptr<SimEventSaccadeStatusEye>
            s(new SimEventSaccadeStatusEye(this, getPreviousDecision(0).p,
                                           itsState, itsPrevState,
                                           itsBlinkState, itsPrevBlinkState));
          q.post(s);
        }
      else
        {
          rutz::shared_ptr<SimEventSaccadeStatusHead>
            s(new SimEventSaccadeStatusHead(this, getPreviousDecision(0).p,
                                            itsState, itsPrevState,
                                            itsBlinkState, itsPrevBlinkState));
          q.post(s);
        }
    }

  // return current decision or (-1, -1) if no change since last time:
  if (nochange) return Point2D<int>(-1, -1); else return loc;
}

// ######################################################################
void SaccadeController::dumpQueues() const
{
  std::string ptxt, dtxt;
  for (unsigned int i = 0; i < percept.size(); i ++) {
    WTAwinner win = percept[i];
    ptxt += sformat("[(%d, %d) %.2fms %.2fmV] ", win.p.i, win.p.j,
                    win.t.msecs(), win.sv * 1000.0);
  }
  CLINFO("PERCEPT:  %s", ptxt.c_str());
  for (unsigned int i = 0; i < decision.size(); i ++) {
    Point2DT pt = decision[i];
    dtxt += sformat("[(%d, %d) %.2fms] ", pt.p.i, pt.p.j, pt.t.msecs());
  }
  CLINFO("DECISION: %s", dtxt.c_str());
}

// ######################################################################
WTAwinner SaccadeController::getPreviousPercept(const unsigned int
                                                index) const
{
  if (percept.size() > index) return percept[index];
  else return WTAwinner::NONE();
}

// ######################################################################
bool SaccadeController::havePercepts() const
{ return !(percept.empty()); }

// ######################################################################
int SaccadeController::getPqlen() const
{ return pqlen; }

// ######################################################################
void SaccadeController::resetPqlen(const int len)
{ percept.clear(); pqlen = len; }

// ######################################################################
void SaccadeController::killPercepts()
{ while(percept.size()) percept.pop_front(); }

// ######################################################################
Point2DT SaccadeController::getPreviousDecision(const unsigned int
                                                index) const
{
  if (decision.size() > index) return decision[index];
  else return Point2DT(-1, -1, SimTime::ZERO());
}

// ######################################################################
bool SaccadeController::haveDecisions() const
{ return !(decision.empty()); }

// ######################################################################
int SaccadeController::getDqlen() const
{ return dqlen; }

// ######################################################################
void SaccadeController::resetDqlen(const int len)
{ decision.clear(); dqlen = len; }

// ######################################################################
void SaccadeController::killDecisions()
{ while(decision.size()) decision.pop_front(); }

// ######################################################################
SaccadeBodyPart SaccadeController::bodyPart() const
{ return itsBodyPart; }

// ######################################################################
void SaccadeController::evolve(SimEventQueue& q)
{
  // keep track of our status from the last cycle:
  itsPrevState = itsState;
  itsPrevBlinkState = itsBlinkState;

  // call the virtual function that will be implemented by derived classes:
  this->doEvolve(q);
}

// ######################################################################
SaccadeState SaccadeController::getState() const
{ return itsState; }

// ######################################################################
bool SaccadeController::getBlinkState() const
{ return itsBlinkState; }


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

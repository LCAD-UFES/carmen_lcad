/*!@file Neuro/WinnerTakeAll.C 2D winner-take-all network */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/WinnerTakeAll.C $
// $Id: WinnerTakeAll.C 13065 2010-03-28 00:01:00Z itti $
//

#include "Neuro/WinnerTakeAll.H"

#include "Channels/ChannelOpts.H" // for OPT_LevelSpec
#include "Component/GlobalOpts.H"
#include "Component/OptionManager.H"
#include "Image/Image.H"
#include "Image/MathOps.H"
#include "Image/Transforms.H" // for chamfer34()
#include "Neuro/NeuroOpts.H"
#include "Neuro/WTAwinner.H"
#include "Neuro/NeuroSimEvents.H"
#include "Simulation/SimEventQueue.H"
#include "Simulation/SimEvents.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/TextLog.H"
#include "Util/log.H"
#include "Util/sformat.H"
#include "rutz/trace.h"

// ######################################################################
// ######################################################################
// ########## WinnerTakeAll implementation
// ######################################################################
// ######################################################################

WinnerTakeAll::WinnerTakeAll(OptionManager& mgr,
                             const std::string& descrName,
                             const std::string& tagName) :
  SimModule(mgr, descrName, tagName)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
WinnerTakeAll::~WinnerTakeAll()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
// ######################################################################
// ########## WinnerTakeAllStub implementation
// ######################################################################
// ######################################################################

WinnerTakeAllStub::WinnerTakeAllStub(OptionManager& mgr,
                                     const std::string& descrName,
                                     const std::string& tagName) :
  WinnerTakeAll(mgr, descrName, tagName)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
WinnerTakeAllStub::~WinnerTakeAllStub()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
// ######################################################################
// ########## WinnerTakeAllAdapter implementation
// ######################################################################
// ######################################################################

WinnerTakeAllAdapter::WinnerTakeAllAdapter(OptionManager& mgr,
                                           const std::string& descrName,
                                           const std::string& tagName) :
  WinnerTakeAll(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventAttentionGuidanceMapOutput),
  SIMCALLBACK_INIT(SimEventSaccadeStatusEye),
  SIMCALLBACK_INIT(SimEventInputFrame),
  SIMCALLBACK_INIT(SimEventSaveOutput),
  itsLogFile(&OPT_TextLogFile, this),
  itsSaveResults(&OPT_WTAsaveResults, this), // see Neuro/NeuroOpts.{H,C}
  itsUseSaccadicSuppression(&OPT_WTAuseSacSuppress, this),
  itsUseBlinkSuppression(&OPT_WTAuseBlinkSuppress, this),
  itsLevelSpec(&OPT_LevelSpec, this),
  itsUseRandom(&OPT_UseRandom, this), // see Component/ModelManager.{H,C}
  itsTooManyShifts(&OPT_BrainTooManyShifts, this), // Neuro/NeuroOpts.{H,C}
  itsTooManyShiftsPerFrame(&OPT_BrainTooManyShiftsPerFrame, this), // Neuro/NeuroOpts.{H,C}
  itsBoringDelay(&OPT_BrainBoringDelay, this),
  itsBoringSMmv(&OPT_BrainBoringSMmv, this),
  itsNumShifts(0),
  itsInputCopy(),
  itsEyePos(-1, -1),
  itsLastWinner(WTAwinner::NONE())
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
WinnerTakeAllAdapter::~WinnerTakeAllAdapter()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void WinnerTakeAllAdapter::reset1()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsNumShifts = 0;
  itsInputCopy.freeMem();
  itsEyePos = Point2D<int>(-1, -1);
  WinnerTakeAll::reset1();
}

// ######################################################################
void WinnerTakeAllAdapter::
onSimEventInputFrame(SimEventQueue& q, rutz::shared_ptr<SimEventInputFrame>& e)
{
  itsNumShifts = 0; // Reset at each new frame
}

// ######################################################################
void WinnerTakeAllAdapter::
onSimEventAttentionGuidanceMapOutput(SimEventQueue& q, rutz::shared_ptr<SimEventAttentionGuidanceMapOutput>& e)
{
  // process the input:
  itsInputCopy = e->agm();
  this->input(itsInputCopy);

  // evolve our internals:
  Point2D<int> winner(-1,-1);
  this->integrate(q.now(), winner);

  // stop here if we have already too many shifts per frame:
  if (itsTooManyShiftsPerFrame.getVal() > 0 &&  itsNumShifts >= itsTooManyShiftsPerFrame.getVal()) return;

  // did we just get a winner?
  if (winner.isValid())
    {
      WTAwinner newwin = WTAwinner::buildFromSMcoords(winner, itsLevelSpec.getVal().mapLevel(),
                                                      itsUseRandom.getVal(), q.now(),
                                                      itsInputCopy.getVal(winner), false);

      // if the last covert attention shift was slow or the SM voltage
      // was low, mark the covert attention shift as boring:
      if ( (itsLastWinner.t > SimTime::ZERO() && newwin.t - itsLastWinner.t > itsBoringDelay.getVal() ) ||
           newwin.sv < itsBoringSMmv.getVal() * 0.001)
        newwin.boring = true;

      LINFO("##### WinnerTakeAll winner (%d,%d) at %fms %s#####",
            newwin.p.i, newwin.p.j, newwin.t.msecs(), newwin.boring ? "[boring] ":"");

      // log what happened:
      textLog(itsLogFile.getVal(), "CovertShift",
              sformat("(%d,%d) %.3fmV", newwin.p.i, newwin.p.j, newwin.sv * 1000.0F), newwin.t);
      itsLastWinner = newwin;

      // post an event so that anyone interested in the winner can grab it:
      q.post(rutz::make_shared(new SimEventWTAwinner(this, newwin, itsNumShifts)));

      // also notify that if we have a SimOutputFrameSeries that is
      // operating in event-driven mode (no framerate was specified on
      // the command-line), this covert shift is a good reason to save
      // outputs now:
      q.post(rutz::make_shared(new SimEventRequestSaveOutput(this)));

      // that's one more shift of attention:
      ++itsNumShifts;

      // too many shifts of attention?
      if (itsTooManyShifts.getVal() > 0 && itsNumShifts >= itsTooManyShifts.getVal())
      {
    	LINFO("#### ! Too many attention shifts ! ####");
        q.post(rutz::make_shared(new SimEventBreak(this, "Too many attention shifts")));
      }
    }
}

// ######################################################################
void WinnerTakeAllAdapter::
onSimEventSaccadeStatusEye(SimEventQueue& q, rutz::shared_ptr<SimEventSaccadeStatusEye>& e)
{
  this->eyeMovement(e->position());

  // should we also turn saccadic suppression on/off?
  TransientStatus evs = e->saccadeStatus();
  if (evs == TSTATUS_BEGIN) this->saccadicSuppression(true);
  else if (evs == TSTATUS_END) this->saccadicSuppression(false);

  // how about blink suppression?
  TransientStatus evb = e->blinkStatus();
  if (evb == TSTATUS_BEGIN) this->blinkSuppression(true);
  else if (evb == TSTATUS_END) this->blinkSuppression(false);
}

// ######################################################################
void WinnerTakeAllAdapter::eyeMovement(const Point2D<int>& curreye)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsEyePos = curreye;
}

// ######################################################################
void WinnerTakeAllAdapter::
onSimEventSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaveOutput>& e)
{
  if (itsSaveResults.getVal())
    {
      // get the OFS to save to, assuming sinfo is of type
      // SimModuleSaveInfo (will throw a fatal exception otherwise):
      nub::ref<FrameOstream> ofs =
        dynamic_cast<const SimModuleSaveInfo&>(e->sinfo()).ofs;
      ofs->writeFloat(this->getV(), FLOAT_NORM_PRESERVE, "WTA",
                      FrameInfo("winner-take-all map", SRC_POS));
    }
}

// ######################################################################
// ######################################################################
// ########## WinnerTakeAllStd implementation
// ######################################################################
// ######################################################################

WinnerTakeAllStd::WinnerTakeAllStd(OptionManager& mgr,
                                   const std::string& descrName,
                                   const std::string& tagName) :
  WinnerTakeAllAdapter(mgr, descrName, tagName),
  itsNeurons(),
  itsGIN(), itsT(), itsGleak(1.0e-8F), itsGinh(1.0e-2F), itsGinput(5.0e-8F)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsGIN.setGleak(itsGleak);
}

// ######################################################################
WinnerTakeAllStd::~WinnerTakeAllStd()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void WinnerTakeAllStd::reset1()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsNeurons.freeMem();
  itsGleak = 1.0e-8F;
  itsGinh = 1.0e-2F;
  itsGinput = 5.0e-8F;
  itsT = SimTime::ZERO();

  WinnerTakeAllAdapter::reset1();
}

// ######################################################################
void WinnerTakeAllStd::input(const Image<float>& in)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (itsNeurons.initialized() == false) {
    // first input, let's initialize our array
    itsNeurons.resize(in.getDims(), ZEROS);

    Image<LeakyIntFire>::iterator
      nptr = itsNeurons.beginw(), stop = itsNeurons.endw();
    while (nptr != stop)
      {
        nptr->setGleak(itsGleak);
        nptr->setG(0.0F, itsGinh);
        ++nptr;
      }
  }
}

// ######################################################################
void WinnerTakeAllStd::integrate(const SimTime& t, Point2D<int>& winner)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  winner.i = -1;
  const int w = itsNeurons.getWidth();
  const int h = itsNeurons.getHeight();

  // the array of neurons receive excitatory inputs from outside.
  // here we update the inputs and let the neurons evolve; here we
  // need to run this loop time step by time step, since this is a
  // feedback network, so we have code similar to that in
  // LeakyIntFire::evolve() to figure out how many time steps to run:
  const SimTime dt =
    SimTime::computeDeltaT((t - itsT), itsGIN.getTimeStep());

  for (SimTime tt = itsT; tt < t; tt += dt)
    {
      Image<LeakyIntFire>::iterator nptr = itsNeurons.beginw();
      Image<float>::const_iterator inptr = itsInputCopy.begin();
      for (int j = 0 ; j < h; j ++)
        for (int i = 0; i < w; i ++)
          {
            nptr->input(itsGinput * (*inptr++));
            if (nptr->integrate(tt)) { winner.i = i; winner.j = j; }
            nptr->setG(0.0F, 0.0F);  // only leak conductance
            ++nptr;
          }

      // if there is a winner, the winner triggers the global inhibition:
      if (winner.i > -1) itsGIN.setG(itsGleak * 10.0F, 0.0F);

      // when the global inhibition fires, it triggers inhibitory
      // conductances for one time step in the array of excited
      // neurons, shuts off excitatory conductances, and turns itself
      // off:
      if (itsGIN.integrate(tt)) inhibit();
    }
  itsT = t;
}

// ######################################################################
Image<float> WinnerTakeAllStd::getV() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<float> result(itsNeurons.getDims(), NO_INIT);

  Image<float>::iterator dptr = result.beginw(), stop = result.endw();
  Image<LeakyIntFire>::const_iterator nptr = itsNeurons.begin();
  while(dptr != stop) *dptr++ = (nptr++)->getV();

  return result;
}

// ######################################################################
void WinnerTakeAllStd::inhibit()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<LeakyIntFire>::iterator
    nptr = itsNeurons.beginw(), stop = itsNeurons.endw();
  while(nptr != stop) (nptr++)->setG(0.0F, itsGinh);
  itsGIN.setG(0.0F, 0.0F);
  LDEBUG("WTA inhibition firing...");
}

// ######################################################################
void WinnerTakeAllStd::saccadicSuppression(const bool on)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (itsUseSaccadicSuppression.getVal() == false) return;
  if (on) inhibit();
  LINFO("------- WTA saccadic suppression %s -------", on ? "on":"off");
}

// ######################################################################
void WinnerTakeAllStd::blinkSuppression(const bool on)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (itsUseBlinkSuppression.getVal() == false) return;
  if (on) inhibit();
  LINFO("------- WTA blink suppression %s -------", on ? "on":"off");
}

// ######################################################################
// ######################################################################
// ########## WinnerTakeAllFast implementation
// ######################################################################
// ######################################################################

WinnerTakeAllFast::WinnerTakeAllFast(OptionManager& mgr,
                                     const std::string& descrName,
                                     const std::string& tagName) :
  WinnerTakeAllAdapter(mgr, descrName, tagName)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
WinnerTakeAllFast::~WinnerTakeAllFast()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void WinnerTakeAllFast::input(const Image<float>& in)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void WinnerTakeAllFast::integrate(const SimTime& t, Point2D<int>& winner)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  float maxval;
  findMax(itsInputCopy, winner, maxval);
  if (maxval <= 1.0e-20F) { winner.i = -1; winner.j = -1; }

}

// ######################################################################
Image<float> WinnerTakeAllFast::getV() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsInputCopy;
}

// ######################################################################
void WinnerTakeAllFast::saccadicSuppression(const bool on)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (itsUseSaccadicSuppression.getVal() == false) return;
  if (on) itsInputCopy.clear();
  LINFO("------- WTA saccadic suppression %s -------", on ? "on":"off");
}

// ######################################################################
void WinnerTakeAllFast::blinkSuppression(const bool on)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (itsUseBlinkSuppression.getVal() == false) return;
  if (on) itsInputCopy.clear();
  LINFO("------- WTA blink suppression %s -------", on ? "on":"off");
}

// ######################################################################
// ######################################################################
// ########## WinnerTakeAllGreedy implementation
// ######################################################################
// ######################################################################

WinnerTakeAllGreedy::WinnerTakeAllGreedy(OptionManager& mgr,
                                         const std::string& descrName,
                                         const std::string& tagName) :
  WinnerTakeAllStd(mgr, descrName, tagName),
  itsThresholdFac(&OPT_WinnerTakeAllGreedyThreshFac, this)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
WinnerTakeAllGreedy::~WinnerTakeAllGreedy()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void WinnerTakeAllGreedy::integrate(const SimTime& t, Point2D<int>& winner)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (itsEyePos.isValid() == false) CLFATAL("I need a valid eyepos");

  winner.i = -1;
  const int w = itsNeurons.getWidth();
  const int h = itsNeurons.getHeight();

  // the array of neurons receive excitatory inputs from outside.
  // here we update the inputs and let the neurons evolve; here we
  // need to run this loop time step by time step, since this is a
  // feedback network, so we have code similar to that in
  // LeakyIntFire::evolve() to figure out how many time steps to run:
  const SimTime dt =
    SimTime::computeDeltaT((t - itsT), itsGIN.getTimeStep());

  for (SimTime tt = itsT; tt < t; tt += dt)
    {
      Image<LeakyIntFire>::iterator nptr = itsNeurons.beginw();
      Image<float>::const_iterator inptr = itsInputCopy.begin();
      for (int j = 0 ; j < h; j ++)
        for (int i = 0; i < w; i ++)
          {
            nptr->input(itsGinput * (*inptr++));
            if (nptr->integrate(tt)) { winner.i = i; winner.j = j; }
            nptr->setG(0.0F, 0.0F);  // only leak conductance
            ++nptr;
          }

      // if there is a winner, the winner triggers the global
      // inhibition; but before that let's replace our true WTA winner
      // by whoever is above threshold and closest to current eye
      // position:
      if (winner.i > -1)
        {
          // get our current voltages:
          Image<float> v = getV();

          // do not consider values below threshold:
          float thresh = v.getVal(winner) * itsThresholdFac.getVal();

          // scale the current eye position down to salmap level coords:
          Point2D<int> eye = itsEyePos;
          const int smlevel = itsLevelSpec.getVal().mapLevel();
          eye.i = int(eye.i / double(1 << smlevel) + 0.49);
          eye.j = int(eye.j / double(1 << smlevel) + 0.49);

          // if eye out of image (or unknown), we will not do our
          // special greedy processing here and instead will just act
          // like a normal WTA:
          if (v.coordsOk(eye) && (eye.i != 0 || eye.j != 0))
            {
              // create distance map from a single pixel at our eye position:
              Image<float> dmap(v.getDims(), ZEROS);
              dmap.setVal(eye, 100.0f);
              dmap = chamfer34(dmap, 1000.0f);

              // let's scan the dmap and find the closest location
              // that is a local max and is above threshold:
              const int w = v.getWidth(), h = v.getHeight();
              float bestd = 1000.0f; Point2D<int> p;

              for (p.j = 0; p.j < h; p.j ++)
                for (p.i = 0; p.i < w; p.i ++)
                  if (v.getVal(p) >= thresh &&  // we are above threshold
                      dmap.getVal(p) < bestd && // we are closer to eye
                      isLocalMax(v, p) == true)  // we are a local max
                    {
                      winner.i = p.i;
                      winner.j = p.j;
                      bestd = dmap.getVal(p);
                    }
            }

          // trigger global inhibition:
          itsGIN.setG(itsGleak * 10.0F, 0.0F);
        }

      // when the global inhibition fires, it triggers inhibitory
      // conductances for one time step in the array of excited
      // neurons, shuts off excitatory conductances, and turns itself
      // off:
      if (itsGIN.integrate(tt)) inhibit();
    }
  itsT = t;
}

// ######################################################################
// ######################################################################
// ########## WinnerTakeAll Temporal Noticing  implementation
// ######################################################################
// ######################################################################

WinnerTakeAllTempNote::WinnerTakeAllTempNote(OptionManager& mgr,
                                   const std::string& descrName,
                                   const std::string& tagName) :
  WinnerTakeAllStd(mgr, descrName, tagName),
  itsNeurons(),
  itsGIN(), itsT(), itsGleak(1.0e-8F), itsGinh(1.0e-2F), itsGinput(5.0e-8F)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsGIN.setGleak(itsGleak);
  itsHighMaskBound.resize(1,256.0F);
  itsLowMaskBound.resize(1,0.0002F);
}

// ######################################################################
WinnerTakeAllTempNote::~WinnerTakeAllTempNote()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void WinnerTakeAllTempNote::reset1()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsNeurons.freeMem();
  itsGleak = 1.0e-8F;
  itsGinh = 1.0e-2F;
  itsGinput = 5.0e-8F;
  itsT = SimTime::ZERO();

  WinnerTakeAllStd::reset1();
}

// ######################################################################
void WinnerTakeAllTempNote::input(const Image<float>& in)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (itsNeurons.initialized() == false) {
    // first input, let's initialize our array
    itsNeurons.resize(in.getDims(), ZEROS);

    Image<LeakyIntFireAdp>::iterator
      nptr = itsNeurons.beginw(), stop = itsNeurons.endw();
    while (nptr != stop)
    {
      nptr->setGleak(itsGleak);
      nptr->setG(0.0F, itsGinh);
      ++nptr;
    }

    int width  = in.getWidth();
    int height = in.getHeight();

    itsInitMask.resize(width,height);

    // set up mask segmenting
    itsMaskSegment.SIsetFrame(&width,&height);
    itsMaskSegment.SIsetValThresh(itsHighMaskBound,itsLowMaskBound);
    itsMaskSegment.SIsetAvg(1);
    itsMaskSegment.SItoggleCandidateBandPass(false);
  }
}

// ######################################################################
void WinnerTakeAllTempNote::integrate(const SimTime& t, Point2D<int>& winner)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  winner.i = -1;
  const int w = itsNeurons.getWidth();
  const int h = itsNeurons.getHeight();

  // the array of neurons receive excitatory inputs from outside.
  // here we update the inputs and let the neurons evolve; here we
  // need to run this loop time step by time step, since this is a
  // feedback network, so we have code similar to that in
  // LeakyIntFire::evolve() to figure out how many time steps to run:
  const SimTime dt =
    SimTime::computeDeltaT((t - itsT), itsGIN.getTimeStep());

  for (SimTime tt = itsT; tt < t; tt += dt)
  {
    Image<LeakyIntFireAdp>::iterator nptr = itsNeurons.beginw();
    Image<float>::const_iterator inptr    = itsInputCopy.begin();
    for (int j = 0 ; j < h; j ++)
    {
      for (int i = 0; i < w; i ++)
      {
        nptr->input(itsGinput * (*inptr++));
        if (nptr->integrate(tt)) { winner.i = i; winner.j = j; }
        nptr->setG(0.0F, 0.0F);  // only leak conductance
        ++nptr;
      }
    }

    // if there is a winner, the winner triggers the global inhibition:
    if (winner.i > -1)
    {
      itsGIN.setG(itsGleak * 10.0F, 0.0F);
    }

    if (winner.i > -1)
    {
      const float Vfire = itsNeurons.getVal(winner.i,winner.j).getVfire();
      const float V = itsNeurons.getVal(winner.i,winner.j).getV();
      LINFO("VFIRE IS %f",Vfire);
      LINFO("V is %f",V);
      std::vector<Image<float> > sal_input(1, itsInputCopy);

      // find out which neurons around the winner are salient based
      // on their salmap values
      itsMaskSegment.SIsegment(&sal_input,true);
      Image<long> maskCandidates = itsMaskSegment.SIreturnBlobs();
      const long winnerID = maskCandidates.getVal(winner.i, winner.j);

      itsInitMask = itsMaskSegment.SIreturnNormalizedCandidates();

      // for each salient pixel/neuron that is grouped with the winner
      // update its threshold fire value
      Image<long>::iterator  imask = maskCandidates.beginw();
      Image<float>::iterator isal  = itsInputCopy.beginw();
      Image<LeakyIntFireAdp>::iterator nptr = itsNeurons.beginw();
      for (int j = 0 ; j < h; j ++)
      {
        for (int i = 0; i < w; i ++)
        {
          if(*imask == winnerID)
          {
            LINFO("WINNER %d, %d - SLAVE %d, %d - ID %d",winner.i,winner.j,i,j
                  ,(int)winnerID);
            const float dist = sqrt(pow(winner.i - i,2) + pow(winner.j - j,2));
            nptr->setNewVth(*isal,Vfire,dist);
          }
          ++imask; ++isal; ++nptr;
        }
      }
    }

    // when the global inhibition fires, it triggers inhibitory
    // conductances for one time step in the array of excited
    // neurons, shuts off excitatory conductances, and turns itself
    // off:
    if (itsGIN.integrate(tt))
    {
      inhibit();
    }
  }
  itsT = t;
}

// ######################################################################
Image<float> WinnerTakeAllTempNote::getV() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<float> result(itsNeurons.getDims(), NO_INIT);

  Image<float>::iterator dptr = result.beginw(), stop = result.endw();
  Image<LeakyIntFireAdp>::const_iterator nptr = itsNeurons.begin();
  while(dptr != stop) *dptr++ = (nptr++)->getV();

  return result;
}

// ######################################################################
Image<float> WinnerTakeAllTempNote::getVth(const bool normalize) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<float> result(itsNeurons.getDims(),NO_INIT);

  Image<float>::iterator dptr = result.beginw(), stop = result.endw();
  Image<LeakyIntFireAdp>::const_iterator nptr = itsNeurons.begin();
  while(dptr != stop)
  {
    *dptr = (nptr)->getVth();
    ++dptr; ++nptr;
  }

  if (normalize)
    inplaceNormalize(result, 0.0F, 255.0F);

  result = result * 1000.0F;

  return result;
}

// ######################################################################
void WinnerTakeAllTempNote::inhibit()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<LeakyIntFireAdp>::iterator
    nptr = itsNeurons.beginw(), stop = itsNeurons.endw();
  while(nptr != stop) (nptr++)->setG(0.0F, itsGinh);
  itsGIN.setG(0.0F, 0.0F);
  LDEBUG("WTA inhibition firing...");
}

// ######################################################################
void WinnerTakeAllTempNote::saccadicSuppression(const bool on)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (itsUseSaccadicSuppression.getVal() == false) return;
  if (on) inhibit();
  LINFO("------- WTA saccadic suppression %s -------", on ? "on":"off");
}

// ######################################################################
void WinnerTakeAllTempNote::blinkSuppression(const bool on)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (itsUseBlinkSuppression.getVal() == false) return;
  if (on) inhibit();
  LINFO("------- WTA blink suppression %s -------", on ? "on":"off");
}

// ######################################################################
void WinnerTakeAllTempNote::save1(const ModelComponentSaveInfo& sinfo)
{
GVX_TRACE(__PRETTY_FUNCTION__);

// get the OFS to save to, assuming sinfo is of type
// SimModuleSaveInfo (will throw a fatal exception otherwise):
 nub::ref<FrameOstream> ofs =
   dynamic_cast<const SimModuleSaveInfo&>(sinfo).ofs;

  ofs->writeFloat(this->getV(), FLOAT_NORM_PRESERVE, "WTA",
                  FrameInfo("winner-take-all map", SRC_POS));

  ofs->writeFloat(getVth(false), FLOAT_NORM_0_255, "WTA-Vth",
                  FrameInfo("winner-take-all threshold", SRC_POS));

  ofs->writeFloat(itsInitMask,FLOAT_NORM_0_255, "WTA-MASK",
                  FrameInfo("winner-take-all mask", SRC_POS));

}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

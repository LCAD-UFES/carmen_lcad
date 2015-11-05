/*!@file Neuro/SaliencyMap.C Implementation for saliency map class */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SaliencyMap.C $
// $Id: SaliencyMap.C 13377 2010-05-09 15:55:07Z itti $
//

#include "Neuro/SaliencyMap.H"

#include "Channels/ChannelOpts.H" // for OPT_LevelSpec
#include "Component/OptionManager.H"
#include "Image/Image.H"
#include "Image/MathOps.H"   // for inplaceAddWeighted()
#include "Image/FilterOps.H"   // for inplaceAddWeighted()
#include "Neuro/NeuroOpts.H"
#include "Neuro/SpatialMetrics.H"
#include "Neuro/NeuroSimEvents.H"
#include "Simulation/SimulationOpts.H"
#include "Simulation/SimEventQueue.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/Assert.H"
#include "Util/MathFunctions.H"
#include "Util/Types.H"
#include "Util/log.H"
#include "rutz/trace.h"

#include <cmath>

namespace
{
  //! Helper function to compute inhibition of return
  double iorHelper(const double d, const double ampl, const double sdev)
  {
    double d_dev = d / sdev; d_dev = -0.5 * d_dev * d_dev;
    if (d_dev < -20.0) return 0.0;
    return ampl * exp(d_dev);
  }

  //! Helper function to compute inhibition of return
  double iorHelper(const Point2D<int>& p1, const Point2D<int>& p2,
                   const double pampl, const double mampl,
                   const double psdev, const double msdev)
  {
    double dx = double(p1.i - p2.i);
    double dy = double(p1.j - p2.j);
    double d = sqrt(squareOf(dx) + squareOf(dy));
    return iorHelper(d, pampl, psdev) - iorHelper(d, mampl, msdev);
  }
}

// ######################################################################
// ######################################################################
// ########## SaliencyMap implementation
// ######################################################################
// ######################################################################

SaliencyMap::SaliencyMap(OptionManager& mgr,
                         const std::string& descrName,
                         const std::string& tagName) :
  SimModule(mgr, descrName, tagName)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
SaliencyMap::~SaliencyMap()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
// ######################################################################
// ########## SaliencyMapStub implementation
// ######################################################################
// ######################################################################

SaliencyMapStub::SaliencyMapStub(OptionManager& mgr,
                                 const std::string& descrName,
                                 const std::string& tagName) :
  SaliencyMap(mgr, descrName, tagName)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
SaliencyMapStub::~SaliencyMapStub()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
// ######################################################################
// ########## SaliencyMapAdapter implementation
// ######################################################################
// ######################################################################

SaliencyMapAdapter::SaliencyMapAdapter(OptionManager& mgr,
                                       const std::string& descrName,
                                       const std::string& tagName) :
  SaliencyMap(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventWTAwinner),
  SIMCALLBACK_INIT(SimEventVisualCortexOutput),
  SIMCALLBACK_INIT(SimEventSaccadeStatusEye),
  SIMCALLBACK_INIT(SimEventSaveOutput),
  itsMetrics(new SpatialMetrics(mgr)),
  itsIORtype(&OPT_IORtype, this),  // see Neuro/NeuroOpts.{H,C}
  itsLevelSpec(&OPT_LevelSpec, this), //Channels/ChannelOpts.{H,C}
  itsSaveResults(&OPT_SMsaveResults, this), // see Neuro/NeuroOpts.{H,C}
  itsSaveCumResults(&OPT_SMsaveCumResults, this), // idem
  itsUseSaccadicSuppression(&OPT_SMuseSacSuppress, this),
  itsUseBlinkSuppression(&OPT_SMuseBlinkSuppress, this),
  itsMaxWinMv(&OPT_SMmaxWinV, this)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  this->addSubComponent(itsMetrics);
}

// ######################################################################
SaliencyMapAdapter::~SaliencyMapAdapter()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void SaliencyMapAdapter::
onSimEventWTAwinner(SimEventQueue& q, rutz::shared_ptr<SimEventWTAwinner>& e)
{
  const WTAwinner winner = e->winner();
  const float winV = this->getV(winner.smpos);

  // show SM voltage at winning location:
  LINFO("SM voltage at winner: %fmV above rest", 1000.0 * winV);

  // if current winning voltage is too high, calm things down with
  // some mild broad inhibition:
  if (1000.0 * winV > itsMaxWinMv.getVal())
    {
      LINFO("Triggering global inhibition to avoid map explosion.");
      this->depress(q, winner.smpos,
                    0.00005 * (winV - itsMaxWinMv.getVal() / 1000.0),
                    0.0,
                    1000.0 * double(itsMetrics->getFOAradius()),
                    100.0);
    }

  // now do our inhibition-of-return, if any
  if (itsIORtype.getVal() != IORnone)
    {
      // any available ShapeEstimator IOR mask?  NOTE: The shape
      // estimator will not work without also specifying
      // --shape-estim-mode= at the command line otherwise the shape
      // estimator will not run and this will ALWAYS be not
      // initialized!
      Image<float> IORmask;
      if (SeC<SimEventShapeEstimatorOutput> ee =
          q.check<SimEventShapeEstimatorOutput>(this, SEQ_ANY))
        IORmask = ee->iorMask();

      if (itsIORtype.getVal() == IORdisc || IORmask.initialized() == false)
        {
          // compute the parameters of the local depression, and depress
          // amplitudes for M-hat:
          const double pAmpl = 0.1 * double(winV); // strongly inhib winner
          const double mAmpl = 1e-4 * pAmpl;      // mildly excite surround
          // standard deviations for M_hat: inhibit of the foa size
          const double pSdev = 0.3 * double(itsMetrics->getFOAradius()) /
            double(1 << itsLevelSpec.getVal().mapLevel());      //foa inhib
          const double mSdev = 4.0 * pSdev;  // wide mild excit outside foa
          this->depress(q, winner.smpos, pAmpl, mAmpl, pSdev, mSdev);
          LINFO("Inhibition of return fired.");
        }
      else if (itsIORtype.getVal() == IORshapeEst && IORmask.initialized())
        {
          this->depress(q, winner.smpos, winV, IORmask);
          LINFO("ShapeEstimator-based inhibition of return fired.");
        }
    }
}

// ######################################################################
void SaliencyMapAdapter::
onSimEventVisualCortexOutput(SimEventQueue& q, rutz::shared_ptr<SimEventVisualCortexOutput>& e)
{
  // Use the new VisualCortex output as our inputs:
  this->input(q, e->vco());
}

// ######################################################################
void SaliencyMapAdapter::
onSimEventSaccadeStatusEye(SimEventQueue& q, rutz::shared_ptr<SimEventSaccadeStatusEye>& e)
{
  // get any eye activity so we can check for saccades or blinks:

  // any eye saccade and we are using saccadic suppression?
  if (itsUseSaccadicSuppression.getVal())
    {
      if (e->saccadeStatus() == TSTATUS_BEGIN) this->saccadicSuppression(q, true);
      else if (e->saccadeStatus() == TSTATUS_END) this->saccadicSuppression(q, false);
    }

  // any eye blink and we are using blink suppression?
  if (itsUseBlinkSuppression.getVal())
    {
      if (e->blinkStatus() == TSTATUS_BEGIN) this->blinkSuppression(q, true);
      else if (e->blinkStatus() == TSTATUS_END) this->blinkSuppression(q, false);
    }
}

// ######################################################################
void SaliencyMapAdapter::
onSimEventSaveOutput(SimEventQueue& q, rutz::shared_ptr<SimEventSaveOutput>& e)
{
  // get the OFS to save to, assuming sinfo is of type
  // SimModuleSaveInfo (will throw a fatal exception otherwise):
  nub::ref<FrameOstream> ofs = dynamic_cast<const SimModuleSaveInfo&>(e->sinfo()).ofs;

  // get current membrane potentials:
  const Image<float> rawsm = this->getV();

  if (itsSaveResults.getVal())
    // save un-normalized float images:
    ofs->writeFloat(rawsm, FLOAT_NORM_PRESERVE, "SM", FrameInfo("saliency map (bottom-up)", SRC_POS));

  if (itsSaveCumResults.getVal())
    {
      // FIXME: this is hacky, probably we should accumulate itsCumMap
      // in doEvolve() instead of here...

      if (itsCumMap.initialized())
        const_cast<SaliencyMapAdapter*>(this)->itsCumMap += rawsm;
      else
        const_cast<SaliencyMapAdapter*>(this)->itsCumMap = rawsm;

      ofs->writeFloat(itsCumMap, FLOAT_NORM_PRESERVE, "CUMSM",
                      FrameInfo("cumulative saliency map", SRC_POS));
    }
}

// ######################################################################
// ######################################################################
// ########## SaliencyMapStd implementation
// ######################################################################
// ######################################################################

// ######################################################################
SaliencyMapStd::SaliencyMapStd(OptionManager& mgr,
                               const std::string& descrName,
                               const std::string& tagName) :
  SaliencyMapAdapter(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventClockTick),
  itsGinhDecay(&OPT_SMginhDecay, this), itsNeurons()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
SaliencyMapStd::~SaliencyMapStd()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void SaliencyMapStd::input(SimEventQueue& q, const Image<float>& in)
{
  // if this is our first, initialize our neurons:
  if (itsNeurons.initialized() == false)
    {
      itsNeurons.resize(in.getDims(), true);
      Image<LeakyIntegrator>::iterator
        nptr = itsNeurons.beginw(), stop = itsNeurons.endw();
      const float decay = itsGinhDecay.getVal();
      while(nptr != stop) (nptr++)->setGinhDecay(decay);
    }

  // set every neuron's input:
  Image<LeakyIntegrator>::iterator nptr = itsNeurons.beginw();
  Image<float>::const_iterator cptr = in.begin(), stop = in.end();
  while (cptr != stop) (nptr++)->input(*cptr++);
}

// ######################################################################
void SaliencyMapStd::depress(SimEventQueue& q, const Point2D<int>& winner,
                             const double& pampl, const double& mampl,
                             const double& psdev, const double& msdev)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Point2D<int> p;
  const int ww = itsNeurons.getWidth();
  const int hh = itsNeurons.getHeight();
  Image<LeakyIntegrator>::iterator src = itsNeurons.beginw();

  // open inhibitory conductances:
  for (p.j = 0; p.j < hh; p.j ++)
    for (p.i = 0; p.i < ww; p.i ++)
      (src++)->addGinh(float(iorHelper(winner, p, pampl, mampl, psdev, msdev)));

  LDEBUG("Peak IOR conductance: %fmS",
         iorHelper(winner, winner, pampl, mampl, psdev, msdev) * 1000.0);
}

// ######################################################################
void SaliencyMapStd::depress(SimEventQueue& q, const Point2D<int>& winner, const double& ampl,
                             const Image<byte>& objectMask)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Point2D<int> p;
  const int ww = itsNeurons.getWidth();
  const int hh = itsNeurons.getHeight();
  Image<LeakyIntegrator>::iterator src = itsNeurons.beginw();

  // open inhibitory conductances:
  for (p.j = 0; p.j < hh; p.j ++)
    for (p.i = 0; p.i < ww; p.i ++)
        (src++)->addGinh(0.1F / 255.F * ampl * float(objectMask.getVal(p)));

  LDEBUG("Peak IOR conductance: %fmS",
         0.1F / 255.F * ampl * float(objectMask.getVal(winner)) * 1000.0F);
}

// ######################################################################
void SaliencyMapStd::
onSimEventClockTick(SimEventQueue& q, rutz::shared_ptr<SimEventClockTick>& e)
{
  // evolve our neurons one time step:
  Image<LeakyIntegrator>::iterator itr = itsNeurons.beginw();
  Image<LeakyIntegrator>::iterator stop = itsNeurons.endw();
  while (itr != stop) (itr++)->integrate(q.now());

  // post our current saliency map if we have one:
  const Image<float> sm = this->getV();
  q.post(rutz::make_shared(new SimEventSaliencyMapOutput(this, sm, itsLevelSpec.getVal().mapLevel())));
}

// ######################################################################
void SaliencyMapStd::saccadicSuppression(SimEventQueue& q, const bool on)
{
  // we setup a weak inhibitory conductance for saccadic suppression
  // of the Saliency Map, as there is evidence that some
  // intra-saccadic visual input may still influence eye movements.
  const float inG = on ? 1.0e-6F : 0.0F;  // inhibitory conductance in Siemens

  // NOTE: this will erase any previous conductance pattern that may
  // have been present in the neurons, e.g., due to recent IORs. So it
  // is assumed here that IOR and saccadic suppression share the same
  // inhibitory mechanism.
  Image<LeakyIntegrator>::iterator src = itsNeurons.beginw();
  Image<LeakyIntegrator>::iterator stop = itsNeurons.endw();
  while(src != stop) (src++)->setGinh(inG);

  LINFO("------- SaliencyMapStd saccadic suppression %s -------", on ? "on" : "off");
}

// ######################################################################
void SaliencyMapStd::blinkSuppression(SimEventQueue& q, const bool on)
{
  // we setup a weak inhibitory conductance for blink suppression of
  // the Saliency Map, as the visual input should already have been
  // turned off by the Retina:
  const float inG = on ? 1.0e-6F : 0.0F;  // inhibitory conductance in Siemens

  // NOTE: this will erase any previous conductance pattern that may
  // have been present in the neurons, e.g., due to recent IORs. So it
  // is assumed here that IOR and saccadic suppression share the same
  // inhibitory mechanism.
  Image<LeakyIntegrator>::iterator src = itsNeurons.beginw();
  Image<LeakyIntegrator>::iterator stop = itsNeurons.endw();
  while(src != stop) (src++)->setGinh(inG);

  LINFO("------- SaliencyMapStd blink suppression %s -------", on ? "on" : "off");
}

// ######################################################################
Image<float> SaliencyMapStd::getV() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (itsNeurons.initialized() == false) return Image<float>();

  Image<float> ret(itsNeurons.getDims(), NO_INIT);
  Image<float>::iterator dest = ret.beginw();
  Image<LeakyIntegrator>::const_iterator src = itsNeurons.begin();
  Image<LeakyIntegrator>::const_iterator stop = itsNeurons.end();
  while (src != stop) *dest++ = (src++)->getV();

  return ret;
}

// ######################################################################
float SaliencyMapStd::getV(const Point2D<int>& p) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsNeurons.getVal(p).getV();
}

// ######################################################################
void SaliencyMapStd::reset1()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsNeurons.freeMem();
}


// ######################################################################
// ########## SaliencyMapTrivial implementation
// ######################################################################
// ######################################################################

// ######################################################################
SaliencyMapTrivial::SaliencyMapTrivial(OptionManager& mgr,
                                       const std::string& descrName,
                                       const std::string& tagName) :
  SaliencyMapAdapter(mgr, descrName, tagName),
  itsItoVcoeff(&OPT_SMItoVcoeff, this),
  itsNeurons()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
SaliencyMapTrivial::~SaliencyMapTrivial()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void SaliencyMapTrivial::reset1()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsNeurons.freeMem();
}

// ######################################################################
void SaliencyMapTrivial::input(SimEventQueue& q, const Image<float>& current)
{
GVX_TRACE(__PRETTY_FUNCTION__);
 itsNeurons = current * itsItoVcoeff.getVal();

  // post our current saliency map if we have one:
  if (itsNeurons.initialized())
    q.post(rutz::make_shared(new SimEventSaliencyMapOutput(this, itsNeurons, itsLevelSpec.getVal().mapLevel())));
}

// ######################################################################
void SaliencyMapTrivial::depress(SimEventQueue& q, const Point2D<int>& winner,
                                 const double& pampl, const double& mampl,
                                 const double& psdev, const double& msdev)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Point2D<int> p;
  const int ww = itsNeurons.getWidth();
  const int hh = itsNeurons.getHeight();
  Image<float>::iterator src = itsNeurons.beginw();

  // open inhibitory conductances:
  for (p.j = 0; p.j < hh; p.j ++)
    for (p.i = 0; p.i < ww; p.i ++)
      {
        *src -= float(iorHelper(winner, p, pampl, mampl, psdev, msdev));
        if (*src < 0.0F) *src = 0.0F;
        ++ src;
      }

  LDEBUG("Peak IOR conductance: %fmS",
         float(iorHelper(winner, winner, pampl, mampl, psdev, msdev)) * 1000.0F);

  // post our current saliency map if we have one:
  if (itsNeurons.initialized())
    q.post(rutz::make_shared(new SimEventSaliencyMapOutput(this, itsNeurons, itsLevelSpec.getVal().mapLevel())));
}

// ######################################################################
void SaliencyMapTrivial::depress(SimEventQueue& q, const Point2D<int>& winner, const double& ampl,
                                 const Image<byte>& objectMask)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<float> inhib = objectMask * (ampl / 255.0F);
  itsNeurons -= inhib;        // subtract inhib for current saliency values
  inplaceRectify(itsNeurons); // remove possible negative values

  // post our current saliency map if we have one:
  if (itsNeurons.initialized())
    q.post(rutz::make_shared(new SimEventSaliencyMapOutput(this, itsNeurons, itsLevelSpec.getVal().mapLevel())));
}

// ######################################################################
void SaliencyMapTrivial::saccadicSuppression(SimEventQueue& q, const bool on)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // we setup a weak inhibitory conductance for saccadic suppression
  // of the Saliency Map, as there is evidence that some intra-saccadic visual
  // input may still influence eye movements.
  if (on) itsNeurons -= 1.0e-6F;
  LINFO("------- SaliencyMapTrivial saccadic suppression %s -------", on ? "on":"off");

  // post our current saliency map if we have one:
  if (itsNeurons.initialized())
    q.post(rutz::make_shared(new SimEventSaliencyMapOutput(this, itsNeurons, itsLevelSpec.getVal().mapLevel())));
}

// ######################################################################
void SaliencyMapTrivial::blinkSuppression(SimEventQueue& q, const bool on)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // we setup a weak inhibitory conductance for blink suppression of
  // the Saliency Map, as the visual input should also be turned off
  // in Brain
  if (on) itsNeurons -= 1.0e-6F;
  LINFO("------- SaliencyMapTrivial blink suppression %s -------", on ? "on":"off");

  // post our current saliency map if we have one:
  if (itsNeurons.initialized())
    q.post(rutz::make_shared(new SimEventSaliencyMapOutput(this, itsNeurons, itsLevelSpec.getVal().mapLevel())));
}

// ######################################################################
Image<float> SaliencyMapTrivial::getV() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
 return itsNeurons;
}

// ######################################################################
float SaliencyMapTrivial::getV(const Point2D<int>& p) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsNeurons.getVal(p);
}

// ######################################################################
// ######################################################################
// ########## SaliencyMapFast implementation
// ######################################################################
// ######################################################################

// ######################################################################
SaliencyMapFast::SaliencyMapFast(OptionManager& mgr,
                                 const std::string& descrName,
                                 const std::string& tagName) :
  SaliencyMapAdapter(mgr, descrName, tagName),
  itsTimeStep(&OPT_SimulationTimeStep, this), // see Neuro/NeuroOpts.{H,C}
  itsInputCoeff(&OPT_SMfastInputCoeff, this), // ModelOptionDefs.C
  itsItoVcoeff(&OPT_SMItoVcoeff, this), // idem
  itsNeurons(), itsInputCopy(), itsT()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
SaliencyMapFast::~SaliencyMapFast()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void SaliencyMapFast::reset1()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsNeurons.freeMem(); itsT = SimTime::ZERO();
}

// ######################################################################
void SaliencyMapFast::input(SimEventQueue& q, const Image<float>& current)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // make sure we are up to date:
  integrate(q);

  // get the new inputs:
  itsInputCopy = current * itsItoVcoeff.getVal();
  itsNeurons = itsInputCopy;

  // post current map:
  // post our current saliency map if we have one:
  if (itsNeurons.initialized())
    q.post(rutz::make_shared(new SimEventSaliencyMapOutput(this, itsNeurons, itsLevelSpec.getVal().mapLevel())));
}

// ######################################################################
void SaliencyMapFast::depress(SimEventQueue& q, const Point2D<int>& winner,
                              const double& pampl, const double& mampl,
                              const double& psdev, const double& msdev)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // make sure we are up to date:
  integrate(q);

  Point2D<int> p;
  const int ww = itsNeurons.getWidth();
  const int hh = itsNeurons.getHeight();
  Image<float>::iterator src = itsNeurons.beginw();

  // open inhibitory conductances:
  for (p.j = 0; p.j < hh; p.j ++)
    for (p.i = 0; p.i < ww; p.i ++)
      {
        *src -= float(iorHelper(winner, p, pampl*30, mampl, psdev, msdev));
        if (*src < 0.0F) *src = 0.0F;
        ++ src;
      }

  LDEBUG("Peak IOR conductance: %fmS",
         float(iorHelper(winner, winner, pampl*30, mampl, psdev, msdev)) * 1000.0F);

  // post our current saliency map if we have one:
  if (itsNeurons.initialized())
    q.post(rutz::make_shared(new SimEventSaliencyMapOutput(this, itsNeurons, itsLevelSpec.getVal().mapLevel())));
}

// ######################################################################
void SaliencyMapFast::depress(SimEventQueue& q, const Point2D<int>& winner, const double& ampl,
                              const Image<byte>& objectMask)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // make sure we are up to date:
  integrate(q);

  Image<float> inhib = objectMask * (ampl / 255.0F);
  itsNeurons -= inhib;        // subtract inhib for current saliency values
  inplaceRectify(itsNeurons); // remove possible negative values

  // post our current saliency map if we have one:
  if (itsNeurons.initialized())
    q.post(rutz::make_shared(new SimEventSaliencyMapOutput(this, itsNeurons, itsLevelSpec.getVal().mapLevel())));
}

// ######################################################################
void SaliencyMapFast::integrate(SimEventQueue& q)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  double nsteps = (q.now() - itsT).secs() / itsTimeStep.getVal().secs();
  if (nsteps >= 1.0 && itsInputCoeff.getVal() < 1) // If we need to merge with old ones
    {
      float c1 = pow(itsInputCoeff.getVal(), nsteps), c2 = 1.0F - c1;
      itsNeurons *= c1;
      inplaceAddWeighted(itsNeurons, itsInputCopy, c2);
    }
  itsT = q.now();
}

// ######################################################################
void SaliencyMapFast::saccadicSuppression(SimEventQueue& q, const bool on)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // make sure we are up to date:
  integrate(q);

  // we setup a weak inhibitory conductance for saccadic suppression
  // of the Saliency Map, as there is evidence that some intra-saccadic visual
  // input may still influence eye movements.
  if (on) itsNeurons -= 1.0e-6F;
  LINFO("------- SaliencyMapFast saccadic suppression %s -------", on ? "on":"off");

  // post our current saliency map if we have one:
  if (itsNeurons.initialized())
    q.post(rutz::make_shared(new SimEventSaliencyMapOutput(this, itsNeurons, itsLevelSpec.getVal().mapLevel())));
}

// ######################################################################
void SaliencyMapFast::blinkSuppression(SimEventQueue& q, const bool on)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // make sure we are up to date:
  integrate(q);

  // we setup a weak inhibitory conductance for blink suppression of
  // the Saliency Map, as the visual input should also be turned off
  // in Brain
  if (on) itsNeurons -= 1.0e-6F;
  LINFO("------- SaliencyMapFast blink suppression %s -------", on ? "on":"off");

  // post our current saliency map if we have one:
  if (itsNeurons.initialized())
    q.post(rutz::make_shared(new SimEventSaliencyMapOutput(this, itsNeurons, itsLevelSpec.getVal().mapLevel())));
}

// ######################################################################
Image<float> SaliencyMapFast::getV() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsNeurons;
}

// ######################################################################
float SaliencyMapFast::getV(const Point2D<int>& p) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsNeurons.getVal(p);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

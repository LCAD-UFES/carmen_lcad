/*!@file Neuro/SaliencyMapStdOptim.C */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SaliencyMapStdOptim.C $
// $Id: SaliencyMapStdOptim.C 13377 2010-05-09 15:55:07Z itti $
//

#ifndef NEURO_SALIENCYMAPSTDOPTIM_C_DEFINED
#define NEURO_SALIENCYMAPSTDOPTIM_C_DEFINED

#include "Neuro/SaliencyMapStdOptim.H"

#include "Image/MathOps.H"
#include "Neuro/NeuroOpts.H"
#include "Simulation/SimEventQueue.H"
#include "Util/JobWithSemaphore.H"
#include "Util/MainJobServer.H"
#include "Util/MathFunctions.H"
#include "rutz/shared_ptr.h"
#include "rutz/trace.h"

#include <vector>

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

struct SaliencyMapStdOptim::UpdateJob : public JobWithSemaphore
{
  UpdateJob(Image<float>::iterator vitr_,
            Image<float>::iterator vstop_,
            Image<float>::const_iterator iitr_,
            Image<float>::iterator ginhitr_,
            const float decay_, const float dt_c_,
            const float gleak_,
            const SimTime start_tm_, const SimTime end_tm_,
            const SimTime dt_)
    :
    vitr(vitr_),
    vstop(vstop_),
    iitr(iitr_),
    ginhitr(ginhitr_),
    decay(decay_),
    dt_c(dt_c_),
    gleak(gleak_),
    start_tm(start_tm_),
    end_tm(end_tm_),
    dt(dt_)
  {}

  virtual ~UpdateJob()
  {}

  virtual void run()
  {
    while (vitr != vstop)
      {
        for (SimTime tt = start_tm; tt < end_tm; tt += dt)
          {
            (*vitr) += dt_c * ( (*iitr) - ( gleak + (*ginhitr) ) * (*vitr) );
            if (*vitr < 0.0F) *vitr = 0.0F;
            (*ginhitr) *= decay; // progressively loose inhibitory influences
          }

        ++vitr;
        ++ginhitr;
        ++iitr;
      }

    this->markFinished();
  }

  virtual const char* jobType() const
  { return "SaliencyMapStdOptimUpdateJob"; }

  Image<float>::iterator vitr;
  Image<float>::iterator vstop;
  Image<float>::const_iterator iitr;
  Image<float>::iterator ginhitr;
  const float decay;
  const float dt_c;
  const float gleak;
  const SimTime start_tm;
  const SimTime end_tm;
  const SimTime dt;
};

// ######################################################################
// ######################################################################
// ########## SaliencyMapStdOptim implementation
// ######################################################################
// ######################################################################

// ######################################################################
SaliencyMapStdOptim::SaliencyMapStdOptim(OptionManager& mgr,
                               const std::string& descrName,
                               const std::string& tagName) :
  SaliencyMapAdapter(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventClockTick),
  itsTimeStep(SimTime::SECS(0.0001)),
  itsT(SimTime::ZERO()),
  itsC(5.0E-8F),
  itsGleak(1.0E-7F),
  itsV(),
  itsGinhDecay(&OPT_SMginhDecay, this)
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
SaliencyMapStdOptim::~SaliencyMapStdOptim()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void SaliencyMapStdOptim::reset()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsV.freeMem();
}

// ######################################################################
void SaliencyMapStdOptim::input(SimEventQueue& q, const Image<float>& current)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (itsV.initialized() == false)
    {
      itsGinh.resize(current.getDims(), true);
      itsV.resize(current.getDims(), true);
    }
  itsI = current;
}

// ######################################################################
void SaliencyMapStdOptim::depress(SimEventQueue& q, const Point2D<int>& winner,
                             const double& pampl, const double& mampl,
                             const double& psdev, const double& msdev)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Point2D<int> p;
  const int ww = itsV.getWidth();
  const int hh = itsV.getHeight();
  Image<float>::iterator src = itsGinh.beginw();

  // open inhibitory conductances:
  for (p.j = 0; p.j < hh; ++p.j)
    for (p.i = 0; p.i < ww; ++p.i)
      (*src++) += float(iorHelper(winner, p, pampl, mampl, psdev, msdev));

  LDEBUG("Peak IOR conductance: %fmS",
         float(iorHelper(winner, winner, pampl, mampl, psdev, msdev)) * 1000.0F);
}

// ######################################################################
void SaliencyMapStdOptim::depress(SimEventQueue& q, const Point2D<int>& winner, const double& ampl,
                             const Image<byte>& objectMask)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Point2D<int> p;
  const int ww = itsV.getWidth();
  const int hh = itsV.getHeight();
  Image<float>::iterator src = itsGinh.beginw();

  // open inhibitory conductances:
  for (p.j = 0; p.j < hh; ++p.j)
    for (p.i = 0; p.i < ww; ++p.i)
      (*src++) += 0.1F / 255.F * ampl * float(objectMask.getVal(p));

  LDEBUG("Peak IOR conductance: %fmS",
         0.1F / 255.F * ampl * float(objectMask.getVal(winner)) * 1000.0F);
}

// ######################################################################
void SaliencyMapStdOptim::
onSimEventClockTick(SimEventQueue& q, rutz::shared_ptr<SimEventClockTick>& e)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const SimTime t = q.now();

  // we run our difference equations with a time step of itsTimeStep;
  // let's here figure out how many iterations we will need to go from
  // itsT to t. We will iterate for a number of equal steps, with each
  // step as close to itsTimeStep as possible to that we end up at
  // time t after iterating for an integer number of time steps:
  const SimTime dt = SimTime::computeDeltaT((t - itsT), itsTimeStep);
  const float dtsc = float(dt.secs()) / itsC;

  JobServer& srv = getMainJobServer();

  const unsigned int ntiles = srv.getParallelismHint();

  std::vector<rutz::shared_ptr<UpdateJob> > jobs;

  for (unsigned int i = 0; i < ntiles; ++i)
    {
      const int start = (i*itsV.getSize()) / ntiles;
      const int end = ((i+1)*itsV.getSize()) / ntiles;

      jobs.push_back
        (rutz::make_shared(new UpdateJob
                           (itsV.beginw() + start,
                            itsV.beginw() + end,
                            itsI.begin() + start,
                            itsGinh.beginw() + start,
                            itsGinhDecay.getVal(),
                            dtsc,
                            itsGleak,
                            itsT, t, dt)));

      srv.enqueueJob(jobs.back());
    }

  // barrier:
  for (size_t i = 0; i < jobs.size(); ++i) jobs[i]->wait();

  // post our map for everyone to enjoy:
  q.post(rutz::make_shared(new SimEventSaliencyMapOutput(this, itsV, itsLevelSpec.getVal().mapLevel())));

  // we are done, just keep track of new current time:
  itsT = t;
}

// ######################################################################
void SaliencyMapStdOptim::saccadicSuppression(SimEventQueue& q, const bool on)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // we setup a weak inhibitory conductance for saccadic suppression
  // of the Saliency Map, as there is evidence that some intra-saccadic visual
  // input may still influence eye movements.
  const float inG =
    on
    ? 1.0e-6F  // on inhibitory conductance, in Siemens
    : 0.0F;    // turning off

  itsGinh.clear(inG);

  LINFO("------- SaliencyMapStdOptim saccadic suppression %s -------", on ? "on":"off");
}

// ######################################################################
void SaliencyMapStdOptim::blinkSuppression(SimEventQueue& q, const bool on)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // we setup a weak inhibitory conductance for blink suppression of
  // the Saliency Map, as the visual input should also be turned off
  // in Brain
  const float inG =
    on
    ? 1.0e-6F  // on inhibitory conductance, in Siemens
    : 0.0F;    // turning off

  itsGinh.clear(inG);

  LINFO("------- SaliencyMapStdOptim blink suppression %s -------", on ? "on":"off");
}

// ######################################################################
Image<float> SaliencyMapStdOptim::getV() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsV;
}

// ######################################################################
float SaliencyMapStdOptim::getV(const Point2D<int>& p) const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsV.getVal(p);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_SALIENCYMAPSTDOPTIM_C_DEFINED

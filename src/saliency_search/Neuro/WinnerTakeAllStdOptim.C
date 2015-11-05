/*!@file Neuro/WinnerTakeAllStdOptim.C Optimized version of WinnerTakeAllStd */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/WinnerTakeAllStdOptim.C $
// $Id: WinnerTakeAllStdOptim.C 10729 2009-02-02 03:44:27Z itti $
//

#ifndef NEURO_WINNERTAKEALLSTDOPTIM_C_DEFINED
#define NEURO_WINNERTAKEALLSTDOPTIM_C_DEFINED

#include "Neuro/WinnerTakeAllStdOptim.H"

#include "Util/JobWithSemaphore.H"
#include "Util/MainJobServer.H"
#include "rutz/trace.h"

struct WinnerTakeAllStdOptim::EvolveJob : public JobWithSemaphore
{
  EvolveJob(Image<float>::iterator vitr_,
            Image<float>::iterator vstop_,
            Image<float>::const_iterator initr_,
            const float ginput_,
            const float dt_c_,
            const float gsum_,
            const float isum_,
            const float Ei_,
            const float Vth_)
    :
    vitr(vitr_),
    vstop(vstop_),
    initr(initr_),
    ginput(ginput_),
    dt_c(dt_c_),
    gsum(gsum_),
    isum(isum_),
    Ei(Ei_),
    Vth(Vth_),
    vwinner()
  {}

  virtual ~EvolveJob()
  {}

  virtual void run()
  {
    while (vitr != vstop)
      {
        const float Iin = ginput * (*initr++);

        // Integrate :    all units MKSA
        (*vitr) += dt_c * (Iin - (*vitr) * gsum + isum);

        // Check if the potential is lower than Ei -> if so, then clamp:
        if ((*vitr) < Ei) (*vitr) = Ei;

        // Check if voltage has exceeded threshold -> if so, then fire:
        if ((*vitr) >= Vth) { vwinner = vitr; *vitr = 0.0F; }

        ++vitr;
      }

    this->markFinished();
  }

  virtual const char* jobType() const
  { return "WinnerTakeAllStdOptimEvolveJob"; }

  Image<float>::iterator vitr;
  Image<float>::iterator vstop;
  Image<float>::const_iterator initr;
  const float ginput;
  const float dt_c;
  const float gsum;
  const float isum;
  const float Ei;
  const float Vth;
  Image<float>::iterator vwinner;
};

// ######################################################################
// ######################################################################
// ########## WinnerTakeAllStdOptim implementation
// ######################################################################
// ######################################################################

WinnerTakeAllStdOptim::WinnerTakeAllStdOptim(OptionManager& mgr,
                                             const std::string& descrName,
                                             const std::string& tagName) :
  WinnerTakeAllAdapter(mgr, descrName, tagName),
  itsTimeStep(SimTime::SECS(0.0001)),
  itsEl(0.0F),
  itsEe(100.0e-3F),
  itsEi(-20.0e-3F),
  itsC(1.0E-9F),
  itsVth(0.001F),
  itsV(),
  itsT(),
  itsGleak(1.0e-8F),
  itsGinh(1.0e-2F),
  itsGinput(5.0e-8F),
  itsGIN_Gl(1.0e-8F),       // in Siemens
  itsGIN_Ge(0.0F),          // in Siemens
  itsGIN_El(0.0F),          // in Volts
  itsGIN_Ee(100.0e-3F),     // in Volts
  itsGIN_Ei(-20.0e-3F),     // in Volts
  itsGIN_C(1.0E-9F),        // in Farads
  itsGIN_Vth(0.001F),       // in Volts
  itsGIN_V(itsGIN_Ei),
  itsInputCopy()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
WinnerTakeAllStdOptim::~WinnerTakeAllStdOptim()
{
GVX_TRACE(__PRETTY_FUNCTION__);
}

// ######################################################################
void WinnerTakeAllStdOptim::reset1()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsV.freeMem();
  itsInputCopy.freeMem();
  itsT = SimTime::ZERO();

  WinnerTakeAllAdapter::reset1();
}

// ######################################################################
void WinnerTakeAllStdOptim::input(const Image<float>& in)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (itsV.initialized() == false)
    {
      // first input, let's initialize our array
      itsV.resize(in.getDims(), NO_INIT);
      itsV.clear(itsEi);

      itsGe = 0.0F;
      itsGi = itsGinh;
    }

  // keep a copy of the input for use in evolve():
  itsInputCopy = in;
}

// ######################################################################
void WinnerTakeAllStdOptim::integrate(const SimTime& t, Point2D<int>& winner)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  winner.i = -1;

  // the array of neurons receive excitatory inputs from outside.
  // here we update the inputs and let the neurons evolve

  // we run our difference equations with a time step of itsTimeStep;
  // let's here figure out how many iterations we will need to go from
  // itsT to t. We will iterate for a number of equal steps, with each
  // step as close to itsTimeStep as possible to that we end up at
  // time tt after iterating for an integer number of time steps:
  const SimTime dt = SimTime::computeDeltaT(t - itsT, itsTimeStep);
  const float dt_c = float(dt.secs()) / itsC;

  for (SimTime tt = itsT; tt < t; tt += dt)
    {
      if (tt == SimTime::ZERO())
        continue;

      ASSERT(dt != SimTime::ZERO());

      const float gsum = itsGleak + itsGe + itsGi;
      const float isum = itsGleak * itsEl + itsGe * itsEe + itsGi * itsEi;

      JobServer& srv = getMainJobServer();

      const unsigned int ntiles = srv.getParallelismHint();

      std::vector<rutz::shared_ptr<EvolveJob> > jobs;

      for (unsigned int i = 0; i < ntiles; ++i)
        {
          const int start = (i*itsV.getSize()) / ntiles;
          const int end = ((i+1)*itsV.getSize()) / ntiles;

          jobs.push_back
            (rutz::make_shared(new EvolveJob
                               (itsV.beginw() + start,
                                itsV.beginw() + end,
                                itsInputCopy.begin() + start,
                                itsGinput,
                                dt_c,
                                gsum,
                                isum,
                                itsEi,
                                itsVth)));

          srv.enqueueJob(jobs.back());
        }

      for (size_t i = 0; i < jobs.size(); ++i)
        {
          jobs[i]->wait();

          if (jobs[i]->vwinner != Image<float>::iterator())
            {
              const size_t offset = jobs[i]->vwinner - itsV.beginw();
              winner.i = offset % itsV.getWidth();
              winner.j = offset / itsV.getWidth();

              // we got a winner, so let's trigger the global inhibition:
              itsGIN_Ge = itsGleak * 10.0F;
            }
        }

      itsGe = 0.0F;
      itsGi = 0.0F;

      // when the global inhibition fires, it triggers inhibitory
      // conductances for one time step in the array of excited
      // neurons, shuts off excitatory conductances, and turns itself
      // off:

      // Integrate GIN
      const float dt_c2 = float(dt.secs()) / itsGIN_C;

      itsGIN_V += dt_c2 *
        (- itsGIN_Gl * (itsGIN_V - itsGIN_El)
         - itsGIN_Ge * (itsGIN_V - itsGIN_Ee));

      // Check if the GIN potential is lower than Ei -> if so, then clamp:
      if (itsGIN_V < itsGIN_Ei) itsGIN_V = itsGIN_Ei;

      // Check if GIN voltage has exceeded threshold -> if so, then fire:
      if (itsGIN_V >= itsGIN_Vth)
        {
          itsGIN_V = 0.0F;
          this->inhibit();
        }
    }

  // we are done, just keep track of new current time:
  itsT = t;
}

// ######################################################################
Image<float> WinnerTakeAllStdOptim::getV() const
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return itsV;
}

// ######################################################################
void WinnerTakeAllStdOptim::inhibit()
{
GVX_TRACE(__PRETTY_FUNCTION__);
  itsGe = 0.0F;
  itsGi = itsGinh;
  itsGIN_Ge = 0.0F;
  LDEBUG("WTA inhibition firing...");
}

// ######################################################################
void WinnerTakeAllStdOptim::saccadicSuppression(const bool on)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (itsUseSaccadicSuppression.getVal() == false) return;
  if (on) inhibit();
  LINFO("------- WTA saccadic suppression %s -------", on ? "on":"off");
}

// ######################################################################
void WinnerTakeAllStdOptim::blinkSuppression(const bool on)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  if (itsUseBlinkSuppression.getVal() == false) return;
  if (on) inhibit();
  LINFO("------- WTA blink suppression %s -------", on ? "on":"off");
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_WINNERTAKEALLSTDOPTIM_C_DEFINED

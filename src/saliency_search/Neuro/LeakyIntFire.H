/*!@file Neuro/LeakyIntFire.H Class declarations for a leaky integrator neuron */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/LeakyIntFire.H $
// $Id: LeakyIntFire.H 10729 2009-02-02 03:44:27Z itti $
//

#ifndef LEAKY_INTFIRE_H_DEFINED
#define LEAKY_INTFIRE_H_DEFINED

#include "Util/SimTime.H"

// ######################################################################
//! A leaky integrate & fire neuron, used by standard winner-take-all (WTA)
// ######################################################################
/*! This is the integrate & fire neuron used in conjunction with WTA
  which is a 2D array of LeakyIntFire. It has a slightly more
  complicated Euler equation then LeakyIntegrator used by SM but the
  basic idea is the same.  All parameters are in MKSA (SI) units
  (i.e., volts, amperes, siemens, etc).  This neuron uses an Euler
  integration of the following form:

  At each time step,

  \f[ V \leftarrow V + \frac{\delta t}{C} \left(I - G_{leak}(V - E_{leak}) -
  G_{ex}(V - E_{ex}) - G_{inh}(V - E_{inh})\right) \f]

  V is also clamped to never become smaller than \f$E_{inh}\f$ and
  a spike is generated when \f$V \ge V_{th}\f$ (and V is then reset to zero).*/
class LeakyIntFire
{
public:
  //! constructor
  /*! Constructor with default params
    @param timeStep is the integration time step, in s.
    See the general description for the other params. */
  inline LeakyIntFire(const SimTime timeStep = SimTime::SECS(0.0001),
                      const float El = 0.0F,          // in Volts
                      const float Ee = 100.0e-3F,     // in Volts
                      const float Ei = -20.0e-3F,     // in Volts
                      const float Gl = 5.0e-8F,       // in Siemens
                      const float Ge = 0.0F,          // in Siemens
                      const float Gi = 0.0F,          // in Siemens
                      const float Vth = 0.001F,       // in Volts
                      const float C = 1.0E-9F);       // in Farads

  //! set input current (A)
  inline void input(const float current);

  //! set membrane potential to given value relative to Ei (in Volts)
  inline void setV(const double val);

  //! integrate for up to given time (in s)
  /*! Returns true if a spike was generated. */
  inline bool integrate(const SimTime& t);

  //! get current membrane potential (in V)
  inline float getV() const;

  //! set excitatory and inhibitory conductances (S)
  inline void setG(const float Exc, const float Inh);

  //! set leak conductance (S)
  inline void setGleak(const float Leak);

  //! return our internal time step:
  inline SimTime getTimeStep() const;

private:
  inline void reset();// reset when a spike occurs

  SimTime itsTimeStep;// time step to use for difference equations (in s)
  float itsV;         // membrane potential in Volts
  float itsI;         // input current in Amperes
  float itsGl;        // leak conductance in Siemens
  float itsGe;        // excitatory conductance in Siemens
  float itsGi;        // inhibitory conductance in Siemens
  float itsC;         // capacitance in Farads
  float itsEl;        // driving potential for leak part, in Volts
  float itsEe;        // driving potential for excitatory part, in Volts
  float itsEi;        // driving potential for inhibitory part, in Volts
  float itsVth;       // spike threshold voltage in Volts
  SimTime itsT;       // time of last integration
};

// ######################################################################
// ##### Inline functions for LeakyIntFire:
// ######################################################################

inline LeakyIntFire::LeakyIntFire(const SimTime timeStep,
                                  const float El,
                                  const float Ee,
                                  const float Ei,
                                  const float Gl,
                                  const float Ge,
                                  const float Gi,
                                  const float Vth,
                                  const float C) :
  itsTimeStep(timeStep), itsV(Ei), itsI(0.0), itsGl(Gl), itsGe(Ge), itsGi(Gi),
  itsC(C), itsEl(El), itsEe(Ee), itsEi(Ei), itsVth(Vth), itsT(SimTime::ZERO())
{  }

// ######################################################################
inline void LeakyIntFire::input(const float current)
{ itsI = current; }

// ######################################################################
inline bool LeakyIntFire::integrate(const SimTime& t)
{
  // we run our difference equations with a time step of itsTimeStep;
  // let's here figure out how many iterations we will need to go from
  // itsT to t. We will iterate for a number of equal steps, with each
  // step as close to itsTimeStep as possible to that we end up at
  // time t after iterating for an integer number of time steps:
  const SimTime dt = SimTime::computeDeltaT((t - itsT), itsTimeStep);
  const float dtsc = float(dt.secs()) / itsC;

  bool spike = false;
  for (SimTime tt = itsT; tt < t; tt += dt)
    {
      // Integrate :    all units MKSA
      itsV += dtsc * (itsI - itsGl * (itsV - itsEl) -
         itsGe * (itsV - itsEe) - itsGi * (itsV - itsEi));

      // Check if the potential is lower than Ei -> if so, then clamp:
      if (itsV < itsEi) itsV = itsEi;

      // Check if voltage has exceeded threshold -> if so, then fire:
      if (itsV >= itsVth) { spike = true; reset(); }
    }

  // we are done, just keep track of new current time:
  itsT = t;
  return spike;
}

// ######################################################################
inline void LeakyIntFire::setV(const double val)
{ itsV = val + itsEi; }

// ######################################################################
inline float LeakyIntFire::getV() const
{ return itsV; }

// ######################################################################
inline void LeakyIntFire::setG(const float Exc, const float Inh)
{ itsGe = Exc; itsGi = Inh; }

// ######################################################################
inline void LeakyIntFire::setGleak(const float Leak)
{ itsGl = Leak; }

// ######################################################################
inline void LeakyIntFire::reset()
{ itsV = 0.0F; }

// ######################################################################
inline SimTime LeakyIntFire::getTimeStep() const
{ return itsTimeStep; }

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

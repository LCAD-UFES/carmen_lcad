/*!@file Neuro/LeakyIntegrator.H Class declarations for a leaky integrator neuron */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/LeakyIntegrator.H $
// $Id: LeakyIntegrator.H 10729 2009-02-02 03:44:27Z itti $
//

#ifndef LEAKY_INTEGRATOR_H_DEFINED
#define LEAKY_INTEGRATOR_H_DEFINED

#include "Util/SimTime.H"

// ######################################################################
//! A leaky integrator neuron, used by standard saliency map (SM)
// ######################################################################
/*! This is the saliency map neuron used in conjunction with SM which is
  a 2D array of LeakyIntegrator. */
class LeakyIntegrator
{
public:
  //! constructor
  /*! Constructor with default params
    @param timeStep is the integration time step, in s
    @param C is the membrane capacitance, in Farads
    @param Gleak is the leak conductance, in Siemens
    @param GinhDecay decay factor applied to Ginh at every time step */
  inline LeakyIntegrator(const SimTime timeStep = SimTime::SECS(0.0001),
                         const float C = 5.0E-8F,
                         const float Gleak = 1.0E-7F,
                         const float GinhDecay = 0.9999F);

  //! set input current (A)
  inline void input(const float current);

  //! reset membrane potential to given value (in Volts)
  inline void resetV(const double val);

  //! integrate for up to given time (in s)
  inline void integrate(const SimTime& t);

  //! get current membrane potential (in V)
  inline float getV() const;

  //! set inhibitory conductance (S)
  inline void setGinh(const float g);

  //! add to inhibitory conductance (S)
  inline void addGinh(const float g);

  //! set the decay factor for Ginh
  inline void setGinhDecay(const float decay);

private:
  SimTime itsTimeStep;// time step to use for difference equations (in s)
  float itsV;         // membrane potential in Volts
  float itsI;         // input current in Amperes
  float itsGinh;      // inhibitory conductance in Siemens
  float itsGinhDecay; // per time step decay factor on Ginh
  float itsC;         // capacitance in Farads
  float itsGleak;     // leak conductance in Siemens
  SimTime itsT;       // time of last integration
};

// ######################################################################
// ##### Inline functions for LeakyIntegrator:
// ######################################################################

inline LeakyIntegrator::LeakyIntegrator(const SimTime timeStep,
                                        const float C,
                                        const float Gleak,
                                        const float GinhDecay) :
  itsTimeStep(timeStep), itsV(0.0F), itsI(0.0F), itsGinh(0.0F),
  itsGinhDecay(GinhDecay), itsC(C), itsGleak(Gleak), itsT(SimTime::ZERO())
{  }

// ######################################################################
inline void LeakyIntegrator::input(const float current)
{ itsI = current; }

// ######################################################################
inline void LeakyIntegrator::resetV(const double val)
{ itsV = val; }

// ######################################################################
inline void LeakyIntegrator::integrate(const SimTime& t)
{
  // we run our difference equations with a time step of itsTimeStep;
  // let's here figure out how many iterations we will need to go from
  // itsT to t. We will iterate for a number of equal steps, with each
  // step as close to itsTimeStep as possible to that we end up at
  // time t after iterating for an integer number of time steps:
  const SimTime dt = SimTime::computeDeltaT((t - itsT), itsTimeStep);
  const float dtsc = float(dt.secs()) / itsC;

  for (SimTime tt = itsT; tt < t; tt += dt)
    {
      itsV += dtsc * ( itsI - ( itsGleak + itsGinh ) * itsV );
      if (itsV < 0.0F) itsV = 0.0F;
      itsGinh *= itsGinhDecay; // progressively loose inhibitory influences
    }

  // we are done, just keep track of new current time:
  itsT = t;
}

// ######################################################################
inline float LeakyIntegrator::getV() const
{ return itsV; }

// ######################################################################
inline void LeakyIntegrator::setGinh(const float g)
{ itsGinh = g; }

// ######################################################################
inline void LeakyIntegrator::addGinh(const float g)
{ itsGinh += g; }

// ######################################################################
inline void LeakyIntegrator::setGinhDecay(const float decay)
{ itsGinhDecay = decay; }

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

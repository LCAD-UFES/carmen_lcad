/*!@file Neuro/SaliencyMapStdOptim.H Optimized version of SaliencyMapStd */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SaliencyMapStdOptim.H $
// $Id: SaliencyMapStdOptim.H 10723 2009-02-01 22:49:43Z itti $
//

#ifndef NEURO_SALIENCYMAPSTDOPTIM_H_DEFINED
#define NEURO_SALIENCYMAPSTDOPTIM_H_DEFINED

#include "Neuro/SaliencyMap.H"

// ######################################################################
//! Optimized version of the standard saliency map
// ######################################################################
/*! This is an optimized version of SaliencyMapStd. */
class SaliencyMapStdOptim : public SaliencyMapAdapter
{
public:
  //! Ininitialized constructor
  SaliencyMapStdOptim(OptionManager& mgr,
                      const std::string& descrName = "Optimized Saliency Map Std",
                      const std::string& tagName = "SaliencyMapStdOptim");

  //! Destructor
  virtual ~SaliencyMapStdOptim();

  //! Reset to initial state just after construction
  virtual void reset();

protected:
  //! Callback for every clock tick, to run our diff equations in integrate()
  SIMCALLBACK_DECLARE(SaliencyMapStdOptim, SimEventClockTick);

  //! Set new input currents for all neurons
  virtual void input(SimEventQueue& q, const Image<float>& in);

  //! Difference-of-Gaussians inhibition of return
  virtual void depress(SimEventQueue& q, const Point2D<int>& winner,
                       const double& pampl, const double& mampl,
                       const double& psdev, const double& msdev);

  //! Object-based inhibition of return
  virtual void depress(SimEventQueue& q, const Point2D<int>& winner, const double& ampl,
                       const Image<byte>& objectMask);

  //! Turn saccadic suppression on/off
  virtual void saccadicSuppression(SimEventQueue& q, const bool on);

  //! Turn blink suppression on/off
  virtual void blinkSuppression(SimEventQueue& q, const bool on);

  //! Return all our membrane potential voltages as an Image<float>
  virtual Image<float> getV() const;

  //! Access function using Point2D<int>
  virtual float getV(const Point2D<int>& p) const;

private:
  SimTime itsTimeStep;// time step to use for difference equations (in s)
  SimTime itsT;       // time of last integration

  const float itsC;         // capacitance in Farads
  const float itsGleak;     // leak conductance in Siemens

  Image<float> itsGinh;
  Image<float> itsI;
  Image<float> itsV;

  OModelParam<float> itsGinhDecay; //!< decay for IOR inhibition

  class UpdateJob;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_SALIENCYMAPSTDOPTIM_H_DEFINED

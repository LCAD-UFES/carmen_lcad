/*!@file Neuro/WinnerTakeAllStdOptim.H Optimized version of WinnerTakeAllStd */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/WinnerTakeAllStdOptim.H $
// $Id: WinnerTakeAllStdOptim.H 10670 2009-01-23 19:42:06Z itti $
//

#ifndef NEURO_WINNERTAKEALLSTDOPTIM_H_DEFINED
#define NEURO_WINNERTAKEALLSTDOPTIM_H_DEFINED

#include "Neuro/WinnerTakeAll.H"

// ######################################################################
//! Optimized version of the standard winner-take-all
// ######################################################################
/*! This is an optimized version of WinnerTakeAllStd which should give
  the same results, but with better CPU efficiency. */
class WinnerTakeAllStdOptim : public WinnerTakeAllAdapter
{
public:
  //! Ininitialized constructor
  /*! The WTA will be resized and initialized the first time input() is
    called */
  WinnerTakeAllStdOptim(OptionManager& mgr,
                   const std::string& descrName = "Optimized Winner-Take-All Std",
                   const std::string& tagName = "WinnerTakeAllStdOptim");

  //! Destructor
  virtual ~WinnerTakeAllStdOptim();

  //! Reset to initial state just after construction
  virtual void reset1();

protected:
  //! Set new input currents for all neurons
  /*! This will initialize and resize the network if the network is
    currently uninitialized (e.g., just after construction or reset()). */
  virtual void input(const Image<float>& in);

  //! Return all our membrane potential voltages as an Image<float>
  virtual Image<float> getV() const;

  //! Integrate inputs until time t (in s) and update membrane potentials
  virtual void integrate(const SimTime& t, Point2D<int>& winner);

  //! Turn saccadic suppression on/off
  virtual void saccadicSuppression(const bool on);

  //! Turn blink suppression on/off
  virtual void blinkSuppression(const bool on);

private:

  const SimTime itsTimeStep;// time step to use for difference equations (in s)
  float itsGe;        // excitatory conductance in Siemens
  float itsGi;        // inhibitory conductance in Siemens
  const float itsEl;  // driving potential for leak part, in Volts
  const float itsEe;  // driving potential for excitatory part, in Volts
  const float itsEi;  // driving potential for inhibitory part, in Volts
  const float itsC;   // capacitance in Farads
  const float itsVth; // spike threshold voltage in Volts
  Image<float> itsV;
  SimTime itsT;                 // time of last integration
  const float itsGleak;         // leak conductance in Siemens
  const float itsGinh;          // in Siemens
  const float itsGinput;        // in Siemens

  // GIN == Global Inhibitory Neuron
  const float itsGIN_Gl;  // GIN leak conductance in Siemens
  float itsGIN_Ge;        // GIN excitatory conductance in Siemens
  const float itsGIN_El;  // GIN driving potential for leak part, in Volts
  const float itsGIN_Ee;  // GIN driving potential for excitatory part, in V
  const float itsGIN_Ei;  // GIN driving potential for inhibitory part, in V
  const float itsGIN_C;   // GIN capacitance in Farads
  const float itsGIN_Vth; // GIN spike threshold voltage in Volts
  float itsGIN_V;         // GIN membrane potential in Volts
  Image<float> itsInputCopy;

  void inhibit();               // inhibit the whole layer

  class EvolveJob;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_WINNERTAKEALLSTDOPTIM_H_DEFINED

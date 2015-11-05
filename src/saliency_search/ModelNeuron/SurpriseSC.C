/*!@file Surprise/SurpriseSC.C a local (single-point) model of surprise */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Surprise/SurpriseSC.C $

#include "ModelNeuron/SurpriseSC.H"
#include "Util/MathFunctions.H"
#include "Util/Assert.H"
#include "Util/log.H"

// ######################################################################
// ######################################################################
// SurpriseSC implementation
// ######################################################################
// ######################################################################
SurpriseSC::SurpriseSC(const double updatefac, const double sampleval, const double noise, const bool useKL) 
{
  init(updatefac, sampleval, noise, useKL);
}

// ######################################################################
SurpriseSC::~SurpriseSC()
{ }

// ######################################################################
void SurpriseSC::init(const double updfac, const double sampleval, const double noise, const bool useKL)
{ 
  itsInitialVal = sampleval; 
  itsUpdateFac = updfac;
  itsAlpha = 1.0 * sampleval / (1.0 - updfac);
  itsBeta = 1.0 / (1.0 - updfac);
  itsNoise = noise;
  itsUseKL = useKL;
}

// ######################################################################
void SurpriseSC::resetUpdFac(const double updfac)
{ itsUpdateFac = updfac; }

// ######################################################################
double SurpriseSC::input(const double& sample)
{
  // a zero response can trip us up since phi(x) >= 0.0
  if (itsAlpha <= 0.0) itsAlpha = 0.0000000000001;

  //Compute the Posterior
  double nalpha = itsAlpha * itsUpdateFac + sample + itsNoise;
  double nbeta = itsBeta * itsUpdateFac + 1.0;
  
  //Surprise as KL(post,prior)
  double s = (itsUseKL) 
    ? KLgamma(nalpha, nbeta, itsAlpha, itsBeta, true)
    : nalpha / nbeta;
  
  // the posterior becomes our new prior:
  itsAlpha = nalpha; itsBeta = nbeta;
  
  return s;
}

// ######################################################################
double SurpriseSC::getMean() const
{ return itsAlpha / itsBeta; }

// ######################################################################
double SurpriseSC::getVar() const
{ return itsAlpha / (itsBeta * itsBeta); }

// ######################################################################
double SurpriseSC::getUpdateFac() const
{ return itsUpdateFac; }

// ######################################################################
SurpriseMapSC::SurpriseMapSC() : Image<SurpriseSC>(), itsOut()
{ }

// ######################################################################
SurpriseMapSC::SurpriseMapSC(const uint x, const uint y, const SurpriseSC& sc) 
    : Image<SurpriseSC>(x, y, NO_INIT), itsOut(x, y, ZEROS)
{ 
  clear(sc);
}

// ######################################################################
void SurpriseMapSC::input(const Image<double> & image)
{
  Image<double>::const_iterator im(image.begin()), end(image.end());
  iterator s(beginw());
  Image<double>::iterator o(itsOut.beginw());
  while (im != end)
    *o++ = (s++)->input(*im++);
}

// ######################################################################
Image<double> SurpriseMapSC::getOutput()
{
  return itsOut;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

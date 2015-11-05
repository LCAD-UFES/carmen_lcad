/** @file Psycho/GaborPatch.H represent the physical parameters of a gabor patch */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/GaborPatch.H $
// $Id: GaborPatch.H 9078 2007-12-11 21:11:45Z rjpeters $
//

// Code herein is derived from GroovX, also licensed under the GPL
// Copyright (c) 2002-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// [http://ilab.usc.edu/rjpeters/groovx/]

#ifndef PSYCHO_GABORPATCH_H_DEFINED
#define PSYCHO_GABORPATCH_H_DEFINED

#include "Image/Image.H"
#include "Psycho/SearchItem.H"
#include "rutz/rand.h"

#include <cmath>

struct GaborSpec
{
  /// Construct with given values.
  /** Some values may be discretized into a (smallish) finite number of
      possible values to minimize the total number of GaborPatch objects
      that have to be created. */
  GaborSpec(double s, double o, double t, double p);

  /// Comparison operator for sorting and associative arrays.
  bool operator<(const GaborSpec& x) const;

  const double theta;     ///< Orientation of primary axis
  const double phi;       ///< Phase angle of grating
  const double sigma;     ///< Std dev of gaussian mask
  const double omega;     ///< Spatial frequency of grating
};

/// Manages a pixmap representation of a gabor patch.
/** Can either compute values directly, or can cache the entire patch. */
class GaborPatch
{
public:
  GaborPatch(const GaborSpec& spec);

  ~GaborPatch();

  static const GaborPatch& lookup(const GaborSpec& s);

  static const GaborPatch& lookup(double sigma, double omega,
                                  double theta, double phi);

  int size() const { return itsSize; }

  Image<double> image(double contrast) const
  {
    if (contrast == 1.0)
      return itsData;
    return itsData * contrast;
  }

private:
  GaborPatch(const GaborPatch&);
  GaborPatch& operator=(const GaborPatch&);

  const GaborSpec itsSpec;
  const int itsSize;
  const double itsCenter;
  const double itsCosTheta;
  const double itsSinTheta;
  const double itsSigmaSqr;
  Image<double> itsData;
};

class GaborPatchItem : public SearchItem
{
public:
  GaborPatchItem();

  virtual ~GaborPatchItem();

  virtual Image<double> getPatch() const;

  double theta;      // angle to x-axis, 0 to TWOPI
  double phi;
  double contrast;

  double itsPeriod;
  double itsSigma;
};

class GaborPatchItemFactory : public SearchItemFactory
{
public:
  GaborPatchItemFactory(int thetaSeed,
                        int phaseSeed,
                        int contrastSeed,
                        double gaborPeriod = 15.0, double gaborSigma = 7.5);

  virtual ~GaborPatchItemFactory();

  virtual rutz::shared_ptr<SearchItem> make(const geom::vec2d& pos);

  rutz::shared_ptr<GaborPatchItem> makeForeground(const geom::vec2d& pos,
                                                  const double theta);

private:
  rutz::urand itsThetaRand;
  rutz::urand itsPhaseRand;
  rutz::urand itsContrastRand;

  double itsThetaJitter;
  double itsContrastJitter;

  double itsPeriod;
  double itsSigma;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_GABORPATCH_H_DEFINED

/** @file Psycho/GaborPatch.C represent the physical parameters of a gabor patch */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/GaborPatch.C $
// $Id: GaborPatch.C 9078 2007-12-11 21:11:45Z rjpeters $
//

// Code herein is derived from GroovX, also licensed under the GPL
// Copyright (c) 2002-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// [http://ilab.usc.edu/rjpeters/groovx/]

#ifndef PSYCHO_GABORPATCH_C_DEFINED
#define PSYCHO_GABORPATCH_C_DEFINED

#include "Psycho/GaborPatch.H"
#include "Image/geom.h"

#include <map>

namespace
{
  typedef std::map<GaborSpec, GaborPatch*> MapType;
  MapType theMap;

  const int NUM_THETA = 64;
  const int NUM_PHASE = 8;
  const double DELTA_THETA = M_PI / NUM_THETA;
  const double DELTA_PHASE = 2 * M_PI / NUM_PHASE;
}

GaborSpec::GaborSpec(double s, double o, double t, double p) :
  theta(DELTA_THETA * (int(geom::rad_0_pi(t)/DELTA_THETA + 0.5) % NUM_THETA)),
  phi(DELTA_PHASE * (int(geom::rad_0_2pi(p)/DELTA_PHASE + 0.5) % NUM_PHASE)),
  sigma(s),
  omega(o)
{}

bool GaborSpec::operator<(const GaborSpec& x) const
{
  if (theta < x.theta) return true;
  else if (theta == x.theta)
    {
      if (phi < x.phi) return true;
      else if (phi == x.phi)
        {
          if (sigma < x.sigma) return true;
          else if (sigma == x.sigma)
            {
              return (omega < x.omega);
            }
        }
    }

  return false;
}

GaborPatch::GaborPatch(const GaborSpec& spec)
  :
  itsSpec      ( spec ),
  itsSize      ( int(8*spec.sigma + 0.5) ),
  itsCenter    ( itsSize/2.0 + 0.5 ),
  itsCosTheta  ( cos(spec.theta) ),
  itsSinTheta  ( sin(spec.theta) ),
  itsSigmaSqr  ( 2.0* spec.sigma * spec.sigma ),
  itsData      ( itsSize, itsSize, ZEROS )
{
  Image<double>::iterator ptr = itsData.beginw();

  for (int y = 0; y < itsSize; ++y)
    for (int x = 0; x < itsSize; ++x)
      {
        const double fy = y - itsCenter;
        const double fx = x - itsCenter;

        const double dx = itsCosTheta * fx - itsSinTheta * fy;
        const double dy = itsSinTheta * fx + itsCosTheta * fy;

        const double dsqr  = (dx*dx + dy*dy) / itsSigmaSqr;

        const double sinus = cos(itsSpec.omega * dx + itsSpec.phi);

        const double gauss = exp(-dsqr);
        *ptr++ = sinus * gauss;
      }
}

GaborPatch::~GaborPatch()
{}

const GaborPatch& GaborPatch::lookup(const GaborSpec& spec)
{
  GaborPatch*& patch = theMap[spec];

  if (patch == 0)
    patch = new GaborPatch(spec);

  return *patch;
}

const GaborPatch& GaborPatch::lookup(double sigma, double omega,
                                     double theta, double phi)
{
  GaborSpec spec(sigma, omega, theta, phi);

  return lookup(spec);
}

GaborPatchItem::GaborPatchItem()
{}

GaborPatchItem::~GaborPatchItem()
{}

Image<double> GaborPatchItem::getPatch() const
{
  const GaborPatch& p =
    GaborPatch::lookup(itsSigma, 2*M_PI/itsPeriod,
                       this->theta, this->phi);

  return p.image(this->contrast);
}

GaborPatchItemFactory::GaborPatchItemFactory(int thetaSeed,
                                             int phaseSeed,
                                             int contrastSeed,
                                             double period, double sigma)
  :
  itsThetaRand(thetaSeed),
  itsPhaseRand(phaseSeed),
  itsContrastRand(contrastSeed),
  itsThetaJitter(0.0),
  itsContrastJitter(0.0),
  itsPeriod(period),
  itsSigma(sigma)
{
}

GaborPatchItemFactory::~GaborPatchItemFactory()
{}

rutz::shared_ptr<SearchItem>
GaborPatchItemFactory::make(const geom::vec2d& pos)
{
  rutz::shared_ptr<GaborPatchItem> el(new GaborPatchItem);

  el->type = SearchItem::BACKGROUND;
  el->pos = pos;

  el->phi = 2 * M_PI * itsPhaseRand.fdraw();
  el->theta = 2 * M_PI * itsThetaRand.fdraw();
  el->contrast = exp(-itsContrastJitter * itsContrastRand.fdraw());

  el->itsPeriod = this->itsPeriod;
  el->itsSigma = this->itsSigma;

  return el;
}

rutz::shared_ptr<GaborPatchItem>
GaborPatchItemFactory::makeForeground(const geom::vec2d& pos,
                                              const double theta)
{
  rutz::shared_ptr<GaborPatchItem> el(new GaborPatchItem);

  el->type = SearchItem::FOREGROUND;
  el->pos = pos;

  el->phi = 2 * M_PI * itsPhaseRand.fdraw();
  const double rand_theta = 2*M_PI * itsThetaRand.fdraw();
  el->theta =
    geom::rad_0_2pi(itsThetaJitter * (rand_theta - M_PI) +
                    (theta + M_PI_2));
  el->contrast = exp(-itsContrastJitter * itsContrastRand.fdraw());

  el->itsPeriod = this->itsPeriod;
  el->itsSigma = this->itsSigma;

  return el;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // PSYCHO_GABORPATCH_C_DEFINED

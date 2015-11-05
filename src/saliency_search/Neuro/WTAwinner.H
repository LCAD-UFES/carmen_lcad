/*!@file Neuro/WTAwinner.H a winner-take-all winner (i.e., covert shift of attention) */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/WTAwinner.H $
// $Id: WTAwinner.H 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef WTAWINNER_H_DEFINED
#define WTAWINNER_H_DEFINED

#include "Image/Point2D.H"
#include "Util/Assert.H"
#include "Util/MathFunctions.H"
#include "Util/SimTime.H"

//! This is a an open class representing a WTA winner (covert attention shift)
/*! This class is an open (all members public) container for a
Point2D<int>, time stamp, saliency voltage and possibly other things that
characterize a WTA winner, or covert shift of attention. Like Point2D<int>,
this class is fully inlined, so there is no WTAwinner.C file. */

class WTAwinner
{
public:
  // note: no default constructor, use WTAwinner::NONE() if you want a
  // dummy invalid WTAwinner

  static inline WTAwinner NONE();

  //! Constructor given a Point2D<int>, time and other stuff
  inline WTAwinner(const Point2D<int>& pp, const SimTime& tt,
                   const double svv, const bool bor);

  //! Return coordinates scaled down to scale of saliency map
  inline Point2D<int> getSMcoords(const int smlevel) const;

  //! Build from coordinates given at the scale of the saliency map
  static inline WTAwinner buildFromSMcoords(const Point2D<int> smcoords,
                                            const int smlevel,
                                            const bool useRandom,
                                            const SimTime& tt,
                                            const double svv,
                                            const bool bor);

  //! returns true if coordinates are not (-1, -1)
  inline bool isValid() const;

  // public data members:
  Point2D<int> p;  //!< The spatial coordinates within the full-resolution input
  Point2D<int> smpos; //<! The scaled-down coordinates within the saliency map
  SimTime t;  //!< The time, in seconds
  double sv;  //!< The saliency map voltage at winning location, in Volts
  bool boring;//!< True is this shift was made out of boredom
};

// ######################################################################
// #################### INLINED METHODS:
// ######################################################################
inline WTAwinner WTAwinner::NONE()
{
  return WTAwinner(Point2D<int>(-1,-1), SimTime::ZERO(), 0.0, false);
}

// ######################################################################
inline WTAwinner::WTAwinner(const Point2D<int>& pp, const SimTime& tt,
                            const double svv, const bool bor) :
  p(pp), smpos(-1, -1), t(tt), sv(svv), boring(bor)
{ }

// ######################################################################
inline Point2D<int> WTAwinner::getSMcoords(const int smlevel) const
{
  // the 0.49 offset below is to eliminate possible random jitter
  // introduced by buildFromSMcoords():
  return Point2D<int>(int(p.i / double(1 << smlevel) + 0.49),
                 int(p.j / double(1 << smlevel) + 0.49));
}

// ######################################################################
inline WTAwinner WTAwinner::buildFromSMcoords(const Point2D<int> smcoords,
                                              const int smlevel,
                                              const bool useRandom,
                                              const SimTime& tt,
                                              const double svv,
                                              const bool bor)
{
  WTAwinner result = WTAwinner::NONE();

  // scale coords up: By construction of our pyramids and of decX(),
  // decY() and friends, the coordinates of a saliency-map pixel
  // represent information around the top-left corner of that pixel,
  // not the center of tha pixel. So we simply scale the coordinates
  // up and do not attempt to center them over the extent of the
  // saliency map pixel:
  result.p.i = smcoords.i << smlevel;
  result.p.j = smcoords.j << smlevel;

  // add random jitter:
  if (useRandom)
    {
      int jitter = 1 << smlevel;  // saliency pixel size
      result.p.i +=
        int(jitter * (randomDouble()*0.98 - 0.49));  // up to +/- SMpix*0.49
      result.p.j +=
        int(jitter * (randomDouble()*0.98 - 0.49));
      if (result.p.i < 0) result.p.i = 0;
      if (result.p.j < 0) result.p.j = 0;
    }

  // initialize other members:
  result.smpos = smcoords;
  result.t = tt;
  result.sv = svv;
  result.boring = bor;

  // double-check that round-trip scaling-up/scaling-down gives us
  // back the same saliency map coords that we started with:
  ASSERT(result.getSMcoords(smlevel) == result.smpos);

  return result;
}

// ######################################################################
inline bool WTAwinner::isValid() const
{ return p.isValid(); }

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
#endif

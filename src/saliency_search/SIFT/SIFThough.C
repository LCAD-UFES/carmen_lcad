/*!@file SIFT/SIFThough.C Data structure for SIFT Hough transform */

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
// Primary maintainer for this file: James Bonaiuto <bonaiuto@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/SIFThough.C $
// $Id: SIFThough.C 5188 2005-08-02 18:09:34Z rjpeters $
//

#include "SIFT/SIFThough.H"
#include "Util/Assert.H"
#include "Util/Promotions.H"  // for clamped_convert<T>()
#include <cmath>

#define NBINX 8
#define NBINY 8
#define NBINO 8
#define NBINS 8

// ######################################################################
SIFThough::SIFThough() :
  itsData(NBINX * NBINY * NBINO * NBINS, 0.0F)
{ }

// ######################################################################
SIFThough::~SIFThough()
{ }

// ########################################################################
void SIFThough::addValue(const float dx, const float dy, const float doo,
                         const float ds, const float value)
{
  int xi0, xi1, yi0, yi1, oi0, oi1, si0, si1;   // bins
  float wx0, wy0, wo0, ws0, wx1, wy1, wo1, ws1; // corresponding weights

  // if close to bounds then the values go fully into the end bins,
  // otherwise they split between two adjacent bins. Note: a value of
  // 2.0 should equally split between bins 1 and 2:
  if (dx <= 0.5F)
    { xi0 = 0; xi1 = 0; wx0 = 0.5F; wx1 = 0.5F; }
  else if (dx >= NBINX-0.5F)
    { xi0 = 3; xi1 = 3; wx0 = 0.5F; wx1 = 0.5F; }
  else
    {
      const float xx = dx - 0.5F;
      xi0 = int(xx); xi1 = xi0 + 1;
      wx1 = xx - float(xi0); wx0 = 1.0F - wx1;
    }

  if (dy <= 0.5F)
    { yi0 = 0; yi1 = 0; wy0 = 0.5F; wy1 = 0.5F; }
  else if (dy >= NBINY-0.5F)
    { yi0 = 3; yi1 = 3; wy0 = 0.5F; wy1 = 0.5F; }
  else
    {
      const float yy = dy - 0.5F;
      yi0 = int(yy); yi1 = yi0 + 1;
      wy1 = yy - float(yi0); wy0 = 1.0F - wy1;
    }

  // the situation is different for orientation as we wrap around:
  if (doo <= 0.5F)
    {
      oi0 = 0; oi1 = 7;
      wo0 = 0.5F + doo; wo1 = 1.0F - wo0;
    }
  else if (doo >= NBINO-0.5F)
    {
      oi0 = 7; oi1 = 0;
      wo0 = 8.5F - doo; wo1 = 1.0F - wo0;
    }
  else
    {
      const float oo = doo - 0.5F;
      oi0 = int(oo); oi1 = oi0 + 1;
      wo1 = oo - float(oi0); wo0 = 1.0F - wo1;
    }

  if (ds <= 0.5F)
    { si0 = 0; si1 = 0; ws0 = 0.5F; ws1 = 0.5F; }
  else if (ds >= NBINS-0.5F)
    { si0 = 3; si1 = 3; ws0 = 0.5F; ws1 = 0.5F; }
  else
    {
      const float ss = ds - 0.5F;
      si0 = int(ss); si1 = si0 + 1;
      ws1 = ss - float(si0); ws0 = 1.0F - ws1;
    }

  // convention: we add 1 for each unit of o (our fastest varying
  // index), then NBINO for each unit of s, then NBINO*NBINS for each
  // unit of y, finally NBINO*NBINS*NBINY for each unit of x. Let's
  // populate our 16 bins:
  xi0 = xi0 * NBINO*NBINS*NBINY; xi1 = xi1 * NBINO*NBINS*NBINY;
  yi0 = yi0 * NBINO*NBINS; yi1 = yi1 * NBINO*NBINS;
  si0 = si0 * NBINO; si1 = si1 * NBINO;

  itsData[xi0 + yi0 + oi0 + si0] += value * wx0 * wy0 * wo0 * ws0;
  itsData[xi1 + yi0 + oi0 + si0] += value * wx1 * wy0 * wo0 * ws0;
  itsData[xi0 + yi1 + oi0 + si0] += value * wx0 * wy1 * wo0 * ws0;
  itsData[xi1 + yi1 + oi0 + si0] += value * wx1 * wy1 * wo0 * ws0;
  itsData[xi0 + yi0 + oi1 + si0] += value * wx0 * wy0 * wo1 * ws0;
  itsData[xi1 + yi0 + oi1 + si0] += value * wx1 * wy0 * wo1 * ws0;
  itsData[xi0 + yi1 + oi1 + si0] += value * wx0 * wy1 * wo1 * ws0;
  itsData[xi1 + yi1 + oi1 + si0] += value * wx1 * wy1 * wo1 * ws0;
  itsData[xi0 + yi0 + oi0 + si1] += value * wx0 * wy0 * wo0 * ws1;
  itsData[xi1 + yi0 + oi0 + si1] += value * wx1 * wy0 * wo0 * ws1;
  itsData[xi0 + yi1 + oi0 + si1] += value * wx0 * wy1 * wo0 * ws1;
  itsData[xi1 + yi1 + oi0 + si1] += value * wx1 * wy1 * wo0 * ws1;
  itsData[xi0 + yi0 + oi1 + si1] += value * wx0 * wy0 * wo1 * ws1;
  itsData[xi1 + yi0 + oi1 + si1] += value * wx1 * wy0 * wo1 * ws1;
  itsData[xi0 + yi1 + oi1 + si1] += value * wx0 * wy1 * wo1 * ws1;
  itsData[xi1 + yi1 + oi1 + si1] += value * wx1 * wy1 * wo1 * ws1;
}

// ######################################################################
void SIFThough::getPeak(float& dx, float& dy, float& doo, float& ds) const
{
  const uint siz = itsData.size();
  float maxi = -1.0e-30; uint maxindex = 0;

  // find index of the max:
  for (uint i = 0; i < siz; i ++)
    if (itsData[i] > maxi) { maxi = itsData[i]; maxindex = i; }

  // get back from maxindex to bin indices in all 4 dimensions:
  const uint ix = maxindex / (NBINO*NBINS*NBINY);
  maxindex -= ix * NBINO*NBINS*NBINY;
  const uint iy = maxindex / (NBINO*NBINS);
  maxindex -= iy * NBINO*NBINS;
  const uint is = maxindex / NBINO;
  maxindex -= is * NBINO;
  const uint io = maxindex;

  // we could try to do some interpolation or fitting of a
  // hyperparabola to the peak, etc. Maybe in the future:
  dx = float(ix); dy = float(iy); doo = float(io); ds = float(is);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

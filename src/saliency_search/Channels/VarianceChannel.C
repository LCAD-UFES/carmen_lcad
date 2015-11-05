/*!@file Channels/VarianceChannel.C */

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
// Primary maintainer for this file:
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/VarianceChannel.C $
// $Id: VarianceChannel.C 8939 2007-11-04 07:38:34Z itti $
//

#ifndef VARIANCECHANNEL_C_DEFINED
#define VARIANCECHANNEL_C_DEFINED

#include "Channels/VarianceChannel.H"

// ######################################################################
// Variance channel member definitions
// ######################################################################
VarianceChannel::VarianceChannel(OptionManager& mgr) :
  SingleChannel(mgr, "Variance", "variance", VARIANCE,
                rutz::shared_ptr< PyrBuilder<float> >(NULL)),
  itsMap()
{ }

// ######################################################################
VarianceChannel::~VarianceChannel()
{ }

// ######################################################################
bool VarianceChannel::outputAvailable() const
{ return itsMap.initialized(); }

// ######################################################################
void VarianceChannel::doInput(const InputFrame& inframe)
{
  const LevelSpec ls = itsLevelSpec.getVal();
  ASSERT(ls.levMin() == ls.levMax());
  ASSERT(ls.delMin() == 0 && ls.delMax() == 0);
  ASSERT(ls.levMin() == ls.mapLevel());
  ASSERT(inframe.grayFloat().initialized());

  const uint lev = ls.mapLevel();

  // figure out the tile and map sizes:
  const int siz = 1 << lev;
  const int w = inframe.getWidth();
  const int h = inframe.getHeight();
  itsMap.resize(w >> lev, h >> lev);

  // let's loop over the tiles and compute variance for each: We use
  // the well-known identity E[(X-E[X])^2] = E[X^2] - E[X]^2 and we
  // also use the small-sample approximation to the variance (divide
  // by n-1 instead of n):
  Image<float>::iterator dest = itsMap.beginw();
  for (int j = 0; j <= h-siz; j += siz)
    {
      const int jmax = std::min(j + siz, h);
      for (int i = 0; i <= w-siz; i += siz)
        {
          const int imax = std::min(i + siz, w);
          double sum = 0.0, sumsq = 0.0;
          Image<float>::const_iterator src =
            inframe.grayFloat().begin() + i + j * w;

          // compute sum and sum of squares:
          for (int jj = j; jj < jmax; jj ++)
            {
              for (int ii = i; ii < imax; ii ++)
                {
                  const double val = *src++;
                  sum += val; sumsq += val * val;
                }
              // skip to next input row:
              src += w - imax + i;
           }

          // now compute the variance:
          const double npix = double((jmax - j) * (imax - i));
          const double mean = sum / npix;
          double var = (sumsq - mean * mean * npix) / (npix - 1.0);
          if (var < 0.0) var = 0.0; // sometimes happens due to rounding

          *dest++ = sqrtf(float(var)) / 255.0F;
        }
    }
}

// ######################################################################
Image<float> VarianceChannel::getOutput()
{ return itsMap; }


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // VARIANCECHANNEL_C_DEFINED

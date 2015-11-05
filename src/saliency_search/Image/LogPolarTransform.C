/*!@file Image/LogPolarTransform.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/LogPolarTransform.C $
// $Id: LogPolarTransform.C 14984 2011-10-14 00:17:14Z dberg $
//

#ifndef IMAGE_LOGPOLARTRANSFORM_C_DEFINED
#define IMAGE_LOGPOLARTRANSFORM_C_DEFINED

#include "Image/LogPolarTransform.H"

#include "Image/MathOps.H"
#include "Image/Pixels.H"
#include "Image/Range.H"
#include "Util/MathFunctions.H"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

// ######################################################################
LogPolarTransform::LogPolarTransform()
{
}

// ######################################################################
LogPolarTransform::LogPolarTransform(const Dims& indims, const Dims& outdims,
                                     const double deg_per_pixel,
                                     const double step)
  :
  itsInDims(indims),
  itsOutDims(outdims)
{
  const int w = indims.w();
  const int h = indims.h();

  Range<double> xrng;
  Range<double> yrng;

  const double A = 3.0;
  const double B_x = 1.4;
  const double B_y = 1.8;

  for (int j = 0; j < h; ++j)
    for (int i = 0; i < w; ++i)
      {
        if (j > 0 && j < h-1 && i > 0 && i < w-1)
          continue;
        
        const double x = (i - w / 2.0) * deg_per_pixel;
        const double y = (j - h / 2.0) * deg_per_pixel;

        const double sx = (x < 0.0) ? 1.0 : -1.0;

        const double r = sqrt(x*x + y*y);
        const double th = atan2(y, fabs(x));

        const double X = sx * B_x * log(sqrt(r*r + 2*A*r*cos(th) + A*A) / A);
        const double Y = -B_y * atan(r*sin(th) / (r*cos(th) + A));

        // (s * X / B_x) = log(sqrt(r*r + 2*A*r*cos(th) + A*A) / A)
        // exp(s* X / B_x) = sqrt(r*r + 2*A*r*cos(th) + A*A) / A
        // A * exp(s* X / B_x) = sqrt(r*r + 2*A*r*cos(th) + A*A)

        xrng.merge(X);
        yrng.merge(Y);
      }

  const double scale = std::min((outdims.w() - 1) / xrng.range(),
                                (outdims.h() - 1) / yrng.range());

  const int dw = outdims.w();

  for (double j = 0; j < h; j += step)
    for (double i = 0; i < w; i += step)
      {
        const double x = (i - w / 2.0) * deg_per_pixel;
        const double y = (j - h / 2.0) * deg_per_pixel;

        const double sx = (x < 0.0) ? 1.0 : -1.0;

        const double r = sqrt(x*x + y*y);
        const double th = atan2(y, fabs(x));

        const double X = sx * B_x * log(sqrt(r*r + 2*A*r*cos(th) + A*A) / A);
        const double Y = -B_y * atan(r*sin(th) / (r*cos(th) + A));

        //if (!isFinite(X))
        //LINFO("i,j = %f,%f", i, j);

        // (s * X / B_x) = log(sqrt(r*r + 2*A*r*cos(th) + A*A) / A)
        // exp(s* X / B_x) = sqrt(r*r + 2*A*r*cos(th) + A*A) / A
        // A * exp(s* X / B_x) = sqrt(r*r + 2*A*r*cos(th) + A*A)
        if (isFinite(X))
        {
          const int NX = int((X - xrng.min()) * scale);
          const int NY = int((Y - yrng.min()) * scale);
          
          std::pair<int, int> p(int(j)*w + int(i), NY*dw + NX);
          if ((p.second > 0) && (p.second < itsOutDims.sz()))
            itsCoords.push_back(p);
        }
      }
}

// ######################################################################
template <class T>
Image<T> logPolarTransform(const LogPolarTransform& t,
                           const Image<T>& inp, const T bkg_color)
{
  ASSERT(inp.getDims() == t.itsInDims);

  typedef typename promote_trait<T, double>::TP TF;

  Image<TF> txf(t.itsOutDims, ZEROS);
  Image<int> counts(t.itsOutDims, ZEROS);

  typename Image<T>::const_iterator sptr = inp.begin();
  typename Image<TF>::iterator txfitr = txf.beginw();
  Image<int>::iterator citr = counts.beginw();

  for (size_t n = 0; n < t.itsCoords.size(); ++n)
    {
      txfitr[t.itsCoords[n].second] += TF(sptr[t.itsCoords[n].first]);
      citr[t.itsCoords[n].second] += 1;
    }

  txfitr = txf.beginw();
  citr = counts.beginw();

  Image<T> btxf(txf.getDims(), ZEROS);
  typename Image<T>::iterator btxfitr = btxf.beginw();

  for (int j = 0; j < txf.getHeight(); ++j)
    for (int i = 0; i < txf.getWidth(); ++i)
      {
        if (*citr == 0)
          *btxfitr = bkg_color;
        else
          *btxfitr = T(*txfitr / double(*citr));
        ++btxfitr; ++txfitr; ++citr;
      }

  return btxf;
}

// Include the explicit instantiations
#include "inst/Image/LogPolarTransform.I"

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_LOGPOLARTRANSFORM_C_DEFINED

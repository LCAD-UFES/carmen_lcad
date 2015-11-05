/*!@file Image/Convolver.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Convolver.C $
// $Id: Convolver.C 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef IMAGE_CONVOLVER_C_DEFINED
#define IMAGE_CONVOLVER_C_DEFINED

#include "Image/Convolver.H"

#include "Image/Coords.H"
#include "Image/CutPaste.H"
#include "Image/FilterOps.H"
#include "rutz/trace.h"

namespace
{
  Image<float> zeropad(const Image<float>& img,
                              const Dims& sz,
                              const Point2D<int>& loc)
  {
    Image<float> result(sz, ZEROS);
    inplacePaste(result, img, loc);
    return result;
  }
}

Convolver::Convolver(const Image<float>& kernel,
                     const Dims& imagesize)
  :
  itsSrcDims(imagesize),
  itsPadDims(itsSrcDims + kernel.getDims()),
  itsKernel(kernel),
  itsFwdFft(itsPadDims),
  itsInvFft(itsPadDims),
  itsFreqKernel() // delay initialization until the first fftConvolve()
{}

Image<float> Convolver::fftConvolve(const Image<float>& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  ASSERT(src.getDims() == itsSrcDims);

  if (!itsFreqKernel.initialized())
    {
      const Image<float> padkern =
        zeropad(itsKernel, itsPadDims, Point2D<int>(0,0));

      itsFreqKernel = itsFwdFft.fft(padkern);
    }

  const Point2D<int> padpoint(itsKernel.getDims()/2);

  const Image<float> zpsrc = zeropad(src, itsPadDims, padpoint);

  const Image<complexd> fft_zpsrc = itsFwdFft.fft(zpsrc);

  const Image<float> zpres =
    itsInvFft.ifft(fft_zpsrc*itsFreqKernel) / zpsrc.getSize();

  return crop(zpres, padpoint*2, itsSrcDims);
}

Image<float> Convolver::spatialConvolve(const Image<float>& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return convolve(src, itsKernel, CONV_BOUNDARY_ZERO);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_CONVOLVER_C_DEFINED

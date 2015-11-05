/*!@file Surprise/test-surprise.C test basic behavior of SurpriseMap and contents */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Surprise/test-surprise.C $
// $Id: test-surprise.C 6191 2006-02-01 23:56:12Z rjpeters $
//

#include "Image/MathOps.H"
#include "Raster/Raster.H"
#include "Surprise/SurpriseMap.H"

#define QLEN       3
#define UPDFAC     0.75
#define NUPDFAC    0.75
#define INIVAL     0.0
#define VARIANCE   25.0
#define NEIGHSIGMA 0.5f
#define LOCSIGMA   3.0f
#define NITER      20

int main(const int argc, const char** argv)
{
  if (argc != 2) LFATAL("USAGE: %s <image.pgm>", argv[0]);

  // let's start by trying out a single model: it starts with our
  // initial conditions and we give it a steady input of 255:
  SurpriseModelSP m(UPDFAC, INIVAL, VARIANCE);  // the model
  SurpriseModelSP s(UPDFAC, 255.0f, VARIANCE);  // the sample
  for (int i = 0; i < NITER; i ++)
    LINFO("iter = %d, mean = %f, stdev = %f, surprise = %f",
          i, m.getMean(), sqrt(m.getVar()), m.surprise(s));

  // get the input feature map:
  Image<byte> input = Raster::ReadGray(argv[1]);

  // convert to double:
  Image<double> in(input);

  // create sample variances:
  Image<double> invar(input.getDims(), NO_INIT);
  invar.clear(VARIANCE);

  // create SurpriseImage from our samples and their variances:
  SurpriseImage<SurpriseModelSP> sample(UPDFAC, in, invar);

  // create an ImageCache to accumulate our results:
  ImageCacheMinMax<float> cache;

  // create a surprise map:
  SurpriseMap<SurpriseModelSP> smap;
  smap.init(QLEN, UPDFAC, NUPDFAC, INIVAL, VARIANCE, NEIGHSIGMA, LOCSIGMA);

  // let's do it!
  for (int i = 0; i < NITER; i ++)
    {
      // get the surprise:
      Image<float> surp = smap.surprise(sample);
      float mi, ma; getMinMax(surp, mi, ma);
      LINFO("Done %d/%d: [%f .. %f]", i+1, NITER, mi, ma);

      // cache it:
      cache.push_back(surp);
    }

  // ok, let's save the results. First find the global max:
  Image<float> imax = cache.getMax();
  float mi, ma; getMinMax(imax, mi, ma);
  LINFO("Global max is %f", ma);

  for (int i = 0; i < NITER; i ++)
    {
      Image<byte> sav(cache.pop_front() * 255.0f / ma);
      Raster::WriteGray(sav, sformat("SURP%03d%s", i, argv[1]));
    }

  return 0;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

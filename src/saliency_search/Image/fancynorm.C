/*!@file Image/fancynorm.C Intrafeature competition with maxNormalize(). */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/fancynorm.C $
// $Id: fancynorm.C 9993 2008-07-29 00:04:18Z lior $
//

#include "Image/fancynorm.H"

#include "Util/Assert.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/FilterOps.H"
#include "Image/Kernels.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Image/Transforms.H"
#include "Image/PyramidOps.H"
#include "Util/StringConversions.H"
#include "Util/log.H"
#include "rutz/compat_cmath.h"
#include "rutz/trace.h"

#include <algorithm>
#include <cmath>

// uncomment this define to get some info on how maxNormalize works with your
// images. Intended for debugging purposes.
//#define SHOW_MAXNORM_INFO

// ######################################################################
template <class T>
Image<T> maxNormalize(const Image<T>& src,
                      const T mi, const T ma, const MaxNormType normtyp,
                      int nbiter, const Image<float> *lrexcit)
{
GVX_TRACE(__PRETTY_FUNCTION__);
#ifdef SHOW_MAXNORM_INFO
  T mm, mmm; getMinMax(src, mm, mmm);
  LDEBUG("mini = %f, maxi= %f", float(mm), float(mmm));
#endif

  Image<T> result;

  // do normalization depending on desired type:
  switch(normtyp)
    {
    case VCXNORM_NONE:
      result = maxNormalizeNone(src, mi, ma);
      break;
    case VCXNORM_MAXNORM:
      result = maxNormalizeStd(src, mi, ma);
      break;
    case VCXNORM_FANCYFAST:
      result = maxNormalizeFancyFast(src, mi, ma, nbiter);
      break;
    case VCXNORM_FANCYONE:
      nbiter = 1;
    case VCXNORM_FANCY:
      result = maxNormalizeFancy(src, mi, ma, nbiter, 1.0, lrexcit);
      break;
    case VCXNORM_FANCYLANDMARK:
      result = maxNormalizeFancyLandmark(src, mi, ma, nbiter);
      break;
    case VCXNORM_LANDMARK:
      result = maxNormalizeLandmark(src, mi, ma);
      break;
    case VCXNORM_FANCYWEAK:
      result = maxNormalizeFancy(src, mi, ma, nbiter, 0.5);
      break;
    case VCXNORM_FANCYVWEAK:
      result = maxNormalizeFancy(src, mi, ma, nbiter, 0.1);
      break;
    case VCXNORM_IGNORE:
      result = src;
      break;
    case VCXNORM_SURPRISE:
      result = src;
      break;
    case VCXNORM_STDEV:
      result = maxNormalizeStdev(src);
      break;
    case VCXNORM_STDEV0:
      result = maxNormalizeStdev0(src);
      break;
    default:
      LFATAL("Unknown normalization type: %d", int(normtyp));
    }

#ifdef SHOW_MAXNORM_INFO
  getMinMax(result, mm, mmm);
  LDEBUG("newmin = %f, newmax = %f", float(mm), float(mmm));
#endif

  return result;
}

// ######################################################################
template <class T>
Image<T> maxNormalizeNone(const Image<T>& src, const T mi, const T ma)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<T> result = src;

  // first clamp negative values to zero
  inplaceRectify(result);

  // then, normalize between mi and ma if not zero
  if (mi != T() || ma != T()) inplaceNormalize(result, mi, ma);

  return result;
}

// ######################################################################
// ##### maxNorm from Itti et al, IEEE-PAMI, 1998:
template <class T>
Image<T> maxNormalizeStd(const Image<T>& src, const T mi, const T ma)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());
  T zero = T();

  Image<T> result = src;

  // first clamp negative values to zero
  inplaceRectify(result);

  // then, normalize between mi and ma if not zero
  if (mi != zero || ma != zero) inplaceNormalize(result, mi, ma);

  const int w = result.getWidth();
  const int h = result.getHeight();

  // normalize between mi and ma and multiply by (max - mean)^2

  // we want to detect quickly local maxes, but avoid getting local mins
  T thresh = T(mi + (ma - mi) / 10.0);

  // then get the mean value of the local maxima:
  double lm_sum = 0.0;
  int lm_nb = 0;
  for (int j = 1; j < h - 1; j ++)
    for (int i = 1; i < w - 1; i ++)
      {
        int index = i + w * j;
        T val = result.getVal(index);
        if (val >= thresh &&
            val >= result.getVal(index - w) &&
            val >= result.getVal(index + w) &&
            val >= result.getVal(index - 1) &&
            val >= result.getVal(index + 1))  // local max
          {
            lm_sum += double(val);
            lm_nb ++;
          }
      }
  // scale factor is (max - mean_local_max)^2:
  if (lm_nb > 1)
    {
      double factor = ma - (T)(lm_sum / double(lm_nb));
      result *= T(factor * factor);
    }
  else if (lm_nb == 1)  // a single narrow peak
    result *= (ma * ma);
  else
    {
      /* LERROR("No local maxes found !!"); */
    }

  return result;
}

// ######################################################################
// ##### fancyNorm from Itti et al, JEI, 2001 -- FAST implementation:
template <class T>
Image<T> maxNormalizeFancyFast(const Image<T>& src, const T mi, const T ma,
                               const int nbiter)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());
  T zero = T();

  Image<T> result = src;

  // first clamp negative values to zero
  inplaceRectify(result);

  // then, normalize between mi and ma if not zero
  if (mi != zero || ma != zero) inplaceNormalize(result, mi, ma);

  const int w = result.getWidth();
  const int h = result.getHeight();

  int wh = std::max(w, h);
  // we want an inhibitory 1D sigma of at least FANCYISIG % of wh
  // we know that 1D sigma of the lowpass9 filter is 1.5 pixel
  // each level down a pyramid, 1D sigma increases by sqrt(5)
  int ilev = 1+(int)(log(float(wh) / 1.5 * FANCYISIG / 100.0) /
                     log(sqrt(5)));

  // This is needed for log() <= -2*log(sqrt(5)), i.e. wh <= 1.2 = 6 / 5
  if (ilev < 0) ilev = 0;

  int elev = (int)(log(float(wh) / 1.5 * FANCYESIG / 100.0) /
                   log(sqrt(5)));

  // This is needed for log() <= -log(sqrt(5)), i.e. wh <= 33.5 = 75 / sqrt(5)
  if (elev < 0) elev = 0;

  for (int i = 0; i < nbiter; ++i)
    {
      ImageSet<T> inhibPyr = buildPyrGaussian(result, 0, ilev+1, 9);
      Image<T> excit = inhibPyr[elev]; // excitatory center
      Image<T> inhib = inhibPyr[ilev]; // inhibitory surround
      excit *= T(FANCYCOEX); inhib *= T(FANCYCOIN);
      excit = rescale(excit, w, h);
      inplaceAttenuateBorders(excit, wh/16);
      inhib = rescale(inhib, w, h);
      excit -= inhib;

      T globinhi, minim, maxim; getMinMax(result, minim, maxim);
      globinhi = (T)(0.01 * FANCYINHI * maxim);

      result += excit;        // we get broad inhibition from everywhere
      result += -globinhi;    // we get fixed global inhibition
      inplaceRectify(result);
    }

  return result;
}

// ######################################################################
// ##### fancyNorm from Itti et al, JEI, 2001 -- FULL implementation:
template <class T>
Image<T> maxNormalizeFancy(const Image<T>& src, const T mi, const T ma,
                           const int nbiter, const double weakness,
                           const Image<float>* lrexcit)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // Normalize using fancy normalization: multiple iterations of
  // filtering by a DoG

  ASSERT(src.initialized());
  T zero = T();

  Image<T> result = src;

  // first clamp negative values to zero
  inplaceRectify(result);

  // then, normalize between mi and ma if not zero
  if (mi != zero || ma != zero) inplaceNormalize(result, mi, ma);

  const int w = result.getWidth();
  const int h = result.getHeight();

  int siz = std::max(w, h);
  int maxhw = std::max(0, std::min(w, h) / 2 - 1);
  // build separable Gaussians for DoG computation:
  float esig = (float(siz) * FANCYESIG) * 0.01F;
  float isig = (float(siz) * FANCYISIG) * 0.01F;
  Image<float> gExc =
    gaussian<float>(FANCYCOEX/(esig*sqrt(2.0*M_PI)) * weakness, esig, maxhw);
  Image<float> gInh =
    gaussian<float>(FANCYCOIN/(isig*sqrt(2.0*M_PI)) * weakness, isig, maxhw);

  for (int i = 0; i < nbiter; ++i)
    {
      if (lrexcit)           // tuned long-range excitation
        {
          Image<T> tmp(result);
          tmp = downSize(tmp, w / (1<<LRLEVEL), h / (1<<LRLEVEL));
          tmp = convolve(tmp, *lrexcit, CONV_BOUNDARY_ZERO);  // full influence
          tmp += T(1.0);     // will be used as multiplicative excitation
          inplaceClamp(tmp, T(1.0), T(1.25));  // avoid crazyness
          tmp = rescale(tmp, w, h);
          T tmi, tma; getMinMax(tmp, tmi,tma);
          LDEBUG("lrexcit(%d): [%f..%f]", i, double(tmi), double(tma));
          result *= tmp;
        }

      Image<T> excit = sepFilter(result, gExc, gExc, CONV_BOUNDARY_CLEAN); // excitatory part
      Image<T> inhib = sepFilter(result, gInh, gInh, CONV_BOUNDARY_CLEAN); // inhibitory part
      excit -= inhib;
      T minim, maxim; getMinMax(result, minim, maxim);
      T globinhi = (T)(0.01 * FANCYINHI * maxim);

      result += excit;        // we get broad inhibition from everywhere
      result += -globinhi;    // we get fixed global inhibition
      inplaceRectify(result);
      //      sigmoid(FANCYG, FANCYH, FANCYS);
    }

  return result;
}

// ######################################################################
// ##### fancyNorm from Itti et al, JEI, 2001 -- FULL implementation:
// adapted to find landmarks
template <class T>
Image<T> maxNormalizeFancyLandmark(const Image<T>& src, const T mi, const T ma,
                           const int nbiter)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // Normalize using fancy normalization: multiple iterations of
  // filtering by a DoG

  ASSERT(src.initialized());
  T zero = T();

  Image<T> result = src;

  // first clamp negative values to zero
  inplaceRectify(result);

  // then, normalize between mi and ma if not zero
  if (mi != zero || ma != zero) inplaceNormalize(result, mi, ma);

  const int w = result.getWidth();
  const int h = result.getHeight();

  int siz = std::max(w, h);
  int maxhw = std::max(0, std::min(w, h) / 2 - 1);
  // build separable Gaussians for DoG computation:
  float esig = (float(siz)*2)*0.01;
  float isig = (float(siz)*3)*0.01;
  Image<float> gExc = gaussian<float>(FANCYCOEX/(esig*sqrt(2.0*M_PI)),
                                      esig, maxhw);
  Image<float> gInh = gaussian<float>(FANCYCOIN/(isig*sqrt(2.0*M_PI)),
                                      isig, maxhw);

  for (int i = 0; i < nbiter; ++i)
    {
      Image<T> excit = sepFilter(result, gExc, gExc, CONV_BOUNDARY_CLEAN); // excitatory part
      Image<T> inhib = sepFilter(result, gInh, gInh, CONV_BOUNDARY_CLEAN); // inhibitory part
      excit -= inhib;
      T minim, maxim; getMinMax(result, minim, maxim);
      T globinhi = (T)(0.01 * FANCYINHI * maxim);

      result += excit;        // we get broad inhibition from everywhere
      result += -globinhi;    // we get fixed global inhibition
      inplaceRectify(result);
      //      sigmoid(FANCYG, FANCYH, FANCYS);
    }

  return result;
}

// ######################################################################

template <class T>
Image<T> maxNormalizeLandmark(const Image<T>& src, const T mi, const T ma)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());
  Image<T> result = src;
  inplaceNormalize(result, (T)MAXNORMMIN, (T)MAXNORMLANDMARK);

  // AIM: to find goodness of map
  // compare maps b4 and after maxnormalization fancy
  // comp_next = # components after maxnormalisation of map
  // area_prev = area of component b4 maxnorm
  // area_next = "     "         after "
  // activity_prev = activity of component b4 maxnorm

  Image<byte> target;
  Image<T> comp_map;
  T max, thresh;
  float area_prev = 0.0f;
  int comp_next = 0;
  double activity_prev = 0.0, real_activity = 0.0, standout = 0.0;
  double sum_prev = 0.0, sum_next = 0.0;
  float goodness_map = 0.0f; // is this a good map for landmarks?
  float landmarkness = 0.0f; // is this a good landmark?

  thresh = (T) MAXNORMMIN;
  Point2D<int> max_locn;
  findMax(result, max_locn, max);

  // find values b4 maxnorm
  while(max > thresh)
    {
      area_prev = 0.0f;
      activity_prev = 0.0;
      int val = segmentLandmark(result, max_locn, target, activity_prev,
                               standout, area_prev);

      if (val != -1)
        {
          result =  result - result * (Image<T>)target;
          real_activity = (double)src.getVal(max_locn);
          findMax(result, max_locn, max);

           if(area_prev > 0.125)
            {
              sum_prev += (double)area_prev * activity_prev;

              // find peaks within this component
              int peaks = findPeaks((Image<T>)target,(T) MAXNORMMIN,
                                    (T) MAXNORMLANDMARK,
                                    sum_next);
              if(peaks == 0)
                {
                  peaks = 10; // very irregular, hence cancelled
                }

              // compute landmarkness value
              LINFO("area_prev = %f, peaks = %d",
                    area_prev, peaks);
              LINFO("real_activity  = %f, activity_prev = %f, standout = %f",
                    real_activity, activity_prev, standout);
              landmarkness = (float) (real_activity * standout);
              //landmarkness = (float)real_activity /
              //  (area_prev * peaks * peaks);
              //landmarkness *= 10.0f;
              LINFO("-----landmarkness = %f", landmarkness);

              if (!comp_map.initialized())
                comp_map = (Image<T>)target / 255.0f * (T)landmarkness ;
              else comp_map += (Image<T>)target / 255.0f * (T)landmarkness;
            }
        }
      else break;
    }

  // find peaks in this image
  result = src;
  comp_next = findPeaks(result,(T) MAXNORMMIN, (T) MAXNORMLANDMARK,
                                sum_next);

  // compute goodness
  if(comp_next == 0)
    {
      comp_next = 10; // very irregular, hence cancelled
    }
  if (sum_prev != 0.0)
    goodness_map = (float)(sum_next / (sum_prev * comp_next * comp_next)) ;

  goodness_map = pow(goodness_map, 0.5);
  LINFO("# of comps = %d", comp_next);

  LINFO("goodness_map = %f", goodness_map);

  // all components found
  if (comp_map.initialized())
    comp_map *= (T)(goodness_map);

  else comp_map.resize(src.getDims(), true);
  return comp_map;
}

// ######################################################################
template <class T>
int findPeaks(const Image<T>& src, const T mi, const T ma,
              double& sum_activity)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<T> result = src;
  result = maxNormalize(result, mi, ma, VCXNORM_FANCY);
  T max, thresh; Point2D<int> max_locn; double standout = 0.0;
  int peaks = 0;
  thresh = (T)((ma - mi)/10 + mi);
  sum_activity = 0.0;

  findMax(result, max_locn, max);
  while(max > thresh)
    {
      float area = 0.0f; double activity = 0.0; Image<byte> target;
      int val = segmentLandmark(result, max_locn, target,
                                   activity, standout, area);
      if(val != -1)
        {
          if(area > 0.0675)
            {
              peaks++;
              sum_activity += (double)area * activity;
            }
          result = result - result * (Image<T>)target;
          findMax(result, max_locn, max);
        }
      else break;
    }
  return peaks;
}

//******************************************************************
template <class T>
float goodness_map(const Image<T>& src)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(src.initialized());

  Image<T> result = src;
  inplaceNormalize(result, (T)MAXNORMMIN, (T)MAXNORMLANDMARK);

  // AIM: to find goodness of map
  // compare maps b4 and after maxnormalization fancy
  // comp_next = # components after maxnormalisation of map
  // area_prev = area of component b4 maxnorm
  // area_next = "     "         after "
  // activity_prev = activity of component b4 maxnorm

  Image<byte> target;
  Image<T> comp_map;
  T max, thresh;
  float area_prev = 0.0f;
  int comp_next = 0;
  double standout = 0.0, activity_prev = 0.0;
  double sum_prev = 0.0, sum_next = 0.0;
  float goodness_map = 0.0f; // is this a good map for landmarks?


  thresh = (T) MAXNORMMIN;
  Point2D<int> max_locn;
  findMax(result, max_locn, max);

  // find values b4 maxnorm
  while(max > thresh)
    {
      area_prev = 0.0f;
      activity_prev = 0.0, standout = 0.0 ;
      int val = segmentLandmark(result, max_locn, target, activity_prev,
                                standout, area_prev);

      if (val != -1)
        {
          result =  result - result * (Image<T>)target;
          findMax(result, max_locn, max);

           if(area_prev > 0.125)
            {
              sum_prev += (double)area_prev * activity_prev;
            }
        }
      else break;
    }
  result = src;
  comp_next = findPeaks(result,(T) MAXNORMMIN, (T) MAXNORMLANDMARK,
                        sum_next);

  // compute goodness
  if(comp_next == 0)
    {
      comp_next = 10; // very irregular, hence cancelled
    }
  if (sum_prev != 0.0)
    goodness_map = (float)(sum_next / (sum_prev * comp_next * comp_next)) ;
  LINFO("# of comps = %d", comp_next);
  LINFO("goodness_map = %f", goodness_map);

  return goodness_map;
}

// ######################################################################
template <class T>
Image<T> maxNormalizeStdev(const Image<T>& src)
{
  // renormalize the image to have stdev=1 and minval=0

  const float s = float(stdev(src));

  if (s == 0.0f)
    return Image<T>(src.getDims(), ZEROS);

  typedef typename promote_trait<T, float>::TP TF;

  Image<TF> result(src);
  result /= s;

  TF mini, maxi; getMinMax(result, mini, maxi);
  result -= mini;

  return Image<T>(result);
}

// ######################################################################
template <class T>
Image<T> maxNormalizeStdev0(const Image<T>& src)
{
  // renormalize the image to have mean=0 and stdev=1

  const float s = float(stdev(src));
  const float u = float(mean(src));

  if (s == 0.0f)
    return Image<T>(src.getDims(), ZEROS);

  typedef typename promote_trait<T, float>::TP TF;

  Image<TF> result(src);
  result -= u; // mean -> 0
  result /= s; // stdev -> 1

  return Image<T>(result);
}


// ######################################################################
// converters for MaxNormType
// ######################################################################

std::string convertToString(const MaxNormType val)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  return maxNormTypeName(val);
}

void convertFromString(const std::string& str, MaxNormType& val)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  // CAUTION: assumes types are numbered and ordered!
  for (int i = 0; i < NBMAXNORMTYPES; i ++)
    if (str.compare(maxNormTypeName(MaxNormType(i))) == 0)
      { val = MaxNormType(i); return; }

  conversion_error::raise<MaxNormType>(str);
}



// ######################################################################
#include "inst/Image/fancynorm.I"

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

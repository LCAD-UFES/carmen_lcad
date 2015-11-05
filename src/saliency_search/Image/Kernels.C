/*@file Image/Kernels.C Functions to construct various kinds of
  filter kernels
*/

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
// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Kernels.C $
// $Id: Kernels.C 13757 2010-08-04 22:11:39Z siagian $
//

#include "Image/Kernels.H"

#include "Image/Image.H"
#include "Image/MathOps.H"  // for mean()
#include "Image/CutPaste.H" // for crop()
#include "Util/Assert.H"
#include "Util/MathFunctions.H"
#include "rutz/compat_cmath.h"
#include <cmath>


// ######################################################################
template <class T>
Image<T> dogFilterHmax(const float theta, const float gamma, const int size, const float div)
{
  // resize the data buffer

  Image<T> dest(size, size, NO_INIT);

  // change the angles in degree to the those in radian : rotation degree
  float rtDeg = M_PI / 180.0F * theta;

  // calculate constants
  float lambda = size*2.0F/div;
  float sigma = lambda*0.8F;
  float sigq = sigma*sigma;
  int center    = (int)ceil(size/2.0F);
  int filtSizeL = center-1;
  int filtSizeR = size-filtSizeL-1;
  typename Image<T>::iterator aptr = dest.beginw();

  // for DOG operation : to give orientation, it uses omit y-directional
  // component
  for (int x = -filtSizeL; x <= filtSizeR; x ++)
    for (int y = -filtSizeL; y <= filtSizeR; y ++) {
      if(sqrt(x*x +y*y)>size/2.0F){
        *aptr++ = 0;
      }
      else{
        float rtX =  y * cos(rtDeg) - x * sin(rtDeg);
        float rtY = y * sin(rtDeg) + x * cos(rtDeg);
        *aptr++ = T(exp(-(rtX*rtX + gamma*gamma*rtY*rtY)/(2.0F*sigq)) *
                    cos(2*M_PI*rtX/lambda));
      }
    }
  return dest;
}

// ######################################################################
template <class T>
Image<T> dogFilter(const float stddev, const float theta, const int halfsize)
{
  // resize the data buffer
  int size = halfsize;
  if (size <= 0) size = int(ceil(stddev * sqrt(7.4F)));
  Image<T> dest(2 * size + 1, 2 * size + 1, NO_INIT);

  // change the angles in degree to the those in radian : rotation degree
  float rtDeg = M_PI / 180.0F * theta;

  // calculate constants
  float sigq = stddev * stddev;
  typename Image<T>::iterator aptr = dest.beginw();

  // for DOG operation : to give orientation, it uses omit y-directional
  // component
  for (int x = -size; x <= size; x ++)
    for (int y = -size; y <= size; y ++) {
      float rtX = x * cos(rtDeg) + y * sin(rtDeg);
      float rtY = -x * sin(rtDeg) + y * cos(rtDeg);
      *aptr++ = T((((rtX * rtX)) / sigq - 1.0F)/sigq *
                  exp(-(rtX * rtX + rtY * rtY) / (2.0F * sigq)));
    }
  return dest;
}



// ######################################################################
template <class T>
Image<T> dogFilter(const float stddev, const int halfsize)
{
  float sFact = 4.0F;

  // resize the data buffer
  int size = halfsize;
  if (size <= 0) size = int(ceil(2*sFact*stddev));
  Image<T> dest(2 * size + 1, 2 * size + 1, NO_INIT);

  // calculate constants
  float NC = 1.0F/(2.0F*M_PI*stddev * stddev);
  float NS = 1.0F/(sFact*sFact) * NC;
  typename Image<T>::iterator aptr = dest.beginw();

  // for DOG operation : to give orientation, it uses omit y-directional
  // component
  for (int x = -size; x <= size; x ++)
    for (int y = -size; y <= size; y ++) {
      float xy = -1.0F * (x*x + y*y)/(2.0F*stddev*stddev);
      *aptr++ = T( NS*exp((1.0F/(sFact*sFact))*xy) - NC*exp(xy));
    }
  return dest;
}

// ######################################################################
template <class T>
Image<T> dogFilterHmax(const float stddev, const float theta,
                       const int cBegin, const int cEnd)
{
  const Image<T> srcFilt = dogFilter<T>(stddev, theta);
  const Rectangle cropRegion =
    Rectangle::tlbrI(cBegin, cBegin, cEnd-1, cEnd-1);
  const Image<T> cropped = crop(srcFilt, cropRegion, false);
  const Image<T> diff = cropped - T(mean(cropped));
  return diff / T(sum(squared(diff)));
}

// ######################################################################
template <class T>
Image<T> gaborFilter(const float stddev, const float period,
                     const float phase, const float theta,
                     const float bg, const float ampl)
{
  // figure the proper size for the result
  int size = int(ceil(stddev * sqrt(-2.0F * log(exp(-5.0F)))));

  Image<T> result(2 * size + 1, 2 * size + 1, NO_INIT);

  // change the angles in degree to the those in radians:
  float psi = M_PI / 180.0F * phase;

  float rtDeg = M_PI / 180.0F * theta;

  // calculate constants:
  float omega = (2.0F * M_PI) / period;
  float co = cos(rtDeg), si = sin(rtDeg);
  float sigq = 2.0F * stddev * stddev;
  typename Image<T>::iterator aptr = result.beginw();
  float a = float(ampl), b = float(bg);

  // compute gabor:
  for (int y = -size; y <= size; y ++)
    for (int x = -size; x <= size; x ++)
      *aptr++ = T(a * cos(omega * (x * co + y * si) + psi) *
                  exp(-(x * x + y * y) / sigq) + b);
  return result;
}

// ######################################################################
template <class T>
Image<T> gaborFilter(const float scale, const float theta)
{
  // figure the proper size for the result
  int size = int(scale * 12+0.5f);

  Image<T> result(2 * size + 1, 2 * size + 1, NO_INIT);

  // calculate constants:
  float cosTheta = cos(theta + M_PI/2), sinTheta = sin(theta + M_PI/2);
  float normConst = 50/M_PI/scale/scale;
  typename Image<T>::iterator aptr = result.beginw();

  // compute gabor:
  for (int y = -size; y <= size; ++y)
    for (int x = -size; x <= size; ++x)
    {
      if (x*x+y*y > size*size)
        *aptr++ = T(); //zero out side the circle
      else
      {
        float xx = (x*cosTheta + y * sinTheta)/scale;
        float yy = (y*cosTheta - x * sinTheta)/scale;
        *aptr++ = T(exp(-(4.0F*xx*xx+yy*yy)/100.0F)/normConst);
      }
    }


  return result;
}


// ######################################################################
//pixRGB version for colored gabor patches

Image<PixRGB<byte> > gaborFilterRGB(const float stddev, const float freq,
                                    const float theta,const float hueShift)
{


  // figure the proper size for the result
   int size = int(ceil(stddev * sqrt(-2.0F * log(exp(-5.0F)))));

  typedef std::complex<float> complexf;

  complexf gauss, sinu, gResult, ctemp;
  complexf i(0.0, 1.0);

  Image<PixHSV<float> > resultHSV(2 * size + 1, 2 * size + 1, NO_INIT);


  // change the angles in degree to the those in radians:
  float rtDeg = M_PI / 180.0F * theta;
  Image<PixHSV<float> >::iterator aptr = resultHSV.beginw();


  float tempf = 0.0;
  PixHSV<float> tempPix;

  int totalcnt = (2*size + 1)*(2*size + 1);
  std::vector<float> hVals(totalcnt), vVals(totalcnt);

  int cnt=0;
  for (int y = -size; y <= size; y ++)
   for (int x = -size; x <= size; x ++)
        {
          gauss   = std::complex<float>(exp(-(x * x + y * y) /(stddev*stddev)),0.0);
          ctemp   = std::complex<float>(0.0,(2.0*M_PI*freq)*(x*cos(rtDeg) + y*sin(rtDeg)));
          sinu    = std::exp(ctemp);
          gResult = gauss*sinu;

          tempf = arg(gResult);

          tempf = tempf * 180/M_PI; //convert back to degs
          hVals[cnt]   = tempf;
          vVals[cnt++] = real(gResult)*real(gResult);

           *aptr++ = tempPix;

          }


    //find min max of hues and values and rescale h from 0-360 and v from 0-255
  cnt = 0;
  float minH = hVals[0], maxH = hVals[0], minV = vVals[0], maxV = vVals[0];
  while(cnt!=totalcnt)
    {
      if(hVals[cnt] < minH)
        minH = hVals[cnt];
      else if(hVals[cnt] > maxH)
        maxH = hVals[cnt];


      if(vVals[cnt] < minV)
        minV = vVals[cnt];
      else if(vVals[cnt] > maxV)
        maxV = vVals[cnt];


      cnt++;
    }

  aptr = resultHSV.beginw();

  for(int i = 0; i <= totalcnt ; i++)
    {
      hVals[i] = ((hVals[i] - minH)/(maxH - minH)) * 360;
      hVals[i] = fabs(hVals[i] + hueShift);

      if(hVals[i] > 360)
        hVals[i] = hVals[i] - 360;

      tempPix.setH(hVals[i]);
      tempPix.setS(100.0) ;
      tempPix.setV(vVals[i]);
      *aptr++ = tempPix;
    }

  LINFO("stddev:%f freq:%f theta:%f hueShift:%f",stddev,freq,theta,hueShift);

  Image<PixRGB<float> > resultfloat = resultHSV;
  Image<PixRGB<byte> > result = resultfloat*256;

  return result;

}

// ######################################################################
template <class T>
Image<T> gaborFilter2(const float stddev, const float period,
                      const float phase, const float theta,
                      const float sigMod = 1.0F,
                      const float amplitude = 128.0F)
{
  // figure the proper size for the result
  int size = int(ceil(stddev * sqrt(-2.0F * log(exp(-5.0F)))));

  Image<T> result(2 * size + 1, 2 * size + 1, NO_INIT);

  // change the angles in degree to the those in radians:
  float psi = M_PI / 180.0F * phase;
  float rtDeg = M_PI / 180.0F * theta;

  // calculate constants:
  float omega = (2.0F * M_PI) / period;
  float co = cos(rtDeg), si = sin(rtDeg);
  float sigq = sigMod * stddev * stddev;
  typename Image<T>::iterator aptr = result.beginw();

  // compute gabor:
  for (int y = -size; y <= size; y ++)
    for (int x = -size; x <= size; x ++)
      *aptr++ = T(amplitude*(cos(omega * (x * co + y * si) + psi) *
                             exp(-(x * x + y * y) / sigq)));
  // who's your daddy?
  return result;
}

// ######################################################################
// Produces a Gabor kernel with optionally unequal major+minor axis lengths.
Image<float> gaborFilter3(const float major_stddev, const float minor_stddev,
                          const float period, const float phase,
                          const float theta, int size)
{
  const float max_stddev =
    major_stddev > minor_stddev ? major_stddev : minor_stddev;

  // figure the proper size for the result
  if (size == -1) size = int(ceil(max_stddev * sqrt(-2.0F * log(exp(-5.0F)))));
  else size = size/2;
  //const int size = int(ceil(max_stddev * 2));

  Image<float> result(2 * size + 1, 2 * size + 1, NO_INIT);

  // change the angles in degree to the those in radians:
  const float psi = M_PI / 180.0F * phase;
  const float rtDeg = M_PI / 180.0F * theta;

  // calculate constants:
  const float omega = (2.0F * M_PI) / period;
  const float co = cos(rtDeg), si = sin(rtDeg);
  const float major_sigq = 2.0F * major_stddev * major_stddev;
  const float minor_sigq = 2.0F * minor_stddev * minor_stddev;
  Image<float>::iterator aptr = result.beginw();

  // compute gabor:
  for (int y = -size; y <= size; ++y)
    for (int x = -size; x <= size; ++x)
      {
        const float major = x*co + y*si;
        const float minor = x*si - y*co;
        *aptr++ = float(cos(omega * major + psi)
                        * exp(-(major*major) / major_sigq)
                        * exp(-(minor*minor) / minor_sigq));
      }

  // make the result have (mean == 0.0)
  return (result - mean(result));
}

// ######################################################################

template <class T>
Image<T> gaussian2D(const float stddev, const float sigMod = 1.0F,
                    const float amplitude = 255.0F)
{
  // figure the proper size for the result
  int size = int(ceil(stddev * sqrt(-2.0F * log(exp(-5.0F)))));
  //float norm = 40.010587F;
  Image<T> result(2 * size + 1, 2 * size + 1, NO_INIT);
  T resultX,resultY;

  typename Image<T>::iterator aptr = result.beginw();
  for (int y = -size; y <= size; y++)
    {
      resultY = T(/*((1.0F/(stddev*sqrt(2.0F*M_PI)))*/1.0F*
                  exp(-((y*y)/(stddev*stddev*sigMod))));
      for (int x = -size; x <= size; x++)
        {
          resultX = T(/*((1.0F/(stddev*sqrt(2.0F*M_PI)))*/1.0F*
                      exp(-((x*x)/(stddev*stddev*sigMod))));
          *aptr++ = T((resultX*resultY)*amplitude);
        }
    }
  return result;
}

// ######################################################################
template <class T>
Image<T> gaussianBlob(const Dims& dims, const Point2D<int>& center,
                      const float sigmaX, const float sigmaY)
{
  Image<T> ret(dims, NO_INIT);
  const int w = dims.w(), h = dims.h();

  const float fac = 0.5f / (M_PI * sigmaX * sigmaY);
  const float vx = -0.5f / (sigmaX * sigmaX);
  const float vy = -0.5f / (sigmaY * sigmaY);
  for (int jj = 0; jj < h; jj ++)
    {
      float vydy2 = float(jj - center.j); vydy2 *= vydy2 * vy;
      for (int ii = 0; ii < w; ii ++)
        {
          float dx2 = float(ii - center.i); dx2 *= dx2;

          ret.setVal(ii, jj, clamped_convert<T>(fac * expf(vx * dx2 + vydy2)));
        }
    }
  return ret;
}

// ######################################################################
template <class T>
Image<T> gaussianBlobUnnormalized(const Dims& dims, const Point2D<int>& center,
                                  const float sigmaX, const float sigmaY)
{
  Image<T> ret(dims, NO_INIT);
  const int w = dims.w(), h = dims.h();

  const float vx = -0.5f / (sigmaX * sigmaX);
  const float vy = -0.5f / (sigmaY * sigmaY);
  for (int jj = 0; jj < h; jj ++)
    {
      float vydy2 = float(jj - center.j); vydy2 *= vydy2 * vy;
      for (int ii = 0; ii < w; ii ++)
        {
          float dx2 = float(ii - center.i); dx2 *= dx2;

          ret.setVal(ii, jj, clamped_convert<T>(expf(vx * dx2 + vydy2)));
        }
    }
  return ret;
}

// ######################################################################
namespace
{
  // Helper function for binomialKernel():
  double binomial(int n, int k)
  {
    /*
      binomial coefficient refresher:

      k:    0   1   2   3   4   5   6   7   8
      N:  +------------------------------------
      0   |   1
      1   |   1   1
      2   |   1   2   1
      3   |   1   3   3   1
      4   |   1   4   6   4   1
      5   |   1   5  10  10   5   1
      6   |   1   6  15  20  15   6   1
      7   |   1   7  21  35  35  21   7   1
      8   |   1   8  28  56  70  56  28   8   1

      n-choose-k := n! / k! (n-k)!

      == n*(n-1)*...(n-(k-1)) / k*(k-1)*...*1

      gamma(n+1) == n! for integral n

      lgamma(x) := ln(gamma(x))

      By doing the factorial multiplications as additions in the
      log-domain, we avoid numerical overflow in intermediate results.
    */
    return floor(0.5 + exp(lngamma(n+1) - lngamma(k+1) - lngamma(n-k+1)));
  }
}

Image<float> binomialKernel(const int sz)
{
  ASSERT(sz > 0);

  Image<float> result(sz, 1, NO_INIT);

  const int N = sz-1;

  const double div = pow(2.0, double(N));

  // compute a series of N-choose-k, where N == (sz-1), with k running
  // from 0 to N, inclusive
  for (int k = 0; k <= N; ++k)
    {
      result.setVal(k, 0, float(binomial(N, k) / div));
    }

  return result;
}


// ######################################################################
template <class T>
Image<T> grating(const int width, const int height,
                 const float period, const float phase, const float theta)
{
  Image<T> result(width, height, NO_INIT);

  // change the angles in degree to the those in radian
  float psi = M_PI / 180.0F * phase;
  float rtDeg = M_PI / 180.0F * theta;

  // calculate constants
  float omega = (2.0F * M_PI) / period;
  float co = cos(rtDeg), si = sin(rtDeg);
  typename Image<T>::iterator aptr = result.beginw();

  // for gabor operation
  for (int y = 0; y < height; y ++)
    for (int x = 0; x < width; x ++)
      *aptr++ = T(cos(omega * (x * co + y * si) + psi));
  return result;
}

// ######################################################################
template <class T>
Image<T> gaussian(const float coeff, const float sigma,
                  const int maxhw, const float threshperc)
{
  // determine size: keep only values larger that threshperc*max (here max=1)
  int hw = (int)(sigma * sqrt(-2.0F * log(threshperc / 100.0F)));

  // if kernel turns out to be too large, cut it off:
  if (maxhw > 0 && hw > maxhw) hw = maxhw;

  // allocate image for result:
  Image<T> result(2 * hw + 1, 1, NO_INIT);

  // if coeff is given as 0, compute it from sigma:
  float c = coeff;
  if (coeff == 0.0f) c = 1.0f / (sigma * sqrtf(2.0f * float(M_PI)));

  // build both halves simultaneously
  result.setVal(hw, T(c));
  const float sig22 = - 0.5F / (sigma * sigma);
  for (int i = 1; i <= hw; ++i)
    {
      T val = T(c * exp(float(i * i) * sig22));
      result.setVal(hw + i, val);
      result.setVal(hw - i, val);
    }
  return result;
}

// ######################################################################
template <class T>
Image<T> longRangeExcFilter(const float factor, const float orient)
{
  int siz = 9;         // kernel size
  float peak = 3.0F;   // [3] excitation is max 'peak' pixels away from center
  float sigmaR = 1.0F; // [1] radial sigma, in pixels
  float sigmaT = 5.0F; // [15] angular sigma, in degrees

  Image<T> result(siz, siz, NO_INIT); int mid = (siz - 1) / 2;
  float o = orient * M_PI / 180.0F, st = sigmaT * M_PI / 180.0F;
  st = 2.0F * st * st; float sr = 2.0F * sigmaR * sigmaR;

  typename Image<T>::iterator aptr = result.beginw();
  for (int j = 0; j < siz; j ++)
    for (int i = 0; i < siz; i ++)
      {
        float x = (float)(i - mid), y = (float)(j - mid);
        float r = sqrt(x * x + y * y) - peak, t = atan2(y, x);
        float odiff = t - o;
        while (odiff < -M_PI_2) odiff += M_PI;
        while (odiff > M_PI_2) odiff -= M_PI;
        *aptr++ = T(factor * exp(- r*r/sr - odiff*odiff/st));
      }
  return result;
}

// ######################################################################
template <class T>
Image<T> fixationMask(const Dims& dims, const Point2D<int>& fixation,
                      const float pixperdeg, const T maxval, const float sigma)
{
  Image<T> img(dims, ZEROS);
  return fixationMask(img, fixation, pixperdeg, maxval, sigma);
}

// ######################################################################
template <class T>
Image<T> fixationMask(const Image<T>& mask, const Point2D<int>& fixation,
                      const float pixperdeg, const T maxval, const float sigma)
{
  Image<T> img = mask;
  typename Image<T>::iterator src = img.beginw();
  float mvf = float(maxval); int w = img.getWidth(), h = img.getHeight();
  float sigfac = 0.5f / (sigma * sigma);
  int fi = fixation.i, fj = fixation.j;
  for (int j = 0; j < h; j ++)
    for (int i = 0; i < w; i ++)
      {
        float ang = sqrt((i-fi) * (i-fi) + (j-fj) * (j-fj)) / pixperdeg;
        float val = exp(- ang * ang * sigfac);
        *src++ += T(mvf * val);  // range 0..maxval
      }
  return img;
}

// ######################################################################
Image<byte> twofiftyfives(Dims d)
{
  Image<byte> result(d,NO_INIT);
  result.clear(255);
  return result;
}

// ######################################################################
Image<byte> twofiftyfives(int w, int h)
{
  return twofiftyfives(Dims(w,h));
}

// ######################################################################
Image<byte> twofiftyfives(int w)
{
  return twofiftyfives(Dims(w,w));
}

// Include the explicit instantiations
#include "inst/Image/Kernels.I"

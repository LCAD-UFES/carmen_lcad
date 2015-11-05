/*!@file Util/MathFunctions.C Miscellaneous math functions */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/MathFunctions.C $
// $Id: MathFunctions.C 14590 2011-03-10 22:25:43Z lior $
//

#include "Util/MathFunctions.H"
#include "Util/log.H"
#include <cmath>
#include <sys/types.h> // for getpid()
#include <unistd.h>    // for getpid()

// ######################################################################
bool isFinite(const byte& arg) { return true; }
bool isFinite(const int16& arg) { return true; }
bool isFinite(const int32& arg) { return true; }

#if !defined(MISSING_ISINF)
bool isFinite(const float& arg) { return !isinf(arg); }
bool isFinite(const double& arg) { return !isinf(arg); }
#elif defined(HAVE_STD_ISFINITE)
bool isFinite(const float& arg) { return std::isfinite(arg); }
bool isFinite(const double& arg) { return std::isfinite(arg); }
#elif defined(HAVE_ISFINITE)
bool isFinite(const float& arg) { return isfinite(arg); }
bool isFinite(const double& arg) { return isfinite(arg); }
#elif defined(HAVE___ISFINITEF)
// Use built-in functions, e.g. on Mac OSX
bool isFinite(const float& arg) { return __isfinitef(arg); }
bool isFinite(const double& arg) { return __isfinited(arg); }
#else
bool isFinite(const float& arg) { return true; }
bool isFinite(const double& arg) { return true; }
#endif

// #####################################################################
void initRandomNumbers()
{
#ifdef HAVE_SRAND48
  srand48(time(0) + getpid() * 123);
#endif
  srand(time(0) + getpid() * 345);
}

// #####################################################################
void initRandomNumbersZero()
{
#ifdef HAVE_SRAND48
  srand48(0);
#endif
  srand(0);
}

// #####################################################################
double randomDouble()
{
#ifdef HAVE_SRAND48
  return drand48();
#else
  return double(rand()) / (double(RAND_MAX) + 1.0);
#endif
}

// #####################################################################
double randomDoubleFromNormal(const double s)
{
        double sum = 0;
        for(int i=0; i<12; i++){
                sum += randomDouble()*2*s - s;
        }
        return sum/2;
}

// #####################################################################
int randomUpToIncluding(const int n)
{
  return randomUpToNotIncluding(n + 1);
}

// #####################################################################
int randomUpToNotIncluding(const int n)
{
  // NOTE: we don't do (RAND%n) because the low-order bits of the
  // random number may be less random than the high-order bits

  const int result = int(randomDouble() * n);

  // randomDouble() is supposed to be in [0.0, 1.0), but just in case
  // randomDouble()*n happens to equal n due to floating-point
  // weirdness, let's return n-1 to fulfill the "NotIncluding" part of
  // our contract:
  if (result == n)
    return n-1;

  return result;
}

// #####################################################################
double ran2 (int& idum)
{
  const int IM1 = 2147483563, IM2 = 2147483399;
  const double AM = (1.0/IM1);
  const int IMM1 = IM1-1;
  const int IA1 = 40014, IA2 = 40692, IQ1 = 53668, IQ2 = 52774;
  const int IR1 = 12211, IR2 = 3791, NTAB = 32;
  const int NDIV = 1+IMM1/NTAB;
  const double EPS = 3.0e-16, RNMX = 1.0-EPS;

  int j, k;
  static int idum2 = 123456789, iy = 0;
  static int iv[NTAB];
  double temp;

  if (idum <= 0) {
    idum = (idum == 0 ? 1 : -idum);
    idum2=idum;
    for (j=NTAB+7;j>=0;j--) {
      k=idum/IQ1;
      idum=IA1*(idum-k*IQ1)-k*IR1;
      if (idum < 0) idum += IM1;
      if (j < NTAB) iv[j] = idum;
    }
    iy=iv[0];
  }
  k=idum/IQ1;
  idum=IA1*(idum-k*IQ1)-k*IR1;

  if (idum < 0) idum += IM1;
  k=idum2/IQ2;
  idum2=IA2*(idum2-k*IQ2)-k*IR2;
  if (idum2 < 0) idum2 += IM2;
  j=iy/NDIV;
  iy=iv[j]-idum2;
  iv[j] = idum;
  if (iy < 1) iy += IMM1;
  if ((temp=AM*iy) > RNMX) return RNMX;
  else return temp;
}

// ######################################################################
int getIdum(const bool useRandom)
{
  if (useRandom)
    return -(time(NULL) + 17 * getpid());
  else
    return -123456;
}

// ######################################################################
double gasdev (int& idum) {
  static int iset = 0;
  static double gset;

  double fac, rsq, v1, v2;
  if (idum < 0) iset = 0;
  if (iset == 0) {
    do {
      v1 = 2.0*ran2(idum)-1.0;
      v2 = 2.0*ran2(idum)-1.0;
      rsq = v1*v1 + v2*v2;
    } while (rsq >= 1.0 || rsq == 0.0);

    fac = std::sqrt(-2.0*std::log(rsq)/rsq);
    gset = v1*fac; iset = 1;
    return v2*fac;
  } else {
    iset = 0;
    return gset;
  }
}

// ######################################################################
double expdev(int& idum)
{
  double dum;
  do dum = ran2(idum); while (dum == 0.0);
  return -log(dum);
}

/* Lanczos method for real x > 0;
 * gamma=7, truncated at 1/(z+8)
 * [J. SIAM Numer. Anal, Ser. B, 1 (1964) 86]
 */
double lngamma(double x)
{

#ifdef HAVE_LGAMMA
  return lgamma(x);
#else

  // ripped from GSL (see above).
  /* coefficients for gamma=7, kmax=8  Lanczos method */
  static const double lanczos_7_c[9] = {
    0.99999999999980993227684700473478,
    676.520368121885098567009190444019,
    -1259.13921672240287047156078755283,
    771.3234287776530788486528258894,
    -176.61502916214059906584551354,
    12.507343278686904814458936853,
    -0.13857109526572011689554707,
    9.984369578019570859563e-6,
    1.50563273514931155834e-7
  };

  x -= 1.0; /* Lanczos writes z! instead of Gamma(z) */

  double Ag = lanczos_7_c[0];
  for (int k = 1; k <= 8; k++) { Ag += lanczos_7_c[k] / (x+k); }

  /* (x+0.5)*log(x+7.5) - (x+7.5) + LogRootTwoPi_ + log(Ag(x)) */
  const double term1 = (x + 0.5) * log((x + 7.5) / M_E);
  const double term2 = D_LOG_SQRT_2_PI + log(Ag);
  return term1 + (term2 - 7.0);
#endif
}

// ripped from GSL (see above). Compute probability of getting a value
// k from a Poisson distribution with mean mu:
double poisson(const unsigned int k, const double mu)
{
  double lf = lngamma(k+1);
  return exp(log(mu) * k - lf - mu);
}

//computes the AUC, inspired by ROC, between two data sets
//the intended use is to compare human eye movements to a random
//model
double AUC(const float* model, const float* rand,
                                size_t sm, size_t sr,
                                const double step)
{
        //if ((max(model) > 1.0F) || (max(rand) > 1.0F)
        //|| (min(model) < 0.0F) || (min(rand) < 0.0F))
        //LFATAL("Vectors must be normalized");

    double auc = 0.0;
    double last_tp = 1.0;
    double last_fp = 1.0;

    for (double thresh = 0.0; thresh <= 1.0+step; thresh+=step)
    {
        uint tp = 0;
        uint fp = 0;
        for (size_t jj = 0; jj < sm; ++jj)
            if (model[jj] >= thresh)
                tp++;

        for (size_t jj = 0; jj < sr; ++jj)
            if (rand[jj] >= thresh)
                fp++;

        double fpr = (double)fp/(double)sr;
        double tpr = (double)tp/(double)sm;

        auc+= ( (last_fp - fpr) * (tpr) ) +
            ( 0.5 * (last_fp - fpr) * (last_tp - tpr) );

        last_tp = tpr;
        last_fp = fpr;
    }
    return auc;
}

Point2D<float> getEllipseFromCov(const Image<double>& cov)
{
  ASSERT(cov.getWidth() == 2 && cov.getHeight() == 2);

  //Using matlab Compute the Mahalanobis radius of the ellipsoid that encloses
  //the desired probability mass.
  //k = chi2inv(p, 2); where p = 0.9

  double k = 4.605170185988091; //From matlab 

  //Compute the eigen values
  double trace = (cov.getVal(0,0) + cov.getVal(1,1))/2;

  double a = cov.getVal(0,0) - trace;
  double b = cov.getVal(0,1);

  double ab2 = sqrt((a*a) + (b*b));

  double l1 = ab2 + trace;
  double l2 = -ab2 + trace;

  return Point2D<float>(k*sqrt(l1), k*sqrt(l2));

}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

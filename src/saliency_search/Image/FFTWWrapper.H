/*! @file Image/FFTWWrapper.H -- find the Fourier Transform of an image  */

// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/FFTWWrapper.H $
// $Id: FFTWWrapper.H 9993 2008-07-29 00:04:18Z lior $

#ifndef HAHAH_H
#define HAHAH_H

//compile with g++ FFTWWrapper.cpp -L ~/fftwinstall/lib/ -I ~/fftwinstall/include/ -l fftw3 -o FFTWWrapper -lm
#include <stdlib.h>
#include <math.h>
#include <float.h>

#ifdef HAVE_FFTW3_H
#include <fftw3.h>
#endif

class FFTWWrapper
{

public:
  FFTWWrapper(int width, int height);
  void init(double *image);
  ~FFTWWrapper();
  void compute(double **magspec);
  //! arraydata implementation
  void compute(double *magspec);
private:
#ifdef HAVE_FFTW3_H
  inline double mag(fftw_complex c)
  {
    return (sqrt(c[0]*c[0] + c[1]*c[1]));
  }

  double *in;
  int imageheight;
  int imagewidth;
  fftw_complex *out;
  fftw_plan p;
#endif // HAVE_FFTW3_H
};

#endif

/*! @file Image/FFTWWrapper.C --
  Bare bones wrapper for freely dowloadable
  fftw fast Fourier transfom C libraries, which also come
  with lots of helpful documentation from fftw.com
  Basic idea is, assuming you've got a certain image size
  that you're going to be computing FTs for again and again, you
  pass in a pointer to the image memory location at initialization,
  and FFTW comes up with "a plan".
  Then you can call the compute fuction repeatedly as needed,
  passing in a memory location for it to put the FT.
  FFTW returns just a halfwidth+1 size image,
  because the rest is redundatnt, but I make the fullsize image here.


  Christopher Ackerman
  7/30/2003
*/


// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/FFTWWrapper.C $
// $Id: FFTWWrapper.C 10794 2009-02-08 06:21:09Z itti $

#include "Image/FFTWWrapper.H"
#include "Util/log.H"

FFTWWrapper::FFTWWrapper(int width, int height){
#ifndef HAVE_FFTW3_H
  LFATAL("you must have fftw3 installed to use this function");
#else
  imagewidth  = width;
  imageheight = height;

  // Allocate here to make sure that we only every allocate
  // with the constructor since we free with the destructor
  out = (double(*)[2])fftw_malloc(sizeof(fftw_complex) *
                                  imageheight * (imagewidth/2+1));
  if(out == NULL)
    LINFO("Memory allocation error");

#endif
}

// ######################################################################
void FFTWWrapper::init(double *image){
#ifndef HAVE_FFTW3_H
  LFATAL("you must have fftw3 installed to use this function");
#else
  in = image;
  p = fftw_plan_dft_r2c_2d(imageheight, imagewidth, in, out, FFTW_MEASURE);
#endif
}

// ######################################################################
void FFTWWrapper::compute(double** magspec){
#ifndef HAVE_FFTW3_H
  LFATAL("you must have fftw3 installed to use this function");
#else
  fftw_execute(p);

  const int halfwidth = imagewidth/2 + 1;
  // make other half --skip zero freq?
  for(int i = 0; i < imageheight; i++)
    for(int j = 0; j < halfwidth; j++)
      magspec[i][j] = mag(out[i*halfwidth+j/*+1*/]);
#endif
}

// ######################################################################
void FFTWWrapper::compute(double* magspec){
#ifndef HAVE_FFTW3_H
  LFATAL("you must have fftw3 installed to use this function");
#else
  fftw_execute(p);

  const int halfwidth = imagewidth/2 + 1;
  // make other half --skip zero freq?
  for(int i = 0; i < imageheight; i++)
    for(int j = 0; j < halfwidth; j++)
      magspec[i * halfwidth + j] = mag(out[i*halfwidth+j/*+1*/]);
#endif
}

// ######################################################################
FFTWWrapper::~FFTWWrapper(){
#ifndef HAVE_FFTW3_H
  // don't use LFATAL in a destructor
  LERROR("you must have fftw3 installed to use this function");
#else
  fftw_destroy_plan(p);
  // Since the image data comes from outside, we do not own the data
  // so do not free it. We have only malloc'd  out, so we free that
  //fftw_free(in);
  fftw_free(out);
#endif
}

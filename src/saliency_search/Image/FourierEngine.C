/*!@file Image/FourierEngine.C Thin wrapper around the fftw3 library */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/FourierEngine.C $
// $Id: FourierEngine.C 9993 2008-07-29 00:04:18Z lior $
//

#ifndef IMAGE_FOURIERENGINE_C_DEFINED
#define IMAGE_FOURIERENGINE_C_DEFINED

#include "Image/FourierEngine.H"

#include "rutz/trace.h"

#ifdef HAVE_FFTW3_H

namespace
{
  // see comments in FourierEngine::fft() about USE_FFTW_GURU
  const bool USE_FFTW_GURU = true;

// #define USE_FFTW_THREADS

  void init_threads()
  {
    // in order to use threads, we need to also link against
    // -lfftw3_threads and -lpthread on the command-line
#ifdef USE_FFTW_THREADS
    static bool inited = false;

    if (!inited)
      {
        inited = true;
        if (fftw_init_threads() == 0)
          LFATAL("fftw_init_threads() failed");

        fftw_plan_with_nthreads(4);
      }
#endif
  }

  template <class T>
  struct fftw;

  template <>
  struct fftw<float>
  {
    static float* alloc_real(size_t n)
    {
      return static_cast<float*>(fftwf_malloc(sizeof(float)*n));
    }

    static complexf* alloc_complex(size_t n)
    {
      return static_cast<complexf*>(fftwf_malloc(sizeof(complexf)*n));
    }

    static void dealloc(void* mem)
    {
      fftwf_free(mem);
    }

    static void* plan_dft_r2c_2d(const Dims& d,
                                 float* src,
                                 complexf* dst)
    {
      GVX_TRACE(__PRETTY_FUNCTION__);

      return fftwf_plan_dft_r2c_2d
        (d.h(), d.w(),
         src,
         // this reinterpret_cast is 'safe', because according to fftw
         // documentation, the fftwf_complex type (just a typedef for
         // float[2]) should be bit-compatible with both the C99
         // complex type as well as the c++ std::complex<float> type
         reinterpret_cast<fftwf_complex*>(dst),
         FFTW_MEASURE);
    }

    static void* plan_dft_c2r_2d(const Dims& d,
                                 complexf* src,
                                 float* dst)
    {
      GVX_TRACE(__PRETTY_FUNCTION__);

      return fftwf_plan_dft_c2r_2d
        (d.h(), d.w(),
         reinterpret_cast<fftwf_complex*>(src),
         dst,
         FFTW_MEASURE);
    }

    static void destroy_plan(void* p)
    {
      fftwf_destroy_plan(static_cast<fftwf_plan>(p));
    }

    static void execute_dft_r2c(void* p,
                                const float* src,
                                complexf* dst)
    {
      fftwf_execute_dft_r2c
        (static_cast<fftwf_plan>(p),
         const_cast<float*>(src),
         reinterpret_cast<fftwf_complex*>(dst));
    }

    static void execute_dft_c2r(void* p,
                                const complexf* src,
                                float* dst)
    {
      fftwf_execute_dft_c2r
        (static_cast<fftwf_plan>(p),
         reinterpret_cast<fftwf_complex*>(const_cast<complexf*>(src)),
         dst);
    }

    static void execute(void* p)
    {
      fftwf_execute(static_cast<fftwf_plan>(p));
    }
  };

  template <>
  struct fftw<double>
  {
    static double* alloc_real(size_t n)
    {
      return static_cast<double*>(fftw_malloc(sizeof(double)*n));
    }

    static complexd* alloc_complex(size_t n)
    {
      return static_cast<complexd*>(fftw_malloc(sizeof(complexd)*n));
    }

    static void dealloc(void* mem)
    {
      fftw_free(mem);
    }

    static void* plan_dft_r2c_2d(const Dims& d,
                                 double* src,
                                 complexd* dst)
    {
      GVX_TRACE(__PRETTY_FUNCTION__);

      return fftw_plan_dft_r2c_2d
        (d.h(), d.w(),
         src,
         // this reinterpret_cast is 'safe', because according to fftw
         // documentation, the fftw_complex type (just a typedef for
         // double[2]) should be bit-compatible with both the C99
         // complex type as well as the c++ std::complex<double> type
         reinterpret_cast<fftw_complex*>(dst),
         FFTW_MEASURE);
    }

    static void* plan_dft_c2r_2d(const Dims& d,
                                 complexd* src,
                                 double* dst)
    {
      GVX_TRACE(__PRETTY_FUNCTION__);

      return fftw_plan_dft_c2r_2d
        (d.h(), d.w(),
         reinterpret_cast<fftw_complex*>(src),
         dst,
         FFTW_MEASURE);
    }

    static void destroy_plan(void* p)
    {
      fftw_destroy_plan(static_cast<fftw_plan>(p));
    }

    static void execute_dft_r2c(void* p,
                                const double* src,
                                complexd* dst)
    {
      fftw_execute_dft_r2c
        (static_cast<fftw_plan>(p),
         const_cast<double*>(src),
         reinterpret_cast<fftw_complex*>(dst));
    }

    static void execute_dft_c2r(void* p,
                                const complexd* src,
                                double* dst)
    {
      fftw_execute_dft_c2r
        (static_cast<fftw_plan>(p),
         reinterpret_cast<fftw_complex*>(const_cast<complexd*>(src)),
         dst);
    }

    static void execute(void* p)
    {
      fftw_execute(static_cast<fftw_plan>(p));
    }
  };
}

#endif // HAVE_FFTW3_H

// ##############################################################
template <class T>
FourierEngine<T>::FourierEngine(const Dims& d)
  :
  itsInputDims(d),
  itsSrc(0),
  itsDst(0),
  itsPlan(0)
{
GVX_TRACE(__PRETTY_FUNCTION__);

#ifndef HAVE_FFTW3_H
  LFATAL("you must have fftw3 installed to use this function");
#else
  init_threads();

  // Use fftw_malloc() here instead of plain malloc() or new[],
  // because fftw_malloc() is supposed to provide stronger alignment
  // guarantees, so that more efficient 'aligned' SIMD operations can
  // be used.
  itsSrc = fftw<T>::alloc_real(itsInputDims.h() * itsInputDims.w());

  if (itsSrc == 0)
    LFATAL("memory allocation failure");

  itsDst = fftw<T>::alloc_complex(itsInputDims.h() * (itsInputDims.w()/2+1));

  if (itsDst == 0)
    {
      fftw<T>::dealloc(itsSrc);
      LFATAL("memory allocation failure");
    }

  // delay initialization of itsPlan until the first call to fft()
#endif // HAVE_FFTW3_H
}

// ##############################################################
template <class T>
FourierEngine<T>::~FourierEngine()
{
GVX_TRACE(__PRETTY_FUNCTION__);
#ifndef HAVE_FFTW3_H
  // don't LFATAL() in a destructor
  LERROR("you must have fftw3 installed to use this function");
#else
  if (itsPlan != 0)
    fftw<T>::destroy_plan(itsPlan);
  fftw<T>::dealloc(itsDst);
  fftw<T>::dealloc(itsSrc);
#endif
}

// ##############################################################
template <class T>
Image<std::complex<T> > FourierEngine<T>::fft(const Image<T>& x)
{
GVX_TRACE(__PRETTY_FUNCTION__);
#ifndef HAVE_FFTW3_H
  LFATAL("you must have fftw3 installed to use this function");
  /* can't happen */ return Image<std::complex<T> >();
#else
  ASSERT(x.getDims() == itsInputDims);

  if (itsPlan == 0)
    {
      itsPlan = fftw<T>::plan_dft_r2c_2d(itsInputDims, itsSrc, itsDst);

      if (itsPlan == 0)
        {
          fftw<T>::dealloc(itsSrc); fftw<T>::dealloc(itsDst);
          LFATAL("couldn't construct fftw_plan");
        }
    }

  if (USE_FFTW_GURU)
    {
      // It's possible to use fftw's guru interface to get some
      // speed-up (~5-8% faster) by avoiding copying the input+output
      // arrays; note that this relies on the input+output arrays
      // having the maximal alignment that fftw expects (16-byte
      // boundary), which is now guaranteed by the fact that
      // ArrayData<T> uses invt_allocate() to create its T array.

      Image<std::complex<T> > result(itsInputDims.w()/2 + 1,
                                     itsInputDims.h(), NO_INIT);

      fftw<T>::execute_dft_r2c
        (itsPlan, x.getArrayPtr(), result.getArrayPtr());

      return result;
    }
  else
    {
      std::copy(x.begin(), x.end(), itsSrc);

      {
        GVX_TRACE("fftw_execute-forward");
        fftw<T>::execute(itsPlan);
      }

      return Image<std::complex<T> >(itsDst,
                             itsInputDims.w()/2 + 1, itsInputDims.h());
    }
#endif // HAVE_FFTW3_H
}

// ##############################################################
template <class T>
FourierInvEngine<T>::FourierInvEngine(const Dims& d)
  :
  itsOutputDims(d),
  itsSrc(0),
  itsDst(0),
  itsPlan(0)
{
GVX_TRACE(__PRETTY_FUNCTION__);
#ifndef HAVE_FFTW3_H
  LFATAL("you must have fftw3 installed to use this function");
#else
  init_threads();

  // See comments in FourierEngine::FourierEngine()

  itsSrc = fftw<T>::alloc_complex(itsOutputDims.h() * (itsOutputDims.w()/2+1));

  if (itsSrc == 0)
    LFATAL("memory allocation failure");

  itsDst = fftw<T>::alloc_real(itsOutputDims.h() * itsOutputDims.w());

  if (itsDst == 0)
    {
      fftw<T>::dealloc(itsSrc);
      LFATAL("memory allocation failure");
    }

  // delay initialization of itsPlan until the first call to ifft()
#endif // HAVE_FFTW3_H
}

// ##############################################################
template <class T>
FourierInvEngine<T>::~FourierInvEngine()
{
GVX_TRACE(__PRETTY_FUNCTION__);
#ifndef HAVE_FFTW3_H
  // don't LFATAL() in a destructor
  LERROR("you must have fftw3 installed to use this function");
#else
  if (itsPlan != 0)
    fftw<T>::destroy_plan(itsPlan);
  fftw<T>::dealloc(itsDst);
  fftw<T>::dealloc(itsSrc);
#endif
}

// ##############################################################
template <class T>
Image<T> FourierInvEngine<T>::ifft(const Image<std::complex<T> >& x)
{
GVX_TRACE(__PRETTY_FUNCTION__);
#ifndef HAVE_FFTW3_H
  LFATAL("you must have fftw3 installed to use this function");
  /* can't happen */ return Image<T>();
#else
  ASSERT(x.getWidth() == itsOutputDims.w()/2+1);
  ASSERT(x.getHeight() == itsOutputDims.h());

  if (itsPlan == 0)
    {
      itsPlan = fftw<T>::plan_dft_c2r_2d(itsOutputDims, itsSrc, itsDst);

      if (itsPlan == 0)
        {
          fftw<T>::dealloc(itsSrc); fftw<T>::dealloc(itsDst);
          LFATAL("couldn't construct fftw_plan");
        }
    }

  if (USE_FFTW_GURU)
    {
      Image<T> result(itsOutputDims, NO_INIT);

      // need a copy of the input, because fftw_execute will destroy
      // its input when operating in the complex->real direction
      Image<std::complex<T> > xcopy(x);

      fftw<T>::execute_dft_c2r
        (itsPlan,
         xcopy.getArrayPtr(),
         result.getArrayPtr());

      return result;
    }
  else
    {
      std::copy(x.begin(), x.end(), itsSrc);

      {
        GVX_TRACE("fftw_execute-reverse");
        fftw<T>::execute(itsPlan);
      }

      return Image<T>(itsDst, itsOutputDims.w(), itsOutputDims.h());
    }
#endif
}

// ##############################################################
template <class T>
Image<T> magnitude(const Image<std::complex<T> >& img)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const std::complex<T>* c = img.getArrayPtr();

  Image<T> result(img.getDims(), NO_INIT);
  for (typename Image<T>::iterator
         itr = result.beginw(), stop = result.endw();
       itr != stop;
       ++itr)
    {
      *itr = std::abs(*c);
      ++c;
    }
  return result;
}

// ##############################################################
template <class T>
Image<T> logmagnitude(const Image<std::complex<T> >& img)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const std::complex<T>* c = img.getArrayPtr();

  Image<T> result(img.getDims(), NO_INIT);
  for (typename Image<T>::iterator
         itr = result.beginw(), stop = result.endw();
       itr != stop;
       ++itr)
    {
      const double a = std::abs(*c);
      *itr = log(a > 0.0 // avoid doing log(0)
                 ? a
                 : std::numeric_limits<double>::min());
      ++c;
    }
  return result;
}

// ##############################################################
template <class T>
Image<T> phase(const Image<std::complex<T> >& img)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const std::complex<T>* c = img.getArrayPtr();

  Image<T> result(img.getDims(), NO_INIT);
  for (typename Image<T>::iterator
         itr = result.beginw(), stop = result.endw();
       itr != stop;
       ++itr)
    {
      *itr = std::arg(*c);
      ++c;
    }
  return result;
}

// ##############################################################
template <class T>
Image<T> cartesian(const Image<T>& in,
                        const Dims& dims)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<T> result(dims, ZEROS);
  Image<int> counts(dims, ZEROS);

  const int nfreq = dims.w();
  const int nori = dims.h();

  const double theta_max = M_PI;
  const double rad_max =
    sqrt(in.getWidth()*in.getWidth() +
         in.getHeight()*in.getHeight()/4);

  const int h = in.getHeight(), h2 = in.getHeight()/2;

  const Point2D<int> left(-1, 0);
  const Point2D<int> right(1, 0);
  const Point2D<int> up(0, -1);
  const Point2D<int> down(0, 1);

  for (int y = 0; y < h; ++y)
    if (y < h2)
      for (int x = 0; x < in.getWidth(); ++x)
        {
          const double th = atan2(y, x);
          const double r = sqrt(x*x+y*y);

          Point2D<int> pos(int(nfreq*r/rad_max),
                      int(nori*th/theta_max));

          result[pos] += in[Point2D<int>(x,y)];
          ++counts[pos];

          if (result.coordsOk(pos+left))
            { result[pos+left] += in[Point2D<int>(x,y)]; ++counts[pos+left]; }
          if (result.coordsOk(pos+right))
            { result[pos+right] += in[Point2D<int>(x,y)]; ++counts[pos+right]; }
          if (result.coordsOk(pos+up))
            { result[pos+up] += in[Point2D<int>(x,y)]; ++counts[pos+up]; }
          if (result.coordsOk(pos+down))
            { result[pos+down] += in[Point2D<int>(x,y)]; ++counts[pos+down]; }
        }
    else
      for (int x = 0; x < in.getWidth(); ++x)
        {
          int y2 = h - y;

          const double th = atan2(y2, -x);
          const double r = sqrt(x*x+y2*y2);

          Point2D<int> pos(int(nfreq*r/rad_max),
                      int(nori*th/theta_max));

          result[pos] += in[Point2D<int>(x,y)];
          ++counts[pos];

          if (result.coordsOk(pos+left))
            { result[pos+left] += in[Point2D<int>(x,y)]; ++counts[pos+left]; }
          if (result.coordsOk(pos+right))
            { result[pos+right] += in[Point2D<int>(x,y)]; ++counts[pos+right]; }
          if (result.coordsOk(pos+up))
            { result[pos+up] += in[Point2D<int>(x,y)]; ++counts[pos+up]; }
          if (result.coordsOk(pos+down))
            { result[pos+down] += in[Point2D<int>(x,y)]; ++counts[pos+down]; }
        }

  for (int i = 0; i < result.getSize(); ++i)
    if (counts[i] > 0)
      result[i] /= counts[i];

  return result;
}

#if 0
template class FourierEngine<float>;
template class FourierInvEngine<float>;
#endif

template class FourierEngine<double>;
template class FourierInvEngine<double>;

template Image<float> magnitude(const Image<std::complex<float> >&);
template Image<float> logmagnitude(const Image<std::complex<float> >&);
template Image<float> phase(const Image<std::complex<float> >&);
template Image<float> cartesian(const Image<float>&, const Dims&);

template Image<double> magnitude(const Image<std::complex<double> >&);
template Image<double> logmagnitude(const Image<std::complex<double> >&);
template Image<double> phase(const Image<std::complex<double> >&);
template Image<double> cartesian(const Image<double>&, const Dims&);

// ##############################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_FOURIERENGINE_C_DEFINED

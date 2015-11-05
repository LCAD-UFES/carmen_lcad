/*!@file Learn/BackpropNetwork.C Backpropagation neural network with one hidden layer */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Learn/BackpropNetwork.C $
// $Id: BackpropNetwork.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef LEARN_BACKPROPNETWORK_C_DEFINED
#define LEARN_BACKPROPNETWORK_C_DEFINED

#include "Learn/BackpropNetwork.H"

#include "Image/CutPaste.H"
#include "Image/MathOps.H"
#include "Image/MatrixOps.H"
#include "Util/CpuTimer.H"
#include "Util/sformat.H"
#include "rutz/rand.h"
#include "rutz/trace.h"

#include <limits>
#include <sys/types.h>
#include <unistd.h>


namespace
{
#if 1 // use the "standard" logistic sigmoid function

  // dst = 1/(1+exp(-src))
  void inplaceBackpropSigmoid(Image<float>& dst)
  {
  GVX_TRACE(__PRETTY_FUNCTION__);

    Image<float>::iterator dptr = dst.beginw(), dstop = dst.endw();

    while (dptr != dstop)
      {
        *dptr = 1.0f / (1.0f + exp(-(*dptr)));
        ++dptr;
      }
  }

  inline float backpropSigmoidDerivf(const float src)
  {
    return src * (1.0f-src);
  }

#else // use a cheaper-to-compute sigmoid function

  // dst =  1 - 1/(1+x) if x>=0
  // dst = -1 + 1/(1-x) if x<0
  void inplaceBackpropSigmoid(Image<float>& dst)
  {
  GVX_TRACE(__PRETTY_FUNCTION__);

    Image<float>::iterator dptr = dst.beginw(), dstop = dst.endw();

    while (dptr != dstop)
      {
        if (*dptr >= 0.0f)
          *dptr = 1.0f - 1.0f/(1.0f + (*dptr));
        else
          *dptr = -1.0f + 1.0f/(1.0f - (*dptr));

        *dptr = (*dptr + 1.0f) * 0.5f;

        ++dptr;
      }
  }

  inline float backpropSigmoidDerivf(const float src)
  {
    const float rawsrc = (src * 2.0f) - 1.0f;

    if (rawsrc >= 0.0)
      return (rawsptr - 1.0f) * (rawsptr - 1.0f);
    else
      return (rawsptr + 1.0f) * (rawsptr + 1.0f);
  }
#endif

  // some extra Image operators that we need here; FIXME these should
  // move into Image.H eventually

  template <class T>
  Image<T> operator*(T numer, const Image<T>& denom)
  {
    Image<T> result(denom.getDims(), NO_INIT);

    typename Image<T>::iterator dptr = result.beginw();
    typename Image<T>::iterator dstop = result.endw();
    typename Image<T>::const_iterator sptr = denom.begin();

    while (dptr != dstop)
      *dptr++ = numer * (*sptr++);

    return result;
  }

  template <class T>
  Image<T> operator/(T numer, const Image<T>& denom)
  {
    Image<T> result(denom.getDims(), NO_INIT);

    typename Image<T>::iterator dptr = result.beginw();
    typename Image<T>::iterator dstop = result.endw();
    typename Image<T>::const_iterator sptr = denom.begin();

    while (dptr != dstop)
      *dptr++ = numer / (*sptr++);

    return result;
  }

  template <class T>
  Image<T> operator+(T numer, const Image<T>& denom)
  {
    Image<T> result(denom.getDims(), NO_INIT);

    typename Image<T>::iterator dptr = result.beginw();
    typename Image<T>::iterator dstop = result.endw();
    typename Image<T>::const_iterator sptr = denom.begin();

    while (dptr != dstop)
      *dptr++ = numer + (*sptr++);

    return result;
  }

  template <class T>
  Image<T> operator-(T numer, const Image<T>& denom)
  {
    Image<T> result(denom.getDims(), NO_INIT);

    typename Image<T>::iterator dptr = result.beginw();
    typename Image<T>::iterator dstop = result.endw();
    typename Image<T>::const_iterator sptr = denom.begin();

    while (dptr != dstop)
      *dptr++ = numer - (*sptr++);

    return result;
  }
}

void BackpropNetwork::train(const Image<float>& X,
                            const Image<float>& D,
                            const int h, // number of hidden units
                            const float eta,
                            const float alph,
                            const int iters,
                            double* Efinal,
                            double* Cfinal)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  const int n = X.getHeight(); // number of dimensions per data sample
  const int N = X.getWidth(); // number of data samples
  const int m = D.getHeight(); // number of output units

  LINFO("%d samples, %d input, %d hidden units, %d output units",
        N, n, h, m);

  ASSERT(D.getWidth() == X.getWidth());

  // evaluate the network at a few random initial locations, and keep
  // the best one as our starting weight matrix for backprop
  {
    double bestE = std::numeric_limits<double>::max();
    Image<float> bestW, bestV;

    for (int r = 0; r < 10; ++r)
      {
        // initialize weight matrices with random weights
        rutz::urand_frange g(-1.0, 1.0, time((time_t*)0)+getpid());
        this->W = Image<float>(n+1, h, NO_INIT); g = fill(this->W, g);
        this->V = Image<float>(h+1, m, NO_INIT); g = fill(this->V, g);

        const Image<float> Y = this->compute(X);
        double E = RMSerr(Y, D);

        if (E < bestE)
          {
            LINFO("new best E=%f at init iteration %d", E, r);
            bestE = E;
            bestW = this->W;
            bestV = this->V;
          }
      }

    this->W = bestW;
    this->V = bestV;
  }

  Image<float> Xp; // input matrix with row of bias inputs
  Image<float> Z;  // hidden layer output
  Image<float> Zp; // hidden layer output with row of bias inputs
  Image<float> Y;  // network output, should match D
  Image<float> eY;
  Image<float> eZ;
  Image<float> dEdV;
  Image<float> dEdW;
  Image<float> delW;
  Image<float> delV;
  Image<float> delWprev;
  Image<float> delVprev;

  double E = 1.0; // rms error between Y and D
  double C = 0.0; // corrcoef between Y and D

  {GVX_TRACE("generate-Xp");
  Xp = Image<float>(X.getWidth(), X.getHeight() + 1, NO_INIT);
  Xp.clear(-1.0f);
  inplacePaste(Xp, X, Point2D<int>(0,1));
  }

  CpuTimer t;
  CpuTimer t2;

  for (int i = 0; i < iters; ++i)
    {
      GVX_TRACE("backprop loop");

      {GVX_TRACE("compute-Z");
      Z = matrixMult(this->W, Xp);
      inplaceBackpropSigmoid(Z);
      }

      {GVX_TRACE("generate-Zp");
      if (!Zp.initialized())
        {
          Zp = Image<float>(Z.getWidth(), Z.getHeight() + 1, NO_INIT);
          Zp.clear(-1.0f);
        }
      inplacePaste(Zp, Z, Point2D<int>(0,1));
      }

      {GVX_TRACE("compute-Y");
      Y = matrixMult(this->V, Zp);
      inplaceBackpropSigmoid(Y);
      }

      ASSERT(Y.getDims() == D.getDims());

      E = RMSerr(Y, D);
#if 0
      const float materr =
        (0.5 * E * E * double(Y.getSize()) / N);
#endif
      C = corrcoef(Y, D);

      {GVX_TRACE("compute-eY");
      eY.resize(Y.getDims());
      const int sz = eY.getSize();
      Image<float>::iterator const eYptr = eY.beginw();
      Image<float>::const_iterator const Yptr = Y.begin();
      Image<float>::const_iterator const Dptr = D.begin();

      for (int k = 0; k < sz; ++k)
        eYptr[k] = (Yptr[k] - Dptr[k]) * backpropSigmoidDerivf(Yptr[k]);
      }

      {GVX_TRACE("compute-eZ");
      const Image<float> eY_V = transpose(matrixMult(transpose(eY), V));
      eZ.resize(Zp.getDims());
      const int sz = eZ.getSize();
      Image<float>::iterator const eZptr = eZ.beginw();
      Image<float>::const_iterator const eY_Vptr = eY_V.begin();
      Image<float>::const_iterator const Zpptr = Zp.begin();

      for (int k = 0; k < sz; ++k)
        eZptr[k] = eY_Vptr[k] * backpropSigmoidDerivf(Zpptr[k]);
      }

      {GVX_TRACE("compute-dEdV");
      dEdV = matrixMult(eY, transpose(Zp));
      }
      {GVX_TRACE("compute-dEdW");
      dEdW = matrixMult(eZ, transpose(Xp));
      }

      {GVX_TRACE("compute-delW");
      delW = (-eta) * crop(dEdW, Point2D<int>(0,1), Dims(n+1, h));
      if (delWprev.initialized())
        delW += alph * delWprev;
      delWprev = delW;
      }

      {GVX_TRACE("compute-delV");
      delV = (-eta) * dEdV;
      if (delVprev.initialized())
        delV += alph * delVprev;
      delVprev = delV;
      }

      this->V += delV;
      this->W += delW;

      t2.mark();
      if (t2.real_secs() > 0.5)
        {
          t2.reset();
          t.mark();
          t.report(sformat("iteration %d, E=%f, C=%f", i, E, C).c_str());
        }
    }

  if (Efinal != 0)
    *Efinal = E;
  if (Cfinal != 0)
    *Cfinal = C;
}

Image<float> BackpropNetwork::compute(const Image<float>& X) const
{
GVX_TRACE(__PRETTY_FUNCTION__);

  Image<float> Xp(X.getWidth(), X.getHeight() + 1, NO_INIT);
  Xp.clear(-1.0f);
  inplacePaste(Xp, X, Point2D<int>(0,1));

  Image<float> Z = matrixMult(this->W, Xp);
  inplaceBackpropSigmoid(Z);

  Image<float> Zp(Z.getWidth(), Z.getHeight() + 1, NO_INIT);
  Zp.clear(-1.0f);
  inplacePaste(Zp, Z, Point2D<int>(0,1));

  Image<float> Y = matrixMult(this->V, Zp);
  inplaceBackpropSigmoid(Y);

  ASSERT(Y.getWidth() == X.getWidth());

  return Y;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // LEARN_BACKPROPNETWORK_C_DEFINED

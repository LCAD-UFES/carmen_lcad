/*!@file Image/LinearAlgebra.C */

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
// Primary maintainer for this file: Rob Peters <rjpeters@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/LinearAlgebra.C $
// $Id: LinearAlgebra.C 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef IMAGE_LINEARALGEBRA_C_DEFINED
#define IMAGE_LINEARALGEBRA_C_DEFINED

#include "Image/LinearAlgebra.H"

#include "Image/Image.H"
#include "Image/MatrixOps.H"
#include "Image/gsl.H"
#include "Image/lapack.H"
#include "rutz/trace.h"

namespace
{
  // ############################################################
  template <class T>
  Image<T> invdiag(const Image<T>& x,
                   const T tolfactor,
                   int* const rank)
  {
    ASSERT(x.getWidth() == x.getHeight());

    Image<T> result(x.getDims(), ZEROS);
    if (rank != NULL) *rank = 0;

    const T thresh = tolfactor*x[0];

    const T eigtot = trace(x);
    T eigsum = 0.0;
    T eigkept = 0.0;

    for (int i = 0; i < x.getWidth(); ++i)
      {
        const T xval = x[Point2D<int>(i,i)];

        eigsum += xval;

        // if tolfactor is less than 1.0, then we take it to mean a
        // threshold at factor*eigenvalue0; if it is greater than 1.0,
        // then we take it to mean a count of the number of
        // eigenvectors to keep

        const bool discard =
          tolfactor < 1.0
          ? (xval < thresh)
          : (i >= int(tolfactor));

        LINFO("x[%d]=%g %s %5.2f%% (thresh=%g)",
              i, double(xval),
              discard ? "discarded" : "kept",
              eigtot > 0.0 ? double(100.0 * eigsum) / eigtot : 0.0,
              double(thresh));

        if (discard)
          result[Point2D<int>(i,i)] = 0.0;
        else
          {
            result[Point2D<int>(i,i)] = 1.0 / xval;
            if (rank != NULL) ++*rank;
            eigkept = eigsum;
          }
      }

    LINFO("retained eigenvalues account for %5.2f%% of total",
          eigtot > 0.0 ? double(100.0 * eigkept) / eigtot : 0.0);
    return result;
  }

} // end unnamed namespace

// ############################################################
void svd(const Image<double>& A,
         Image<double>& U, Image<double>& S, Image<double>& V,
         SvdAlgo algo, SvdFlag flags)
{
GVX_TRACE(__PRETTY_FUNCTION__);

#ifndef HAVE_LAPACK
  if (algo == SVD_LAPACK)
    {
      LINFO("SVD_LAPACK not available, using SVD_GSL instead");
      algo = SVD_GSL;
      flags |= SVD_TALL;
    }
#endif

  switch (algo)
    {
#ifdef HAVE_LAPACK
    case SVD_LAPACK:
      lapack::svd(A, U, S, V, flags);
      break;
#endif

    case SVD_GSL:
      gsl::svd(A, U, S, V, flags);
      break;

    default:
      LFATAL("unknown SvdAlgo enumerant '%d'", int(algo));
    }
}

// ############################################################
void svdf(const Image<float>& A,
          Image<float>& U, Image<float>& S, Image<float>& V,
          SvdAlgo algo, SvdFlag flags)
{
GVX_TRACE(__PRETTY_FUNCTION__);

#ifndef HAVE_LAPACK
  if (algo == SVD_LAPACK)
    {
      LINFO("SVD_LAPACK not available, using SVD_GSL instead");
      algo = SVD_GSL;
      flags |= SVD_TALL;
    }
#endif

  switch (algo)
    {
#ifdef HAVE_LAPACK
    case SVD_LAPACK:
      lapack::svdf(A, U, S, V, flags);
      break;
#endif

    case SVD_GSL:
      gsl::svdf(A, U, S, V, flags);
      break;

    default:
      LFATAL("unknown SvdAlgo enumerant '%d'", int(algo));
    }
}

// ############################################################
Image<double> svdPseudoInv(const Image<double>& x,
                           const SvdAlgo algo,
                           int* const rank,
                           const double tolfactor)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  Image<double> U, S, V;
  svd(x, U, S, V, algo);

  LINFO("U is %dx%d", U.getHeight(), U.getWidth());
  LINFO("S is %dx%d", S.getHeight(), S.getWidth());
  LINFO("V is %dx%d", V.getHeight(), V.getWidth());

  // FIXME this could probably be made significantly more efficient,
  // by taking advantage of the fact that S is a diagonal matrix, and
  // by rolling transpose(U) directly into the matrix multiplication
  return matrixMult(V, matrixMult(invdiag(S, tolfactor, rank),
                                  transpose(U)));
}

// ############################################################
Image<float> svdPseudoInvf(const Image<float>& x,
                           const SvdAlgo algo,
                           int* const rank,
                           const float tolfactor)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  Image<float> U, S, V;
  svdf(x, U, S, V, algo);

  LINFO("U is %dx%d", U.getHeight(), U.getWidth());
  LINFO("S is %dx%d", S.getHeight(), S.getWidth());
  LINFO("V is %dx%d", V.getHeight(), V.getWidth());

  return matrixMult(V, matrixMult(invdiag(S, tolfactor, rank),
                                  transpose(U)));
}

// ############################################################
template <class T>
Image<typename promote_trait<T, float>::TP>
naiveUnstablePseudoInv(const Image<T>& M)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  /*
    pseudoinverse(M) = inv(M'*M) * M'

    where M' is the transpose of M, and '*' is matrix
    multiplication

    see comments in the header file about why this approach is
    numerically unstable, though
  */

  typedef typename promote_trait<T, float>::TP TF;

  const Image<TF> m(M);
  const Image<TF> mt(transpose(M));
  const Image<TF> mtm = matrixMult(mt, m);
  const Image<TF> invmtm = matrixInv(mtm);

  return matrixMult(invmtm, mt);
}

// Include the explicit instantiations
#include "inst/Image/LinearAlgebra.I"

template Image<promote_trait<double, float>::TP> naiveUnstablePseudoInv(const Image<double>&);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_LINEARALGEBRA_C_DEFINED

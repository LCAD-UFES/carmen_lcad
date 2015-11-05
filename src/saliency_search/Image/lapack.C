/*!@file Image/lapack.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/lapack.C $
// $Id: lapack.C 14627 2011-03-23 16:29:28Z lior $
//

#ifndef IMAGE_LAPACK_C_DEFINED
#define IMAGE_LAPACK_C_DEFINED

#ifdef HAVE_LAPACK

#include "Image/lapack.H"

#include "Image/Image.H"
#include "Image/LinearAlgebraFlags.H"
#include "Image/MatrixOps.H" // for transpose()
#include "Image/f77lapack.H" // for dgesdd_()
#include "Util/log.H"
#include "rutz/trace.h"
#include <stdio.h>

namespace
{
  // ############################################################
  /** Compute the Singular Value Decomposition.

     Compute the singular value decomposition (SVD) of a complex M-by-N
     matrix A, also computing the left and right singular vectors, by
     using a divide-and-conquer method.

     In lapack this is dgesdd. dgesdd computes the singular value
     decomposition (SVD) of a real M-by-N matrix A, optionally computing
     the left and/or right singular vectors, by using divide-and-conquer
     method. The SVD is written

     \f[A = U \cdot Sigma \cdot V^T\f]

     where Sigma is an M-by-N matrix which is zero except for its \c
     min(m,n) diagonal elements, U is an M-by-M orthogonal matrix, and V
     is an N-by-N orthogonal matrix.  The diagonal elements of SIGMA are
     the singular values of A; they are real and non- negative, and are
     returned in descending order.  The first \c min(m,n) columns of U
     and V are the left and right singular vectors of A.

     Note that the routine returns VT = V**T (transposed), not V.

     Now watch out: This routine has several modes of operation,
     depending on the size of the input matrices \c U and \c VT. This is:

     - If \c U is M-by-M and \c VT is N-by-N (the normal mode), then \e
     all left and right singular vectors are calculated and are returned
     in \c U and \c VT.

     - If \c U is M-by-min(M,N) and \c VT is min(M,N)-by-N, then the
     first min(M,N) left and right singular vectors are calculated,
     respectively, and are returned in \c U and \c VT. FIXME: needs verification.

     - If M >= N, \c U is of size 0, and \c VT is N-by-N, then the first
     N left singular vectors are calculated and returned in the first
     columns of \c A, and all right singular vectors are calculated and
     returned in \c VT. In this mode, \c U is unused. FIXME: needs verification.

     - If M < N, \c U is M-by-M, and \c VT is of size 0, then all left
     singular vectors are calculated and returned in \c U, and the first
     M right singular vectors are calculated and returned in the first M
     rows of \c A. In this mode, \c VT is unused. FIXME: needs verification.

     In any other combination of matrix sizes, an exception is thrown.

     @param A The M-by-N input matrix to be decomposed. It will be
     destroyed during the computation.

     @param Sigma A real-valued vector of length \c min(M,N) that will
     return the singular values. WATCH OUT: The length has to be \e
     exactly \c min(M,N) or else an exception will be thrown.

     @param U In the normal mode of calculation, the M-by-M matrix of
     the left singular vectors. In other modes this might be unused.

     @param VT In the normal mode of calculation, the N-by-N matrix of
     the right singular vectors. In other modes this might be unused.

     NOTE: This wrapper code is based heavily on code from the C++
     library "lapackpp" (http://www.sourceforge.net/projects/lapackpp)
     by Christian Stimming <stimming at tuhh dot de>, also licensed
     under the GPL. Specifically, this function is based on code from
     the source file "lapackpp/src/lasvd.cc", with the main changes
     being that the code is adapted to use our Image class rather than
     the LaGenMatDouble of lapackpp, and the preceding doc comment is
     taken verbatim from "lapackpp/include/lasvd.h".
   */
  void svd_lapack(Image<double>& A, Image<double>& Sigma,
                  Image<double>& U, Image<double>& VT)
  {
  GVX_TRACE(__PRETTY_FUNCTION__);

    char jobz = '?';
    f77_integer info = 0;
    int M = A.getWidth();
    int N = A.getHeight();
    int MNmin = std::min(M,N);
    f77_integer Ml = M;
    f77_integer Nl = N;
    f77_integer lda = A.getWidth();

    if (Sigma.getSize() != MNmin)
      LFATAL("Sigma is not of correct size");

    if ((U.getWidth() == M && U.getHeight() == M)
        && (VT.getWidth() == N && VT.getHeight() == N))
      jobz = 'A';
    else if ((U.getWidth() == M && U.getHeight() == MNmin)
             && (VT.getWidth() == MNmin && VT.getHeight() == N))
      jobz = 'S';
    else if (M >= N
             && U.getWidth() == 0
             && (VT.getWidth() == N && VT.getHeight() == N))
      jobz = 'O';
    else if (M < N
             && (U.getWidth() == M && U.getHeight() == M)
             && VT.getWidth() == 0)
      jobz = 'O';
    else
      LFATAL("U or VT is not of correct size");

    f77_integer ldu = U.getWidth();
    f77_integer ldvt = VT.getWidth();

    int liwork = 8*MNmin;
    Image<f77_integer> iwork(liwork, 1, NO_INIT);

    f77_integer lwork = -1;
    Image<double> work(1, 1, ZEROS);
    // Calculate the optimum temporary workspace
    dgesdd_(&jobz, &Ml, &Nl, A.getArrayPtr(), &lda,
            Sigma.getArrayPtr(), U.getArrayPtr(), &ldu,
            VT.getArrayPtr(), &ldvt,
            work.getArrayPtr(), &lwork, iwork.getArrayPtr(),
            &info);
    lwork = int(work[0]);
    work.resize(lwork, 1);

    // Now the real calculation
    dgesdd_(&jobz, &Ml, &Nl, A.getArrayPtr(), &lda,
            Sigma.getArrayPtr(), U.getArrayPtr(), &ldu,
            VT.getArrayPtr(), &ldvt,
            work.getArrayPtr(), &lwork, iwork.getArrayPtr(),
            &info);

    if (info != 0)
      LFATAL("Internal error in LAPACK: dgesdd() (info=%ld)", info);
  }

  //! Single-precision counterpart of svd_lapack()
  void svdf_lapack(Image<float>& A, Image<float>& Sigma,
                   Image<float>& U, Image<float>& VT)
  {
  GVX_TRACE(__PRETTY_FUNCTION__);

    char jobz = '?';
    f77_integer info = 0;
    int M = A.getWidth();
    int N = A.getHeight();
    int MNmin = std::min(M,N);
    f77_integer Ml = M;
    f77_integer Nl = N;
    f77_integer lda = A.getWidth();

    if (Sigma.getSize() != MNmin)
      LFATAL("Sigma is not of correct size");

    if ((U.getWidth() == M && U.getHeight() == M)
        && (VT.getWidth() == N && VT.getHeight() == N))
      jobz = 'A';
    else if ((U.getWidth() == M && U.getHeight() == MNmin)
             && (VT.getWidth() == MNmin && VT.getHeight() == N))
      jobz = 'S';
    else if (M >= N
             && U.getWidth() == 0
             && (VT.getWidth() == N && VT.getHeight() == N))
      jobz = 'O';
    else if (M < N
             && (U.getWidth() == M && U.getHeight() == M)
             && VT.getWidth() == 0)
      jobz = 'O';
    else
      LFATAL("U or VT is not of correct size");

    f77_integer ldu = U.getWidth();
    f77_integer ldvt = VT.getWidth();

    int liwork = 8*MNmin;
    Image<f77_integer> iwork(liwork, 1, NO_INIT);

    f77_integer lwork = -1;
    Image<float> work(1, 1, ZEROS);
    // Calculate the optimum temporary workspace
    sgesdd_(&jobz, &Ml, &Nl, A.getArrayPtr(), &lda,
            Sigma.getArrayPtr(), U.getArrayPtr(), &ldu,
            VT.getArrayPtr(), &ldvt,
            work.getArrayPtr(), &lwork, iwork.getArrayPtr(),
            &info);
    lwork = int(work[0]);
    work.resize(lwork, 1);

    // Now the real calculation
    sgesdd_(&jobz, &Ml, &Nl, A.getArrayPtr(), &lda,
            Sigma.getArrayPtr(), U.getArrayPtr(), &ldu,
            VT.getArrayPtr(), &ldvt,
            work.getArrayPtr(), &lwork, iwork.getArrayPtr(),
            &info);

    if (info != 0)
      LFATAL("Internal error in LAPACK: sgesdd() (info=%ld)", info);
  }

} // end unnamed namespace

// ######################################################################
void lapack::svd(const Image<double>& A,
                 Image<double>& U, Image<double>& S, Image<double>& V,
                 const SvdFlag flags)
{
  const int N = A.getWidth();
  const int M = A.getHeight();

  // we require M >= N
  if (M < N)
    LFATAL("expected M >= N, got M=%d and N=%d", M, N);

  Image<double> MMtxp = transpose(A);
  Image<double> SS(N, 1, ZEROS);
  Image<double> UUtxp(M,
                      (flags & SVD_FULL) ? M : N,
                      ZEROS);
  Image<double> VV(N, N, ZEROS);

  // lapack does everything in column-major format, but our
  // Image class is in row-major format, so we have to pass
  // tranposed matrices to lapack and the un-transpose the
  // result matrices
  svd_lapack(MMtxp, SS, UUtxp, VV);

  U = transpose(UUtxp);
  V = VV;
  S = Image<double>(N,
                    (flags & SVD_FULL) ? M : N,
                    ZEROS);
  for (int c = 0; c < N; ++c)
    S[Point2D<int>(c, c)] = SS[c];
}

// ######################################################################
void lapack::svdf(const Image<float>& A,
                  Image<float>& U, Image<float>& S, Image<float>& V,
                  const SvdFlag flags)
{
  const int N = A.getWidth();
  const int M = A.getHeight();

  // we require M >= N
  if (M < N)
    LFATAL("expected M >= N, got M=%d and N=%d", M, N);

  Image<float> MMtxp = transpose(A);
  Image<float> SS(N, 1, ZEROS);
  Image<float> UUtxp(M,
                     (flags & SVD_FULL) ? M : N,
                     ZEROS);
  Image<float> VV(N, N, ZEROS);

  // lapack does everything in column-major format, but our
  // Image class is in row-major format, so we have to pass
  // tranposed matrices to lapack and the un-transpose the
  // result matrices
  svdf_lapack(MMtxp, SS, UUtxp, VV);

  U = transpose(UUtxp);
  V = VV;
  S = Image<float>(N,
                   (flags & SVD_FULL) ? M : N,
                   ZEROS);
  for (int c = 0; c < N; ++c)
    S[Point2D<int>(c, c)] = SS[c];
}

// #################################################################
Image<double> lapack::dgemv(const Image<double>* v,
                            const Image<double>* Mat)
{
  GVX_TRACE(__PRETTY_FUNCTION__);

  const int wv = v->getWidth(), hv = v->getHeight();
  const int wm = Mat->getWidth(), hm = Mat->getHeight();

  ASSERT(wv == hm);
  ASSERT(hv == 1);

  Image<double> y(wm, hv /* == 1*/, NO_INIT);

  /* As with the matrix-matrix math (see lapack::dgemm() below),
     we're dealing with row-major vs. column-major issues here. The
     lapack function dgemv() actually wants to do M*v, with a
     column-major matrix right-multiplied by a column vector,
     returning the result in another column vector. But we actually
     want to do v*M, with a row-major matrix left-multiplied by a
     row vector, returning the result in a row vector. Those two are
     actually equivalent operations.
  */

  char trans = 'N';
  f77_integer M = Mat->getWidth(),
    N = Mat->getHeight(), lda = Mat->getWidth(),
    incv = 1, incy = 1;

  double alpha = 1.0, beta = 0.0;

  assert(Mat->getWidth() == y.getSize());
  assert(Mat->getHeight() == v->getSize());

  {GVX_TRACE("dgemv");
  dgemv_(&trans, &M, &N, &alpha,
         Mat->getArrayPtr(), &lda,
         v->getArrayPtr(), &incv,
         &beta, y.getArrayPtr(), &incy);
  }

  return y;
}

// #################################################################
Image<float> lapack::sgemv(const Image<float>* v,
                           const Image<float>* Mat)
{
  GVX_TRACE(__PRETTY_FUNCTION__);

  const int wv = v->getWidth(), hv = v->getHeight();
  const int wm = Mat->getWidth(), hm = Mat->getHeight();

  ASSERT(wv == hm);
  ASSERT(hv == 1);

  Image<float> y(wm, hv /* == 1*/, NO_INIT);

  // see comments in lapack::dgemv() above

  char trans = 'N';
  f77_integer M = Mat->getWidth(),
    N = Mat->getHeight(), lda = Mat->getWidth(),
    incv = 1, incy = 1;

  float alpha = 1.0, beta = 0.0;

  assert(Mat->getWidth() == y.getSize());
  assert(Mat->getHeight() == v->getSize());

  {GVX_TRACE("sgemv");
  sgemv_(&trans, &M, &N, &alpha,
         Mat->getArrayPtr(), &lda,
         v->getArrayPtr(), &incv,
         &beta, y.getArrayPtr(), &incy);
  }

  return y;
}

// #################################################################
Image<double> lapack::dgemm(const Image<double>* A,
                            const Image<double>* B)
{
  GVX_TRACE(__PRETTY_FUNCTION__);

  /* This is slightly tricky to get the conversions between row-major
     and column-major just right. dgemm() expects to multiply two
     column-major matrices, giving C=A*B. But what we actually have
     are two row-major matrices. Or, if we swap getWidth() with
     getHeight(), then what we have are two transposed column-major
     matrices. So as far as lapack is concerned, the layout of our
     input matrices means we have A^T and B^T. Then, if we swap the
     order in which the matrices are passed to lapack we can have it
     compute (B^T * A^T), which is just (A*B)^T. So what lapack is
     computing for us is the column-major transpose C^T, but that is
     exactly the UN-transposed row-major matrix C that we want.

     (The tricky part for me was realizing that I needed no explicit
     transpose() calls, nor did I need to pass any 'T' arguments to
     tell lapack to do any transposition).
  */

  Image<double> C(B->getWidth(), A->getHeight(), NO_INIT);

  char t = 'N'; // 'N' means "don't transpose", 'T' means "do transpose"

  f77_integer m = B->getWidth(), k = B->getHeight(), n = A->getHeight();
  f77_integer lda = B->getWidth(), ldb = A->getWidth(), ldc = C.getWidth();
  assert(B->getWidth() == C.getWidth());
  assert(A->getHeight() == C.getHeight());
  assert(B->getHeight() == A->getWidth());

  double alpha = 1.0, beta = 0.0;

  {GVX_TRACE("dgemm");
  dgemm_(&t, &t, &m, &n, &k, &alpha,
         B->getArrayPtr(), &lda,
         A->getArrayPtr(), &ldb,
         &beta, C.getArrayPtr(), &ldc);
  }

  return C;
}

// #################################################################
Image<float> lapack::sgemm(const Image<float>* A,
                           const Image<float>* B)
{
  GVX_TRACE(__PRETTY_FUNCTION__);

  // see comments in lapack::dgemm() above

  Image<float> C(B->getWidth(), A->getHeight(), NO_INIT);

  char t = 'N'; // 'N' means "don't transpose", 'T' means "do transpose"

  f77_integer m = B->getWidth(), k = B->getHeight(), n = A->getHeight();
  f77_integer lda = B->getWidth(), ldb = A->getWidth(), ldc = C.getWidth();
  assert(B->getWidth() == C.getWidth());
  assert(A->getHeight() == C.getHeight());
  assert(B->getHeight() == A->getWidth());

  float alpha = 1.0, beta = 0.0;

  {GVX_TRACE("sgemm");
  sgemm_(&t, &t, &m, &n, &k, &alpha,
         B->getArrayPtr(), &lda,
         A->getArrayPtr(), &ldb,
         &beta, C.getArrayPtr(), &ldc);
  }

  return C;
}

// #################################################################
Image<double> lapack::dpotrf(const Image<double>* Mat)
{

  GVX_TRACE(__PRETTY_FUNCTION__);


  Image<double> A = *Mat;

  f77_integer order =  Mat->getWidth(), lda = Mat->getWidth(),
              flags = 0;

  {GVX_TRACE("dpotrf");

  char uplo = 'L';

  dpotrf_(&uplo, &order,
         A.getArrayPtr(), &lda, &flags);
  }

  return A;
}

double lapack::det(const Image<double>* Mat)
{

  GVX_TRACE(__PRETTY_FUNCTION__);

  //Derived from http://vismod.media.mit.edu/pub/tpminka/MRSAR/lapack.c

  Image<double> A = *Mat;

  f77_integer m =  Mat->getHeight(), n =  Mat->getWidth(), lda = Mat->getWidth(), info = 0;
  f77_integer ipvt[std::min(m,n)];

  {
    GVX_TRACE("dgetrf");

    dgetrf_(&m,&n, A.getArrayPtr(), &lda, ipvt, &info);
    if (info > 0)
    {
      LINFO("ERROR: singuler matrix");
      return -1.0;
    }
  }

  //Take the product of the diagonal elements
  double det = 1.0;
  bool neg = false;
  for(int i=0; i<m; i++)
  {
    det *= A.getVal(i,i);
    if (ipvt[i] != (i+1))
      neg = !neg;
  }

  /* Since A is an LU decomposition of a rowwise permutation of A,
     multiply by appropriate sign */
  return neg ? -det : det;
}


#endif // HAVE_LAPACK

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_LAPACK_C_DEFINED

/*!@file Image/f77lapack.H forward declarations of fortran-77 functions from lapack */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/f77lapack.H $
// $Id: f77lapack.H 14618 2011-03-18 22:49:35Z lior $
//

#ifndef IMAGE_F77LAPACK_H_DEFINED
#define IMAGE_F77LAPACK_H_DEFINED

#if 0
// This is a fake include that is here so that depoptions.in can
// trigger liblapack to be linked in -- we need a "fake" include
// because it seems that std distros of lapack don't actually include
// a C header file
#include <link-lapack>
#endif

// Forward declarations of functions that we expect should be defined
// in liblapack

#ifdef HAVE_LAPACK

typedef long int f77_integer;

// ###################################################################
// ###################################################################
/*! @name BLAS level 2 functions (matrix-vector operations) */
//@{

//! Double-precision GEneral Matrix Vector multiply (DGEMV)
/*! See http://www.netlib.org/blas/dgemv.f */
extern "C" void dgemv_(char* trans, f77_integer* M, f77_integer* N,
                       double* alpha, const double* A, f77_integer* lda,
                       const double* dx, f77_integer* incx,
                       double* beta, double* dy, f77_integer* incy);

//! Single-precision GEneral Matrix Vector multiply (SGEMV)
/*! See http://www.netlib.org/blas/sgemv.f */
extern "C" void sgemv_(char* trans, f77_integer* M, f77_integer* N,
                       float* alpha, const float* A, f77_integer* lda,
                       const float* dx, f77_integer* incx,
                       float* beta, float* dy, f77_integer* incy);

//@}

// ###################################################################
// ###################################################################
/*! @name BLAS level 3 functions (matrix-matrix operations) */
//@{

//! Double-precision GEneral Matrix Matrix multiply (DGEMM)
/*! See http://www.netlib.org/blas/dgemm.f */
extern "C" void dgemm_(char *transa, char *transb,
                       f77_integer *m, f77_integer *n, f77_integer *k,
                       double *alpha, const double *a, f77_integer *lda,
                       const double *b, f77_integer *ldb,
                       double *beta, double *c, f77_integer *ldc);

//! Single-precision GEneral Matrix Matrix multiply (SGEMM)
/*! See http://www.netlib.org/blas/sgemm.f */
extern "C" void sgemm_(char *transa, char *transb,
                       f77_integer *m, f77_integer *n, f77_integer *k,
                       float *alpha, const float *a, f77_integer *lda,
                       const float *b, f77_integer *ldb,
                       float *beta, float *c, f77_integer *ldc);

//@}

// ###################################################################
// ###################################################################
/*! @name LAPACK functions (linear algebra operations) */
//@{

//! Double-precision GEneral matrix Singular value Decomposition using Divide-and-conquer (DGESDD)
/*! See http://www.netlib.org/lapack/double/dgesdd.f

    Note that this algorithm, based on divide-and-conquer, is expected
    to be substantially faster than the older DGESVD, which is based
    on QR-decomposition (see
    http://www.cs.berkeley.edu/~demmel/DOE2000/Report0100.html).
*/
extern "C" void dgesdd_(char *jobz, f77_integer *m, f77_integer *n,
                        double *a, f77_integer *lda,
                        double *s, double *u, f77_integer *ldu,
                        double *vt, f77_integer *ldvt,
                        double *work, f77_integer *lwork,
                        f77_integer *iwork, f77_integer *info);

//! Single-precision GEneral matrix Singular value Decomposition using Divide-and-conquer (SGESDD)
/*! See http://www.netlib.org/lapack/double/sgesdd.f */
extern "C" void sgesdd_(char *jobz, f77_integer *m, f77_integer *n,
                        float *a, f77_integer *lda,
                        float *s, float *u, f77_integer *ldu,
                        float *vt, f77_integer *ldvt,
                        float *work, f77_integer *lwork,
                        f77_integer *iwork, f77_integer *info);

//@}

//! double-precision Computes the Cholesky factorization of a symmetric
/*! See http://www.netlib.org/lapack/double/sgesdd.f */
extern "C" void dpotrf_(char *UPLO, f77_integer *n,
                        double *a, f77_integer *lda,
                        f77_integer *info);


//! computes an LU factorization of a general M-by-N matrix A using partial pivoting with row interchanges.
/*! See http://www.netlib.org/lapack/double/dgetrf.f */
extern "C" void dgetrf_(f77_integer *m, f77_integer *n,
                        double *a, f77_integer *lda,
                        f77_integer *ipiv, 
                        f77_integer *info);

//@}



#endif // HAVE_LAPACK

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_F77LAPACK_H_DEFINED

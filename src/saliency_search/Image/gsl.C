/*!@file Image/gsl.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/gsl.C $
// $Id: gsl.C 15310 2012-06-01 02:29:24Z itti $
//

/* NOTE: Most of the code here in Image/gsl.C is drawn from
   the GNU Scientific Library (also under the GPL) with little or no
   modification. See http://www.gnu.org/software/gsl/ for information
   about the GSL. Original copyright notice:

   Copyright (C) 1996, 1997, 1998, 1999, 2000 Gerard Jungman, Brian Gough
*/

#ifndef IMAGE_GSL_C_DEFINED
#define IMAGE_GSL_C_DEFINED

#include "Image/gsl.H"

#include "Image/Image.H"
#include "Image/LinearAlgebraFlags.H"
#include "Util/log.H"
#include "rutz/compat_cmath.h" // for isnan()
#include "rutz/shared_ptr.h"
#include "rutz/trace.h"

#include <algorithm>
#include <cstdlib>

namespace
{
  inline int OFFSET(int N, int incX)
  {
    return
      incX > 0
      ? 0
      : (N - 1) * (-incX);
  }

  const double DBL_EPSILON = 2.2204460492503131e-16;
}

// ############################################################
// declarations of types in gsl namespace
namespace gsl
{
  template <class T> class view_of;
  class block;
  class vector;
  class matrix;
}

// ############################################################
// gsl::view_of
template <class T>
struct gsl::view_of
{
  view_of(const T& x) : viewee(x) {}

  view_of(const view_of& that) : viewee(that.viewee) {}

  T viewee;

private:
  // no default constructor allowed
  view_of();

  // no assignment operator allowed, because it would be confusing as
  // to whether the assignment involves assignment of value or of
  // reference
  view_of& operator=(const view_of&);
};

// ############################################################
// gsl::block
struct gsl::block
{
  block(const size_t n)
    :
    size(n),
    data(new double[n])
  {}

  ~block()
  {
    delete [] data;
  }

  size_t size;
  double* data;

private:
  block(const block&);
  block& operator=(const block&);
};

// ############################################################
// gsl::vector
struct gsl::vector
{
  vector() : size(0), stride(0), data(0), block(0), owner(0) {}

  vector(size_t n)
    :
    block()
  {
    if (n == 0)
      {
        LFATAL ("vector length n must be positive integer");
      }

    block.reset( new gsl::block (n) );
    size = n;
    stride = 1;
    owner = 1;
    data = block->data;
  }

  void copy_of(const vector& src)
  {
    const size_t src_size = src.size;
    const size_t this_size = this->size;

    if (src_size != this_size)
      {
        LFATAL("vector lengths are not equal");
      }

    const size_t src_stride = src.stride;
    const size_t this_stride = this->stride;

    for (size_t j = 0; j < src_size; j++)
      {
        this->data[this_stride * j] = src.data[src_stride * j];
      }
  }

  void swap_elements(const size_t i, const size_t j)
  {
    double* const data = this->data;
    const size_t size = this->size;
    const size_t stride = this->stride;

    if (i >= size)
      {
        LFATAL("first index is out of range");
      }

    if (j >= size)
      {
        LFATAL("second index is out of range");
      }

    if (i != j)
      {
        const double tmp = data[j*stride];
        data[j*stride] = data[i*stride];
        data[i*stride] = tmp;
      }
  }

  double at(const size_t i) const
  {
#if GSL_RANGE_CHECK
    if (i >= this->size)
      {
        LFATAL ("index out of range");
      }
#endif
    return this->data[i * this->stride];
  }

  double& at(const size_t i)
  {
#if GSL_RANGE_CHECK
    if (i >= this->size)
      {
        LFATAL ("index out of range");
      }
#endif
    return this->data[i * this->stride];
  }

  double get(const size_t i) const
  {
    return this->at(i);
  }

  void set(const size_t i, const double x)
  {
    this->at(i) = x;
  }

  void set_all (double x)
  {
    double* const data = this->data;
    const size_t n = this->size;
    const size_t stride = this->stride;

    for (size_t i = 0; i < n; ++i)
      {
        data[i * stride] = x;
      }
  }

  view_of<vector> subvector(const size_t offset, const size_t n)
  {
    return subvector_impl(offset, n);
  }

  view_of<const vector> subvector(const size_t offset,
                                  const size_t n) const
  {
    return subvector_impl(offset, n);
  }

  size_t size;
  size_t stride;
  double* data;
  rutz::shared_ptr<gsl::block> block;
  int owner;

private:
  vector subvector_impl(const size_t offset, const size_t n) const
  {
    if (n == 0)
      {
        LFATAL ("vector length n must be positive integer");
      }

    if (offset + (n - 1) >= this->size)
      {
        LFATAL ("view would extend past end of vector");
      }

    vector s;

    s.data = this->data +  this->stride * offset;
    s.size = n;
    s.stride = this->stride;
    s.block = this->block;
    s.owner = 0;

    return s;
  }
};

// ############################################################
// gsl::matrix
struct gsl::matrix
{
  matrix()
    :
    mrows(0), ncols(0), tda(0), data(0), block(), owner()
  {}

  matrix(const size_t n1, const size_t n2)
  {
    this->block.reset( new gsl::block (n1 * n2) );

    this->data = block->data;
    this->mrows = n1;
    this->ncols = n2;
    this->tda = n2;
    this->owner = 1;
  }

  double at(const size_t i, const size_t j) const
  {
#if GSL_RANGE_CHECK
    if (i >= this->mrows)
      {
        LFATAL("first index out of range");
      }
    else if (j >= this->ncols)
      {
        LFATAL("second index out of range");
      }
#endif
    return this->data[i * this->tda + j];
  }

  double& at(const size_t i, const size_t j)
  {
#if GSL_RANGE_CHECK
    if (i >= this->mrows)
      {
        LFATAL("first index out of range");
      }
    else if (j >= this->ncols)
      {
        LFATAL("second index out of range");
      }
#endif
    return this->data[i * this->tda + j];
  }

  double get(const size_t i, const size_t j) const
  {
    return this->at(i,j);
  }

  void set(const size_t i, const size_t j, const double x)
  {
    this->at(i,j) = x;
  }

  size_t mrows;
  size_t ncols;
  size_t tda;
  double* data;
  rutz::shared_ptr<gsl::block> block;
  int owner;
};

// ############################################################
// helper functions for gsl::matrix, gsl::vector, etc.
namespace
{
  // ############################################################
  void set_identity (gsl::matrix& m)
  {
    GVX_TRACE("set_identity");
    double* const data = m.data;
    const size_t p = m.mrows;
    const size_t q = m.ncols;
    const size_t tda = m.tda;

    for (size_t i = 0; i < p; ++i)
      {
        for (size_t j = 0; j < q; ++j)
          {
            data[i * tda + j] = ((i == j) ? 1.0 : 0.0);
          }
      }
  }

  // ############################################################
  void set_all (gsl::matrix& m, const double x)
  {
    GVX_TRACE("set_all");

    double* const data = m.data;
    const size_t p = m.mrows;
    const size_t q = m.ncols;
    const size_t tda = m.tda;

    for (size_t i = 0; i < p; ++i)
      {
        for (size_t j = 0; j < q; ++j)
          {
            data[i * tda + j] = x;
          }
      }
  }

  // ############################################################
  gsl::view_of<gsl::matrix>
  submatrix_of (gsl::matrix& m,
                const size_t i, const size_t j,
                const size_t n1, const size_t n2)
  {
    GVX_TRACE("submatrix_of");

    if (i >= m.mrows)
      {
        LFATAL ("row index is out of range");
      }
    else if (j >= m.ncols)
      {
        LFATAL ("column index is out of range");
      }
    else if (n1 == 0)
      {
        LFATAL ("first dimension must be non-zero");
      }
    else if (n2 == 0)
      {
        LFATAL ("second dimension must be non-zero");
      }
    else if (i + n1 > m.mrows)
      {
        LFATAL ("first dimension overflows matrix");
      }
    else if (j + n2 > m.ncols)
      {
        LFATAL ("second dimension overflows matrix");
      }

    gsl::matrix s;

    s.data = m.data + (i * m.tda + j);
    s.mrows = n1;
    s.ncols = n2;
    s.tda = m.tda;
    s.block = m.block;
    s.owner = 0;

    return gsl::view_of<gsl::matrix>(s);
  }

  // ############################################################
  gsl::view_of<gsl::vector>
  row_of (gsl::matrix& m, const size_t i)
  {
    if (i >= m.mrows)
      {
        LFATAL("row index is out of range");
      }

    gsl::vector v;

    v.data = m.data + i * m.tda;
    v.size = m.ncols;
    v.stride = 1;
    v.block = m.block;
    v.owner = 0;

    return gsl::view_of<gsl::vector>(v);
  }

  // ############################################################
  gsl::view_of<gsl::vector>
  column_of (gsl::matrix& m, const size_t j)
  {
    if (j >= m.ncols)
      {
        LFATAL("column index is out of range");
      }

    gsl::vector v;

    v.data = m.data + j;
    v.size = m.mrows;
    v.stride = m.tda;
    v.block = m.block;
    v.owner = 0;

    return gsl::view_of<gsl::vector>(v);
  }

  // ############################################################
  gsl::view_of<const gsl::vector>
  const_row (const gsl::matrix& m, const size_t i)
  {
    GVX_TRACE("const_row");
    if (i >= m.mrows)
      {
        LFATAL("row index is out of range");
      }

    gsl::vector v;

    v.data = m.data + i * m.tda;
    v.size = m.ncols;
    v.stride = 1;
    v.block = m.block;
    v.owner = 0;

    return gsl::view_of<const gsl::vector>(v);
  }

  // ############################################################
  gsl::view_of<const gsl::vector>
  const_column (const gsl::matrix& m, const size_t j)
  {
    GVX_TRACE("const_column");

    if (j >= m.ncols)
      {
        LFATAL("column index is out of range");
      }

    gsl::vector v;

    v.data = m.data + j;
    v.size = m.mrows;
    v.stride = m.tda;
    v.block = m.block;
    v.owner = 0;

    return gsl::view_of<const gsl::vector>(v);
  }

  // ############################################################
  void swap_rows (gsl::matrix& m, const size_t i, const size_t j)
  {
    GVX_TRACE("swap_rows");
    const size_t mrows = m.mrows;
    const size_t ncols = m.ncols;

    if (i >= mrows)
      {
        LFATAL("first row index is out of range");
      }

    if (j >= mrows)
      {
        LFATAL("second row index is out of range");
      }

    if (i != j)
      {
        double* row1 = m.data + i * m.tda;
        double* row2 = m.data + j * m.tda;

        size_t k;

        for (k = 0; k < ncols; k++)
          {
            double tmp = row1[k];
            row1[k] = row2[k];
            row2[k] = tmp;
          }
      }
  }

  // ############################################################
  void swap_columns (gsl::matrix& m, const size_t i, const size_t j)
  {
    GVX_TRACE("swap_columns");
    const size_t mrows = m.mrows;
    const size_t ncols = m.ncols;

    if (i >= ncols)
      {
        LFATAL("first column index is out of range");
      }

    if (j >= ncols)
      {
        LFATAL("second column index is out of range");
      }

    if (i != j)
      {
        double* const col1 = m.data + i;
        double* const col2 = m.data + j;

        for (size_t p = 0; p < mrows; ++p)
          {
            size_t n = p * m.tda;

            double tmp = col1[n];
            col1[n] = col2[n];
            col2[n] = tmp;
          }
      }
  }

  // ############################################################
  inline double blas_ddot (const gsl::vector& X, const gsl::vector& Y)
  {
    if (X.size != Y.size)
      LFATAL("invalid length");

    const int N = X.size;
    const double* const Xdat = X.data;
    const int incX = X.stride;
    const double* const Ydat = Y.data;
    const int incY = Y.stride;

    double r = 0.0;

    if (incX == 1 && incY == 1)
      {
        const int m = N % 8;

        for (int i = 0; i < m; ++i)
          r += Xdat[i] * Ydat[i];

        for (int i = m; i + 7 < N; i += 8)
          {
            r += Xdat[i + 0] * Ydat[i + 0];
            r += Xdat[i + 1] * Ydat[i + 1];
            r += Xdat[i + 2] * Ydat[i + 2];
            r += Xdat[i + 3] * Ydat[i + 3];
            r += Xdat[i + 4] * Ydat[i + 4];
            r += Xdat[i + 5] * Ydat[i + 5];
            r += Xdat[i + 6] * Ydat[i + 6];
            r += Xdat[i + 7] * Ydat[i + 7];
          }
      }
    else
      {
        int ix = OFFSET(N, incX);
        int iy = OFFSET(N, incY);

        for (int i = 0; i < N; i++)
          {
            r += Xdat[ix] * Ydat[iy];
            ix += incX;
            iy += incY;
          }
      }

    return r;
  }

  // ############################################################
  double blas_dnrm2 (const gsl::vector& Xv)
  {
    const int N = Xv.size;
    const double* X = Xv.data;
    const int incX = Xv.stride;

    if (N <= 0 || incX <= 0)
      return 0;

    if (N == 1)
      return fabs(X[0]);

    double scale = 0.0;
    double ssq = 1.0;
    int ix = 0;

    for (int i = 0; i < N; ++i) {
      const double x = X[ix];

      if (x != 0.0) {
        const double ax = fabs(x);

        if (scale < ax) {
          ssq = 1.0 + ssq * (scale / ax) * (scale / ax);
          scale = ax;
        } else {
          ssq += (ax / scale) * (ax / scale);
        }
      }

      ix += incX;
    }

    return scale * sqrt(ssq);
  }

  // ############################################################
  inline void blas_dscal (const double alpha, gsl::vector& Xv)
  {
    const int N = Xv.size;
    double* X = Xv.data;
    const int incX = Xv.stride;

    if (incX <= 0)
      return;

    int ix = OFFSET(N, incX);

    for (int i = 0; i < N; ++i)
      {
        X[ix] *= alpha;
        ix += incX;
      }
  }

  // ############################################################
  // Y = Y + alpha x
  void blas_daxpy (double alpha, const gsl::vector& X, gsl::vector& Y)
  {
    if (X.size != Y.size)
      LFATAL("invalid length");

    const int N = X.size;
    const double* const Xdat = X.data;
    const int incX = X.stride;
    double* const Ydat = Y.data;
    const int incY = Y.stride;

    if (alpha == 0.0)
      {
        return;
      }

    if (incX == 1 && incY == 1)
      {
        const int m = N % 4;

        for (int i = 0; i < m; i++)
          Ydat[i] += alpha * Xdat[i];

        for (int i = m; i + 3 < N; i += 4)
          {
            Ydat[i + 0] += alpha * Xdat[i + 0];
            Ydat[i + 1] += alpha * Xdat[i + 1];
            Ydat[i + 2] += alpha * Xdat[i + 2];
            Ydat[i + 3] += alpha * Xdat[i + 3];
          }
      }
    else
      {
        int ix = OFFSET(N, incX);
        int iy = OFFSET(N, incY);

        for (int i = 0; i < N; i++)
          {
            Ydat[iy] += alpha * Xdat[ix];
            ix += incX;
            iy += incY;
          }
      }
  }

  // ############################################################
  double householder_transform (gsl::vector& v)
  {
    GVX_TRACE("householder_transform");
    /* replace v[0:n-1] with a householder vector (v[0:n-1]) and
       coefficient tau that annihilate v[1:n-1] */

    const size_t n = v.size;

    if (n == 1)
      {
        return 0.0; /* tau = 0 */
      }
    else
      {
        gsl::view_of<gsl::vector> x = v.subvector(1, n - 1);

        const double xnorm = blas_dnrm2 (x.viewee);

        if (xnorm == 0)
          {
            return 0.0; /* tau = 0 */
          }

        const double alpha = v.get(0);
        const double beta = - (alpha >= 0.0 ? +1.0 : -1.0) * hypot(alpha, xnorm);
        const double tau = (beta - alpha) / beta;

        blas_dscal (1.0 / (alpha - beta), x.viewee);
        v.set(0, beta);

        return tau;
      }
  }

  // ############################################################
  void householder_hm (double tau, const gsl::vector& v, gsl::matrix& A)
  {
    GVX_TRACE("householder_hm");
    /* applies a householder transformation v,tau to matrix m */

    if (tau == 0.0)
      return;

    gsl::view_of<const gsl::vector> v1 = v.subvector(1, v.size - 1);
    gsl::view_of<gsl::matrix> A1 = submatrix_of (A, 1, 0, A.mrows - 1, A.ncols);

    for (size_t j = 0; j < A.ncols; ++j)
      {
        gsl::view_of<gsl::vector> A1j = column_of(A1.viewee, j);
        double wj = blas_ddot (A1j.viewee, v1.viewee);
        wj += A.get(0,j);

        A.at(0, j) -= (tau * wj);

        blas_daxpy (-tau * wj, v1.viewee, A1j.viewee);
      }
  }

  // ############################################################
  int householder_mh (double tau, const gsl::vector& v, gsl::matrix& A)
  {
    GVX_TRACE("householder_mh");
    /* applies a householder transformation v,tau to matrix m from the
       right hand side in order to zero out rows */

    if (tau == 0)
      return 0;

    /* A = A - tau w v' */

    gsl::view_of<const gsl::vector> v1 = v.subvector(1, v.size - 1);
    gsl::view_of<gsl::matrix> A1 = submatrix_of (A, 0, 1, A.mrows, A.ncols-1);

    for (size_t i = 0; i < A.mrows; ++i)
      {
        gsl::view_of<gsl::vector> A1i = row_of(A1.viewee, i);
        double wi = blas_ddot (A1i.viewee, v1.viewee);
        wi += A.get(i,0);

        A.at(i,0) -= (tau * wi);

        blas_daxpy(-tau * wi, v1.viewee, A1i.viewee);
      }

    return 0;
  }

  // ############################################################
  int householder_hv (double tau, const gsl::vector& v, gsl::vector& w)
  {
    GVX_TRACE("householder_hv");
    /* applies a householder transformation v to vector w */
    const size_t N = v.size;

    if (tau == 0)
      return 0;

    {
      /* compute d = v'w */

      const double d0 = w.get(0);

      gsl::view_of<const gsl::vector> v1 = v.subvector(1, N-1);
      gsl::view_of<gsl::vector> w1 = w.subvector(1, N-1);

      const double d1 = blas_ddot (v1.viewee, w1.viewee);

      const double d = d0 + d1;

      /* compute w = w - tau (v) (v'w) */

      w.at(0) -= (tau * d);

      blas_daxpy (-tau * d, v1.viewee, w1.viewee);
    }

    return 0;
  }


  // ############################################################
  void householder_hm1 (const double tau, gsl::matrix& A)
  {
    GVX_TRACE("householder_hm1");
    /* applies a householder transformation v,tau to a matrix being
       build up from the identity matrix, using the first column of A as
       a householder vector */

    if (tau == 0)
      {
        A.set(0, 0, 1.0);

        for (size_t j = 1; j < A.ncols; ++j)
          {
            A.set(0, j, 0.0);
          }

        for (size_t i = 1; i < A.mrows; ++i)
          {
            A.set(i, 0, 0.0);
          }

        return;
      }

    /* w = A' v */

    gsl::view_of<gsl::matrix> A1 = submatrix_of (A, 1, 0, A.mrows - 1, A.ncols);
    gsl::view_of<gsl::vector> v1 = column_of (A1.viewee, 0);

    for (size_t j = 1; j < A.ncols; j++)
      {
        GVX_TRACE("householder_hm1-loop");

        /* A = A - tau v w' */

        gsl::view_of<gsl::vector> A1j = column_of(A1.viewee, j);

        const double wj = blas_ddot (v1.viewee, A1j.viewee);
        blas_daxpy(-tau*wj, v1.viewee, A1j.viewee);

        A.set(0, j, - tau *  wj);
      }

    blas_dscal(-tau, v1.viewee);

    A.set(0, 0, 1.0 - tau);
  }


  // ############################################################
  void create_givens (const double a, const double b,
                      double* c, double* s)
  {
    /* Generate a Givens rotation (cos,sin) which takes v=(x,y) to (|v|,0)

    From Golub and Van Loan, "Matrix Computations", Section 5.1.8 */

    if (b == 0)
      {
        *c = 1;
        *s = 0;
      }
    else if (fabs (b) > fabs (a))
      {
        const double t = -a / b;
        const double s1 = 1.0 / sqrt (1 + t * t);
        *s = s1;
        *c = s1 * t;
      }
    else
      {
        const double t = -b / a;
        const double c1 = 1.0 / sqrt (1 + t * t);
        *c = c1;
        *s = c1 * t;
      }
  }

  // ############################################################
  void chop_small_elements (gsl::vector& d, gsl::vector& f)
  {
    const size_t N = d.size;
    double d_i = d.get(0);

    for (size_t i = 0; i < N - 1; ++i)
      {
        double f_i = f.get(i);
        double d_ip1 = d.get(i + 1);

        if (fabs (f_i) < DBL_EPSILON * (fabs (d_i) + fabs (d_ip1)))
          {
            f.set(i, 0.0);
          }
        d_i = d_ip1;
      }

  }

  // ############################################################
  double trailing_eigenvalue (const gsl::vector& d, const gsl::vector& f)
  {
    GVX_TRACE("trailing_eigenvalue");
    const size_t n = d.size;

    const double da = d.get(n - 2);
    const double db = d.get(n - 1);
    const double fa = (n > 2) ? f.get(n - 3) : 0.0;
    const double fb = f.get(n - 2);

    const double ta = da * da + fa * fa;
    const double tb = db * db + fb * fb;
    const double tab = da * fb;

    const double dt = (ta - tb) / 2.0;

    double mu;

    if (dt >= 0)
      {
        mu = tb - (tab * tab) / (dt + hypot (dt, tab));
      }
    else
      {
        mu = tb + (tab * tab) / ((-dt) + hypot (dt, tab));
      }

    return mu;
  }

  // ############################################################
  void create_schur (double d0, double f0, double d1,
                     double* c, double* s)
  {
    GVX_TRACE("create_schur");
    const double apq = 2.0 * d0 * f0;

    if (apq != 0.0)
      {
        double t;
        const double tau = (f0*f0 + (d1 + d0)*(d1 - d0)) / apq;

        if (tau >= 0.0)
          {
            t = 1.0/(tau + hypot(1.0, tau));
          }
        else
          {
            t = -1.0/(-tau + hypot(1.0, tau));
          }

        *c = 1.0 / hypot(1.0, t);
        *s = t * (*c);
      }
    else
      {
        *c = 1.0;
        *s = 0.0;
      }
  }

  // ############################################################
  void svd2 (gsl::vector& d, gsl::vector& f, gsl::matrix& U, gsl::matrix& V)
  {
    GVX_TRACE("svd2");

    double c, s, a11, a12, a21, a22;

    const size_t M = U.mrows;
    const size_t N = V.mrows;

    const double d0 = d.get(0);
    const double f0 = f.get(0);

    const double d1 = d.get(1);

    if (d0 == 0.0)
      {
        /* Eliminate off-diagonal element in [0,f0;0,d1] to make [d,0;0,0] */

        create_givens (f0, d1, &c, &s);

        /* compute B <= G^T B X,  where X = [0,1;1,0] */

        d.set(0, c * f0 - s * d1);
        f.set(0, s * f0 + c * d1);
        d.set(1, 0.0);

        /* Compute U <= U G */

        for (size_t i = 0; i < M; ++i)
          {
            const double Uip = U.get(i, 0);
            const double Uiq = U.get(i, 1);
            U.set(i, 0, c * Uip - s * Uiq);
            U.set(i, 1, s * Uip + c * Uiq);
          }

        /* Compute V <= V X */

        swap_columns (V, 0, 1);

        return;
      }
    else if (d1 == 0.0)
      {
        /* Eliminate off-diagonal element in [d0,f0;0,0] */

        create_givens (d0, f0, &c, &s);

        /* compute B <= B G */

        d.set(0, d0 * c - f0 * s);
        f.set(0, 0.0);

        /* Compute V <= V G */

        for (size_t i = 0; i < N; ++i)
          {
            const double Vip = V.get(i, 0);
            const double Viq = V.get(i, 1);
            V.set(i, 0, c * Vip - s * Viq);
            V.set(i, 1, s * Vip + c * Viq);
          }

        return;
      }
    else
      {
        /* Make columns orthogonal, A = [d0, f0; 0, d1] * G */

        create_schur (d0, f0, d1, &c, &s);

        /* compute B <= B G */

        a11 = c * d0 - s * f0;
        a21 = - s * d1;

        a12 = s * d0 + c * f0;
        a22 = c * d1;

        /* Compute V <= V G */

        for (size_t i = 0; i < N; ++i)
          {
            double Vip = V.get(i, 0);
            double Viq = V.get(i, 1);
            V.set(i, 0, c * Vip - s * Viq);
            V.set(i, 1, s * Vip + c * Viq);
          }

        /* Eliminate off-diagonal elements, bring column with largest
           norm to first column */

        if (hypot(a11, a21) < hypot(a12,a22))
          {
            /* B <= B X */

            const double t1 = a11; a11 = a12; a12 = t1;
            const double t2 = a21; a21 = a22; a22 = t2;

            /* V <= V X */

            swap_columns(V, 0, 1);
          }

        create_givens (a11, a21, &c, &s);

        /* compute B <= G^T B */

        d.set(0, c * a11 - s * a21);
        f.set(0, c * a12 - s * a22);
        d.set(1, s * a12 + c * a22);

        /* Compute U <= U G */

        for (size_t i = 0; i < M; ++i)
          {
            double Uip = U.get(i, 0);
            double Uiq = U.get(i, 1);
            U.set(i, 0, c * Uip - s * Uiq);
            U.set(i, 1, s * Uip + c * Uiq);
          }

        return;
      }
  }


  // ############################################################
  void chase_out_intermediate_zero (gsl::vector& d, gsl::vector& f, gsl::matrix& U, size_t k0)
  {
    GVX_TRACE("chase_out_intermediate_zero");
    const size_t M = U.mrows;
    const size_t n = d.size;
    double c, s;

    double x = f.get(k0);
    double y = d.get(k0+1);

    for (size_t k = k0; k < n - 1; ++k)
      {
        create_givens (y, -x, &c, &s);

        /* Compute U <= U G */

        for (size_t i = 0; i < M; ++i)
          {
            double Uip = U.get(i, k0);
            double Uiq = U.get(i, k + 1);
            U.set(i, k0, c * Uip - s * Uiq);
            U.set(i, k + 1, s * Uip + c * Uiq);
          }

        /* compute B <= G^T B */

        d.set(k + 1, s * x + c * y);

        if (k == k0)
          f.set(k, c * x - s * y );

        if (k < n - 2)
          {
            double z = f.get(k + 1);
            f.set(k + 1, c * z);

            x = -s * z;
            y = d.get(k + 2);
          }
      }
  }

  // ############################################################
  void chase_out_trailing_zero (gsl::vector& d, gsl::vector& f, gsl::matrix& V)
  {
    GVX_TRACE("chase_out_trailing_zero");
    const size_t N = V.mrows;
    const size_t n = d.size;
    double c, s;

    double x = d.get(n - 2);
    double y = f.get(n - 2);

    for (size_t k = n - 1; k > 0 && k--; /* */)
      {
        create_givens (x, y, &c, &s);

        /* Compute V <= V G where G = [c, s; -s, c] */

        for (size_t i = 0; i < N; ++i)
          {
            double Vip = V.get(i, k);
            double Viq = V.get(i, n - 1);
            V.set(i, k, c * Vip - s * Viq);
            V.set(i, n - 1, s * Vip + c * Viq);
          }

        /* compute B <= B G */

        d.set(k, c * x - s * y);

        if (k == n - 2)
          f.set(k, s * x + c * y );

        if (k > 0)
          {
            double z = f.get(k - 1);
            f.set(k - 1, c * z);

            x = d.get(k - 1);
            y = s * z;
          }
      }
  }

  // ############################################################
  void qrstep (gsl::vector& d, gsl::vector& f, gsl::matrix& U, gsl::matrix& V)
  {
    GVX_TRACE("qrstep");
    const size_t M = U.mrows;
    const size_t N = V.mrows;
    const size_t n = d.size;
    double y, z;
    double ak, bk, zk, ap, bp, aq;
    size_t i, k;

    if (n == 1)
      return;  /* shouldn't happen */

    /* Compute 2x2 svd directly */

    if (n == 2)
      {
        svd2 (d, f, U, V);
        return;
      }

    /* Chase out any zeroes on the diagonal */

    for (i = 0; i < n - 1; i++)
      {
        const double d_i = d.get(i);

        if (d_i == 0.0)
          {
            chase_out_intermediate_zero (d, f, U, i);
            return;
          }
      }

    /* Chase out any zero at the end of the diagonal */

    {
      const double d_nm1 = d.get(n - 1);

      if (d_nm1 == 0.0)
        {
          chase_out_trailing_zero (d, f, V);
          return;
        }
    }


    /* Apply QR reduction steps to the diagonal and offdiagonal */

    {
      const double d0 = d.get(0);
      const double f0 = f.get(0);

      const double d1 = d.get(1);
      /*const double f1 =*/ f.get(1);

      {
        const double mu = trailing_eigenvalue (d, f);

        y = d0 * d0 - mu;
        z = d0 * f0;
      }

      /* Set up the recurrence for Givens rotations on a bidiagonal matrix */

      ak = 0;
      bk = 0;

      ap = d0;
      bp = f0;

      aq = d1;
    }

    for (k = 0; k < n - 1; k++)
      {
        double c, s;
        create_givens (y, z, &c, &s);

        /* Compute V <= V G */

        for (i = 0; i < N; i++)
          {
            double Vip = V.get(i, k);
            double Viq = V.get(i, k + 1);
            V.set(i, k, c * Vip - s * Viq);
            V.set(i, k + 1, s * Vip + c * Viq);
          }

        /* compute B <= B G */

        {
          const double bk1 = c * bk - s * z;

          const double ap1 = c * ap - s * bp;
          const double bp1 = s * ap + c * bp;
          const double zp1 = -s * aq;

          const double aq1 = c * aq;

          if (k > 0)
            {
              f.set(k - 1, bk1);
            }

          ak = ap1;
          bk = bp1;
          zk = zp1;

          ap = aq1;

          if (k < n - 2)
            {
              bp = f.get(k + 1);
            }
          else
            {
              bp = 0.0;
            }

          y = ak;
          z = zk;
        }

        create_givens (y, z, &c, &s);

        /* Compute U <= U G */

        for (i = 0; i < M; i++)
          {
            const double Uip = U.get(i, k);
            const double Uiq = U.get(i, k + 1);
            U.set(i, k, c * Uip - s * Uiq);
            U.set(i, k + 1, s * Uip + c * Uiq);
          }

        /* compute B <= G^T B */

        {
          const double ak1 = c * ak - s * zk;
          const double bk1 = c * bk - s * ap;
          const double zk1 = -s * bp;

          const double ap1 = s * bk + c * ap;
          const double bp1 = c * bp;

          d.set(k, ak1);

          ak = ak1;
          bk = bk1;
          zk = zk1;

          ap = ap1;
          bp = bp1;

          if (k < n - 2)
            {
              aq = d.get(k + 2);
            }
          else
            {
              aq = 0.0;
            }

          y = bk;
          z = zk;
        }
      }

    f.set(n - 2, bk);
    d.set(n - 1, ap);
  }



  // ############################################################
  // bidiag_decomp

  /* Factorise a matrix A into
   *
   * A = U B V^T
   *
   * where U and V are orthogonal and B is upper bidiagonal.
   *
   * On exit, B is stored in the diagonal and first superdiagonal of A.
   *
   * U is stored as a packed set of Householder transformations in the
   * lower triangular part of the input matrix below the diagonal.
   *
   * V is stored as a packed set of Householder transformations in the
   * upper triangular part of the input matrix above the first
   * superdiagonal.
   *
   * The full matrix for U can be obtained as the product
   *
   *       U = U_1 U_2 .. U_N
   *
   * where
   *
   *       U_i = (I - tau_i * u_i * u_i')
   *
   * and where u_i is a Householder vector
   *
   *       u_i = [0, .. , 0, 1, A(i+1,i), A(i+3,i), .. , A(M,i)]
   *
   * The full matrix for V can be obtained as the product
   *
   *       V = V_1 V_2 .. V_(N-2)
   *
   * where
   *
   *       V_i = (I - tau_i * v_i * v_i')
   *
   * and where v_i is a Householder vector
   *
   *       v_i = [0, .. , 0, 1, A(i,i+2), A(i,i+3), .. , A(i,N)]
   *
   * See Golub & Van Loan, "Matrix Computations" (3rd ed), Algorithm 5.4.2
   *
   * Note: this description uses 1-based indices. The code below uses
   * 0-based indices
   */

  void bidiag_decomp (gsl::matrix& A, gsl::vector& tau_U, gsl::vector& tau_V)
  {
    GVX_TRACE("bidiag_decomp");
    if (A.mrows < A.ncols)
      {
        LFATAL("bidiagonal decomposition requires M>=N");
      }
    else if (tau_U.size  != A.ncols)
      {
        LFATAL("size of tau_U must be N");
      }
    else if (tau_V.size + 1 != A.ncols)
      {
        LFATAL("size of tau_V must be (N - 1)");
      }
    else
      {
        const size_t M = A.mrows;
        const size_t N = A.ncols;

        for (size_t i = 0; i < N; ++i)
          {
            /* Apply Householder transformation to current column */

            {
              gsl::view_of<gsl::vector> c = column_of (A, i);
              gsl::view_of<gsl::vector> v = c.viewee.subvector(i, M - i);
              const double tau_i = householder_transform (v.viewee);

              /* Apply the transformation to the remaining columns */

              if (i + 1 < N)
                {
                  gsl::view_of<gsl::matrix> m =
                    submatrix_of (A, i, i + 1, M - i, N - (i + 1));
                  householder_hm (tau_i, v.viewee, m.viewee);
                }

              tau_U.set(i, tau_i);

            }

            /* Apply Householder transformation to current row */

            if (i + 1 < N)
              {
                gsl::view_of<gsl::vector> r = row_of (A, i);
                gsl::view_of<gsl::vector> v = r.viewee.subvector(i + 1, N - (i + 1));
                const double tau_i = householder_transform (v.viewee);

                /* Apply the transformation to the remaining rows */

                if (i + 1 < M)
                  {
                    gsl::view_of<gsl::matrix> m =
                      submatrix_of (A, i+1, i+1, M - (i+1), N - (i+1));
                    householder_mh (tau_i, v.viewee, m.viewee);
                  }

                tau_V.set(i, tau_i);
              }
          }
      }
  }

  // ############################################################
  /* Form the orthogonal matrices U, V, diagonal d and superdiagonal sd
     from the packed bidiagonal matrix A */
  void bidiag_unpack (const gsl::matrix& A,
                      const gsl::vector& tau_U,
                      gsl::matrix& U,
                      const gsl::vector& tau_V,
                      gsl::matrix& V,
                      gsl::vector& diag,
                      gsl::vector& superdiag)
  {
    GVX_TRACE("bidiag_unpack");
    const size_t M = A.mrows;
    const size_t N = A.ncols;

    const size_t K = std::min(M, N);

    if (M < N)
      {
        LFATAL("matrix A must have M >= N");
      }
    else if (tau_U.size != K)
      {
        LFATAL("size of tau must be MIN(M,N)");
      }
    else if (tau_V.size + 1 != K)
      {
        LFATAL("size of tau must be MIN(M,N) - 1");
      }
    else if (U.mrows != M || U.ncols != N)
      {
        LFATAL("size of U must be M x N");
      }
    else if (V.mrows != N || V.ncols != N)
      {
        LFATAL("size of V must be N x N");
      }
    else if (diag.size != K)
      {
        LFATAL("size of diagonal must match size of A");
      }
    else if (superdiag.size + 1 != K)
      {
        LFATAL("size of subdiagonal must be (diagonal size - 1)");
      }

    /* Copy diagonal into diag */

    for (size_t i = 0; i < N; ++i)
      {
        const double Aii = A.get(i, i);
        diag.set(i, Aii);
      }

    /* Copy superdiagonal into superdiag */

    for (size_t i = 0; i < N - 1; ++i)
      {
        const double Aij = A.get(i, i+1);
        superdiag.set(i, Aij);
      }

    /* Initialize V to the identity */

    set_identity (V);

    for (size_t i = N - 1; i > 0 && i--; /* */)
      {
        /* Householder row transformation to accumulate V */
        gsl::view_of<const gsl::vector> r = const_row (A, i);
        gsl::view_of<const gsl::vector> h =
          r.viewee.subvector(i + 1, N - (i+1));

        const double ti = tau_V.get(i);

        gsl::view_of<gsl::matrix> m =
          submatrix_of (V, i + 1, i + 1, N-(i+1), N-(i+1));

        householder_hm (ti, h.viewee, m.viewee);
      }

    /* Initialize U to the identity */

    set_identity (U);

    for (size_t j = N; j > 0 && j--; /* */)
      {
        /* Householder column transformation to accumulate U */
        gsl::view_of<const gsl::vector> c = const_column (A, j);
        gsl::view_of<const gsl::vector> h = c.viewee.subvector(j, M - j);
        const double tj = tau_U.get(j);

        gsl::view_of<gsl::matrix> m =
          submatrix_of (U, j, j, M-j, N-j);

        householder_hm (tj, h.viewee, m.viewee);
      }

    return;
  }

  // ############################################################
  void bidiag_unpack2 (gsl::matrix& A,
                       gsl::vector& tau_U,
                       gsl::vector& tau_V,
                       gsl::matrix& V)
  {
    GVX_TRACE("bidiag_unpack2");
    const size_t M = A.mrows;
    const size_t N = A.ncols;

    const size_t K = std::min(M, N);

    if (M < N)
      {
        LFATAL("matrix A must have M >= N");
      }
    else if (tau_U.size != K)
      {
        LFATAL("size of tau must be MIN(M,N)");
      }
    else if (tau_V.size + 1 != K)
      {
        LFATAL("size of tau must be MIN(M,N) - 1");
      }
    else if (V.mrows != N || V.ncols != N)
      {
        LFATAL("size of V must be N x N");
      }

    /* Initialize V to the identity */

    set_identity (V);

    for (size_t i = N - 1; i > 0 && i--; /* */)
      {
        /* Householder row transformation to accumulate V */
        gsl::view_of<const gsl::vector> r = const_row (A, i);
        gsl::view_of<const gsl::vector> h =
          r.viewee.subvector(i + 1, N - (i+1));

        const double ti = tau_V.get(i);

        gsl::view_of<gsl::matrix> m =
          submatrix_of (V, i + 1, i + 1, N-(i+1), N-(i+1));

        householder_hm (ti, h.viewee, m.viewee);
      }

    /* Copy superdiagonal into tau_v */

    for (size_t i = 0; i < N - 1; ++i)
      {
        const double Aij = A.get(i, i+1);
        tau_V.set(i, Aij);
      }

    /* Allow U to be unpacked into the same memory as A, copy
       diagonal into tau_U */

    for (size_t j = N; j > 0 && j--; /* */)
      {
        /* Householder column transformation to accumulate U */
        const double tj = tau_U.get(j);
        const double Ajj = A.get(j, j);
        gsl::view_of<gsl::matrix> m = submatrix_of (A, j, j, M-j, N-j);

        tau_U.set(j, Ajj);
        householder_hm1 (tj, m.viewee);
      }

  }


  // ############################################################
  void bidiag_unpack_B (const gsl::matrix& A,
                        gsl::vector& diag,
                        gsl::vector& superdiag)
  {
    GVX_TRACE("bidiag_unpack_B");
    const size_t M = A.mrows;
    const size_t N = A.ncols;

    const size_t K = std::min(M, N);

    if (diag.size != K)
      {
        LFATAL("size of diagonal must match size of A");
      }
    else if (superdiag.size + 1 != K)
      {
        LFATAL("size of subdiagonal must be (matrix size - 1)");
      }
    else
      {

        /* Copy diagonal into diag */

        for (size_t i = 0; i < K; ++i)
          {
            const double Aii = A.get(i, i);
            diag.set(i, Aii);
          }

        /* Copy superdiagonal into superdiag */

        for (size_t i = 0; i < K - 1; ++i)
          {
            const double Aij = A.get(i, i+1);
            superdiag.set(i, Aij);
          }
      }
  }

  // ############################################################
  /* Factorise a general M x N matrix A into,
   *
   *   A = U D V^T
   *
   * where U is a column-orthogonal M x N matrix (U^T U = I),
   * D is a diagonal N x N matrix,
   * and V is an N x N orthogonal matrix (V^T V = V V^T = I)
   *
   * U is stored in the original matrix A, which has the same size
   *
   * V is stored as a separate matrix (not V^T). You must take the
   * transpose to form the product above.
   *
   * The diagonal matrix D is stored in the vector S,  D_ii = S_i
   */

  void SV_decomp (gsl::matrix& A, gsl::matrix& V, gsl::vector& S,
                  gsl::vector& work)
  {
    GVX_TRACE("SV_decomp");
    size_t a, b;

    const size_t M = A.mrows;
    const size_t N = A.ncols;
    const size_t K = std::min(M, N);

    if (M < N)
      {
        LFATAL("svd of [M=%" ZU "]x[N=%" ZU "] matrix, M<N, is not implemented",
               M, N);
      }
    else if (V.mrows != V.ncols)
      {
        LFATAL("matrix V [=%" ZU "x%" ZU "] must be square", V.mrows, V.ncols);
      }
    else if (V.mrows != N)
      {
        LFATAL("square matrix V [=%" ZU "x%" ZU "] must match second dimension of matrix A [=%" ZU "]",
               V.mrows, V.ncols, N);
      }
    else if (S.size != N)
      {
        LFATAL("length of vector S [=%" ZU "] must match second dimension of matrix A [=%" ZU "]",
               S.size, N);
      }
    else if (work.size != N)
      {
        LFATAL("length of workspace [=%" ZU "] must match second dimension of matrix A [=%" ZU "]",
               work.size, N);
      }

    /* Handle the case of N = 1 (SVD of a column vector) */

    if (N == 1)
      {
        gsl::view_of<gsl::vector> column = column_of (A, 0);
        const double norm = blas_dnrm2 (column.viewee);

        S.set(0, norm);
        V.set(0, 0, 1.0);

        if (norm != 0.0)
          {
            blas_dscal (1.0/norm, column.viewee);
          }

        return;
      }

    {
      gsl::view_of<gsl::vector> f = work.subvector(0, K - 1);

      /* bidiagonalize matrix A, unpack A into U S V */

      bidiag_decomp (A, S, f.viewee);
      bidiag_unpack2 (A, S, f.viewee, V);

      /* apply reduction steps to B=(S,Sd) */

      chop_small_elements (S, f.viewee);

      /* Progressively reduce the matrix until it is diagonal */

      b = N - 1;

      while (b > 0)
        {
          const double fbm1 = f.viewee.get(b - 1);

          if (fbm1 == 0.0 || isnan (fbm1))
            {
              b--;
              continue;
            }

          /* Find the largest unreduced block (a,b) starting from b
             and working backwards */

          a = b - 1;

          while (a > 0)
            {
              const double fam1 = f.viewee.get(a - 1);

              if (fam1 == 0.0 || isnan (fam1))
                {
                  break;
                }

              a--;
            }

          {
            const size_t n_block = b - a + 1;
            gsl::view_of<gsl::vector> S_block = S.subvector(a, n_block);
            gsl::view_of<gsl::vector> f_block = f.viewee.subvector(a, n_block - 1);

            gsl::view_of<gsl::matrix> U_block =
              submatrix_of (A, 0, a, A.mrows, n_block);
            gsl::view_of<gsl::matrix> V_block =
              submatrix_of (V, 0, a, V.mrows, n_block);

            qrstep (S_block.viewee, f_block.viewee, U_block.viewee, V_block.viewee);

            /* remove any small off-diagonal elements */

            chop_small_elements (S_block.viewee, f_block.viewee);
          }
        }
    }
    /* Make singular values positive by reflections if necessary */

    for (size_t j = 0; j < K; ++j)
      {
        const double Sj = S.get(j);

        if (Sj < 0.0)
          {
            for (size_t i = 0; i < N; ++i)
              {
                const double Vij = V.get(i, j);
                V.set(i, j, -Vij);
              }

            S.set(j, -Sj);
          }
      }

    /* Sort singular values into decreasing order */

    for (size_t i = 0; i < K; ++i)
      {
        double S_max = S.get(i);
        size_t i_max = i;

        for (size_t j = i + 1; j < K; ++j)
          {
            const double Sj = S.get(j);

            if (Sj > S_max)
              {
                S_max = Sj;
                i_max = j;
              }
          }

        if (i_max != i)
          {
            /* swap eigenvalues */
            S.swap_elements(i, i_max);

            /* swap eigenvectors */
            swap_columns (A, i, i_max);
            swap_columns (V, i, i_max);
          }
      }
  }


  // ############################################################
  /* Modified algorithm which is better for M>>N */
  void linalg_SV_decomp_mod (gsl::matrix& A,
                             gsl::matrix& X,
                             gsl::matrix& V,
                             gsl::vector& S,
                             gsl::vector& work)
  {
    GVX_TRACE("linalg_SV_decomp_mod");

    const size_t M = A.mrows;
    const size_t N = A.ncols;

    if (M < N)
      {
        LFATAL("svd of [M=%" ZU "]x[N=%" ZU "] matrix, M<N, is not implemented",
               M, N);
      }
    else if (V.mrows != V.ncols)
      {
        LFATAL("matrix V [=%" ZU "x%" ZU "] must be square", V.mrows, V.ncols);
      }
    else if (V.mrows != N)
      {
        LFATAL("square matrix V [=%" ZU "x%" ZU "] must match second dimension of matrix A [=%" ZU "]",
               V.mrows, V.ncols, N);
      }
    else if (X.mrows != X.ncols)
      {
        LFATAL("matrix X [=%" ZU "x%" ZU "] must be square", X.mrows, X.ncols);
      }
    else if (X.mrows != N)
      {
        LFATAL("square matrix X [=%" ZU "x%" ZU "] must match second dimension of matrix A [=%" ZU "]",
               X.mrows, X.ncols, N);
      }
    else if (S.size != N)
      {
        LFATAL("length of vector S [=%" ZU "] must match second dimension of matrix A [=%" ZU "]",
               S.size, N);
      }
    else if (work.size != N)
      {
        LFATAL("length of workspace [=%" ZU "] must match second dimension of matrix A [=%" ZU "]",
               work.size, N);
      }

    if (N == 1)
      {
        gsl::view_of<gsl::vector> column = column_of (A, 0);
        const double norm = blas_dnrm2 (column.viewee);

        S.set(0, norm);
        V.set(0, 0, 1.0);

        if (norm != 0.0)
          {
            blas_dscal (1.0/norm, column.viewee);
          }

        return;
      }

    /* Convert A into an upper triangular matrix R */

    for (size_t i = 0; i < N; ++i)
      {
        GVX_TRACE("linalg_SV_decomp_mod-loop1");
        gsl::view_of<gsl::vector> c = column_of (A, i);
        gsl::view_of<gsl::vector> v = c.viewee.subvector(i, M - i);
        const double tau_i = householder_transform (v.viewee);

        /* Apply the transformation to the remaining columns */

        if (i + 1 < N)
          {
            gsl::view_of<gsl::matrix> m =
              submatrix_of (A, i, i + 1, M - i, N - (i + 1));
            householder_hm (tau_i, v.viewee, m.viewee);
          }

        S.set(i, tau_i);
      }

    /* Copy the upper triangular part of A into X */

    for (size_t i = 0; i < N; ++i)
      {
        GVX_TRACE("linalg_SV_decomp_mod-loop2");
        for (size_t j = 0; j < i; j++)
          {
            X.set(i, j, 0.0);
          }

        {
          const double Aii = A.get(i, i);
          X.set(i, i, Aii);
        }

        for (size_t j = i + 1; j < N; ++j)
          {
            const double Aij = A.get(i, j);
            X.set(i, j, Aij);
          }
      }

    /* Convert A into an orthogonal matrix L */

    for (size_t j = N; j > 0 && j--; /* */)
      {
        GVX_TRACE("linalg_SV_decomp_mod-loop3");
        /* Householder column transformation to accumulate L */
        const double tj = S.get(j);
        gsl::view_of<gsl::matrix> m = submatrix_of (A, j, j, M - j, N - j);
        householder_hm1 (tj, m.viewee);
      }

    /* unpack R into X V S */

    {
      GVX_TRACE("linalg_SV_decomp_mod-loop4");
      SV_decomp (X, V, S, work);
    }

    /* Multiply L by X, to obtain U = L X, stored in U */

    {
      GVX_TRACE("linalg_SV_decomp_mod-loop5");

      gsl::view_of<gsl::vector> sum = work.subvector(0, N);

      for (size_t i = 0; i < M; ++i)
        {
          gsl::view_of<gsl::vector> L_i = row_of (A, i);
          sum.viewee.set_all(0.0);

          for (size_t j = 0; j < N; ++j)
            {
              const double Lij = L_i.viewee.get(j);
              gsl::view_of<gsl::vector> X_j = row_of (X, j);
              blas_daxpy (Lij, X_j.viewee, sum.viewee);
            }

          L_i.viewee.copy_of(sum.viewee);
        }
    }
  }

  // ############################################################
  gsl::matrix image2matrix(const Image<double>& M)
  {
    const int h = M.getHeight();
    const int w = M.getWidth();
    gsl::matrix m(h, w);
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x)
        m.set(y, x, M.getVal(x, y));
    return m;
  }

  // ############################################################
  Image<double> matrix2image(const gsl::matrix& m)
  {
    const int h = m.mrows;
    const int w = m.ncols;
    Image<double> M(w, h, NO_INIT);
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x)
        M.setVal(x, y, m.get(y, x));
    return M;
  }

  // ############################################################
  Image<double> vector2image(const gsl::vector& m)
  {
    const int h = m.size;
    Image<double> M(1, h, NO_INIT);
    for (int y = 0; y < h; ++y)
      M.setVal(y, m.get(y));
    return M;
  }

  // ############################################################
  Image<double> diag2image(const Image<double>& diag)
  {
    ASSERT(diag.isVector());
    const int s = diag.getSize();
    Image<double> result(s, s, ZEROS);
    for (int i = 0; i < s; ++i)
      result.setVal(i, i, diag.getVal(i));
    return result;
  }

} // end unnamed namespace

// ######################################################################
void gsl::svd(const Image<double>& A,
              Image<double>& U, Image<double>& S, Image<double>& V,
              const SvdFlag flags)
{
  gsl::matrix m = image2matrix(A);
  gsl::matrix v(m.ncols, m.ncols);
  gsl::vector s(m.ncols);
  gsl::vector work(m.ncols);

  if (flags & SVD_FULL)
    LFATAL("SVD_FULL not supported (try the lapack svd instead)");

  if (flags & SVD_TALL)
    {
      gsl::matrix x(m.ncols, m.ncols);
      linalg_SV_decomp_mod(m, x, v, s, work);
    }
  else
    {
      SV_decomp(m, v, s, work);
    }

  U = matrix2image(m);
  S = diag2image(vector2image(s));
  V = matrix2image(v);
}

// ######################################################################
void gsl::svdf(const Image<float>& A,
               Image<float>& U, Image<float>& S, Image<float>& V,
               const SvdFlag flags)
{
  // we don't have a single-precision gsl version, so we'll just
  // have to convert to double and back

  Image<double> Ud, Sd, Vd;

  gsl::svd(Image<double>(A), Ud, Sd, Vd, flags);
  U = Ud;
  S = Sd;
  V = Vd;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_GSL_C_DEFINED

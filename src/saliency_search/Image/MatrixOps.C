/*!@file Image/MatrixOps.C Matrix operations on Image
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/MatrixOps.C $
// $Id: MatrixOps.C 14720 2011-04-12 19:58:53Z itti $

#include "Image/MatrixOps.H"

// WARNING: Try not include any other "Image*Ops.H" headers here -- if
// you find that you are needing such a header, then the function
// you're writing probably belongs outside Image_MatrixOps.C.
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/lapack.H" // for BLAS and LAPACK decls (e.g. dgemm_())
#include "Util/Assert.H"
#include "Util/MathFunctions.H"
#include "rutz/trace.h"

namespace
{
  // #################################################################
  // hand-rolled implementation of matrix-matrix multiplication
  template <class T>
  Image<typename promote_trait<T,T>::TP>
  basic_vm_mul(const Image<T>& v, const Image<T>& M)
  {
  GVX_TRACE(__PRETTY_FUNCTION__);

    typedef typename promote_trait<T,T>::TP TF;

    const int wv = v.getWidth(), hv = v.getHeight();
    const int wm = M.getWidth(), hm = M.getHeight();

    ASSERT(wv == hm);
    ASSERT(hv == 1);

    Image<TF> y(wm, hv /* == 1 */, ZEROS);

    // for efficiency, use pointers to the input matrices' raw storage
    const T* const vptr = v.getArrayPtr();
    const T* const mptr = M.getArrayPtr();
    TF*      const yptr = y.getArrayPtr();

    for (int xv = 0; xv < wv; ++xv)
      {
        const T* const mrow = mptr + xv*wm;

        const T vval = vptr[xv];
        if (vval != 0.0)
          for (int xm = 0; xm < wm; ++xm)
            yptr[xm] += vval * mrow[xm];
      }

    return y;
  }

  // #################################################################
  // hand-rolled implementation of matrix-matrix multiplication
  template <class T>
  Image<typename promote_trait<T,T>::TP>
  basic_mm_mul(const Image<T>& A, const Image<T>& B)
  {
  GVX_TRACE(__PRETTY_FUNCTION__);

    typedef typename promote_trait<T,T>::TP TF;

    const int wa = A.getWidth(), ha = A.getHeight();
    const int wb = B.getWidth(), hb = B.getHeight();
    ASSERT(wa == hb);

    Image<TF> C(wb, ha, ZEROS);

    // for efficiency, use pointers to the input matrices' raw storage
    const T* const aptr = A.getArrayPtr();
    const T* const bptr = B.getArrayPtr();
    TF*      const cptr = C.getArrayPtr();

    /*
      NOTE: in principle, we are doing the following loop

      for (int xa = 0; xa < wa; ++xa)
      for (int yc = 0; yc < ha; ++yc)
      for (int xc = 0; xc < wb; ++xc)
      cptr[xc + yc*wb] += aptr[xa + yc*wa] * bptr[xc + xa*wb];

      and in principle, we could put the individual for-loops in any
      order we want, since the inner loop computation is the same.

      However, in practice, there is a significant cpu efficiency
      advantage to be gained by ordering the loops as done below, with
      the 'xc' loop as the inner loop. That way, we access both the B
      and C arrays with a stride of 1 in the inner loop. With a
      different ordering, we would be accessing only one of the three
      arrays with stride 1 in the inner loop, and this hurts
      performance because it pessimizes memory cache retrievals.
    */

    for (int xa = 0; xa < wa; ++xa)
      {
        const T* const bpos = bptr + xa*wb;

        for (int yc = 0; yc < ha; ++yc)
          {
            const T aval = aptr[xa + yc*wa];
            TF* const cpos = cptr + yc*wb;
            if (aval != 0.0)
              for (int xc = 0; xc < wb; ++xc)
                cpos[xc] += aval * bpos[xc];
          }
      }

    return C;
  }

}

// ######################################################################
template <class T>
Image<typename promote_trait<T,T>::TP>
vmMult(const Image<T>& v, const Image<T>& M)
{
  return basic_vm_mul(v, M);
}

#ifdef HAVE_LAPACK

// specializations to use lapack functions if available

template <>
Image<double>
vmMult(const Image<double>& v, const Image<double>& M)
{
  return lapack::dgemv(&v, &M);
}

template <>
Image<float>
vmMult(const Image<float>& v, const Image<float>& M)
{
  return lapack::sgemv(&v, &M);
}

#endif // HAVE_LAPACK

// ######################################################################
template <class T>
Image<typename promote_trait<T,T>::TP>
matrixMult(const Image<T>& A, const Image<T>& B)
{
  return basic_mm_mul(A, B);
}

#ifdef HAVE_LAPACK

// specializations to use lapack functions if available

template <>
Image<double>
matrixMult(const Image<double>& A, const Image<double>& B)
{
  return lapack::dgemm(&A, &B);
}

template <>
Image<float>
matrixMult(const Image<float>& A, const Image<float>& B)
{
  return lapack::sgemm(&A, &B);
}

#endif // HAVE_LAPACK

// ######################################################################
template <class T>
Image<typename promote_trait<T,T>::TP>
matrixMult(const Image<T>& A, const Image<T>& B,
           const uint beginAX, const uint endAX,
           const uint beginBX, const uint endBX,
           const uint beginAY, const uint endAY)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(A.initialized()); ASSERT(B.initialized());
  ASSERT(beginAX <= endAX); ASSERT(beginBX <= endBX);
  ASSERT(beginAY <= endAY);
  const uint hb = B.getHeight(); ASSERT(hb == endAX - beginAX + 1);
  typedef typename promote_trait<T,T>::TP TF;

  Image<TF> C(endBX - beginBX, endAY - beginAY, ZEROS);
  typename Image<TF>::iterator pc = C.beginw();
  uint ya;

  for (uint yc = beginAY; yc < endAY; yc++)
    for (uint xc = beginBX; xc < endBX; xc++, ++pc)
      {
        ya = 0;
        for (uint xa = beginAX; xa < endAX; xa++ , ya++)
          *pc += A.getVal(xa, yc) * B.getVal(xc, ya);
      }
  return C;
}

// ######################################################################
template <class T_or_RGB>
Image<T_or_RGB> transpose(const Image<T_or_RGB>& M)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const int w = M.getWidth(), h = M.getHeight();
  Image<T_or_RGB> result(h, w, NO_INIT);
  typename Image<T_or_RGB>::const_iterator mptr, mptr2 = M.begin();
  typename Image<T_or_RGB>::iterator rptr = result.beginw();

  for (int y = 0; y < w; ++y)
    {
      mptr = mptr2;
      for (int x = 0; x < h; ++x)
        {
          *rptr = *mptr;
          ++rptr; mptr += w;
        }
      ++mptr2;
    }

  return result;
}

// ######################################################################
template <class T_or_RGB>
Image<T_or_RGB> flipHoriz(const Image<T_or_RGB>& img)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const int w = img.getWidth(), h = img.getHeight();
  Image<T_or_RGB> result(w, h, NO_INIT);

  typename Image<T_or_RGB>::const_iterator sptr = img.begin();
  typename Image<T_or_RGB>::iterator dptr = result.beginw() + w - 1;

  for (int y = 0; y < h; ++y)
    {
      for (int x = 0; x < w; ++x) *dptr-- = *sptr++;
      dptr += 2 * w;
    }

  return result;
}

// ######################################################################
template <class T_or_RGB>
Image<T_or_RGB> flipVertic(const Image<T_or_RGB>& img)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  const int w = img.getWidth(), h = img.getHeight();
  Image<T_or_RGB> result(w, h, NO_INIT);

  typename Image<T_or_RGB>::const_iterator sptr = img.begin();
  typename Image<T_or_RGB>::iterator dptr = result.beginw() + w * (h-1);

  for (int y = 0; y < h; ++y)
    {
      // NOTE: we may revert to a more object-safe implementation if
      // the need arises...
      memcpy(dptr, sptr, w * sizeof(T_or_RGB));
      sptr += w;  dptr -= w;
    }

  return result;
}

// ######################################################################
template <class T>
Image<T> eye(const uint size)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  Image<T> result(size, size, ZEROS);
  typename Image<T>::iterator rptr = result.beginw();
  T one(1);

  for (uint i = 0; i < size; ++i)
    {
      *rptr++ = one;
      rptr += size;
    }

  return result;
}

// ######################################################################
template <class T>
typename promote_trait<T,T>::TP trace(const Image<T>& M)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(M.isSquare());
  typename promote_trait<T,T>::TP result(0);
  typename Image<T>::const_iterator ptr = M.begin();
  const uint size = M.getWidth();

  for (uint i = 0; i < size; ++i)
    {
      result += *ptr++;
      ptr += size;
    }

  return result;
}

// ######################################################################
template <class T>
int matrixPivot(Image<T>& M, const int y)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(M.isSquare()); const int siz = M.getWidth();
  ASSERT(y < siz);

  // find the element with largest absolute value in the column y and
  // below row y:
  int bestj = siz+1; T maxi(0);
  for (int j = y; j < siz; j ++)
    {
      const T tmp = std::abs(M.getVal(y, j));
      if (tmp > maxi) { maxi = tmp; bestj = j; }
    }

  // if we did not find any non-zero value, let the caller know:
  if (bestj == siz + 1) return -1;

  // if our pivot is at the given y, do nothing and return that y:
  if (bestj == y) return y;

  T* M_arr = M.getArrayPtr();

  // otherwise, exchange rows y and bestj in M, and return bestj:
  for (int i = 0; i < siz; i ++)
    {
      std::swap(M_arr[i + bestj*siz],
                M_arr[i + y*siz]);
    }

  return bestj;
}

// ######################################################################
template <class T>
Image<typename promote_trait<T, float>::TP> matrixInv(const Image<T>& M)
{
GVX_TRACE(__PRETTY_FUNCTION__);

  // NOTE: In the future if we need a faster matrix inverse, we could
  // specialize for double and float to use dgetri() and sgetri(),
  // respectively, from lapack. This would be analogous to the
  // specializations that we currently have for matrix-matrix and
  // vector-matrix multiplication in this file, as well as the
  // specializations for svd that we have in LinearAlgebra.C.

  ASSERT(M.isSquare());
  typedef typename promote_trait<T, float>::TP TF;
  const int siz = M.getWidth();

  // get an identity matrix:
  Image<TF> res = eye<TF>(siz);

  // make a promoted copy of M that we can modify:
  Image<TF> m(M);

  // for efficiency, we'll now pull pointers to the raw storage of our
  // mutable M copy, and of our result matrix -- this way, we can use
  // array indexing without all of the ASSERT()s in Image::getVal()
  // and Image::setVal() -- this turns out to cut the run time by
  // about a factor of 8x
  TF* m_arr   = m.getArrayPtr();
  TF* res_arr = res.getArrayPtr();

  // ONLY USE THIS MACRO LOCALLY WITHIN THIS FUNCTION! It relies on
  // the local 'siz' variable, and on the fact that any array for
  // which it is called is a square matrix with dimensions siz*siz.
#define ARR_IDX(arrptr, i, j) ((arrptr)[(i) + ((j)*siz)])

  // let's pivote!
  for (int k = 0; k < siz; k ++)
    {
      // pivote at k:
      const int piv = matrixPivot(m, k);

      // could we pivote?
      if (piv == -1)
        throw SingularMatrixException(M, SRC_POS);

      // if we did pivote, then let's also exchange rows piv and k in temp:
      if (piv != k)
        for (int i = 0; i < siz; i ++)
          {
            std::swap(ARR_IDX(res_arr, i, piv),
                      ARR_IDX(res_arr, i, k));
          }

      // divide row k by our pivot value in both matrices:
      const TF val = 1.0F / ARR_IDX(m_arr, k, k);
      for (int i = 0; i < siz; i ++)
        {
          ARR_IDX(m_arr,   i, k)  *= val;
          ARR_IDX(res_arr, i, k)  *= val;
        }

      // make sure everybody else in that column becomes zero in the
      // original matrix:
      for (int j = 0; j < siz; j ++)
        if (j != k)
          {
            const TF v = ARR_IDX(m_arr, k, j);
            for (int i = 0; i < siz; i ++)
              {
                ARR_IDX(m_arr,   i, j)  -= ARR_IDX(m_arr,   i, k) * v;
                ARR_IDX(res_arr, i, j)  -= ARR_IDX(res_arr, i, k) * v;
              }
          }
    }
  return res;

#undef ARR_IDX
}

// ######################################################################
template <class T>
typename promote_trait<T,T>::TP dotprod(const Image<T>& A, const Image<T>& B)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(A.isSameSize(B));
  typename promote_trait<T,T>::TP result = 0;
  typename Image<T>::const_iterator aptr = A.begin(),
    stop = A.end(), bptr = B.begin();

  while (aptr != stop) result += (*aptr++) * (*bptr++);

  return result;
}

// ######################################################################
template <class T>
typename promote_trait<T, float>::TP matrixDet(const Image<T>& M)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(M.isSquare()); const int siz = M.getWidth();
  typedef typename promote_trait<T, float>::TP TF;

  // get a local copy we can modify:
  Image<TF> tmp = M;
  TF det = 1;

   for (int k = 0; k < siz; k++)
     {
       int idx = matrixPivot(tmp, k);
       if (idx == -1) return 0; // singular matrix
       if (idx != 0) det = -det;

       TF pv = tmp.getVal(k, k);
       det *= pv;
       for (int j = k+1; j < siz; j++)
         {
           TF piv = TF(tmp.getVal(k, j)) / pv;

           for (int i = k+1; i < siz; i++)
             tmp.setVal(i, j, tmp.getVal(i, j) - piv * tmp.getVal(i, k));
         }
     }
   return det;
}

// Include the explicit instantiations
#include "inst/Image/MatrixOps.I"

template Image<promote_trait<double, double>::TP> vmMult(const Image<double>&,const Image<double>&);
template Image<promote_trait<double, double>::TP> matrixMult(const Image<double>&,const Image<double>&);
template Image<double> transpose(const Image<double>&);
template Image<short> transpose(const Image<short> &);
template Image<unsigned short> transpose(const Image<unsigned short> &);
template promote_trait<double, double>::TP trace(const Image<double>&);
template Image<promote_trait<double, float>::TP> matrixInv(const Image<double>&);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

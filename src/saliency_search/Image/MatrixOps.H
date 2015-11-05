/*!@file Image/MatrixOps.H Matrix operations on Image
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/MatrixOps.H $
// $Id: MatrixOps.H 7963 2007-02-21 03:55:43Z itti $
//

#ifndef IMAGE_MATRIXOPS_H_DEFINED
#define IMAGE_MATRIXOPS_H_DEFINED

#include "Image/Image.H"
#include "Util/Promotions.H"
#include "rutz/error.h"

//! Exception class thrown if a singular matrix is encountered
/*! The exception object contains a copy of the singular matrix, so
    that the matrix can be examined, displayed, etc. for debugging
    purposes. */
class SingularMatrixException : public rutz::error
{
public:
  SingularMatrixException(const Image<float>& m,
                          const rutz::file_pos& pos)
    :
    rutz::error("Matrix is singular", pos),
    mtx(m) {}

  virtual ~SingularMatrixException() throw() {}

  const Image<float> mtx;
};

//! Vector-Matrix Multiplication: y = v * M
/*! @param v row vector (w x h == n x 1),
    @param M matrix with (w x h == p x n)
    @return row vector (w x h == p x 1)
*/
template <class T>
Image<typename promote_trait<T,T>::TP>
vmMult(const Image<T>& v, const Image<T>& M);

//! Matrix-Matrix Multiplication: C = A * B
/*! The images A and B are interpreted as matrices. For multiplying a
    (m x n) with a (n x p) matrix, this is the generic matrix
    multiplication with complexity (m * n * p).*/
template <class T>
Image<typename promote_trait<T,T>::TP>
matrixMult(const Image<T>& A, const Image<T>& B);

//! Matrix-Matrix Multiplication: C = A * B
/*! The images A and B are interpreted as matrices. For multiplying
  a (m x n) with a (n x p) matrix, this is the generic matrix multiplication
  with complexity (m * n * p). This version will allow you to multiply
  values in a matrix that lie between index endA/B and beginA/B. The
  destination matrix will be the same size as:
  (endAX - beginAX) x (endAX - beginAX)
  per usual the width of A (indexed between beginAX and endAX)
  and height of B MUST be equal. If not then
  perhaps use transpose...
  @param A input matrix 1
  @param B input matrix 2
  @param beginAX starting (width) index in matrix A
  @param endAX ending (width) index in matrix A
  @param beginBX starting (width) index in matrix B
  @param endBX ending (width) index in matrix B
  @param beginAY starting (height) index in matrix A
  @param endAY ending (height) index in matrix A
*/
template <class T>
Image<typename promote_trait<T,T>::TP>
matrixMult(const Image<T>& A, const Image<T>& B,
           const uint beginAX, const uint endAX,
           const uint beginBX, const uint endBX,
           const uint beginAY, const uint endAY);

//! transpose matrix M
template <class T_or_RGB>
Image<T_or_RGB> transpose(const Image<T_or_RGB>& M);

//! flip horizontally
template <class T_or_RGB>
Image<T_or_RGB> flipHoriz(const Image<T_or_RGB>& img);

//! flip vertically
template <class T_or_RGB>
Image<T_or_RGB> flipVertic(const Image<T_or_RGB>& img);

//! return the identity matrix of dimensions (size x size)
template <class T>
Image<T> eye(const uint size);

//! Compute the trace of a square matrix, i.e., sum of its diagonal elements
template <class T>
typename promote_trait<T,T>::TP trace(const Image<T>& M);

//! Get a partial pivot and pivote a square matrix, at y
/*! returns -1 if we can't find one */
template <class T>
int matrixPivot(Image<T>& M, const int y);

//! Inverse a nonsingular square matrix
template <class T>
Image<typename promote_trait<T, float>::TP> matrixInv(const Image<T>& M);

//! Compute dot product between two images
/*! The two images must have same dims. This is just the sum of all
  pointwise products. */
template <class T>
typename promote_trait<T,T>::TP dotprod(const Image<T>& A, const Image<T>& B);

//! Compute the determinant of a square matrix
template <class T>
typename promote_trait<T,float>::TP matrixDet(const Image<T>& M);


#endif // !IMAGE_MATRIXOPS_H_DEFINED

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

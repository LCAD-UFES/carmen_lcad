/*!@file Image/lapack.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/lapack.H $
// $Id: lapack.H 14618 2011-03-18 22:49:35Z lior $
//

#ifndef IMAGE_LAPACK_H_DEFINED
#define IMAGE_LAPACK_H_DEFINED

#include "Image/LinearAlgebraFlags.H"

template <class T> class Image;

#ifdef HAVE_LAPACK
namespace lapack
{
  /// Double-precision real svd using lapack's dgesdd()
  void svd(const Image<double>& A,
           Image<double>& U, Image<double>& S, Image<double>& V,
           const SvdFlag flags);

  /// Single-precision real svd using lapack's sgesdd()
  void svdf(const Image<float>& A,
            Image<float>& U, Image<float>& S, Image<float>& V,
            const SvdFlag flags);

  /// Double-precision vector-matrix multiplication using dgemm()
  /** NOTE: we take pointer arguments to ensure that there are no
      implicit conversions -- we want to use this function for double
      and only double */
  Image<double> dgemv(const Image<double>* v, const Image<double>* Mat);

  /// Single-precision vector-matrix multiplication using lapack's sgemm()
  /** NOTE: we take pointer arguments to ensure that there are no
      implicit conversions -- we want to use this function for float
      and only float */
  Image<float> sgemv(const Image<float>* v, const Image<float>* Mat);

  /// Double-precision matrix-matrix multiplication using lapack's dgemm()
  /** NOTE: we take pointer arguments to ensure that there are no
      implicit conversions -- we want to use this function for double
      and only double */
  Image<double> dgemm(const Image<double>* A, const Image<double>* B);

  /// Single-precision matrix-matrix multiplication using lapack's sgemm()
  /** NOTE: we take pointer arguments to ensure that there are no
      implicit conversions -- we want to use this function for float
      and only float */
  Image<float> sgemm(const Image<float>* A, const Image<float>* B);

  /// Double-precision Cholesky factorization
  /** NOTE: we take pointer arguments to ensure that there are no
      implicit conversions -- we want to use this function for float
      and only float */
  Image<double> dpotrf(const Image<double>* Mat);


  /// Double-precision determinant through LU decomposition
  /** NOTE: we take pointer arguments to ensure that there are no
      implicit conversions -- we want to use this function for double
      and only double */
  double det(const Image<double>* Mat);


}
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_LAPACK_H_DEFINED

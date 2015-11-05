/*!@file Image/LinearAlgebra.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/LinearAlgebra.H $
// $Id: LinearAlgebra.H 5744 2005-10-18 23:30:18Z rjpeters $
//

#ifndef IMAGE_LINEARALGEBRA_H_DEFINED
#define IMAGE_LINEARALGEBRA_H_DEFINED

#include "Image/LinearAlgebraFlags.H"
#include "Util/Promotions.H"

template <class T> class Image;

//! Compute the singular decomposition of M: M = U * S * V^T
/*! The U, S, V matrices have special properties:

    - U is column-orthogonal, such that (U^T * U) = I
    - S is square, diagonal, so that S is easily invertible
    - V is square and orthogonal, so (V^T * V) = (V * V^T) = I

    Note that these properties mean that the svd can be used to
    compute the pseudo-inverse of a general matrix M:

    - M = U*S*V^T
    - U^T*M = S*V^T  (since U is column-orthogonal)
    - S^-1*U^T*M = V^T  (since S is diagonal and invertible)
    - V*S^-1*U^T*M = I  (since V is square and orthogonal)

    by definition, pinv(M)*M = I, therefore

    - V*S^-1*U^T = pinv(M)

    (Note ^T is used to designate the matrix transpose, and ^-1 is
    used to designate the matrix inverse).

    You can use the routine svdPseudoInv to compute the pseudo-inverse
    using just this method.
*/
void svd(const Image<double>& M,
         Image<double>& U, Image<double>& S, Image<double>& V,
         SvdAlgo algo, const SvdFlag flags = 0);

//! Single-precision counterpart of svd()
void svdf(const Image<float>& M,
          Image<float>& U, Image<float>& S, Image<float>& V,
          SvdAlgo algo, const SvdFlag flags = 0);

//! Compute the pseudo-inverse using the method described in svd()
/*! Optionally throw out small values from the diagonal S matrix --
    this helps numerical stability.

    The number of eigenvalues retained is controlled by tolfactor. If
    tolfactor is less than 1.0, then it serves as a multiplier of the
    largest eigenvalue, below which any smaller eigenvalues are thrown
    out. If tolfactor is greater than 1.0, then the integer portion of
    tolfactor specifies the number of eigenvalues to be retained.

    If rank is a non-null pointer, then the rank of the diagonal
    matrix (after setting small values to zero) will be returned
    through that pointer. */
Image<double> svdPseudoInv(const Image<double>& x,
                           const SvdAlgo algo,
                           int* const rank = 0,
                           const double tolfactor = 1.0e-8);

//! Single-precision counterpart of svdPseudoInv()
Image<float> svdPseudoInvf(const Image<float>& x,
                           const SvdAlgo algo,
                           int* const rank = 0,
                           const float tolfactor = 1.0e-8f);

//! NOTE that this procedure is called "naive, unstable" for a reason!
/*! This procedure computes the pseudo-inverse using the direct
    formula pinv(M) = (M^T * M)^-1 * M^T. The formula is analytically
    correct, but in practice it can be numerically unstable. Indeed it
    might work just fine for your matrix, but to be safe you'll need
    to check the identity pinv(M)*M ~= I -- for example, do
    RMSerr(matrixMult(M,pinv(M)), eye<float>(M.getHeight())) and make
    sure that it's small (say, less than 0.01 or so). If you do get a
    large RMS error, then you can consider using svdPseudoInv()
    instead, which offers more numerical stability. */
template <class T>
Image<typename promote_trait<T, float>::TP>
naiveUnstablePseudoInv(const Image<T>& M);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_LINEARALGEBRA_H_DEFINED

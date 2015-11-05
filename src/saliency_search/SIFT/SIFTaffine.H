/*!@file SIFT/SIFTaffine.H Simple helper struct for affine transforms */

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
// Primary maintainer for this file: Philip Williams <plw@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/SIFTaffine.H $
// $Id: SIFTaffine.H 6005 2005-11-29 18:49:14Z rjpeters $
//

#ifndef SIFTAFFINE_H_DEFINED
#define SIFTAFFINE_H_DEFINED

#include "Util/sformat.H"
#include "SIFT/KeypointMatch.H"

#include <ostream>

//! A simple struct to store 2D affine transforms as used in the SIFT code
class SIFTaffine {
public:
  //! Constructor; initializes to identity
  inline SIFTaffine();

  //! Constructor
  inline SIFTaffine(const float mm1, const float mm2, const float mm3,
                    const float mm4, const float ttx, const float tty);

  //! Transform (x,y) into (u,v) using aff forward
  inline void transform(const float x, const float y,
                        float& u, float& v) const;

  //! Returns true if the affine can be inverted
  inline bool isInversible() const;

  //! Get the backward affine given a forward one
  inline SIFTaffine inverse() const;

  //! Decompose into rotation, scaling, and stretching components
  /*! This decomposition requires that the affine be inversible.  It
    is based on a polar decomposition as proposed in this excellent
    paper: 'Matrix animation and polar decomposition', Ken Shoemake
    and Tom Duff, Proceedings of the conference on Graphics interface
    '92, Vancouver, British Columbia, Canada, pp. 258-264. The
    resulting decomposition is such that:

    <PRE>

    [ m1 m2 ]   [ cos(theta) -sin(theta) ]   [ sx  str ]
    [ m3 m4 ] = [ sin(theta)  cos(theta) ] * [ str  sy ]

    </PRE>

    which you can also interpret as:

    <PRE>

    [ m1 m2 ]   [ cos(theta) -sin(theta) ]   [ sx  0 ]   [   1   str/sx ]
    [ m3 m4 ] = [ sin(theta)  cos(theta) ] * [ 0  sy ] * [ str/sy   1   ]

    </PRE>

    (if sx and sy are non-null), i.e.,

    <PRE>

    M = ROT(theta) * SCALE(sx, sy) * SHEAR(str/sx, str/sy)

    </PRE> */
  inline void decompose(float& theta, float& sx, float& sy,
                        float& str) const;

  //! Get residual distance squared for a given KeypointMatch
  /*! The return value is the squared distance between the coordinates
    of the ref Keypoint in the Keypointmatch, transformed by our
    affine transform, and the coordinates of the test Keypoint. Thus
    it is in squared units of the pixel size in the test image. */
  inline float getResidualDistSq(const KeypointMatch& km) const;

  //! Compose two affine transforms
  /*! We are the one that is applied first, other is applied second
    (i.e., to our results). Transforming by the resulting affine is
    like transforming first by us then by other. */
  inline SIFTaffine compose(const SIFTaffine& other) const;

  // our data members are public for easy access
  float m1, m2, m3, m4, tx, ty; //!< our contents
};

// ######################################################################
// SIFTaffine I/O:
// ######################################################################

//! Print a SIFTaffine to an ostream
inline std::ostream& operator<<(std::ostream& os, const SIFTaffine& a);





// ######################################################################
// ########## Implementation of inline functions
// ######################################################################
inline SIFTaffine::SIFTaffine() :
  m1(1.0F), m2(0.0F), m3(0.0F), m4(1.0F), tx(0.0F), ty(0.0F)
{ }

// ######################################################################
inline SIFTaffine::SIFTaffine(const float mm1, const float mm2,
                              const float mm3, const float mm4,
                              const float ttx, const float tty) :
  m1(mm1), m2(mm2), m3(mm3), m4(mm4), tx(ttx), ty(tty)
{ }

// ######################################################################
inline void SIFTaffine::transform(const float x, const float y,
                                  float& u, float& v) const
{
  u = m1 * x + m2 * y + tx;
  v = m3 * x + m4 * y + ty;
}

// ######################################################################
inline bool SIFTaffine::isInversible() const
{ return (fabsf(m1 * m4 - m2 * m3) > 1.0e-10F); }

// ######################################################################
inline SIFTaffine SIFTaffine::inverse() const
{
  // M^-1 is given by 1/det [ d -b; -c -a] with det = ad - bc
  const float det = m1 * m4 - m2 * m3;

  ASSERT(fabs(det) > 1.0e-10);

  SIFTaffine ret;
  ret.m1 =  m4 / det; ret.m2 = -m2 / det;
  ret.m3 = -m3 / det; ret.m4 =  m1 / det;

  ret.tx = - ret.m1 * tx - ret.m2 * ty;
  ret.ty = - ret.m3 * tx - ret.m4 * ty;

  return ret;
}

// ######################################################################
inline void SIFTaffine::decompose(float& theta, float& sx, float& sy,
                                  float& str) const
{
  float mm1 = m1, mm2 = m2, mm3 = m3, mm4 = m4;
  const float det = mm1 * mm4 - mm2 * mm3; ASSERT(fabs(det) > 1.0e-10);

  // make sure the determinant is positive:
  float fac = 1.0F;
  if (det < 0.0F)
    { mm1 = -mm1; mm2 = -mm2; mm3 = -mm3; mm4 = -mm4; fac = -1.0F; }

  // polar decomposition: M = QS where Q is a rotation and S a
  // symmetric non-negative matrix. In the 2x2 case, Q = M +
  // det(M).M^-T, scaled so that the columns have unit norm:
  float q1 = mm1 + mm4, q2 = mm2 - mm3; // note: q3 = -q2, q4 = q1;
  float coln = sqrtf(q1*q1 + q2*q2);
  q1 /= coln; q2 /= coln; fac /= coln;
  //LINFO("q1: %f q2:%f",q1,q2);

  // ok, q1 = cos(theta) while q2 = -sin(theta):
  theta = acosf(q1); // in [0 .. pi]
  //LINFO("theta: %f",theta);
  const float theta2 = asinf(-q2); // in [-pi/2 .. pi/2]
  //LINFO("theta2: %f",theta2);
  if (theta2 < 0.0F) theta = -theta;  // theta now in [-pi .. pi]

  // to get S, we just need to apply the inverse rotation, i.e.,
  // rotate M by -theta. Note that q1 = cos(-theta), q2 =
  // sin(-theta). After simplification, we obtain: sx = q1*mm1-q2*mm3,
  // sy= q2*mm2 + q1*mm4, and str=(mm1*mm2+mm3*mm4)*fac:
  sx = q1 * mm1 - q2 * mm3;
  sy = q1 * mm4 + q2 * mm2;
  str = (mm1 * mm2 + mm3 * mm4) * fac;

  //LINFO("fac=%f t2=%f t=%f sx=%f sy=%f str=%f",
  //     fac, theta2, theta, sx, sy, str);
}

// ######################################################################
inline std::ostream& operator<<(std::ostream& os, const SIFTaffine& a)
{
  os<<sformat("[testX]   [ %- .3f %- .3f ] [refX]   [%- .3f]",
              a.m1, a.m2, a.tx)<<std::endl;
  os<<sformat("[testY] = [ %- .3f %- .3f ] [refY] + [%- .3f]",
              a.m3, a.m4, a.ty)<<std::endl;
  return os;
}

// ######################################################################
inline float SIFTaffine::getResidualDistSq(const KeypointMatch& km) const
{
  // get the coords of the ref keypoint in the ref image:
  const float x = km.refkp->getX();
  const float y = km.refkp->getY();

  // get transformed coords of ref keypoint, now in the test image:
  float u, v; transform(x, y, u, v);

  // compute squared distance to the test keypoint:
  const float dx = km.tstkp->getX();
  const float dy = km.tstkp->getY();

  return (u-dx) * (u-dx) + (v-dy) * (v-dy);
}

// ######################################################################
inline SIFTaffine SIFTaffine::compose(const SIFTaffine& other) const
{
  // the resulting M is other.M * ours.M, the resulting T is other.M *
  // ours.T + other.T:
  SIFTaffine r;

  r.m1 = m1 * other.m1 + m2 * other.m3;
  r.m2 = m1 * other.m2 + m2 * other.m4;
  r.m3 = m3 * other.m1 + m4 * other.m3;
  r.m4 = m3 * other.m2 + m4 * other.m4;

  r.tx = other.m1 * tx + other.m2 * ty + other.tx;
  r.ty = other.m3 * tx + other.m4 * ty + other.ty;

  return r;
}

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

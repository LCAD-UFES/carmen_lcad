/*!@file SIFT/SIFTegomotion.C Calculates egomotion given a set of
  correspondence match */

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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/SIFTegomotion.C $
// $Id: SIFTegomotion.C 15310 2012-06-01 02:29:24Z itti $
//
#include "SIFT/SIFTegomotion.H"
#include "SIFT/VisualObject.H"
#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Raster/Raster.H"
#include "Image/LinearAlgebra.H"
#include "Image/MatrixOps.H" // for matrixMult(), matrixInv(), etc

// Solving Egomotion based on epipolar equation:
// (p^T)(v_hat)(p_dot) + (p^T)(S)(p) = 0    eqn.(1a) - for calibrated case
// (m^T)(W)(m_dot) + (m^T)(C)(m) = 0        eqn.(1b) - for uncalibrated case

// the diffference is the availability of A (the camera intrinsic parameter matrix)
// that is m = Ap
// below we will deal with the uncalibrated version with some simplification of A

// eqn.(1a) - for calibrated case
// p is a homogeneous coordinate [p1 p2 p3]^T, p3 = 1
// mdot is an instantaneous optical flow  [p1dot p2dot p3dot]^T, p3dot = 0
// S: 3 by 3 symmetric matrix (6 unknowns: s11, s12, s13, s22, s23, s23)
// v_hat: 3 by 3 anti-symmetric matrix (3 unknowns: v12, v13, v23)

// eqn.(1b) - for uncalibrated case
// m is a homogeneous coordinate [m1 m2 m3]^T, m3 = 1
// mdot is an instantaneous optical flow  [m1dot m2dot m3dot]^T, m3dot = 0
// C: 3 by 3 symmetric matrix (6 unknowns: c11, c12, c13, c22, c23, c23)
// W: 3 by 3 anti-symmetric matrix (3 unknowns: w12, w13, w23)

// both eqn(1a. and 1b.) can berearranged to  U*T = 0
// in equation below the solution uses the uncalibrated variable name
// with the following substitution to the calibrated version:
// p is equal to m
// S is equal to C
// v_hat is equal to W

  /* Ui = [
           m1*m1
           2*m1*m2
           2*m1*m3
           m2*m2
           2*m2*m3
           m3*m3
           m1*m2dot - m2*m1dot
           m1*m3dot - m3*m1dot
           m2*m3dot - m3*m2dot
          ]
     i = 0, 1, ..., 6
  */

  /*
    T = [
          c11
          c12
          c13
          c22
          c23
          c33
          w12
          w13
          w23
        ]
  */

// there is also an additional contraint
// (w^T)(C)(w) = 0            eqn.(2)
// w = [-w23 w13 -w12]^T
// we will deal with it after solving eqn(1) using SVD

// NOTE: the solution is still to a CONSTANT
// (whether it be negative or positive)

// ######################################################################
// get a magnitude of a vector
double getMag(Image<double> v)
{
  double mag = 0.0;
  for(int i = 0; i < v.getHeight(); i++)
      mag += v.getVal(0,i)*v.getVal(0,i);

  return sqrt(mag);
}

// ######################################################################
SIFTegomotion::SIFTegomotion(rutz::shared_ptr<VisualObjectMatch> match,
                             rutz::shared_ptr<CameraIntrinsicParam> cip,
                             rutz::shared_ptr<XWinManaged> matchWin) :
  itsVoMatch(match),
  itsCip(cip),
  itsMatchWin(matchWin)
{
  setCuMatch();
  calc();
}

// ######################################################################
void SIFTegomotion::setCuMatch()
{
  for(uint i = 0; i < itsVoMatch->size(); i++)
    {
      double u1vo = (*itsVoMatch)[i].refkp->getX();
      double v1vo = (*itsVoMatch)[i].refkp->getY();
      double u2vo = (*itsVoMatch)[i].tstkp->getX();
      double v2vo = (*itsVoMatch)[i].tstkp->getY();

      // some points are counted twice because of matches in
      // multiple scale-space
      bool isIdentical = 0;
      for(uint j = 0; j < itsCuMatch.size(); j++)
        {
          double u1cu = itsCuMatch[j].refkp->getX();
          double v1cu = itsCuMatch[j].refkp->getY();
          double u2cu = itsCuMatch[j].tstkp->getX();
          double v2cu = itsCuMatch[j].tstkp->getY();

          // if the coord. of ref and tst are within .05 in both axis
          if(fabs(u1vo - u1cu) < .05 &&
             fabs(v1vo - v1cu) < .05 &&
             fabs(u2vo - u2cu) < .05 &&
             fabs(v2vo - v2cu) < .05 )
            {
              // it is assumed identical
              isIdentical = 1;
              // LDEBUG("[%4d] DUP[%10.5f,%10.5f,%10.5f,%10.5f: %10.5f,%10.5f,%10.5f,%10.5f]",
              //      i, u1vo, v1vo, u2vo, v2vo, u1cu, v1cu, u2cu, v2cu);
            }
        }

      if(!isIdentical)
        {
          //LDEBUG("[%4d]    [%10.5f,%10.5f,%10.5f,%10.5f]",i, u1vo, v1vo, u2vo, v2vo);
          itsCuMatch.push_back((*itsVoMatch)[i]);
        }
    }

  LINFO("there are %" ZU " unique coordinate (%" ZU " duplicates)",
        itsCuMatch.size(), itsVoMatch->size() - itsCuMatch.size());
  //Raster::waitForKey();
}

// ######################################################################
Image<double> SIFTegomotion::getU(int nPoint)
{
  uint size;
  if (nPoint == -1)  size = itsCuMatch.size();
  else               size = nPoint;

  Image<double> U(9,size,ZEROS);

  int w = itsMatchWin->getDims().w()/2;
  int h = itsMatchWin->getDims().h()/2;
  Image< PixRGB<byte> > img(w,2*h,ZEROS);

  // if the camera calibration params are available
  Image<double> A; Image<double> Ainv;
  if(itsCip.is_valid())
    {
      A    = itsCip->getCameraMatrix();
      Ainv = matrixInv(A);
      print(A,    "Camera Matrix");
      print(Ainv, "Inverse Camera Matrix");
    }

  LINFO("there are %d points in the match", itsVoMatch->size());
  //Raster::waitForKey();
  Image< PixRGB<byte> > rImg = itsVoMatch->getVoRef()->getImage();
  Image< PixRGB<byte> > tImg = itsVoMatch->getVoTest()->getImage();
  for(uint i = 0; i < size; i++)
    {
      // default to uncalibrated first for visual purposes
      double tU = itsCuMatch[i].refkp->getX();  // m1
      double tV = itsCuMatch[i].refkp->getY();  // m2
      double rU = itsCuMatch[i].tstkp->getX();
      double rV = itsCuMatch[i].tstkp->getY();
      double rUdot = tU - rU;   // m1dot
      double rVdot = tV - rV;   // m2dot
      LDEBUG("UNCALIB:  (%10.5f,%10.5f) & (%10.5f,%10.5f) -> (%10.5f,%10.5f)",
             rU, rV, tU, tV, rUdot, rVdot);

      Point2D<int> locR(int(rU + 0.5F), int(rV + 0.5F));
      drawDisk(rImg, locR, 2, PixRGB<byte>(255,0,0));
      Point2D<int> locT(int(tU + 0.5F), int(tV + 0.5F));
      drawDisk(tImg, locT, 2, PixRGB<byte>(255,0,0));

      drawLine(rImg, locR, locT, PixRGB<byte>(255,255,0));

      // after the visuals we can calculate the calibrated values
      if(itsCip.is_valid())
        {
          Image<double> mR(1,3,ZEROS);
          mR.setVal(0, 0, rU); mR.setVal(0, 1, rV); mR.setVal(0, 2, 1.0);
          Image<double> pR = matrixMult(Ainv, mR);
          //print(mR,"mR"); print(pR,"pR");
          rU = pR.getVal(0,0);  // p1
          rV = pR.getVal(0,1);  // p2

          Image<double> mT(1,3,ZEROS);
          mT.setVal(0, 0, tU); mT.setVal(0, 1, tV); mT.setVal(0, 2, 1.0);
          Image<double> pT = matrixMult(Ainv, mT);
          tU = pT.getVal(0,0);
          tV = pT.getVal(0,1);

          rVdot = tU - rU;   // p1dot
          rUdot = tV - rV;   // p2dot

          LDEBUG("CALIB:    (%10.5f,%10.5f) & (%10.5f,%10.5f) -> (%10.5f,%10.5f)",
                 rU, rV, tU, tV, rUdot, rVdot);
        }

      // we add m3 = 1.0, m3dot = 0.0 below
      double m1    = rU;
      double m2    = rV;
      double m3    = 1.0f;
      double m1dot = rUdot;
      double m2dot = rVdot;
      double m3dot = 0.0f;

      U.setVal(0, i, m1*m1);
      U.setVal(1, i, 2*m1*m2);
      U.setVal(2, i, 2*m1*m3);
      U.setVal(3, i, m2*m2);
      U.setVal(4, i, 2*m2*m3);
      U.setVal(5, i, m3*m3);
      U.setVal(6, i, m1*m2dot - m2*m1dot);
      U.setVal(7, i, m1*m3dot - m3*m1dot);
      U.setVal(8, i, m2*m3dot - m3*m2dot);

    }

  inplacePaste(img, rImg,   Point2D<int>(0, 0));
  inplacePaste(img, tImg,   Point2D<int>(0, h));
  itsMatchWin->drawImage(img,w,0);
  //Raster::waitForKey();

  return U;
}

// ######################################################################
void SIFTegomotion::calc()
{
  uint techNum = 0;

  if(techNum == 0)
    leastSquaresAlgebraic();
  else
    sevenPointAlgorithm();
}

// ######################################################################
void SIFTegomotion::leastSquaresAlgebraic()
{
  // prepare the matrices
  Image<double> U = getU(); print(U, "U");
  Image<double> A = matrixMult(transpose(U), U); print(A, "A");
  //Raster::waitForKey();

  // perform SVD on A; A = [V1][D][V2]T
  // A is [9 x 9] matrix
  Image<double> V1;
  Image<double>  D;
  Image<double> V2;
  svd(A, V1, D, V2, SVD_LAPACK, SVD_FULL);
  print(V1,"V1"); print(D, "D"); print(V2, "V2");
  //Raster::waitForKey();

  // the least sqaures answer is the last column of V2
  Image<double> T = crop(V2, Point2D<int>(8,0), Dims(1, 9)); print(T, "T");
  //print(transpose(T),"Tt");print(transpose(U),"Ut");
  Image<double> ans = matrixMult(U,T); print(ans,"ans");
  //Raster::waitForKey();

  //T = constraint2(T);
  itsVel   = getVel(T);
  itsOmega = getOmega(T);
  print(T, "T");  print(itsVel, "vel");  print(itsOmega, "omega");
  //Raster::waitForKey();
}

// ######################################################################
// Seven Point Estimator Algorithm
// for minimizing the Differential Epipolar Constraint
void SIFTegomotion::sevenPointAlgorithm()
{
  // prepare the matrices
  // get seven random points from the correspondence
  Image<double> U = getU(7); print(U, "U");

  // perform SVD on U; U = [V1][D][V2]T
  Image<double> V1;
  Image<double>  D;
  Image<double> V2;
  svd(U, V1, D, V2, SVD_LAPACK, SVD_FULL);
  print(V1,"V1"); print(D, "D"); print(V2, "V2");

  uint zeig = 0;
  for(uint i = 0; i < 7; i++)
    {
      LINFO("eig[%d] = %20.10f",i+1,D.getVal(i,i));
      if(D.getVal(i,i) < .0000001) zeig++;
    }
  if( zeig > 0)
    {
      LINFO("> 2 null spaces; points are ill conditioned");
      Raster::waitForKey();
    }

  // get the last two columns T1 and T2 out of V1
  // the null space of U, solve UT = 0
  Image<double> T1 = crop(V1, Point2D<int>(7,0), Dims(1, 9)); print(T1, "T1");
  Image<double> T2 = crop(V1, Point2D<int>(8,0), Dims(1, 9)); print(T2, "T2");

  // test if it works
  //Image<double> UT1 = matrixMult(transpose(U), T1); print(UT1, "UT1");
  //Image<double> UT2 = matrixMult(transpose(U), T2); print(UT2, "UT2");

  // solve to take account constraint eqn.(2)
  Image<double> T =  constraint2(T1,T2);  print(T,"T");
  checkConstraint2(U, T);
}

// ######################################################################
// check if eqn(2) constraint is fulfilled
void SIFTegomotion::checkConstraint2(Image<double> U, Image<double> T)
{
  Image<double> Wtemp(1,3,ZEROS);
  Image<double> Ctemp(3,3,ZEROS);
  Wtemp.setVal(0,0, -T.getVal(0,8));
  Wtemp.setVal(0,1,  T.getVal(0,7));
  Wtemp.setVal(0,2, -T.getVal(0,6));

  Ctemp.setVal(0,0, T.getVal(0,0));
  Ctemp.setVal(0,1, T.getVal(0,1));
  Ctemp.setVal(0,2, T.getVal(0,2));
  Ctemp.setVal(1,0, T.getVal(0,1));
  Ctemp.setVal(1,1, T.getVal(0,3));
  Ctemp.setVal(1,2, T.getVal(0,4));
  Ctemp.setVal(2,0, T.getVal(0,2));
  Ctemp.setVal(2,1, T.getVal(0,4));
  Ctemp.setVal(2,2, T.getVal(0,5));
  print(Ctemp,"Ctemp"); print(Wtemp,"Wtemp");
  Image<double> wCw = matrixMult(matrixMult(transpose(Wtemp), Ctemp), Wtemp);
  print(wCw, "wCw");
}

// ######################################################################
Image<double> SIFTegomotion::constraint2(Image<double> T1)
{
  /*  T1.setVal(0,0, 3.0);
  T1.setVal(0,1, 4.0);
  T1.setVal(0,2, 5.0);
  T1.setVal(0,3, 3.0);
  T1.setVal(0,4, 1.0);
  T1.setVal(0,5, 5.0);
  T1.setVal(0,6, 7.0);
  T1.setVal(0,7, 6.0);
  T1.setVal(0,8, 2.0);
  LINFO("set T1");
  Raster::waitForKey();*/

  Image<double> W_hat = getW(T1); print(W_hat, "W_hat");
  Image<double> C_hat = getC(T1); print(C_hat, "C_hat");
  Raster::waitForKey();

  Image<double> I(3,3, ZEROS);
  I.setVal(0,0, 1.0);I.setVal(1,1, 1.0);I.setVal(2,2, 1.0);
  Image<double> omega = getOmega(T1); print(omega, "omega");
  Raster::waitForKey();

  double magOmega = 1.0;//getMag(omega);
  LINFO("magOmega: %f",magOmega);
  Image<double> P = I - matrixMult(W_hat,W_hat) * magOmega; print(P, "P");
  Raster::waitForKey();

  Image<double> C = C_hat - matrixMult(matrixMult(P,C_hat),P); ;
  Image<double> W = W_hat;
  double magT = getMag(getThetaCW(C,W));
  W = W/magT;
  C = C/magT;
  Image<double> T = getThetaCW(C,W);
      Raster::waitForKey();

  return T;
}

// ######################################################################
Image<double> SIFTegomotion::constraint2(Image<double> T1, Image<double> T2)
{
  /*
    // the solution T is linear combination of T1 and T2
    T_i = a * T1_i + (1 - a) * T2_i
        = T2_i + a(T1_i - T2_i)
    solve for 'a'
    using the constraint eqn.(2), in algebraic form
        w23*w23*c11 - w13*w23*c12 + w12*w23*c13 +
       -w23*w13*c12 + w13*w13*c22 - w12*w13*c23 +
        w23*w12*c13 - w13*w12*c23 + w12*w12*c33 = 0
    Substitute T_i to the equation, e.g. w23 =  T_8

    we have to go one term at a time
    the form is:
    T_i*T_j*T_k =

    (T2_i + a(T1_i - T2_i))*(T2_j + a(T1_j - T2_j))*(T2_k + a(T1_k - T2_k)) =

    (aa   + a*bb          )*(cc   + a*dd          )*(ee   + a*ff          ) =

    ((aa*cc) + a*(aa*dd) + a*(bb*cc) + a^2*(bb*dd))*(ee   + a*ff          ) =
    ((aa*cc*ee) + a*(aa*cc*ff) + a*(aa*dd*ee) + a^2*(aa*dd*ff) +
      a*(bb*cc*ee) + a^2*(bb*cc*ff) + a^2*(bb*dd*ee) + a^3*(bb*dd*ff)) =
    ( a^3*(bb*dd*ff                      ) +
      a^2*(aa*dd*ff + bb*cc*ff + bb*dd*ee) +
      a  *(aa*cc*ff + aa*dd*ee + bb*cc*ee) +
          (aa*cc*ee                      ) ) =
    p_i*a^3 + q_i*a^2 + r_i*a + s_i
  */
  Image<double> T(1, 9, ZEROS);

  // after accumulating them we end up with the realization that
  // eqn.(2) is a cubic constraint: p + q*a + r*a^2 + s*a^3 = 0
  double p = 0.0, q = 0.0, r = 0.0, s = 0.0;
  double aa, bb, cc, dd, ee, ff;

  // we'll use indI, indJ,indK for theta indexes i,j, and k, respectively
  // we have to note the sign in front of the term as well
  uint indI, indJ, indK; double sign;



//   T2.setVal(0,0, 5.0);
//   T2.setVal(0,1, 2.0);
//   T2.setVal(0,2, 9.0);

//   T2.setVal(0,3, 4.0);
//   T2.setVal(0,4, 7.0);
//   T2.setVal(0,5, 3.0);

//   T2.setVal(0,6, 4.0);
//   T2.setVal(0,7, 8.0);
//   T2.setVal(0,8, 4.0);
//  print(T1, "T1"); print(T2, "T2");
  for(uint i = 0; i < 9; i++)
    {
      if(i == 0)
        {
          // term 1: w23*w23*c11 = T_i*T_j*T_k, i = 8, j = 8, k = 0
          //         sign: positive
          indI = 8, indJ = 8, indK = 0; sign = 1.0;
        }
      else if(i == 1)
        {
          // term 2:-w13*w23*c12 = T_i*T_j*T_k, i = 7, j = 8, k = 1
          //         sign: negative
          indI = 7, indJ = 8, indK = 1; sign = -1.0;
        }
      else if(i == 2)
        {
          // term 3: w12*w23*c13 = T_i*T_j*T_k, i = 6, j = 8, k = 2
          //         sign: positive
          indI = 6, indJ = 8, indK = 2; sign = 1.0;
        }
      else if(i == 3)
        {
          // term 4:-w23*w13*c12 = T_i*T_j*T_k, i = 8, j = 7, k = 1
          //         sign: negative
          indI = 8, indJ = 7, indK = 1; sign = -1.0;
        }
      else if(i == 4)
        {
          // term 5: w13*w13*c22 = T_i*T_j*T_k, i = 7, j = 7, k = 3
          //         sign: positive
          indI = 7, indJ = 7, indK = 3; sign = 1.0;
        }
      else if(i == 5)
        {
          // term 6:-w12*w13*c23 = T_i*T_j*T_k, i = 6, j = 7, k = 4
          //         sign: negative
          indI = 6, indJ = 7, indK = 4; sign = -1.0;
        }
      else if(i == 6)
        {
          // term 7: w23*w12*c13 = T_i*T_j*T_k, i = 8, j = 6, k = 2
          //         sign: positive
          indI = 8, indJ = 6, indK = 2; sign = 1.0;
        }
      else if(i == 7)
        {
          // term 8:-w13*w12*c23 = T_i*T_j*T_k, i = 7, j = 6, k = 4
          //         sign: negative
          indI = 7, indJ = 6, indK = 4; sign = -1.0;
        }
      else
        {
          // term 9: w12*w12*c33 = T_i*T_j*T_k, i = 6, j = 6, k = 5
          //         sign: positive
          indI = 6, indJ = 6, indK = 5; sign = 1.0;
        }

      aa = T2.getVal(0, indI);
      bb = T1.getVal(0, indI) - T2.getVal(0, indI);
      cc = T2.getVal(0, indJ);
      dd = T1.getVal(0, indJ) - T2.getVal(0, indJ);
      ee = T2.getVal(0, indK);
      ff = T1.getVal(0, indK) - T2.getVal(0, indK);

      p+= sign * (bb*dd*ff);
      q+= sign * (aa*dd*ff + bb*cc*ff + bb*dd*ee);
      r+= sign * (aa*cc*ff + aa*dd*ee + bb*cc*ee);
      s+= sign * (aa*cc*ee);

      LINFO(" %f*alpha^3 + %f*alpha^2 + %f*alpha + %f",
            sign * (bb*dd*ff),
            sign * (aa*dd*ff + bb*cc*ff + bb*dd*ee),
            sign * (aa*cc*ff + aa*dd*ee + bb*cc*ee),
            sign * (aa*cc*ee));

      LINFO("%d:[%d,%d,%d] (%f,%f,%f,%f,%f,%f) sign:(%f) p:%f, q:%f r:%f s:%f",
            i, indI, indJ, indK,
            aa, bb, cc, dd, ee, ff, sign, p,q,r,s);
    }

  // cubic constraint has either 1 or 3 real solutions
  std::vector<std::complex<double> > alpha;
  uint numSoln = solveCubic(p, q, r, s, alpha);
  LINFO("Number of real solutions: %d",numSoln);
  LINFO("X1 = %f + %fi, X2 = %f + %fi, X3 = %f + %fi",
        alpha[0].real(), alpha[0].imag(),
        alpha[1].real(), alpha[1].imag(),
        alpha[2].real(), alpha[2].imag() );

  // get the first solution for now
  for(uint ns = 0; ns < numSoln; ns++)
    {
      // calculate the theta
      for(uint i= 0; i < 9; i++)
        T.setVal(0,i, (alpha[ns].real()*T1.getVal(0,i) +
                       (1.0 - alpha[ns].real())*T2.getVal(0,i)));

      // get the camera motion velocities
      if(itsCip.is_valid())
        {
          itsVel   = getVel(T);
          itsOmega = getOmega(T);
        }
      else
        getUncalibCW(T);
    }
  Raster::waitForKey();

  return T;
}
// ######################################################################
Image<double> SIFTegomotion::getThetaCW(Image<double> C, Image<double> W)
{
  Image<double> T(1,9, ZEROS);

  T.setVal(0,0, C.getVal(0,0));
  T.setVal(0,1, C.getVal(0,1));
  T.setVal(0,2, C.getVal(0,2));
  T.setVal(0,3, C.getVal(1,1));
  T.setVal(0,4, C.getVal(1,2));
  T.setVal(0,5, C.getVal(2,2));

  T.setVal(0,6, W.getVal(1,0));
  T.setVal(0,7, W.getVal(2,0));
  T.setVal(0,8, W.getVal(2,1));

  return T;
}

// ######################################################################
Image<double> SIFTegomotion::getC(Image<double> T)
{
  Image<double> C(3,3, ZEROS);
  C.setVal(0,0,  T.getVal(0,0));
  C.setVal(0,1,  T.getVal(0,1));
  C.setVal(0,2,  T.getVal(0,2));

  C.setVal(1,0,  T.getVal(0,1));
  C.setVal(1,1,  T.getVal(0,3));
  C.setVal(1,2,  T.getVal(0,4));

  C.setVal(2,0,  T.getVal(0,2));
  C.setVal(2,1,  T.getVal(0,4));
  C.setVal(2,2,  T.getVal(0,5));
  return C;
}

// ######################################################################
Image<double> SIFTegomotion::getW(Image<double> T)
{
  Image<double> W(3,3, ZEROS);
  W.setVal(1,0,  T.getVal(0,6));
  W.setVal(2,0,  T.getVal(0,7));
  W.setVal(2,1,  T.getVal(0,8));
  W.setVal(0,1, -T.getVal(0,6));
  W.setVal(0,2, -T.getVal(0,7));
  W.setVal(1,2, -T.getVal(0,8));
  return W;
}

// ######################################################################
Image<double> SIFTegomotion::getThetaVelOmega
  (Image<double> vel, Image<double> omega)
{
  Image<double> T(1,9, ZEROS);
  /*
  Image<double> T(1,9, ZEROS);
  T.setVal(0,6, C.getVal(0,));
  T.setVal(0,7, C.getVal(0,));
  T.setVal(0,8, C.getVal(0,));

  Image<double> CW = matrixMult(C,W); // or S
  T.setVal(0,0, CW.getVal(0,0));
  T.setVal(0,1, CW.getVal(0,1));
  T.setVal(0,2, CW.getVal(0,2));
  T.setVal(0,3, CW.getVal(1,1));
  T.setVal(0,4, CW.getVal(1,2));
  T.setVal(0,5, CW.getVal(2,2));
  */
  return T;
}

// ######################################################################
Image<double> SIFTegomotion::getVel(Image<double> T)
{
  // in eqn(1b) S (or C in the uncalibrated version):
  // v_hat is the translational velocity of the camera in anti-symmetric form
  double v1 = -T.getVal(0,8); // -w23
  double v2 =  T.getVal(0,7); //  w13
  double v3 = -T.getVal(0,6); // -w12

  LINFO("translational velocity (%f,%f,%f)", v1, v2, v3);
  Image<double> vel(1,3,ZEROS);
  vel.setVal(0,0, v1);
  vel.setVal(0,1, v2);
  vel.setVal(0,2, v3);
  return vel;
}

// ######################################################################
Image<double> SIFTegomotion::getOmega(Image<double> T)
{
  // in eqn(1b) S (or C in the uncalibrated version):
  // v_hat is the translational velocity of the camera in anti-symmetric form
  double v1 = -T.getVal(0,8); // -w23
  double v2 =  T.getVal(0,7); //  w13
  double v3 = -T.getVal(0,6); // -w12

  // in eqn(1b) S (or C in the uncalibrated version) = (v_hat)(w_hat)
  // v_hat is known, while w_hat is devivable to:
  // S = | s11 s12 s13 | = (v_hat)(w_hat) =
  //     | s12 s22 s23 |
  //     | s13 s23 s33 |
  //
  // |  0 -v3  v2 ||  0 -w3  w2 | = |(-v3w3 - v2w2)   v2w1            v3w1        |
  // | v3   0 -v1 || w3   0 -w1 |   | v1w2           ( -v3w3 - v1w1)  v3w2        |
  // |-v2  v1   0 ||-w2  w1   0 |   | v1w3            v2w3          (-v2w2 - v1w1)|
  //
  // note: wX = omegaX

  double omega1 = 0.0, omega2 = 0.0, omega3 = 0.0;
  if(v1 == 0.0 && v2 == 0.0 && v3 == 0.0)
    {
      // if the camera is stationary we can only assume
      // that the angle does not change
    }
  else if(v1 == 0.0 && v2 == 0.0)
    {
      omega1 =  T.getVal(0,2)/v3; // s13/v3
      omega2 =  T.getVal(0,4)/v3; // s23/v3
      omega3 = -T.getVal(0,3)/v3; // s22/v3
    }
  else if(v1 == 0.0 && v3 == 0.0)
    {
      omega1 =  T.getVal(0,1)/v2; // s12/v2
      omega2 = -T.getVal(0,0)/v2; // s11/v2
      omega3 =  T.getVal(0,4)/v2; // s23/v2
    }
  else if(v2 == 0.0 && v3 == 0.0)
    {
      omega1 = -T.getVal(0,5)/v1; // s33/v1
      omega2 =  T.getVal(0,1)/v1; // s12/v1
      omega3 =  T.getVal(0,2)/v1; // s13/v1
    }
  else if(v1 == 0.0)
    {
      omega1 =  T.getVal(0,1)/v2; // s12/v2
      omega2 =  T.getVal(0,4)/v3; // s23/v3
      omega3 =  T.getVal(0,3)/v2; // s23/v2
    }
  else if(v2 == 0.0)
    {
      omega1 =  T.getVal(0,2)/v3; // s13/v3
      omega2 =  T.getVal(0,4)/v3; // s23/v3
      omega3 =  T.getVal(0,3)/v1; // s13/v1
    }
  else
    {
      omega1 =  T.getVal(0,1)/v2; // s12/v2
      omega2 =  T.getVal(0,1)/v1; // s12/v1
      omega3 =  T.getVal(0,2)/v1; // s13/v3
    }

  LINFO("angular velocity: (%f,%f,%f)", omega1, omega2, omega3);
  Image<double> omega(1,3,ZEROS);
  omega.setVal(0,0, omega1);
  omega.setVal(0,1, omega2);
  omega.setVal(0,2, omega3);
  return omega;
}

// ######################################################################
void SIFTegomotion::getUncalibCW(Image<double> T)
{
  // extract the components of W and C
  // from eqn.(1) differential Epipolar Constraint
  // we now can extract the camera motion up to a scalar
  // angular change (3 vals)
  // translational change (2 vals)
  double c11 = T.getVal(0,0);
  double c12 = T.getVal(0,1);
  double c13 = T.getVal(0,2);
  double c22 = T.getVal(0,3);
  double c23 = T.getVal(0,4);
  double c33 = T.getVal(0,5);
  double w12 = T.getVal(0,6); // = w1
  double w13 = T.getVal(0,7); // = w2
  double w23 = T.getVal(0,8); // = w3
  double w1  = w12;
  double w2  = w13;
  double w3  = w23;

  //
  double delta1 = (2*c12*w2 - (c22 - c11)*w1) / (w1*w1 + w2*w2);
  double delta2 = (2*c12*w1 + (c22 - c11)*w2) / (w1*w1 + w2*w2);
  double delta3 = (c11*w1*w1 + 2*c12*w1*w2 + c22*w2*w2) / (w3*(w1*w1 + w2*w2));
  double psi = (w1*w1 + w2*w2 + w3*w3)*(w2*delta1 + w2*delta2);
  double d1 = 2*c13 + w1*delta3;
  double d2 = 2*c23 + w2*delta3;
  double d3 = c33;
  double delta4 = - d3 / (w1*delta1 + w3*delta2);
  double delta5 = (1.0/psi) * ((w1*w2*delta1 + (w2*w2 + w3*w3)*delta2)*d1
                               - ((w1*w1 + w3*w3)*delta1 + w1*w2*delta2)*d2
                               + (w2*w3*delta1 - w1*w3*delta2)*d3);

  LINFO("d1: %f, d2: %f, d3: %f", d1, d2, d3);
  LINFO("psi: %f",psi);
  LINFO("delta:(1: %f, 2: %f, 3: %f, 4:%f, 5:%f",
        delta1, delta2, delta3, delta4, delta5);

  double flen   = sqrt(delta4);
  double omega1 = delta1 * sqrt(flen);
  double omega2 = delta2 * sqrt(flen);
  double omega3 = delta3;
  double flen_dot = delta5 * sqrt(flen);

  double v1 = -w1 / flen;
  double v2 = -w2 / flen;
  double v3 = w3;

  LINFO("focal length: %f, flen_dot: %f",flen, flen_dot);
  LINFO("angular velocity: (%f,%f,%f)", omega1, omega2, omega3);
  LINFO("translational velocity (%f,%f,%f)", v1, v2, v3);
}

// ######################################################################
// sign of number
double sgn(double x)
{
  if (x < 0.0) return -1.0;
  return 1.0;
}

// ######################################################################
uint SIFTegomotion::solveCubic(double p, double q, double r, double s,
                               std::vector<std::complex<double> >& x)
{
  // compute real or complex roots of cubic polynomial
  // x^3 + A*x^2 + B*x + C = 0
  // roots are X1, X2, and X3
  // try A = -11.0, B = 49.0, C = -75.0,
  //     ans: x1 = 3, x2 = 4+3i, x3 = 4-3i

  // Credit due to:  Stephen R. Schmitt
  // http://home.att.net/~srschmitt/script_exact_cubic.html

  double A,B,C;
  if(p != 0.0)
    {
      A = q/p;
      B = r/p;
      C = s/p;
    }
  else if(q != 0.0)
    {
      // this is a quadratic formula
      // x = (-r + sqrt(r^2 -4 qs)/2q
      // x = (-r - sqrt(r^2 -4 qs)/2q

      double temp = r*r - 4.0*q*s;
      if(temp == 0.0)
        {
          x.push_back(std::complex<double>(-r/(2.0*q), 0.0));
          return 1;
        }
      else if(temp > 0.0)
        {
          x.push_back(std::complex<double>((-r + sqrt(temp))/(2.0*q), 0.0));
          x.push_back(std::complex<double>((-r - sqrt(temp))/(2.0*q), 0.0));
          return 2;
        }
      else
        {
          x.push_back(std::complex<double>(-r/(2.0*q),  sqrt(-temp)/(2.0*q)));
          x.push_back(std::complex<double>(-r/(2.0*q), -sqrt(-temp)/(2.0*q)));
          return 0;
        }
    }
  else if(r != 0.0)
    {
      // this is rx + s = 0
      x.push_back(std::complex<double>(-s/r, 0.0));
      return 1;
    }
  else
    // this has to be s = 0
    return 0;

  double X1, X2, X3, Im;
  double Q, R, D, S, T;

  Q = (3*B - (A*A))/9.0;
  R = (9*A*B - 27*C - 2*(A*A*A))/54.0;
  D = (Q*Q*Q) + (R*R);    // polynomial discriminant

  if (D >= 0)                                 // complex or duplicate roots
  {
    S = sgn(R + sqrt(D)) * pow(fabs(R + sqrt(D)),(1.0/3.0));
    T = sgn(R - sqrt(D)) * pow(fabs(R - sqrt(D)),(1.0/3.0));

    X1 = -A/3.0 + (S + T);                    // real root
    X2 = -A/3.0 - (S + T)/2.0;                  // real part of complex root
    X3 = -A/3.0 - (S + T)/2.0;                  // real part of complex root
    Im = fabs(sqrt(3)*(S - T)/2.0);           // complex part of root pair
  }
  else                                        // distinct real roots
  {
    double th = acos(R/sqrt(-pow(Q, 3)));

    X1 = 2*sqrt(-Q) * cos(th/3.0) - A/3.0;
    X2 = 2*sqrt(-Q) * cos((th + 2*M_PI)/3) - A/3.0;
    X3 = 2*sqrt(-Q) * cos((th + 4*M_PI)/3) - A/3.0;
    Im = 0.0;
  }

  // 0.0 if real roots
  if (Im == 0.0)                              // real roots
  {
    x.push_back(std::complex<double>(X1, 0.0));
    x.push_back(std::complex<double>(X2, 0.0));
    x.push_back(std::complex<double>(X3, 0.0));
    LDEBUG("X1 = %f, X2 = %f, X3 = %f", X1, X2, X3);
    return 3;
  }
  else                                        // real and complex pair
  {
    x.push_back(std::complex<double>(X1, 0.0));
    x.push_back(std::complex<double>(X2,  Im));
    x.push_back(std::complex<double>(X3, -Im));
    LDEBUG("X1 = %f, X2 = %f + %fi, X3 = %f - %fi", X1, X2, Im, X3, Im);
    return 1;
  }
}

// ######################################################################
void SIFTegomotion::print(Image<double> img, const std::string& name)
{
//   LINFO("%s:  %d by %d", name.c_str(), img.getWidth(),  img.getHeight());
//   for(int j = 0; j < img.getHeight(); j++)
//     {
//       for(int i = 0; i < img.getWidth(); i++)
//         {
//           printf("%13.5f ", img.getVal(i,j));
//         }
//       printf("\n");
//     }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

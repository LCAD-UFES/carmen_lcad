/*!@file Image/KalmanFilter.C implementation of a 2nd order linear Kalman Filter
 */
// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2002   //
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
// Primary maintainer for this file: Dirk Walther <walther@caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/KalmanFilter.C $
// $Id: KalmanFilter.C 4899 2005-07-12 20:21:11Z itti $
//

#include "Image/KalmanFilter.H"

#include "Image/IO.H"
#include "Image/MathOps.H"
#include "Image/MatrixOps.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include <cmath>
#include <iostream>

// ######################################################################
inline float sqr(float x) {
  return (x * x);
}

// ######################################################################
KalmanFilter::KalmanFilter()
  : initialized(false)
{}

// ######################################################################
KalmanFilter::KalmanFilter(float initialM, float pNoise,
                           float mNoise, float timeStep)
  : initialized(true)
{
  init(initialM, pNoise, mNoise, timeStep);
}


// ######################################################################
void KalmanFilter::init(float initialM, float pNoise,
                        float mNoise, float timeStep)
{
  // set the noise
  itsPNoise = pNoise;
  itsMNoise2 = mNoise * mNoise;

  // populate H
  H.resize(3,1,ZEROS);
  H.setVal(0,0,1.0F);
  H.setVal(1,0,0.0F);
  H.setVal(2,0,0.0F);
  HT = transpose(H);


  // set the inner state to the initial measurement
  //x = HT * initialM;
  x.resize(1,3,ZEROS);
  x.setVal(0,0,initialM);
  x.setVal(0,1,0.0F);
  x.setVal(0,2,0.0F);

  // set the identity matrix
  I = eye<float>(3);

  // fill P with some large numbers on the diagonal
  P = I * 1000.0F;

  // generate powers of timeStep
  std::vector<float> ts(6);
  ts[0] = 1.0F;
  for (uint i = 1; i <= 5; ++i)
    ts[i] = ts[i-1] * timeStep;

  // populate Phi
  Phi = I;
  Phi.setVal(1,0,ts[1]); Phi.setVal(2,0,ts[2]/2.0F); Phi.setVal(2,1,ts[1]);
  PhiT = transpose(Phi);

  // populate Q
  Q.resize(3,3,NO_INIT);
  Q.setVal(0,0,ts[5]/20.0F); Q.setVal(1,0,ts[4]/8.0F); Q.setVal(2,0,ts[3]/6.0F);
  Q.setVal(0,1,ts[4]/ 8.0F); Q.setVal(1,1,ts[3]/3.0F); Q.setVal(2,1,ts[2]/2.0F);
  Q.setVal(0,2,ts[3]/ 6.0F); Q.setVal(1,2,ts[2]/2.0F); Q.setVal(2,2,ts[1]);
  Q *= pNoise;

  // run a first update for M, K, and P
  updateFilter();

  // LINFO("x =  [%g %g %g]; 1st prediction: %g; Phi(0,0) = %g",
  //    x.getVal(0,0),x.getVal(0,1),x.getVal(0,2),
  //    getXEstimate().getVal(0,0),Phi.getVal(0,0));

  initialized = true;
}

// ######################################################################
void KalmanFilter::updateFilter()
{
  //writeImageToStream(std::cout,H);

  // first, update the covariance matrix before update M
  // M = Phi*P*PhiT + Q
  M = matrixMult(matrixMult(Phi,P),PhiT) + Q;

  // now update the Kalman marix itself
  // K = M*HT*(H*M*HT + R)^(-1)
  Image<float> HMHT = matrixMult(matrixMult(H,M),HT);
  ASSERT(HMHT.getSize() == 1);
  K = matrixMult(M,HT) / (HMHT.getVal(0,0) + itsMNoise2);

  // finally, update the covariance matrix after update P
  // P = (I-K*H)*M
  P = matrixMult(I - matrixMult(K,H),M);
}

// ######################################################################
void KalmanFilter::writeToStream(std::ostream& os) const
{
  if (initialized) os << "1\n";
  else os << "0\n";

  if (initialized)
    {
      os << itsPNoise << ' ' << itsMNoise2 << '\n';

      writeImageToStream(os,x);
      writeImageToStream(os,I);
      writeImageToStream(os,M);
      writeImageToStream(os,K);
      writeImageToStream(os,P);
      writeImageToStream(os,H);
      writeImageToStream(os,HT);
      writeImageToStream(os,Phi);
      writeImageToStream(os,PhiT);
      writeImageToStream(os,Q);
    }
}

// ######################################################################
void KalmanFilter::readFromStream(std::istream& is)
{
  int i; is >> i;
  initialized = (i == 1);

  if (initialized)
    {
      is >> itsPNoise;
      is >> itsMNoise2;

      readImageFromStream(is,x);
      readImageFromStream(is,I);
      readImageFromStream(is,M);
      readImageFromStream(is,K);
      readImageFromStream(is,P);
      readImageFromStream(is,H);
      readImageFromStream(is,HT);
      readImageFromStream(is,Phi);
      readImageFromStream(is,PhiT);
      readImageFromStream(is,Q);
    }
}

// ######################################################################
Image<float> KalmanFilter::getXEstimate() const
{
  return matrixMult(Phi,x);
}

// ######################################################################
Image<float> KalmanFilter::getXEstimate(float z) const
{
  Image<float> Px = getXEstimate();
  return (Px + K * (z - matrixMult(H,Px).getVal(0,0)));
}

// ######################################################################
float KalmanFilter::getEstimate() const
{
  ASSERT(initialized);
  float est = getXEstimate().getVal(0,0);
  //LINFO("currVal = %g; est = %g",x.getVal(0,0),est);
  return est;
}

// ######################################################################
float KalmanFilter::getEstimate(float measurement) const
{
  ASSERT(initialized);
  float est = getXEstimate(measurement).getVal(0,0);
  //LINFO("currVal = %g; est = %g",x.getVal(0,0),est);
  return est;
}

// ######################################################################
float KalmanFilter::getSpeed() const
{
  ASSERT(initialized);
  return x.getVal(0,1);
}

// ######################################################################
float KalmanFilter::getCost(float measurement) const
{
  ASSERT(initialized);

  float cost =  sqr(getEstimate(measurement) - measurement);

  if (!(cost == cost))
    {
      writeToStream(std::cout);
      LFATAL("Kalman Filter error");
    }

  return cost;
}

// ######################################################################
float KalmanFilter::update()
{
  ASSERT(initialized);
  x = getXEstimate();
  updateFilter();

  if (!(x.getVal(0,0) == x.getVal(0,0)))
    {
      writeToStream(std::cout);
      LFATAL("Kalman Filter error");
    }

  return x.getVal(0,0);
}

// ######################################################################
float KalmanFilter::update(float measurement)
{
  ASSERT(initialized);
  x = getXEstimate(measurement);
  updateFilter();

  if (!(x.getVal(0,0) == x.getVal(0,0)))
    {
      writeToStream(std::cout);
      LFATAL("Kalman Filter error");
    }

  return x.getVal(0,0);
}

// ######################################################################
Image<float> KalmanFilter::getStateVector() const
{
  ASSERT(initialized);
  return x;
}

// ######################################################################
Image<float> KalmanFilter::getCovariances() const
{
  ASSERT(initialized);
  return P;
}

// ######################################################################
bool KalmanFilter::isInitialized() const
{
  return initialized;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

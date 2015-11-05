/*!@file Image/KalmanFilter.H implementation of a 2nd order linear Kalman Filter
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/KalmanFilter.H $
// $Id: KalmanFilter.H 6005 2005-11-29 18:49:14Z rjpeters $
//

#ifndef KALMANFILTER_H_DEFINED
#define KALMANFILTER_H_DEFINED

#include "Util/Types.H"
#include "Image/Image.H"

#define DEF_PNOISE 1.0F
#define DEF_MNOISE 1.0F
#define DEF_TSTEP  1.0F

//! implementation of a 2nd order linear Kalman Filter
/*! This class can be used for tracking. The state vector is second order,
  i.e. it maintains location, speed, and accelaration to generate
  predictions.*/
class KalmanFilter
{
public:

  //! default constructor - need to call init before the filter can be used
  KalmanFilter();

  //! constructor that initializes the filter
  /*!@param initialM - the initial measurement used to jump start the filter
    @param pNoise - std of the (Gaussian) proess noise
    @param mNoise - std of the (Gaussian) measurement noise
    @param timeStep - the time interval between measurements (default: 1)*/
  KalmanFilter(float initialM, float pNoise = DEF_PNOISE,
               float mNoise = DEF_MNOISE, float timeStep = DEF_TSTEP);

  // default copy constructor and destructor are fine

  //! initialize the filter
  /*!@param initialM - the initial measurement used to jump start the filter
    @param pNoise - std of the (Gaussian) process noise
    @param mNoise - std of the (Gaussian) measurement noise
    @param timeStep - the time interval between measurements (default: 1)*/
  void init(float initialM, float pNoise = DEF_PNOISE,
            float mNoise = DEF_MNOISE, float timeStep = DEF_TSTEP);

  //! write the entire KalmanFilter to the output stream os
  void writeToStream(std::ostream& os) const;

  //! read the KalmanFilter from the input stream is
  void readFromStream(std::istream& is);

  //! returns a prediction for the next value
  float getEstimate() const;

  //! returns a prediction for the next value given a measurement
  float getEstimate(float measurement) const;

  //! returns the speed (second entry in the state vector)
  float getSpeed() const;

  //! returns the cost for associating measurement with this Kalman tracker
  float getCost(float measurement) const;

  //! update the filter for the next time step without a measurement (skipped value)
  float update();

  //! update the filter for the next time step with a measurement
  float update(float measurement);

  //! returns the current state vector [x, v, a]
  Image<float> getStateVector() const;

  //! returns the covariance matrix P
  Image<float> getCovariances() const;

  //! test whether the filter is initialized
  bool isInitialized() const;


private:
  // update the Kalman matrix and the covariance matrices
  void updateFilter();

  // returns the predicted state vector without a measurement
  Image<float> getXEstimate() const;

  // returns the predicted state vector given a measurement
  Image<float> getXEstimate(float z) const;


  // a number of matrices used for the processing
  Image<float> x, I, M, K, P, H, HT, Phi, PhiT, Q;
  float itsPNoise, itsMNoise2;
  bool initialized;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

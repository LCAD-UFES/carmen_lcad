/*!@file Surprise/SurpriseImage.H a 2D array of SurpriseModel objects */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Surprise/SurpriseImage.H $
// $Id: SurpriseImage.H 9983 2008-07-27 06:34:32Z mundhenk $
//

#ifndef SURPRISEIMAGE_H_DEFINED
#define SURPRISEIMAGE_H_DEFINED

#include "Image/Image.H"
#include "Surprise/SurpriseModel.H"

// ######################################################################
//! A 2D array of SurpriseModel objects
/*! This class is derived fromImage and inherits all its
  functionality, and in addition provides batch access to several of
  the SurpriseModel functions. Although the template argument T could
  in principle be anything, it is required that it supports the
  SurpriseModel functions like update(), surprise(), etc */
template <class T>
class SurpriseImage : public Image<T>
{
public:
  //! Uninitialized constructor
  SurpriseImage();

  //! Constructor for an empty SurpriseImage with uninitialized models
  SurpriseImage(const Dims& dims);

  //! Constructor from given sample means and variances
  SurpriseImage(const double updfac, const Image<double>& sampleval,
                const Image<double>& samplevar);

  //! Virtual destructor ensures proper destruction of derived classes
  virtual ~SurpriseImage();

  //! Reset all models to initial state
  virtual void reset();

  //! Init to given sample mean and variance
  virtual void init(const double updfac, const Image<double>& sampleval,
                    const Image<double>& samplevar);

  //! reset our update factor
  virtual void resetUpdFac(const double updfac);

  //! Compute surprise between us and another model
  virtual Image<double> surprise(const SurpriseImage<T>& other);

  //! For models with covariance, pre-compute hyper parameters
  virtual void preComputeHyperParams(const SurpriseImage<T>& models);

  //! Initialize us as a weighted combination of the given map of models
  /*! This will initialize us so that each of our pixels is built from
    the SurpriseModel combineFrom() function, with a Gaussian weighing
    mask centered onto that pixel and with sigma given here as
    argument. */
  virtual void neighborhoods(const SurpriseImage<T>& models,
                             const float neighsigma);

  //! Initialize us as a weighted combination of the given map of models
  /*! This will initialize us so that each of our pixels is built from
    the SurpriseModel combineFrom() function. In this version, weights
    should be an image with double+1 the width and height of
    models. */
  virtual void neighborhoods(const SurpriseImage<T>& models,
                             const Image<float>& weights);

  //! Call this to re-use the model and not initialize the model with a new one
  /*! This is a variant to use neighborhoods without needing to copy in a new
    model but insted preserves the current model params
  */
  virtual void neighborhoods(const SurpriseImage<T>& models,
                             const Image<float>& weights,
                             const bool NO_INIT);
  //! get our means
  virtual Image<double> getMean() const;

  //! get our variances
  virtual Image<double> getVar() const;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

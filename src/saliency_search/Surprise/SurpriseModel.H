/*!@file Surprise/SurpriseModel.H a local (single-point) model of surprise */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Surprise/SurpriseModel.H $
// $Id: SurpriseModel.H 12962 2010-03-06 02:13:53Z irock $
//

#ifndef SURPRISEMODEL_H_DEFINED
#define SURPRISEMODEL_H_DEFINED

#include "Image/Point2D.H"
#include "Image/Image.H"
#include "Util/MathFunctions.H"

enum SU_KL_BIAS
  {
    SU_KL_NONE,
    SU_KL_STATIC
  };

// ######################################################################
//! A local (single-point) surprise model
/*! This the base class and it cannot be implemented as some of its
  functions are purely virtual. It defines the basic interface. See
  the derivative classes for actual implementations. */
class SurpriseModel
{
public:
  //! Constructor
  /*! Build a new SurpriseModel object and initialize it as if an
    infinitely long sequence of samples had been previously received
    by the model.
    @param updatefac factor by which the current model is combine with
    a new model on an update. Valid values are in [0..1]
    @param sampleval the value of the samples previously received.
    @param samplevar a variance expressing the intrinsic variability
    in the measurement of those samples. */
  SurpriseModel(const double updatefac = 0.5, const double sampleval = 0.0,
                const double samplevar = 1.0);

  // default copy-construction and copy operators OK.

  //! Virtual destructor ensures proper destruction of derived classes
  virtual ~SurpriseModel();

  //! Reset to initial state
  virtual void reset() = 0;

  //! Change intial state parameters
  virtual void init(const double updatefac = 0.5, const double sampleval = 0.0,
                    const double samplevar = 1.0);

  //! Reset our update factor
  virtual void resetUpdFac(const double updfac);

  //! Load given sample mean and variance
  virtual void load(const double sampleval = 0.0,
                    const double samplevar = 1.0) = 0;

  //! Compute surprise between us and another model
  /*! Surprise is the KL distance between our current model and what
    that model will become once we update it with the sample passed in
    as argument.  For the update, our current model is weighted by
    itsUpdateFac while the new model by (1.0-itsUpdateFac). Programmer
    note: derived classes will not provide a direct overload of this
    (and other) base functions, as instead they will implement
    functions with basically the same syntax but using an argument of
    the derived class. Since an implementation is not provided for
    this base function, you will get an error at link time if you call
    the derived functions with a base-class argument. */
  virtual double surprise(const SurpriseModel& sample);

  //! Pre compute hyper parameters before combineFrom
  /*! Some models may need to update their hyper parameters BEFORE
    calling combineFrom. Particularly if the parameters have a covariance
    measure which must be updated before the models are combined
  */
  virtual void preComputeHyperParams(const SurpriseModel& sample);

  //! Initialize us as a weighted combination of the given map of models
  /*! @param models an array of models, one model at each pixel of the image.
    @param weights the weight to be given to each model. Weights should sum
    to 1.0 and this is not checked for here for computational efficiency. */
  virtual void combineFrom(const Image<SurpriseModel>& models,
                           const Image<float>& weights);

  //! Initialize us as a weighted combination of the given map of models
  /*! @param models an array of models, one model at each pixel of the image.
    @param weights the weight to be given to each model. Weights
    should sum to 1.0 and this is not checked for here for
    computational efficiency. In this version, the weight image should
    have double+1 the width and height of the Surprise image, and the
    weight for the current point will be taken at the center of the
    weight map, etc.
    @param pos indicates the 2D coordinates of the center of the
    neighborhood in the Image
    @param minw is the minimum weight below which we just don't include a
    model in the update*/
  virtual void combineFrom(const Image<SurpriseModel>& models,
                           const Image<float>& weights,
                           const Point2D<int>& pos,
                           const int width, const int height, const int offset);

  //! Set up biases if needed for KL
  virtual void setBias(const double slfac,
                       const double ssfac,
                       const SU_KL_BIAS klbias);
  //! get our mean
  virtual double getMean() const = 0;

  //! get our variance
  virtual double getVar() const = 0;

  //! get our UpdateFac
  virtual double getUpdateFac() const = 0;



protected:
  double itsUpdateFac;  //!< our update factor
  double itsInitialVal; //!< our initial value
  double itsInitialVar; //!< our initial variance
};

// ######################################################################
//! A single-Gaussian SurpriseModel
/*! This is a very simple SurpriseModel consisting of a single
  Gaussian, which is updated in a sliding-average manner. */
class SurpriseModelSG : public SurpriseModel
{
public:
  //! Constructor. See base class for details.
  SurpriseModelSG(const double updatefac = 0.5, const double sampleval = 0.0,
                  const double samplevar = 1.0);

  //! Virtual destructor ensures proper destruction of derived classes
  virtual ~SurpriseModelSG();

  //! Reset to initial state
  virtual void reset();

  //! Change intial state parameters
  virtual void init(const double updatefac = 0.5, const double sampleval = 0.0,
                    const double samplevar = 1.0);

  //! Load given sample mean and variance
  virtual void load(const double sampleval, const double samplevar = 1.0);

  //! Compute surprise between us and another model
  virtual double surprise(const SurpriseModelSG& other);

  //! Is empty in this model
  virtual void preComputeHyperParams(const SurpriseModelSG& sample);

  //! Initialize us as a weighted combination of the given map of models
  virtual void combineFrom(const Image<SurpriseModelSG>& models,
                           const Image<float>& weights);

  //! Initialize us as a weighted combination of the given map of models
  virtual void combineFrom(const Image<SurpriseModelSG>& models,
                           const Image<float>& weights,
                           const Point2D<int>& pos,
                           const int width, const int height, const int offset);

  //! get our mean
  virtual double getMean() const;

  //! get our variance
  virtual double getVar() const;

  //! get our UpdateFac
  virtual double getUpdateFac() const;

protected:
  int itsN;        //!< our sample N
  double itsMean;  //!< our current Gaussian mean
  double itsVar;   //!< our current Gaussian variance
};

// ######################################################################
//! A single-Poisson/Gamma SurpriseModel
/*! This is a very simple SurpriseModel consisting of a single Gamma
  prior over Poisson, which is updated in a sliding-average manner. */
class SurpriseModelSP : public SurpriseModel
{
public:
  //! Constructor. See base class for details.
  SurpriseModelSP(const double updatefac = 0.5, const double sampleval = 0.0,
                  const double samplevar = 1.0 /* ignored */);

  //! Virtual destructor ensures proper destruction of derived classes
  virtual ~SurpriseModelSP();

  //! Reset to initial state
  virtual void reset();

  //! Change intial state parameters
  virtual void init(const double updatefac = 0.5, const double sampleval = 0.0,
                    const double samplevar = 1.0 /* ignored */);

  //! Load given sample mean and variance
  virtual void load(const double sampleval, const double samplevar = 1.0);

  //! Compute surprise between us and another model
  virtual double surprise(const SurpriseModelSP& other);

  //! Is empty in this model
  virtual void preComputeHyperParams(const SurpriseModelSP& sample);

  //! Initialize us as a weighted combination of the given map of models
  virtual void combineFrom(const Image<SurpriseModelSP>& models,
                           const Image<float>& weights);

  //! Initialize us as a weighted combination of the given map of models
  virtual void combineFrom(const Image<SurpriseModelSP>& models,
                           const Image<float>& weights,
                           const Point2D<int>& pos,
                           const int width, const int height, const int offset);

  //! get our mean
  virtual double getMean() const;

  //! get our variance
  virtual double getVar() const;

  //! get our UpdateFac
  virtual double getUpdateFac() const;

  //! get our alpha
  double getAlpha() const;

  //! get out beta
  double getBeta() const;

  //! Pre set model alpha values
  void preSetAlpha();

protected:
  int itsN;             //!< our sample N
  double itsAlpha;      //!< our current Gamma alpha
  double itsBeta;       //!< our current Gamma beta

};
// ######################################################################
//! A single-Poisson/Gamma SurpriseModel
/*! This is a very simple SurpriseModel consisting of a single Gamma
  prior over Poisson, which is updated in a sliding-average manner.

  In this implementation surprise is calculated before the alpha and
  beta parameters decay so that there is no baseline surprise when the
  prior and posterior are the same.*/

class SurpriseModelSP1 : public SurpriseModel
{
public:
  //! Constructor. See base class for details.
  SurpriseModelSP1(const double updatefac = 0.5, const double sampleval = 0.0,
                   const double samplevar = 1.0 /* ignored */);

  //! Virtual destructor ensures proper destruction of derived classes
  virtual ~SurpriseModelSP1();

  //! Reset to initial state
  virtual void reset();

  //! Change intial state parameters
  virtual void init(const double updatefac = 0.5, const double sampleval = 0.0,
                    const double samplevar = 1.0 /* ignored */);

  //! Load given sample mean and variance
  virtual void load(const double sampleval, const double samplevar = 1.0);

  //! Compute surprise between us and another model
  virtual double surprise(const SurpriseModelSP1& other);

  //! Is empty in this model
  virtual void preComputeHyperParams(const SurpriseModelSP1& sample);

  //! Initialize us as a weighted combination of the given map of models
  virtual void combineFrom(const Image<SurpriseModelSP1>& models,
                           const Image<float>& weights);

  //! Initialize us as a weighted combination of the given map of models
  virtual void combineFrom(const Image<SurpriseModelSP1>& models,
                           const Image<float>& weights,
                           const Point2D<int>& pos,
                           const int width, const int height, const int offset);

  //! get our mean
  virtual double getMean() const;

  //! get our variance
  virtual double getVar() const;

  //! get our UpdateFac
  virtual double getUpdateFac() const;

  //! get our alpha
  double getAlpha() const;

  //! get out beta
  double getBeta() const;

  //! Pre set model alpha values
  void preSetAlpha();

protected:
  int itsN;             //!< our sample N
  double itsAlpha;      //!< our current Gamma alpha
  double itsBeta;       //!< our current Gamma beta

};


// ######################################################################
//! A single-Poisson/Gamma SurpriseModel
/*! This is a very simple SurpriseModel consisting of a single Gamma
  prior over Poisson, which is updated in a sliding-average manner.

  This variant fixes the beta term which is useful for multi frame inputs

*/
class SurpriseModelSPC : public SurpriseModel
{
public:
  //! Constructor. See base class for details.
  SurpriseModelSPC(const double updatefac = 0.5, const double sampleval = 0.0,
                   const double samplevar = 1.0 /* ignored */);

  //! Virtual destructor ensures proper destruction of derived classes
  virtual ~SurpriseModelSPC();

  //! Reset to initial state
  virtual void reset();

  //! Change intial state parameters
  virtual void init(const double updatefac = 0.5, const double sampleval = 0.0,
                    const double samplevar = 1.0 /* ignored */);

  //! Load given sample mean and variance
  virtual void load(const double sampleval, const double samplevar = 1.0);

  //! Compute surprise between us and another model
  virtual double surprise(const SurpriseModelSPC& other);

  //! Is empty in this model
  virtual void preComputeHyperParams(const SurpriseModelSPC& sample);

  //! Initialize us as a weighted combination of the given map of models
  virtual void combineFrom(const Image<SurpriseModelSPC>& models,
                           const Image<float>& weights);

  //! Initialize us as a weighted combination of the given map of models
  virtual void combineFrom(const Image<SurpriseModelSPC>& models,
                           const Image<float>& weights,
                           const Point2D<int>& pos,
                           const int width, const int height, const int offset);

  //! get our mean
  virtual double getMean() const;

  //! get our variance
  virtual double getVar() const;

  //! get our UpdateFac
  virtual double getUpdateFac() const;

  //! get our alpha
  double getAlpha() const;

  //! get out beta
  double getBeta() const;

  //! Pre set model alpha values
  void preSetAlpha();

protected:
  int    itsN;          //!< our sample N
  double itsAlpha;      //!< our current Gamma alpha
  double itsBeta;
};

// ######################################################################
//! A single-Poisson/Gamma SurpriseModel
/*! This is a very simple SurpriseModel consisting of a single Gamma
  prior over Poisson, which is updated in a sliding-average manner.

  This variant floats the beta term which may be useful for multi frame inputs

  <<Experimental>>

*/
class SurpriseModelSPF : public SurpriseModel
{
public:
  //! Constructor. See base class for details.
  SurpriseModelSPF(const double updatefac = 0.5, const double sampleval = 0.0,
                   const double samplevar = 1.0 /* ignored */);

  //! Virtual destructor ensures proper destruction of derived classes
  virtual ~SurpriseModelSPF();

  //! Reset to initial state
  virtual void reset();

  //! Change intial state parameters
  virtual void init(const double updatefac = 0.5, const double sampleval = 0.0,
                    const double samplevar = 1.0 /* ignored */);

  //! Load given sample mean and variance
  virtual void load(const double sampleval, const double samplevar = 1.0);

  //! Compute surprise between us and another model
  virtual double surprise(const SurpriseModelSPF& other);

  //! Is empty in this model
  virtual void preComputeHyperParams(const SurpriseModelSPF& sample);

  //! Initialize us as a weighted combination of the given map of models
  virtual void combineFrom(const Image<SurpriseModelSPF>& models,
                           const Image<float>& weights);

  //! Initialize us as a weighted combination of the given map of models
  virtual void combineFrom(const Image<SurpriseModelSPF>& models,
                           const Image<float>& weights,
                           const Point2D<int>& pos,
                           const int width, const int height, const int offset);

  //! get our mean
  virtual double getMean() const;

  //! get our variance
  virtual double getVar() const;

  //! get our UpdateFac
  virtual double getUpdateFac() const;

  //! get our alpha
  double getAlpha() const;

  //! get out beta
  double getBeta() const;

  //! Pre set model alpha values
  void preSetAlpha();

protected:
  int    itsN;          //!< our sample N
  double itsAlpha;      //!< our current Gamma alpha
  double itsBeta;
  double itsLastS;
  double itsSFac;
};

// ######################################################################
//! A Chi-Square SurpriseModel
/*! This is a very simple SurpriseModel consisting of a single Gamma
  prior over Poisson, which is updated in a sliding-average manner.
  Beta is fixed as 1/2 so this is a special case where the Gamma is
  the Chi Square distribution.
*/
class SurpriseModelCS : public SurpriseModel
{
public:
  //! Constructor. See base class for details.
  SurpriseModelCS(const double updatefac = 0.5, const double sampleval = 0.0,
                  const double samplevar = 1.0 /* ignored */);

  //! Virtual destructor ensures proper destruction of derived classes
  virtual ~SurpriseModelCS();

  //! Reset to initial state
  virtual void reset();

  //! Change intial state parameters
  virtual void init(const double updatefac = 0.5, const double sampleval = 0.0,
                    const double samplevar = 1.0 /* ignored */);

  //! Load given sample mean and variance
  virtual void load(const double sampleval, const double samplevar = 1.0);

  //! Compute surprise between us and another model
  virtual double surprise(const SurpriseModelCS& other);

  //! Is empty in this model
  virtual void preComputeHyperParams(const SurpriseModelCS& sample);

  //! Initialize us as a weighted combination of the given map of models
  virtual void combineFrom(const Image<SurpriseModelCS>& models,
                           const Image<float>& weights);

  //! Initialize us as a weighted combination of the given map of models
  virtual void combineFrom(const Image<SurpriseModelCS>& models,
                           const Image<float>& weights,
                           const Point2D<int>& pos,
                           const int width, const int height, const int offset);

  //! get our mean
  virtual double getMean() const;

  //! get our variance
  virtual double getVar() const;

  //! get our UpdateFac
  virtual double getUpdateFac() const;

  //! get our alpha
  double getAlpha() const;

  //! get out beta
  double getBeta() const;

  //! Pre set model alpha values
  void preSetAlpha();

protected:
  int itsN;             //!< our sample N
  double itsAlpha;      //!< our current Gamma alpha
  double itsBeta;       //!< our current Gamma beta

};

// ######################################################################
//! A joint Gamma/Gaussian SurpriseModel
/*! This is a very simple SurpriseModel consisting of a single Gamma
  prior over Poisson (time), which is updated in a sliding-average manner.
  space is updated as a gaussian and the two are combined as a joint
  KL to compute surprise*/
class SurpriseModelGG : public SurpriseModel
{
public:
  //! Constructor. See base class for details.
  SurpriseModelGG(const double updatefac = 0.5, const double sampleval = 0.0,
                  const double samplevar = 1.0 /* ignored */);

  //! Virtual destructor ensures proper destruction of derived classes
  virtual ~SurpriseModelGG();

  //! Reset to initial state
  virtual void reset();

  //! Change intial state parameters
  virtual void init(const double updatefac = 0.5, const double sampleval = 0.0,
                    const double samplevar = 1.0 /* ignored */);

  //! Load given sample mean and variance
  virtual void load(const double sampleval, const double samplevar = 1.0);

  //! Compute surprise between us and another model
  virtual double surprise(const SurpriseModelGG& other);

  //! Is empty in this model
  virtual void preComputeHyperParams(const SurpriseModelGG& sample);

  //! Initialize us as a weighted combination of the given map of models
  virtual void combineFrom(const Image<SurpriseModelGG>& models,
                           const Image<float>& weights);

  //! Initialize us as a weighted combination of the given map of models
  virtual void combineFrom(const Image<SurpriseModelGG>& models,
                           const Image<float>& weights,
                           const Point2D<int>& pos,
                           const int width, const int height, const int offset);

  //! get our mean
  virtual double getMean() const;

  //! get our variance
  virtual double getVar() const;

  //! get our UpdateFac
  virtual double getUpdateFac() const;

  //! Set up biases if needed for KL
  virtual void setBias(const double slfac,
                       const double ssfac,
                       const SU_KL_BIAS klbias);

  //! get our alpha
  double getAlpha() const;

  //! get out beta
  double getBeta() const;

protected:
  bool itsCombineFromRun; //!< Make sure we run combineFrom each iter
  int itsN;               //!< our sample N
  SU_KL_BIAS itsJointKLBiasType; //!< enum for kl bias type
  double itsAlpha;        //!< our current Gamma alpha    (time)
  double itsBeta;         //!< our current Gamma beta     (time)
  double itsMean;         //!< our current Gauss mean     (space)
  double itsVar;          //!< our current Gauss variance (space)
  double itsSig;          //!< our current sigma          (space)
  double itsSum;          //!< Holds the sum from combineFrom
  double itsSS;           //!< Holds the sum of squares from combineFrom
  double itsWeightSum;    //!< Holds the weight sum from combineFrom
  double itsSLfac;        //!< Temporal bias to apply if needed
  double itsSSfac;        //!< Spatial bias to apply if needed

};

// ######################################################################
//! A single-Poisson/Gamma SurpriseModel with inertia
/*! This is a very simple SurpriseModel consisting of a single Gamma
  prior over Poisson, which is updated in a sliding-average manner. */
class SurpriseModelPM : public SurpriseModel
{
public:
  //! Constructor. See base class for details.
  SurpriseModelPM(const double updatefac = 0.5, const double sampleval = 0.0,
                  const double samplevar = 1.0 /* ignored */);

  //! Virtual destructor ensures proper destruction of derived classes
  virtual ~SurpriseModelPM();

  //! Reset to initial state
  virtual void reset();

  //! Change intial state parameters
  virtual void init(const double updatefac = 0.5, const double sampleval = 0.0,
                    const double samplevar = 1.0 /* ignored */);

  //! Load given sample mean and variance
  virtual void load(const double sampleval, const double samplevar = 1.0);

  //! Compute surprise between us and another model
  virtual double surprise(const SurpriseModelPM& other);

  //! pre compute alpha and beta terms before calling combine from
  virtual void preComputeHyperParams(const SurpriseModelPM& sample);

  //! Initialize us as a weighted combination of the given map of models
  virtual void combineFrom(const Image<SurpriseModelPM>& models,
                           const Image<float>& weights);

  //! Initialize us as a weighted combination of the given map of models
  virtual void combineFrom(const Image<SurpriseModelPM>& models,
                           const Image<float>& weights,
                           const Point2D<int>& pos,
                           const int width, const int height, const int offset);

  //! get our mean
  virtual double getMean() const;

  //! get our variance
  virtual double getVar() const;

  //! get our UpdateFac
  virtual double getUpdateFac() const;

protected:
  int    itsN;           //!< our sample N

  double itsSample;
  double itsAlpha0;      //!< our current Gamma alpha
  double itsAlpha1;      //!< our current Gamma alpha
  double itsAlpha2;      //!< our current Gamma alpha
  double itsBeta0;       //!< our current Gamma beta
  double itsBeta1;       //!< our current Gamma beta
  double itsBeta2;       //!< our current Gamma beta
  double itsInitBeta;    //!< Initial Value of beta
  double itsExpectAlpha1;//!< The expected value of alpha1
  double itsExpectAlpha2;//!< The expected value of alpha2
  double itsXBar1;       //!< The estimated value of alpha1
  double itsXBar2;       //!< The estimated value of alpha2
  double itsLgamma1;     //!< Computing lgamma is expensive, so we keep it
  double itsLgamma2;     //!< Computing lgamma is expensive, so we keep it
};

// ######################################################################
//! A single-Poisson Outlier-based model
/*! This is not really a true SurpriseModel, but rather an information
  model based on outlier detection using a single adaptive Poisson
  model. */
class SurpriseModelOD : public SurpriseModel
{
public:
  //! Constructor. See base class for details.
  SurpriseModelOD(const double updatefac = 0.5, const double sampleval = 0.0,
                  const double samplevar = 1.0 /* ignored */);

  //! Virtual destructor ensures proper destruction of derived classes
  virtual ~SurpriseModelOD();

  //! Reset to initial state
  virtual void reset();

  //! Change intial state parameters
  virtual void init(const double updatefac = 0.5, const double sampleval = 0.0,
                    const double samplevar = 1.0 /* ignored */);

  //! Load given sample mean and variance
  virtual void load(const double sampleval, const double samplevar = 1.0);

  //! Compute surprise between us and another model
  virtual double surprise(const SurpriseModelOD& other);

  //! Is empty in this model
  virtual void preComputeHyperParams(const SurpriseModelOD& sample);

  //! Initialize us as a weighted combination of the given map of models
  virtual void combineFrom(const Image<SurpriseModelOD>& models,
                           const Image<float>& weights);

  //! Initialize us as a weighted combination of the given map of models
  virtual void combineFrom(const Image<SurpriseModelOD>& models,
                           const Image<float>& weights,
                           const Point2D<int>& pos,
                           const int width, const int height, const int offset);

  //! get our mean
  virtual double getMean() const;

  //! get our variance
  virtual double getVar() const;

  //! get our UpdateFac
  virtual double getUpdateFac() const;

protected:
  int itsN;          //!< our sample N
  double itsLambda;  //!< our current Poisson lambda
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

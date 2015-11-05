/*!@file ModelNeuron/Weights.H Class A class to compute interactions
  between neurons. */

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
// Primary maintainer for this file: David Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/Weights.H $

#ifndef MODELNEURON_WEIGHTS_H_DEFINED
#define MODELNEURON_WEIGHTS_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "Image/Image.H"
#include "Image/LowPassLpt.H"
#include "Image/Convolutions.H"

#include <vector>

// ######################################################################
// Policies for different weighting schemes to be used as a template
// argument by Layer. A valid policy should have at least:
//
// a default constructor.
// Image<double> compute(const Image<double>& in);
// const bool initialized() const;
//
// All equations for the below classes are inspired by Neural Field
// dynamics: Amari & Arbib, 77; Coombs, 05; Erlhagen & Bicho, 2006
// ######################################################################

// ######################################################################
// A weight class that defines minimum interface and can be used virtually
// ######################################################################
struct Weights
{
public:
  Weights() : itsInit(false) { };
  virtual ~Weights() { };
  virtual Image<double> compute(const Image<double>& in) = 0;
  virtual Weights* clone() const = 0;
  virtual const bool initialized() const { return itsInit; };
protected:
  bool itsInit;
};

// ######################################################################
//! A class to derive from to create new Weightss. New
//! Weights derived types can derive from this class to inherit
//! the clone() function if desired.
/*
  Programmer Note: This class uses the 'quriously recursive template
  pattern' to simplify creation of new classes for the programmer. As
  such, to create a new simulation module, classes should adhear the
  following convention:
  
  class mymodule : public WeightsDerived<mymodule>
  {
      mymodule(//params//) : Weights<mymodule>(//params//) { };
      //...rest of functions
  };
*/
// ######################################################################
template <class Derived>
class WeightsDerived : public Weights
{
public:
  Weights* clone() const 
  { return new Derived(dynamic_cast<const Derived&>(*this)); };               
protected:
  WeightsDerived() : Weights() { };
  virtual ~WeightsDerived() { };
};


// ######################################################################
// A simple weights configuration that just multiplies the input by a
// constant.
// ######################################################################
class WeightsEmpty : public WeightsDerived<WeightsEmpty>
{
public:
  //default constructor
  WeightsEmpty() : WeightsDerived<WeightsEmpty>() { itsInit = true; };

  //destructor
  ~WeightsEmpty() { };  

  //compute the spread of activity
  Image<double> compute(const Image<double>& in) {return Image<double>(in.getDims(), ZEROS); };
};

// ######################################################################
// A weights configuration where the every neural connects to every
// other neuron.
// ######################################################################
class WeightsAll : public WeightsDerived<WeightsAll>
{
public:
  //default constructor
  WeightsAll();

  //constructor
  WeightsAll(const double& weight);

  //destructor
  ~WeightsAll() { };  

  //compute the spread of activity
  Image<double> compute(const Image<double>& in);
  
private:
  double itsW;
};

// ######################################################################
// A simple weights configuration that just multiplies the input by a
// constant.
// ######################################################################
class WeightsUniform : public WeightsDerived<WeightsUniform>
{
public:
  //default constructor
  WeightsUniform();

  //constructor
  WeightsUniform(const double& weight);

  //destructor
  ~WeightsUniform() { };  

  //compute the spread of activity
  Image<double> compute(const Image<double>& in);

private:
  double itsW;
};

// ######################################################################
// A weight configuration that uses a 1D binomial kernel
//
// Iterative seperable convolution is applied to achieve the desired
// standard deviation. The output is then scaled so that the response
// to an impulse is unity at the center. Self activation is optionally
// subtracted. This object state is set to initialzed after
// its first call.
//
// Several convolution strategies are supported. If the BorderType ==
// 0, then clean convolution will be performed. The filter is
// truncated and normalized at the image edge.

// Other border types listed below, were designed for radially
// transformed (RT) images that represent visual space in
// hemifields. Only 3 and 5-tap binomial filtering is
// implemented. Three boundary conditions are supported for these
// images:
//
// 1) SEPARATE_HEMI 
//   No hemifield interactions. Clean (normalized energy) border
//   conditions apply at each hemifild in both directions.
// 2) CROSS_HEMI
//   Convolutions will cross hemifields in both the R (ring) and W
//   (wedge) dirctions.
// 3) CROSS_ANGLE
//   Convolutions will only cross hemifields in the W (wedge)
//   direction. Clean (normalized energy) border condition will be
//   applied in the R (ring) dircection.
// ######################################################################
class WeightsBinomial  : public WeightsDerived<WeightsBinomial>
{
public:
  //default constructor
  WeightsBinomial();

  //constructor
  WeightsBinomial(const double& std,  const double& weight,
                  const bool doSubCenter, 
                  const BorderPolicy bp,
                  const uint taps = 3);
  //destructor
  ~WeightsBinomial() { };  

  //compute the spread of activity
  Image<double> compute(const Image<double>& in);
  
private:
  uint itsTaps;
  uint itsIter;
  bool subCenter;
  double itsCenter;
  double itsWeight;
  BorderPolicy itsBp;
  bool itsSetup;
};

// ######################################################################
// a simple center surround pattern using a gaussian minus a constant
// ######################################################################
class WeightsCS  : public WeightsDerived<WeightsCS>
{
public:
  //default constructor
  WeightsCS();

  //constructor
  WeightsCS(const double& estd, const double& eweight, 
            const double& inhibit, const bool doSubCenter,
            const BorderPolicy bp, const uint taps = 3);
  
  //destructor
  ~WeightsCS() { };  

  //compute the spread of activity
  Image<double> compute(const Image<double>& in);

private:
  WeightsBinomial itsW;
  double itsInh;
};

// ######################################################################
// A weight configuration that uses 1D binomial kernels to approximate
// a difference of gaussians
//
// Iterative seperable convolution is applied to achieve the desired
// standard deviation. The output is then scaled so that the response
// to an impulse is the desired weight. 
// ######################################################################
class WeightsDoG  : public WeightsDerived<WeightsDoG>
{
public:
  //default constructor
  WeightsDoG();

  //constructor
  WeightsDoG(const double& estd, const double& istd,  
             const double& eweight, const double& iweight,
             const bool subCenter, const BorderPolicy bp,
             const uint taps = 3);
  //destructor
  ~WeightsDoG() { };  

  //compute the spread of activity
  Image<double> compute(const Image<double>& in);
  
private:
  WeightsBinomial itsE, itsI;
};

/*
// ######################################################################
// A weight configuration that uses 1D seperable Gaussian kernels
//
// ######################################################################
class WeightsGaussian : public WeightsDerived<WeightsGaussian>
{
public:
  //default constructor
  WeightsGaussian();

  //constructor
  WeightsGaussian(const double& std, const double& weight, const bool doSubCenter,
                  const ConvolutionBoundaryStrategy strat = CONV_BOUNDARY_CLEAN);
  
  //destructor
  ~WeightsGaussian() { };  
  
  //compute weighted output
  Image<double> compute(const Image<double>& in);
  
private:
  bool subCenter;
  double itsCenter;
  ConvolutionBoundaryStrategy itsStrat;
  Image<double> itsKernel;
};
*/

// ######################################################################
// A weight configuration that uses 1D seperable kernels
//
// If more than one filter set (x and y filters) is added then filters
// are applied in the order they were added and other processing (such
// as subtracting self activation).
// ######################################################################
class Weights1D : public WeightsDerived<Weights1D>
{
public:
  //default constructor
  Weights1D();

  //constructor
  Weights1D(const bool doSubCenter,
            const ConvolutionBoundaryStrategy strat = CONV_BOUNDARY_CLEAN);
  
  //destructor
  ~Weights1D() { };  
  
  //add a kernel to the bank, to be used in both x and y direction
  void addWeight(const Image<double>& wxy);

  //add the kernels to be used in the x and y direction
  void addWeight(const Image<double>& wx, const Image<double>& wy);

  //!clear our weights  
  void clear();

  //compute weighted output
  Image<double> compute(const Image<double>& in);
  
private:
  bool subCenter;
  double itsCenter;
  ConvolutionBoundaryStrategy itsStrat;
  std::vector<Image<double> > itsX;
  std::vector<Image<double> > itsY;
  bool itsSetup;
};

// ######################################################################
// A weight configuration that uses a 2D kernel
//
// If more than one filter is added the filter are applied in the order
// they were added.
//######################################################################
class Weights2D  : public WeightsDerived<Weights2D>
{
public:
  //constructor
  Weights2D(const ConvolutionBoundaryStrategy strat = CONV_BOUNDARY_CLEAN);
  
  //destructor
  ~Weights2D() { };  

  //add a kernel to the bank
  void addWeight(const Image<double>& w);
  
  //!clear our weights  
  void clear();
  
  //compute weighted output
  Image<double> compute(const Image<double>& in);

private:
  ConvolutionBoundaryStrategy itsStrat;
  std::vector<Image<double> > itsW;
};

// ######################################################################
// A weight configuration that uses a Mask
//
// In this weight scheme the connection pattern is fully
// specified. That means that you must add a seperate Image<double>
// for each pixels which describes which other pixels contribute to
// its response. The objects state is set to initialzed when the
// object is constructed, but calls to compute will return images of
// zeros unless one weight mask has been added for every pixel in the
// input.
// ######################################################################
class WeightsMask  : public WeightsDerived<WeightsMask>
{
public:
  //constructor
  WeightsMask();

  //destructor
  ~WeightsMask() { };  
  
  //add a kernel to the bank
  void addWeight(const Image<double>& w);
  
  //!clear our weights  
  void clear();
  
  //compute weighted output
  Image<double> compute(const Image<double>& in);

private:
  std::vector<Image<double> > itsW;
};

// ######################################################################
//helper functions
// ######################################################################

// for weights that use binomial
Image<double> filterBinomial(const Image<double>& in, const uint iterations, 
                             const uint taps, const BorderPolicy bp);

//print the impulse response of a weight 
template<class W>
void printImpulse(const uint w, const uint h,  W& weights);

//print the image 
void printImage(const Image<double>& in);


#endif
#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

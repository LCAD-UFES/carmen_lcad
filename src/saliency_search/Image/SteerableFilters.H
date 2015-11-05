
/*!@file Image/SteerableFilters.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/SteerableFilters.H $
//

#ifndef STEERABLEFILTERS_H_DEFINED
#define STEERABLEFILTERS_H_DEFINED

#ifdef INVT_USE_CPP11//we need c++ 0X features for this to work

#include "Image/ImageSet.H"
#include "Image/Convolutions.H"

// ###################################################################### 
// An implementation of steerable filters using 3D 2nd derivitive of Gaussians
// and their Hilbert transforms. The filters detect static edges, flicker and
// motion at different speeds and orientations. The sum of the squares of
// quadrature pair filter are returned.
//
// see: Derpanis, K.G. and Gryn, J.M. Three-dimensional nth derivative of
// Gaussian separable steerable filters. IEEE International Conference on Image
// Processing, 2005. ICIP 2005.
// ######################################################################


// ###################################################################### 
// hold a 3D seperable filter set
// ######################################################################
struct SeperableFilter3D
{
    Image<float> xfilt;
    Image<float> yfilt;
    Image<float> zfilt;
};
  
// ###################################################################### 
// Hold parameters for a stearable filter
// ######################################################################
struct Gauss2ndDerivParams
{
    //Emptry constructor
    Gauss2ndDerivParams();
      
    //!Constructor using direction cosines (alpha, beta, gamma form) of the axis of symmetry. 
    /*
      1,0,0 is static vertical edge
      0,1,0 is a vertical edge
      0,0,1 is flicker at no orientation
      1/sqrt(2), 0, 1/sqrt(2) is rightward
      -1/sqrt(2), 0, 1/sqrt(2) is leftward
      0, 1/sqrt(2), 1/sqrt(2) is upward
      0, -1/sqrt(2), 1/sqrt(2) is downward
    */
    Gauss2ndDerivParams(float const & a, float const & b, float const & g);
      
    //!Constructor in theta, phi form
    /*
      These values describe the direction cosines (alpha, beta, gamma) of the
      axis of the symmetry parametrically by spherical coordinates (theta,
      phi, row=1).

      This parameterization can be thought of as the orientaiton of the filter
      in the 3D space-time (x,y,t) hypercube that extends from the x-y face at
      minimum t to the x-y face at maximum t. Theta describes the angle of
      orientation in x-y (spatial orientation) and phi represents the angle of
      the filter from the x-y face at minimim t. In practice, theta can range
      from [0...180) degrees without redundancy. Phi can range from (-90 90]
      degrees. At phi=0, the filter is perpendicular to the x-y face and the
      spatial orientation components are non-existence. This detects temporal
      flicker. At 90 degrees the filter is parallel with the x-y face and we
      only detect static edges. All othe values will detect motion
      perpendicular to the direciton of the edge.  

      Ex: 
      (0,45) rightward motion
      (0,-45) leftward motion
      (90,45) upward motion
      (90,-45) downward motion
      (0,90) vertical edge
      (any value, 0) flicker;
    */
    Gauss2ndDerivParams(float const & theta, float const & phi);
      
    float alpha;
    float beta;
    float gamma;
};
  
// ###################################################################### 
// Spatio-temporal filter basis set
// ######################################################################
class SpatioTemporalEnergy
{
  public:
    //! Constructor
    SpatioTemporalEnergy();

    //! Constructor
    SpatioTemporalEnergy(std::vector<Gauss2ndDerivParams> const & params);
      
    //! Destructor
    ~SpatioTemporalEnergy();

    //! setup the filters
    void setupFilters(std::vector<Gauss2ndDerivParams> const & params);

    //! setup the filters
    void setupFilters();

    //! Add a new image and get the current output
    ImageSet<float> getNextOutput(Image<float> const & img);

    //! Add a new image and get the desired filter responses using the specified
    //! thread worker. The worker must implement a "void
    //! push(std::vector<std::function<void()> > job)" function that blocks
    //! until all vector of functions has been compeleted
    template <class T> 
    ImageSet<float> getNextOutput(Image<float> const & img, T & thread_worker);
 
    //! get the current output using another systems already computed basis
    ImageSet<float> getNextOutput(SpatioTemporalEnergy const & ste);
    
  private:      
    //get the response
    ImageSet<float> getResponse() const;

    //get a weighted sum
    Image<float> weightedSum(ImageSet<float> const & imgset, std::vector<float> const & weights) const;

    //filter the current queue with 'filter'
    Image<float> filter3D(SeperableFilter3D const & filter) const;
      
    ImageSet<float> itsTimeQ;//hold a queue of images      
      
    std::vector<SeperableFilter3D> itsFilters; //filters
    std::vector<SeperableFilter3D> itsFiltersH;//filters
      
    ImageSet<float> itsBasisSet;//steerable basis set
    ImageSet<float> itsBasisSetH;//steerable basis set
      
    std::vector<std::vector<float> > itsWeights; //weights
    std::vector<std::vector<float> > itsWeightsH;//weights
};

// ######################################################################
// template implementation
// ######################################################################
template <class T> 
ImageSet<float> SpatioTemporalEnergy::getNextOutput(Image<float> const & img, T & thread_worker)
{
  //update the queue
  if (itsTimeQ.size() == 0)
    itsTimeQ = ImageSet<float>(9, img.getDims(), ZEROS);
  
  itsTimeQ.push_front(img);
  itsTimeQ.pop_back();
  
  std::vector<std::function<void()> > jobs;

  //get the response from our filter set, normal filter first
  uint fcount = 0;
  for (std::vector<SeperableFilter3D>::iterator filter = itsFilters.begin(); filter != itsFilters.end(); ++filter)    
  {
    std::function<void()> f = [this, fcount, filter](){itsBasisSet[fcount] = filter3D(*filter);};
    jobs.push_back(f);
    ++fcount;
  }
  
  //then with the hilbert transformed filters
  fcount = 0;
  for (std::vector<SeperableFilter3D>::iterator filter = itsFiltersH.begin(); filter != itsFiltersH.end(); ++filter)
  {
    std::function<void()> f  = [this, fcount, filter](){itsBasisSetH[fcount] = filter3D(*filter);};
    jobs.push_back(f);
    ++fcount;
  }

  //wait for jobs to complete
  thread_worker.push(std::move(jobs));

  return getResponse();
}

#endif
#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

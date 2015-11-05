/*!@file Surprise/SurpriseMap.H a surprise map */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Surprise/SurpriseMap.H $
// $Id: SurpriseMap.H 10147 2008-08-27 03:01:00Z mundhenk $
//

#ifndef SURPRISEMAP_H_DEFINED
#define SURPRISEMAP_H_DEFINED

#define SUP_STD_MODEL 1
#define SUP_COV_MODEL 2

#include "Image/Image.H"
#include "Image/ImageCache.H"
#include "Surprise/SurpriseImage.H"
#include "Surprise/SurpriseModel.H"
#include <vector>
#include <typeinfo>

// ######################################################################
//! A Surprise Map
/*! This class contains a SurpriseImage of our current models at each
  pixel, plus an ImageCache of SurpriseImage objects built to
  represent the neighborhood distributions. */
template <class T>
class SurpriseMap
{
public:
  //! Constructor; you must call init() before using
  SurpriseMap();

  //! initialization
  /*! @param qlen number of maps
    @param updatefac factor by which the current model is combined with
    a new model on an update. Valid values are in [0..1]
    @param neighupdatefac factor by which local data is combined with
    neighborhood models, or 0.0 to use the value of updatefac
    @param sampleval the value of the samples previously received.
    @param samplevar a variance expressing the intrinsic variability
    in the measurement of those samples.
    @param neighsigma sigma to use for the neighborhoods as a fraction of
    max(map width, map height)
    @param locsigma sigma to use for local neighborhood, in pixels */
  void init(const uint qlen = 1, const double updatefac = 0.7,
            const double neighupdatefac = 0.7,
            const double sampleval = 0.0, const double samplevar = 1.0,
            const float neighsigma = 0.5f, const float locsigma = 3.0f,
            const Point2D<int>& probe = Point2D<int>(-1, -1),
            const double slfac = 1.0, const double ssfac = 0.1,
            const SU_KL_BIAS = SU_KL_NONE,
            const bool takeSTMax = false);

  //! Virtual destructor ensures proper destruction of derived classes
  ~SurpriseMap();

  //! Reset to initial state
  void reset();

  //! reset and create empty surprise models for temporal surprise
  void initModels(const SurpriseImage<T>& sample,
                  const bool setBias = false);

  //! Update by receiving a new sample and sample measurement error
  /*! The returned image is the surprise (combined temporal and
    spatial for each map, then combined over all maps) resulting from
    that update. NOTE: SurpriseModelPM has a special implementation, other
    surprise models follow the generic template implementation. */
  Image<double> surprise(const SurpriseImage<T>& sample);

  //! Access to underlying SurpriseImage members:
  const SurpriseImage<T>& getSurpriseImage(const uint index) const;

private:
  std::vector< SurpriseImage<T> > itsModels;
  uint itsQlen;
  T itsInitialModel;
  float itsNeighSigma, itsLocSigma;
  Image<float> itsNweights;
  float itsNWmin;
  double itsNeighUpdFac;
  Point2D<int> itsProbe;
  double itsSLfac, itsSSfac;
  SU_KL_BIAS itsJointKLBiasType;
  bool itsTakeSTMax;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

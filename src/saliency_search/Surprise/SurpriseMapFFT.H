/*!@file Surprise/SurpriseMapFFT.H a surprise map */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Surprise/SurpriseMapFFT.H $
// $Id: SurpriseMapFFT.H 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef SURPRISEMAPFFT_H_DEFINED
#define SURPRISEMAPFFT_H_DEFINED

#ifdef HAVE_FFTW3_H

#define FFT_TIME_SLICE 32

#include "Image/Image.H"
#include "Image/ImageCache.H"
#include "Surprise/SurpriseImage.H"
#include "Surprise/SurpriseModel.H"
#include <vector>
#include <math.h>
#include "Image/FFTWWrapper.H"

// ######################################################################
//! A Surprise Map
/*! This class contains a SurpriseImage of our current models at each
  pixel, plus an ImageCache of SurpriseImage objects built to
  represent the neighborhood distributions. */
template <class T>
class SurpriseMapFFT
{
public:
  //! Constructor; you must call init() before using
  SurpriseMapFFT();
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
            const double sfsfac = 1.0, const double sfpfac = 1.0,
            const std::string& descrName = "blank",
            const std::string& tagName = "blank");

  //! Virtual destructor ensures proper destruction of derived classes
  ~SurpriseMapFFT();

  //! Reset to initial state
  void reset();

  //! Update by receiving a new sample and sample measurement error
  /*! The returned image is the surprise (combined temporal and
    spatial for each map, then combined over all maps) resulting from
    that update */
  Image<double> surprise(const SurpriseImage<T>& sample,
                         const Image<double>& inputI,
                         const Image<double>& var);

  //! Access to underlying SurpriseImage members:
  const SurpriseImage<T>& getSurpriseImage(const uint index) const;

  //! Access to underlying SurpriseImage members for FFT spectal mag
  const SurpriseImage<T>& getSurpriseImageSFFT(const uint i) const;
  //! Access to underlying SurpriseImage members for FFT phase
  const SurpriseImage<T>& getSurpriseImagePFFT(const uint i) const;

  void setFFTModels(const SurpriseImage<T>& models,
                    const Image<float>& weights,
                    const float wmin,
                    std::vector<Image<double> >& sfs,
                    std::vector<Image<double> >& sfp);


private:
  std::vector<SurpriseImage<T> >  itsModels;
  std::vector<SurpriseImage<T> >  itsFFTSModels;
  std::vector<SurpriseImage<T> >  itsFFTPModels;
  Image<double> itsVarianceImage;
  uint itsQlen;
  T itsInitialModel;
  float itsNeighSigma, itsLocSigma;
  Image<float> itsNweights;
  float itsNWmin;
  double itsNeighUpdFac;
  double itsVariance, itsUpdatefac;
  Point2D<int> itsProbe;
  double itsSLfac, itsSSfac,  itsSFSfac, itsSFPfac;
  std::string itsDescrName, itsTagName;
  //! Store each frame over t iterations (usually a second) for fft
  std::deque<Image<double> > fftStack;
  //! create octave bins for fft to keep model number smaller
  std::vector<unsigned char> fftBinSize;
  //! keep track of the size of fftStack, tells when to start computing fft
  unsigned char fftStackCount;
  //! how many fft octave bins do we have?
  unsigned char fftBins;
  //! when fftStackCount is the right size this is set to true
  bool fftStackReady;
  fftw_complex *in, *out;
  fftw_plan p;
  //! a basic counter for this class
  unsigned int itsCounter;
};

#endif // HAVE_FFTW3_H

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif

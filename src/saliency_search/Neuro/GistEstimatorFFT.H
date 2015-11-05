/*!@file Neuro/GistEstimatorFFT.H Extract gist of image using the Fourier Transform */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
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
// Primary maintainer for this file: Christian Siagian <siagian at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/GistEstimatorFFT.H $
// $Id: GistEstimatorFFT.H 11797 2009-10-06 05:35:58Z kai $
//

// ######################################################################
/*! Extract gist of image using the Fourier Transform                  */

#ifndef NEURO_GISTESTIMATORFFT_H_DEFINED
#define NEURO_GISTESTIMATORFFT_H_DEFINED

#include "Image/Image.H"
#include "Neuro/GistEstimator.H"
#include "Neuro/VisualCortex.H"
#include "Image/FFTWWrapper.H"
#include "GUI/XWinManaged.H"

class GistEstimatorFFT : public GistEstimatorAdapter
{
public:
  //! Constructor
  GistEstimatorFFT(OptionManager& mgr,
                   const std::string& descrName = "GistEstimatorFFT",
                   const std::string& tagName = "GistEstimatorFFT");

  //! Destructor
  virtual ~GistEstimatorFFT();

  //! get the gist features
  void computeGistFeatureVector(Image<float> img);

  //! visualization of the Fourier Transform of the input image
  Image<float> getFftImage();

  //! return the gist feature values
  virtual Image<double> getGist();

protected:
  //! Callback for when a new input (retina) frame is available
  SIMCALLBACK_DECLARE(GistEstimatorFFT, SimEventRetinaImage);

private:
  void setupFFTW(int w, int h);
  void setupGaborMask(int w, int h);
  void fftCompute(Image<float> img, double **fftOut);

  void computeFFTgabor(const float stddev, const float period,
                       const float theta, uint w, uint h,
                       double **gm);

  Image<float> getFftImage(double **res, int w, int h);
  Image<float> normalize(Image<float> img);

  //! gist feature vector
  Image<double> itsGistVector;

  //! its FFTW wrappers
  FFTWWrapper *itsFftw;
  double *itsFftwBuffer;
  double **itsOutFftwBuffer;

  double ****gaborMask;
  Image<float>** gaborMaskImg;
  float itsStddev;

  Image<float> itsFftImage;
  XWinManaged* xw;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_GISTESTIMATORFFT_H_DEFINED

/*!@file Neuro/GistEstimatorStd.H extract estimated gist
         using available features of the image */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/GistEstimatorStd.H $
// $Id: GistEstimatorStd.H 14596 2011-03-14 17:32:24Z sophie $
//

// ######################################################################
/*! Extract gist of image                                               */

#ifndef GISTESTIMATORSTD_H_DEFINED
#define GISTESTIMATORSTD_H_DEFINED

#include "Neuro/GistEstimator.H"
#include "Neuro/VisualCortex.H"
#include "Media/MediaSimEvents.H"

#include "Component/ModelComponent.H" //s
#include "Component/RawGistEstimatorStd.H"//s


class RawGistEstimatorStd;

//! implementation of the gist estimation procedure
//! yet to be published
class GistEstimatorStd : public GistEstimatorAdapter
{
public:
  //! Constructor
  GistEstimatorStd(OptionManager& mgr,
                   const std::string& descrName = "Gist Estimator",
                   const std::string& tagName = "gistestimator");

  //! Destructor
  virtual ~GistEstimatorStd();

  //! return the gist feature values
  virtual Image<double> getGist();

  //! Get started. See ModelComponent.
  virtual void start1();



protected:
  //! Callback for when new data is available from VisualCortex
  SIMCALLBACK_DECLARE(GistEstimatorStd, SimEventVisualCortexOutput);

  nub::ref<RawGistEstimatorStd> itsRawGistEstimStd;


private:
  // get the gist features
  void getFeatureVector();

  // the visual cortex hook
  VisualCortex* itsVC;
  
  // gist feature vector
  Image<float> itsGistVector;
};

#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

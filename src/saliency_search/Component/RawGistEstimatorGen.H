/*!@file Component/RawGistEstimatorGen.H extract estimated gist
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
// Primary maintainer for this file: Chin-Kai Chang <chinkaic@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/RawGistEstimatorGen.H $
// $Id: RawGistEstimatorGen.H $
//

// ######################################################################
/*! Extract gist of image                                               */

#ifndef RAWGISTESTIMATORGEN_H_DEFINED
#define RAWGISTESTIMATORGEN_H_DEFINED

#include "Channels/ChannelMaps.H"
#include "Image/DrawOps.H"
#include "Neuro/gistParams.H"
#include "Neuro/GistEstimator.H"
#include "Neuro/VisualCortex.H"
#include "Media/MediaSimEvents.H"
#include "Component/ModelComponent.H" 
#include "Image/LevelSpec.H"
#include "Component/RawGistEstimatorHelper.H"

//! implementation of the gist estimation procedure
//! yet to be published
class RawGistEstimatorGen: public ModelComponent //GistEstimatorAdapter //s
{
public:
  //! Constructor
  RawGistEstimatorGen(OptionManager& mgr,
                      const std::string& descrName = "Raw Gist Estimator",
                      const std::string& tagName = "rawgistestimator");
  
  //! Destructor
  virtual ~RawGistEstimatorGen();
  
  //! Update the gist feature vector
  void computeFeatureVector(rutz::shared_ptr<ChannelMaps> chm);
  
  Image<float> compute(rutz::shared_ptr<ChannelMaps> chm);
  
  std::vector<float> computeOnPolygon(rutz::shared_ptr<ChannelMaps> chm, std::vector<Point2D<float> > polygon); //before Image<float>
  
  //! get the gist display of the current gist
  Image<float> getGistImage
  (int sqSize,
   const float minO = -1.0, const float maxO = -1.0,
   const float minC = -1.0, const float maxC = -1.0,
   const float minI = -1.0, const float maxI = -1.0);
  
  //! return the similarity measure
  //! between 2 sets of gist features
  Image<double> diffGist(Image<double> in);
 
protected:
  
private:
  //! LevelSpec used by our channels, used to compute output dims
  OModelParam<LevelSpec> itsLevelSpec;

// use the center surround or not or both
  OModelParam<int> itsUseCS;  
  // the visual cortex hook
  VisualCortex* itsVC;
  
  // gist feature vector
  Image<double> itsGistVector;

};

#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

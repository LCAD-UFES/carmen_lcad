/*!@file Neuro/ShapeEstimator.H Estimate the shape/size of an attended object */

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
// Primary maintainer for this file: Dirk Walther <walther>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/ShapeEstimator.H $
// $Id: ShapeEstimator.H 14857 2011-07-26 01:14:21Z siagian $
//

#ifndef SHAPEESTIMATOR_H_DEFINED
#define SHAPEESTIMATOR_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Simulation/SimModule.H"
#include "Image/Image.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/ShapeEstimatorModes.H"
#include "Simulation/SimEvents.H"
#include <string>
#include "Image/NamedImage.H"

#include "Gist/SalientRegionSegmenter.H" 
#include "Gist/CenterSurroundHistogramSegmenter.H" 

#include "GUI/XWinManaged.H"

class VisualCortex;
class SpatialMetrics;

//! everything related to the shape estimation procedure
/*! class to take care of all things related to the shape estimation
  procedure published in Walther et al, BMCV 02 */
class ShapeEstimator : public SimModule
{
public:

  //! Constructor
  /*! @param vcx this is the hook for the class to obtain its information
    on channels, submaps etc. */
  ShapeEstimator(OptionManager& mgr,
                 const std::string& descrName = "Shape Estimator",
                 const std::string& tagName = "shapeestimator",
                 const nub::soft_ref<VisualCortex> vcx =
                 nub::soft_ref<VisualCortex>());

protected:
  //! Callback for when a new WTA winner is available
  SIMCALLBACK_DECLARE(ShapeEstimator, SimEventWTAwinner);

  //! Callback for every time we should save our outputs
  SIMCALLBACK_DECLARE(ShapeEstimator, SimEventSaveOutput);

  //!< metrics that depend on the input size:
  nub::ref<SpatialMetrics> itsMetrics;

  //! Text log file name
  OModelParam<std::string> itsLogFile;

  //! Determines what information is used as a source for the shape estimation
  /*! possible values:<ul><li>"None" - shape estimator is not used;
    <li>"FeatureMap" - the shape is extracted from the winning Feature Map;
    <li>"ConspicuityMap" - shape is extracted from the winning Conspicuity Map;
    <li>"SaliencyMap" - the shape is extracted from the Saliency Map</ul>*/
  OModelParam<ShapeEstimatorMode> itsMode;

  //! Determines Smoothing Method used to extract a map in image coordinates
  /*! possible values:<ul>
    <li>"None" - no smoothing, the result is just scaled up in blocks;
    <li>"Gaussian" - smoothing by convolving with a large 2D Gaussian kernel;
    <li>"Chamfer" - smoothing with Opening and Chamfering (takes about half
    the time compared to "Gaussian"</ul>*/
  OModelParam<ShapeEstimatorSmoothMethod> itsSmMethod;

  //! Save our internals?
  OModelParam<bool> itsSaveObjMask;

  //! use a larger neighborhood when tracking down a local max?
  OModelParam<bool> itsUseLargeNeigh;

  //! Reset ShapeEstimator
  /*! See the base function in ModelComponent.H for info. */
  virtual void reset1();

private:
  //! Locate a local max in submap (any size smaller than input size)
  //! around winner (given in coordinates of an image of dims indims)
  //! and returns the matching coordinates in submap's resolution as
  //! well as the value at the max.
  float locateLocalMax(const Image<float>& submap,
                       const Point2D<int>& winner,
                       const Dims& fulldims,
                       Point2D<int>& winloc,
                       int mini = -1, int maxi = 1);

  //! estimate and post region shape
  //! using the various original saliency model features 
  void postSaliencyFeatureSimEventShapeEstimatorOutput
  (SimEventQueue& q, rutz::shared_ptr<SimEventWTAwinner>& e);

  //! estimate and post region shape
  //! using motion cues
  void postMotionSimEventShapeEstimatorOutput
  (SimEventQueue& q, rutz::shared_ptr<SimEventWTAwinner>& e);

  //! estimate and post region shape
  //! using contour boundaries around the most salient point
  void postContourBoundarySimEventShapeEstimatorOutput
  (SimEventQueue& q, rutz::shared_ptr<SimEventWTAwinner>& e);

  //! get the name of the winning map
  std::string getWinningFeatureMapName
  (Point2D<int> winner, Dims indims, rutz::shared_ptr<ChannelMaps> chm);

  //! correct the salient point location away from contours
  Point2D<int> correctSalientPointLocation
  (Point2D<int> pt, std::string name, Image<float> salMap);
    
  //! estimate and post region shape
  //! using adaptive center surround histogram difference
  void postCSHistogramSimEventShapeEstimatorOutput
  (SimEventQueue& q, rutz::shared_ptr<SimEventWTAwinner>& e);

  //! shared function to create and then post object map 
  void postSimEventShapeEstimatorOutput
  (NamedImage<float> winmap, Point2D<int> winloc, 
   Point2D<int> winner, Dims indims, SimEventQueue& q);  


  // the structuring element for eroding and dilating
  // for the chamfer smoothing method
  Image<byte> structEl;

  // keep a copy of our last smooth mask so we can save it
  Image<float> itsSmoothMask;

  // our cumulative smooth mask
  Image<float> itsCumMask;

  //! contour based salient region segmenter
  rutz::shared_ptr<SalientRegionSegmenter> itsSalientRegionSegmenter;

  //! center surround histogram differencing
  //! shape estimator
  rutz::shared_ptr<CenterSurroundHistogramSegmenter> 
  itsCenterSurroundHistogramSegmenter;

  //! debug window
  rutz::shared_ptr<XWinManaged> itsWin;

  //! the stem name to save the intermediate files
  std::string itsInputImageStem;
};

#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/*!@file Robots2/Beobot2/Navigation/FOE_Navigation/FoeDetector.H 
  detects an FOE from the input image sequence */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Navigation/FOE_Navigation/FoeDetector.H
// $Id: $
//

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

#include "Image/Image.H"
#include "Image/Layout.H"

#include "Robots/Beobot2/Navigation/FOE_Navigation/SpatioTemporalEnergy.H"
#include "Robots/Beobot2/Navigation/FOE_Navigation/MiddleTemporal.H"
#include "Robots/Beobot2/Navigation/FOE_Navigation/MotionOps.H"
#include "Raster/Raster.H"

#ifndef ROBOTS_BEOBOT2_NAVIGATION_FOENAVIGATION_FOEDETECTOR_H
#define ROBOTS_BEOBOT2_NAVIGATION_FOENAVIGATION_FOEDETECTOR_H

#define FOE_METHOD_TEMPLATE    10000
#define FOE_METHOD_AVERAGE     10001

class FoeDetector: public ModelComponent
{
public:

  FoeDetector(OptionManager& mgr,
                const std::string& descrName = "FoeDetector",
                const std::string& tagName = "FoeDetector");

  void reset(uint numPyrLevel, uint numDirs, uint numSpeeds);

  ~FoeDetector();

  //! various ways to get an estimate of focus of expansion
  //! point or the probability map
  //! Note: if new features/image is inputted
  //! FOE will be recomputed using those features
  //! otherwise it will return the current information 
  //! without computation

  //! compute and return FOE location given an input image
  Point2D<int> getFoe(Image<byte> lum, 
                      uint method = FOE_METHOD_TEMPLATE,
                      bool tempFilter = true);

  //! compute and return FOE map given an input image
  Image<float> getFoeMap(Image<byte> lum, 
                         uint method = FOE_METHOD_TEMPLATE,
                         bool tempFilter = true);

  //! compute and return FOE location given a set of correspondences
  Point2D<int> getFoe
  ( rutz::shared_ptr<OpticalFlow> flow,
    uint method = FOE_METHOD_TEMPLATE,
    bool tempFilter = true);

  //! compute and return FOE location given a set of correspondences
  Image<float> getFoeMap
  ( rutz::shared_ptr<OpticalFlow> flow,
    uint method = FOE_METHOD_TEMPLATE,
    bool tempFilter = true);
  
  //! compute and return FOE location given MT features
  //! use MiddleTemporal.C & .H to compute it
  Point2D<int> getFoe
  ( std::vector <Image<float> > mtFeatures,
    std::vector <Image<float> > mtOptimalShift,
    uint method = FOE_METHOD_TEMPLATE, 
    bool tempFilter = true);
  
  //! compute and return FOE map given MT features
  //! use MiddleTemporal.C & .H to compute it
  Image<float> getFoeMap
  ( std::vector <Image<float> > mtFeatures,
    std::vector <Image<float> > mtOptimalShift,
    uint method = FOE_METHOD_TEMPLATE,
    bool tempFilter = true);
  
  //! return the current information without computation
  Point2D<int> getFoe();
  Image<float> getFoeMap();

  //! hard-code input observer motion
  void setObserverRotation(uint dir, float speed);
  void setObserverRotation(float di, float dj);

  //! return displayable result of FOE computation
  Layout<byte> getMTfeaturesDisplay(Image<byte> image);

private:

  //! internal functions for different types of FOE computation 
  Point2D<int> getFoeTemplate(Image<byte> lum);
  Point2D<int> getFoeAvg(Image<byte> lum);

  //! temporally filter the location of the FOE
  //! as images are inputted
  Point2D<int> temporalFilterFoeComputation();

  //! reset direction weights for FOE calculation
  //! at the MST level
  void resetDirectionWeights(uint width, uint height);

  //! compute the visual cortex features related to motion
  //! this is if the input is in series of image
  //! if input is MT features or correspondences
  //! this step is skipped
  void computeV1features();

  //! detect whether observer is stationary
  //! FIXXX_NOTE: need some work
  float detectObserverStationarity();

  //! detect observer rotation correct for FOE templates
  //! FIXXX_NOTE: need some work
  void detectObserverRotation();
  float maxMean(Image<float> image);

  //! correct FOE templates to account for observer rotation
  //! FIXXX_NOTE: need some work
  void correctForObserverRotation();

  //! compute the Focus of Expansion from the MT features
  //! using Perrone 1992 templates method 
  Point2D<int> computeFoeTemplate();
  float computeFoeTemplateValue(uint foeI, uint foeJ);
  Point2D<int> computeFoeTemplate(rutz::shared_ptr<OpticalFlow> flow);
  float computeFoeTemplateValue
  (uint foeI, uint foeJ, rutz::shared_ptr<OpticalFlow> flow);

  //! return direction weight
  float getDirectionWeight
  (Point2D<float> pt, Point2D<float> foe, float length, float mangle);
  float getDirectionWeight2(uint quad, uint dir);

  //! compute the Focus of Expansion from the MT features
  //! using Bonn 1994 vertical and horizontal averaging method 
  Point2D<int> computeFoeAverage();
  Point2D<int> computeFoeAverage(rutz::shared_ptr<OpticalFlow> flow);

  //! printing/displaying procedures
  //! for debugging
  void print(Image<float> img, 
             uint si, uint ei, uint sj, uint ej, bool stop);
  void display(Image<float> img, std::string info);
  void display(ImageSet<float> imgs, std::string info);



  //! the various directional pyrbuilders
  std::vector<std::vector
              <rutz::shared_ptr<SpatioTemporalEnergyPyrBuilder<float> > > >
  itsSpatioTemporalPyrBuilders; 

  //! its Medial Temporal module
  rutz::shared_ptr<MiddleTemporal> itsMT;

  std::vector<std::vector<std::vector<Image<float> > > > itsDirWeights;

  uint  itsNumPyrLevels;
  uint  itsNumDirs;
  uint  itsNumSpeeds;
  //uint  itsNumDFrames; // 1 pix in between, 2, 4 

  //! current image holder
  Image<byte>  itsCurrentImage;

  //! raw motion energy for each direction
  std::vector<std::vector<ImageSet<float> > > itsRawSpatioTemporalEnergy;
  std::vector<ImageSet<float> > itsSpatioTemporalEnergy;
  //std::vector<ImageSet<float> > itsSpatioTemporalEnergyOptimalShift;

  //! filtered: collapsed to the number of directions 
  //! this already includes:
  //!   lateral inhibition
  //!   center surround opponencies
  std::vector <Image<float> > itsMTfeatures;
  std::vector <Image<float> > itsMToptimalShift;

  //! map of likelihood 
  //! that a coordinate location is the focus of expansion
  Image<float> itsFoeMap; 
  std::vector<Image<float> > itsRecentFoeMaps;
  int itsCurrentFoeMapIndex;

  //! most likely focus of expansion location
  Point2D<int> itsFoe;

  //! estimated current observer motion
  //! can (or have been) be used to adjust FOE 
  uint  itsCurrentRotMotionDir;
  float itsCurrentRotMotionSpeed;
  float itsCurrentRotMotionDi;
  float itsCurrentRotMotionDj;

  //! debugging window
  rutz::shared_ptr<XWinManaged> itsWin;
};
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

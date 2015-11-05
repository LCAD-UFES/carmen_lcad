/*!@file Gist/SalientRegionSegmenter.H segment out object depicted by the
salient point. Here we use both region growing and boundary detection
 */
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
// Primary maintainer for this file: Christian Siagian <siagian@caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/SalientRegionSegmenter.H $
// $Id: $
//
//////////////////////////////////////////////////////////////////////////

#ifndef SALIENT_REGION_SEGMENTER
#define SALIENT_REGION_SEGMENTER

#include "Gist/SuperPixel.H"  // FIXXX: need to change to GraphBasedSegmenter
#include "Gist/ContourBoundaryDetector.H"
#include "SIFT/Histogram.H"
#include <list>

struct Segment
{
  Segment() 
  { 
    orgIndex      = -1;
    contourIndex  = -1;
    segStartIndex = -1;
    segEndIndex   = -1;

    pVal   = 0.0;
    lenVal = 0.0;
    simVal = 0.0;
    strVal = 0.0;
    ratVal = 0.0;
    disVal = 0.0;
  };

  Segment(const int inOrgIndex,
          const int inContourIndex,
          const int inSegStartIndex,
          const int inSegEndIndex) :
    orgIndex     (inOrgIndex),
    contourIndex (inContourIndex),
    segStartIndex(inSegStartIndex),
    segEndIndex  (inSegEndIndex)
  {
    pVal   = 0.0;
    lenVal = 0.0;
    simVal = 0.0;
    strVal = 0.0;
    ratVal = 0.0;
    disVal = 0.0;
  }

  int  orgIndex;      //!< original index 
  uint contourIndex;  //!< contour index
  uint segStartIndex; //!< segment start index
  uint segEndIndex;   //!< segment end index

  float strength;     //!< raw, un-normalized strength of the segment boundary 

  float pVal;         //!< total priority value
  float lenVal;       //!< length of the contour boundary 
  float simVal;       //!< similarity with the closer side 
  float strVal;       //!< strength of boundary
  float ratVal;       //!< ratio of in:out
  float disVal;       //!< distance between point and middle of segment

  // we flip the sign because want non-ascending order
  bool operator < (const Segment& other)
  {
    return pVal > other.pVal;
  }
};



//! salient region segmenter
//! which utilizes boundary detector and graph based region segmentation
class SalientRegionSegmenter
{
public:

  // ######################################################################
  //! @name Constructor, assigment and destructor
  //@{

  //! constructor
  SalientRegionSegmenter();

  //! destructor
  ~SalientRegionSegmenter();

  //! set the input image
  void setImage
  (Image<PixRGB<byte> > image, std::string name = "");

  //! get the salient region given the salient point 
  Image<float> getSalientRegion(Point2D<int> pt);

  //@}

  // ######################################################################
  //! @name Access functions
  //@{

  //! image getter
  inline Image<PixRGB<byte> > getImage();

  //! current region assignment map getter
  inline Image<int> getCurrentRegionAssignmentMap();

  //! get the step size used for grid size
  inline uint getBoundaryStepSize();

  //! get the resulting object mask
  inline Image<PixRGB<byte> > getObjectMask(uint i);

  //! get the resulting binary salient region mask
  inline Image<byte> getSalientRegionMask(uint i);

  //@}

private:

  //! original input image
  Image<PixRGB<byte> > itsImage;

  //! contour boundary detector
  rutz::shared_ptr<ContourBoundaryDetector> 
  itsContourBoundaryDetector;

  //! graph based segmenter
  //rutz::shared_ptr<GraphBasedSegmenter> 
  //  itsGraphBasedSegmenter;
  
  //! the current salient region mask
  //! in original resolution
  std::vector<Image<byte> >          itsSalientRegionMask;
  std::vector<Image<PixRGB<byte> > > itsObjectMask;

  //! the current region assignment map
  Image<int> itsCurrentRegionAssignmentMap;

  //! the current number of salient regions
  int itsCurrentNumSalientRegions;

  //! the final boundary image
  Image<float> itsBoundaryImage;

  //! list of salient points
  //! inputted for the current image
  std::vector<Point2D<int> > itsSalientPoints;

  //! the initial group of oversegmented regions
  Image<int> itsInitialRegionImage;

  //! the initial group of oversegmented regions
  //! in vectors of points
  std::vector<std::vector<Point2D<int> > > itsInitialRegions;

  //! the initial group of oversegmented regions
  //! in image but with average color of region
  Image<PixRGB<byte> > itsInitialRegionColorImage;

  //! the initial region adjecency list
  std::vector<std::vector<uint> > itsRegionAdjecencyList;

  //! the list of contour boundaries
  std::vector<rutz::shared_ptr<Contour> > itsContourBoundaries;

  //! the list of edgels
  Image<rutz::shared_ptr<Edgel> > itsEdgels;

  //! store the maximum edgel strength
  float itsMaximumEdgelStrength;

  //! list of histogram entries 
  //! for various image features:
  //! FIXXX_NOTE: currently just brightness and color (from CIELab)
  std::vector<Image<int> > itsImageHistogramEntries;

  //! smoothing kernel for histogram smoothing
  Image<float> itsLKernel;
  Image<float> itsAKernel;
  Image<float> itsBKernel;

  //! store all the already computed histograms 
  Image<rutz::shared_ptr<Histogram> > itsImageFeatureHistograms;
 
  //! debug window
  rutz::shared_ptr<XWinManaged> itsWin;

  uint itsConsAssignedCount;
  uint itsIteration;

  std::string itsInputImageStem;

  uint itsHistogramGenerationCountPt;
  uint itsHistogramGenerationCountPts;
  uint64 itsTimePt;
  uint64 itsTimePts;
  uint64 itsTimeCompare;

  // ######################################################################
  //! @name Compute functions (all the inner-working functions)
  //@{

  //! compute the boundaries and initial over-segmented regions 
  void computeBoundaryAndRegionInformation();

  //! compute the contour boundaries related information
  void computeBoundaryInformation();

  //! compute the initial over-segmented regions
  void computeRegionInformation();

  //! compute the region adjecency list
  std::vector<std::vector<uint> > getRegionAdjecencyList();

  //! display the region adjecency
  void displayRegionAdjecencyList();

  //! display the boundaries and over-segmented regions   
  void displayBoundaryAndRegionInformation();

  //! the main function 
  //! to estimate the salient region
  Image<float> computeSalientRegion();

  //! fill the segment with contour boundary segments 
  //! sorted of most likely fit
  std::list<Segment> fillSegmentQueue(Point2D<int> pt);

  //! set the priority for this segment
  bool setPriority(Segment &segment, Point2D<int> pt);
  
  //! get histogram distribution around the point
  rutz::shared_ptr<Histogram> 
  getHistogramDistribution(Point2D<int> pt);

  //! get histogram distribution from a list of points
  rutz::shared_ptr<Histogram> 
  getHistogramDistribution (std::vector<Point2D<int> > pts);

  //! get point distribution around a segment
  //! dist1 always the one closer to the point pt
  void getSegmentPointDistributions
  (Segment segment, Point2D<int> pt, 
   std::vector<Point2D<int> > &pts1, 
   std::vector<Point2D<int> > &pts2);

  //! compute the image feature histogram values
  void setImageFeatureHistogramValues();

  //! quantize values to histogram bins
  Image<int> quantize_values
  (Image<float> image, int num_bins, bool normalize = false);

  //! filter out invalid segments 
  void filterSegments
  (std::list<Segment> &segments, 
   Point2D<int> ptGrid, Image<int> assignmentMap);

  //! adding unvisited neighbors
  void addUnvisitedNeighbors
  (std::vector<Point2D<int> > &currentUnassignedLocations, 
   Point2D<int> currPoint, 
   Image<int> &assignmentMap);

  //! PAMI2012  imagine contours and use them to grow contours
  bool checkUnvisitedNeighborsNeigbor
  (Point2D<int> currPointGrid, Image<int> &assignmentMap,
   std::vector<Point2D<int> > &currentBackgroundLocations,
   rutz::shared_ptr<Histogram> histA,
   rutz::shared_ptr<Histogram> histBobj,
   rutz::shared_ptr<Histogram> histBbg,
   rutz::shared_ptr<Histogram> histCobj,
   float borderDiff, float ratio, float threshDiff2);

  //! get the pixels connecting the passed in points
  std::vector<Point2D<int> > getLine
  (Point2D<int> p1, Point2D<int> p2);

  //! check if the two points go through a region
  //! assigned the passed in region label
  bool throughRegion
  (Point2D<int> ptGrid1, Point2D<int> ptGrid2, 
   Image<int> assigmentMap, int regionLabel);

  //! assign regions next to the segment
  void assignRegionsAroundSegment
  (Segment segment, 
   std::vector<Point2D<int> > pts2, 
   std::vector<Point2D<int> > pts3, 
   Image<int>                 &assignmentMap,
   Image<int>                 &coreAssignmentMap,
   std::vector<Point2D<int> > &currentObjectLocations,
   std::vector<Point2D<int> > &currentBackgroundLocations,
   std::vector<Point2D<int> > &currentUnassignedLocations,
   std::vector<Point2D<int> > &currentIncludedEdgelLocations,
   Image<int>       &regionAssignmentMap,
   std::vector<int> &currentObjectRegionList, 
   std::vector<int> &currentBackgroundRegionList);

  Point2D<int> evaluateCoreAssignment
  (std::vector<Point2D<int> > currentObjectLocations, 
   Image<int> assignmentMap, 
   Image<int> &coreAssignmentMap,
   uint       &index);

  //! grow the surrounds
  void growSurround
  (Image<int>                 &assignmentMap,
   std::vector<Point2D<int> >  currentObjectLocations,
   std::vector<Point2D<int> > &currentBackgroundLocations,
   std::vector<Point2D<int> > &currentUnassignedLocations,
   std::vector<Point2D<int> >  currentIncludedEdgelLocations,
   Image<int>                  coreAssignmentMap);

  //! imagine contours and use them to grow
  void growUsingImaginedContours
  (Image<int>                 &assignmentMap,
   std::vector<Point2D<int> >  currentObjectLocations,
   std::vector<Point2D<int> > &currentBackgroundLocations,
   std::vector<Point2D<int> > &currentUnassignedLocations,
   std::vector<Point2D<int> >  currentIncludedEdgelLocations,
   Image<int>                  coreAssignmentMap);


  //! check if point is closer 
  //! to any of the points in the background
  bool checkBackgroundRatio
  (float ptDiff, rutz::shared_ptr<Histogram> ptHist, Point2D<int> ptGrid,
   Image<int> assignmentMap,
   std::vector<Point2D<int> > currentBackgroundLocations,
   std::vector<Point2D<int> > currentIncludedEdgelLocations);
    
  //! set the thin boundary 
  void setBoundaryImage
  (Image<int> assignmentMap, Image<float> mask);

  //! display the distribution setup
  void displayDistributionSetup
  (Segment segment, Point2D<int> pt, 
   std::vector<Point2D<int> > pts2, 
   std::vector<Point2D<int> > pts3);

  //! display segments
  void displaySegments(std::list<Segment> segments);

  //! display segment
  void displaySegment(Segment segment);

  //! display the current state of growing
  void displayGrowMap
  (Image<int> assignmentMap, Point2D<int> currPointGrid,
   std::vector<Point2D<int> > currentObjectLocations,
   std::vector<Point2D<int> > currentBackgroundLocations,
   std::vector<Point2D<int> > currentUnassignedLocations,
   std::vector<Point2D<int> > currentIncludedEdgelLocations,
   Image<bool> isComparedWith, Segment segment,
   Image<int>  regionAssignmentMap,
   Image<int>  coreAssignmentMap);
  
  //@}
};

// ######################################################################
// Implementation for SalientRegionSegmenter inline functions
// ######################################################################
inline Image<PixRGB<byte> > SalientRegionSegmenter::getImage()
{ return itsImage; }

inline Image<int> SalientRegionSegmenter::getCurrentRegionAssignmentMap()
{ return itsCurrentRegionAssignmentMap; }

inline uint SalientRegionSegmenter::getBoundaryStepSize()
{ return BOUNDARY_STEP_SIZE; }

inline Image<PixRGB<byte> > SalientRegionSegmenter::getObjectMask(uint i)
{ 
  ASSERT(i < itsObjectMask.size());
  return itsObjectMask[i]; 
}

inline Image<byte> SalientRegionSegmenter::getSalientRegionMask(uint i)
{ 
  ASSERT(i < itsSalientRegionMask.size());
  return itsSalientRegionMask[i]; 
}

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

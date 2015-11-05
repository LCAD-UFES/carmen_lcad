/*!@file Robots2/Beobot2/Navigation/BeoRoadFinder.H 
 find road using image */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Navigation/BeoRoadFinder.H
// $Id: $
//
//////////////////////////////////////////////////////////////////////////

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Util/Timer.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include <IceUtil/Thread.h>

#include "Robots/Beobot2/BeoCommon.H"

#include "Media/FrameSeries.H"
#include "Transport/FrameIstream.H"

#include "Image/Image.H"

#ifndef BEOBOT2_NAVIGATION_BEOROADFINDERI_H
#define BEOBOT2_NAVGIATION_BEOROADFINDERI_H

// ######################################################################

//! a segment is defined by the two end-points
struct Segment
{
  Segment(Point2D<int> in_p1,
          Point2D<int> in_p2,
          float in_angle,
          float in_length) :
    p1(in_p1),
    p2(in_p2),
    angle(in_angle),
    length(in_length)
  {
  }

  Point2D<int> p1;
  Point2D<int> p2;

  float angle;
  float length;

  bool operator < (const Segment& s)
  {
    return length < s.length;
  }
};

//! Keeps all the supporting information about
//! a specific vanishing point
struct VanishingPoint
{
  VanishingPoint(Point2D<int> in_vp,
                 float        in_likelihood):
    vp(in_vp),
    likelihood(in_likelihood)
  {
    supportingSegments.clear();
  }

  Point2D<int>         vp;
  float                prior;
  float                likelihood;
  float                posterior;
  std::vector<Segment> supportingSegments;
};

//! keeps all the ready to use information of a supporting line 
//! as it pertains to describing the road
struct Line
{
  Line()
  {
    length   = 0.0;
    angle    = 0.0;
    score    = 0.0;
    isActive = false;
    index    = -1;
  }

  //! basic information to specify the line
  float length;
  float angle;
  float score;

  //float theta;
  //float radius;

  //! the points that are fit to the line
  std::vector<Point2D<int> > points;

  //! quick information for various locations
  //! with respect to the road
  Point2D<float> horizonPoint;
  Point2D<float> horizonSupportPoint;
  Point2D<float> roadBottomPoint;
  Point2D<float> onScreenRoadBottomPoint;
  Point2D<float> onScreenHorizonPoint;
  Point2D<float> onScreenHorizonSupportPoint;

  //! original supporting segments
  //! out of sync after initial frame
  std::vector<Segment> segments;

  //! tracking information to monitor health of the line
  std::vector<float> scores;
  std::vector<float> start_scores; 

  //! tracks whether the line can be used for 
  //! finding the road center 
  bool isActive;
  float angleToCenter;
  Point2D<float> pointToServo;
  float offset;

  int index;
};

struct RoadModel
{
  std::vector<Line>            lines;
  std::vector<int>             lastActiveIndex;
  std::vector<Point2D<float> > lastSeenHorizonPoint;
  std::vector<Point2D<float> > lastSeenLocation;
  std::vector<int>             numMatches;
};

//! Navigation by finding road
class BeoRoadFinder : public RobotBrainComponent
{
public:

  //! constructor
  BeoRoadFinder(OptionManager& mgr,
		const std::string& descrName = "BeoRoadFinder",
		const std::string& tagName   = "BeoRoadFinder");

  //! desctructor
  ~BeoRoadFinder();

  //! evolve: thread that process the vanishing point location
  //!         using the full blown algorithm
  virtual void evolve();

  //! update message: estimate the vanishing point 
  //!                 using just the tracked lines
  virtual void updateMessage
  (const RobotSimEvents::EventMessagePtr& eMsg,
   const Ice::Current&);

  //! list of topics the BeoRoadFinder publish and subscribed to
  virtual void registerTopics();

  void start1();
  
private:

  //! compute canny edges
  Image<byte> getCannyEdge(Image<PixRGB<byte> > image, IplImage*& cvImage);

  //! compute the hough segments in the image
  void computeHoughSegments(IplImage *cvImage);

  //! main function to detect the road
  std::vector<Line> computeVanishingLines
  (Image<byte> edgeMap, Point2D<int> vanishingPoint);

  //! computes the road center point to servo to 
  Point2D<float> computeRoadCenterPoint
  (std::vector<Line>& lines, Point2D<int>& vanishing_point, 
   Point2D<float>& road_center_point, float& confidence);

  //! update the road model and the incoming lines
  //! NOTE: also change the line parameters to sync them
  //!       this avoids drifts
  void updateRoadModel(std::vector<Line> &lines, int index);

  //! estimate the vanishing point from the tracked lines
  Point2D<int> getVanishingPoint
  (std::vector<Line> lines, float &confidence);

  //! track vanishing lines by to fit to the new, inputted, edgemap
  void trackVanishingLines
  (Image<byte> edgeMap, std::vector<Line>& currentLines, bool print = false);

  //! get pixels that make up the segment defined by p1 and p2
  //! have added complexity to search within 1.5 pixels of the line
  std::vector<Point2D<int> >  
  getPixels(Point2D<int> p1, Point2D<int> p2, Image<byte> edgeMap);

  //! get pixels that make up the segment defined by p1 and p2
  //! have added complexity to search within 1.5 pixels of the line
  std::vector<Point2D<int> >  
  getPixels(Point2D<int> p1, Point2D<int> p2, Image<byte> edgeMap,
            std::vector<uint>& startIndexes);

  //! get pixels that make up the segment defined by p1 and p2
  std::vector<Point2D<int> >  
  getPixelsQuick(Point2D<int> p1, Point2D<int> p2, Image<byte> edgeMap);

  //! FIXXX not working.
  //! find lines given the found supporting segments
  Line findLine
  (Segment s, Image<byte> edgeMap, 
   std::list<Segment> supportingSegments, std::vector<bool>& is_used);

  //! find lines given the found supporting segments
  Line findLine2
  (Segment s, Image<byte> edgeMap, 
   std::list<Segment> supportingSegments, std::vector<bool>& is_used,
   float& totalLength, uint& numSegments);

  //! openCV wrapper function to fit a line to an input vector of points
  void fitLine
  (std::vector<Point2D<int> > points, Point2D<float>&p1,Point2D<float>&p2);

  //! compute how well the line equation fit the edgels in edgemap
  float getLineFitness
  (Point2D<int> horizonPoint, Point2D<int> roadBottomPoint, 
   Image<byte> edgeMap, bool printResult = false);

  //! compute how well the line equation fit the edgels in edgemap
  float getLineFitness
  (Point2D<int> horizonPoint, Point2D<int> roadBottomPoint, 
   Image<byte> edgeMap, std::vector<Point2D<int> >& points, 
   bool printResult = false);

  //! update the information in by updating the input points, score
  //! and various handy coordinate locations
  void updateLine(Line& l, std::vector<Point2D<int> > points,
                  float score);

  //! update the lines with the inputted set of edgemaps
  void projectForwardVanishingLines
  (std::vector<Line>& lines, std::vector<Image<byte> > edgeMaps);

  //! combine two lines sets, discard duplicates and overlaps
  std::vector<Line> combine
  (std::vector<Line> prevLines, std::vector<Line> currentLines);

  //! discard duplicate lines in a set
  std::vector<Line> discardDuplicates(std::vector<Line> currentLines);

  //! compute and publish the motor command using the target point
  void publishMotorCommand(Point2D<int> max_vp);

  //! compute and publish traversal map using the target point
  void publishTraversalMap
  (Point2D<int> max_vp, Point2D<float> road_center_point, int index);

  //! get a generic traversal map to go to the vanishing point
  Image<float> getTraversalMap();

  //! get the display image that describe the current state
  Image<PixRGB<byte> > getDisplayImage
  (Image<PixRGB<byte> > image, Image<byte> edgeMap,
   std::vector<Line> currentLines, double rHead);

  void publishScreenShot(Image<PixRGB<byte> > img);
  
  Image<PixRGB<byte> > itsCurrImage;             //!< current received image
  IceUtil::Mutex its_Current_Image_mutex;        //!< locking current input image 
  int    itsCurrImageID;                         //!< current received image ID
  int    itsProcImageID;                         //!< current processed image ID

  Image<byte> itsEdgeMap;                        //!< the current edge image
  IplImage *itsCVedgeMap;                        //!< the current openCV edge image
  
  //! the accumulated edgemaps during 
  //! the longer evolve recognition process
  std::vector<Image<byte> > itsEdgeMaps;

  //! the current road heading
  double itsRoadHeading;

  //! the accumulated trajectory
  Point2D<float> itsAccumulatedTrajectory;

  //! locking accunulated trajectory
  IceUtil::Mutex its_Acc_Traj_mutex;        

  //! current segments found using CVHoughlines
  std::vector<Segment> itsCurrentSegments;

  IceUtil::Mutex its_Line_Tracker_mutex;         //!< locking line trackers vars

  //! indicate whether tracking
  bool itsTrackingFlag;                         

  //! the current lines being tracked
  std::vector<Line> itsCurrentLines;

  //! indicate how many unique lines have been identified
  //! NOTE: never reset
  uint itsNumIdentifiedLines;
  
  RoadModel itsRoadModel;

  //! vanishing points being considered
  std::vector<VanishingPoint> itsVanishingPoints;


  IceUtil::Mutex its_Road_Information_mutex;     //!< locking road info vars
  Point2D<int>      itsVanishingPoint;           //!< current vanishing point
  Point2D<float>    itsCenterPoint;              //!< current center of road point 
  Point2D<float>    itsTargetPoint;              //!< target servo point
  float             itsVanishingPointConfidence; //!< current vanishing point 
                                                 //!< confidence value
  std::vector<bool> itsVanishingPointStability;  //!< vanishing point score tracker

  //! for display
  nub::soft_ref<OutputFrameSeries> itsOfs;

  //! timer
  Timer itsTimer;

  //! for visualizer
  int itsCurrentMessageID;
};
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

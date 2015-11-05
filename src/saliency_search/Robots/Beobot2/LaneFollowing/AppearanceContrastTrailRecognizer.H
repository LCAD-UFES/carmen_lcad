/*!@file
   Robots/Beobot2/LaneRecognition/AppearanceContrastTrailRecognizer.H
   Lane recognition using appearance contrast [Rasmussen, etal. 09iros] */
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Beobot2/LaneRecognition/AppearanceContrastTrailRecognizer.C $
// $Id: $
//
//////////////////////////////////////////////////////////////////////////

#ifndef APPEARANCE_CONTRAST_TRAIL_RECOGNIZER
#define APPEARANCE_CONTRAST_TRAIL_RECOGNIZER

#include "Image/Image.H"
#include "Raster/Raster.H"
#include "Image/Pixels.H"
#include "SIFT/Histogram.H"

#include "GUI/XWinManaged.H"

#include <queue>

class RoadTemplate
{
public:
  RoadTemplate(){}
  ~RoadTemplate(){}
  RoadTemplate(Point2D<int> v,Point2D<int> l,Point2D<int> r,Dims dims)
  {
    vp = v;
    lp = l;
    rp = r;
		d = dims;
    //if left or right x point is out of image, then set to image size
    int left  = lp.i < 0 ? 0 : lp.i;
    int right = rp.i > dims.w() ? dims.w() : rp.i;
    mp = Point2D<int>((left+right)/2, (lp.j+rp.j)/2);
    diff = 0.0;
    index = 0;
  }

  Point2D<int> getMiddlePoint(){return mp;}
  Point2D<int> getMiddlePoint(Dims dims){float scale = dims.w()/d.w(); return Point2D<int>(mp*scale);}
  Point2D<int> getVanishingPoint(){return vp;}
  Point2D<int> getLeftPoint(){return lp;}
  Point2D<int> getRightPoint(){return rp;}
  Dims 	       getDims(){return d;}
  float getDiff(){return diff;}
  int getIndex(){return index;}
  void setDiff(float d){diff = d;}
  void setIndex(int i){index = i;}
private:
  float leftAngle;
  float rightAngle;
  int index;

  Point2D<int> vp;//vanishing point
  Point2D<int> mp;//middle point
  Point2D<int> lp;//left bottom point
  Point2D<int> rp;//right bottom point
  Dims d;


  float diff;//diff score between Center and Surround
};
class CompareRoadTemplate
{
public:
  bool operator()(RoadTemplate& r1, RoadTemplate& r2)
  {
    return r1.getDiff() > r2.getDiff();	//pop lowest first
  }
};


//! center surround histogram road recognition
//! uses various road templates with differencing 
//! to estimata shape of object
class AppearanceContrastTrailRecognizer
{
public:

  // ######################################################################
  //! @name Constructor, assigment and destructor
  //@{

  //! constructor
  AppearanceContrastTrailRecognizer();

  //! destructor
  ~AppearanceContrastTrailRecognizer();

  //@}

  // ######################################################################
  //! @name Compute functions
  //@{

  //! compute the road
  void computeRoad(Image<PixRGB<byte> > image);

  //@}

  // ######################################################################
  //! @name Access functions
  //@{

  //! get the display image
  Image<PixRGB<byte> > getDisplayImage();

  //! the Kmeans labeling display image
  Image<PixRGB<byte> > getKMeansDisplayImage();

  //! the middle point of the road
  Point2D<int> getMiddlePoint();

  //@}
	RoadTemplate getHighestRoadTemplate(){return itsHighestRoadTemplate;}	

private:

  // ######################################################################
  //! @name Compute functions (all the inner-working functions)
  //@{

  //! get the normalized CIElab features
  void computeNormalizedCIElab();

  //! compute k-means on the normalized CIElab faetures
  void computeKMeans();

  float getDiff(std::vector<float> a, std::vector<float> b);


  //! compute the various center surround shape combination
  //! in rectangle shapes
  void computeAllCSRoadTemplates(int w, int h);

  //! compute only one road template
  std::pair<std::vector<Point2D<int> >,
            std::pair<std::vector<Point2D<int> >,
                      std::vector<Point2D<int> > > >
  computeCSRoadTemplates(Point2D<int> vp,int leftAngle,int rightAngle,Dims dims);
  
  //! find corresponding bottom point 
  //! given a point, angle out and the dimension of the image
  Point2D<int> computeBottomPoint
  (Point2D<int> point,float angle, Dims dims);

  //! intersect 2 lines
  Point2D<int> intersectPoint
  (Point2D<int> p1, Point2D<int> p2,Point2D<int> p3,Point2D<int> p4);

  //! compute histogram for each grid location 
  void computeGridHistogram();

  //! compute the integral image histogram
  void computeHistogramIntegralImage();

  //! calculate the image distribution 
  //! from the given area
  rutz::shared_ptr<Histogram> getHistogramDistribution
  (Point2D<int> pt, int grid_size);

  //! get the histogram distribution 
  //! from the specified grid locations
  Histogram getGridHistogramDistribution(Rectangle r);

  //! 
  void setKMeansDisplayImage();

  //!
  void findRoad();

  //@}  

  //! original input image
  Image<PixRGB<byte> > itsImage;

  //! current center surround template dimensions
  int itsCurrentCStemplateWidth;
  int itsCurrentCStemplateHeight;

  //! lab image
  std::vector<Image<float> > itsLabImage;

  //! histogram image
  Image<int> itsLabelImage;

  //! debug window
  rutz::shared_ptr<XWinManaged> itsWin;

  //! debug image
  Image<PixRGB<byte> > itsKmeansDisplayImage;
  Image<PixRGB<byte> > itsDisplayImage;
  
  //! middle point estimate by template
  Point2D<int> itsMiddlePoint;
  
  //! remember last vanishing point
  Point2D<int> itsVanishingPoint;
  
  //!
  std::vector<float> itsPreviousRoadLikelihood;

  //! center surround shape templates 
  //! in a form of vector points
  //std::vector<std::pair<std::vector<Point2D<int> >,Rectangle > > itsCSpoints;
  std::vector<std::pair<std::vector<Point2D<int> >,
                        std::pair<std::vector<Point2D<int> >,
                                  std::vector<Point2D<int> > > > >
  itsCSpoints;
  
  std::vector<RoadTemplate> itsRoadTemplates;

  //! stored grid histogram
  Image<rutz::shared_ptr<Histogram> > itsGridHistogram;

  //! stored integral image
  Image<rutz::shared_ptr<Histogram> > itsIntegralHistogram;

  std::priority_queue<RoadTemplate,std::vector<RoadTemplate>,CompareRoadTemplate > 
  itsTopRoadTemplates;

	RoadTemplate itsHighestRoadTemplate;
};

// ######################################################################
// Implementation for AppearanceContrastTrailRecognizer inline functions
// ######################################################################


#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

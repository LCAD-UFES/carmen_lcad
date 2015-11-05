/*!@file Gist/VanishingPointDetector.H Detect vanishing point by
   gabor filter */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Gist/VanishingPointDetector.H
// $ $Id: $
//
//////////////////////////////////////////////////////////////////////////
#ifndef VANISHING_POINT_DETECTOR
#define VANISHING_POINT_DETECTOR


#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameIstream.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"
#include "Image/CutPaste.H"     // for inplacePaste()
#include "Image/LowPass.H"
#include "Image/ShapeOps.H"
#include "Image/DrawOps.H" //for writeText and SimpleFont
#include "Image/ImageSet.H" //for ImageSet
#include "Image/PyrBuilder.H"
#include "Image/PyramidOps.H" //for buildPyrGabor
#include "GUI/ImageDisplayStream.H"
#include "Util/SortUtil.H" //for rank() 
#include <cstdio>//for sprintf
#include <algorithm> //for inner_product(),accumulate()
#include <functional>//for inner_product(),accumulate()
#include <numeric>//for inner_product(),accumulate()
//#include "Gist/SuperPixel.H"
#include "Util/Timer.H"

#include "BayesFilters/UKF.H"//for U Kalman Filter

#define CONFIDENCE_THRESHOLD 0.5
#define ORIENTATION_NUM 36 //180/36 = 5 deg each

// Vantage Point Detector Kalman Filter 
class VPDFilter: public UKF
{
public:
  VPDFilter():
    UKF(8,4)//6 state 4 observations
  {

    itsState.setVal(0,0, 160/2);       // vantage point x-coord	
    itsState.setVal(0,1, 120/2);       // vantage point y-coord
    itsState.setVal(0,2, 120/4);       // left bottom   road boundary x-coord 
    itsState.setVal(0,3, 120-(120/4)); // right bottom  road boundary x-coord
    itsState.setVal(0,4, 160/2);       // middle bottom road boundary x-coord	
    itsState.setVal(0,5, 0.0);         // angle of middle line
    itsState.setVal(0,6, 0.0);         // angle Velocity
    itsState.setVal(0,7, 0.0);         // angle Acceleration

    // Initial covariance 
    itsSigma.setVal(0,0,160/2*160/2);  // vp x sigma^2
    itsSigma.setVal(1,1,120/2*120/2);  // vp y
    itsSigma.setVal(2,2,160*160);      // left
    itsSigma.setVal(3,3,160*160);      // right
    itsSigma.setVal(4,4,160/2*160/2);  // vp x sigma^2
    itsSigma.setVal(5,5,(0.01*M_PI/180)*(0.01*M_PI/180)); // angle of middle line
    itsSigma.setVal(6,6,0.01*0.01);    // ang velocity
    itsSigma.setVal(7,7,0.01*0.01);    // ang acceleration

    // Initial noise matrix
    double vpVar         = 5.0;
    double roadWidthVar  = 1.0;
    double mxVar         = 5.0;
    double angVar        = 0.01*M_PI/180;
    double angVelVar     = 0.01*M_PI/180;
    double tranVelVar    = 0.1;

    itsR.setVal(0,0,vpVar*vpVar);
    itsR.setVal(1,1,vpVar*vpVar);
    itsR.setVal(2,2,roadWidthVar*roadWidthVar);
    itsR.setVal(3,3,roadWidthVar*roadWidthVar);
    itsR.setVal(4,4,mxVar*mxVar);
    itsR.setVal(5,5,angVar*angVar);
    itsR.setVal(6,6,angVelVar*angVelVar);
    itsR.setVal(7,7,tranVelVar*tranVelVar);			
  }

  //! destructor
  ~VPDFilter() {}; 	

  //! update state
  Image<double> getNextState(const Image<double>& X, int k)
  {
    double vpX   = X.getVal(k,0);
    double vpY   = X.getVal(k,1);
    double left  = X.getVal(k,2);
    double right = X.getVal(k,3);
    //double mx = X.getVal(k,4);
    //double ang = X.getVal(k,5);

    double angVel  = X.getVal(k,6);
    double tranVel = X.getVal(k,7);

    Image<double> Xnew(1,itsNumStates, ZEROS);

    double eps = 2.22044604925031e-16;
    double mx = (left+right)/2;
    double ang = atan2(vpY,(vpX-mx)+eps);

    //double xc = vpX - tranVel/(angVel+eps)*sin(ang);
    //double yc = vpY + tranVel/(angVel+eps)*cos(ang);

    //vpX = xc + tranVel/(angVel+eps)*sin(ang + angVel);
    //vpY = yc - tranVel/(angVel+eps)*cos(ang + angVel);
    ang += angVel;

    Xnew[0] = vpX ;
    Xnew[1] = vpY;
    Xnew[2] = left;
    Xnew[3] = right;
    Xnew[4] = mx;
    Xnew[5] = ang;
    Xnew[6] = angVel;
    Xnew[7] = tranVel;

    return Xnew;
  }

  //! get the input observation
  Image<double> getObservation(const Image<double>& X, int k)
  {
    double vpX = X.getVal(k,0);
    double vpY = X.getVal(k,1);
    double left= X.getVal(k,2);
    double right = X.getVal(k,3);

    Image<double> zNew(1,itsNumObservations, ZEROS);
    zNew[0] = vpX;
    zNew[1] = vpY;
    zNew[2] = left;
    zNew[3] = right;
    //zNew[4] = mx;
    //zNew[5] = ang;
    return zNew;
  }

  //! get current vantage point and road
  void getState(Point2D<int>& vp,int& left, int& right)
  {
    vp    = Point2D<int>(itsState[0],itsState[1]);
    left  = itsState[2];
    right = itsState[3];			
  }

  //! get the current middle of the road
  void getState(int& mx,double& slope)
  {
    mx = itsState[0];
    slope = itsState[1];
  }
};//end class VPDF


//! Estimate Vantage Point 
//! by voting using gabor filter outputs from input image
class VanishingPointDetector
{
public:

  // ######################################################################
  //! @name Constructor, assigment and destructor
  //@{

  //! default constructor
  VanishingPointDetector();

  //! constructor with input image
  VanishingPointDetector(Image<PixRGB<byte> > ima);

  //! destructor
  ~VanishingPointDetector();

  //@}

  // ######################################################################
  //! @name Compute functions
  //@{

  //! input the new image
  void updateImage(Image<PixRGB<byte> >ima);

	//! update road from outside source(such as SuperPixel,UKF) FIXXX:don't know it work or not
	void updateRoadColor(PixRGB<byte> roadColor);

  //! get the confidence map Image
  Image<float> getConfidenceMapImg(){return itsConfidenceMap;}

  //! get the confidence map with color on it (for visualize)
	Image<PixRGB<byte> > getConfidenceMapImg(float confidenceThreshold);

  //! get vote map
  Image<float> getVoteMapImg(){return itsVoteMap;}

  //! get input image
  Image<PixRGB<byte> > getInputImage(){return itsImage;}

  //! get output image
	void computeOutputImage(Dims d = Dims(0,0));
  Image<PixRGB<byte> > getOutputImage();
	Image<PixRGB<byte> > getFilteredImage(bool notext=true);
  Image<PixRGB<byte> > getOrientationSetDisplay();

  //! get vanish point
  Point2D<int> getVanishingPoint(){return itsVp;}

  //! get left bottom point 
  Point2D<int> getLeftPoint(){return itsLeftRoadPoint;}

  //! get right bottom point
  Point2D<int> getRightPoint(){return itsRightRoadPoint;}

  //! get center bottom point
  Point2D<int> getMiddlePoint(){return (itsRightRoadPoint+itsLeftRoadPoint)/2;}

  //! get middle point 4 pixel above bottom of image
  Point2D<int> getEstMiddlePoint(){return itsEstMiddlePoint;}
  
  //! get left Ocr bottom point 
  Point2D<int> getLeftOcrPoint(){return itsLeftOcrPoint;}
  //! get right Ocr bottom point
  Point2D<int> getRightOcrPoint(){return itsRightOcrPoint;}
  //! get left Color bottom point 
  Point2D<int> getLeftColorPoint(){return itsLeftColorPoint;}
  //! get right Color bottom point
  Point2D<int> getRightColorPoint(){return itsRightColorPoint;}
	//! get road color cover by road boundary
	PixRGB<byte> getRoadColor(){return itsRoadColor;};

  //@}
  double getZprobColor(){return itsZprobColor;}
  double getZprobGabor(){return itsZprobGabor;}
	int getMaskSize(){return itsMaskSize;}
  

  //!  Return best ocr angle
  //bestOcrRayAngle,bestColorRayAngle : return value for best angle
  //initAngle : start search angle degree +- 20 deg, for left ray, default is 130 +-20
	//return total select ray
  float findBestRay(float& bestOcrRayAngle,float& bestColorRayAngle,float initAngle = 150.0, int colorSpace = RGB);
  
  //! Find better vp from bottom point
  float reversingRay(Point2D<int> bottomPoint,float bottomAngle,int colorSpace = RGB);
  

  // ######################################################################
  //! @name Access functions
  //@{

  Image<float> getSearchMemoryMap(){return itsSearchMemoryMap;}  

  //@}

private:
  void findRoad();

  //! update the current state
  void updateFilter();

  //! build the set of Gabor outputs
  void getOrientationSet(uint num,uint scale);
  
  //! get the index map
  Image<int> getIndexMap();

  //! compute the confidence map & index map
  void computeConfidenceMap
  (ImageSet<float> orientationSet,Image<int>& indexMap,Image<float>& confMap);

  //! compute the quality of confidence
  float confidenceQualityCount(float threshold);

  //! compute angle between point PV and Op  
  float getAngle(int x1,int y1,int x2,int y2,float gaborAngle);
  float getAngle(int x1,int y1,int x2,int y2,int gaborAngleIndex,float radius);

  //! build an image orientation set from input image 
  ImageSet<float> buildOrientationSet
  (Image<PixRGB<byte> > image,uint num,uint scale);

  //! get an average image 
  //! across the scales in the gabor pyramid
  Image<float> getAverageGaborPyrImage(ImageSet<float> gaborPyr);

  //! get a local max from a sorted vector given a range
  float getLocalMaxAverage
  (std::vector<float> v, uint start, uint end);

  //! compute the confidence map
  Image<float> getConfidenceMap();


  Image<float> getVoteMap
  (float confidenceThreshold = 0.5,
   float radiusRatio = 0.35,
   float matchAngleThreshold = 8.0,
   Point2D<int> vp_prior=Point2D<int>(0,0),
   Dims region=Dims(0,0));

  //! 
  Image<float> getVoteMap2
  (Image<float> mask,
   float confidenceThreshold = 0.5,
   float radiusRatio = 0.35,
   float matchAngleThreshold = 8.0);

  //! 
  Image<float> getVoteMap3
  (int pixelGrid = 4, 
   float confidenceThreshold = 0.5,
   float radiusRatio = 0.35,
   float matchAngleThreshold = 8.0);

  //! compute voting map from confidence map & index map 
  //confMap            : confidence image map
  //indexMap           : index of orientation image map
  //threshold          : threshold of confidence to decide which pixel should vote or not. Default is > 0.3
  //radiusRatio        : area of voting region, radius is compute by (diagonal length of image) * (radiusRatio), Default is 0.35
  //matchAngleThreshold: match angle is compute by the difference of vp and op,  only the pixel with match angle less than matchAngleThreshold can vote. Default is < 8.0 degree
  //vp_prior,region    : previous vanishing point location , region window size for vote
  //pixelGrid          : compute vote map in coarse scale, skip in every pixelgrid

  Image<float> computeVoteMap
  (Image<int> indexMap,
   Image<float> confMap,
   float threshold,
   float radiusRatio,
   float matchAngleThreshold,
   Image<float> mask);

  Image<float> computeVoteMap
  (Image<int> indexMap,
   Image<float> confMap,
   float threshold,
   float radiusRatio,
   float matchAngleThreshold,
   Point2D<int> vp_prior,
   Dims region);

  Image<float> computeVoteMap
  (Image<int> indexMap,
   Image<float> confMap,
   float threshold,
   float radiusRatio,
   float matchAngleThreshold);

  Image<float> computeCoarseVoteMap
  (Image<int> indexMap,
   Image<float> confMap,
   float threshold,
   float radiusRatio,
   float matchAngleThreshold,
   int pixelGrid);

	 //! make all value in the window to zero, then we can find second max
	void subtractMax
	 (Image<float>& input,Point2D<int> center,Dims window);

	//!
	 void addMemoryMapRegion
	 (Image<float>& input,
	 Point2D<int> center,
	 Dims window);

  void forgetMemoryMap(Image<float>& input);
  
  float getVanishPointScore
  (int x,int y,
   float threshold,
   float matchAngleThreshold,
   float radius,
   int shape = DISK);

  //! compute orientation consistency ratio (OCR)
  float computeOCR(float angle);
  //! compute angle offset mean (AOM)//my name
  float computeAOM(float angle);
  //! compute gabor response variance(GRA) //my name
  //float computeGRV(float angle);
  float computeGRV(float angle,float& norm,float& maxStdev);
  
  //! compute bottom point of image given vanish point and angle
  Point2D<int> computeBottomPoint(Point2D<int> point,float angle);
  //! compute points that have same height with vanish point and given point from bottom and angle
  Point2D<int> computeVpNeighborPoint(Point2D<int> bottomPoint,float angle);
  float getOffsetAngle(int x,int y,float angle);
  
  //! display the gradient magnitude image 
  void displayGradImage(std::vector<Image<float> > gradImg);

  void drawFanArea(float angle1,float angle2,PixRGB<byte> color);
  PixRGB<byte> getFanAreaColor(float angle1,float angle2,Point2D<int> topPoint = Point2D<int>(-1,-1));
	PixRGB<byte> getFanAreaColor(Point2D<int> topPoint,Point2D<int> leftPoint,Point2D<int> rightPoint);
  PixLab<byte> getFanAreaColorLab(float angle1,float angle2);
  PixRGB<byte> getFanAreaColorFromBottom(float angle1,float angle2,Point2D<int> bottomPoint);
  // ######################################################################
  //! @name Compute functions (all the inner-working functions)
  //@{

  //! its current input image
  Image<PixRGB<byte> > itsImage;

  //! its output image with vanish point on it
  Image<PixRGB<byte> > itsOutputImage;
 
  //! the radius of voting region 
  int itsRadius;

  //! orientation angle (180/number of angle)
  float itsAngle;

  //! total number of angle 
  int itsNumOfAngle;

  //! gabor pyramid scale
  int itsScale;

  //! the direction map
  ImageSet<float> itsOrientationSet;

  //! the winning index (for each pixel) of the gabor direction map
  Image<int> itsAngleIndexMap;
  
  //! the confidence map
  Image<float> itsConfidenceMap;

  Image<float> itsVoteMap;

  Image<float> itsSearchMemoryMap; 
  int itsMaskSize;//Count current pixel in mask 
  //! output debug image 
  Image<PixRGB<byte> > itsDebugImage;

  //! Keep track current frame
  int itsFrameID;

  //After processed the image, increment this id
  int itsProcessedFrameID;
  
  float itsThreshold;//for conf
  
  Point2D<int> itsVp;
  Point2D<int> itsTrackedVp;//previous vp
  Point2D<int> itsFilteredVp;//using UKF

  PixRGB<byte> itsRoadColor;
  Point2D<int> itsLeftOcrPoint;   
  Point2D<int> itsLeftColorPoint; 
  float itsLeftColorAngle; 
  float itsLeftOcrAngle;   

  Point2D<int> itsRightOcrPoint;
  Point2D<int> itsRightColorPoint;
  float itsRightColorAngle;
  float itsRightOcrAngle;

  Point2D<int> itsMiddleOcrPoint;
  Point2D<int> itsMiddleColorPoint;

  Point2D<int> itsLeftRoadPoint;//filtered by UKF
  Point2D<int> itsRightRoadPoint;//filtered by UKF
  Point2D<int> itsEstMiddlePoint;//filtered by UKF
  //! debug window
  //rutz::shared_ptr<XWinManaged> itsWin;

  //Kalman filter
  VPDFilter itsVpdf;
  //Set the Sensor noise
  Image<double> itszNoise;
  double itsZprobColor;
  double itsZprobGabor;

  // |error| < 0.005
  float fast_atan2f( float const y, float const x )
  {
    float const PI_FLOAT = 3.14159265f;
    float const PIBY2_FLOAT = 1.5707963f;

    if ( x == 0.0f )
    {
      if ( y > 0.0f ) return PIBY2_FLOAT;
      if ( y == 0.0f ) return 0.0f;
      return -PIBY2_FLOAT;
    }
    float atan;
    float const z = y/x;
    if ( fabsf( z ) < 1.0f )
    {
      atan = z/(1.0f + 0.28f*z*z);
      if ( x < 0.0f )
      {
	if ( y < 0.0f ) return atan - PI_FLOAT;
	return atan + PI_FLOAT;
      }
    }
    else
    {
      atan = PIBY2_FLOAT - z/(z*z + 0.28f);
      if ( y < 0.0f ) return atan - PI_FLOAT;
    }
    return atan;
  }

  //lookup table of sin & cos`
  float lookup_cos(int index)
  {
    //ang = index*itsAngle
    return itsCos_table[index];
  }

  float lookup_sin(int index)
  {
    return itsSin_table[index];
  }

  void buildSinCosTable(int num)
  {
    for(int i = 0 ;i < num;i++)
    {
      float ang = ((i*itsAngle)/180.0)*M_PI;
      itsSin_table.push_back(sin(ang));
      itsCos_table.push_back(cos(ang));
    }
  }

  void buildAtanSinCos_table(int num)
  {
    for(int i = 0 ;i < num;i++)
    {
      float ang = ((i*itsAngle)/180.0)*M_PI;
      float atan = atan2(sin(ang),cos(ang));
      itsAtanSinCos_table.push_back(atan);
    }
  }

  void buildXboundTable(float radius)
  {
    //int xbound = int(sqrtf(float(radius*radius) - yy*yy));
    float const radius2 = radius*radius;
    for(int yy = 0; yy <= radius;yy++)
    {

      int xbound = int(sqrtf(radius2 - yy*yy));
      itsXbound_table.push_back(xbound);
    }
  }

  void buildXYdistanceTable(int radius)
  {

    //x start from -radius to radius
    //y start from 0 to radius
    //int xbound = int(sqrtf(float(radius*radius) - yy*yy));
    int const radius2x = radius+radius+2;
    itsXYdistance_table.resize(radius2x*(radius+1));
    for(int yy = 0; yy <= radius;yy++)
    {
      for(int xx= -radius;xx <= radius ;xx++){	
	int index = (xx+radius)*radius + yy;
	float distance = sqrt(xx*xx + yy*yy);
		itsXYdistance_table[index] = distance; 
	}
    }
  }

  float xyDistance(int xx,int yy,int radius)
  {
	int index = (xx+radius)*radius + yy;
	return itsXYdistance_table[index];
  }

  float xyAtan(int yy,int xx,int radius)
  {
	int index = (xx+radius)*radius + yy;
	return itsAtanXY_table[index];
  }

  void buildXYatanTable(int radius)
  {
    //x start from -radius to radius
    //y start from 0 to radius
    //int xbound = int(sqrtf(float(radius*radius) - yy*yy));
    int const radius2x = radius+radius+2;
    itsAtanXY_table.resize(radius2x*(radius+1));
    for(int yy = 0; yy <= radius;yy++)
    {
      for(int xx= -radius;xx <= radius ;xx++){	
	int index = (xx+radius)*radius + yy;
	float atan = atan2(yy,xx);
		itsAtanXY_table[index] = atan; 
	}
    }
  }

  std::vector<float> itsSin_table;
  std::vector<float> itsCos_table;
  std::vector<int>   itsXbound_table;
  std::vector<float> itsXYdistance_table;
  std::vector<float> itsAtanSinCos_table;
  std::vector<float> itsAtanXY_table;
  
  //!
  enum shape {DISK,RECT};
  enum colorSpace {RGB,LAB};  
};

// ######################################################################
inline Point2D<int> intersectPoint
(Point2D<int> p1, Point2D<int> p2,Point2D<int> p3,Point2D<int> p4)
{
 //Find intersection point Algorithm can be find here :
 //http://paulbourke.net/geometry/lineline2d/

  double mua,mub;
  double denom,numera,numerb;
  double x,y;
  double EPS = 0.0001;//Epsilon : a small number to enough to be insignificant
  

  denom  = (p4.j-p3.j) * (p2.i-p1.i) - (p4.i-p3.i) * (p2.j-p1.j);
  numera = (p4.i-p3.i) * (p1.j-p3.j) - (p4.j-p3.j) * (p1.i-p3.i);
  numerb = (p2.i-p1.i) * (p1.j-p3.j) - (p2.j-p1.j) * (p1.i-p3.i);

  /* Are the line coincident? */
  if (abs(numera) < EPS && abs(numerb) < EPS && abs(denom) < EPS) 
    {
      x = (p1.i + p2.i) / 2;
      y = (p1.j + p2.j) / 2;
      return Point2D<int>(x,y);
    }

  /* Are the line parallel */
  if (abs(denom) < EPS) {
    x = 0;
    y = 0;
    return Point2D<int>(x,y);
  }

  /* Is the intersection along the the segments */
  mua = numera / denom;
  mub = numerb / denom;
  if (mua < 0 || mua > 1 || mub < 0 || mub > 1) {
    x = 0;
    y = 0;
    
  }
  x = p1.i + mua * (p2.i - p1.i);
  y = p1.j + mua * (p2.j - p1.j);

  //LINFO("Intersection Point is (%f,%f)",x,y);
  return Point2D<int>(x,y);
}

// ######################################################################
//float diff(const PixHyper<T,3>& pix1,const PixHyper<T,3>& pix2)
inline float diffLAB(PixLab<byte> const pix1, PixLab<byte> const pix2)
{
  //in LAB space, we only use euclid distance from A & B
  float const da = (pix1.A() - pix2.A());
  float const db = (pix1.B() - pix2.B());

  return sqrt( da*da + db*db );
}

inline float diffLAB(PixRGB<byte> const pix1, PixRGB<byte> const pix2)
{
  //in LAB space, we only use euclid distance from A & B
  PixLab<byte> labpix1(pix1);//convert RGB to LAB here
  PixLab<byte> labpix2(pix2);
  float const da = (labpix1.A() - labpix2.A());
  float const db = (labpix1.B() - labpix2.B());

  return sqrt( da*da + db*db );
}
inline float diffRGB(PixRGB<byte> const pix1, PixRGB<byte> const pix2)
{

  float const dr = (pix1.red() - pix2.red());
  float const dg = (pix1.green() - pix2.green());
  float const db = (pix1.blue() - pix2.blue());

  return sqrt( dr*dr + dg*dg + db*db );
}
inline float distance(Point2D<int> const p1, Point2D<int> const p2)
{
	int dx = p1.i-p2.i;
	int dy = p1.j-p2.j;
	return sqrt(dx*dx + dy*dy);
}
// ######################################################################
// Implementation for VanishingPointDetector inline functions
// ######################################################################
//FIXXX should just use templete

//FIXXX it later
//inline PixRGB<long> stdev(std::vector<PixRGB<byte> > data)
//{
//  PixRGB<long> mean = accumulate( data.begin(), data.end(), PixRGB<long>(0,0,0) )/ data.size();
//
//	std::vector<PixRGB<long> > zero_mean(data);
//
//  transform( zero_mean.begin(), zero_mean.end(), 
//             zero_mean.begin(), std::bind2nd(std::minus<PixRGB<long> >(), mean));
//
// 	PixRGB<long>  deviation = 
//    inner_product(zero_mean.begin(), zero_mean.end(), 
//                  zero_mean.begin(), PixRGB<long>(0,0,0) );
//  deviation = sqrt( deviation / ( data.size() - 1 ) );
//  return deviation;
//	
//}
inline float stdev(std::vector<float> data)
{
  float mean = accumulate( data.begin(), data.end(), 0.0f )/ data.size();
  std::vector<float> zero_mean( data );
  transform( zero_mean.begin(), zero_mean.end(), 
             zero_mean.begin(), std::bind2nd(std::minus<float>(), mean));

  float deviation = 
    inner_product(zero_mean.begin(), zero_mean.end(), 
                  zero_mean.begin(), 0.0f );
  deviation = sqrt( deviation / ( data.size() - 1 ) );
  return deviation;
}
inline int max(int a,int b)
{
	return a > b ? a : b;
}
inline int min(int a,int b)
{
	return a < b ? a : b;
}
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

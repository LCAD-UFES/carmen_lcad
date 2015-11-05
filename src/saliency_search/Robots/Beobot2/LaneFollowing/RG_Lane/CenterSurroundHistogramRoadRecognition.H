/*!@file Robots/Beobot2/LaneRecognition/CenterSurroundHistogramRoadRecognition.C   Lane recognition using center surround histogram difference */
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Beobot2/LaneRecognition/CenterSurroundHistogramRoadRecognition.C $
// $Id: $
//
//////////////////////////////////////////////////////////////////////////

#ifndef CENTER_SURROUND_HISTOGRAM_ROAD_RECOGNITION
#define CENTER_SURROUND_HISTOGRAM_ROAD_RECOGNITION

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

			//if left or right x point is out of image, then set to image size
			int left  = lp.i < 0 ? 0 : lp.i;
			int right = rp.i > dims.w() ? dims.w() : rp.i;
			mp = Point2D<int>((left+right)/2, (lp.j+rp.j)/2);
			diff = 0.0;
			index = 0;
		}

		Point2D<int> getMiddlePoint(){return mp;}
		Point2D<int> getVanishingPoint(){return vp;}
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
class CenterSurroundHistogramRoadRecognition //CenterSurroundHistogramRoadRecognition
{
public:

  // ######################################################################
  //! @name Constructor, assigment and destructor
  //@{

  //! constructor
  CenterSurroundHistogramRoadRecognition();

  //! destructor
  ~CenterSurroundHistogramRoadRecognition();

  //! set the input image
  void setImage(Image<PixRGB<byte> > image);

  //! get the salient region given the salient point 
  Image<float> getSalientRegion(Point2D<int> pt);

	inline Image<PixRGB<byte> >getDispImage(){return itsDispImage;}
	inline Point2D<int> getMiddlePoint(){return itsMiddlePoint;}
  //@}

private:

  //! original input image
  Image<PixRGB<byte> > itsImage;

	//! debug image
  Image<PixRGB<byte> > itsDispImage;

  //! debug window
  rutz::shared_ptr<XWinManaged> itsWin;

  //! list of histogram entries for various image features:
  //! FIXXX_NOTE: currently just brightness and color (from CIELab)
  std::vector<Image<int> > itsImageHistogramEntries;

  //! stored grid histogram
  Image<rutz::shared_ptr<Histogram> > itsGridHistogram;

  //! stored integral image
  Image<rutz::shared_ptr<Histogram> > itsIntegralHistogram;

  //! center surround shape templates 
  //! in a form of vector points
  //std::vector<std::vector<Point2D<int> > > itsCSpoints;

  //! center surround shape templates 
  //! in a form of vector points
  std::vector<std::pair<std::vector<Point2D<int> >,Rectangle > > itsCSpoints;

  std::vector<RoadTemplate> itsRoadTemplates;
  std::priority_queue<RoadTemplate,std::vector<RoadTemplate>,CompareRoadTemplate > itsTopFiveRoadTemplates;

  //! smoothing kernel for histogram smoothing
  Image<float> itsLKernel;
  Image<float> itsAKernel;
  Image<float> itsBKernel;

  //! middle point estimate by template
	Point2D<int> itsMiddlePoint;

	//! remember last vanishing point
	Point2D<int> itsVanishingPoint;

  // ######################################################################
  //! @name Compute functions (all the inner-working functions)
  //@{

  //! find corresponding bottom point 
  //! given a point, angle out and the dimension of the image
  Point2D<int> computeBottomPoint
  (Point2D<int> point,float angle, Dims dims);

  //! intersect 2 lines
  Point2D<int> intersectPoint
  (Point2D<int> p1, Point2D<int> p2,Point2D<int> p3,Point2D<int> p4);
  
  //! set the histogram of feature values
  void setImageFeatureHistogramValues();

  //! quantize the input feature map 
  //! to the specified bin numbers
  Image<int> quantize_values(Image<float> image, int num_bins);

  //! compute histogram for each grid location 
  void computeGridHistogram();

  //! compute the integral image histogram
  void computeHistogramIntegralImage();

  //! compute the various center surround shape combination
  //! in rectangle shapes
  void computeAllCSRoadTemplates();

  //! compute only one road template
  std::pair<std::vector<Point2D<int> >,Rectangle> 
  computeCSRoadTemplates(Point2D<int> vp,int leftAngle,int rightAngle,Dims dims);
  
  //! compute a series of road template given vanishing point	
  void computeVpCSRoadTemplates(Point2D<int> vp);

  //! calculate the image distribution 
  //! from the given area
  rutz::shared_ptr<Histogram> getHistogramDistribution
  (Point2D<int> pt, int grid_size);

  //! get the histogram distribution 
  //! from the specified grid locations
  Histogram getGridHistogramDistribution(Rectangle r);

  //! smooth and normalize the feature vector distribution
  Histogram smoothAndNormalize(Histogram h, int numPoints);
 
  //! find the road
  void findRoad();

  
  //@}
};

// ######################################################################
// Implementation for CenterSurroundHistogramRoadRecognition inline functions
// ######################################################################


#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

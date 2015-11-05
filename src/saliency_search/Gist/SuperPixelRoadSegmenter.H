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
#ifndef SUPERPIXEL_ROAD_SEGMENTER
#define SUPERPIXEL_ROAD_SEGMENTER
#include "Image/OpenCVUtil.H"

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
#include "GUI/ImageDisplayStream.H"
#include <cstdio>//for sprintf
#include <algorithm> //for inner_product(),accumulate()
#include <functional>//for inner_product(),accumulate()
#include <numeric>//for inner_product(),accumulate()
#include "Util/Timer.H"

//################################################################
//################################################################
//################################################################

class SuperPixelRoadSegmenter
{
public:
	//################################################################
  //! @name Constructor, assigment and destructor
  //@{

  //! default constructor
	SuperPixelRoadSegmenter();

	//! constructor with input image
	SuperPixelRoadSegmenter(Image<PixRGB<byte> > ima);
	//! destructor

	~SuperPixelRoadSegmenter();
	//@}


	//! Compute region segments and find the road region
	void updateRoadColor(PixRGB<byte> roadColor);
	void updateImage(Image<PixRGB<byte> >ima);
	Image<PixRGB<byte> > getSuperPixel(Image<PixRGB<byte> > img);
	Image<PixRGB<byte> > findRoad();
	Image<PixRGB<byte> > getOutputImage();
	Image<PixRGB<byte> > getRoadFindingMap(){return itsRoadFindingMap;}
	Image<PixRGB<byte> > getSuperPixelMap(){return itsSuperPixelMap;}
	Image<PixRGB<byte> > getRawSuperPixelImg(){return itsRawSuperPixelImg;}
	Image<PixRGB<byte> > getRawRoadSuperPixelImg(){return itsRawRoadSuperPixelImg;}
	Image<PixRGB<byte> > getFanImg(){return itsFanImg;}
	Image<int>  				 getRoadIndexMap(){return itsRoadIndexMap;}
	Point2D<int> 				 getMiddlePoint(){return itsEstMiddlePoint;}
	Point2D<int> 				 getHighMiddlePoint(){return itsHighMiddlePoint;}
	PixRGB<byte> 				 getRoadColor(){return itsRoadColor;}
	

	PixRGB<byte> getFanAreaColor(Point2D<int> topPoint,Point2D<int> leftPoint,Point2D<int> rightPoint, Image<PixRGB<byte> > img);
	Image<PixRGB<byte> > drawFanArea( Point2D<int> topPoint,Point2D<int> leftPoint,Point2D<int> rightPoint,
																		Image<PixRGB<byte> > img, PixRGB<byte> color);
private:
		//! its current input image
		Image<PixRGB<byte> > itsImage;

		//! its output image with vanish point on it
		Image<PixRGB<byte> > itsDispImg;

		//! Input/Output Images
		nub::soft_ref<OutputFrameSeries> itsOfs;

		Image<PixRGB<byte> > itsSuperPixelMap;
		Image<PixRGB<byte> > itsRawSuperPixelImg;//raw from fezhat code
		Image<PixRGB<byte> > itsRawRoadSuperPixelImg;//road region colored with red grid
		Image<PixRGB<byte> > itsRoadFindingMap;
		Image<PixRGB<byte> > itsFanImg;//fan segment from vp
		Image<int>  	     itsRoadIndexMap;//1 for road,0 for none road

		int itsFrameID;
		int itsProcessedFrameID;
		int itsCurrImgID;
		int itsPrevProcImgID;

		Image<double> itszNoise;
		double itsZprob;		
		void updateUKF(Point2D<int> midPoint,PixRGB<byte>roadColor);

		//! SuperPixel Road finding flags
		bool itsUseFloatWindow;


		//! Road parameters
		Point2D<int> itsEstMiddlePoint;//Current road middle point
		Point2D<int> itsHighMiddlePoint;//p1,get from fitLine of itsMiddlePoint
		PixRGB<byte>  itsRoadColor;//Current road color
		float itsRoadColorDiff;//Current road color difference from filted road color
		PixRGB<byte> itsRoadColorDiffSub;//Color diff for each color channel
		int itsMiddlePoint[5];//Bottom road middle point for line fitting

		//! Open another window for debug display
		void debugWin(Image<PixRGB<byte> >disp,std::string title = "Debug Win");
		//! Debug Window
		std::vector<rutz::shared_ptr<XWinManaged> > itsWinVec;
		uint itsDebugWinCounter;



		// Math calculation function
		// ######################################################################
		float colorDiff(PixRGB<byte> c1,PixRGB<byte> c2)
		{
			int r = c1.red() - c2.red();
			int g = c1.green() - c2.green();
			int b = c1.blue() - c2.blue();
			return sqrt(r*r+g*g+b*b);
		} 
		// ######################################################################
		PixRGB<byte> colorDiffSub(PixRGB<byte> c1,PixRGB<byte> c2)
		{
			int r = abs(c1.red() - c2.red());
			int g = abs(c1.green() - c2.green());
			int b = abs(c1.blue() - c2.blue());
			return PixRGB<byte>(r,g,b);
		} 
		// Weighted Average color
		// ######################################################################
		PixRGB<byte> colorAvg(PixRGB<byte> c1,PixRGB<byte> c2,float ratio1 = 0.5)
		{
			//int ratio1 = 8;//80%
			float ratio2 = 1.0 - ratio1;//20%

			int r = (c1.red()*ratio1 + c2.red()*ratio2);
			int g = (c1.green()*ratio1 + c2.green()*ratio2);
			int b = (c1.blue()*ratio1 + c2.blue()*ratio2);
			return PixRGB<byte>(r,g,b);
		} 
		//! Given a vector of points, return the best fit line 
		// ######################################################################
		void fitLine(std::vector<Point2D<int> > points, Point2D<int>&p1,Point2D<int>&p2)
		{
			float *line = new float[4];
			//float linearity = 0.0f;
			CvMemStorage* storage = cvCreateMemStorage(0);
			//	CvSeq* point_seq = cvCreateSeq( CV_32FC2, sizeof(CvSeq), sizeof(CvPoint2D32f), storage );
			CvPoint* cvPoints = (CvPoint*)malloc(points.size()*sizeof(Point2D<int>));
			for(uint i = 0; i < points.size(); i++){
				int x = points.at(i).i;
				int y = points.at(i).j;
				//cvSeqPush(point_seq,cvPoint2D32f(x,y));
				cvPoints[i].x = x;
				cvPoints[i].y = y;
			}
			//	linearity = myLinearity(point_seq);
			CvMat point_mat = cvMat(1,points.size(), CV_32SC2, cvPoints);
			cvFitLine(&point_mat, CV_DIST_L2,0, 0.01,0.01,line);
			//LINFO("v=(%f,%f),vy/vx=%f,(x,y)=(%f,%f), Linearity=%f\n",line[0],line[1],line[1]/line[0],line[2],line[3],linearity);
			cvReleaseMemStorage(&storage);

			double a, b, c, d, e, f,x0,y0,x1,y1;              // y = a + b*x, b is slop
			b = line[1]/ line[0];                  // y = c + d*x
			a = line[3]- b*line[2];                // y = e + f*x
			d = -1/b;
			c = points[0].j - d*points[0].i;
			f = d;
			e = points[points.size()-1].j - f*points[points.size()-1].i;

			x0 = (a-c)/(d-b);                  // x = (a-c)/(d-b)
			y0 = c+d*x0;                   // y = a + b*x
			x1 = (a-e)/(f-b);
			y1 = e+f*x1;

			p1.i = (int)x0;              
			p1.j = (int)y0;           
			p2.i = (int)x1;
			p2.j = (int)y1;
		}

		// ######################################################################
		Point2D<int> intersectPoint
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
};//end class SuperPixelRoadSegmenter
#endif

/*!@file Robots/Beobot2/LaneRecognition/RoadGenerator.C   
Generate Synthetic Road*/
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
// Primary maintainer for this file: Chin-Kai Chang<chinkaic@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Beobot2/LaneRecognition/RoadGenerator.C $
// $Id: $
//
//////////////////////////////////////////////////////////////////////////

#ifndef ROAD_GENERATOR
#define ROAD_GENERATOR 

#include "Image/Image.H"
#include "Raster/Raster.H"
#include "Image/Pixels.H"
#include "SIFT/Histogram.H"

#include "GUI/XWinManaged.H"

#include <queue>
#define DEFAULT_WIDTH                 80 
#define DEFAULT_HEIGHT                60
class RoadGroundTruthTemplate
{
	public:
		RoadGroundTruthTemplate(Point2D<int> v,Point2D<int> l,Point2D<int> r,int la,int ra,Dims dims)
		{
			vp = v;
			lp = l;
			rp = r;
			leftAngle = la;
			rightAngle = ra;
			itsDims = dims;
			//if left or right x point is out of image, then set to image size
			int left  = lp.i < 0 ? 0 : lp.i;
			int right = rp.i >= dims.w() ? dims.w()-1 : rp.i;
			mp = Point2D<int>((left+right)/2, (lp.j+rp.j)/2);
			diff = 0.0;
			index = 0;
		}
		~RoadGroundTruthTemplate(){}

		Point2D<int> getMiddlePoint(){return mp;}
		Image<PixRGB<byte> > getRoadImg(){
			generateRoadImage(itsDims);
			return roadImg;
		}
		float getDiff(){return diff;}
	  int getIndex(){return index;}
		void setDiff(float d){diff = d;}
		void setIndex(int i){index = i;}
		int getLeftAngle(){return leftAngle;}
		int getRightAngle(){return rightAngle;}
		Point2D<int> getVp(){return vp;}
		Point2D<int> getLp(){return lp;}
		Point2D<int> getRp(){return rp;}
		Dims getDims(){return itsDims;}
	private:
		int leftAngle;
		int rightAngle;
		int index;
		Dims itsDims;
		Point2D<int> vp;//vanishing point
		Point2D<int> mp;//middle point
		Point2D<int> lp;//left bottom point
		Point2D<int> rp;//right bottom point
		Image<PixRGB<byte> > roadImg;

		float diff;//diff score between Center and Surround

		void generateRoadImage(Dims dims){
			roadImg = Image<PixRGB<byte> >(dims,ZEROS);
			
			Image<PixRGB<byte> >::iterator iptr = roadImg.beginw(), stop = roadImg.endw(); 
			while(iptr != stop) *iptr++ = PixRGB<byte>(128,128,128);


			drawFanArea(vp,lp,rp,roadImg.getDims(),PixRGB<byte>(255,255,0));
			drawLine(roadImg,vp,lp,PixRGB<byte>(255,0,0),1); //left boundary
			drawLine(roadImg,vp,rp,PixRGB<byte>(255,0,0),1); //right boundary
		}
		void drawFanArea(Point2D<int> p1,Point2D<int> p2,Point2D<int> p3,Dims dims,PixRGB<byte> color)
		{
			int w = dims.w();
			int h = dims.h();

			for(int y = p1.j; y < h;y++)
			{
				Point2D<int> it1 = intersectPoint(p1,p2,Point2D<int>(0,y),Point2D<int>(w,y));
				Point2D<int> it2 = intersectPoint(p1,p3,Point2D<int>(0,y),Point2D<int>(w,y));	
				int length = abs(it1.i - it2.i);
				int x_init = (it1.i < it2.i) ? it1.i : it2.i; 
				for(int x = x_init; x < length+x_init;x++){
					if(roadImg.coordsOk(x,y)){
						roadImg.setVal(x,y,color);
					}
				}
			}
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
};
class CompareRoadGroundTruthTemplate
{
	public:
	bool operator()(RoadGroundTruthTemplate& r1, RoadGroundTruthTemplate& r2)
	{
		return r1.getDiff() > r2.getDiff();	//pop lowest first
	}
};



//! center surround histogram road recognition
//! uses various road templates with differencing 
//! to estimata shape of object
class RoadGenerator //RoadGenerator
{
public:

  // ######################################################################
  //! @name Constructor, assigment and destructor
  //@{

  //! constructor
	RoadGenerator();
	RoadGenerator(int w,int h);

  //! destructor
  ~RoadGenerator(){};

  //! set the input image
  void setImage(Image<PixRGB<byte> > image);


	Image<PixRGB<byte> >getRoad();
	int getError(int loc,Dims dims);
	int getLeftAngle();
	int getRightAngle();
	RoadGroundTruthTemplate getGT();

  //@}

private:

  //! original input image
  Image<PixRGB<byte> > itsImage;

	//! debug image
  Image<PixRGB<byte> > itsDispImage;

  //! debug window
  rutz::shared_ptr<XWinManaged> itsWin;

  void generateRoadGroundTruthTemplates(int w = DEFAULT_WIDTH,int h = DEFAULT_HEIGHT);
  //! center surround shape templates 
  //! in a form of vector points
  std::vector<std::pair<std::vector<Point2D<int> >,Rectangle > > itsCSpoints;

  std::vector<RoadGroundTruthTemplate> itsRoadGroundTruthTemplates;

	int itsCount;

	Point2D<int> computeBottomPoint(Point2D<int> point,float angle, Dims dims);
};

// ######################################################################
// Implementation for RoadGenerator inline functions
// ######################################################################


#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

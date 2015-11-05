/*!@file Robots/Beobot2/LaneRecognition/RoadGenerator.C   
Generate Synthetic Road */
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

#include "Image/Kernels.H"
#include "Image/ColorOps.H"
#include "Image/MathOps.H"
#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Image/ShapeOps.H"

#include "Util/Timer.H"

#include "GUI/XWinManaged.H"

#include "Robots/Beobot2/LaneFollowing/RG_Lane/RoadGenerator.H"



#define VANTAGE_POINT_GRID_SIZE       10
#define GRID_SIZE                      4

#define LEFT_BOUNDARY_START_ANGLE     20
#define RIGHT_BOUNDARY_END_ANGLE     160
#define ANGLE_INCREMENT                5
#define MIN_ROAD_ANGLE_SPACING        40
#define MAX_ROAD_ANGLE_SPACING       140

#define GRID_AREA                    GRID_SIZE*GRID_SIZE
#define INCLUDE_THRESHOLD            0.50F 

#define NUM_CHANNELS                   3 // current number of channels   
#define NUM_L_BINS                    25 // # bins for L component of CIELab
#define NUM_A_BINS                    25 // # bins for a component of CIELab
#define NUM_B_BINS                    25 // # bins for b component of CIELab
#define NUM_HISTOGRAM_DIMS            75 // current total number of bins

// ######################################################################
RoadGenerator::RoadGenerator()
{
	itsCount = -1;
  //Generate Road Templates 
  generateRoadGroundTruthTemplates();
}
// ######################################################################
RoadGenerator::RoadGenerator(int w,int h)
{
	itsCount = -1;
  //Generate Road Templates 
  generateRoadGroundTruthTemplates(w,h);
}
// ######################################################################
Image<PixRGB<byte> > RoadGenerator::getRoad()
{
	if(itsCount >= (int)itsRoadGroundTruthTemplates.size()) itsCount = -1;
	itsCount++;
	return itsRoadGroundTruthTemplates[itsCount].getRoadImg();	
}

// ######################################################################
int RoadGenerator::getLeftAngle()
{
	return itsRoadGroundTruthTemplates[itsCount].getLeftAngle();	
}
// ######################################################################
int RoadGenerator::getRightAngle()
{
	return itsRoadGroundTruthTemplates[itsCount].getRightAngle();	
}
// ######################################################################
RoadGroundTruthTemplate RoadGenerator::getGT()
{
	return itsRoadGroundTruthTemplates[itsCount];	
}
// ######################################################################
void RoadGenerator::generateRoadGroundTruthTemplates(int w, int h)
{
  Dims dims(w,h);

  Timer tim(1000000); tim.reset();

  // get the vantage point locations
  std::vector<Point2D<int> > vp;
  int vpgsize = VANTAGE_POINT_GRID_SIZE;
	for(int j = vpgsize; j <= h/2; j += vpgsize)
		for(int i = vpgsize; i < w; i += vpgsize)
      vp.push_back(Point2D<int>(i,j));

  int angi = ANGLE_INCREMENT;

  int ls = LEFT_BOUNDARY_START_ANGLE;
  int re = RIGHT_BOUNDARY_END_ANGLE;
  int mins = MIN_ROAD_ANGLE_SPACING;
  int maxs = MAX_ROAD_ANGLE_SPACING;

  itsCSpoints.clear();
  itsRoadGroundTruthTemplates.clear();

  // create the fan area for each vantage point
  for(uint i = 0; i < vp.size(); i++)
    {
      //uint ct = 0;
      // right boundary
      for(int j = ls ; j <= (re-mins)/2; j+= angi) 
        {
          // left boundary
          for(int k = j+mins; k <= maxs; k += angi)
            {
              //LINFO("[%3d]: %d %d", ct, j, k); ct++;

              Point2D<int> bpr = computeBottomPoint(vp[i],j, dims);
              Point2D<int> bpl = computeBottomPoint(vp[i],k, dims);
							int left  = bpl.i < 0 ? 0 : bpl.i;
							int right = bpr.i >= w ? w-1 : bpr.i;
							if(abs(right - left) > w/4)//road width need greater than 1/4 of image
								itsRoadGroundTruthTemplates.push_back(RoadGroundTruthTemplate(vp[i],bpl,bpr,k,j,dims));	

            }
        }
    }


  LINFO("Create Template time: %f", tim.get()/1000.0F);
  //Raster::waitForKey();
}

// ######################################################################
Point2D<int> RoadGenerator::
computeBottomPoint(Point2D<int> point,float angle, Dims dims)
{
  //tan(a) = (H - Y1) / (X2 - X1)
  //X2 = ((H-Y1) / tan(a) ) + x1
  //Angle from right to left, ex 45 deg is from middle to right bottom
  if(angle <= 0.0){
    LINFO("Angle %3.1f is too small, set to 5.00",angle);
    angle = 5.0;
  }
  int x1 = point.i,y1 = point.j,y2 = dims.h()-1;
  float x2 = ((y2 - y1) / tan((angle/180.0)*M_PI))+x1;
  //LINFO("x1,y1=(%d,%d), x2,y2=(%d,%d),angle %f",x1,y1,(int)x2,y2,angle);
  return Point2D<int>(x2,y2);	
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

/*!@file
   Robots/Beobot2/LaneRecognition/AppearanceContrastTrailRecognizer.C
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

#include "Gist/Clustering.H"
#include "Robots/Beobot2/LaneFollowing/AppearanceContrastTrailRecognizer.H"

#include "Image/ColorOps.H"
#include "Image/CutPaste.H"
#include "Image/ShapeOps.H"
#include "Image/MathOps.H"
#include "Image/DrawOps.H"
#include "Util/Timer.H"

#define NUM_CLUSTERS     10
#define NUM_ITERATIONS   10

#define DEFAULT_WIDTH    160
#define DEFAULT_HEIGHT   120
#define DEFAULT_DIAGONAL 200

#define VANISHING_POINT_GRID_SIZE      DEFAULT_WIDTH/8
#define GRID_SIZE                      4

#define LEFT_BOUNDARY_START_ANGLE     20
#define RIGHT_BOUNDARY_END_ANGLE     160
#define ANGLE_INCREMENT               10  // WAS 5
#define MIN_ROAD_ANGLE_SPACING        20
#define MAX_ROAD_ANGLE_SPACING       140

#define GRID_AREA                    GRID_SIZE*GRID_SIZE
#define INCLUDE_THRESHOLD            0.50F 


// random color
inline PixRGB<byte> random_rgb(){ 
  PixRGB<byte> c((byte)random(),
                 (byte)random(),
                 (byte)random());
  return c;
}

// ######################################################################
AppearanceContrastTrailRecognizer::AppearanceContrastTrailRecognizer()
{
  itsWin.reset();
  
  // Center Surround templates to estimate salient region shape
  Timer tim(1000000); tim.reset();
  computeAllCSRoadTemplates(DEFAULT_WIDTH, DEFAULT_HEIGHT);


  itsPreviousRoadLikelihood.clear(); 
  itsPreviousRoadLikelihood.resize(itsRoadTemplates.size());
  for(uint i = 0; i < itsPreviousRoadLikelihood.size(); i++) 
    itsPreviousRoadLikelihood[i] = 0.0f;

  LINFO("compute templates: %f", tim.get()/1000.0F); tim.reset();

  itsMiddlePoint = Point2D<int>(-1,-1);
  itsVanishingPoint = Point2D<int>(-1,-1);

}

// ######################################################################
AppearanceContrastTrailRecognizer::~AppearanceContrastTrailRecognizer()
{ }

// ######################################################################
void AppearanceContrastTrailRecognizer::computeAllCSRoadTemplates(int w, int h)
{
  itsCurrentCStemplateWidth  = w;
  itsCurrentCStemplateHeight = h;

  Dims dims(w,h);

  Timer tim(1000000); tim.reset();

  // get the vanishing point locations
  std::vector<Point2D<int> > vp;
  int vpgsize = VANISHING_POINT_GRID_SIZE;
  for(int i = vpgsize; i < w-vpgsize; i += vpgsize)
    for(int j = 2*vpgsize; j <= h/2; j += vpgsize)
      vp.push_back(Point2D<int>(i,j));

  itsCSpoints.clear();
  itsRoadTemplates.clear();
  int angi = ANGLE_INCREMENT;
  
  int ls = LEFT_BOUNDARY_START_ANGLE;
  int re = RIGHT_BOUNDARY_END_ANGLE;
  int mins = MIN_ROAD_ANGLE_SPACING;
  int maxs = MAX_ROAD_ANGLE_SPACING;

  // create the fan area for each vanishing point
  for(uint i = 0; i < vp.size(); i++)
    {
      //uint ct = 0;
      // right boundary
      for(int j = ls ; j <= re-mins; j+= angi) 
        {
          // left boundary
          for(int k = j+mins; k <= maxs; k += angi)
            {
              //LINFO("CS: (%3d %3d): Angle: %3d %3d", vp[i].i, vp[i].j, j, k);

              std::pair
                <std::vector<Point2D<int> >,
                 std::pair<std::vector<Point2D<int> >,
                           std::vector<Point2D<int> > > > 
                cs = 
                computeCSRoadTemplates(vp[i],k,j,dims);

              // make sure the road size is large
              if(cs.first.size() > 30)
                {
                  Point2D<int> bpr = computeBottomPoint(vp[i],j, dims);
                  Point2D<int> bpl = computeBottomPoint(vp[i],k, dims);
                  //int left  = bpl.i < 0 ? 0 : bpl.i;
                  //int right = bpr.i >= w ? w-1 : bpr.i;
                  int left  = bpl.i;
                  int right = bpr.i;
                  //road width need greater than 1/4 of image
                  if(abs(right - left) > w/4 && left > -vpgsize && right < w+vpgsize)
                    {
                      itsCSpoints.push_back(cs);
                      //LINFO("added: (%3d %3d) - (%3d %3d) --> %d", 
                      //      dims.h(), left, dims.h(), right, abs(right - left));
                      itsRoadTemplates.push_back(RoadTemplate(vp[i],bpl,bpr,dims));
                    }
                }
            }
        }
    }  

  
  LINFO("Create %3d Templates in time: %f", int(itsRoadTemplates.size()), tim.get()/1000.0F);
  //Raster::waitForKey();
}

// ######################################################################
std::pair<std::vector<Point2D<int> >,
          std::pair<std::vector<Point2D<int> >,
                    std::vector<Point2D<int> > > >
AppearanceContrastTrailRecognizer::computeCSRoadTemplates
(Point2D<int> vp, int leftAngle, int rightAngle, Dims dims)
{
  int k = leftAngle;
  int j = rightAngle;
  int w = dims.w();
  int h = dims.h();

  int spacing = 4;

  Point2D<int> bpr = computeBottomPoint(vp,j, dims);
  Point2D<int> bpl = computeBottomPoint(vp,k, dims);

  Image<float> disp(dims, ZEROS);

  int top = vp.j;

  std::vector<Point2D<int> > rTemplate;
  std::vector<Point2D<int> > sTemplateL;
  std::vector<Point2D<int> > sTemplateR;
  //Rectangle sRect(Point2D<int>(0,top/GRID_SIZE), Dims(w,h-top)/GRID_SIZE);

  for(int jj = top; jj < h; jj += GRID_SIZE)
    {
      int bottom = jj + GRID_SIZE-1; 

      Point2D<int> pleft  = 
        intersectPoint(vp,bpl, Point2D<int>(0,bottom),Point2D<int>(w,bottom));
      Point2D<int> pright = 
        intersectPoint(vp,bpr, Point2D<int>(0,bottom),Point2D<int>(w,bottom));

      //LINFO("jj: %d", jj);
      //LINFO("bpl[%3d %3d] bpr[%3d %3d]", bpl.i, bpl.j, bpr.i, bpr.j);
      //LINFO("pl [%3d %3d]  pr[%3d %3d]", pleft.i, pleft.j, pright.i, pright.j);

      int left = pleft.i  < vp.i ? pleft.i  : vp.i;
      int right= pright.i > vp.i ? pright.i : vp.i;

      left  = left/GRID_SIZE * GRID_SIZE;
      right = right/GRID_SIZE* GRID_SIZE + GRID_SIZE;

      bool surroundStarted = false;

      //LINFO("left: %d right: %d", left, right);
      for(int ii = left; ii < right; ii += GRID_SIZE)
        {
          int vcount = 0; int maxcount = GRID_AREA; 
          for(int iii = ii; iii < ii+GRID_SIZE; iii++)
            for(int jjj = jj; jjj < jj+GRID_SIZE; jjj++)
              {
                int Ax = vp.i;  int Ay = vp.j;
                int Bx = bpl.i;    int By = bpl.j;
                int  sRoad1 = (Bx-Ax)*(jjj-Ay) - (By-Ay)*(iii-Ax);
                bool inRoad1 = (sRoad1 <= 0);
                Bx = bpr.i;        By = bpr.j;
                int  sRoad2 = (Bx-Ax)*(jjj-Ay) - (By-Ay)*(iii-Ax);
                bool inRoad2 = (sRoad2 >= 0);

                if(inRoad1 && inRoad2) vcount++;
                //LINFO("%d %d => %d",inRoad1, inRoad2, vcount);
              }

          // LINFO("<<%d %d>>", ii,jj);
          Point2D<int> pt(ii,jj); 
          if(disp.coordsOk(pt) && vcount > maxcount* INCLUDE_THRESHOLD)
            {
              // add the suround              
              if(!surroundStarted) 
                {
                  // add surround to end the previous row first
                  if(rTemplate.size() > 0)
                    {
                      Point2D<int> rpoint = rTemplate[rTemplate.size()-1];
                      int kk = rpoint.i+1;
                      int max = rpoint.i + spacing;
                      if(max > (w/GRID_SIZE-1)) max = w/GRID_SIZE-1;
                      while(kk <= max)
                        {
                          sTemplateR.push_back(Point2D<int>(kk,rpoint.j));
                          kk++;
                        }
                    }
                }
              
              rTemplate.push_back(pt/GRID_SIZE);

              if(!surroundStarted) 
                {
                  // add surround to start the current row first
                  Point2D<int> lpoint = rTemplate[rTemplate.size()-1];
                  int kk = lpoint.i-1;
                  int min = lpoint.i - spacing;
                  if(min < 0) min = 0;
                  while(kk >= min)
                    {
                      sTemplateL.push_back(Point2D<int>(kk,lpoint.j));
                      kk--;
                    }
                  surroundStarted = true;
                }
              // LINFO("%d %d  --> %d", ii/GRID_SIZE, jj/GRID_SIZE, vcount);
              // Raster::waitForKey();
            }
        }
    }

  // add surround to end of the last row
  if(rTemplate.size() > 0)
    {
      Point2D<int> rpoint = rTemplate[rTemplate.size()-1];
      int kk = rpoint.i+1;
      int max = rpoint.i + spacing;
      if(max > (w/GRID_SIZE-1)) max = w/GRID_SIZE-1;
      while(kk <= max)
        {
          sTemplateR.push_back(Point2D<int>(kk,rpoint.j));
          kk++;
        }
    }
  
// Image<float> disp2(dims/GRID_SIZE, ZEROS);
// if(itsWin.is_invalid())
//   itsWin.reset(new XWinManaged(disp.getDims(), 0, 0, "CSHroadrec"));
// else itsWin->setDims(disp.getDims());

// for(uint ti = 0; ti < rTemplate.size(); ti++)
//   disp2.setVal(rTemplate[ti], 1.0F);
// inplacePaste(disp, zoomXY(disp2,GRID_SIZE), Point2D<int>(0,0));
// itsWin->drawImage(disp,0,0);
//Raster::waitForKey();          
 
  return 
    (std::pair<std::vector<Point2D<int> >,
               std::pair<std::vector<Point2D<int> >,
                         std::vector<Point2D<int> > > >
    (rTemplate, 
     std::pair<std::vector<Point2D<int> >, std::vector<Point2D<int> > >
     (sTemplateL, sTemplateR)));
}

// ######################################################################
Point2D<int> AppearanceContrastTrailRecognizer::
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
Point2D<int> AppearanceContrastTrailRecognizer::intersectPoint
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
void AppearanceContrastTrailRecognizer::computeRoad(Image<PixRGB<byte> > image)
{
  itsImage = image;
  int w = image.getWidth();
  int h = image.getHeight();

  // check if we need to recompute templates 
  // if the dimensions are not identical
  if(itsCurrentCStemplateWidth !=  w || itsCurrentCStemplateHeight != h)
    computeAllCSRoadTemplates(w,h);

  Timer tim(1000000); tim.reset();

  computeNormalizedCIElab();
  LINFO("nLAB: %f", tim.get()/1000.0F); tim.reset();

  float thresh1 = 0.55F;
  float thresh2 = 0.80F;

  //float thresh = 0.45;
  //for(uint ii = 0; ii < 25; ii++)
  //  {
      
      Image<PixRGB<byte> > timage(w,h, ZEROS);
      Image<PixRGB<byte> > timage2(w,h, ZEROS);
      for(int i = 0; i < w; i++)
        for(int j = 0; j < h; j++)
          {
            float l = itsLabImage[0].getVal(i,j);
            float a = itsLabImage[1].getVal(i,j);
            float b = itsLabImage[2].getVal(i,j);
            
            float  c2 = a*a + b*b;
            float denom = pow(c2+(l*l), 0.5); 
            //float denom = pow((l*l), 0.5); 
            float sat = 1.0F;
            if(denom != 0.0) sat = pow(c2, 0.5)/denom;
       
            if(sat < thresh1) 
              {
                timage.setVal (i,j, itsImage.getVal(i,j));
                timage2.setVal(i,j, PixRGB<byte>(255,255,255));

                itsLabImage[0].setVal(i,j, 0.0F);
                itsLabImage[1].setVal(i,j, 0.0F);
                itsLabImage[2].setVal(i,j, 0.0F);

                //itsImage.setVal(i,j, PixRGB<byte>(255,255,255));
              }

            if(sat > thresh2) 
              {
                timage.setVal (i,j, itsImage.getVal(i,j));
                timage2.setVal(i,j, PixRGB<byte>(255,255,255));

                itsLabImage[0].setVal(i,j, 1.0F);
                itsLabImage[1].setVal(i,j, 1.0F);
                itsLabImage[2].setVal(i,j, 1.0F);
              }
          }
      // LINFO("thresh %f", thresh);
      // thresh += 0.01F;

      // Image<PixRGB<byte> > disp(3*w,h, ZEROS);
      // inplacePaste(disp, itsImage, Point2D<int>(0,0));
      // inplacePaste(disp, timage,   Point2D<int>(w,0));
      // inplacePaste(disp, timage2,   Point2D<int>(2*w,0));
      
      // Image<PixRGB<byte> > disp = itsImage;
      // if(itsWin.is_invalid())
      //   itsWin.reset(new XWinManaged(disp.getDims(), 0, 0, "CSHroadrec"));
      // else itsWin->setDims(disp.getDims());
      // itsWin->drawImage(disp,0,0);
      // Raster::waitForKey();
      //}

  // compute the k-means
  // Image<PixRGB<byte> > labImage;
  // labImage = 
  //   makeRGB(Image<byte>(itsLabImage[0]*255.0),
  //           Image<byte>(itsLabImage[1]*255.0),
  //           Image<byte>(itsLabImage[2]*255.0) );
  //itsLabelImage = getKMeansLabel(labImage, NUM_CLUSTERS, 0.0);

  computeKMeans();

  setKMeansDisplayImage();

  //computeKMeans();

 // create the grid histogram and integral images
  computeGridHistogram();
  computeHistogramIntegralImage();
  LINFO("cHistII time: %f",tim.get()/1000.0F);

  // calculate the road likelihoods
  findRoad();
  LINFO("total time: %f",tim.get()/1000.0F);
}

// ######################################################################
void AppearanceContrastTrailRecognizer::computeNormalizedCIElab()
{
  // convert to CIElab color space
  Image<float> lImg; 
  Image<float> aImg;
  Image<float> bImg;
  getNormalizedLAB(itsImage, lImg, aImg, bImg);

  itsLabImage.clear();
  itsLabImage.push_back(lImg);
  itsLabImage.push_back(aImg);
  itsLabImage.push_back(bImg);
}

// ######################################################################
void AppearanceContrastTrailRecognizer::computeKMeans()
{
  uint w = itsImage.getWidth();
  uint h = itsImage.getHeight();

  std::vector<std::vector<float> > centroids(NUM_CLUSTERS);

  // get an initial centroid
  std::vector<float> c0(3);
  for(uint j = 0; j < 3; j++) c0[j] = 0.0F;      
  centroids[0] = c0;  
  std::vector<float> c1(3);
  for(uint j = 0; j < 3; j++) c1[j] = 1.0F;      
  centroids[1] = c1;  

  for(uint i = 2; i < centroids.size(); i++)
    {     
      bool picked = false;
      while(!picked)
        {
          int x = int(rand()/(RAND_MAX + 1.0) * w);
          int y = int(rand()/(RAND_MAX + 1.0) * h);
          
          std::vector<float> c(3);
          for(uint j = 0; j < 3; j++) c[j] = itsLabImage[j].getVal(x,y);      

          float maxDiff = 0.0F;
          for(uint ii = 0; ii < i; ii++)
            { 
              float diff = 0.0F;
              for(uint j = 0; j < 3; j++) 
                diff += (c[j]-centroids[ii][j])*(c[j]-centroids[ii][j]); 
              diff = pow(diff, 0.5);

              if(maxDiff < diff) maxDiff = diff;
              
            }

          if(maxDiff > .2)
            { centroids[i] = c; picked = true; }                 
        }
    }

  // for initialize clusters
  std::vector<std::vector<std::vector<float> > > clusters(NUM_CLUSTERS);
  for(uint i = 0; i < NUM_CLUSTERS; i++)
    clusters.push_back(std::vector<std::vector<float> >());

  uint total = 0;
  while(total++ < NUM_ITERATIONS)
    {
      // go through each pixel
      for(uint i = 0; i < w; i++)
	for(uint j = 0; j < h; j++)
	  {
	    // put pixel to cluster
	    std::vector<float> cpix(3);
	    for(uint k = 0; k < 3; k++) 
	      cpix[k] = itsLabImage[k].getVal(i,j);
	    
	    uint mindex = 0; 
	    float min_diff = getDiff(centroids[0], cpix);
	    for(uint k = 1; k < NUM_CLUSTERS; k++)
	      {
		float diff = getDiff(centroids[k], cpix);
		if(min_diff > diff)
		  { mindex = k; min_diff = diff; }	    
	      }
	
	    clusters[mindex].push_back(cpix);
	  }

      // recompute the centroid
      for(uint i = 0; i < NUM_CLUSTERS; i++)
	{
	  std::vector<float> totalc(3);
	  for(uint j = 0; j < 3; j++) totalc[j] = 0.0;

	  for(uint j = 0; j < clusters[i].size(); j++)
	    {
	      for(uint k = 0; k < 3; k++)
		totalc[k] += clusters[i][j][k];
	    }

	  for(uint k = 0; k < 3; k++)
	    centroids[i][k] = totalc[k]/(clusters[i].size()+0.0F);	  
	}

      //LINFO("iteration %d", total);
    }

  // label the pixels
  itsLabelImage = Image<int>(w,h, NO_INIT);
  for(uint i = 0; i < w; i++)
    for(uint j = 0; j < h; j++)
      {
	// put pixel to cluster
	std::vector<float> cpix(3);
	for(uint k = 0; k < 3; k++) 
	  cpix[k] = itsLabImage[k].getVal(i,j);

	uint mindex = 0; 
	float min_diff = getDiff(centroids[0], cpix);
	for(uint k = 1; k < NUM_CLUSTERS; k++)
	  {
	    float diff = getDiff(centroids[k], cpix);
	    if(min_diff > diff)
	      { mindex = k; min_diff = diff; }	    
	  }
	//LINFO("mindex[%3d %3d]: %d", i,j, mindex);
	
	itsLabelImage.setVal(i,j, mindex);
      }
}

// ######################################################################
float AppearanceContrastTrailRecognizer::getDiff
(std::vector<float> a, std::vector<float> b)
{
  ASSERT(a.size() == b.size());

  float diff = 0.0;
  for(uint i = 0; i < a.size(); i++)
    diff += (a[i]-b[i])*(a[i]-b[i]);
  
  return pow(diff, 0.5);
}

// ######################################################################
void AppearanceContrastTrailRecognizer::computeGridHistogram()
{
  int gwidth  = itsImage.getWidth() /GRID_SIZE;
  int gheight = itsImage.getHeight()/GRID_SIZE;

  itsGridHistogram = 
    Image<rutz::shared_ptr<Histogram> >(gwidth, gheight, ZEROS);

  // compute histogram for each grid location
  for(int y = 0; y < gheight; y++)
    {
      int yy = y*GRID_SIZE;
      for(int x = 0; x < gwidth; x++)
        {
          int xx = x*GRID_SIZE;
          rutz::shared_ptr<Histogram> h = 
            getHistogramDistribution(Point2D<int>(xx,yy), GRID_SIZE);
          itsGridHistogram.setVal(x,y, h);
        }
    }
}

// ######################################################################
void AppearanceContrastTrailRecognizer::computeHistogramIntegralImage()
{
  int gwidth  = itsImage.getWidth() /GRID_SIZE;
  int gheight = itsImage.getHeight()/GRID_SIZE;

  // create integral image from the grid histogram
  itsIntegralHistogram = 
    Image<rutz::shared_ptr<Histogram> >(gwidth, gheight, ZEROS);

  std::vector<Histogram> s(gwidth);
  for (int i = 0; i < gwidth; i++) 
    {
      Histogram h(NUM_CLUSTERS);
      for(int j = 0; j < NUM_CLUSTERS; j++)
        h.setValue(j, 0.0F);
      s[i] = h;
    }

  for(int y = 0; y < gheight; y++)
    {
      rutz::shared_ptr<Histogram> hist = itsGridHistogram.getVal(0,y);      
      s[0] = s[0] + *hist;

      itsIntegralHistogram.setVal
        (0,y, rutz::shared_ptr<Histogram>(new Histogram(s[0])));
      
      for(int x = 1; x < gwidth; x++)
        {
          rutz::shared_ptr<Histogram> h1 = itsIntegralHistogram.getVal(x-1,y);
          rutz::shared_ptr<Histogram> h2 = itsGridHistogram.getVal(x,y); 

          s[x] = s[x] + *h2;
          Histogram th2 = *h1 + s[x]; 

          itsIntegralHistogram.setVal
            (x,y, rutz::shared_ptr<Histogram>(new Histogram(th2)));
        }
    }
}

// ######################################################################
void AppearanceContrastTrailRecognizer::findRoad()
{
  Timer tim(1000000); tim.reset();

  // display window
  uint width  = itsImage.getWidth();
  uint height = itsImage.getHeight();

  Point2D<int> gpt(0,0);

  itsTopRoadTemplates.empty();

  LINFO("prev vp: %d %d", itsVanishingPoint.i, itsVanishingPoint.j);

  // try various center surround combination
  float maxDiff = 0.0F;
  std::vector<float> tempDiff(itsCSpoints.size());
  for(uint i = 0; i < itsCSpoints.size(); i++)
    {
      Timer tim(1000000); tim.reset();
      std::vector<Point2D<int> > cPoints  = itsCSpoints[i].first;
      std::vector<Point2D<int> > slPoints = itsCSpoints[i].second.first;
      std::vector<Point2D<int> > srPoints = itsCSpoints[i].second.second;
      //Rectangle                  sRect   = itsCSpoints[i].second;

      // create center histogram from the center points
      // make sure the points are in bounds
      Histogram hC; bool cinit = false; 
      std::vector<Point2D<int> > cAPoints;
      for(uint j = 0; j < cPoints.size(); j++)
        {
          Point2D<int> pt = gpt + cPoints[j];
          if(!itsGridHistogram.coordsOk(pt)) continue;
             
          cAPoints.push_back(pt);
          if(!cinit)
            { hC = *itsGridHistogram.getVal(pt); cinit = true; }
          else
            hC = hC + *itsGridHistogram.getVal(pt);  
        }

      Histogram hSL; bool slinit = false; 
      std::vector<Point2D<int> > slAPoints;
      for(uint j = 0; j < slPoints.size(); j++)
        {
          Point2D<int> pt = gpt + slPoints[j];
          if(!itsGridHistogram.coordsOk(pt)) continue;
             
          slAPoints.push_back(pt);
          if(!slinit)
            { hSL = *itsGridHistogram.getVal(pt); slinit = true; }
          else
            hSL = hSL + *itsGridHistogram.getVal(pt);  
        }

      Histogram hSR; bool srinit = false; 
      std::vector<Point2D<int> > srAPoints;
      for(uint j = 0; j < srPoints.size(); j++)
        {
          Point2D<int> pt = gpt + srPoints[j];
          if(!itsGridHistogram.coordsOk(pt)) continue;
             
          srAPoints.push_back(pt);
          if(!srinit)
            { hSR = *itsGridHistogram.getVal(pt); srinit = true; }
          else
            hSR = hSR + *itsGridHistogram.getVal(pt);  
        }
      Histogram hS = hSL + hSR;

      //Histogram hCS = getGridHistogramDistribution(sRect);
      //Histogram hS = hCS - hC;
      //int npointS = sRect.area()*GRID_SIZE*GRID_SIZE - npointC;
  
      // normalize the resulting histogram
      int npointC  = cAPoints.size() *GRID_SIZE*GRID_SIZE;
      int npointSL = slAPoints.size()*GRID_SIZE*GRID_SIZE;
      int npointSR = srAPoints.size()*GRID_SIZE*GRID_SIZE;
      hC.divide(npointC);
      hSL.divide(npointSL);
      hSR.divide(npointSR);
      hS.divide(npointSL+npointSR);

      float diff1 = hC.getChiSqDiff(hSL);
      float diff2 = hC.getChiSqDiff(hSR);
      float diff3 = hSL.getChiSqDiff(hSR);      

      // print the difference
      float diff = (diff1 + diff2 + (1.0F - diff3))/3.0F;
      //float diff = hC.getChiSqDiff(hS);

      tempDiff[i] = diff;

      //float prevDiff  = itsPreviousRoadLikelihood[i];

      Point2D<int> vp = itsRoadTemplates[i].getVanishingPoint();
      float dist = vp.distance(itsVanishingPoint);
      float prevDiff = 0.0;
      if(itsVanishingPoint.i >= 0)
        {
          if(dist <= 40) 
            {
              prevDiff = 0.25*((40 - dist)/DEFAULT_DIAGONAL);
            }
          if(dist >= 80)
            diff = 0.0f;
 
        }

      float finalDiff = diff + prevDiff;
      //LINFO("[%3d] diff: %f + prevDiff %f (%f/%d) = %f", 
      //      i, diff, prevDiff, dist, DEFAULT_DIAGONAL, finalDiff);

      if(finalDiff > maxDiff) { maxDiff = finalDiff;}
      itsRoadTemplates[i].setDiff(finalDiff);
      itsRoadTemplates[i].setIndex(i);
      itsTopRoadTemplates.push(itsRoadTemplates[i]);
      if(itsTopRoadTemplates.size() > 3)//remove extra
        itsTopRoadTemplates.pop();

      // // update the center belief estimation
      // // we store the max as the best estimation of belief
      // for(uint cc = 0; cc < cAPoints.size(); cc++)
      //   {
      //     Point2D<int> pt = cAPoints[cc];
      //     float prevC = itsGridCenterBelief.getVal(pt);
      //     if(prevC < diff) itsGridCenterBelief.setVal(pt, diff);
      //   }        

      // // update the surround belief estimation
      // for(uint ss = 0; ss < sAPoints.size(); ss++)
      //   {
      //     Point2D<int> pt = sAPoints[ss];
      //     float prevS = itsGridSurroundBelief.getVal(pt);
      //     if(prevS < diff) itsGridSurroundBelief.setVal(pt, diff);
      //   }
    }

  // display the window      
  itsDisplayImage.resize(width, height, ZEROS); 

  for(uint i = 0; i < itsPreviousRoadLikelihood.size(); i++)
    itsPreviousRoadLikelihood[i] = 0.0;

  RoadTemplate rt = itsTopRoadTemplates.top(); 
	itsHighestRoadTemplate = rt;
  while(!itsTopRoadTemplates.empty())
    {
      itsTopRoadTemplates.pop();
      rt = itsTopRoadTemplates.top();     
      //float diff = rt.getDiff();
      LINFO("[%3d] road contrast difference: %f", rt.getIndex(), rt.getDiff());
      itsPreviousRoadLikelihood[rt.getIndex()] = tempDiff[rt.getIndex()];
    }
  int index = rt.getIndex();
    
  std::vector<Point2D<int> > cPoints  = itsCSpoints[index].first;
  std::vector<Point2D<int> > slPoints = itsCSpoints[index].second.first;
  std::vector<Point2D<int> > srPoints = itsCSpoints[index].second.second;
  //Rectangle                  sRect   = itsCSpoints[index].second;
  
  itsMiddlePoint = itsRoadTemplates[index].getMiddlePoint();
  itsVanishingPoint = itsRoadTemplates[index].getVanishingPoint();
  
  float mVal = 127;
  float bVal = 255 - mVal;
  
  Image<byte> dImaR, dImaG, dImaB;
  getComponents(itsImage, dImaR, dImaG, dImaB);
  inplaceNormalize(dImaR, byte(0), byte(mVal));
  inplaceNormalize(dImaG, byte(0), byte(mVal));
  inplaceNormalize(dImaB, byte(0), byte(mVal));
  Image<PixRGB<byte> > dIma  = makeRGB(dImaR,dImaG,dImaB);      
  //inplacePaste (disp, dIma, Point2D<int>(0,0));
  
  Image<PixRGB<byte> > dImaCS(width,height, ZEROS);
  //drawFilledRect(dImaCS, sRect*GRID_SIZE, PixRGB<byte>(0, byte(bVal),0));
  for(uint j = 0; j < cPoints.size(); j++)
    drawFilledRect(dImaCS, Rectangle(cPoints[j],Dims(1,1))*GRID_SIZE, 
                   PixRGB<byte>(byte(bVal),0,0));
  // for(uint j = 0; j < slPoints.size(); j++)
  //   drawFilledRect(dImaCS, Rectangle(slPoints[j],Dims(1,1))*GRID_SIZE, 
  //                  PixRGB<byte>(0,byte(bVal),0));
  // for(uint j = 0; j < srPoints.size(); j++)
  //   drawFilledRect(dImaCS, Rectangle(srPoints[j],Dims(1,1))*GRID_SIZE, 
  //                  PixRGB<byte>(0,0,byte(bVal)));
  
  Point2D<int> vp = rt.getVanishingPoint();
  Point2D<int> lp = rt.getLeftPoint();
  Point2D<int> rp = rt.getRightPoint();
  //Point2D<int> mp = rt.getMiddlePoint();

  Point2D<int> t1 = rp - vp;
  Point2D<int> t2 = lp - vp;
  float rang = atan2(t1.j,t1.i);
  float lang = atan2(t2.j,t2.i);

  float slack = 15*M_PI/180.0;
  float rlen  =  fabs(t1.j/sin(rang-slack));
  float llen  =  fabs(t2.j/sin(lang+slack));

  int bottom = lp.j;
  Point2D<int> rfp(vp.i+ rlen*cos(rang-slack), bottom);
  Point2D<int> lfp(vp.i+ llen*cos(lang+slack), bottom);

  LINFO("vp: %d %d| lp: %d %d | rp: %d %d", vp.i, vp.j, lp.i, lp.j, rp.i, rp.j);
  LINFO("t1: %d %d -> %f | t2: %d %d -> %f", t1.i, t1.j, rang, t2.i, t2.j, lang);
  LINFO("l: %d r: %d ", lfp.i, rfp.i);

  itsDisplayImage = itsImage;
  itsDisplayImage = Image<PixRGB<byte> >(dIma+dImaCS);
  drawLine(itsDisplayImage, vp, lp,  PixRGB<byte>(255,0,0),1);
  drawLine(itsDisplayImage, vp, rp,  PixRGB<byte>(255,0,0),1);
  drawLine(itsDisplayImage, vp, lfp, PixRGB<byte>(0,0,255),1);
  drawLine(itsDisplayImage, vp, rfp, PixRGB<byte>(0,0,255),1);
  //drawLine(itsDisplayImage, vp, mp, PixRGB<byte>(255,0,0),2);

  // if(itsWin.is_invalid())
  //   itsWin.reset(new XWinManaged(itsDisplayImage.getDims(), 0, 0, "CSHroadrec"));
  // else itsWin->setDims(itsDisplayImage.getDims());
  // itsWin->drawImage(itsDisplayImage,0,0);
  // Raster::waitForKey();

  LINFO("time: %f", tim.get()/1000.0F);
}

// ######################################################################
Histogram 
AppearanceContrastTrailRecognizer::getGridHistogramDistribution
(Rectangle r)
{
  int tt = r.top(), bb = r.bottomI(), ll = r.left(), rr = r.rightI();

  // integral image sum: l4 + l1 - (l2 + l3)
  rutz::shared_ptr<Histogram> l1, l2, l3, l4; 
  if(ll != 0 && tt != 0) l1 = itsIntegralHistogram.getVal(ll-1, tt-1); 
  if(tt != 0)            l2 = itsIntegralHistogram.getVal(rr  , tt-1);
  if(ll != 0)            l3 = itsIntegralHistogram.getVal(ll-1, bb  );
  l4 = itsIntegralHistogram.getVal(rr,bb);

  Histogram temp;
  if(ll == 0 && tt == 0) temp = *l4;
  else if(tt == 0)       temp = *l4 - *l3; 
  else if(ll == 0)       temp = *l4 - *l2; 
  else                   temp = *l4 + *l1 - (*l2 + *l3);

  return temp; 
}

// ######################################################################
rutz::shared_ptr<Histogram> 
AppearanceContrastTrailRecognizer::getHistogramDistribution
(Point2D<int> pt, int grid_size)
{
  rutz::shared_ptr<Histogram> histogram(new Histogram(NUM_CLUSTERS));

  int rad = grid_size;
  for(int i = 0; i < rad; i++) 
    for(int j = 0; j < rad; j++) 
      {
        Point2D<int> pt2 = pt + Point2D<int>(i,j);
        if(itsImage.coordsOk(pt2))
          {
            int val = itsLabelImage.getVal(pt2);
            histogram->addValue(val, 1.0F);
          }
      }

  return histogram;
}

// ######################################################################
void AppearanceContrastTrailRecognizer::setKMeansDisplayImage()
{
  uint w = itsImage.getWidth();
  uint h = itsImage.getHeight();

  std::vector<PixRGB<byte> > labels(NUM_CLUSTERS);
  for(uint i = 0; i < NUM_CLUSTERS; i++)
    {
      labels[i] = random_rgb();
      //LINFO("%d %d %d", labels[i].red(), labels[i].green(), labels[i].blue());
    }

  Image<PixRGB<byte> > disp(w,h, ZEROS);
  for(uint i = 0; i < w; i++)
    for(uint j = 0; j < h; j++)
      {
	disp.setVal
	  (i,j, labels[itsLabelImage.getVal(i,j)]);
      }

  itsKmeansDisplayImage = disp;
}

// ######################################################################
Image<PixRGB<byte> > AppearanceContrastTrailRecognizer::getKMeansDisplayImage()
{
  return itsKmeansDisplayImage;
}

// ######################################################################
Image<PixRGB<byte> > AppearanceContrastTrailRecognizer::getDisplayImage()
{
  return itsDisplayImage;
}

// ######################################################################
Point2D<int> AppearanceContrastTrailRecognizer::getMiddlePoint()
{
  return itsMiddlePoint;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

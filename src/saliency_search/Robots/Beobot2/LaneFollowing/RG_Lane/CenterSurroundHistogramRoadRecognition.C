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

#include "Image/Kernels.H"
#include "Image/ColorOps.H"
#include "Image/MathOps.H"
#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Image/ShapeOps.H"

#include "Util/Timer.H"

#include "GUI/XWinManaged.H"

#include "Robots/Beobot2/LaneFollowing/RG_Lane/CenterSurroundHistogramRoadRecognition.H"


#define DEFAULT_WIDTH                 80 
#define DEFAULT_HEIGHT                60

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
CenterSurroundHistogramRoadRecognition::CenterSurroundHistogramRoadRecognition()
{
  itsWin.reset();

  // These color features are inspired by Martin, PAMI 2004 pb paper
  float bg_smooth_sigma    = 0.1;       // bg histogram smoothing sigma
  float cg_smooth_sigma    = 0.05;      // cg histogram smoothing sigma

  // Gaussian smoothing kernel for each color channel
  itsLKernel = 
    gaussian<float>(1.0F, NUM_L_BINS*bg_smooth_sigma, 
                    int(3.0F*NUM_L_BINS*bg_smooth_sigma + 0.5F));
  itsAKernel = 
    gaussian<float>(1.0F, NUM_A_BINS*cg_smooth_sigma, 
                    int(3.0F*NUM_A_BINS*cg_smooth_sigma + 0.5F));
  itsBKernel = 
    gaussian<float>(1.0F, NUM_B_BINS*cg_smooth_sigma, 
                    int(3.0F*NUM_B_BINS*cg_smooth_sigma + 0.5F));

  // normalize the kernels
  float suml = sum(itsLKernel); itsLKernel /= suml;
  float suma = sum(itsAKernel); itsAKernel /= suma;
  float sumb = sum(itsBKernel); itsBKernel /= sumb;

  // Center Surround templates to estimate salient region shape
  computeAllCSRoadTemplates();

  itsMiddlePoint = Point2D<int>(0,0);
  itsVanishingPoint = Point2D<int>(0,0);
}

// ######################################################################
std::pair<std::vector<Point2D<int> >,Rectangle>
CenterSurroundHistogramRoadRecognition::computeCSRoadTemplates
(Point2D<int> vp,int leftAngle,int rightAngle,Dims dims)
{

  int k = leftAngle;
  int j = rightAngle;
  int w = dims.w();
  int h = dims.h();

  Point2D<int> bpr = computeBottomPoint(vp,j, dims);
  Point2D<int> bpl = computeBottomPoint(vp,k, dims);

  Image<float> disp(dims, ZEROS);

  int top = vp.j;

  std::vector<Point2D<int> > rTemplate;
  Rectangle sRect(Point2D<int>(0,top/GRID_SIZE), Dims(w,h-top)/GRID_SIZE);

  for(int jj = top; jj < h; jj += GRID_SIZE)
    {
      int bottom = jj + GRID_SIZE-1; 

      Point2D<int> pleft  = 
        intersectPoint(vp,bpl, Point2D<int>(0,bottom),Point2D<int>(w,bottom));
      Point2D<int> pright = 
        intersectPoint(vp,bpr, Point2D<int>(0,bottom),Point2D<int>(w,bottom));

      // LINFO("jj: %d", jj);
      // LINFO("bpl[%3d %3d] bpr[%3d %3d]", bpl.i, bpl.j, bpr.i, bpr.j);
      // LINFO("pl [%3d %3d]  pr[%3d %3d]", pleft.i, pleft.j, pright.i, pright.j);

      int left = pleft.i  < vp.i ? pleft.i  : vp.i;
      int right= pright.i > vp.i ? pright.i : vp.i;

      left  = left/GRID_SIZE * GRID_SIZE;
      right = right/GRID_SIZE* GRID_SIZE + GRID_SIZE;

      // LINFO("left: %d right: %d", left, right);
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
            rTemplate.push_back(pt/GRID_SIZE);

          // LINFO("%d %d  --> %d", ii/GRID_SIZE, jj/GRID_SIZE, vcount);
          // Raster::waitForKey();
        }
    }//int jj,top..h

  // Image<float> disp2(dims/GRID_SIZE, ZEROS);


  // if(itsWin.is_invalid())
  //   itsWin.reset(new XWinManaged(disp.getDims(), 0, 0, "CSHroadrec"));
  // else itsWin->setDims(disp.getDims());

  // for(uint ti = 0; ti < rTemplate.size(); ti++)
  //   disp2.setVal(rTemplate[ti], 1.0F);
  // inplacePaste(disp, zoomXY(disp2,GRID_SIZE), Point2D<int>(0,0));
  // itsWin->drawImage(disp,0,0);
  // Raster::waitForKey();          
  return (std::pair<std::vector<Point2D<int> >,Rectangle>(rTemplate, sRect));
}

// ######################################################################
void CenterSurroundHistogramRoadRecognition::computeVpCSRoadTemplates
(Point2D<int> vp)
{
  int w = DEFAULT_WIDTH;
  int h = DEFAULT_HEIGHT;
  Dims dims(w,h);

  itsCSpoints.clear();
  itsRoadTemplates.clear();
  int angi = ANGLE_INCREMENT;
  
  int ls = LEFT_BOUNDARY_START_ANGLE;
  int re = RIGHT_BOUNDARY_END_ANGLE;
  int mins = MIN_ROAD_ANGLE_SPACING;
  int maxs = MAX_ROAD_ANGLE_SPACING;
	
  for(int j = ls ; j <= (re-mins)/2; j+= angi) 
    {
      // left boundary
      for(int k = j+mins; k <= maxs; k += angi)
        {
          std::pair<std::vector<Point2D<int> >,Rectangle> cs = 
            computeCSRoadTemplates(vp,k,j,dims);
          
          if(cs.first.size() > 10)
            {
              itsCSpoints.push_back(cs);
              
              Point2D<int> bpr = computeBottomPoint(vp,j, dims);
              Point2D<int> bpl = computeBottomPoint(vp,k, dims);
              itsRoadTemplates.push_back(RoadTemplate(vp,bpl,bpr,dims));	
              
            }		
        }
    }
}

// ######################################################################
void CenterSurroundHistogramRoadRecognition::computeAllCSRoadTemplates()
{
  int w = DEFAULT_WIDTH;
  int h = DEFAULT_HEIGHT;
  Dims dims(w,h);

  Timer tim(1000000); tim.reset();

  // get the vantage point locations
  std::vector<Point2D<int> > vp;
  int vpgsize = VANTAGE_POINT_GRID_SIZE;
  for(int i = vpgsize; i < w; i += vpgsize)
    for(int j = vpgsize; j <= h/2; j += vpgsize)
      vp.push_back(Point2D<int>(i,j));

  itsCSpoints.clear();
  itsRoadTemplates.clear();
	int angi = ANGLE_INCREMENT;

	int ls = LEFT_BOUNDARY_START_ANGLE;
	int re = RIGHT_BOUNDARY_END_ANGLE;
	int mins = MIN_ROAD_ANGLE_SPACING;
	int maxs = MAX_ROAD_ANGLE_SPACING;
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
              std::pair<std::vector<Point2D<int> >,Rectangle> cs = 
                computeCSRoadTemplates(vp[i],k,j,dims);
              
              if(cs.first.size() > 10)
                {

                  Point2D<int> bpr = computeBottomPoint(vp[i],j, dims);
                  Point2D<int> bpl = computeBottomPoint(vp[i],k, dims);
                  int left  = bpl.i < 0 ? 0 : bpl.i;
                  int right = bpr.i >= w ? w-1 : bpr.i;
                  //road width need greater than 1/4 of image
                  if(abs(right - left) > w/4)
                    {
                      itsCSpoints.push_back(cs);
                      itsRoadTemplates.push_back(RoadTemplate(vp[i],bpl,bpr,dims));	
                    }
                }		
            }
        }
    }  
  
  LINFO("Create Template time: %f", tim.get()/1000.0F);
  //Raster::waitForKey();
}

// ######################################################################
Point2D<int> CenterSurroundHistogramRoadRecognition::
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
Point2D<int> CenterSurroundHistogramRoadRecognition::intersectPoint
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
CenterSurroundHistogramRoadRecognition::
~CenterSurroundHistogramRoadRecognition()
{ }

// ######################################################################
void CenterSurroundHistogramRoadRecognition::setImage
(Image<PixRGB<byte> > image)
{ 
  // original input image
  itsImage = image;

  // final result

  Timer tim(1000000); tim.reset();

  // create the CIE lab histogram entry images
  // this is the feature histogram in Martin's pb PAMI 2004
  setImageFeatureHistogramValues();
  LINFO("fHist   time: %f",tim.get()/1000.0F); tim.reset();

  // create the grid histogram and integral images
  computeGridHistogram();
  computeHistogramIntegralImage();
  LINFO("cHistII time: %f",tim.get()/1000.0F);


  //FIXXXXXXXX: store belief
  // reset the center surround belief 
  //int gwidth  = itsImage.getWidth() /GRID_SIZE;
  //int gheight = itsImage.getHeight()/GRID_SIZE;
  //itsGridCenterBelief   = Image<float>(gwidth, gheight, ZEROS);
  //itsGridSurroundBelief = Image<float>(gwidth, gheight, ZEROS);

  // get the road 
  findRoad();
  //LINFO("road found");
  //Raster::waitForKey();
}

// ######################################################################
void CenterSurroundHistogramRoadRecognition::setImageFeatureHistogramValues()
{
  Timer tim(1000000); tim.reset();

  // convert to CIElab color space
  Image<float> lImg; 
  Image<float> aImg;
  Image<float> bImg;
  getNormalizedLAB(itsImage, lImg, aImg, bImg);
  LINFO("nLAB: %f", tim.get()/1000.0F); tim.reset();

  // convert to bin numbers
  itsImageHistogramEntries.clear();
  itsImageHistogramEntries.push_back(quantize_values(lImg, NUM_L_BINS));
  itsImageHistogramEntries.push_back(quantize_values(aImg, NUM_A_BINS));
  itsImageHistogramEntries.push_back(quantize_values(bImg, NUM_B_BINS));
  LINFO("qLAB: %f", tim.get()/1000.0F);
}

// ######################################################################
Image<int> CenterSurroundHistogramRoadRecognition::quantize_values
(Image<float> image, int num_bins)
{
  // FIXXX: May want to move to MathOps later
  uint w = itsImage.getWidth();
  uint h = itsImage.getHeight();
  Image<int> result(w,h,NO_INIT);

  Image<float>::iterator iptr = image.beginw();  
  Image<int>::iterator aptr = result.beginw(), stop = result.endw();  
  float nbins = num_bins;
  while(aptr != stop)
    {
      float in = *iptr++;

      // bins will be 0 to num_bins-1
      int bin = int(in*nbins);
      if(bin == num_bins) bin = num_bins - 1;
      *aptr++ = bin;
    }

  return result;
}

// ######################################################################
void CenterSurroundHistogramRoadRecognition::computeGridHistogram()
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
void CenterSurroundHistogramRoadRecognition::computeHistogramIntegralImage()
{
  int gwidth  = itsImage.getWidth() /GRID_SIZE;
  int gheight = itsImage.getHeight()/GRID_SIZE;

  // create integral image from the grid histogram
  itsIntegralHistogram = 
    Image<rutz::shared_ptr<Histogram> >(gwidth, gheight, ZEROS);

  std::vector<Histogram> s(gwidth);
  for (int i = 0; i < gwidth; i++) 
    {
      Histogram h(NUM_HISTOGRAM_DIMS);
      for(int j = 0; j < NUM_HISTOGRAM_DIMS; j++) 
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
rutz::shared_ptr<Histogram> 
CenterSurroundHistogramRoadRecognition::getHistogramDistribution
(Point2D<int> pt, int grid_size)
{
  rutz::shared_ptr<Histogram> histogram;

  Histogram lHist(NUM_L_BINS);
  Histogram aHist(NUM_A_BINS);
  Histogram bHist(NUM_L_BINS);

  int rad = grid_size;
  int numPoints = 0;
  for(int i = 0; i < rad; i++) 
    for(int j = 0; j < rad; j++) 
      {
        Point2D<int> pt2 = pt + Point2D<int>(i,j);
        if(itsImage.coordsOk(pt2))
          {
            // L component
            int l_val = itsImageHistogramEntries[0].getVal(pt2);
            lHist.addValue(l_val, 1.0F);

            // a component
            int a_val = itsImageHistogramEntries[1].getVal(pt2);
            aHist.addValue(a_val, 1.0F);

            // b component
            int b_val = itsImageHistogramEntries[2].getVal(pt2);
            bHist.addValue(b_val, 1.0F);

            numPoints++;
          }
      }

  // maybe later want to add the weight for each channel
  //(Point2D<int> pt, std::vector<float> &dist, float weight)

  // combine the histograms
  std::vector<Histogram> histograms;
  histograms.push_back(lHist);
  histograms.push_back(aHist);
  histograms.push_back(bHist);
  histogram.reset(new Histogram(histograms));

  return histogram;
}

// ######################################################################
void CenterSurroundHistogramRoadRecognition::findRoad()
{
  Timer tim(1000000); tim.reset();

  // display window
  uint width  = itsImage.getWidth();
  uint height = itsImage.getHeight();

  Point2D<int> gpt(0,0);

  // try various center surround combination
  float maxDiff = 0.0F; uint maxIdx = 0;
  for(uint i = 0; i < itsCSpoints.size(); i++)
    {
      Timer tim(1000000); tim.reset();
      std::vector<Point2D<int> > cPoints = itsCSpoints[i].first;
      Rectangle                  sRect   = itsCSpoints[i].second;

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

      Histogram hCS = getGridHistogramDistribution(sRect);
      Histogram hS = hCS - hC;
  
      // smooth and normalize the resulting histogram
      int npointC = cAPoints.size()*GRID_SIZE*GRID_SIZE;
      int npointS = sRect.area()*GRID_SIZE*GRID_SIZE - npointC;
      Histogram shC = smoothAndNormalize(hC, npointC);
      Histogram shS = smoothAndNormalize(hS, npointS);

      // print the difference
      float diff = shS.getChiSqDiff(shC);

      if(diff > maxDiff) { maxDiff = diff; maxIdx = i; }
			itsRoadTemplates[i].setDiff(diff);
			itsRoadTemplates[i].setIndex(i);
			itsTopFiveRoadTemplates.push(itsRoadTemplates[i]);
			if(itsTopFiveRoadTemplates.size() > 8)//remove extra
				itsTopFiveRoadTemplates.pop();


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
  itsDispImage.resize(width*4, height*4, ZEROS); 

	




  int ct = 0;
	while(!itsTopFiveRoadTemplates.empty()){
		RoadTemplate rt = itsTopFiveRoadTemplates.top(); itsTopFiveRoadTemplates.pop();

		int index = rt.getIndex();
		float diff = rt.getDiff();
		std::vector<Point2D<int> > cPoints = itsCSpoints[index].first;
		Rectangle                  sRect   = itsCSpoints[index].second;

		itsMiddlePoint = itsRoadTemplates[index].getMiddlePoint();
		//itsVanishingPoint = itsRoadTemplates[index].getVanishingPoint();

		LINFO("%d diff: %f", ct,diff);


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
		drawFilledRect(dImaCS, sRect*GRID_SIZE, PixRGB<byte>(0, byte(bVal),0));
		for(uint j = 0; j < cPoints.size(); j++)
			drawFilledRect(dImaCS, Rectangle(cPoints[j],Dims(1,1))*GRID_SIZE, 
					PixRGB<byte>(byte(bVal),0,0));

		Image<PixRGB<byte> > tdIma(dIma+dImaCS);
		inplacePaste (itsDispImage, tdIma, Point2D<int>(width*(ct%4),height*(ct/4)));
		ct++;
	}
        //itsWin->drawImage(itsDispImage,0,0);
        //Raster::waitForKey();

  //Cut down the template by using last vp
  LINFO("===========>CS size %d",(int)itsCSpoints.size());
  if(itsVanishingPoint == Point2D<int>(0,0))
    {
      itsVanishingPoint = itsRoadTemplates[maxIdx].getVanishingPoint();
      //computeVpCSRoadTemplates(itsVanishingPoint);
      //LINFO("Redo Template==========>New CS size %d",(int)itsCSpoints.size());
    }

  LINFO("time: %f", tim.get()/1000.0F);
}

// ######################################################################
Histogram 
CenterSurroundHistogramRoadRecognition::getGridHistogramDistribution
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
Histogram CenterSurroundHistogramRoadRecognition::smoothAndNormalize
(Histogram h, int numPoints)
{
  // maybe later want to add the weight for each channel
  //(Point2D<int> pt, std::vector<float> &dist, float weight)

  uint sindex = 0; uint eindex = NUM_L_BINS - 1;
  Histogram lHist(h, sindex, eindex);
  sindex = eindex + 1; eindex += NUM_A_BINS;
  Histogram aHist(h, sindex, eindex);
  sindex = eindex + 1; eindex += NUM_B_BINS;
  Histogram bHist(h, sindex, eindex);
  
  // smooth the histogram individually
  lHist.smooth(itsLKernel);
  aHist.smooth(itsAKernel);
  bHist.smooth(itsBKernel);

  // combine the histograms
  std::vector<Histogram> histograms;
  histograms.push_back(lHist);
  histograms.push_back(aHist);
  histograms.push_back(bHist);
  //Histogram histogram.reset(new Histogram(histograms));
  Histogram rh(histograms);
  
  // normalize with the number of points
  rh.divide(numPoints);

  return rh;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

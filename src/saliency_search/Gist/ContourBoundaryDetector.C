/*!@file Gist/ContourBoundaryDetector.C Detect meaningful contours by
   weighting longer countour chains more */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Navigation/FOE_Navigation/MiddleTemporal.C
// $ $Id: $
//
//////////////////////////////////////////////////////////////////////////

#include "Gist/ContourBoundaryDetector.H"
#include "Image/ColorOps.H"
#include "Image/MathOps.H"
#include "Image/DrawOps.H"
#include  <cstdio>

// ######################################################################
// ######################################################################

// ######################################################################
ContourBoundaryDetector::ContourBoundaryDetector()
{
  itsWin.reset();
}

// ######################################################################
ContourBoundaryDetector::~ContourBoundaryDetector()
{ }

// ######################################################################
void ContourBoundaryDetector::computeContourBoundary
(Image<PixRGB<byte> > ima, int r)
{
  itsImage = ima;
  if(r == -1)
    itsRadius = DEFAULT_NEIGHBORHOOD_RAD;
  else
    itsRadius = r;

  // execute each step
  computeVarianceRidgeBoundaryMap();
  computeNmsBoundaryMap();
  computeContourBoundaryEdgels();
  computeContourBoundaryMap();
}

// ######################################################################
void ContourBoundaryDetector::computeVarianceRidgeBoundaryMap()
{
  // get the CIELab variance of the input image
  Image<float> varImg   = getLabStandardDeviation(itsImage, itsRadius);

  // get the gradient of the variance of the image
  std::vector<Image<float> > gradImg  = calculateGradient(varImg, itsRadius);

  // get the ridges in the gradient of the variance image
  Image<float> ridgeImg = getRidge(gradImg, itsRadius);

  // substract the gradient from the ridge image
  itsBoundaryImage  = substractGradImg(ridgeImg, gradImg);
}

// ######################################################################
void ContourBoundaryDetector::computeNmsBoundaryMap()
{
  // run non-max suppression to thin out image
  itsBoundaryImageNMS = getNonMaxSuppression(itsBoundaryImage);
  //bImgNMS = getNonMaxSuppression(bImgNMS);
}

// ######################################################################
void ContourBoundaryDetector::computeContourBoundaryEdgels()
{
  // compute the contour edgels
  itsEdgelBoundaryImage = getContourBoundaryEdgels();
}

// ######################################################################
void ContourBoundaryDetector::computeContourBoundaryMap()
{
  // connect the contour edgels
  connectContourEdgels();

  itsContourBoundaryImage = getContourBoundaryImage();

  //displayContourBoundary();
}

// ######################################################################
Image<float> ContourBoundaryDetector::getVarianceRidgeBoundaryMap
(Image<PixRGB<byte> > ima, int r)
{
  itsImage = ima;
  if(r == -1)
    itsRadius = DEFAULT_NEIGHBORHOOD_RAD;
  else
    itsRadius = r;

  computeVarianceRidgeBoundaryMap();

  return itsBoundaryImage;
}

// ######################################################################
Image<float> ContourBoundaryDetector::getVarianceRidgeBoundaryMap()
{
  return itsBoundaryImage;
}

// ######################################################################
Image<float> ContourBoundaryDetector::getNmsBoundaryMap
(Image<PixRGB<byte> > ima, int r)
{
  // assume that we need to recompute everything
  itsImage = ima;
  if(r == -1)
    itsRadius = DEFAULT_NEIGHBORHOOD_RAD;
  else
    itsRadius = r;

  computeVarianceRidgeBoundaryMap();
  computeNmsBoundaryMap();

  return itsBoundaryImageNMS;
}

// ######################################################################
Image<float> ContourBoundaryDetector::getNmsBoundaryMap()
{
  return itsBoundaryImageNMS;
}

// ######################################################################
Image<float> ContourBoundaryDetector::getEdgelBoundaryMap
(Image<PixRGB<byte> > ima, int r)
{
  // assume that we need to recompute everything
  itsImage = ima;
  if(r == -1)
    itsRadius = DEFAULT_NEIGHBORHOOD_RAD;
  else
    itsRadius = r;

  computeVarianceRidgeBoundaryMap();
  computeNmsBoundaryMap();
  computeContourBoundaryEdgels();

  return itsEdgelBoundaryImage;
}

// ######################################################################
Image<float> ContourBoundaryDetector::getEdgelBoundaryMap()
{
  return itsEdgelBoundaryImage;
}

// ######################################################################
Image<PixRGB<byte> > ContourBoundaryDetector::getContourBoundaryMap
(Image<PixRGB<byte> > ima, int r)
{
  // assume that we need to recompute everything
  itsImage = ima;
  if(r == -1)
    itsRadius = DEFAULT_NEIGHBORHOOD_RAD;
  else
    itsRadius = r;

  computeVarianceRidgeBoundaryMap();
  computeNmsBoundaryMap();
  computeContourBoundaryEdgels();
  computeContourBoundaryMap();

  return itsContourBoundaryImage;
}

// ######################################################################
Image<PixRGB<byte> > ContourBoundaryDetector::getContourBoundaryMap()
{
  return itsContourBoundaryImage;
}

// ######################################################################
Image<float> ContourBoundaryDetector::getLabStandardDeviation
(Image<PixRGB<byte> > ima, int r)
{
  // convert to CIElab color space
  Image<float> lImg; 
  Image<float> aImg;
  Image<float> bImg;
  getLAB(ima, lImg, aImg, bImg);

  // compute squares of each channel
  Image<float> lSqImg = lImg * lImg; 
  Image<float> aSqImg = aImg * aImg;
  Image<float> bSqImg = bImg * bImg;

  // compute box blur for each channel
  Image<float> lBbImg = boxBlur(lImg, r);
  Image<float> aBbImg = boxBlur(aImg, r); 
  Image<float> bBbImg = boxBlur(bImg, r); 

  Image<float> lSqBbImg = boxBlur(lSqImg, r);
  Image<float> aSqBbImg = boxBlur(aSqImg, r); 
  Image<float> bSqBbImg = boxBlur(bSqImg, r); 

  //calculate standard deviation of each l a b channel
  Image<float> vl = lBbImg*lBbImg - lSqBbImg;
  Image<float> va = aBbImg*aBbImg - aSqBbImg;
  Image<float> vb = bBbImg*bBbImg - bSqBbImg;
  Image<float> varImg = sqCombine(vl, va, vb);

  return varImg;
}

// ######################################################################
Image<float> ContourBoundaryDetector::boxBlur(Image<float> img, int rad)
{
  int w = img.getWidth();
  int h = img.getHeight();

  Image<float> ret(w,h,NO_INIT);
  
  int r = rad;

  // go through each location
  for(int i = 0; i < w; i++)
    {
      for(int j = 0; j < h; j++)
	{
	  float sum   = 0.0;
	  uint  count = 0; 

	  for(int di = -r; di <= r; di++)
	    {
	      for(int dj = -r; dj <= r; dj++)
		{
		  int ii = i + di;
		  int jj = j + dj;
                  
                  // pixel mirroring at the image borders
		  if(ii <  0) ii = -ii;
		  if(jj <  0) jj = -jj;
		  if(ii >= w) ii = 2*w - 2 - ii;
		  if(jj >= h) jj = 2*h - 2 - jj;

                  sum += img.getVal(ii,jj);
                  count++;
		}
	    }
	  ret.setVal(i,j, sum/count);
	}
    }

  return ret;
}

// ######################################################################
Image<float> ContourBoundaryDetector::sqCombine
(Image<float> a, Image<float> b, Image<float> c)
{
  ASSERT(a.isSameSize(b));
  ASSERT(b.isSameSize(c));

  int w = a.getWidth();
  int h = a.getHeight();
  Image<float> ret(w,h,NO_INIT);

  Image<float>::const_iterator aitr = a.begin(), stop = a.end();
  Image<float>::const_iterator bitr = b.begin();
  Image<float>::const_iterator citr = c.begin();
  Image<float>::iterator ritr = ret.beginw();

  while (aitr != stop)
    {
      float av = *aitr++;
      float bv = *bitr++;
      float cv = *citr++;

      *ritr++ = pow(av*av + bv*bv + cv*cv, 0.5); 	
    }
 
  return ret;
}

// ######################################################################
std::vector<Image<float> > ContourBoundaryDetector::calculateGradient
(Image<float> varImg, int r)
{  
  int w = varImg.getWidth();
  int h = varImg.getHeight();

  Image<float> gradImgX(w,h,NO_INIT);
  Image<float> gradImgY(w,h,NO_INIT);

  // smooth the variance Image using BoxBlur
  Image<float> varBbImg = boxBlur(varImg, r);

  std::vector<float> dx(NUM_GRADIENT_DIRECTIONS);
  std::vector<float> dy(NUM_GRADIENT_DIRECTIONS);    
  for(uint k = 0; k < NUM_GRADIENT_DIRECTIONS; k++)
    {
      dx[k] = cos(k*2*M_PI/NUM_GRADIENT_DIRECTIONS); 
      dy[k] = sin(k*2*M_PI/NUM_GRADIENT_DIRECTIONS);
      LDEBUG("%d %f %f", k, dx[k], dy[k]);
    }

  // go through each location
  for(int i = 0; i < w; i++)
    {
      for(int j = 0; j < h; j++)
	{
	  float sumX = 0.0;
	  float sumY = 0.0;
	  for(uint k = 0; k < NUM_GRADIENT_DIRECTIONS; k++)
	    {
	      int i1 = i + r*dx[k];
	      int j1 = j + r*dy[k];

	      int i2 = i - r*dx[k];		
	      int j2 = j - r*dy[k];
              
              // pixel mirroring at the image borders
	      if(i1 <  0) i1 = -i1;
	      if(j1 <  0) j1 = -j1;
	      if(i1 >= w) i1 = 2*w - 2 - i1;
	      if(j1 >= h) j1 = 2*h - 2 - j1;
	      
	      if(i2 <  0) i2 = -i2;
	      if(j2 <  0) j2 = -j2;
	      if(i2 >= w) i2 = 2*w - 2 - i2;
	      if(j2 >= h) j2 = 2*h - 2 - j2;
	      
	      float val = varBbImg.getVal(i1,j1) - varBbImg.getVal(i2,j2);

	      sumX +=  val * dx[k];
	      sumY +=  val * dy[k]; 

	      // LINFO("%d %d %d| val: %f, (%f %f) %f %f", 
	      // 	    i,j,k, val, val*dx[k], val*dy[k], sumX, sumY);
	    }

	  gradImgX.setVal(i,j, sumX);
	  gradImgY.setVal(i,j, sumY);
	}
    }

  std::vector<Image<float> > gradImg(2);
  gradImg[0] = gradImgX;
  gradImg[1] = gradImgY;

  return gradImg;
}

// ######################################################################
Image<float> ContourBoundaryDetector::getRidge
(std::vector<Image<float> > gradImg, int r)
{
  Image<float> gradImgX = gradImg[0];
  Image<float> gradImgY = gradImg[1];

  int w = gradImg[0].getWidth();
  int h = gradImg[0].getHeight();
  Image<float> ridgeImg(w,h,NO_INIT);

  std::vector<float> dx(NUM_GRADIENT_DIRECTIONS);
  std::vector<float> dy(NUM_GRADIENT_DIRECTIONS);    
  for(uint k = 0; k < NUM_GRADIENT_DIRECTIONS; k++)
    {
      dx[k] = cos(k*2*M_PI/NUM_GRADIENT_DIRECTIONS); 
      dy[k] = sin(k*2*M_PI/NUM_GRADIENT_DIRECTIONS);
    }

  // threshold the gradient image
  std::vector<std::vector<Image<float> > > dVin(NUM_GRADIENT_DIRECTIONS); 
  for(uint k = 0; k < NUM_GRADIENT_DIRECTIONS; k++)
    dVin[k] = std::vector<Image<float> >(2);

  for(uint k = 0; k < NUM_GRADIENT_DIRECTIONS; k++)
    {
      dVin[k][0] = Image<float>(w,h,NO_INIT);
      dVin[k][1] = Image<float>(w,h,NO_INIT);

      for(int i = 0; i < w; i++)
	{
	  for(int j = 0; j < h; j++)
	    {
	      float x = 0.0;
	      float y = 0.0;

	      int ii = int(i + r*dx[k]);
	      int jj = int(j + r*dy[k]); 

	      if(ii <  0) ii = -ii;
	      if(jj <  0) jj = -jj;
	      if(ii >= w) ii = 2*w - 2 - ii;
	      if(jj >= h) jj = 2*h - 2 - jj;
	      
	      float vX = gradImgX.getVal(ii,jj); 
	      float vY = gradImgY.getVal(ii,jj);
	      if((vX*dx[k] + vY*dy[k]) < 0.0)
		{
		  x = vX;
		  y = vY;
		}
	      
	      dVin[k][0].setVal(i,j, x);
	      dVin[k][1].setVal(i,j, y);	      
	    }
	}
    }
  itsDVin = dVin;

  std::vector<Image<float> > rDir (NUM_GRADIENT_DIRECTIONS); 
  Image<float> rDirIndex(w,h,NO_INIT);

  // calculate the geometric and arithmetic ridge direction
  // and sum the two together
  for(uint k = 0; k < NUM_GRADIENT_DIRECTIONS; k++)
    {
      rDir[k] = Image<float>(w,h,NO_INIT);

      for(int i = 0; i < w; i++)
	{
	  for(int j = 0; j < h; j++)
	    {
	      float x1 = dVin[k][0].getVal(i,j);
	      float y1 = dVin[k][1].getVal(i,j);

	      uint k2 = k + NUM_RIDGE_DIRECTIONS;
	      if(k >= NUM_RIDGE_DIRECTIONS) 
		k2 = k - NUM_RIDGE_DIRECTIONS/2;

	      float x2 = dVin[k2][0].getVal(i,j);
	      float y2 = dVin[k2][1].getVal(i,j);

	      float gVal = -(x1*x2 + y1*y2);	     	      
	      if(gVal < 0.0) 
		gVal = 0.0;
	      else 
		gVal = pow(gVal, 0.5);

	      // float ax = x2 - x1;
	      // float ay = y2 - y1;
	      // float aVal =  pow(ax*ax + ay*ay, 0.5)/ 2.0;	      
	      // rDir[k].setVal(i,j, gVal + aVal);

	      rDir[k].setVal(i,j, gVal);
	    }
	}
    }

  itsRidgeDirection = rDir;

  std::vector<Image<float> > rDirMax(NUM_RIDGE_DIRECTIONS); 
  for(uint k = 0; k < NUM_RIDGE_DIRECTIONS; k++)    
    rDirMax[k] = Image<float>(w,h,ZEROS);

  // get the maximum among the directions
  for(int i = 0; i < w; i++)
    {
      for(int j = 0; j < h; j++)
	{
	  float max = 0.0; int maxk = -1;
	  for(uint k = 0; k < NUM_RIDGE_DIRECTIONS; k++)
	    {
	      float val = rDir[k].getVal(i,j);
	      if(val > max) { max = val; maxk = k; }
	    }
	  ridgeImg.setVal(i,j, max);
	  rDirIndex.setVal(i,j, maxk);
	  if(maxk != -1)
	    rDirMax[maxk].setVal(i,j, max);
	}
    }

  itsRidgeDirectionIndex = rDirIndex;
  itsRidgeDirectionMax   = rDirMax;

  return ridgeImg;
}

// ######################################################################
Image<float> ContourBoundaryDetector::substractGradImg
(Image<float> ridgeImg, std::vector<Image<float> >  gradImg)
{
  Image<float> gradImgX = gradImg[0];
  Image<float> gradImgY = gradImg[1];

  Image<float> res = 
    //clampedDiff(ridgeImg, sqrt(gradImgX*gradImgX + gradImgY*gradImgY));
    ridgeImg - sqrt(gradImgX*gradImgX + gradImgY*gradImgY);

  return res;
}

// ######################################################################
Image<float> ContourBoundaryDetector::getNonMaxSuppression
(Image<float> bImg)
{
  int w = bImg.getWidth();
  int h = bImg.getHeight();
  Image<float> bImgNMS(w,h,ZEROS);

  // set up kernel for non-max suppression
  int wSize = BOUNDARY_STEP_SIZE+1;
  std::vector<std::vector<Point2D<int> > > sCoordsL(NUM_RIDGE_DIRECTIONS);
  std::vector<std::vector<Point2D<int> > > sCoordsR(NUM_RIDGE_DIRECTIONS);
  std::vector<std::vector<Point2D<int> > > cCoords (NUM_RIDGE_DIRECTIONS);


  for(uint k = 0; k < NUM_RIDGE_DIRECTIONS; k++)
    {
      cCoords [k] = std::vector<Point2D<int> >();  
      sCoordsL[k] = std::vector<Point2D<int> >();  
      sCoordsR[k] = std::vector<Point2D<int> >();  
    }

  // Non-max suppressed values for each direction
  itsRidgeDirectionNMS.clear();
  for(uint k = 0; k < NUM_RIDGE_DIRECTIONS; k++)
    itsRidgeDirectionNMS.push_back( Image<float>(w,h,ZEROS));  

  for(int i = -wSize/2; i <= wSize/2; i++)
    {
      for(int j = -wSize/2; j <= wSize/2; j++)
	{
	  if( i == 0) cCoords [0].push_back(Point2D<int>(i,j));
	  if( i <  0) sCoordsL[0].push_back(Point2D<int>(i,j));
	  if( i >  0) sCoordsR[0].push_back(Point2D<int>(i,j));

	  if(-j == i) cCoords [1].push_back(Point2D<int>(i,j));
	  if(-j >  i) sCoordsL[1].push_back(Point2D<int>(i,j));
	  if( i > -j) sCoordsR[1].push_back(Point2D<int>(i,j));	  

	  if( j == 0) cCoords [2].push_back(Point2D<int>(i,j));
	  if( j <  0) sCoordsL[2].push_back(Point2D<int>(i,j));
	  if( j >  0) sCoordsR[2].push_back(Point2D<int>(i,j));
	  
	  if( i == j) cCoords [3].push_back(Point2D<int>(i,j));
	  if( j >  i) sCoordsL[3].push_back(Point2D<int>(i,j));
	  if( i >  j) sCoordsR[3].push_back(Point2D<int>(i,j));
	}
    }

  // go through each point
  for(int i = 0; i < w; i++)
    {
      for(int j = 0; j < h; j++)
	{
	  // get the value
	  float val = bImg.getVal(i,j);
	  Point2D<int> cpt(i,j);
	  
	  for(uint k = 0; k < NUM_RIDGE_DIRECTIONS; k++)
	    {
	      float totalC = 0.0; uint ctC = 0;
	      for(uint cc = 0; cc < cCoords[k].size(); cc++)
		{
		  Point2D<int> pt = cCoords[k][cc] + cpt;  
		  if(bImg.coordsOk(pt)) 
		    {
		      totalC += bImg.getVal(pt);
		      ctC++;
		    }
		}
	      
	      float totalL = 0.0; uint ctL = 0;
	      for(uint cl = 0; cl < sCoordsL[k].size(); cl++)
		{
		  Point2D<int> pt = sCoordsL[k][cl] + cpt;  
		  if(bImg.coordsOk(pt)) 
		    {
		      totalL += bImg.getVal(pt); ctL++;
		    }
		}
	      
	      float totalR = 0.0; uint ctR = 0;
	      for(uint cr = 0; cr < sCoordsR[k].size(); cr++)
		{
		  Point2D<int> pt = sCoordsR[k][cr] + cpt;  
		  if(bImg.coordsOk(pt)) 
		    {
		      totalR += bImg.getVal(pt); ctR++;
		    }
		}
	      
	      if(totalC/ctC > totalR/ctR && totalC/ctC > totalL/ctL 
		 && val > 0.0)  
		{
		  bImgNMS.setVal(i,j, val);

		  itsRidgeDirectionNMS[k].setVal
		    (i,j, totalC/ctC*2 - totalR/ctR - totalL/ctL);
		}
	    }
	}
    }

  return bImgNMS;
}

// ######################################################################
Image<float> ContourBoundaryDetector::getContourBoundaryEdgels()
{
  // NOTE: FIXXXX: TENSOR-VOTING IS PROBABLY A BETTER IDEA

  int w = itsImage.getWidth();
  int h = itsImage.getHeight();
  Image<float> edgelBoundaryImage(w,h,ZEROS);

  int step  = BOUNDARY_STEP_SIZE;
  int hstep = step/2;
  //int wSize = BOUNDARY_STEP_SIZE+1;

  // set up the center and surround opponency locations
  std::vector<std::vector<Point2D<int> > > cCoords(NUM_RIDGE_DIRECTIONS);
  std::vector<std::vector<Point2D<int> > > sCoords(NUM_RIDGE_DIRECTIONS);
  std::vector<float> angles;

  for(uint k = 0; k < NUM_RIDGE_DIRECTIONS; k++)
    {
      cCoords[k] = std::vector<Point2D<int> >();  
      sCoords[k] = std::vector<Point2D<int> >();  

      angles.push_back(k*2.0*M_PI/float(NUM_RIDGE_DIRECTIONS));
    }

  // fill the center coordinates
  for(int i = -step/2; i < step/2; i++)
    {
      cCoords[0].push_back(Point2D<int>( i, 0));
      cCoords[1].push_back(Point2D<int>( i, i));
      cCoords[2].push_back(Point2D<int>( 0, i));
      cCoords[3].push_back(Point2D<int>( i,-i));
    }

  // fill the surround coordinates (bottom or right)
  for(int i = 0; i < hstep; i++)
    {
      sCoords[0].push_back(Point2D<int>( i+hstep,       0));
      sCoords[0].push_back(Point2D<int>( i+hstep,   hstep));
      sCoords[0].push_back(Point2D<int>( i+hstep,  -hstep));

      sCoords[1].push_back(Point2D<int>( i+hstep, i+hstep));
      sCoords[1].push_back(Point2D<int>( i+step , i      ));
      sCoords[1].push_back(Point2D<int>( i      ,-i-step ));

      sCoords[2].push_back(Point2D<int>(       0, i+hstep));
      sCoords[2].push_back(Point2D<int>(   hstep, i+hstep));
      sCoords[2].push_back(Point2D<int>(  -hstep, i+hstep));

      sCoords[3].push_back(Point2D<int>( i+hstep,-i-hstep));
      sCoords[3].push_back(Point2D<int>( i      ,-i-step ));
      sCoords[3].push_back(Point2D<int>( i+step ,-i      ));
    }

  // fill the surround coordinates (top or left)
  for(int i = -hstep; i < 0; i++)
    {
      sCoords[0].push_back(Point2D<int>( i-hstep,       0));
      sCoords[0].push_back(Point2D<int>( i-hstep,   hstep));
      sCoords[0].push_back(Point2D<int>( i-hstep,  -hstep));

      sCoords[1].push_back(Point2D<int>( i-hstep, i-hstep));
      sCoords[1].push_back(Point2D<int>( i-step , i      ));
      sCoords[1].push_back(Point2D<int>( i      , i-step ));

      sCoords[2].push_back(Point2D<int>(       0, i-hstep));
      sCoords[2].push_back(Point2D<int>(   hstep, i-hstep));
      sCoords[2].push_back(Point2D<int>(  -hstep, i-hstep));

      sCoords[3].push_back(Point2D<int>( i-hstep,-i+hstep));
      sCoords[3].push_back(Point2D<int>( i      ,-i+step ));
      sCoords[3].push_back(Point2D<int>( i-step ,-i      ));
    }

  // reset the edgel storage
  // NOTE: we will keep edgel at index 0 empty
  int wEdgel = (w+hstep)/step; 
  int hEdgel = (h+hstep)/step;
  itsCompleteEdgels = 
    Image<std::vector<rutz::shared_ptr<Edgel> > >(wEdgel, hEdgel, ZEROS);
  itsEdgels = Image<rutz::shared_ptr<Edgel> >(wEdgel, hEdgel, ZEROS);

  // go through each point
  // with the specified step size
  int wLimit = (w/step)*step;
  int hLimit = (h/step)*step;
  for(int j = step; j < hLimit; j+= step)
    {
      for(int i = step; i < wLimit; i+= step)
        {
	  Point2D<int> cpt(i,j);
	  
	  int maxk = -1;
	  Point2D<int> maxPt(-1,-1);
	  float maxVal = -1.0F;

          uint iEdgel = i/step;
          uint jEdgel = j/step;

	  // for each direction
	  for(uint k = 0; k < NUM_RIDGE_DIRECTIONS; k++)
	    {
	      Point2D<int> maxCKpt(-1,-1);
	      float maxCKval = 0.0;

	      // get maximum contour value for the center size
	      // to make the contour detector phase invariant
	      for(uint ci = 0; ci < cCoords[k].size(); ci++)
		{
		  Point2D<int> pt = cCoords[k][ci] + cpt;  
		  if(edgelBoundaryImage.coordsOk(pt)) 
		    {
		      float val = itsRidgeDirectionNMS[k].getVal(pt); 
		      if(maxCKval < val) 
			{
			  maxCKval = val; maxCKpt = pt;
			}
		    }
		}

	      float maxSKval = 0.0;

	      // get the maximum value for the surround 
	      for(uint si = 0; si < sCoords[k].size(); si++)
		{
		  Point2D<int> pt = sCoords[k][si] + cpt;  
		  if(edgelBoundaryImage.coordsOk(pt)) 
		    {
		      float val = itsRidgeDirectionNMS[k].getVal(pt); 
		      if(maxSKval < val) maxSKval = val;
		    }
		}
	      
	      // if center > 0 and wins
	      if(maxCKval > 0.0F && maxCKval > maxSKval)
		{
		  if(maxCKval > maxVal) 
		    {
		      maxPt  = maxCKpt;
		      maxVal = maxCKval;
		      maxk   = k;
		    }
                  
                  // put the new edgel in the right position
                  rutz::shared_ptr<Edgel> 
                    edgel(new Edgel(maxCKpt, angles[k], k, maxCKval)); 
                  
                  std::vector<rutz::shared_ptr<Edgel> >
                    cEdgelList = itsCompleteEdgels.getVal(iEdgel,jEdgel);

                  uint eNum = cEdgelList.size();
                  uint place = 0;
                  for(uint ce = 0; ce < eNum; ce++)
                    {
                      if(cEdgelList[ce]->val < maxCKval)
                        {
                          place = ce; ce = eNum;
                        }
                      else place = ce+1;
                    }
                  //LINFO("place: %d  | eNum: %d", place, eNum);
                  
                  cEdgelList.push_back(edgel);
                  
                  // for(uint ce = 0; ce < cEdgelList.size(); ce++)
                  //   LINFO("%12.5f %3d", cEdgelList[ce]->val,  
                  //         cEdgelList[ce]->angleIndex);

                  if(place != eNum)
                    {
                      // LINFO("move one");
                  
                      for(int ce = int(eNum-1); ce >= int(place); ce--)
                        {
                          cEdgelList[ce+1] = cEdgelList[ce];
                        }
                      
                      // for(uint ce = 0; ce < cEdgelList.size(); ce++)
                      //   LINFO("%12.5f %3d", cEdgelList[ce]->val,  
                      //         cEdgelList[ce]->angleIndex);
                      
                      cEdgelList[place] = edgel;                  
                      
                      // LINFO("place the new one properly");

                      // for(uint ce = 0; ce < cEdgelList.size(); ce++)
                      //   LINFO("%12.5f %3d", cEdgelList[ce]->val,  
                      //         cEdgelList[ce]->angleIndex);
                    }
                  // else LINFO("last place");

                  itsCompleteEdgels.setVal(iEdgel,jEdgel, cEdgelList);


                  // LINFO("%d %d: size: %d ", i,j, 
                  //       int(itsCompleteEdgels.getVal(iEdgel,jEdgel).size()));

                  // for(uint ce = 0; ce < cEdgelList.size(); ce++)
                  //   LINFO("%12.5f %3d", cEdgelList[ce]->val,  
                  //         cEdgelList[ce]->angleIndex);
		}
	    }

	  // if there is a winner
	  if(maxk != -1)
	    {
              itsEdgels.setVal
                (iEdgel,jEdgel, itsCompleteEdgels.getVal(iEdgel,jEdgel)[0]);

	      float borderK = 
		fmod((maxk+(NUM_RIDGE_DIRECTIONS/2)),NUM_RIDGE_DIRECTIONS);

	      float dx = cos(borderK * M_PI/4.0) * hstep; 
	      float dy = sin(borderK * M_PI/4.0) * hstep;

	      Point2D<int> pt = maxPt;
	      Point2D<int> p1 = pt + Point2D<int>( dx+.5,  dy+.5); 
	      Point2D<int> p2 = pt + Point2D<int>(-dx-.5, -dy-.5);

              //uint iEdgel = i/step;
              //uint jEdgel = j/step;
              //if(iEdgel >= 10 && iEdgel <= 25 && jEdgel >= 1 && jEdgel <= 14)
              //   {
              //     LINFO("maxk: %d -> %d  -> %f  %f (%f %f %f %f)  |%f %f", 
              //           maxk, (maxk+(NUM_RIDGE_DIRECTIONS/2)), 
              //           (borderK*M_PI)/float(NUM_RIDGE_DIRECTIONS), borderK, 
              //           cos(0), cos(M_PI/4.0), cos(M_PI/2.0), cos(M_PI*.75),
              //           dx, dy);
	      


              //LINFO("%d %d | %d %d | %d %d::::: %d %d  %d", 
              //         pt.i, pt.j, p1.i, p1.j, p2.i, p2.j, iEdgel, jEdgel, maxk);
                  
              // draw the straightline contour in the image 
              // for visualization
              drawLine(edgelBoundaryImage, p1, p2, 255.0F);
              //drawDisk(edgelBoundaryImage, pt, 2,  255.0F);
              // }      
              
	    }
	}
    }

  return edgelBoundaryImage;
}

// ######################################################################
void ContourBoundaryDetector::connectContourEdgels()
{
  int w = itsImage.getWidth();
  int h = itsImage.getHeight();

  // if contour map don't have a contour on the location
  // that means the edgel on that location 
  // do not belong to any contours yet

  // reset the contours storage
  // NOTE: we will keep edgel at index 0 empty
  int step   = BOUNDARY_STEP_SIZE;
  int wEdgel = w/step; 
  int hEdgel = h/step;
  itsContourMap = 
    Image<rutz::shared_ptr<Contour> >(wEdgel, hEdgel, ZEROS);
  itsContourBoundaries.clear();

  // for each edgel RF location: we move horizontally first
  for(int j = 0; j < hEdgel; j++)
    {
      for(int i = 0; i < wEdgel; i++)
        {
          std::vector<rutz::shared_ptr<Edgel> >
            cEdgelList = itsCompleteEdgels.getVal(i,j);
          uint eNum = cEdgelList.size();

          // skip if there is no edgel in the location
          if(eNum == 0) continue;

          // skip if the edgel already belong to a contours
          if(itsContourMap.getVal(i,j).is_valid()) continue;

          // start growing:
          rutz::shared_ptr<Edgel> edgel = cEdgelList[0];          
          rutz::shared_ptr<Contour> contour(new Contour());
          contour->edgels.push_back(edgel);
          itsContourBoundaries.push_back(contour);
          itsContourMap.setVal(i, j, contour);

          // FIXXX: ADD THE JUNCTION RECOGNITION AS WELL HERE

          //LINFO("edgel: %3d %3d: ang: %d", i,j, edgel->angleIndex);
          
          // while can add an edgel
          // to the right (and/or bottom) of the contour
          bool isAddingRight = true;
          int ci = i, cj = j;
          //LINFO("start: ci: %3d cj: %3d: cd: %3d sign: %d", 
          //      ci, cj, edgel->angleIndex, 1);
          while(isAddingRight)
            { 
              isAddingRight = 
                addEdgelToContour(ci, cj, edgel, contour, 1);
              if(isAddingRight) 
                itsContourMap.setVal(ci, cj, contour);
              //LINFO("[%3d %3d]: %3d %3d %3d", ci,cj, 
              //      edgel->pt.i, edgel->pt.j, edgel->angleIndex);
            }

          //LINFO("Final Size: %d \n", 
          //      int(itsContourMap.getVal(i,j)->edgels.size()));
          //Raster::waitForKey();

          // // while can add an edgel
          // // to the left (and/or top) 
          // bool isAddingLeft = true;
          // while(isAddingLeft)
          //   {
          //     isAddingLeft = addEdgelToContour(i,j, edgel, contour, -1);  
          //   }

        }
    }

  // go through the contours

  // only link with negative edgels if the two contours are 

  // report the contours from the most salient first
  //  using equation that includes both length of contours and magnitude

  // better graphical drawing 
  //  --> probably need the stdev gradient info


}

// ######################################################################
bool ContourBoundaryDetector::addEdgelToContour
(int &ci, int &cj, 
 rutz::shared_ptr<Edgel> &edgel, 
 rutz::shared_ptr<Contour> &contour,
 int sign)
{
  // direction of the current edgel
  int cd = edgel->angleIndex;

  // get the neighborhood of edgel 
  // depending on the direction
  int ni1=0, nj1=0, ni2=0, nj2=0, ni3=0, nj3=0;
  getEdgelDirectionNeighborhood
    (ci, cj, cd, sign, ni1, nj1, ni2, nj2, ni3, nj3);

  // LINFO("ni1: %3d nj1: %3d", ni1, nj1);
  // LINFO("ni2: %3d nj2: %3d", ni2, nj2);
  // LINFO("ni3: %3d nj3: %3d", ni3, nj3);

  // only examine if coordinate is within image
  // and  there is an edgel in the coordinate 
  uint bEdge1 = 0;  uint bEdge2 = 0;  uint bEdge3 = 0;

  if(itsCompleteEdgels.coordsOk(ni1, nj1) &&      
     itsCompleteEdgels.getVal(ni1, nj1).size() != 0) bEdge1 = 1;

  if(itsCompleteEdgels.coordsOk(ni2, nj2) &&      
     itsCompleteEdgels.getVal(ni2, nj2).size() != 0) bEdge2 = 1;

  if(itsCompleteEdgels.coordsOk(ni3, nj3) &&      
     itsCompleteEdgels.getVal(ni3, nj3).size() != 0) bEdge3 = 1;

  uint totalEdgels = bEdge1+bEdge2+bEdge3;

  //LINFO("  have %d neighbors: "
  //      "(%d %d <%d>) (%d %d <%d>) (%d %d <%d>)", totalEdgels,
  //      ni1, nj1, bEdge1, ni2, nj2, bEdge2, ni3, nj3, bEdge3);

  // if no edgels to continue, we're done 
  if(totalEdgels == 0) 
    return false;

  // if there is only 1 edgel to continue to 
  else if(totalEdgels == 1) 
    {
      // get the next edgel
      int ni = -1; int nj = -1; int nd = -1;
      if(bEdge1 == 1)      { ni = ni1; nj = nj1; }
      else if(bEdge2 == 1) { ni = ni2; nj = nj2; }
      else if(bEdge3 == 1) { ni = ni3; nj = nj3; }      
      
      std::vector<rutz::shared_ptr<Edgel> >
        nEdgelList = itsCompleteEdgels.getVal(ni,nj);          
      rutz::shared_ptr<Edgel> nEdgel = nEdgelList[0]; 

      // get the next direction 
      nd = nEdgel->angleIndex;

      // if unclaimed and legal direction to add
      bool legalDirection = 
        isLegalDirection(ci, cj, cd, sign, ni, nj, nd);
      if(itsContourMap.getVal(ni,nj).is_invalid() && 
         legalDirection)
        {      
          // add and advance the end of contour
          contour->edgels.push_back(nEdgel);
          edgel = nEdgel;
          ci = ni; cj = nj; cd = edgel->angleIndex;
          
          //LINFO("  --> [1]  adding ci: %3d cj: %3d: cd: %3d sign: %d", 
          //      ci, cj, cd, sign);

          // FIXXX: maybe there is a second go round 
          //        to connect even more
          //        so may not want to return right away
          return true;
        }
      else return false;
    }
  else if(totalEdgels == 2)
    {
      // FIXXX: have to check for junction example:
      //        where there are more than 1 valid direction 

      // priority on what??
      int n1i = -1, n1j = -1, n2i = -1, n2j = -1;
      if(bEdge1 == 0)
        {
          n1i = ni2; n1j = nj2; n2i = ni3; n2j = nj3;       
        }
      else if(bEdge2 == 0)
        {
          n1i = ni1; n1j = nj1; n2i = ni3; n2j = nj3;       
      
        }
      else if(bEdge3 == 0)
        {
          n1i = ni1; n1j = nj1; n2i = ni2; n2j = nj2;       
        }
      
      // get the first next edgel
      std::vector<rutz::shared_ptr<Edgel> >
        nEdgelList1 = itsCompleteEdgels.getVal(n1i,n1j);          
      rutz::shared_ptr<Edgel> nEdgel1 = nEdgelList1[0]; 
      //int n1d = nEdgel1->angleIndex;

      // get the second next edgel
      std::vector<rutz::shared_ptr<Edgel> >
        nEdgelList2 = itsCompleteEdgels.getVal(n2i,n2j);          
      rutz::shared_ptr<Edgel> nEdgel2 = nEdgelList2[0]; 
      //int n2d = nEdgel2->angleIndex;

      // if angle is: /
      if(cd == 1)
        {
          if(bEdge1 == 1)
          //if(n1d == 1 && n2d == 1)
          //if(bEdge1 == 1 && n2d == 1)       
            {
              // if edgel unclaimed
              if(itsContourMap.getVal(n2i,n2j).is_invalid())
                {      
                  // add and advance the end of contour
                  contour->edgels.push_back(nEdgel2);
                  edgel = nEdgel2;
                  ci = n2i; cj = n2j;

                  //LINFO("  --> [21a] adding ci: %3d cj: %3d: cd: %3d sign: %d", 
                  //      ci, cj, cd, sign);
                  // FIXXX: maybe there is a second go round 
                  //        to connect even more
                  //        so may not want to return right away
                  return true;
                }
              else
                { 
                  //LINFO("  CLAIMED"); 
                  return false; 
                }
            }
          else 
            {
              //LINFO("  --> [21a]NOT 1(%3d %3d %3d) 2(%3d %3d %3d)", 
              //      n1i, n1j, n1d, n2i, n2j, n2d);
              return false;
            }
        }


      if(cd == 3)
        {
          if(bEdge1 == 1)
          //if(n1d == 3 && n2d == 3)
          //if(bEdge1 == 1 && n2d == 3)
            {
              // if edgel unclaimed
              if(itsContourMap.getVal(n2i,n2j).is_invalid())
                {      
                  // add and advance the end of contour
                  contour->edgels.push_back(nEdgel2);
                  edgel = nEdgel2;
                  ci = n2i; cj = n2j; cd = edgel->angleIndex;

                  //LINFO("  --> [23b] adding ci: %3d cj: %3d: cd: %3d sign: %d", 
                  //      ci, cj, cd, sign);

                  // FIXXX: maybe there is a second go round 
                  //        to connect even more
                  //        so may not want to return right away
                  return true;
                }
              else
                { 
                  //LINFO("  CLAIMED"); 
                  return false; 
                }
            }
          else 
            {
              //LINFO("  --> [23b]NOT 1(%3d %3d %3d) 2(%3d %3d %3d)", 
              //      n1i, n1j, n1d, n2i, n2j, n2d);
              return false;
            }
        }

      if(cd == 0)
        {
          if(bEdge1 == 1)
          //if(n1d == 3 && n2d == 3)
          //if(bEdge1 == 1 && 
          //   ((n1d ==  1 && bEdge2 == 1) || (n1d ==  3 && bEdge3 == 1)))
            {
              // if edgel unclaimed
              if(itsContourMap.getVal(n1i,n1j).is_invalid())
                {      
                  // add and advance the end of contour
                  contour->edgels.push_back(nEdgel1);
                  edgel = nEdgel1;
                  ci = n2i; cj = n2j; cd = edgel->angleIndex;

                  //LINFO("  --> [20a] adding ci: %3d cj: %3d: cd: %3d sign: %d", 
                  //      ci, cj, cd, sign);

                  // FIXXX: maybe there is a second go round 
                  //        to connect even more
                  //        so may not want to return right away
                  return true;
                }
              else
                { 
                  //LINFO("  CLAIMED"); 
                  return false; 
                }
            }
          else 
            {
              // Branch: we stop the contour
              //LINFO("  Branch:0: junction");
              //LINFO("  --> [2_0]NOT 1(%3d %3d %3d) 2(%3d %3d %3d)", 
              //      n1i, n1j, n1d, n2i, n2j, n2d);
              return false;
            }
        }

      if(cd == 2)
        {
          if(bEdge1 == 1)
          //if(n1d == 3 && n2d == 3)
          //if(bEdge1 == 1 && 
          //   ((n1d ==  1 && bEdge2 == 1) || (n1d ==  3 && bEdge3 == 1)))
            {
              // if edgel unclaimed
              if(itsContourMap.getVal(n1i,n1j).is_invalid())
                {      
                  // add and advance the end of contour
                   contour->edgels.push_back(nEdgel1);
                  edgel = nEdgel1;
                  ci = n1i; cj = n1j; cd = edgel->angleIndex;
                  
                  //LINFO("  --> [22a] adding ci: %3d cj: %3d: cd: %3d sign: %d", 
                  //      ci, cj, cd, sign);
                  
                  // FIXXX: maybe there is a second go round 
                  //        to connect even more
                  //        so may not want to return right away
                  return true;
                }
              else
                { 
                  //LINFO("  CLAIMED"); 
                  return false; 
                }
            }          
          else 
            {
              // Branch: we stop
              //LINFO("  Branch:2: junction");
              //LINFO("  --> [2_2]NOT 1(%3d %3d %3d) 2(%3d %3d %3d)", 
              //      n1i, n1j, n1d, n2i, n2j, n2d);
              return false;
            }
        }

  // if we can still include with the rules 
  // (in this order):
  //   1 space lookahead
  //   2 space lookahead
  //  [3 space lookahead]???
  //    note 3 space lookahead means 
  //    a gap of 16 pixels to interpolate    
  //    --> probably an absolute max
  
  // in any of these lookahead 
  // only consider positive NMS difference
  
  
  // if there is branching: also split (it's a junction)
  
  // if it's a junction
  // stop a contour on the end
  // split if it hit the middle
  
  // split if the NMS difference is very not similar 
  // remember merging is easy - but spliting is hard
  
  // add the edgel to the contour
 
  //LINFO("ci: %3d cj: %3d", ci, cj);

      
      // for now return false
      return false;
    }
  else if(totalEdgels == 3)
    {
      // more than likely it's too ambiguous to decide what to do
      //LINFO("3 edgels. May want to take a look at it");
      //Raster::waitForKey();
      return false;
    }

  return false;
}

// ######################################################################
void ContourBoundaryDetector::getEdgelDirectionNeighborhood
(int ci, int cj, int dir, int sign, 
 int &ni1, int &nj1, int &ni2, int &nj2, int &ni3, int &nj3)
{
  // 0 degrees: | boundary 
  if(dir == 0)
    {
      ni1 = ci;      nj1 = cj + sign;
      ni2 = ci-sign; nj2 = cj + sign;
      ni3 = ci+sign; nj3 = cj + sign;
    }      
  // 45 degrees: / boundary
  else if(dir == 1)
    {
      ni1 = ci-sign; nj1 = cj + sign;
      ni2 = ci-sign; nj2 = cj;
      ni3 = ci;      nj3 = cj + sign;
    }

  // 90 degrees: -- boundary
  else if(dir == 2)
    {
      ni1 = ci+sign; nj1 = cj;
      ni2 = ci+sign; nj2 = cj - sign;
      ni3 = ci+sign; nj3 = cj + sign;
    }

  // 135 degrees: \ boundary
  else if(dir == 3)
    {
      ni1 = ci+sign; nj1 = cj + sign;
      ni2 = ci+sign; nj2 = cj;
      ni3 = ci;      nj3 = cj + sign;
    }
}

// ######################################################################
bool ContourBoundaryDetector::isLegalDirection
(int ci, int cj, int cd, int sign, int ni, int nj, int nd)
{
  // important that this operation symmetric!!!
  // that is: (a isLegalDirection b) == (b isLegalDirection a)

  int di = sign*(ni - ci);
  int dj = sign*(nj - cj);

  // LINFO("%d %d %d - %d - %d %d %d ", ci, cj, cd, sign, ni, nj, nd);
  // LINFO("di dj: %d %d", di, dj);

  // 0 degrees: | boundary:  
  if(cd == 0)
    {
      // loc n1
      if     (di ==  0 && dj ==  1)
        {
          return (nd != 2);
        }
      // loc n2
      else if(di == -1 && dj ==  1)
        {
          return (nd != 3);
        }
      // loc n3
      else if(di ==  1 && dj ==  1)
        {
          return (nd != 1);
        }
      else 
        {
          LFATAL("invalid dir. loc:"
                 "%d %d %d - %d - %d %d %d "
                 "di dj: %d %d",
                 ci, cj, cd, sign, ni, nj, nd, di, dj);
          return false;
        }
    }      
  // 45 degrees: / boundary
  else if(cd == 1)
    {
      // loc n1
      if     (di == -1 && dj ==  1)
        {
          return (nd != 3);
        }
      // loc n2
      else if(di == -1 && dj ==  0)
        {
          return (nd != 0);
        }
      // loc n3
      else if(di ==  0 && dj ==  1)
        {
          return (nd != 2);
        }
      else 
        {
          LFATAL("invalid dir. loc:"
                 "%d %d %d - %d - %d %d %d "
                 "di dj: %d %d",
                 ci, cj, cd, sign, ni, nj, nd, di, dj);
          return false;
        }
    }

  // 90 degrees: -- boundary
  else if(cd == 2)
    {
      // loc n1
      if     (di ==  1 && dj ==  0)
        {
          return (nd != 0);
        }
      // loc n2
      else if(di ==  1 && dj == -1)
        {
          return (nd != 3);
        }
      // loc n3
      else if(di ==  1 && dj ==  1)
        {
          return (nd != 1);
        }
      else 
        {
          LFATAL("invalid dir. loc:"
                 "%d %d %d - %d - %d %d %d "
                 "di dj: %d %d",
                 ci, cj, cd, sign, ni, nj, nd, di, dj);
          return false;
        }
    }

  // 135 degrees: \ boundary
  else if(cd == 3)
    {
      // loc n1
      if     (di ==  1 && dj ==  1)
        {
          return (nd != 1);
        }
      // loc n2
      else if(di ==  1 && dj ==  0)
        {
          return (nd != 0);
        }
      // loc n3
      else if(di ==  0 && dj ==  1)
        {
          return (nd != 2);
        }
      else 
        {
          LFATAL("invalid dir. loc:"
                 "%d %d %d - %d - %d %d %d "
                 "di dj: %d %d",
                 ci, cj, cd, sign, ni, nj, nd, di, dj);
          return false;
        }
    }
  else LFATAL("invalid direction");
  return false;
}

// ######################################################################
Image<PixRGB<byte> > ContourBoundaryDetector::getContourBoundaryImage()
{
  int w = itsImage.getWidth();
  int h = itsImage.getHeight();

  Image<PixRGB<byte> > image = itsImage;
  float mVal = 96;
  float bVal = 255 - mVal;
  Image<byte> dImaR, dImaG, dImaB;
  getComponents(image, dImaR, dImaG, dImaB);
  inplaceNormalize(dImaR, byte(0), byte(mVal));
  inplaceNormalize(dImaG, byte(0), byte(mVal));
  inplaceNormalize(dImaB, byte(0), byte(mVal));

  // Image<float> tBI = itsBoundaryImageNMS;
  // inplaceNormalize(tBI, 0.0f, bVal); 
  // Image<byte> tBIb(tBI);

  Image<PixRGB<byte> > dIma = makeRGB(dImaR,dImaG,dImaB);
  // Image<PixRGB<byte> > dIma = 
  //   makeRGB(Image<byte>(dImaR+tBIb),
  //           Image<byte>(dImaG+tBIb),
  //           Image<byte>(dImaB+tBIb) );
  //Image<PixRGB<byte> > dIma = makeRGB(tBIb, tBIb, tBIb);

  int hstep = BOUNDARY_STEP_SIZE/2;

  // clean image
  Image<byte> dCBmapR(w,h,ZEROS);
  Image<byte> dCBmapG(w,h,ZEROS);
  Image<byte> dCBmapB(w,h,ZEROS);

  for(uint i = 0; i < itsContourBoundaries.size(); i++)
    {
      rutz::shared_ptr<Contour> contour = itsContourBoundaries[i];

      if(contour->edgels.size() < 5) continue;

      byte rcVal, gcVal, bcVal;
      rcVal = bVal*rand()/(RAND_MAX+1.0);
      gcVal = bVal*rand()/(RAND_MAX+1.0);
      bcVal = bVal*rand()/(RAND_MAX+1.0);      
 
      for(uint j = 0; j < contour->edgels.size(); j++)
        {
          rutz::shared_ptr<Edgel> edgel = contour->edgels[j];
          
          uint aind = edgel->angleIndex; 
          float baind = 
            fmod((aind+(NUM_RIDGE_DIRECTIONS/2)),NUM_RIDGE_DIRECTIONS);
          
          float dx = cos(baind * M_PI/4.0) * hstep; 
          float dy = sin(baind * M_PI/4.0) * hstep;
          
          Point2D<int> pt = edgel->pt;
          Point2D<int> p1 = pt + Point2D<int>( dx+.5,  dy+.5); 
          Point2D<int> p2 = pt + Point2D<int>(-dx-.5, -dy-.5);
          
          //LINFO("%d %d | %d %d | %d %d::::: %d %d  %d", 
          //         pt.i, pt.j, p1.i, p1.j, p2.i, p2.j, iEdgel, jEdgel, maxk);	      

          // draw the edgel

          // draw the straightline contour in the image 
          // for visualization
          drawLine(dCBmapR, p1, p2, byte(rcVal));
          drawLine(dCBmapG, p1, p2, byte(gcVal));
          drawLine(dCBmapB, p1, p2, byte(bcVal));
          //drawDisk(itsContourBoundaryImage, pt, 2,  255.0F);
        }
    }

  Image<PixRGB<byte> > dCBmap = makeRGB(dCBmapR,dCBmapG,dCBmapB);
  return Image<PixRGB<byte> >(dIma+dCBmap);
}

// ######################################################################
void ContourBoundaryDetector::displayContourBoundary()
{
  int w = itsImage.getWidth();
  int h = itsImage.getHeight();

  if(itsWin.is_invalid())
    itsWin.reset(new XWinManaged(Dims(w,h), w+10, 0, "Contours"));
  else itsWin->setDims(Dims(w,h));

  // display the contour
  itsWin->drawImage(getContourBoundaryImage(),0,0);
  //Raster::waitForKey();
}

// ######################################################################
void ContourBoundaryDetector::displayGradImage
(std::vector<Image<float> > gradImg)
{
  Image<float> gradImgX = gradImg[0];
  Image<float> gradImgY = gradImg[1];

  uint w = gradImgX.getWidth();
  uint h = gradImgX.getHeight();

  Image<PixRGB<byte> > dGradImg(w, h, NO_INIT);

  float mn, mx;
  Image<float> temp = gradImgX*gradImgX + gradImgY*gradImgY;
  getMinMax(temp, mn,mx);
  float max = pow(mx,.5);

  for(uint i = 0; i < w; i++)
    {
      for(uint j = 0; j < h; j++)
	{
	  float x = gradImgX.getVal(i,j);
	  float y = gradImgY.getVal(i,j);
	    	    
	  float dir = atan2(y,x) * 180.0 / M_PI;
	  if(dir < 0.0) dir = 360.0 + dir;
	  float mag = pow(x*x + y*y, 0.5);

	  // PixHSV<byte> hsvp(byte(dir/360.0 * 255.0),
	  //  		    100,
	  //  		    byte(max/max* 255.0));	 	  

	  PixHSV<byte> hsvp(180,100,50);

	  //PixRGB<byte> rgbp(hsvp);
	  byte m = byte(mag/max * 255.0);
	  //byte d = byte(dir/360.0 * 255.0);
	  PixRGB<byte> rgbp(m, m, m);
	  dGradImg.setVal(i,j, rgbp);

	  // PixRGB<byte> a(0,255,255);
	  // PixHSV<byte> b(a);
	  // PixRGB<byte> c(b);
	  // LINFO("%d %d %d ==> %d %d %d ==> %d %d %d",		
	  // 	a.red(), a.green(), a.blue(),
	  // 	b.H(), b.S(), b.V(),
	  // 	c.red(), c.green(), c.blue());
	  // Raster::waitForKey();

	  // LINFO("%f %f || %f %f (%f): %d %d %d ==> %d %d %d", 
	  // 	x,y, dir, mag, mag/max* 255.0,
	  // 	hsvp.H(), hsvp.S(), hsvp.V(),
	  // 	rgbp.red(), rgbp.green(), rgbp.blue());
	}
    }

  itsWin->drawImage(dGradImg, 0,0);
  Raster::waitForKey(); 
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

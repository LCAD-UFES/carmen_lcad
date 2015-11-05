/*!@file Gist/VanishingPointDetector.C Detect vanishing point by
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Gist/VanishingPointDetector.C
// $ $Id: $
//
//////////////////////////////////////////////////////////////////////////

#include "Gist/VanishingPointDetector.H"
#include "Image/ColorOps.H"
#include "Image/MathOps.H"
#include "Image/DrawOps.H"
#include  <cstdio>
#define PI_UNDER_180 57.2957795
#define PI_OVER_180  0.0174532925

// ######################################################################
// ######################################################################
// ######################################################################
VanishingPointDetector::VanishingPointDetector()
{
  //  itsWin.reset();
  itsTrackedVp = Point2D<int>(0,0);
  itszNoise = Image<double> (4,4,ZEROS);

  // Initial noise matrix
  double vpVar=4.0;
  double roadWidthVar=5.0;
  //	double angVar = 5.0*M_PI/180;
  itszNoise.setVal(0,0,vpVar*vpVar);
  itszNoise.setVal(1,1,vpVar*vpVar);
  itszNoise.setVal(2,2,roadWidthVar*roadWidthVar);//left x
  itszNoise.setVal(3,3,roadWidthVar*roadWidthVar);//right x
  //	zNoise.setVal(4,4,roadWidthVar*roadWidthVar);//middle x
  //	zNoise.setVal(5,5,angVar*angVar);//angle of middle line
  itsFrameID = 0;
  itsProcessedFrameID = 0;
	itsRoadColor = PixRGB<byte>(255,255,255);
	itsMaskSize = 0;
}

// ######################################################################
VanishingPointDetector::VanishingPointDetector(Image<PixRGB<byte> >ima)
{
  itsImage = ima;
  itsTrackedVp = Point2D<int>(0,0);
	itsRoadColor = PixRGB<byte>(255,255,255);
  //  itsWin.reset();
}

// ######################################################################
void VanishingPointDetector::updateRoadColor(PixRGB<byte> roadColor)
{
	if(roadColor == PixRGB<byte>(0,0,0)) return; //don't update road, 
	itsRoadColor = roadColor;
	LINFO("road color is %d %d %d",roadColor.red(),roadColor.green(),roadColor.blue());
}
// ######################################################################
void VanishingPointDetector::updateImage(Image<PixRGB<byte> >ima)
{
  if(!ima.initialized()) return;

  LINFO("Image width %d height %d",ima.getWidth(),ima.getHeight());
  Timer timer(1000000); float time,total_time;
  timer.reset();
  time = timer.get()/1000.0F;
  total_time = timer.get()/1000.0F;

  itsImage = Image<PixRGB<byte> > (ima.getDims(),ZEROS);
  itsImage = ima;

  itsFrameID++; // don't increment itsProcessedFrameID until we compute all features

  itsVpdf.predictState();
  itsVpdf.predictObservation(itszNoise);
	
  itsVoteMap = Image<float>(ima.getDims(),NO_INIT);
  itsAngleIndexMap = Image<float>(ima.getDims(),NO_INIT);
  itsConfidenceMap = Image<float>(ima.getDims(),NO_INIT);

  // compute gabor filter output
  getOrientationSet(4,4);
  LINFO("Build Gabor Pyr time: %f", timer.get()/1000.0F - time);
  time = timer.get()/1000.0F;

  // compute orientation confidence map
  getConfidenceMap();
  LINFO("Compute Confidence Map time: %f", timer.get()/1000.0F - time);
  time = timer.get()/1000.0F;
  getIndexMap();			
  LINFO("Compute Index Map time: %f", timer.get()/1000.0F - time);
  time = timer.get()/1000.0F;
  //getVoteMap(0.3,0.35,2.0);

  // Vote for vantage point
	LINFO("Tracked Vp is %d,%d before getVoteMap",itsTrackedVp.i,itsTrackedVp.j);
  if(itsTrackedVp == Point2D<int>(0,0))
    {
      // if we don't have prior vp 
      // (usually in initial frame)
      // vote for full map	
      getVoteMap(0.3,0.35,2.0);
    }
  else
    { 
      // otherwise, detailed vote in region 
      // around previous estimate vantage point
      // and sparser voting for the rest of the image
      getVoteMap2(itsSearchMemoryMap,0.3,0.35,2.0);
    }
  itsTrackedVp = getVanishingPoint();
	LINFO("Tracked Vp is %d,%d after get vote map",itsTrackedVp.i,itsTrackedVp.j);
	LINFO(" after getVote map its Vp is %d,%d",itsVp.i,itsVp.j);


  // update temporal search map
  // on the next frame, only recalculate 
  // on these specified coordinates 
  if(!itsSearchMemoryMap.initialized())
    {
      itsSearchMemoryMap = Image<int>(itsImage.getDims(),ZEROS);
    }
  addMemoryMapRegion(itsSearchMemoryMap,itsTrackedVp,itsImage.getDims()/10);
  forgetMemoryMap(itsSearchMemoryMap);

  LINFO("Compute Vote Map time: %f", timer.get()/1000.0F - time);
  time = timer.get()/1000.0F;

  // TO DO: re-estimate vantage point
  findRoad();//update all left/right color/ocr point	

  LINFO("Find Road time: %f", timer.get()/1000.0F - time);
  LINFO(">>>>Total time: %f", timer.get()/1000.0F - total_time);
  time = timer.get()/1000.0F;
  //updateFilter();//just use GRVS * CD score

  itsProcessedFrameID ++;
  //  itsWin.reset();
}

// ######################################################################
////only use average as filter
//void VanishingPointDetector::updateSimpleFilter()
//{
//  Point2D<int> vp = itsVp; 
//
//}
void VanishingPointDetector::updateFilter()
{
  //update filter from color line
  //itsVpdf.predictState();
  //itsVpdf.predictObservation(itszNoise);

  Point2D<int> vp = itsVp; 
  Point2D<int> left= itsLeftColorPoint;
  Point2D<int> right = itsRightColorPoint;
  //LINFO("Color LR=======>>>>>>> %d,%d",itsLeftColorPoint.i,itsRightColorPoint.i);
  Image<double> z(1,4,ZEROS);
  z[0] = vp.i;
  z[1] = vp.j;
  z[2] = left.i;
  z[3] = right.i;

  itsZprobColor = itsVpdf.getLikelihood(z, Image<double>());
  //LINFO("============Color zProb %e",itsZprobColor);
  if(itsZprobColor > 1.0e-11 || itsFrameID < 10)//don't drop it until 10 frames
    itsVpdf.update(z,itszNoise);

  //update filter from gabor line
  itsVpdf.predictState();
  itsVpdf.predictObservation(itszNoise);

  left= itsLeftOcrPoint;
  right = itsRightOcrPoint;
  //LINFO("OCR LR=======>>>>>>> %d,%d",itsLeftOcrPoint.i,itsRightOcrPoint.i);
  Image<double> z2(1,4,ZEROS);
  z2[0] = vp.i;
  z2[1] = vp.j;
  z2[2] = left.i;
  z2[3] = right.i;

  itsZprobGabor = itsVpdf.getLikelihood(z2, Image<double>());
  //LINFO("============Gabor zProb %e",itsZprobGabor);
			
  if((itsZprobGabor > 1.0e-11 && itsZprobGabor > itsZprobColor)|| itsFrameID < 10)
    itsVpdf.update(z2,itszNoise);

  int filteredLeft,filteredRight;
  itsVpdf.getState(itsFilteredVp,filteredLeft,filteredRight);
  LINFO("UKF LR=======>>>>>>> %d,%d",filteredLeft,filteredRight);
  itsLeftRoadPoint  = Point2D<int>(filteredLeft,itsImage.getHeight());
  itsRightRoadPoint = Point2D<int>(filteredRight,itsImage.getHeight());	
}
// ######################################################################
VanishingPointDetector::~VanishingPointDetector()
{ }


// ######################################################################
// ######################################################################
// ######################################################################
void VanishingPointDetector::getOrientationSet(uint num,uint scale)
{
  itsNumOfAngle = num;
  itsAngle = 180.0/num;
  itsScale = scale;	

  itsOrientationSet = buildOrientationSet(itsImage,itsNumOfAngle,itsScale);
  buildSinCosTable(num);//for lookup later
  buildAtanSinCos_table(num);//for lookup later
}
// ######################################################################
ImageSet<float> VanishingPointDetector::buildOrientationSet(Image<PixRGB<byte> > image,uint num,uint scale)
{
  ImageSet<float> oSet(num);
  Image<float> const fIma = luminance(image);

  float ang = 180.0/num;
  for(uint angIndex = 0;angIndex < num ;angIndex++){
    float angle = ang*angIndex ;
    ImageSet<float> pyr =
      buildPyrGabor(fIma, 0, scale, angle, 2.20, 1.0, 5);//offset 45, for num =4, angle = 45,135,225,315
    Image<float> avg = getAverageGaborPyrImage(pyr);  
		//make vertical(num = 2)/ horizontal(num = 0) garbor weaker because vertical line usually not road
			if(angIndex == 2||angIndex == 0) avg/=2.0;//Try, might not work
     oSet[angIndex] = avg;
  }

  return oSet;

}

// ######################################################################
Image<float> VanishingPointDetector::getAverageGaborPyrImage(ImageSet<float> gaborPyr)
{
  uint numLevels = gaborPyr.size();
  uint w = gaborPyr[0].getWidth();
  uint h = gaborPyr[0].getHeight();
  Image<float> avgIma(w, h, ZEROS);
  uint totalUsableLayers = 0;

  for (uint i = 0; i < numLevels; i++)
  {
    uint scale = pow(2.0, i);

    if(w == uint(gaborPyr[i].getWidth()  * scale) && 
				h == uint(gaborPyr[i].getHeight() * scale) )
    {
      Image<float> temp = gaborPyr[i];
      inplaceNormalize(temp, 0.0F, 255.0F);
      Image<float> tempZoom = zoomXY(temp,scale);
			
      avgIma+=tempZoom;
      totalUsableLayers ++;
    }
    else{ LINFO("Size not fit. skip"); }
  }
  return avgIma/totalUsableLayers;

}
// ######################################################################
Image<float> VanishingPointDetector::getConfidenceMap()
{
	//if(!itsConfidenceMap.initialized())
	if(itsFrameID != itsProcessedFrameID)
		computeConfidenceMap(itsOrientationSet,itsAngleIndexMap,itsConfidenceMap);
	return itsConfidenceMap;
}
// ######################################################################
Image<PixRGB<byte> > VanishingPointDetector::getConfidenceMapImg(float confidenceThreshold)
{
	Image<PixRGB<byte> > colorConf = toRGB(normalizeFloat(itsConfidenceMap, true));
	Image<float>::iterator iptr = itsConfidenceMap.beginw(), stop = itsConfidenceMap.endw();  
	Image<PixRGB<byte> >::iterator iptr2 = colorConf.beginw(); 

	while(iptr != stop)
	{
		float conf = *iptr++;
		if(conf > confidenceThreshold ){
			*iptr2 = PixRGB<byte>(255,0,0);
		}
		iptr2++;
	}
	return colorConf;
}
// ######################################################################
Image<int> VanishingPointDetector::getIndexMap()
{
	//if(!itsAngleIndexMap.initialized())
	if(itsFrameID != itsProcessedFrameID)
		computeConfidenceMap(itsOrientationSet,itsAngleIndexMap,itsConfidenceMap);
	return itsAngleIndexMap;
}
// ######################################################################
void VanishingPointDetector::computeConfidenceMap(ImageSet<float> orientationSet,Image<int>& indexMap,Image<float>& confMap)
{

  uint numLevels = orientationSet.size();
  uint w = orientationSet[0].getWidth();
  uint h = orientationSet[0].getHeight();
  confMap.resize(w,h);
  indexMap.resize(w,h);
	int localMaxStart = numLevels/8 +1;//MY ALGO
	int localMaxEnd = numLevels/2;//MY ALGO
  for (uint x = 0 ; x < w ; x++)
    for (uint y = 0 ; y < h ; y++)
    {
      //build pixel array
      std::vector<float> vec,sorted_vec;
      std::vector<size_t> ix;
      for(uint z = 0 ; z < numLevels ;z++) vec.push_back(orientationSet[z].getVal(x,y));

      util::sortrank (vec,sorted_vec,ix);


      float localMax = getLocalMaxAverage(sorted_vec,numLevels-(localMaxEnd),numLevels-(localMaxStart-1));

      float max = sorted_vec[numLevels-1];
      int maxIndex = ix[numLevels-1];
      float conf = 1 - (localMax/max);
      confMap.setVal(x,y,conf);
      indexMap.setVal(x,y,maxIndex);

    }
  inplaceNormalize(confMap,0.0F,1.0F);

}
// ######################################################################
float VanishingPointDetector::confidenceQualityCount(float threshold)
{
  int w = itsConfidenceMap.getWidth();
  int h = itsConfidenceMap.getHeight();
	int count = 0;
	int total = w*h;
  for (int x = 0 ; x < w ; x++)
    for (int y = 0 ; y < h ; y++)
    {
			if(itsConfidenceMap.coordsOk(x,y) && itsConfidenceMap.getVal(x,y) > threshold){
				count++;				
			}
		}
	return (float)count/(float)total;	
	
}
// ######################################################################
float VanishingPointDetector::getLocalMaxAverage(std::vector<float> v,uint start,uint end)
{
  float sum = 0.0; uint count = 0 ;
  for(uint i = start; i < end ; i++){
    sum += v[i];
    count++;
  }
	if(count == 0) return 0;
  float avg = sum/count;

  return avg;

}

// ######################################################################
Image<float> VanishingPointDetector::getVoteMap(float confidenceThreshold,float radiusRatio,float matchAngleThreshold,Point2D<int> vp_prior,Dims region)
{
	if(vp_prior == Point2D<int>(0,0)){
		int w = itsConfidenceMap.getWidth();
		int h = itsConfidenceMap.getHeight();
		vp_prior = Point2D<int>(w/2,h/2);
		region  = itsConfidenceMap.getDims();
	}
	//float rad = matchAngleThreshold*(M_PI/180.0);
	//if(!itsVoteMap.initialized())
	if(itsFrameID != itsProcessedFrameID)
	  itsVoteMap = computeVoteMap(itsAngleIndexMap,itsConfidenceMap,confidenceThreshold,radiusRatio,matchAngleThreshold,vp_prior,region);
		
	float maxval;
	Point2D<int> maxPoint;
	findMax(itsVoteMap,maxPoint,maxval);	
	itsVp = maxPoint;
	return itsVoteMap;
}
// ######################################################################
Image<float> VanishingPointDetector::getVoteMap2(Image<float> mask, float confidenceThreshold,float radiusRatio,float matchAngleThreshold)
{
	//float rad = matchAngleThreshold*(M_PI/180.0);
	//if(!itsVoteMap.initialized())
	if(itsFrameID != itsProcessedFrameID)
	  itsVoteMap = computeVoteMap(itsAngleIndexMap,itsConfidenceMap,confidenceThreshold,radiusRatio,matchAngleThreshold,mask);
		
	float maxval;
	Point2D<int> maxPoint;
	float minDist = 9999;

	Image<float> copyMap = itsVoteMap;//not sure this will copy value or just pointer


	//My stuff
	//findMax that is closest to previous vp
	//get top 10 vp, choose minimal dist from previous vp
	for(int i = 0;i < 1; i++){//try diff number and see the difference....FIXXX
		Point2D<int> pt;
		findMax(copyMap,pt,maxval);	
		float dist = distance(itsVp,pt);	
		LINFO("%d: pt(%d,%d)'s distance from previous vp is %f, maxval %f",i,pt.i,pt.j,dist,maxval);	

		if(dist < minDist){
			if(minDist != 9999) LINFO("=================================>>>>>>>>>>>>>>>>>>>>>>> That's what I am looking for....non-first max win!!!");
			minDist = dist;
			maxPoint = pt;
		}
		subtractMax(copyMap,pt,itsImage.getDims()/10);	
	}

	LINFO("Max pt is (%d,%d), distance from previous vp is %f",maxPoint.i,maxPoint.j,minDist);	

	itsVp = maxPoint;
	return itsVoteMap;
}
// ######################################################################
//Image<float> VanishingPointDetector::getVoteMap3(int pixelGrid, float confidenceThreshold,float radiusRatio,float matchAngleThreshold)
//{
//	//float rad = matchAngleThreshold*(M_PI/180.0);
//	//if(!itsVoteMap.initialized())
//	if(itsFrameID != itsProcessedFrameID)
//	  itsVoteMap = computeCoarseVoteMap(itsAngleIndexMap,itsConfidenceMap,confidenceThreshold,radiusRatio,matchAngleThreshold,pixelGrid);
//		
//	float maxval;
//	Point2D<int> maxPoint;
//	findMax(itsVoteMap,maxPoint,maxval);	
//	itsVp = maxPoint;
//	return itsVoteMap;
//}
// ######################################################################

Image<float> VanishingPointDetector::computeCoarseVoteMap(Image<int> indexMap,Image<float> confMap,float threshold,float radiusRatio,float matchAngleThreshold,int pixelGrid)
{
	
  Image<float>voteMap (confMap.getDims(),ZEROS);

  int w = confMap.getWidth();
  int h = confMap.getHeight();
  float radius = radiusRatio* sqrt(w*w+h*h);
  buildXboundTable(radius);//for speed optimization
  buildXYdistanceTable((int)radius);
  buildXYatanTable((int)radius);
  int w10 = w/10;
	int h10 = h/10;
  for (int x = w10; x < w-w10; x+= pixelGrid)
  {
    for (int y = h10 ; y < h-h10; y+=pixelGrid)//FIXXX, only look for middle half of image
    {
				float score = getVanishPointScore(x,y,threshold,matchAngleThreshold,radius);
				voteMap.setVal(x,y,score);
    }
  }
  return voteMap;
	
}
// ######################################################################

Image<float> VanishingPointDetector::computeVoteMap(Image<int> indexMap,Image<float> confMap,float threshold,float radiusRatio,float matchAngleThreshold,Image<float> mask)
{
	
  Image<float>voteMap (confMap.getDims(),ZEROS);

  int w = confMap.getWidth();
  int h = confMap.getHeight();
  float radius = radiusRatio* sqrt(w*w+h*h);
  buildXboundTable(radius);//for speed optimization
  buildXYdistanceTable((int)radius);
  buildXYatanTable((int)radius);
  int w10 = w/10;
	int h5 = h/5;
	int pixelGrid = 8;
	int maskSize = 0;
  int x_start = w10;
  int x_end = w-w10;
  int y_start = h5+15;
  int y_end = h-h5*2;


//	int imageArea = w*h;
//	float memRatio = 0.0;
//	int pixelGrid2 = 8;
//
//	//upper half image
//  for (int x = w10; x < w-w10; x++)
//  {
//    for (int y = h10 ; y < h10*5; y++)//FIXXX, only look for middle half of image
//    {
//			if(x % pixelGrid2 == 0 && y % pixelGrid2 == 0){
//				float score = getVanishPointScore(x,y,threshold,matchAngleThreshold,radius);
//				voteMap.setVal(x,y,score);
//			}
//    }
//  }
  //lower half image
  for (int x = x_start; x < x_end; x++)
  {
    for (int y = y_start ; y < y_end; y++)//FIXXX, only look for middle half of image
    {
			if(mask.getVal(x,y) > 0.0 || (x % pixelGrid == 0 && y % pixelGrid == 0)){
				float score = getVanishPointScore(x,y,threshold,matchAngleThreshold,radius);
				voteMap.setVal(x,y,score);
				maskSize++;
			}
    }
  }
	itsMaskSize = maskSize;
  return voteMap;
	
}
// ######################################################################

Image<float> VanishingPointDetector::computeVoteMap(Image<int> indexMap,Image<float> confMap,float threshold,float radiusRatio,float matchAngleThreshold,Point2D<int> vp_prior,Dims region)
{
	
  Image<float>voteMap (confMap.getDims(),ZEROS);

  int w = confMap.getWidth();
  int h = confMap.getHeight();
  float radius = radiusRatio* sqrt(w*w+h*h);
  buildXboundTable(radius);//for speed optimization
  buildXYdistanceTable((int)radius);
  buildXYatanTable((int)radius);
  int w10 = w/10;
	int h5 = h/5;
  int x_start = max(w10,vp_prior.i - region.w()/2);
  int y_start = max(h5+15,vp_prior.j - region.h()/2);
  int x_end = min(w-w10,vp_prior.i + region.w()/2);
  int y_end = min(h-h5*2,vp_prior.j + region.h()/2);

  for (int x = x_start ; x < x_end ; x++)
  {
    for (int y = y_start ; y < y_end ; y++)
    {
			if(confMap.coordsOk(x,y)){
				float score = getVanishPointScore(x,y,threshold,matchAngleThreshold,radius);
				voteMap.setVal(x,y,score);
			}
    }
  }
  return voteMap;
	
}
// ######################################################################
Image<float> VanishingPointDetector::computeVoteMap(Image<int> indexMap,Image<float> confMap,float threshold,float radiusRatio,float matchAngleThreshold)
{
  int w = confMap.getWidth();
  int h = confMap.getHeight();
  Point2D<int> image_center = Point2D<int>(w/2,h/2);

  return computeVoteMap(indexMap,confMap,threshold,radiusRatio,matchAngleThreshold,image_center,indexMap.getDims());
}

// ######################################################################
float VanishingPointDetector::getVanishPointScore(int x,int y,float threshold,float matchAngleThreshold,float radius,int shape)
{
	
	itsThreshold = threshold; 
	float score = 0.0;
  //int shape = DISK;

	//for each half disk
	for(int yy = 0;yy <= radius; ++yy)
	{
		//int xbound = int(sqrtf(float(radius*radius) - yy*yy));
		int xbound;
		if(shape == DISK)
			xbound = itsXbound_table[yy];
		else
			xbound = radius;
		//LINFO("x %d y %d  yy %d xb %d radius %f",x,y,yy,xbound,radius);
		for(int xx = -xbound; xx <= xbound; ++xx)
		{
			if(itsConfidenceMap.coordsOk(x+xx,y+yy) && itsConfidenceMap.getVal(x+xx,y+yy) > threshold){


				//float distance = sqrt(xx*xx + yy*yy);
				float distance = xyDistance(xx,yy,(int)radius);//optimize by using lookup table
				//	distance2 = 0;
				//float orientationAngel = itsAngleIndexMap.getVal(x+xx,y+yy)*angle;//slower  
				int gaborAngleIndex = itsAngleIndexMap.getVal(x+xx,y+yy); //optimize by using lookup table
				float pvoAngle = getAngle(x,y,x+xx,y+yy,gaborAngleIndex,radius);//make it faster by using sin/cos lookup table

				//float cond = 5/(1+2*distance);
				float kongsAngleThreshold = 5/(1+2*distance);
				float kongsScore = 1/(1+((pvoAngle*distance)*(pvoAngle*distance)));

				if(pvoAngle <= kongsAngleThreshold){
					score+= kongsScore;
				}
				//float oneScore = 1/(1 + fabs(pvoAngle)*distance);
				//if(fabs(pvoAngle) <= matchAngleThreshold){ 
				//	score+= oneScore; 
				//	//LINFO("x1%d,y1%d,x2%d,y2%d,dist %f oa %f ang %f,cond %f,sc %f",x,y,x+xx,y+yy,distance,orientationAngel,angle,cond,oneScore);
				//}	
			}
		}
	}
	return score;

}
// ######################################################################

float VanishingPointDetector::getAngle(int x1,int y1,int x2,int y2,float gaborAngle)
{
  //return 0.1;
  float rad = (gaborAngle/180.0)*M_PI;
  int v1x = x2 - x1;
  int v1y = y2 - y1;
  float v2x = cos(rad);
  float v2y = sin(rad);
  //float result  = atan2(v2y,v2x) - atan2(v1y,v1x);//12153 ms 
  float result  = fast_atan2f(v2y,v2x) - fast_atan2f(v1y,v1x);//9132 ms
  return result * 180.0/M_PI;
}

// ######################################################################
float VanishingPointDetector::getAngle(int x1,int y1,int x2,int y2,int gaborAngleIndex,float radius)
{
  //return 0.1;
  //Here is some optimization , 
  //Original speed : 12153ms 
  //Replace atan2 to fast_atan2 : 9132ms
  //Use lookup sin/cos : 6163ms
  //USe lookup xbound  : 6096ms
  //Use lookup sqrt(xx^2+yy^2): 5295ms
  //Use PI_UNDER_180 :4171ms
  //Use radius compairson : 3980ms
  //Base Line(return 0.1): 1681ms 
  //return 0.1;
  int v1x = x2 - x1;
  int v1y = y2 - y1;
  //float v2x = lookup_cos(gaborAngleIndex);//lookup table
  //float v2y = lookup_sin(gaborAngleIndex);//lookup table
  //float result  = atan2(v2y,v2x) - atan2(v1y,v1x);//12153 ms -> 8265ms 
  
  //float result  = fast_atan2f(v2y,v2x) - fast_atan2f(v1y,v1x);//9132 ms -> 5295ms
  //LINFO("atan %f,look atan %f",atan2(v2y,v2x),itsAtanSinCos_table[gaborAngleIndex]);
  float xyatan = xyAtan(v1y,v1x,(int)radius);
  //LINFO("atan %f,look atan %f",atan2(v1y,v1x),xyatan);
  //float result  = itsAtanSinCos_table[gaborAngleIndex] - fast_atan2f(v1y,v1x);//9132 ms -> 5295ms -> 2346ms
  float result  = itsAtanSinCos_table[gaborAngleIndex] - xyatan;//9132 ms -> 5295ms -> 2346ms
  //v1x = v1x; v1y = v1y; v2x = v2x; v2y=v2y;
  /*
  float v2yx = v2x==0.0 ? 0.0 : v2y/v2x;
  float v1yx = v1x==0.0 ? 0.0 : v1y/v1x;
  float result = v2yx - v1yx;//3015ms
  */
  //return  result*180.0/M_PI;//744.45ms -> 5424ms
  return  result*PI_UNDER_180;//742.45ms  -> 4171ms ,if not times PI_UNDER_180 ,3980ms
}




// ######################################################################

// Vanishing Point Tracker
// ######################################################################
// ######################################################################
// ######################################################################

//make all value in the window to zero, then we can find second max
void VanishingPointDetector::subtractMax
(Image<float>& input,Point2D<int> center,Dims window)
{

	int x_start = center.i - window.w()/2;
	int y_start = center.j - window.h()/2;
	int x_end   = center.i + window.w()/2;
	int y_end   = center.j + window.h()/2;

	for (int x = x_start ; x < x_end ; x++)
	{
		for (int y = y_start ; y < y_end ; y++)
		{
			if(input.coordsOk(x,y)) input.setVal(x,y,0.0);
			
		}
	}
}
void VanishingPointDetector::addMemoryMapRegion
(Image<float>& input,Point2D<int> center,Dims window)
{
  int forgetRate = 5.0;//forget after 5 frames;

  int x_start = center.i - window.w()/2;
  int y_start = center.j - window.h()/2;
  int x_end   = center.i + window.w()/2;
  int y_end   = center.j + window.h()/2;

  for (int x = x_start ; x < x_end ; x++)
    {
      for (int y = y_start ; y < y_end ; y++)
	{
	  if(input.coordsOk(x,y)){
	    float val = input.getVal(x,y);

	    //set the max memory rate,in case we never forget something
	    if(val < 500.0) {
	      val+=forgetRate;
	      input.setVal(x,y,val);				
	    }
	  }
	}
    }
}
// ######################################################################
void VanishingPointDetector::forgetMemoryMap(Image<float>& input)
{
	Image<float>::iterator it = input.beginw(),stop = input.endw();
	while(it != stop){
		if(*it > 0) (*it)--;
		it++;		
	}
}

// Road Segmentation Section 
// ######################################################################
// ######################################################################
// ######################################################################
Point2D<int> VanishingPointDetector::computeBottomPoint(Point2D<int> point,float angle)
{
	//tan(a) = (H - Y1) / (X2 - X1)
	//X2 = ((H-Y1) / tan(a) ) + x1
	//Angle from right to left, ex 45 deg is from middle to right bottom
  if(angle < 0){
		LINFO("Angle %3.1f is negative, change sign to %3.0f",angle,fabs(angle));
		angle = fabs(angle);
	}
	if(angle == 0.0){
		LINFO("Angle %3.1f is too small, set to 5.00",angle);
		angle = 5.0;
	}
	int x1 = point.i,y1 = point.j,y2 = itsImage.getHeight();
	float x2 = ((y2 - y1) / tan((angle/180.0)*M_PI))+x1;
	//LINFO("x1,y1=(%d,%d), x2,y2=(%d,%d),angle %f",x1,y1,(int)x2,y2,angle);
	return Point2D<int>(x2,y2);
	
}
// ######################################################################
Point2D<int> VanishingPointDetector::computeVpNeighborPoint(Point2D<int> bottomPoint,float angle)
{
	//tan(a) = (vp.y - Y1) / (X2 - X1)
	//X2 = ((H-Y1) / tan(a) ) + x1
	//Angle from right to left, ex 45 deg is from middle to right bottom

	int x1 = bottomPoint.i,y1 = bottomPoint.j,y2 = itsVp.j;
	float x2 = ((y2 - y1) / tan((angle/180.0)*M_PI))+x1;
	//LINFO("x1,y1=(%d,%d), x2,y2=(%d,%d)",x1,y1,(int)x2,y2);
	return Point2D<int>(x2,y2);
	
}
// ######################################################################
//Given one line and a point x3 between p1,p2. It will return x3,y3
float VanishingPointDetector::computeOCR(float angle)
{
  Point2D<int> p1 = itsVp;
	Point2D<int> p2 = computeBottomPoint(itsVp,angle);

  int dx = p2.i - p1.i, ax = abs(dx) << 1, sx = signOf(dx);
  int dy = p2.j - p1.j, ay = abs(dy) << 1, sy = signOf(dy);
  int x = p1.i, y = p1.j;

  const int w = itsImage.getWidth();
  const int h = itsImage.getHeight();
  int count = 0;
	int consistentPoint  = 0;
  if (ax > ay)
    {
      int d = ay - (ax >> 1);
      while(x != p2.i)
        {
					if (x >= 0 && x < w && y >= 0 && y < h){
						// do some x, y here
						if(itsAngleIndexMap.getVal(x,y)*itsAngle == angle)
							consistentPoint ++;
						count++;
					}
          if (d >= 0) { y += sy; d -= ax; }
          x += sx; d += ay;
        }
    }
  else
    {
      int d = ax - (ay >> 1);
      while(y!= p2.j)
        {
					if (x >= 0 && x < w && y >= 0 && y < h)
					{
						if(itsAngleIndexMap.getVal(x,y)*itsAngle == angle)
							consistentPoint ++;
						count++;
					}
          if (d >= 0) { x += sx; d -= ay; }
          y += sy; d += ax;
        }
		}
		if(count == 0) return -1;
		
		return (float)consistentPoint/(float)count;
	
}
// ######################################################################
//compute angle offset mean
float VanishingPointDetector::computeAOM(float angle)
{
  Point2D<int> p1 = itsVp;
	Point2D<int> p2 = computeBottomPoint(itsVp,angle);

  int dx = p2.i - p1.i, ax = abs(dx) << 1, sx = signOf(dx);
  int dy = p2.j - p1.j, ay = abs(dy) << 1, sy = signOf(dy);
  int x = p1.i, y = p1.j;

  const int w = itsImage.getWidth();
  const int h = itsImage.getHeight();
  int count = 0;
	float sum = 0.0;
  if (ax > ay)
    {
      int d = ay - (ax >> 1);
      while(x != p2.i)
        {
					if (x >= 0 && x < w && y >= 0 && y < h){
						// do some x, y here
						float oa = getOffsetAngle(x,y,angle);
						if(oa >= 0.0){
							sum+= oa;
							count++;
						}
					}
          if (d >= 0) { y += sy; d -= ax; }
          x += sx; d += ay;
        }
    }
  else
    {
      int d = ax - (ay >> 1);
      while(y!= p2.j)
        {
					if (x >= 0 && x < w && y >= 0 && y < h)
					{
						float oa = getOffsetAngle(x,y,angle);
						if(oa >= 0.0){
							sum+= oa;
							count++;
						}
					}
          if (d >= 0) { x += sx; d -= ay; }
          y += sy; d += ax;
        }
		}
		if(count == 0) return -1;
		
		return sum/count;

}

// ######################################################################
//compute gabor response variance
float VanishingPointDetector::computeGRV(float angle,float& norm,float& maxStdev)
{
  Point2D<int> p1 = itsVp;
	Point2D<int> p2 = computeBottomPoint(itsVp,angle);
//	LINFO("vp %d %d bottom %d %d",p1.i,p1.j,p2.i,p2.j);

  int dx = p2.i - p1.i, ax = abs(dx) << 1, sx = signOf(dx);
  int dy = p2.j - p1.j, ay = abs(dy) << 1, sy = signOf(dy);
  int x = p1.i, y = p1.j;

  const int w = itsImage.getWidth();
  const int h = itsImage.getHeight();
	std::vector<std::vector<float> > responseBin;
	responseBin.resize(itsNumOfAngle);

  if (ax > ay)
    {
      int d = ay - (ax >> 1);
      while(x != p2.i)
        {
					if (x >= 0 && x < w && y >= 0 && y < h){
						// do some x, y here
						int angleIndex = itsAngleIndexMap.getVal(x,y);
						responseBin[angleIndex].push_back(itsConfidenceMap.getVal(x,y));
					}
          if (d >= 0) { y += sy; d -= ax; }
          x += sx; d += ay;
        }
    }
  else
    {
      int d = ax - (ay >> 1);
      while(y!= p2.j)
        {
					if (x >= 0 && x < w && y >= 0 && y < h)
					{
						int angleIndex = itsAngleIndexMap.getVal(x,y);
						responseBin[angleIndex].push_back(itsConfidenceMap.getVal(x,y));
					}
          if (d >= 0) { x += sx; d -= ay; }
          y += sy; d += ax;
        }
		}
	
	int totalPixelOnLine = 0;
	int maxBinSize = 0;
	int maxBinSizeIndex = 0;

	for(int i = 0 ;i < itsNumOfAngle;i++)
	{
		totalPixelOnLine+= responseBin[i].size();
		if((int)responseBin[i].size() > maxBinSize)
		{
			maxBinSize = responseBin[i].size();
			maxBinSizeIndex = i;
		}
		//LINFO("Angle %2.2f: Channel %d : Size %2d : Stdev : %2.6f",angle,i,(int)responseBin[i].size(),stdev(responseBin[i]))	;		
	}
	norm = maxBinSize/(float) totalPixelOnLine;
	maxStdev = stdev(responseBin[maxBinSizeIndex]);
	float grvScore = norm*norm /  maxStdev;
	//LINFO("Total Bin size %2d,max bin size[%d] %2d, norm %2.4f,stdev %2.4f, Final GRV Score %2.4f",totalPixelOnLine,maxBinSizeIndex,maxBinSize,norm,maxStdev,grvScore);
//	if(norm < 0.8)
//		return -1;
	return grvScore;
}

// ######################################################################

float VanishingPointDetector::getOffsetAngle(int x,int y,float angle)
{
 float oriAngle = itsAngleIndexMap.getVal(x,y) * itsAngle;
// LINFO("itsThreshold is %f",itsThreshold);
 //if(itsConfidenceMap.getVal(x,y) > itsThreshold)
	 return abs(oriAngle - angle)	;
 //return -1;	 
}

// ######################################################################
PixRGB<byte> VanishingPointDetector::getFanAreaColor(float angle1,float angle2,Point2D<int> topPoint)
{
	if(angle1 <= 0.0){
		//LINFO("Angle1 %2.2f is too small, set to 5.0 ",angle1);
		angle1 = 5.0;
	}
	if(angle2 >= 180.0){
		//LINFO("Angle2 %2.2f is too big, set to 175.0 ",angle2);
		angle2 = 175.0;
	}
	Point2D<int> p1 = (topPoint.i == -1) ? itsVp : topPoint;
	Point2D<int> p2 = computeBottomPoint(itsVp,angle1);
	Point2D<int> p3 = computeBottomPoint(itsVp,angle2);
	
	// LINFO("topPt(%d %d)  itsVP(%d %d): p1(%d %d) p2(%d %d), p3(%d %d)", 
	//       topPoint.i, topPoint.j, itsVp.i, itsVp.j , p1.i, p1.j, p2.i, p2.j, p3.i, p3.j);

	return getFanAreaColor(p1,p2,p3);
	
}
// ######################################################################
PixRGB<byte> VanishingPointDetector::getFanAreaColor(Point2D<int> topPoint,Point2D<int> leftPoint,Point2D<int> rightPoint)
{
  Point2D<int> p1 = topPoint; 
  Point2D<int> p2 = leftPoint; 
  Point2D<int> p3 = rightPoint; 
  int w = itsImage.getWidth();
  int h = itsImage.getHeight();	
  PixRGB<long> avgColor(0,0,0);
  int pixCount = 0;

  for(int y = p1.j; y < h;y++)
    {
      Point2D<int> it1 = intersectPoint(p1,p2,Point2D<int>(0,y),Point2D<int>(w,y));
      Point2D<int> it2 = intersectPoint(p1,p3,Point2D<int>(0,y),Point2D<int>(w,y));	
      if(it1.i < 0) it1.i = 0;
      if(it2.i < 0) it2.i = 0;
      if(it1.i > w) it1.i = w;
      if(it2.i > w) it2.i = w;

      int length = abs(it1.i - it2.i);
      int x_init = (it1.i < it2.i) ? it1.i : it2.i; 

      for(int x = x_init; x < length+x_init;x++)
	{
	  if(itsImage.coordsOk(x,y)){
	    avgColor += itsImage.getVal(x,y);
	    pixCount++;
	  }
	}
    }
  if(pixCount != 0) avgColor/=pixCount;
  return (PixRGB<byte>)avgColor;
	
}
//// ######################################################################
//PixLab<byte> VanishingPointDetector::getFanAreaColorLab(float angle1,float angle2)
//{
//	Point2D<int> p1 = itsVp;
//	Point2D<int> p2 = computeBottomPoint(itsVp,angle1);
//	Point2D<int> p3 = computeBottomPoint(itsVp,angle2);
//	int w = itsImage.getWidth();
//	int h = itsImage.getHeight();	
//	PixLab<long> avgColor(0,0,0);
//	Image<PixLab<byte> > labImage(itsImage);//convert input image from RGB to LAB
//	int pixCount = 0;
//	for(int y = p1.j; y < h;y++)
//	{
//		Point2D<int> it1 = intersectPoint(p1,p2,Point2D<int>(0,y),Point2D<int>(w,y));
//		Point2D<int> it2 = intersectPoint(p1,p3,Point2D<int>(0,y),Point2D<int>(w,y));	
//		int length = abs(it1.i - it2.i);
//		int x_init = (it1.i < it2.i) ? it1.i : it2.i; 
//		for(int x = x_init; x < length+x_init;x++){
//			if(labImage.coordsOk(x,y)){
//				avgColor += labImage.getVal(x,y);
//				pixCount++;
//			}
//		}
//	}
//	if(pixCount != 0) avgColor/=pixCount;
//	return (PixLab<byte>)avgColor;
//	
//}
// ######################################################################

//This is reversed line finding from bottom point to vp
PixRGB<byte> VanishingPointDetector::getFanAreaColorFromBottom(float angle1,float angle2,Point2D<int> bottomPoint)
{
	Point2D<int> p1 = bottomPoint;

	if(angle1 <= 0.0){
		LINFO("Angle1 %2.2f is too small, set to 5.0 ",angle1);
		angle1 = 5.0;
	}
	if(angle2 >= 180.0){
		LINFO("Angle2 %2.2f is too big, set to 175.0 ",angle2);
		angle2 = 175.0;
	}

	Point2D<int> p2 = computeVpNeighborPoint(p1,angle1);
	Point2D<int> p3 = computeVpNeighborPoint(p1,angle2);
	int w = itsImage.getWidth();
	int h = itsImage.getHeight();
	PixRGB<long> avgColor(0,0,0);
	int pixCount = 0;
	for(int y = p2.j; y < h;y++)
	{
		Point2D<int> it1 = intersectPoint(p1,p2,Point2D<int>(0,y),Point2D<int>(w,y));
		Point2D<int> it2 = intersectPoint(p1,p3,Point2D<int>(0,y),Point2D<int>(w,y));	
		int length = abs(it1.i - it2.i);
		int x_init = (it1.i < it2.i) ? it1.i : it2.i; 
		for(int x = x_init; x < length+x_init;x++){
			if(itsOutputImage.coordsOk(x,y)){
				avgColor += itsImage.getVal(x,y);				
				pixCount++;
				//itsOutputImage.setVal(x,y,PixRGB<byte>(255,0,0));
			}
		}
	}
	if(pixCount != 0) avgColor/=pixCount;
	return (PixRGB<byte>)avgColor;
	
}

// ######################################################################
void VanishingPointDetector::drawFanArea(float angle1,float angle2,PixRGB<byte> color)
{
	Point2D<int> p1 = itsVp;
	Point2D<int> p2 = computeBottomPoint(itsVp,angle1);
	Point2D<int> p3 = computeBottomPoint(itsVp,angle2);
	int w = itsImage.getWidth();
	int h = itsImage.getHeight();

	for(int y = p1.j; y < h;y++)
	{
		Point2D<int> it1 = intersectPoint(p1,p2,Point2D<int>(0,y),Point2D<int>(w,y));
		Point2D<int> it2 = intersectPoint(p1,p3,Point2D<int>(0,y),Point2D<int>(w,y));	
		int length = abs(it1.i - it2.i);
		int x_init = (it1.i < it2.i) ? it1.i : it2.i; 
		for(int x = x_init; x < length+x_init;x++){
			if(itsOutputImage.coordsOk(x,y)){
				itsOutputImage.setVal(x,y,color);
			}
		}
	}


}

// ######################################################################
float VanishingPointDetector::findBestRay(float& bestOcrRayAngle,float& bestColorRayAngle,float initAngle,int colorSpace)
{
  LINFO("Current road color %d %d %d",itsRoadColor.red(),itsRoadColor.green(),itsRoadColor.blue());

  //left angle start from 130 +- 20 deg
  //right angle start from (best left - 90)+-20
  float maxGRV = 0.0,bestOcrAngle = 0.0;
  float maxColorDiff = 0.0,maxColorDiffAngle = 0.0;
  float maxGRVSCD = 0.0,   maxGRVSCDang = 0.0;

  float angleRange = 20.0f;

  std::vector<float> normVec;
  std::vector<float> stdevVec;
  std::vector<float> angleVec;
  float colorDiff = 0.0;//color diff from A1/A2
  float roadColorDiff = 0.0;//color diff from A1/RoadColor or A2/RoadColor
  float newColorDiffScore = 0.0;
  float alpha = 1.0;//road diff gain
  //float eps = 0.0000001;//some very small number
  //LINFO("initAngle %f range %f to %f",initAngle,initAngle-20.0,initAngle+20.0);

	float angleStart = initAngle-angleRange >   20.0 ? initAngle-angleRange : 20.0;
	float angleEnd   = initAngle+angleRange <= 160.0 ? initAngle+angleRange : 160.0;

  for(float angle = angleStart ;angle <= angleEnd;angle+=5)
    {
      PixRGB<byte> area1 = getFanAreaColor(angle-angleRange,angle);
      PixRGB<byte> area2 = getFanAreaColor(angle,angle+angleRange);
			
      // LINFO("a1: (%d %d %d) a2: (%d %d %d)", 
      // 	    area1.red(), area1.green(), area1.blue(),
      // 	    area2.red(), area2.green(), area2.blue() );
		
      if(colorSpace == RGB){
	if(initAngle >= 130.0)//if is left boundary
	  {
	    roadColorDiff = diffRGB(itsRoadColor,area1);// A1 is outside, A2 is roadside
							//LINFO("Compute left");
	  }
	else //if is right boundary
	  {
	    roadColorDiff = diffRGB(itsRoadColor,area2);// A1 is roadside, A2 is outside
							//LINFO("Compute right");
	  }

	colorDiff = diffRGB(area1,area2);
      }else{
	colorDiff = diffLAB(area1,area2);
      }
      float colorDiff_in_time = roadColorDiff*alpha;
      // if(colorDiff_in_time > colorDiff)
      // 	{
      // 	  newColorDiffScore = colorDiff;
      // 	}
      // else
	newColorDiffScore = colorDiff - colorDiff_in_time; //FIXXX

      //LINFO("Color Diff for Angle %3f is %4.2f, road diff is %4.2f,new score %4.2f",angle,colorDiff,roadColorDiff,newColorDiffScore);

      //Check we have road prior or not
      if(itsProcessedFrameID < 10)
	{
	  // LINFO("no prior");

	  if(colorDiff> maxColorDiff  ||
	     angle == initAngle-angleRange)
	    {
	      maxColorDiff = colorDiff;
	      maxColorDiffAngle = angle;
	      //	LINFO("Diff %f Angle %f",maxColorDiff,maxColorDiffAngle);
	    }
	}
      else
	{
	  // LINFO("newColorDiffScore > maxColorDiff: %f > %f ==> %d", 
	  // 	newColorDiffScore, maxColorDiff, newColorDiffScore > maxColorDiff);

	  // 
	  if(newColorDiffScore > maxColorDiff ||
	     angle == initAngle-angleRange)
	    {
	      maxColorDiff = newColorDiffScore;
	      maxColorDiffAngle = angle;
	      //LINFO("Diff %f Angle %f",maxColorDiff,maxColorDiffAngle);
	    }
      }

      // LINFO("COLOR testing angle: %f|%f|%f: color  diff: %f", 
      // 	    angle-angleRange,angle, angle+angleRange, colorDiff);

      float norm,maxStdev;					
      float grvs = computeGRV(angle,norm,maxStdev);
      angleVec.push_back(angle);
      normVec.push_back(norm);
      stdevVec.push_back(maxStdev);

      if( grvs > maxGRV  ||
	  angle == initAngle-angleRange)
	{ 
	  maxGRV = grvs;
	  bestOcrAngle = angle;
	}
      float grvscd = 0.0;	
      if(itsProcessedFrameID < 10)
	{
	  grvscd = grvs*colorDiff;//Combine
	}else{
	grvscd = grvs*newColorDiffScore;//Combine
      }
      if(grvscd > maxGRVSCD  ||
	 angle == initAngle-angleRange)
	{
	  maxGRVSCD = grvscd;
	  maxGRVSCDang = angle;
	}
      //if(ocr != 0.0){
      //	drawLine(itsOutputImage,itsVp,computeBottomPoint(itsVp,angle),PixRGB<byte>(255,255,255),1);
      //LINFO("Angle %3d : OCR is %3f",(int)angle,grvs);
      //}

      LINFO("[%5.2f] colorDiff: %f| grv: %f", angle, colorDiff, grvs);


    }

  LINFO("Best Ocr %f,Color %f",bestOcrAngle,maxColorDiffAngle);
		
  //FIXXXX Instead choose top grvs, we sort grvs by norm term and check top five norm
  //Then select the one with max stdev term from top five grvs
  //std::vector<float> sorted_norm;
  //std::vector<size_t> norm_ix;
  //util::sortrank (normVec,sorted_norm,norm_ix);


  //float maxGRVTop5 = 0.0;			
  //int normSize = normVec.size()-1;
  //for(int i = 0;i< 5;i++)//pick top 5 norm score
  //{
  //	int normIndex = norm_ix[normSize-i];
  //	float tmpStdev = stdevVec[normIndex];
  //	float tmpNorm = normVec[normIndex];
  //	float grv = tmpNorm*tmpNorm / tmpStdev;
  //	if(grv > maxGRVTop5)
  //	{
  //		maxGRVTop5 = grv;
  //		bestOcrAngle = angleVec[normIndex];
  //	}
  //}

  bestOcrRayAngle = bestOcrAngle;
  bestColorRayAngle = maxColorDiffAngle;
  return maxGRVSCDang;
	
}

// ######################################################################
//Doing reversing tracking from bottom to find update of vanishing point
float VanishingPointDetector::reversingRay(Point2D<int> bottomPoint,float bottomAngle,int colorSpace)
{
  float maxBottomColorDiff = 0.0;
  float maxBottomColorDiffAngle = 0.0;
  for(float angle = bottomAngle-10;angle < bottomAngle+10;angle+=2)
    {
      if(angle-10 > 0.0 && angle+10 < 180.0)//make sure angle is in the range
	{
	  PixRGB<byte> area1 = getFanAreaColorFromBottom(angle-10,angle,bottomPoint);
	  PixRGB<byte> area2 = getFanAreaColorFromBottom(angle,angle+10,bottomPoint);

	  float colorDiff = diffRGB(area1,area2);
	  if(colorSpace == LAB)
	    colorDiff = diffLAB(area1,area2);
	  else
	    colorDiff = diffRGB(area1,area2);
	  if(colorDiff > maxBottomColorDiff)
	    {
	      maxBottomColorDiff = colorDiff;
	      maxBottomColorDiffAngle = angle;
	    }
	}
    }
  return maxBottomColorDiffAngle;
}
// ######################################################################
void VanishingPointDetector::findRoad()
{
  float bestOcrRayAngle,bestColorRayAngle;

  //LINFO("\n\n\n\n--------------------------------------itsVp before find road%d,%d \n\n\n",itsVp.i,itsVp.j);

  //find left road

  float bestLeftCombAng = findBestRay(bestOcrRayAngle,bestColorRayAngle);
  LINFO("Best Left Angle: GRV %f, Color %f, Combine %f",bestOcrRayAngle,bestColorRayAngle,bestLeftCombAng);

  itsLeftOcrPoint   = computeBottomPoint(itsVp,bestOcrRayAngle);
  itsLeftColorPoint = computeBottomPoint(itsVp,bestColorRayAngle);
  //LINFO("Locr: %d %d", itsLeftOcrPoint.i, itsLeftOcrPoint.j);


  itsLeftColorAngle = bestColorRayAngle;
  itsLeftOcrAngle   = bestOcrRayAngle;

  //find right
  float bestRightCombAng = findBestRay(bestOcrRayAngle,bestColorRayAngle,(180-bestLeftCombAng));
  LINFO("Best Right Angle: GRV %f, Color %f, Combine %f",bestOcrRayAngle,bestColorRayAngle,bestRightCombAng);

  itsRightOcrPoint   = computeBottomPoint(itsVp,bestOcrRayAngle);
  itsRightColorPoint = computeBottomPoint(itsVp,bestColorRayAngle);

  itsRightColorAngle = bestColorRayAngle;
  itsRightOcrAngle   = bestOcrRayAngle;

  itsMiddleOcrPoint = (itsLeftOcrPoint + itsRightColorPoint)/2;
  itsMiddleColorPoint = (itsLeftColorPoint + itsRightColorPoint)/2;
  PixRGB<byte> rc = getFanAreaColor(itsVp,itsLeftColorPoint,itsRightColorPoint);
  if(rc != PixRGB<byte>(0,0,0)) itsRoadColor = rc;//only update color when is valid
  //itsRoadColor = rc;//might be invalid
  itsLeftRoadPoint  = computeBottomPoint(itsVp, bestLeftCombAng);
  itsRightRoadPoint = computeBottomPoint(itsVp, bestRightCombAng);
  //	itsRightColorPoint;
  //	itsRightColorAngle;
  //	itsRightOcrAngle;
  //	itsRightOcrPoint;
  //
  //	itsLeftRoadPoint = itsLeftColorPoint;//FIXXX
  //	itsRightRoadPoint = itsRightColorPoint;//FIXXX
  //	LINFO("vp %d %d left %d right %d",itsVp.i, itsVp.j,itsLeftRoadPoint.i,itsRightRoadPoint.i);


  LINFO("\n\n\n\n--------------------------------------itsVp after find road%d,%d \n\n\n",itsVp.i,itsVp.j);
}
// ######################################################################
Image<PixRGB<byte> > VanishingPointDetector::getFilteredImage(bool notext)
{
  Image<PixRGB<byte> > ima = itsImage;
  char buffer[200];
  drawLine(ima,itsVp,itsLeftRoadPoint,PixRGB<byte>(255,0,0),1);//draw left line in red
  drawLine(ima,itsVp,itsRightRoadPoint,PixRGB<byte>(0,255,0),1);//draw right line in green	
  drawLine(ima,itsVp,getMiddlePoint(),PixRGB<byte>(255,255,0),1);//draw right line in yellow


  if(!notext)
    {
      sprintf(buffer,"OCR Prob %2.2e",itsZprobGabor);			
      writeText(ima,Point2D<int>(0,0),buffer,PixRGB<byte>(255,0,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));
      sprintf(buffer,"Color Prob %2.2e",itsZprobColor);			
      writeText(ima,Point2D<int>(0,10),buffer,PixRGB<byte>(255,0,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));
    }

  return ima;
}

// ######################################################################
//! get middle point 4 pixel above bottom of image
//Point2D<int> VanishingPointDetector::computeEstMiddlePoint()
//{
//	
//			Point2D<int> mid = getMiddlePoint();
//			//LINFO("vp(%d,%d),middle(%d,%d),w %d,h %d",itsVp.i,itsVp.j,mid.i,mid.j,itsImage.getWidth(),itsImage.getHeight());
//			//LINFO("lr(%d,%d),rr(%d,%d)",itsLeftRoadPoint.i,itsLeftRoadPoint.j,itsRightRoadPoint.i,itsRightRoadPoint.j);
//		Point2D<int> point = intersectPoint(itsVp,mid,
//														Point2D<int>(0,itsImage.getHeight()-4),
//														Point2D<int>(itsImage.getWidth(),itsImage.getHeight()-4)
//				);
//				return point*2;//From 160 to 320
//}

// ######################################################################
Image<PixRGB<byte> > VanishingPointDetector::getOutputImage()
{
  return itsOutputImage;
}

void VanishingPointDetector::computeOutputImage(Dims d)
{
  float scale = 1.0;
  if(d != Dims(0,0)){ 
    scale = (float)d.w()/(float)itsImage.getWidth();
    LINFO("scale is %f",scale);	
    itsOutputImage = rescale(itsImage,d);
  }else{
    itsOutputImage = itsImage;
  }
  //if(itsVp.j < itsImage.getHeight()/2) return itsOutputImage;//don't look for line is vp is above half of image


  //char buffer[200];
/*
  //draw left ocr
  sprintf(buffer,"GRV %3.0f",itsLeftOcrAngle);
  writeText(itsOutputImage,Point2D<int>(((itsVp+itsLeftOcrPoint)/2)*scale),buffer,PixRGB<byte>(255,0,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(5));
  drawLine(itsOutputImage,Point2D<int>(itsVp*scale),Point2D<int>(itsLeftOcrPoint*scale),PixRGB<byte>(255,0,0),scale);//draw ocr red line 
  //draw left color
  sprintf(buffer,"Color %3.0f",itsLeftColorAngle);
  writeText(itsOutputImage,Point2D<int>(((itsVp+itsLeftColorPoint)/2+Point2D<int>(0,7))*scale),buffer,PixRGB<byte>(0,0,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(5));
  drawLine(itsOutputImage,Point2D<int>(itsVp*scale),Point2D<int>(itsLeftColorPoint*scale),PixRGB<byte>(0,0,255),scale);//draw max color diff blue line 


  //draw right ocr
  sprintf(buffer,"GRV %3.0f",itsRightOcrAngle);
  writeText(itsOutputImage,Point2D<int>(((itsVp+itsRightOcrPoint)/2)*scale),buffer,PixRGB<byte>(255,0,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(5));
  drawLine(itsOutputImage,Point2D<int>(itsVp*scale),Point2D<int>(itsRightOcrPoint*scale),PixRGB<byte>(255,0,0),scale);//draw ocr red line 
  //draw right color
  sprintf(buffer,"Color %3.0f",itsRightColorAngle);
  writeText(itsOutputImage,Point2D<int>(((itsVp+itsRightColorPoint)/2+Point2D<int>(0,7))*scale),buffer,PixRGB<byte>(0,0,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(5));
  drawLine(itsOutputImage,Point2D<int>(itsVp*scale),Point2D<int>(itsRightColorPoint*scale),PixRGB<byte>(0,0,255),scale);//draw max color diff blue line 


  //draw middle ocr
  sprintf(buffer,"GRV ");
  writeText(itsOutputImage,Point2D<int>(((itsVp-itsMiddleOcrPoint)/2)*scale),buffer,PixRGB<byte>(255,255,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(5));
  drawLine(itsOutputImage,Point2D<int>(itsVp*scale),Point2D<int>(itsMiddleOcrPoint*scale),PixRGB<byte>(255,255,0),scale);//draw ocr yellow line 
  //draw middle color
  sprintf(buffer,"Color ");
  writeText(itsOutputImage,Point2D<int>(((itsVp-itsMiddleColorPoint)/2+Point2D<int>(0,7))*scale),buffer,PixRGB<byte>(128,128,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(5));
  drawLine(itsOutputImage,Point2D<int>(itsVp*scale),Point2D<int>(itsMiddleColorPoint*scale),PixRGB<byte>(128,128,0),scale);//draw color yellow line 
*/


  //! Draw left/right road boundary and vanishing point on given image
			
  LINFO("\n\n\n\n--------------------------------------itsVp before drawCircle %d,%d \n\n\n",itsVp.i,itsVp.j);
  drawCircle(itsOutputImage,Point2D<int>(itsVp*scale) ,5,PixRGB<byte>(20,50,255),2*scale);//vanishing point
  int tmpl,tmpr;
  int w = itsOutputImage.getWidth();
  int h = itsOutputImage.getHeight();

  //swap point if left point on the right side of right point
  if(itsLeftRoadPoint.i > itsRightRoadPoint.i){
    Point2D<int> tmpt = itsLeftRoadPoint;
    itsLeftRoadPoint = itsRightRoadPoint;
    itsRightRoadPoint = tmpt;
  }
  //draw left boundary
  if(itsLeftRoadPoint.i< 0)
    {
      Point2D<int> lep = intersectPoint(Point2D<int>(itsVp*scale),Point2D<int>(itsLeftRoadPoint*scale),Point2D<int>(0,0),Point2D<int>(0,h));//Left edge point :Point on image left edge
      drawLine(itsOutputImage,Point2D<int>(itsVp*scale),lep,PixRGB<byte>(0,255,0),2*scale);//screen center -Green line 
      drawLine(itsOutputImage,lep,Point2D<int>(0,h),PixRGB<byte>(0,255,0),2*scale);//screen center -Green line 
      tmpl = lep.i;
							
    }else	{
    drawLine(itsOutputImage,Point2D<int>(itsVp*scale) ,Point2D<int>(itsLeftRoadPoint*scale) ,PixRGB<byte>(0,255,0),2*scale);//screen center -Green line 
    tmpl = itsLeftRoadPoint.i*scale;
  }
  //draw right boundary
  if(itsRightRoadPoint.i*scale> w)
    {
      Point2D<int> rep = intersectPoint(Point2D<int>(itsVp*scale),Point2D<int>(itsRightRoadPoint*scale),Point2D<int>(w,0),Point2D<int>(w,h));//Right edge point :Point on image right edge
      drawLine(itsOutputImage,Point2D<int>(itsVp*scale),rep,PixRGB<byte>(0,255,0),2*scale);//screen center -Green line 
      drawLine(itsOutputImage,rep,Point2D<int>(w,h),PixRGB<byte>(0,255,0),2*scale);//screen center -Green line 
      tmpr = rep.i;

    }else{
    drawLine(itsOutputImage,Point2D<int>(itsVp*scale) ,Point2D<int>(itsRightRoadPoint*scale) ,PixRGB<byte>(0,255,0),2*scale);//screen center -Green line 
    tmpr = itsRightRoadPoint.i*scale;
  }
  Point2D<int> middleNavigablePoint = Point2D<int>((tmpl+tmpr)/2,h);//final output
  drawLine(itsOutputImage,Point2D<int>(itsVp*scale),middleNavigablePoint,PixRGB<byte>(0,191,255),2*scale);//navigatable line (Depp sky blue) 
  itsEstMiddlePoint = middleNavigablePoint;


  ////draw left reverse color
  //float bestLeftReverseAngle = reversingRay(itsLeftColorPoint,itsLeftColorAngle);
  //Point2D<int> leftVpPoint = computeVpNeighborPoint(itsLeftColorPoint,bestLeftReverseAngle);
  ////Check new vp should not too far from original vp :my algo
  //int vp_dist = abs(leftVpPoint.i - itsVp.i);
  //sprintf(buffer,"RV %2d",vp_dist);
  //writeText(itsOutputImage,(itsVp+leftVpPoint)/2+Point2D<int>(0,14),buffer,PixRGB<byte>(0,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(5));
  //if(vp_dist < itsImage.getWidth()/16)
  //	drawLine(itsOutputImage,itsLeftColorPoint,leftVpPoint,PixRGB<byte>(0,255,255),1);//draw cayenne line for reverse line
  //else
  //	drawLine(itsOutputImage,itsLeftColorPoint,leftVpPoint,PixRGB<byte>(128,128,128),1);//draw invalid line


  //my idea:if vp_dist is too large, probably color is doing wrong, then trust ocr more 

  /*
    std::vector<float> ocrVecNeighbos;
    for(int i = 0;i < (int)ocrVec.size(); i++)
    {
    //if(i == 0) ocrVecNeighbos.push_back((ocrVec[i]+ocrVec[i+1]));
    //else if(i == (int)ocrVec.size()-1) ocrVecNeighbos.push_back((ocrVec[i-1]+ocrVec[i]));
    //else ocrVecNeighbos.push_back((ocrVec[i-1]+ocrVec[i]+ocrVec[i+1]));
    ocrVecNeighbos.push_back(ocrVec[i]);//do not thing,just copy it
			 			 
    }
    std::vector<float> sorted_ocr;
    std::vector<size_t> ix;
    util::sortrank (ocrVecNeighbos,sorted_ocr,ix);
    float maxOcr = sorted_ocr[ocrVecNeighbos.size()-1];
    int maxIndex = ix[ocrVecNeighbos.size()-1];
    float maxAngle = maxIndex*itsAngle;

    int minIndex = ix[0];
    float minOcr = sorted_ocr[0];
    float minAngle = minIndex*itsAngle;
    drawLine(itsOutputImage,itsVp,computeBottomPoint(itsVp,maxAngle),PixRGB<byte>(255,0,0),1);//draw min line
    for(int i = 0 ;i< (int)sorted_ocr.size();i++)
    {
    minOcr = sorted_ocr[i];
    minIndex = ix[i];
    minAngle = minIndex*itsAngle;

    if(minOcr != 0.0) break;
    }

    for(int i = 0 ;i< 6;i++)
    drawLine(itsOutputImage,itsVp,computeBottomPoint(itsVp,ix[i+1]*itsAngle),PixRGB<byte>(255,255,0),1);//draw 2n max
    LINFO("Max Angle %3d : OCR is %3f : Min Angle %3d: OCR is %3ff",(int)maxAngle,maxOcr,(int)minAngle,minOcr);
  */

  //drawLine(itsOutputImage,itsVp,computeBottomPoint(itsVp,135),PixRGB<byte>(0,255,0),1);//draw gabor 135 line
  //drawLine(itsOutputImage,itsVp,computeBottomPoint(itsVp,maxAngle),PixRGB<byte>(255,0,0),1);//draw min line
  //LINFO("Min Angle %3d : OCR is %3f",(int)maxAngle,max);
  //drawLine(itsOutputImage,itsVp,computeBottomPoint(itsVp,angle+90),PixRGB<byte>(255,255,255),1);
  //      return itsOutputImage;
}

// ######################################################################
Image<PixRGB<byte> > VanishingPointDetector::getOrientationSetDisplay()
{
  uint w = itsImage.getWidth();
  uint h = itsImage.getHeight();
  int row = itsNumOfAngle/4;
  int col = 4;
  if(itsNumOfAngle%col != 0) row++; 
  Image<PixRGB<byte> > osd (w*row,h*col,ZEROS); 
  char buffer[200];
  for(uint angIndex = 0;angIndex < (uint)itsNumOfAngle ;angIndex++){
      	Image<PixRGB<byte> > ori = itsOrientationSet[angIndex];
	sprintf(buffer,"Angle %2.2f",itsAngle*angIndex);
	writeText(ori,Point2D<int>(0,0),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(10));
	inplacePaste(osd, ori, Point2D<int>(w*(angIndex%row), h*(angIndex/row)));

  }
  return osd;
}

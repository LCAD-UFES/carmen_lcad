/*!@file Gist/SuperPixelRoadSegmenter.C Detect road region by
   superpixel */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Gist/SuperPixelRoadSegmenter.C
// $ $Id: $
//
//////////////////////////////////////////////////////////////////////////
// ######################################################################
#include "Gist/SuperPixelRoadSegmenter.H"
#include "Gist/SuperPixel.H"

#include "Raster/Raster.H"
#include "Util/sformat.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"		// for 
#include "Image/ColorOps.H"   // for luminance(),colorize()
#include "Image/ShapeOps.H"   // for rescale()
#include "Image/MathOps.H"    // for stdev()
#include "Image/MatrixOps.H"  // for matrixMult()
#include "Image/CutPaste.H"   // for inplacePaste()
#include "Image/Transforms.H" // for segmentObject()
#include "Image/Normalize.H"	// for normalizeFloat()
#include "Util/Timer.H"
//#include "<sys/stat.h>"		//for system time

#include "Transport/FrameInfo.H"


#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"

#include "GUI/DebugWin.H" // for SHOWIMG()

#include <map>
#include <pthread.h>

#define SEARCH_WINDOW_W 20
#define SEARCH_WINDOW_H 5
#define SEARCH_WINDOW_BOTTOM 1 //1 pixel above image
#define SUPERPIXEL_DOWNSIZE_FACTOR   4
// ######################################################################
SuperPixelRoadSegmenter::SuperPixelRoadSegmenter()
{
	itsFrameID = 0;
	itsProcessedFrameID = 0;
	itsRoadColor = PixRGB<byte>(255,255,255);
  itszNoise = Image<double> (5,5,ZEROS);
  double posVar=10.0;
  double roadColorVar=25.0;
  itszNoise.setVal(0,0,posVar*posVar);
  itszNoise.setVal(1,1,posVar*posVar);
  itszNoise.setVal(2,2,roadColorVar*roadColorVar);
  itszNoise.setVal(3,3,roadColorVar*roadColorVar);
  itszNoise.setVal(4,4,roadColorVar*roadColorVar);
  itsRoadColor = PixRGB<byte>(0,0,0); 
  itsMiddlePoint[0] = 0;
  itsMiddlePoint[1] = 0;
  itsMiddlePoint[2] = 0;
  itsMiddlePoint[3] = 0;
  itsMiddlePoint[4] = 0;
  itsRoadColorDiff = 0.0;
  itsEstMiddlePoint = Point2D<int>(0,0);
  itsRoadColorDiffSub = PixRGB<byte>(0,0,0);
	itsUseFloatWindow = false;
	itsDebugWinCounter = 0;

}
// ######################################################################
SuperPixelRoadSegmenter::~SuperPixelRoadSegmenter()
{

}

// ######################################################################
SuperPixelRoadSegmenter::SuperPixelRoadSegmenter(Image<PixRGB<byte> >ima)
{
	itsImage = ima;
	itsFanImg = ima;
	itsRoadColor = PixRGB<byte>(255,255,255);
}
// ######################################################################
void SuperPixelRoadSegmenter::updateRoadColor(PixRGB<byte> roadColor)
{
	if(roadColor == PixRGB<byte>(0,0,0)) return; //don't update road, 
	itsRoadColor = roadColor;
	LINFO("road color is %d %d %d",roadColor.red(),roadColor.green(),roadColor.blue());
}
// ######################################################################
void SuperPixelRoadSegmenter::updateImage(Image<PixRGB<byte> >ima)
{
	if(!ima.initialized()) return;
	itsDebugWinCounter = 0;
	LINFO("Image width %d height %d",ima.getWidth(),ima.getHeight());
  Timer timer(1000000); float time,total_time;
  timer.reset();
  time = timer.get()/1000.0F;
  total_time = timer.get()/1000.0F;
	//Dims dims = ima.getDims();
	int w = ima.getWidth();
	int h = ima.getHeight();

	//LINFO("create image");
  //itsImage(Image<PixRGB<byte> > (w,h,ZEROS));

	//LINFO("Creat debug image");
 	//itsImage = ima;
	//debugWin(ima,"Image Input");
  //itsImage = Image<PixRGB<byte> > (ima.getDims(),ZEROS);
	

  itsFrameID++; // don't increment itsProcessedFrameID until we compute all features
//	itsSpf.predictState();
//	itsSpf.predictObservation(itszNoise);

	//LINFO("rescale image");
	//80x60
  Image<PixRGB<byte> > downSizeImg  = rescale(ima,80,60);  

	//LINFO("scale image back");
  itsSuperPixelMap = rescale(getSuperPixel(downSizeImg), w,h);
  LINFO("Segment time: %f", timer.get()/1000.0F - time);
  time = timer.get()/1000.0F;
	itsRoadFindingMap = findRoad();
  LINFO("Find Road time: %f", timer.get()/1000.0F - time);
  LINFO(">>>>Total time: %f", timer.get()/1000.0F - total_time);
	itsProcessedFrameID ++;

	Point2D<int> vp(160,120);
	Point2D<int> lp(150,230);
	Point2D<int> rp(170,230);

	PixRGB<byte> rc = getFanAreaColor(vp,lp,rp,ima);
	itsFanImg = drawFanArea(vp,lp,rp,ima,rc);
}



// ######################################################################
Image<PixRGB<byte> > SuperPixelRoadSegmenter::getSuperPixel(Image<PixRGB<byte> > img)
{
  if(!img.initialized()) return Image<PixRGB<byte> >(320,240,ZEROS);

  // default parameters for the Superpixel segmentation
  float sigma = .5; uint k = 400; uint minSize = 100;
  int num_ccs;
  std::vector<std::vector<Point2D<int> > > groups;
  Image<int> groupImage = 
    SuperPixelSegment(img,sigma, k, minSize, num_ccs, &groups);
  Image<PixRGB<byte> > sp_img = SuperPixelDebugImage(groups,img);
  Image<int> sp_size_img = SuperPixelRegionSizeImage(groups,groupImage);
  itsRawSuperPixelImg = sp_img;
	//debugWin(itsRawSuperPixelImg,"itsRawSuperPixelImg");
  int w = sp_img.getWidth();
  int h = sp_img.getHeight();

  // Look for road color, 
  // let's assume it always show up in the middle bottom (h-5,w/2 +- 10)

  std::vector<PixRGB<byte> >	color_map;
  std::vector<int>	color_size_map;
  std::vector<int> 	color_map_count;

  // Pick all road pixel candidates 
  int windowL = -SEARCH_WINDOW_W/2; //-10
  int windowR =  SEARCH_WINDOW_W/2; // 10
  int windowB =  SEARCH_WINDOW_BOTTOM;
  int windowT =  SEARCH_WINDOW_H;
  for(int i = windowL; i<= windowR;i++)
    {
      //We are search bottom area as most likely road pixel candidates
      for(int k = windowB; k <=windowT;k++){

        //set our grow window float to the middle of road
        int middlePoint;
        if(itsMiddlePoint[k-1]!=0 && itsUseFloatWindow){
          middlePoint = itsMiddlePoint[k-1]/4;//1/4 size image
          //LINFO("Float Window %d midpoint %d",middlePoint,k-1);
        }else{

          middlePoint = w/2;
          //LINFO("Fixed Window %d midpoint",middlePoint);
        }
        if(sp_img.coordsOk(middlePoint+i,h-k)){
          PixRGB<byte> tmp_color = sp_img.getVal(middlePoint + i,h-k);
          int regionSize = sp_size_img.getVal(middlePoint+i,h-k);

          bool notfound = true;

          // Search color
          for(int j = 0; j < (int)color_map.size() && notfound ; j++)
            {
              if(color_map[j] == tmp_color)
                {
                  notfound = false;
                  color_map_count[j]++;
                }
            }
          if(notfound)
            {
              color_map.push_back(tmp_color);		
              color_map_count.push_back(0);
              color_size_map.push_back(regionSize);
            }
        }
      }	
    }

  if(color_map.size() > 1)
    {//if we found more than one color
      //Some Option Here:
      //1.Choose max count color
      //2.Pick min color difference from previous avg road color pixel

      //if road color is not available, we pick max pixel color
      if(itsRoadColor == PixRGB<byte>(0,0,0)){	
        int max = color_map_count[0];
        int max_index = 0;
        for(int i = 1; i < (int)color_map_count.size() ; i++)
          {
            if(max < color_map_count[i])
              {
                max = color_map_count[i];
                max_index = i;			
              }
          }
        itsRoadColor = color_map[max_index];
        //LINFO("Max count color have count %d",max);
      }else{
        //Pick min color difference color
        int min_index = 0;
        float min = colorDiff(itsRoadColor,color_map[0]);//
        for(int i = 1; i < (int)color_map_count.size() ; i++)
          {
            float cd = colorDiff(itsRoadColor,color_map[i]);
            int rs = color_size_map[i];//region size
            //LINFO("Road Region Size %d",rs);
            if(cd < min && rs > 100)
              {
                min = cd;
                min_index = i;			
              }
          }
        itsRoadColorDiff = colorDiff(itsRoadColor,color_map[min_index]);

        //to prevent jump too much
        if(itsRoadColorDiff < 50.0){
          itsRoadColor = color_map[min_index];
        }else{
          //keep avg color so it will help to solve kid-napping problem	
          PixRGB<byte> avgColor = colorAvg(itsRoadColor,color_map[0],0.8);//first color will have 80% weight
          itsRoadColor = avgColor; 
          //LINFO("COLOR DIFF1 %f",itsRoadColorDiff);
        }
      }
    }
  else
    {
      //if only one region
      itsRoadColorDiff = colorDiff(itsRoadColor,color_map[0]);
      itsRoadColorDiffSub = colorDiffSub(itsRoadColor,color_map[0]);


      if((itsRoadColorDiff < 50.0 && color_map[0].green() > 150)||itsRoadColor == PixRGB<byte>(0,0,0)){//for outdoor concrete road
      //if((itsRoadColorDiff < 90.0 && color_map[0].green()<150)||itsRoadColor == PixRGB<byte>(0,0,0)){//indoor
      //if((itsRoadColorDiff < 90.0)||itsRoadColor == PixRGB<byte>(0,0,0)){//general, high fail rate
        itsRoadColor = color_map[0];
        //itsUseFloatWindow = false;//FIXXXXXX
      }else{
        PixRGB<byte> avgColor = colorAvg(itsRoadColor,color_map[0],0.8);//80% on first one
        itsRoadColor = avgColor; 
        itsUseFloatWindow = true;
        //LINFO("COLOR DIFF2 %f,USE float window",itsRoadColorDiff);
      }
      //LINFO("Only one color (%d,%d,%d)",itsRoadColor.red(),itsRoadColor.green(),itsRoadColor.blue());
    }
 
  // use iterator!!!!!!
  Image<PixRGB<byte> > output(w,h,ZEROS); //image with full red road region 
  Image<PixRGB<byte> > hybrid(w,h,ZEROS); //image with red grid dot road region 
	itsRoadIndexMap = Image<int>(w, h, ZEROS);
  //	inplacePaste(output, sp_img, Point2D<int>(w, 0));
  for(int y = 0 ; y < h ; y ++)
    {
      int dot = 0;
      for(int x = 0 ; x < w ; x ++)
        {
          PixRGB<byte> c = sp_img.getVal(x,y);
          //LINFO("Pixel color(%d,%d,%d)road color(%d,%d,%d)",c.red(),c.green(),c.blue(),
          //		itsRoadColor.red(),itsRoadColor.green(),itsRoadColor.blue());
          if(c == itsRoadColor)	{
            //Set road color to red
						byte red = 250+c.red(); 
						if(red > 255) red = 255;
            //output.setVal(x,y,PixRGB<byte>(red,c.green()/32,c.blue()/32));//this will mess up find red region
            output.setVal(x,y,PixRGB<byte>(255,0,0));
            itsRoadIndexMap.setVal(x,y,255);
            if(dot % 3 == 0){
              hybrid.setVal(x,y,PixRGB<byte>(255,0,0));
            }else{
              hybrid.setVal(x,y,c);
            }
            dot ++;
          }
          else{
            //set to it's color
            output.setVal(x,y,c);
            hybrid.setVal(x,y,c);
						itsRoadIndexMap.setVal(x,y,0);
            //					output.setVal(x,y,c);
          }
        }
    }
	LINFO("hi11");
  if(!output.initialized())
    return img;

	LINFO("hi12");
  //LINFO("Finish Road Finding");
  itsRawRoadSuperPixelImg = hybrid;
  return output;
}

// ######################################################################
Image<PixRGB<byte> > SuperPixelRoadSegmenter::findRoad()
{
	if(!itsSuperPixelMap.initialized()) return itsSuperPixelMap;

	LINFO("hi13");
	//debugWin(itsSuperPixelMap,"itsSuperPixelMap");
	LINFO("hi14");

	// display
	int w = itsSuperPixelMap.getWidth();
	int h = itsSuperPixelMap.getHeight();
	//Dims dims = itsSuperPixelMap.getDims();
	itsDispImg.resize(w*3, 3*h, ZEROS);
	Image<PixRGB<byte> > roadFindingMap(w, h, ZEROS);
	//LINFO("superpixel size1 w %d h %d",itsSuperPixelMap.getWidth(),itsSuperPixelMap.getHeight());
	// Estimate Middle of Road
	LINFO("hi15");
	if(itsSuperPixelMap.initialized())
	{
		//LINFO("Find road color");
		std::vector<Point2D<int> > points;

		LINFO("hi16");
		for(int y = 0 ; y < itsSuperPixelMap.getHeight() ; y ++)
		{
			int middle = 0, pixCount = 0;
			for(int x = 0 ; x < itsSuperPixelMap.getWidth() ; x ++)
			{
				PixRGB<byte> s = itsSuperPixelMap.getVal(x,y);

				//Find avg of middle pixel
				if(s.red()==255 && s.green()==0 && s.blue()==0)
				{
					middle+=x; pixCount++;
				}
				roadFindingMap.setVal(x,y,s);//copy pixel from itsSuperPixelMap
			}
			if(pixCount!=0)
			{
				int midx = (int)middle/pixCount;
				roadFindingMap.setVal(midx,y,PixRGB<byte>(255,255,0));//draw yellow point in the middle line
				drawCircle(roadFindingMap,Point2D<int>(midx,y),2,PixRGB<byte>(255,255,0),1);//Make line thicker
				//Only use bottom 20 pixel for middle line,which is just right front the robot
				if(y > h-21 && y < h-5)
					points.push_back(Point2D<int>(midx,y));

				if(y > h-5)
					itsMiddlePoint[h-y] = midx;
			}
		}//end for
		LINFO("hi17");
		//debugWin(roadFindingMap,"finish copy sp");//ok
		//LINFO("Do Middle Line finder, point size %d",(int)points.size());
		//roadFindingMap = getGroundTruthData(roadFindingMap);//FIXXXX

//		drawLine
//			(roadFindingMap,Point2D<int>(w/2,0),
//			 Point2D<int>(w/2,h),PixRGB<byte>(0,255,0),1);//Green line
			//debugWin(roadFindingMap,"Road Finding Map after green drawLine");

		LINFO("hi18");
		if(points.size() > 1)
		{
			Point2D<int> p1,p2;
			fitLine(points,p1,p2);
			//char buffer[20];

			//LINFO("Compute Navigation Error");
			//Compute Navigation Error
			itsEstMiddlePoint = Point2D<int>((p1.i+p2.i)/2,h-8);
			itsHighMiddlePoint = p1;

			int currMd= itsEstMiddlePoint.i;
			int rf = itsRoadColor.red();
			int gf = itsRoadColor.green();
			int bf = itsRoadColor.blue();
			LINFO("Current SuperPixel Road Middle Point %d Color (%d,%d,%d)",currMd,rf,gf,bf);


			drawLine(roadFindingMap,p1,p2,PixRGB<byte>(255,255,0),3);//Yellow Middle line
			//debugWin(roadFindingMap,"Road Finding Map after yellow drawLine");

			LINFO("hi19");
			//LINFO("Compute Navigation Error Done");
		}//extra,remove it later
		LINFO("hi20");
		inplacePaste(itsDispImg,roadFindingMap, Point2D<int>(w, 0));
	}
	LINFO("hi24");
	//debugWin(roadFindingMap,"Road Finding Map");
	LINFO("hi25");
	return roadFindingMap;
}

// ######################################################################
PixRGB<byte> SuperPixelRoadSegmenter::getFanAreaColor(Point2D<int> topPoint,Point2D<int> leftPoint,Point2D<int> rightPoint,
								Image<PixRGB<byte> > img)
{
  Point2D<int> p1 = topPoint; 
  Point2D<int> p2 = leftPoint; 
  Point2D<int> p3 = rightPoint; 
  int w = img.getWidth();
  int h = img.getHeight();	
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
	  if(img.coordsOk(x,y)){
	    avgColor += img.getVal(x,y);
	    pixCount++;
	  }
	}
    }
  if(pixCount != 0) avgColor/=pixCount;
  return (PixRGB<byte>)avgColor;
	
}

// ######################################################################
Image<PixRGB<byte> >SuperPixelRoadSegmenter::drawFanArea(
	Point2D<int> topPoint,Point2D<int> leftPoint,Point2D<int> rightPoint,
	Image<PixRGB<byte> > img, PixRGB<byte> color)
{
	Point2D<int> p1 = topPoint;
	Point2D<int> p2 = leftPoint;
	Point2D<int> p3 = rightPoint;
	int w = img.getWidth();
	int h = img.getHeight();

	Image<PixRGB<byte> > output(img);
	
	for(int y = p1.j; y < h;y++)
	{
		Point2D<int> it1 = intersectPoint(p1,p2,Point2D<int>(0,y),Point2D<int>(w,y));
		Point2D<int> it2 = intersectPoint(p1,p3,Point2D<int>(0,y),Point2D<int>(w,y));	
		int length = abs(it1.i - it2.i);
		int x_init = (it1.i < it2.i) ? it1.i : it2.i; 
		for(int x = x_init; x < length+x_init;x++){
			if(output.coordsOk(x,y)){
				output.setVal(x,y,color);
			}
		}
	}
	return output;


}
// ######################################################################
Image<PixRGB<byte> > SuperPixelRoadSegmenter::getOutputImage()
{
			return itsDispImg;
}
// ######################################################################
void SuperPixelRoadSegmenter::debugWin(Image<PixRGB<byte> >disp, std::string title)
{
	//rutz::shared_ptr<XWinManaged> itsWin;
	//limit max window to 10
	if(itsDebugWinCounter < 10) itsDebugWinCounter ++;
	 std::string t = std::string(sformat("%s %d",title.c_str(),itsDebugWinCounter));

	 rutz::shared_ptr<XWinManaged> win;
	 //if we don't have enough initialized window
	 if(itsWinVec.size() < itsDebugWinCounter)
	 {
	   win.reset(new XWinManaged(disp.getDims(), 0, 0, t.c_str()));
		 itsWinVec.push_back(win);
		 
	 }else{		 
		 win.reset(new XWinManaged(disp.getDims(), 0, 0, t.c_str()));
		 win = itsWinVec[itsDebugWinCounter-1];		 
	 }

	 win->drawImage(disp,0,0);

	// for(uint ti = 0; ti < rTemplate.size(); ti++)
	//   disp2.setVal(rTemplate[ti], 1.0F);
	// Raster::waitForKey();   
}	


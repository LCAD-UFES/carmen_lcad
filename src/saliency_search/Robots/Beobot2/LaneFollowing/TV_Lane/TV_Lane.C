/*!@file Robots2/Beobot2/LaneFollowing/TV_Lane/TV_Lane.C */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/TV_Lane.C
// $ $Id: TV_Lane.C 13084 2010-03-30 02:42:00Z kai $
//
//////////////////////////////////////////////////////////////////////////
//#include "Image/OpenCVUtil.H"
#include "Robots/Beobot2/LaneFollowing/TV_Lane/TV_Lane.H"

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
#include "Ice/BeobotEvents.ice.H"


#include "SIFT/Keypoint.H"
#include "SIFT/VisualObject.H"
#include "SIFT/VisualObjectMatch.H"
#include "Transport/FrameInfo.H"

#include "Ice/IceImageUtils.H"

#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"

#include "GUI/DebugWin.H" // for SHOWIMG()

#include <map>
#define FOLDER 	"../data/logs"
//#define DBNAME	"Lane"
#define DBNAME	"outdoor"
#define IMAGE_PREFIX 	"image_0000000000"
#define WINDOW_SIZE 5
// ######################################################################
TV_Lane::TV_Lane(OptionManager& mgr,
                 const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsSmt(new SaliencyMT(mgr,descrName,tagName)),
  itsOfs(new OutputFrameSeries(mgr)),
  itsIfs(new InputFrameSeries(mgr)),
  itsKf(80.0,30.0),
  itsKfx(0),
  itsTimer(1000000)
{
  setModelParamVal("InputFameDims",Dims(320,240),MC_RECURSE|MC_IGNORE_MISSING);
  addSubComponent(itsSmt);
  
  itsOfs->addFrameDest("display");//Add default --out=display
  addSubComponent(itsOfs);
  
	addSubComponent(itsIfs);
}

// ######################################################################

// ######################################################################
TV_Lane::~TV_Lane()
{ }

// ######################################################################
void TV_Lane::start1()
{

}

// ######################################################################
void TV_Lane::registerTopics()
{
  // subscribe to all sensor data
  this->registerSubscription("CameraMessageTopic");
}

// ######################################################################
void TV_Lane::evolve()
{
  // if simulation
  drawState();	
  itsOfs->writeRGB(itsDispImg, "display", FrameInfo("TV_nav",SRC_POS));
  //itsOfs->writeRGB(itsDispImg, "TV_nav", FrameInfo("TV_nav",SRC_POS));
}
// ######################################################################
Image<PixRGB<byte> > TV_Lane::getSuperPixel(Image<PixRGB<byte> > img)
{
  if(!img.initialized())
    return Image<PixRGB<byte> >(320,240,ZEROS);
  // default parameters for the Superpixel segmentation
  float sigma = .5; uint k = 500; uint minSize = 20;
  int num_ccs;
  Image<PixRGB<byte> > sp_img = segment_image(img,sigma,k,minSize,num_ccs);
  
  int w = sp_img.getWidth();
  int h = sp_img.getHeight();
  
  //Look for road color, let's assume it always show up in the middle bottom
  //(h-5,w/2 +- 10)
  
  std::vector<PixRGB<byte> >	color_map;
  std::vector<int> 	color_map_count;
  
  PixRGB<byte> roadColor;
  for(int i = -10; i<= 10;i++)
    {
      PixRGB<byte> tmp_color = sp_img.getVal((w/2) + i,h-5);
      
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
          //This color is not in the map
          color_map.push_back(tmp_color);		
          color_map_count.push_back(0);
        }	
    }

  if(color_map.size() != 1)
    {//if we found more than one color
      //Choose max count color
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
      roadColor = color_map[max_index];
      LINFO("Max count color have count %d",max);
    }
  else
    {
      roadColor = color_map[0];
      LINFO("Only one color (%d,%d,%d)",roadColor.red(),roadColor.green(),roadColor.blue());
    }

  // use iterator!!!!!!
  Image<PixRGB<byte> > output(w,h,ZEROS);  
  for(int y = 0 ; y < h ; y ++)
    {
      for(int x = 0 ; x < w ; x ++)
        {
          PixRGB<byte> c = sp_img.getVal(x,y);
          //LINFO("Pixel color(%d,%d,%d)road color(%d,%d,%d)",c.red(),c.green(),c.blue(),
          //		roadColor.red(),roadColor.green(),roadColor.blue());
          if(c == roadColor)	{
            
            //Set road color to red
            output.setVal(x,y,PixRGB<byte>(255,0,0));
            //LINFO("Found road pixel============");

          }
          else{
            output.setVal(x,y,PixRGB<byte>(0,0,0));
            //					output.setVal(x,y,c);
            
          }

        }
    }
  if(!output.initialized())
    return img;
  return output;
  


}
// ######################################################################

int TV_Lane::getVotingCount(Image<PixRGB<byte> > img,int topX,int topY,int windowSize,PixRGB<byte> bgColor)
{
	if(!img.initialized())
		return 0;

	int w = img.getWidth();
	int h = img.getHeight();
	int count = 0;
	for(int i = topX ; i < topX + windowSize ; i++){
		for(int j = topY ; j < topY + windowSize ; j++){

			if(i < w && j < h){
				PixRGB<byte> tmp_color = img.getVal(i,j);
				if(tmp_color != bgColor)
					count++;
			}
		}
	}
	return count;

}
// ######################################################################
PixRGB<byte> TV_Lane::getBackgroundColor(Image<PixRGB<byte> >img)
{

	int w = img.getWidth();
	int h = img.getHeight();

	std::vector<PixRGB<byte> >	color_map;
	std::vector<int> 	color_map_count;
	for(int i = 0; i < w; i++){
		for(int j = 0 ; j < h; j++){

			if(i < w && j < h){
				PixRGB<byte> tmp_color = img.getVal(i,j);
				bool notfound = true;

				// Search color
				for(int k = 0; k < (int)color_map.size() && notfound ; k++)
				{
					if(color_map[k] == tmp_color)
					{
						notfound = false;
						color_map_count[k]++;
					}
				}
				if(notfound)
				{
					//This color is not in the map
					color_map.push_back(tmp_color);		
					color_map_count.push_back(0);
				}	
			}
		}
	}
	PixRGB<byte> bgColor;
	if(color_map.size() != 1){//if we found more than on color
		//Choose max count color
		int max = color_map_count[0];
		int max_index = 0;
		for(int i = 1; i < (int)color_map_count.size() ; i++){
			if(max < color_map_count[i]){
				max = color_map_count[i];
				max_index = i;			
			}
		}
		bgColor= color_map[max_index];
		LINFO("Max count color have count %d",max);
	}else{
		bgColor	= color_map[0];
	}

	return bgColor;


}
//Doing Canny Edge 
Image<PixRGB<byte> > TV_Lane::getCannyEdge(Image<PixRGB<byte> > img,Image<PixRGB<byte> > &rawCannyImg){

	//Doing Canny Edge 
	//img = itsCurrImg;
	IplImage *cvImg = cvCreateImage(cvGetSize(img2ipl(luminance(img))),8,1);
	//IplImage *dst = cvCloneImage(cvImg);
	IplConvKernel* element = 0;
	element = cvCreateStructuringElementEx(2,2,1,1,CV_SHAPE_ELLIPSE,0);
	cvErode(img2ipl(luminance(img)),cvImg,element,3);
	cvDilate(img2ipl(luminance(img)),cvImg,element,1);
	CvMemStorage *s = cvCreateMemStorage(0);
	CvSeq *lines = 0;

	//cvCanny(img2ipl(luminance(img)),cvImg,100,110,3);
	//Image<PixRGB<byte> > edgeImg = toRGB(ipl2gray(cvImg)); 
	Image<PixRGB<byte> > edgeImg = toRGB(ipl2gray(cvImg)); 
	return edgeImg;
	rawCannyImg = edgeImg;

	lines = cvHoughLines2(cvImg,s,CV_HOUGH_PROBABILISTIC,1,CV_PI/180,80,30,10);

	int lx1 = 0,lx2 = 0,ly1 = 0,ly2 = 0,rx1 = 0,rx2 =0 ,ry1 = 0,ry2 = 0;
	LINFO("Total Line %d",lines->total);
	for(int i = 0;i<MIN(lines->total,100);i++)
	{
		CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
		Point2D<int> pt1 = Point2D<int>(line[0].x,line[0].y);
		Point2D<int> pt2 = Point2D<int>(line[1].x,line[1].y);
		int dx = pt2.i - pt1.i;
		int dy = pt2.j - pt1.j;
		int midx = (pt1.i+pt2.i)/2;
		int midy = (pt1.j+pt2.j)/2;
		float slope = 0;
		if(dx*dy != 0)
			slope = dy/dx;	
		PixRGB<byte > line_color;
		if(slope == 0.0)
			line_color = PixRGB<byte>(200,0,0);//Dark Red
		else if(slope < 0.0 && slope > -10.0)	
		{
			line_color = PixRGB<byte>(128,128,200);//Light blue
			LDEBUG("Left edge Road!!!! %f",slope);
			lx1 = pt2.i - (pt2.j*slope);
			ly1 = 0;
			lx2 = pt2.i -(pt2.j-240)*slope;
			ly2 = 240;
		}
		else if(slope <= -10.0 )	
			line_color = PixRGB<byte>(0,128,0);//Green
		else if(slope > 0.0 && slope < 10.0)	
		{
			line_color = PixRGB<byte>(0,255,0);//light green
			rx1 = pt2.i - pt2.j*slope;
			ry1 = 0;
			rx2 = pt2.i -(pt2.j-240)*slope;
			ry2 = 240;
		}
		else {
			line_color = PixRGB<byte>(0,0,128);//blue
		}
		if(slope != 0.0)
		{
			drawLine(edgeImg,pt1,pt2,line_color,2);
			char buffer[128];
			sprintf(buffer,"%1.2f",slope);
			writeText(edgeImg,Point2D<int>(midx,midy),buffer,line_color,PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
		}//end slope != 0.0

		//find center line, using Kalman filter
		//int midx1 = (lx1+rx1)/2;
		//int midx2 = (lx2+rx2)/2;
		//int mid_dx = midx1 -midx2;
		/*
			 if(midx1 != 0){
			 float fkfx = itsKf.update(midx1*1.0);
		//LINFO("update kalman filter old_kfx[%d],measure[%d],new_kfx[%f]",itsKfx,midx1,fkfx);

		itsKfx = (int)fkfx;
		}
		//LINFO("Slope %f",slope);
		drawLine(edgeImg,Point2D<int>(itsKfx,0),Point2D<int>(itsKfx,240),PixRGB<byte>(255,255,0),2);//Yellow line
		*/

	}//end for loop
	return edgeImg;	


}

// ######################################################################
Image<float> TV_Lane::getVotingBlock(Image<PixRGB<byte> > img)
{
  if(!img.initialized())
    return Image<float>(320,240,ZEROS);
  // default parameters for the Superpixel segmentation
  
  int w = img.getWidth();
  int h = img.getHeight();
	PixRGB<byte> bgColor = getBackgroundColor(img);
  Image<float>  vote_map(w/WINDOW_SIZE + 1,h/WINDOW_SIZE +1,ZEROS);  
	char buffer[200];
   for(int y = 0 ; y < h ; y += WINDOW_SIZE)
    {
      for(int x = 0 ; x < w ; x += WINDOW_SIZE)
        {
					int count = getVotingCount(img,x,y,WINDOW_SIZE,bgColor);
					sprintf(buffer,"%2d",count/10);
					//if(count > 1)
					//	writeText(itsCurrImg,Point2D<int>(x,y),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
					int ww = WINDOW_SIZE*WINDOW_SIZE;
					float weight = (ww-count);
					//LINFO("Count:(%d,%d) %d weight %f",x,y,count,weight);
					if(x < w && y < h)
						vote_map.setVal(x/WINDOW_SIZE,y/WINDOW_SIZE,weight);
					
					/*
					for(int i = 0;i< WINDOW_SIZE;i++){
						for(int j = 0;j< WINDOW_SIZE;j++){
							if(x+i<w && y+j<h)
								vote_map.setVal(x+i,y+j,weight);
						}
					}
					*/


        }
    } 

  // use iterator!!!!!!
  Image<PixRGB<byte> > output(w,h,ZEROS);  
  for(int y = 0 ; y < h ; y ++)
    {
      for(int x = 0 ; x < w ; x ++)
        {

        }
    }
  return vote_map;
  


}

// ######################################################################
Image<float> TV_Lane::updateVoteBlock(Image<float> map)
{
	int w = map.getWidth();
	int h = map.getHeight();
	LINFO("Do it w,h = %d %d",w,h);
	Image<float>  new_map(w,h,ZEROS);  
	int ww = WINDOW_SIZE*WINDOW_SIZE;
	for(int y = 0 ; y < h ; y ++)
	{
		for(int x = 0 ; x < w ; x ++ )
		{
			float root_weight = map.getVal(x,y);
			//float newWeight;
			float left_w = 0,right_w = 0,up_w = 0,down_w = 0;
			//update neighborhood value
			if(x-1 > 0)
				left_w = map.getVal(x-1,y);
			if(x+1 < w)
				right_w = map.getVal(x+1,y);
			if(y-1 > 0)
				up_w = map.getVal(x,y-1);
			if(y+1 < h)
				down_w = map.getVal(x,y+1);
			
			//float fill = 0.2;
			//if black-white-black patten appear
			if(root_weight > ww/2)//if block is potential road block
			{
				bool keepgoing = true;
				int length = 0;
				std::vector<Point2D<int> >line;
				int y2 = y;
				while(keepgoing)
				{
					//look down five block
					if(y2+1 < h)
					{
						keepgoing = false;
						for(int child = 0;child < 1;child++)
						{
							if(x+child > 0 && x+child < w)
							{
								float childWeight = map.getVal(x+child,y2+1);
								if(childWeight >= root_weight)
								{
									if(length >60)
										LINFO("(%d,%d) weight %f count %d",x+child,y2+1,childWeight,length);
									if(!keepgoing)//first child found
										length++;
									keepgoing = true;
									line.push_back(Point2D<int>(x+child,y2+1));
									new_map.setVal(x+child,y2+1,childWeight);	
								
								}//found child
							
							}//child in range
						
						}//for 6 child
						y2++;
					}else{
						keepgoing = false;
					}
				
				}//while keep going
				if(length > 70){
					LINFO("find line length:%d",length);

					length = 0;
				}
			}//if block is potential road block

/*
			if(root_weight > ww/2 &&left_w < ww*fill && right_w < ww*fill)
			{
				newWeight = (left_w+right_w)/2; 
				new_map.setVal(x,y,newWeight);
				LINFO("L:%f R:%f C:%f N:%f",left_w,right_w,root_weight,newWeight);
			}else{
				new_map.setVal(x,y,root_weight);
			
			}

			//if black-white-black patten appear
			if(root_weight > ww/2 &&up_w < ww*fill && down_w < ww*fill)
			{
				newWeight = (up_w+down_w)/2; 
				new_map.setVal(x,y,newWeight);
			}
*/
/*
			for(int xx = -1; xx <= 1; xx++)
			{
				for(int yy = -1; yy <= 1; yy++)
				{
					int neighborX = x+xx;
					int neighborY = y+yy;

					if(neighborX < w && neighborX >= 0 && neighborY < h && neighborY >=0)
					{
							double distance = sqrt(xx*xx + yy*yy);
							double normalScale = getNormalWeight(distance);
							double neighborWeight = map.getVal(neighborX,neighborY);
							double newAddWeight = normalScale*root_weight*0.05;
							double newWeight = 0;
						if(xx !=0 || yy !=0)//skip itself
						{
							newWeight = (neighborWeight + newAddWeight);
						}else
						{
							newWeight = root_weight;				
						}
						if(newWeight > 255)
							newWeight = 255;

							if(x%10 == 0 && y%10 == 0)
							{
								//LINFO("(%d,%d) + (%d,%d) rw %f nbw %f new %f newAdd %f ns %f",x,y,xx,yy,root_weight, neighborWeight,newWeight,newAddWeight,normalScale);
							}
								for(int i = 0;i< WINDOW_SIZE;i++){
								for(int j = 0;j< WINDOW_SIZE;j++){
									if(neighborX + i < w && neighborY+j < h)
										new_map.setVal(neighborX+i,neighborY+j,newWeight);
								}
							}
						
					}//if
					
				}//for yy
			}//for xx
*/
		}//for x 
	}//for y
	return new_map;
}	
// ######################################################################
void TV_Lane::drawState()
{
		Image<PixRGB<byte> > voteImg;  
		int w=0,h=0;
  if(!itsCurrImg.initialized()){
		itsCurrImg = itsIfs->readRGB();

		itsVoteMap = getVotingBlock(itsCurrImg);
		voteImg = normalizeFloat(itsVoteMap,FLOAT_NORM_0_255);
		w = itsCurrImg.getWidth()/2;
		h = itsCurrImg.getHeight()/2;
		itsDispImg.resize(w*4,h,ZEROS);

		inplacePaste(itsDispImg,rescale(voteImg,w,h),Point2D<int>(w,0));

		itsVoteMap = updateVoteBlock(itsVoteMap);
		voteImg = normalizeFloat(itsVoteMap,FLOAT_NORM_0_255);
		inplacePaste(itsDispImg,rescale(voteImg,w,h),Point2D<int>(w*2,0));

		Image<PixRGB<byte> > cannyImg;
		cannyImg = getCannyEdge(voteImg,cannyImg);

		inplacePaste(itsDispImg, rescale(itsCurrImg,w,h), Point2D<int>(0, 0));
		inplacePaste(itsDispImg,rescale(cannyImg,w,h),Point2D<int>(w*3,0));

	}else{
		w = itsCurrImg.getWidth()/2;
		h = itsCurrImg.getHeight()/2;
		
	}

	
	//itsVoteMap = updateVoteBlock(itsVoteMap);
  Raster::waitForKey();
	itsVoteMap = updateVoteBlock(itsVoteMap);
	LINFO("Do Once1");
	voteImg = normalizeFloat(itsVoteMap,FLOAT_NORM_0_255);
	LINFO("Do Once2");
	inplacePaste(itsDispImg,rescale(voteImg,w,h),Point2D<int>(w*2,0));
	LINFO("Do Once3");

	//itsDispImg.resize(itsCurrImg.getDims());
	//LINFO("Hi");
	//inplacePaste(itsDispImg, itsCurrImg, Point2D<int>(0, 0));

	//LINFO("Do Canny");
	// Do canny edge 
	// Segment the regions in the image 
	// using Superpixel algorithm (Felzenszwalb & Huttenlocher 2003) 

	// resize to original for display only


	//inplacePaste(itsDispImg,getSaliency(itsCurrImg), Point2D<int>(0, h));
}


// ######################################################################
void TV_Lane:: updateMessage 
(const RobotSimEvents::EventMessagePtr& eMsg, const Ice::Current&)
{
  // camera message
  if(eMsg->ice_isA("::BeobotEvents::CameraMessage"))
    {
      // store the image
      BeobotEvents::CameraMessagePtr cameraMsg =
        BeobotEvents::CameraMessagePtr::dynamicCast(eMsg);
      
      int currRequestID = cameraMsg->RequestID;
      Image<PixRGB<byte> > img = Ice2Image<PixRGB<byte> >(cameraMsg->image);
      
      LDEBUG("Got a CameraMessage with Request ID = %d", currRequestID);
                
      its_Curr_Img_mutex.lock();
      itsCurrImg = img;
      itsCurrImgID = cameraMsg->RequestID;
      its_Curr_Img_mutex.unlock();
    }
  
  // motor message
  else if(eMsg->ice_isA("::BeobotEvents::MotorMessage"))
    {
		BeobotEvents::MotorMessagePtr mtrMsg =
                  BeobotEvents::MotorMessagePtr::dynamicCast(eMsg);
		LDEBUG("Got a MotorMessage with Request ID = %d: RC Trans %f, Rot %f",
                       mtrMsg->RequestID, itsRcTransSpeed, itsRcRotSpeed);
		its_Curr_Mtr_mutex.lock();
		itsRemoteMode = mtrMsg->rcMode;
		itsRcTransSpeed = mtrMsg->rcTransVel;
		itsRcRotSpeed = mtrMsg->rcRotVel;
		its_Curr_Mtr_mutex.unlock();
	}
	//        LINFO("updateMessage");


}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

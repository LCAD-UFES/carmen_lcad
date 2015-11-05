/*!@file Robots2/Beobot2/LaneFollowing/RG_Lane/RG_Lane.C */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/RG_Lane.C
// $ $Id: RG_Lane.C 13084 2010-03-30 02:42:00Z kai $
//
//////////////////////////////////////////////////////////////////////////
#include "Robots/Beobot2/LaneFollowing/RG_Lane/RG_Lane.H"

#include "Raster/Raster.H"
#include "Util/sformat.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "Image/ColorOps.H"   // for luminance()
#include "Image/ShapeOps.H"   // for rescale()
#include "Image/MathOps.H"    // for stdev()
#include "Image/MatrixOps.H"  // for matrixMult()
#include "Image/CutPaste.H"   // for inplacePaste()
#include "Image/Transforms.H" // for segmentObject()
#include "Image/OpenCVUtil.H"
#include "Util/Timer.H"

#include "SIFT/Keypoint.H"
#include "SIFT/VisualObject.H"
#include "SIFT/VisualObjectMatch.H"
#include "Transport/FrameInfo.H"

#include "Ice/IceImageUtils.H"

#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"

#include "GUI/DebugWin.H" // for SHOWIMG()

#define FOLDER 	"../data/logs"
#define DBNAME	"Lane"
#define IMAGE_PREFIX 	"image_0000000000"

// ######################################################################
RG_Lane::RG_Lane(OptionManager& mgr,
                             const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsOfs(new OutputFrameSeries(mgr)),
	itsKf(80.0,30.0),
	itsKfx(0),
  itsTimer(1000000)
{
	itsOfs->addFrameDest("display");//Add default --out=display
  addSubComponent(itsOfs);

  // load image db
	LINFO("load db");
  openDB(sformat("%s/%s/%s.conf",FOLDER,DBNAME,DBNAME));
		
}

// ######################################################################
void RG_Lane::openDB(const std::string& path)
{
	FILE *dbconf = fopen(path.c_str(),"r");
	if(dbconf == NULL){
		LFATAL("Can not open file: %s",path.c_str());
	}else{
		char line[512];
			while(fgets(line,sizeof line,dbconf)!= NULL)
			{
				LINFO("Got line[%s]",line);
				//Skip the line start at#
				if(line[0] != '#')
				{
					char subdb[256];
					int start,end;
					int ret = sscanf(line,"%s %d %d",subdb,&start,&end);
					if(ret == 3){
						LINFO("Got [%s],start[%d],end[%d]",subdb,start,end);
						std::string path = sformat("%s/%s/%s/%s",FOLDER,DBNAME,subdb,IMAGE_PREFIX);
						imageDB tmpDB(path,start,end);
						itsImageDB = tmpDB;
					}
				}			
			}
	}
}

// ######################################################################
RG_Lane::~RG_Lane()
{ }

// ######################################################################
void RG_Lane::start1()
{ }

// ######################################################################
void RG_Lane::registerTopics()
{
  // subscribe to all sensor data
  //this->registerSubscription("CameraMessageTopic");
}

// ######################################################################
void RG_Lane::evolve()
{
  // if simulation
	loadFrame();
  drawState();	
  itsOfs->writeRGB(itsDispImg, "display", FrameInfo("RG_nav",SRC_POS));
	//itsOfs->writeRGB(itsDispImg, "RG_nav", FrameInfo("RG_nav",SRC_POS));
}


// ######################################################################
void RG_Lane::loadFrame()
{
  // load teach image data
  itsTimer.reset();
			itsCurrImg =  itsImageDB.nextImg();

  LDEBUG("Time for Load %f",
        itsTimer.get()/1000.0);
}
// ######################################################################
void RG_Lane::drawState()
{
  uint w = 320, h = 240;

  itsDispImg.resize(w*2, 2*h, NO_INIT);
  inplacePaste(itsDispImg, itsCurrImg, Point2D<int>(0, 0));

	IplImage *cvImg = cvCreateImage(cvGetSize(img2ipl(itsCurrImg)),8,1);
//	IplImage *cvImgColor = cvCreateImage(cvGetSize(img2ipl(itsCurrImg)),8,3);
	
	CvMemStorage *s = cvCreateMemStorage(0);
	CvSeq *lines = 0;


	cvCanny(img2ipl(luminance(itsCurrImg)),cvImg,100,150,3);
	itsProcImg = toRGB(ipl2gray(cvImg)); 

	inplacePaste(itsDispImg, toRGB(ipl2gray(cvImg)), Point2D<int>(w, 0));
//	cvCvtColor(cvImg,cvImgColor,CV_GRAY2BGR);
	
	lines = cvHoughLines2(cvImg,s,CV_HOUGH_PROBABILISTIC,1,CV_PI/180,80,30,10);
	
	int lx1 = 0,lx2 = 0,ly1 = 0,ly2 = 0,rx1 = 0,rx2 =0 ,ry1 = 0,ry2 = 0;
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
			drawLine(itsProcImg,pt1,pt2,line_color,2);
			char buffer[128];
			sprintf(buffer,"%1.2f",slope);
			writeText(itsProcImg,Point2D<int>(midx,midy),buffer,line_color,PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
		}//end slope != 0.0

		//find center line
			int midx1 = (lx1+rx1)/2;
			//int midx2 = (lx2+rx2)/2;
			//int mid_dx = midx1 -midx2;
			if(midx1 != 0){
				float fkfx = itsKf.update(midx1*1.0);
				LINFO("update kalman filter old_kfx[%d],measure[%d],new_kfx[%f]",itsKfx,midx1,fkfx);

				itsKfx = (int)fkfx;
			}
		//LINFO("Slope %f",slope);
			drawLine(itsProcImg,Point2D<int>(itsKfx,0),Point2D<int>(itsKfx,240),PixRGB<byte>(255,255,0),2);//Yellow line


	}//end for loop
	
	inplacePaste(itsDispImg,itsProcImg, Point2D<int>(0, h));
//SHOWIMG(ipl2rgb(cvImgColor));
//	IplImage *cvImg = cvCreateImage(cvGetSize(img2ipl(itsCurrImg)),8,1);

//	cvPyrSegmentation(img2ipl(itsCurrImg),cvImg,s,&comp,20,40,2);
}
// ######################################################################
void RG_Lane:: updateMessage (const RobotSimEvents::EventMessagePtr& eMsg, const Ice::Current&)
{
LINFO("hi");

}
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

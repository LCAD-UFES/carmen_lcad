/*!@file Robots2/Beobot2/Hardware/BeoLaserCam.C Ice Module for sending out
  images from a camera                                                  */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Hardware/BeoLaserCam.C
// $ $Id: BeoLaserCam.C 15190 2012-02-29 04:00:13Z kai $
//
//////////////////////////////////////////////////////////////////////////

#include "Robots/Beobot2/Hardware/BeoLaserCam.H"
#include "Ice/BeobotEvents.ice.H"
#include "Ice/IceImageUtils.H"
#include "Raster/Raster.H"
#include "Image/DrawOps.H"
#include "Image/Point3D.H"
#include "Image/ShapeOps.H"

#include "GUI/XWinManaged.H"
#include "GUI/ImageDisplayStream.H"
#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"

#include "Devices/FrameGrabberConfigurator.H"
#include "Raster/GenericFrame.H"
#include "Transport/FrameIstream.H"
#include "Util/WorkThreadServer.H"


#include "Util/sformat.H"
#include <sys/stat.h>

#define FRAMERATE_INTERVAL        100
#define FRAMERATE_CALC_INTERVAL   20
#define REPORTING_INTERVAL				100
#define DATA_FOLDER                 "../data/logs/laserCam"
#define NONE_FOLDER_NAME           "NONE"
#define IMG_WIDTH									320	
#define IMG_HEIGHT								240
#define SCALE	2 
#define RIGHT_LRF_ON_SCREEN 124 //deg
#define LEFT_LRF_ON_SCREEN 162 //deg
#define LRF_CENTER 131 //deg
#define LRF_HEIGHT 0.52 //meter
#define LRF_BODY_OFFSET 0.1//meter lrf to robot body
#define GRID_SIZE 0.15 //0.15 meter per pixel
#define LEFT_LRF_ON_GROUND LRF_CENTER  - 70
#define RIGHT_LRF_ON_GROUND LRF_CENTER + 70



const ModelOptionCateg MOC_BeoLaserCam = {
        MOC_SORTPRI_3, "Beobot LaserCam Related Options" };

const ModelOptionDef OPT_DisplayFps =
{ MODOPT_ARG(float), "Display Refresh Rate", &MOC_BeoLaserCam, OPTEXP_CORE,
  "When network speed is not fast enough to display camera image "
  "in real time, lower the display fps to reduce display load "
  "This option does not change publish rate (30fps)",
  "fps", '\0', "float", "30.0"};

// identification label for the different camera devices
const ModelOptionDef OPT_DeviceLabel =
{ MODOPT_ARG(int), "id", &MOC_BeoLaserCam, OPTEXP_CORE,
  "When using multiple cameras, we can label them differently ",
  "id", '\0', "int", "0"};

// turn on/off GUI
const ModelOptionDef OPT_LogMode =
{ MODOPT_ARG(bool), "gui", &MOC_BeoLaserCam, OPTEXP_CORE,
  "Turn On/Off Log Mode ", "log-mode", '\0', "bool", "false"};

// foldername location: initialize to NONE
// it has to be specified to start saving
const ModelOptionDef OPT_LogFolderName =
{ MODOPT_ARG(string), "logfoldername", &MOC_BeoLaserCam, OPTEXP_CORE,
  "folder to save the file to ",
  "logfolder-name", '\0', "<string>", NONE_FOLDER_NAME};

// ######################################################################
BeoLaserCam::BeoLaserCam(OptionManager& mgr,
               const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsOfs(new OutputFrameSeries(mgr)),
  itsTimer(1000000),
  itsDisplayTimer(1000000),
  itsCurrMessageID(0),
  itsLrfID(1),
  itsDisplayFps(&OPT_DisplayFps,this,0),
	itsLastUserClickPoint(0,0),
	itsLastSelectedLaserPoint(0,0),
	itsLrfScreenOffset(IMG_WIDTH,0),
	itsLocalMapScreenOffset(IMG_WIDTH*2,0),
	itsCamScreenOffset(0,0),
	itsButtonScreenOffset(IMG_WIDTH*3,0),
	itsLrfDims(IMG_WIDTH,IMG_HEIGHT),
	itsCamDims(IMG_WIDTH,IMG_HEIGHT),
	itsButtonDims(IMG_WIDTH/2,IMG_HEIGHT),
	itsLrfStart(278,127),
	itsLrfEnd(341,126),
	itsCamStart(240,334),
	itsCamEnd(378,331)

{
  addSubComponent(itsOfs);
  itsTimer.reset();
  itsDisplayTimer.reset();

  itsPrevDisplayMessageID = -1;
	itsLrfOffset = 222;
	itsLrfCenterOffset = 184.0*(IMG_HEIGHT/480.0);//in h=480:190=240:x
  itsPreviousTime = 0.0f;
  itsRecentTimes.clear();
  itsRecentTimes.resize(FRAMERATE_INTERVAL);
	itsDataCounter = 0;
	itsZoomScale = 1000;
	itsLaserDeltaOffsetX = 0.0;
	itsLaserDeltaOffsetY = 0.0;
  
  // initial frame rate is invalid
  itsFrameRate = -1.0F;


  // prepare a gamma table for RGBD displays (e.g., Kinect grabber):
  itsGamma.clear(); itsGamma.resize(2048);
  for (int i = 0; i < 2048; ++i) {
    float v = i/2048.0;
    v = powf(v, 3)* 6;
    itsGamma[i] = v*6*256;
  }

}



// ######################################################################
BeoLaserCam::~BeoLaserCam()
{ }

// ######################################################################
void BeoLaserCam::registerTopics()
{
  this->registerPublisher("ColorDepthCameraMessageTopic");

  this->registerSubscription("CameraMessageTopic");
  this->registerSubscription("LRFMessageTopic");
  // send out the log name
  this->registerSubscription("LogFolderNameMessageTopic");  
}

// ######################################################################
void BeoLaserCam::start3()
{
  itsDisplayPeriod = 1000.0f / itsDisplayFps.getVal();
}

// ######################################################################
//! A image to screen display job
class DisplayJob : public JobServer::Job {
public:
  DisplayJob(const Image<PixRGB<byte> > img_, nub::soft_ref<OutputFrameSeries> ofs_,std::vector<double> dist_,nub::ref<BeoLaserCam> blc_) :
    img(img_), ofs(ofs_), dist(dist_),blc(blc_) { }


  virtual ~DisplayJob() { }

  virtual void run() { 

		Point2D<int>  itsLrfScreenOffset(IMG_WIDTH,0);
		Point2D<int>  itsLocalMapScreenOffset(IMG_WIDTH*2,0);

		Image<PixRGB<byte> > lrfMap = blc->drawLaserMap(dist);
		Image<PixRGB<byte> > localMap = blc->drawProjectedLaserMap(dist);
		//inplacePaste(img, lrfMap,   itsLrfScreenOffset);
		//inplacePaste(img, localMap, itsLocalMapScreenOffset);
		ofs->writeRGB(img,"BeoLaserCam");ofs->updateNext(); 
		
	}

  virtual const char* jobType() const { return "DisplayJob"; }

private:
  const Image<PixRGB<byte> > img;
  nub::soft_ref<OutputFrameSeries> ofs;
	std::vector<double> dist;
	nub::ref<BeoLaserCam> blc;
};

// ######################################################################
// ######################################################################
void BeoLaserCam::evolve()
{
          // display the image
					
					itsDispImg = Image<PixRGB<byte> >(IMG_WIDTH*3.5,IMG_HEIGHT,ZEROS);
					its_camera_mutex.lock();
					itsCurrentProcImage = itsCurrentImage;
					its_camera_mutex.unlock();

					its_lrf_mutex.lock();
					std::vector<double> loc_dist = itsCurrentLRF;
					its_lrf_mutex.unlock();

					if(itsCurrentProcImage.initialized())
					{
						inplacePaste(itsDispImg, rescale(itsCurrentProcImage,itsCamDims), itsCamScreenOffset);
					}

					Image<PixRGB<byte> > lrfMap = drawLaserMap(loc_dist);
					itsLocalMapImg = drawProjectedLaserMap(loc_dist);
					inplacePaste(itsDispImg, lrfMap,   itsLrfScreenOffset);
					inplacePaste(itsDispImg, itsLocalMapImg, itsLocalMapScreenOffset);
						handleUserEvent();
						drawUserEvent();
						inplacePaste(itsDispImg, drawButton(), itsButtonScreenOffset);
						//WorkThreadServer wts("Display Server", 1, false); // max of 2 display thread
						//wts.enqueueJob(rutz::make_shared(new DisplayJob(itsDispImg, itsOfs,loc_dist,this)));
						// allow our jobs to get cracking:
						//usleep(50000);

						// wait until all jobs are completed:
						//wts.flushQueue(250000, false);

						itsOfs->writeRGB(itsDispImg, "display");
						itsOfs->updateNext();

}

// ######################################################################
Image<PixRGB<byte> > BeoLaserCam::drawButton()
{
	int w = itsButtonDims.w();
	int h = itsButtonDims.h();
	Image<PixRGB<byte> > buttonImg(itsButtonDims,ZEROS);
	


	itsSaveButton = Button(Point2D<int>(10,10),Dims(w-20,h/3-20));
	itsSaveButton.setLabel(std::string(sformat("Save%2d",itsDataCounter)));
	itsSaveButton.show(buttonImg);

	itsCaliButton = Button(Point2D<int>(10,10+h/3-20),Dims(w-20,h/3-20));
	itsCaliButton.setLabel(std::string("Cali"));
	itsCaliButton.setButtonBgColor(PixRGB<byte>(0,0,255));
	itsCaliButton.show(buttonImg);


	return buttonImg;

	
}
// ######################################################################
void BeoLaserCam::drawUserEvent()
{
	if(itsLastUserClickPoint.i != 0 && itsLastUserClickPoint.j != 0){
		drawLine(itsDispImg, itsLastUserClickPoint,itsLastSelectedLaserPoint,PixRGB<byte>(255,255,0));
		drawCircle(itsDispImg, itsLastUserClickPoint, 5, PixRGB<byte>(0,255,0));
		drawCircle(itsDispImg, itsLastSelectedLaserPoint, 5, PixRGB<byte>(0,0,255));

	}

		
	for(uint i = 1;i< itsLrfRawMap.size();i++)
	{
		if( i == itsLrfRawMap.size()/2 || 1){
			Point2D<int> p = LRFtoCameraMapping(itsLrfRawMap[i]);
			//LINFO("P[%d]: (%d,%d)",(int)i,p.i,p.j);
			if(p.i > 0 && p.i < IMG_WIDTH && p.j > 0 && p.j < IMG_HEIGHT)
				drawCircle(itsDispImg, p, 2, PixRGB<byte>(255,64,0));
		}

	}

}
// ######################################################################
int BeoLaserCam::findNearestLaserPoint(Point2D<int> mouseClickPoint)
{
	double min = dist(mouseClickPoint-itsLrfScreenOffset,itsLrfMap[0]);
	int ang = 0;
	its_lrf_mutex.lock();
	std::vector<double> loc_dist = itsCurrentLRF;
	its_lrf_mutex.unlock();
	for(uint i = 1;i< itsLrfMap.size();i++)
	{
		double d = dist(mouseClickPoint-itsLrfScreenOffset,itsLrfMap[i]);
		if(d < min){
			min = d;
			ang = i;
		}
	}
	itsLastSelectedLaserPoint = itsLrfMap[ang]+itsLrfScreenOffset;
	itsLastUserClickPoint = mouseClickPoint;
	LINFO("Closest Point is at (%d,%d), Angle %d, Dist %4.2f",itsLrfMap[ang].i,itsLrfMap[ang].j,ang,loc_dist[ang]);
	LINFO("Angle %d, Raw(%4.2f,%4.2f)",ang,itsLrfRawMap[ang].i,itsLrfRawMap[ang].j);
	return ang;
}

// ######################################################################
Image<PixRGB<byte> >BeoLaserCam::drawProjectedLaserMap(std::vector<double>& dist)
{

	int w = itsLrfDims.w();
	int h = itsLrfDims.h();
  Image<PixRGB<byte> > localMap = Image<PixRGB<byte> >(itsLrfDims,ZEROS);
	double alpha = 37.16*M_PI/180.0;
	if(dist.size() < RIGHT_LRF_ON_GROUND) return localMap;
	for(int i = LEFT_LRF_ON_GROUND;i < RIGHT_LRF_ON_GROUND;i++)
	{
		double theta = (i - LRF_CENTER)*M_PI/180.0;
		double d = dist[i]/1000.0;
		if(d > 0.0)
		{
		//double dcenter = dist[LRF_CENTER]/1000.0;
		double x = d*sin(theta);
		double y = d*cos(theta);
		Point3D<double> laser = Point3D<double>(x,y,LRF_HEIGHT); 
		Point3D<double> prjPt = tf(laser,0,0,0,alpha,0,0);//rotate at x-axis;
		int xx = int(w/2 - (prjPt.x*100.0));
		int yy = int(h/2 +itsLrfCenterOffset - (prjPt.y*100.0));
		Point2D<int> pt(xx,yy);
		drawCircle(localMap, pt, 2, depth2color(prjPt.z));


		LDEBUG("i:%d Angle %f dist %f x:%f y:%f -> (%f,%f,%f)",i,theta,d,x,y,prjPt.x,prjPt.y,prjPt.z-0.925435);
		
		//int zoom = 10;
		//Point2D<int> pt(int(w/2 - (x*zoom)),int(h/2+itsLrfCenterOffset-(y*zoom)));
		//Rectangle r = Rectangle::centerDims(pt,Dims(zoom,zoom));
		//LINFO("pt(%d,%d) Rect  topLeft(%d,%d) ",pt.i,pt.j,r.topLeft().i,r.topLeft().j);
		//if(localMap.rectangleOk(r))
		//	drawFilledRect(localMap, r, PixRGB<byte>(255,0,255));
		}
	}
	return localMap;

}
// ######################################################################
Image<PixRGB<byte> >BeoLaserCam::drawLaserMap(std::vector<double>& dist)
{
	int size = (int)dist.size();
	int w = itsLrfDims.w();
	int h = itsLrfDims.h();
	int center = size/2;	
	double min = 0,max = 20000.0;
  Image<PixRGB<byte> >lrfImg = Image<PixRGB<byte> >(itsLrfDims,ZEROS);
  drawGrid(lrfImg,10,20,1,PixRGB<byte>(128,128,0));
	itsLrfMap.clear();
	itsLrfRawMap.clear();
	itsLrfCaliPoints.clear();
  //getMinMax(dist, min, max);
		for(int i = 0; i < size; i++)
    {
      // note drawing takes up a lot of time
					double d = dist[i];
          double rad = d; 
					int angle = i + itsLrfOffset;
					//rad = ((rad - min)/(itsZoomScale-min))*h/2;//fixed size
          rad = ((rad - min)/(max-min))*h/2;//float size
          if (rad < 0) rad = 1.0;

          Point2D<int> pt;
          Point2D<double> ptRaw;

					int cal_angle = i - (center-91);//we make center is 90 deg
					//LINFO("Center is %d ,i is %d, cal_angle is %d,dist is %f",center,i,cal_angle,d);
          ptRaw.i = (d/1000.0)*cos((double)cal_angle*M_PI/180.0);
          ptRaw.j = (d/1000.0)*sin((double)cal_angle*M_PI/180.0);					
          pt.i = w/2  - int(rad*sin((double)angle*M_PI/180.0));
          pt.j = h/2+itsLrfCenterOffset - int(rad*cos((double)angle*M_PI/180.0));

					itsLrfMap.push_back(pt);
					itsLrfRawMap.push_back(ptRaw);

					if(i < LEFT_LRF_ON_SCREEN && i > RIGHT_LRF_ON_SCREEN)
					{
						PixRGB<byte> lineColor(0,255,0);
					if(i == center) lineColor = PixRGB<byte>(255,0,0);//make center line red
          drawLine(lrfImg, Point2D<int>(w/2,h/2+itsLrfCenterOffset), pt,lineColor);
          drawCircle(lrfImg, pt, 2, PixRGB<byte>(255,0,0));
					itsLrfCaliPoints.push_back(ptRaw);
							
					}else
					{
          drawCircle(lrfImg, pt, 2, PixRGB<byte>(255,0,255));
					}
          //if(itsDeviceLabel.getVal()==1)
          //  drawRobotBody(pt,count,dist);

    }
		return lrfImg;
}
// ######################################################################
void BeoLaserCam::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{ 
  // record the time
  //uint64 time = itsTimer.get();
  // camera message 
  if(eMsg->ice_isA("::BeobotEvents::CameraMessage"))
  {
    // store the image
    BeobotEvents::CameraMessagePtr cameraMsg =
      BeobotEvents::CameraMessagePtr::dynamicCast(eMsg);

    int currRequestID        = cameraMsg->RequestID;
    int cameraID             = cameraMsg->cameraID;
    Image<PixRGB<byte> > ima = Ice2Image<PixRGB<byte> >(cameraMsg->image);

    if(currRequestID%REPORTING_INTERVAL == 0)
      LINFO("Got a CameraMessage [%3d] with Request ID = %d", 
            cameraID, currRequestID);

			its_camera_mutex.lock();
			itsCurrentImage = ima;
			its_camera_mutex.unlock();


  }
  // LRF message
  else if(eMsg->ice_isA("::BeobotEvents::LRFMessage"))
  {
    // store the LRF data
    BeobotEvents::LRFMessagePtr lrfMsg =
      BeobotEvents::LRFMessagePtr::dynamicCast(eMsg);

    int currRequestID = lrfMsg->RequestID;
    int lrfID         = lrfMsg->LRFID;
    int lrfSize       = lrfMsg->distances.size();
    if(currRequestID%REPORTING_INTERVAL == 0)
      LINFO("Got a LRFMessage [%3d] with Request ID = %d size %d", 
            lrfID, currRequestID,lrfSize);

		its_lrf_mutex.lock();
		itsCurrentLRF = lrfMsg->distances;
		itsLrfID = lrfID;
		its_lrf_mutex.unlock();
  }

}





// ######################################################################
void BeoLaserCam::handleUserEvent()
{
	//handle clicks
	const nub::soft_ref<ImageDisplayStream> ids =
		itsOfs->findFrameDestType<ImageDisplayStream>();

	const rutz::shared_ptr<XWinManaged> uiwin =
		ids.is_valid()
		? ids->getWindow("display")
		: rutz::shared_ptr<XWinManaged>();

	if (uiwin.is_valid())
 {
		int key = uiwin->getLastKeyPress();

		if(key != -1)
						LINFO("key %i", key);
		switch(key)
		{
			case -1:
				break;
			case 13: //g (MAC)
			case 42: //g label ground
				break;
			case 11: //f (mac)
			case 41: //f forward faster
				break;
			case 10: //d (mac) 
			case 40: //d forward slower	
				break;
			case 9:  //s (mac) 
			case 39: //s flip play direction 
				itsLaserDeltaOffsetY-=0.01;
				LINFO("laser Delta Y is %f",itsLaserDeltaOffsetY);
				break;
			case  8: //a (mac) 
			case 38: //a forward normal
				itsLaserDeltaOffsetY+=0.01;
				LINFO("laser Delta Y is %f",itsLaserDeltaOffsetY);
				break;
			case 12: //h (mac) 
			case 43: //h switch labeling ground truth mode 
				break;
				//case 12: //j (mac) 
			case 44: //j play only ground truth image
			case 46: //j (mac) 
				break;
			case 57: //space (mac)(n on PC) 
			case 65: //space pause/resume
				break;
			case 14: //z (mac)
			case 52: //z switch to Anf
				itsZoomScale += 100;
				break;
			case 15: //x switch to SSL 
			case 53: //x switch to SSL 
				itsZoomScale -= 100;
				break;
			case 16: //c (mac) 
			case 54: //c switch to Equad
				break;
			case 17: //v (mac)
			case 55: //v switch to HNB 
				break;
			case 19: //b (mac) 
			case 56: //b switch to RoadGenerator 
				break;
			case 20: //q (mac) 
			case 24: //q Turn on/off VPD 
				itsLaserDeltaOffsetX+=0.01;
				LINFO("laser Delta X is %f",itsLaserDeltaOffsetX);
				break;
			case 21: //w (mac) 
			case 25: //w Turn on/off CSH 
				itsLaserDeltaOffsetX-=0.01;
				LINFO("laser Delta X is %f",itsLaserDeltaOffsetX);
				break;
			case 133: //arrow down(mac)
			case 116: //arrow down
			case 104: //arrow down(beokeyboard) 
				itsLrfCenterOffset++;
				LINFO("offset is %d",itsLrfCenterOffset);
				break;
			case 134: //arrow up(mac) 
			case 111: //arrow up 
			case 98: //arrow up(beokeyboard) 
				itsLrfCenterOffset--;
				break;
			case 100: //arrow left,BeoKeyboard
			case 113: //arrow left
				itsLrfOffset++;
				LINFO("offset is %d",itsLrfOffset);
				break;
			case 102: //arrow right,BeoKeyboard
			case 114: //arrow right 
				itsLrfOffset--;
				break;
			//case 39: //o (mac),conflict with Linux 's' key
			case 32: //o Output Screen Shot

				break;
			default:		
				LINFO("key %i", key);
				break;
		}

		Point2D<int> pos = uiwin->getLastMouseClick();
		if (pos.isValid())
		{
			PixRGB<byte> pixColor = itsDispImg.getVal(pos);
			LINFO("Mouse Click (%d %d) Color (%d,%d,%d)", 
					pos.i,pos.j,pixColor.red(),pixColor.green(),pixColor.blue());

			//check which panel user clicked
			if(inPanel(pos,itsCamScreenOffset,itsCamDims))
			{
				LINFO("You Click Image Panel");
					
			}else if(inPanel(pos,itsLrfScreenOffset,itsLrfDims))
			{				
				LINFO("You Click Laser Panel");
				findNearestLaserPoint(pos);
			}else if(inPanel(pos,itsButtonScreenOffset,itsButtonDims))
			{
				LINFO("You Click Button Panel");
				if(itsCaliButton.handle_events(pos-itsButtonScreenOffset))
					LINFO("You Click Cali Button ");
				if(itsSaveButton.handle_events(pos-itsButtonScreenOffset))
				{
					LINFO("You Click Save Button ");
					saveData();
				}
			}
			
		} 
	}
}	

// ######################################################################
void BeoLaserCam::saveData()
{
	//skip existed files
	std::string saveImgName;
	do{
		saveImgName = sformat("%s/img_target%02d.png",DATA_FOLDER,itsDataCounter++);
	}while(file_exists(saveImgName.c_str()));


	if(itsCurrentProcImage.initialized())
		Raster::WriteRGB(itsCurrentProcImage, saveImgName);

	//LCCT Toolbox format, doesn't work in 2D. Need 3D Scan
	//std::string saveLrfName(sformat("%s/laser_target%d.xyz",DATA_FOLDER,itsDataCounter-1));

	//Q.Zhang & R. Pless format
	std::string saveLrfName(sformat("%s/%02d.log",DATA_FOLDER,itsDataCounter-1));
	FILE *rFile = fopen(saveLrfName.c_str(), "at");
	if (rFile != NULL)
	{
		//For LCCT
		//for(uint i = 0;i< itsLrfCaliPoints.size();i++)
		//{
		//	//LCCT
		//	//std::string line(sformat("%f %f %f\n",itsLrfCaliPoints[i].i,1.0,itsLrfCaliPoints[i].j));

		//	LINFO("%s",line.c_str());	
		//	fputs(line.c_str(), rFile);
		//}

		//For Q.Zhang
		its_lrf_mutex.lock();
		std::vector<double> loc_dist = itsCurrentLRF;
		its_lrf_mutex.unlock();
		int size = (int)loc_dist.size();
		int center = size/2;//center is 0 degree
		//In Q.Zhang's format,we only use front 180 deg of data in meters
		for(int i = center - 90 ;i< center+90;i++)
		{
			std::string line(sformat("%2.5f ",loc_dist[i]/1000.0));//convert mm to meter
			fputs(line.c_str(), rFile);
		}



		fclose (rFile);
	}
	else LFATAL("can't append to file: %s", saveLrfName.c_str());

	
}
// ######################################################################
bool BeoLaserCam::inPanel(Point2D<int> mouseClickPoint, Point2D<int> panelOffeset,Dims panelSize)
{
	int x = mouseClickPoint.i;
	int y = mouseClickPoint.j;
	int lx = panelOffeset.i;
	int rx = lx + panelSize.w();
	int ty = panelOffeset.j;
	int by = ty + panelSize.h();

	if(x >= lx && x <= rx && y>=ty && y<= by)
		return true;
	return false;	
	
}
// ######################################################################
Image<double> BeoLaserCam::laserToCam3D(Point2D<double> point)
{
	LDEBUG("Laser Points (%f,%f)",point.i,point.j);
	Image<double> phi = Image<double>(3,3,ZEROS);
	Image<double> delta = Image<double>(1,3,ZEROS);
	if(itsLrfID)//down laser
	{
		//phi =
		//	  0.9782   -0.0255   -0.2063
		//   -0.0440    0.9446   -0.3252
		//    0.2031    0.3272    0.9229
		phi.setVal(0,0, 0.9782); phi.setVal(1,0,-0.0255);  phi.setVal(2,0,-0.2063);
		phi.setVal(0,1,-0.0440); phi.setVal(1,1, 0.9446);  phi.setVal(2,1,-0.3252);
		phi.setVal(0,2, 0.2031); phi.setVal(1,2, 0.3272);  phi.setVal(2,2, 0.9229);
		//delta
		//   -0.0126
		//    0.1683
		//    0.0060
		delta.setVal(0,0,-0.0126+itsLaserDeltaOffsetX);
		delta.setVal(0,1, 0.1683+itsLaserDeltaOffsetY);
		delta.setVal(0,2, 0.0060);

	}else{//top laser
		//phi =
		//		    0.9990   -0.0424    0.0109
		//    		0.0419    0.9982    0.0419
		//   		 -0.0127   -0.0414    0.9991
		phi.setVal(0,0, 0.9990 ); phi.setVal(1,0,-0.0424);  phi.setVal(2,0,0.0109);
		phi.setVal(0,1, 0.0419 ); phi.setVal(1,1, 0.9982);  phi.setVal(2,1,0.0419);
		phi.setVal(0,2,-0.0127 ); phi.setVal(1,2,-0.0414);  phi.setVal(2,2,0.9991);
		//delta
		//	0.1048
		// -0.5353
		// -0.6716

		delta.setVal(0,0,	0.1048+itsLaserDeltaOffsetX);
		delta.setVal(0,1, -0.5353+itsLaserDeltaOffsetY);
		delta.setVal(0,2, -0.6716);


	}

		//laser point p[x,0,z]
		Image<double> ptl	= Image<double>(1,3,ZEROS);
		ptl.setVal(0,0,point.i);
		ptl.setVal(0,1,0.0);
		ptl.setVal(0,2,point.j);


		Image<double> phi_inv = matrixInv(phi);
		//p - delta
		Image<double> ptl_delta = Image<double>(1,3,ZEROS);
		ptl_delta.setVal(0,0,ptl.getVal(0,0) - delta.getVal(0,0));
		ptl_delta.setVal(0,1,ptl.getVal(0,1) - delta.getVal(0,1));
		ptl_delta.setVal(0,2,ptl.getVal(0,2) - delta.getVal(0,2));

		// Camera Point in 3D = phi_inv * (ptc - delta)
		Image<double> ptc = matrixMult(phi_inv,ptl_delta);
		LDEBUG("Laser Point in Camera space (%f,%f,%f)",ptc.getVal(0,0),ptc.getVal(0,1),ptc.getVal(0,2));

	return ptc;

}
// ######################################################################
Point2D<int> BeoLaserCam::LRFtoCameraMapping(Point2D<double> p)
{
	Image<double> camera_point = laserToCam3D(p);
	Point2D<int> pt = project_point2(camera_point);
	return pt/SCALE;//we calibrate camera and image in 640x480.The display may different

}
// ######################################################################
//Given a 3D point (x,y,z) and Camera K,project it to (u,v) in image space
Point2D<int> BeoLaserCam::project_point2(Image<double> p)
{
	//top camera
	//fc = [ 310.07038   316.27574 ];
	//cc = [ 183.76256   109.10332 ];
	double fx = 323.22012;
	double fy = 328.22387;
	double cx = 182.39032;
	double cy = 106.99918;

	//Distortion
	//k1,k2,k3 are radial distortion coefficients, p1,p2 are tangential distortion coefficients.
	//kc = [ 0.06111   0.42359   -0.03535   0.01713  0.00000 ];
	//				k1				k2				p1				p2				k3
	double k1 =  0.03901 ;
	double k2 =  0.45333 ;
	double p1 = -0.02961 ;
	double p2 =  0.01507 ;
	double k3 =  0.0;
if(itsLrfID)//bottom camera
{
	LINFO("USE Bottom Laser");
	 fx = 644.22486;
	 fy = 649.72331;
	 cx = 299.84217;
	 cy = 247.30126;

	//Distortion
	//k1,k2,k3 are radial distortion coefficients, p1,p2 are tangential distortion coefficients.
	//kc = [ -0.08442   0.93584   0.00726   -0.00986  0.00000 ];
	//				k1				k2				p1				p2				k3
	 k1 =  -0.08442 ;
	 k2 =   0.93584 ;
	 p1 =   0.00726 ;
	 p2 =  -0.00986 ;
	 k3 =   0.0;

}

	double x = p.getVal(0,0);
	double y = p.getVal(0,1);
	double z = p.getVal(0,2);

	double xx = x/z;
	double yy = y/z;

	double rr = xx*xx + yy*yy;
	LDEBUG("RR is %f",rr);
	
	double xxx = xx*(1 + k1*rr + k2*rr*rr + k3*rr*rr*rr) + 2*p1*xx*yy + p2*(rr + 2*xx*xx);
	double yyy = yy*(1 + k1*rr + k2*rr*rr + k3*rr*rr*rr) + p1*(rr+2*yy*yy) + 2*p2*xx*yy;

	LDEBUG("After undistort x,y(%f,%f)",xxx,yyy);

	double u = fx * xxx + cx;
	double v = fy * yyy + cy;

	LDEBUG("Laser Points in image space(%f,%f)",u,v);
	return Point2D<int>(int(u),int(v));

	
}
// ######################################################################
Point3D<double> BeoLaserCam::tf(Point3D<double> p,double x,double y,double z,double roll,double pitch,double yaw)
{
	
	double a = pitch,b = yaw,c = roll;

	Image<double> R = Image<double>(3,3,ZEROS);
	R.setVal(0,0, cos(a)*cos(b)); R.setVal(1,0,cos(a)*sin(b)*c-sin(a)*cos(c));  R.setVal(2,0,cos(a)*sin(b)*cos(c)+sin(a)*sin(c));
	R.setVal(0,1, sin(a)*cos(b)); R.setVal(1,1,sin(a)*sin(b)*c+cos(a)*cos(c));  R.setVal(2,1,sin(a)*sin(b)*cos(c)-cos(a)*sin(c));
	R.setVal(0,2,-sin(b)); 		    R.setVal(1,2,cos(b)*sin(c));  								R.setVal(2,2,cos(b)*cos(c));

	Image<double> pt = pt2matrix(p);//convert Point3D to Matrix format
	Image<double> ptr = matrixMult(R,pt);
	Point3D<double> T = Point3D<double>(x,y,z);
	Point3D<double> result = matrix2pt(ptr)+T;
	return result;


}
// ######################################################################
Image<double> BeoLaserCam::pt2matrix(double x,double y,double z)
{
	
	Image<double> matrix	= Image<double>(1,3,ZEROS);
	matrix.setVal(0,0,x);
	matrix.setVal(0,1,y);
	matrix.setVal(0,2,z);
	return matrix;
	
}
// ######################################################################
Image<double> BeoLaserCam::pt2matrix(Point3D<double> pt)
{
	Image<double> matrix	= Image<double>(1,3,ZEROS);
	matrix.setVal(0,0,pt.x);
	matrix.setVal(0,1,pt.y);
	matrix.setVal(0,2,pt.z);
	return matrix;
	
}
// ######################################################################
Point3D<double> BeoLaserCam::matrix2pt(Image<double> matrix)
{
	Point3D<double> pt = Point3D<double>(
			matrix.getVal(0,0),
			matrix.getVal(0,1),
			matrix.getVal(0,2));
	return pt;
	
}
// ######################################################################
PixRGB<byte> BeoLaserCam::depth2color(double depth)
{

		PixRGB<byte> color;	
		uint v = uint(((depth-0.929059+LRF_HEIGHT/2)/LRF_HEIGHT)*2047.0); 
		LDEBUG("V is %d",int(v));
		
		if (v > 2047) v = 2047;
		int pval = itsGamma[v];
		int lb = pval & 0xff;
		switch (pval>>8) {
			case 0: color = PixRGB<byte>(255, 255-lb, 255-lb); break;
			case 1: color = PixRGB<byte>(255, lb, 0); break;
			case 2: color = PixRGB<byte>(255-lb, 255, 0); break;
			case 3: color = PixRGB<byte>(0, 255, lb); break;
			case 4: color = PixRGB<byte>(0, 255-lb, 255); break;
			case 5: color = PixRGB<byte>(0, 0, 255-lb); break;
		 default: color = PixRGB<byte>(255, 0, 0); break;
		}
		return color;
	
}
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

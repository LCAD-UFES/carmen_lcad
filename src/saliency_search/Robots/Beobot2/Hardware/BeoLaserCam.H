/*!@file Robots2/Beobot2/Hardware/BeoIMU.H Ice Module for IMU           */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/Hardware/BeoLaserCam.H
// $Id: BeoLaserCam.H 15185 2012-02-28 22:33:11Z kait $
//

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Util/Timer.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include <IceUtil/Thread.h>

#include "Image/MathOps.H"
#include "Image/Point3D.H"
#include "Image/MatrixOps.H"
//#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "Media/FrameSeries.H"
//#include "Transport/FrameIstream.H"

#include "GUI/Button.H"

#include <vector>
#include <sys/stat.h>

#ifndef BEOLASERCAM_H
#define BEOLASERCAM_H

class BeoLaserCam : public RobotBrainComponent
{
public:

  BeoLaserCam(OptionManager& mgr,
         const std::string& descrName = "BeoLaserCam",
         const std::string& tagName = "BeoLaserCam");

  ~BeoLaserCam();

  virtual void evolve();

  //! Get a message
  virtual void updateMessage
  (const RobotSimEvents::EventMessagePtr& eMsg,
   const Ice::Current&);

  //! register topics
  virtual void registerTopics();

  //! to set the log mode and folder
  void start3();

	//! Convert polar to image map
	Image<PixRGB<byte> >drawLaserMap(std::vector<double> &dist);

	Image<PixRGB<byte> >drawProjectedLaserMap(std::vector<double>& dist);
private:


  nub::soft_ref<OutputFrameSeries> itsOfs;



  Timer itsTimer;
  Timer itsDisplayTimer;
  std::vector<float> itsRecentTimes;
  float itsFrameRate;
  float itsPreviousTime;


  int itsCurrMessageID;
  int itsLrfID;//When using multiple LRF,id=0 for top laser,id=1 for cliff laser
  int itsPrevDisplayMessageID;
	int itsLrfOffset;
	int itsLrfCenterOffset;

	int itsDataCounter;
  //! The display refresh rate, default is 30 fps
  OModelParam<float>       itsDisplayFps;


  IceUtil::Mutex its_camera_mutex;  //!< locking camera image
  IceUtil::Mutex its_lrf_mutex;  //!< locking laser reading
	Image<PixRGB<byte> > itsCurrentImage;//get update by updateMessage
	Image<PixRGB<byte> > itsCurrentProcImage;//using for evolve works
	Image<PixRGB<byte> > itsDispImg;
	Image<PixRGB<byte> > itsLocalMapImg;
	std::vector<double> itsCurrentLRF;
	std::vector<Point2D<int> > itsLrfMap;
	std::vector<Point2D<double> > itsLrfRawMap;
	std::vector<Point2D<double> > itsLrfCaliPoints;//LRF points hit on chessboard

	Point2D<int> itsLastUserClickPoint;
	Point2D<int> itsLastSelectedLaserPoint;
	Point2D<int> itsLrfScreenOffset; 
	Point2D<int> itsLocalMapScreenOffset; 
	Point2D<int> itsCamScreenOffset; 
	Point2D<int> itsButtonScreenOffset; 

	float				 itsZoomScale;
	float				 itsLaserDeltaOffsetX;
	float				 itsLaserDeltaOffsetY;
	Dims				 itsLrfDims;
	Dims				 itsCamDims;
	Dims				 itsButtonDims;
  float itsDisplayPeriod;
  std::vector<uint16> itsGamma;

	Point2D<int> itsLrfStart; 
	Point2D<int> itsLrfEnd; 
	Point2D<int> itsCamStart; 
	Point2D<int> itsCamEnd; 

	Button itsSaveButton;
	Button itsCaliButton;

	//! Given user mouse click, find nearest laser point
	int findNearestLaserPoint(Point2D<int> mouseClickPoint);


	//! Read Keyboard and Mouse event
	void handleUserEvent();

	//! Draw user generate events such as last mouse click point
	void drawUserEvent();

	//! Draw the button panel and buttons
	Image<PixRGB<byte> > drawButton();
	
	//! Check mouse click point in the panel or not
	bool inPanel(Point2D<int> mouseClickPoint, Point2D<int> panelOffeset,Dims panelSize);

	//! LRF to Image Mapping Function
	Point2D<int> LRFtoCameraMapping(Point2D<double> p);

	//! Convert LRF to Camera 3D space
	Image<double> laserToCam3D(Point2D<double> point);

	//! project 3D point in camera space to image space
	Point2D<int> project_point2(Image<double> p);


	Point3D<double> tf(Point3D<double> p,double x,double y,double z,double roll,double pitch,double yaw);
	Image<double> pt2matrix(double x,double y,double z);
	Image<double> pt2matrix(Point3D<double> pt);
	Point3D<double> matrix2pt(Image<double> matrix);
	PixRGB<byte> depth2color(double depth);


	//! Save Image data and laser data for calibration
	void saveData();

	//! Compute Euclidean distance between two points
	double dist(Point2D<int> p1,Point2D<int> p2){
		double dx = p1.i - p2.i;	
		double dy = p1.j - p2.j;	
		return dx*dx+dy*dy;
	}
	//! check file exist or not
	int file_exists(const char * fileName)
	{
		struct stat buf;
		int i = stat(fileName,&buf);
		if(i == 0)
			return 1;
		return 0;


	}
};

#endif


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

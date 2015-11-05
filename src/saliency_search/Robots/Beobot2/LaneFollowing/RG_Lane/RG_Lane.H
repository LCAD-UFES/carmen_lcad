/*!@file Robots2/Beobot2/LaneFollowing/RG_Lane/RG_Lane.H Ice Module to Log data    */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/LaneFollowing/RG_Lane/RG_Lane.H
// $Id: RG_Lane.H 12962 2010-03-06 02:13:53Z irock $
//

#ifndef RG_LANE_H
#define RG_LANE_H
#include "Image/OpenCVUtil.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Util/Timer.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include <IceUtil/Thread.h>

//#include "Demo/SaliencyMT.H"
#include "Robots/Beobot2/LaneFollowing/RG_Lane/SaliencyMT.H"
//#include "Robots/Beobot2/LaneFollowing/RG_Lane/CenterSurroundHistogramRoadRecognition.H"
#include "Robots/Beobot2/LaneFollowing/AppearanceContrastTrailRecognizer.H"
#include "Robots/Beobot2/LaneFollowing/RG_Lane/RoadGenerator.H"
#include "Robots/Beobot2/BeoCommon.H"

#include "Media/FrameSeries.H"

#include "Image/Image.H"
#include "Image/Point3D.H"
#include "Raster/Raster.H"

#include <vector>
#include "Transport/FrameInfo.H"
#include "GUI/ImageDisplayStream.H"
#include "GUI/XWinManaged.H"//for keyboard & mouse click
#include "Gist/VanishingPointDetector.H" 
#include "Gist/SuperPixelRoadSegmentor.H" 

#include "BayesFilters/UKF.H"//for U Kalman Filter
//################################################################
class SPFilter: public UKF
{
	public:
		SPFilter():
			UKF(5,5)//5 state 5 observations
	{

		itsState.setVal(0,0, 320/2);//middle x	
		itsState.setVal(0,1, 240/2);//middle y
		itsState.setVal(0,2, 0);//R
		itsState.setVal(0,3, 0);//G
		itsState.setVal(0,4, 0);//B

		////Initial covariance 
		itsSigma.setVal(0,0,160/2*160/2); // x sigma^2
		itsSigma.setVal(1,1,120/2*120/2); // y
		itsSigma.setVal(2,2,255*255); //R
		itsSigma.setVal(3,3,255*255); //G
		itsSigma.setVal(4,4,255*255); //B

		//Initial noise matrix
		double posVar=2.0;
		double roadColorVar=2.0;

		itsR.setVal(0,0,posVar*posVar);
		itsR.setVal(1,1,posVar*posVar);
		itsR.setVal(2,2,roadColorVar*roadColorVar);
		itsR.setVal(3,3,roadColorVar*roadColorVar);
		itsR.setVal(4,4,roadColorVar*roadColorVar);

	}

		~SPFilter() {}; 

		Image<double> getNextState(const Image<double>& X, int k)
		{

			double posX = X.getVal(k,0);
			double posY = X.getVal(k,1);
			double colorR=  X.getVal(k,2);
			double colorG = X.getVal(k,3);
			double colorB = X.getVal(k,4);


			Image<double> Xnew(1,itsNumStates, ZEROS);

			Xnew[0] = posX ;
			Xnew[1] = posY;
			Xnew[2] = colorR;
			Xnew[3] = colorG;
			Xnew[4] = colorB;
			return Xnew;
		}
		Image<double> getObservation(const Image<double>& X, int k)
		{
			double posX = X.getVal(k,0);
			double posY = X.getVal(k,1);
			double colorR=  X.getVal(k,2);
			double colorG = X.getVal(k,3);
			double colorB = X.getVal(k,4);

			Image<double> zNew(1,itsNumObservations, ZEROS);
			zNew[0] = posX ;
			zNew[1] = posY;
			zNew[2] = colorR;
			zNew[3] = colorG;
			zNew[4] = colorB;
			return zNew;
		}
		void getState(Point2D<int>& pos,PixRGB<byte>& roadColor )
		{
			pos = Point2D<int>(itsState[0],itsState[1]);
			roadColor = PixRGB<byte>((int)itsState[2],(int)itsState[3],(int)itsState[4]);			
			//LINFO("Road Color UKF (%d,%d,%d)",roadColor.red(),roadColor.green(),roadColor.blue());
		}

};//end class UKF




//################################################################
typedef struct _RegionInformation
{
	uint start;
	uint end;

	float minDistance;
	uint minIndex;
}
RegionInformation;

//################################################################
class roadRegion
{

	Point2D<int> vp;
	Point2D<int> bp;
	Point2D<int> lbp;
	Point2D<int> rbp;
	Point2D<int> np;//true road center,lbp or rbp can be outside of image
	Dims image_dim;
	Point2D<int> lep;//left edge point
	Point2D<int> rep;
	Point2D<int> lnp;
	Point2D<int> rnp;//right navigatable point
	Point2D<int> mnp;//bottom middle navigatable point
	Point2D<int> cp;//8 pixel higher than mnp
	int h,w; 
	bool dontCare;

	public :

	int image_id;

	//! Default constructor
	roadRegion(){}

	//! Don't Care frame constructor
	roadRegion(int i,Dims d)
	{
		image_id = i;
		image_dim = d;		
		h = image_dim.h();
		w = image_dim.w();

		vp  = Point2D<int>(0,0);
		bp  = Point2D<int>(0,0);
		lbp = Point2D<int>(0,0);
		rbp = Point2D<int>(0,0);
		np  = Point2D<int>(0,0);
		mnp = Point2D<int>(0,0); 
		cp  = Point2D<int>(0,0);
		dontCare = true;	
	}
	//Labeled frame constructor
	roadRegion(int i,Point2D<int> v,Point2D<int> b,Point2D<int> l,Point2D<int> r,Point2D<int> n,Dims d)
	{
		image_id = i;
		vp = v;
		bp = b;
		lbp = l;
		rbp = r;
		np = n;
		image_dim = d;		
		h = image_dim.h();
		w = image_dim.w();
		computeLep();
		computeRep();
		mnp = (lnp+rnp)/2;
		cp = vanishPoint(Point2D<int>(0,h-8),Point2D<int>(w,h-8),vp,mnp);
		dontCare = false;
	}

	bool getDontCare(){return dontCare;}


	//! Prepare road info for file output
	std::string toString()
	{
		if(dontCare)
			return std::string(sformat("DR:ID %d\n", image_id));	
		return std::string(sformat("GT:ID %d,VP %d,%d,BP %d,%d,LBP %d,%d,RBP %d,%d,np %d,%d\n",
					image_id,vp.i,vp.j,bp.i,bp.j,lbp.i,lbp.j,rbp.i,rbp.j,np.i,np.j));	
	}
	//! Given estimate road point, we can output result data
	std::string output(float pt,int est)
	{
		return std::string(sformat("GT:ID %d,PT %f,VP %d,%d,BP %d,%d,LBP %d,%d,RBP %d,%d,npx %d,est %d,error %d,error2 %d\n",
					image_id,pt,vp.i,vp.j,bp.i,bp.j,lbp.i,lbp.j,rbp.i,rbp.j,np.i,est,np.i-est,cp.i-est));	
	}
	//! Compute left end point inside image

	Point2D<int> computeLep()
	{
		if(lbp.i < 0){
			//if left point is out of image left edge,we chop it and draw a vertical line to the bottom
			lep = vanishPoint(vp,lbp,Point2D<int>(0,0),Point2D<int>(0,h));//Left edge point :Point on image left edge
			lnp = Point2D<int>(lep.i,h);
		}else{
			lnp = lbp;
		}
		return lnp;
	}

	//! Compute right end point inside image
	Point2D<int> computeRep()
	{
		if(rbp.i > w){
			//if right point is out of image right edge
			rep = vanishPoint(vp,rbp,Point2D<int>(w,0),Point2D<int>(w,h));//right edge point :Point on image right edge
			rnp = Point2D<int>(rep.i,h);
		}else{
			rnp = rbp;
		}
		return rnp;
	}

	//! Getter function
	int getGroundTruth(){
		return np.i;
	}
	int getError(int est){
		return np.i -est;
	}
	int getError2(int est){
		return cp.i -est;
	}

	//! Draw left/right road boundary and vanishing point on given image
	Image<PixRGB<byte> > drawRoadRegion(Image<PixRGB<byte> > img)
	{
		drawCircle(img,vp ,5,PixRGB<byte>(20,50,255),10);//vanishing point
		//drawLine(img,vp ,bp  ,PixRGB<byte>(128,128,0),2);//groudtruth middle line 
		drawLine(img,vp,mnp,PixRGB<byte>(0,191,255),2);//navigatable line (Depp sky blue) 

		//draw left boundary
		if(lbp.i< 0)
		{
			drawLine(img,vp,lep,PixRGB<byte>(0,255,0),2);//screen center -Green line 
			drawLine(img,lep,Point2D<int>(0,h),PixRGB<byte>(0,255,0),2);//screen center -Green line 
		}else	{
			drawLine(img,vp ,lbp ,PixRGB<byte>(0,255,0),2);//screen center -Green line 
		}
		//draw right boundary
		if(rbp.i > w)
		{
			drawLine(img,vp,rep,PixRGB<byte>(0,255,0),2);//screen center -Green line 
			drawLine(img,rep,Point2D<int>(w,h),PixRGB<byte>(0,255,0),2);//screen center -Green line 

		}else{
			drawLine(img,vp ,rbp ,PixRGB<byte>(0,255,0),2);//screen center -Green line 
		}
		return img;
	}

	Point2D<int> vanishPoint(Point2D<int> p1, Point2D<int> p2,Point2D<int> p3,Point2D<int> p4){
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
		if (abs(numera) < EPS && abs(numerb) < EPS && abs(denom) < EPS) {
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
	~roadRegion(){};
};//end class roadRegion

//################################################################
//Store all road region information
class roadRegionDB
{
	std::vector<roadRegion> db;
	int start,end,size;
	int *hashIndexMap;//using hash table to get direct access to each element

	public:
	roadRegionDB(){}

	//! Constructor
	roadRegionDB(int s,int e)
	{
		start = s;
		end = e;
		size = (e-s)+1;

		LINFO("size %d,end %d, start %d",size,end,start);
		hashIndexMap = new int[size];
		for(int i = 0;i< size;i++){
			hashIndexMap[i] = -1;//default -1 means no data
		}
	}

	//! This will only check if there is a labeled frame exist or not
	//  Labeled Frame can be GT or Don't Care(DR)
	bool find(int image_id)
	{

		int id = image_id - start;	
		int index = hashIndexMap[id];

		if(index < 0 || id <= 0){
			return false;
		}else{
			return true;
		}
	}

	//! Find image only have groundtruth data, don't look for don't care frame
	bool find(int image_id,roadRegion &rr)
	{
		int id = image_id - start;	
		int index = hashIndexMap[id];

		if(index < 0 || id <= 0){		
			return false;
		}else{
			if(db[index].getDontCare())
				return false;			
			rr = db[index];
			return true;
		}
	}
	//! only find GT frame, but don't return road region
	bool find2(int image_id)
	{
		int id = image_id - start;	
		int index = hashIndexMap[id];

		if(index < 0 || id <= 0){		
			return false;
		}else{
			if(db[index].getDontCare())
				return false;			
			return true;
		}
	}

	//! Add road region in to road db
	void add(roadRegion rr)
	{
		int index = rr.image_id - start; //should >= 0
		int lastId = end  - rr.image_id ;//should >= 0
		//LINFO("index %d,id %d, start %d",index,rr.image_id,start);
		if(index >= 0 && lastId >= 0)
		{
			hashIndexMap[index] = (int)db.size();
			db.push_back(rr);
		}
	}
	~roadRegionDB(){};

};//End class roadRegionDB

//################################################################
//! for loading simulation images and play those image
class imageDB 
{
	std::string path;//Full path such as "../data/corner/HNB/2010_12_34/image00000"
	std::string postfix;//Extension type,such as ".pnm", "ppm"
	int start;
	int end;
	int current;	
	int fps;
	int sign;
	roadRegionDB rdb;//for ground truth playing
	int skipFrame;//
	bool usingRdb;// using road region database
	public:

	imageDB(){}

	imageDB(std::string p,std::string f,int s,int e)
	{
		start = s;
		end = e;
		path = p;
		postfix = f;
		current = s;
		fps = 1;
		sign = 1;
		usingRdb = false;
		skipFrame = 4;
	}

	//Image playing function 
	void forwardFaster(){
		fps+=skipFrame;
	}
	void forwardSlower(){
		fps-=skipFrame;
		if(fps <= 0){
			fps = 1;
		}
	}
	void forwardNormal(){
		fps = skipFrame;
	}
	void flipDirection(){
		sign *= -1;
	}

  //Get next image
	Image<PixRGB<byte> > nextImg()
	{
		current+=(fps*sign);
		if(current > end){
			current = start;//loop			
		  LFATAL("Finish loop,exit");
		}
		else if(current < start)
			current = end;

  std::string imgpath;
  //Range only in 3 digits
	if(end<1000)
		imgpath = sformat("%s%03d%s",path.c_str(),current,postfix.c_str());
	else if(end<10000) //Range only in 4 digits
		imgpath = sformat("%s%04d%s",path.c_str(),current,postfix.c_str());
  else
		imgpath = sformat("%s%05d%s",path.c_str(),current,postfix.c_str());
	LDEBUG("Image path is %s",imgpath.c_str());
		return Raster::ReadRGB(imgpath.c_str());
	}

	//If road region db is available, find next available gt image 
	Image<PixRGB<byte> > nextGroundTruthImg()
	{
		if(usingRdb){
			int fpsCount = 0;
			while(current < end && fpsCount < fps)
			{
				current++;
				if( rdb.find2(current)){
					fpsCount++;
				}		
			}
			if(current >= end) {
				current = start;//loop
			  LFATAL("Finish loop,exit");
			}
		}

		//Range only in 3 digits
		std::string imgpath;
		if(end<1000)
			imgpath = sformat("%s%03d%s",path.c_str(),current,postfix.c_str());
		else if(end<10000) //Range only in 4 digits
			imgpath = sformat("%s%04d%s",path.c_str(),current,postfix.c_str());
		else
			imgpath = sformat("%s%05d%s",path.c_str(),current,postfix.c_str());
		LDEBUG("Image path is %s",imgpath.c_str());
		return Raster::ReadRGB(imgpath.c_str());
	}

	int getCurrentImageID(){
		return current;
	}
	int getFps(){
		return fps*sign;
	}
	void setFps(int f){ fps = f;}

	//Time of frame , 0.0~1.0
	float getPercentage(){

		float c = current - start;
		float d = end - start;	
		float rate = c / d;
		if(rate < 0.01){
			rate = 0.01;
		}
		return rate;
	}

	float getCurrentTime(){

		float c = current - start;
		float time = c / 30.0;
		return time;
	}

	float getTotalTime(){
		float t = end - start;
		float time = t / 30.0;
		return time;
	}

	void setSkipFrame(int sk){ skipFrame = sk;}

	void setRoadRegionDB(roadRegionDB r){
		rdb = r;
		usingRdb = true;
	}
	~imageDB(){}
};//end class imageDB


//################################################################
//################################################################
//################################################################
class RG_Lane: public RobotBrainComponent
{
	public:

		RG_Lane(OptionManager& mgr,
				const std::string& descrName = "RG_Lane",
				const std::string& tagName = "RG_Lane");

		~RG_Lane();

		virtual void evolve();

		//! Get a message
		virtual void updateMessage (const RobotSimEvents::EventMessagePtr& eMsg, const Ice::Current&);

		//! Register Ice Message Topic such as camera/motor info
		virtual void registerTopics();

		//! System call this function before start, the system opt is ready here
		void start1();

	private:

		//! Switching to different database
		void switchDB(const std::string& dbname);

		//! Open database
		void openDB(const std::string& dbname);

		//! Load Next input frame. Source can be RoadGenerator,Real Camera or Image Database
		void loadFrame();

		//! find the road, algorithms entry
		void findRoad();

		//! Given a vector of points, return the best fit line 
		void fitLine(std::vector<Point2D<int> > points, Point2D<int>&p1,Point2D<int>&p2);

		//! Compute region segments and find the road region
		Image<PixRGB<byte> > getSuperPixel(Image<PixRGB<byte> > img);

		//!	Not in Use
		Image<PixRGB<byte> > getCannyEdge(Image<PixRGB<byte> > img, Image<PixRGB<byte> > &rawCannyImg);

		//! Not in Use
		Image<PixRGB<byte> > getSaliency(Image<PixRGB<byte> > img);


		//! Ground Truth Labeling Mode - For Annotater
		void groundTruthMode(const rutz::shared_ptr<XWinManaged> uiwin);

		//! Generate Hotkey List and Playing bar
		Image<PixRGB<byte> > getMeterImage(Dims dims);

		//! Draw ground truth lines if available
		Image<PixRGB<byte> > getGroundTruthData(Image<PixRGB<byte> > superCannyMap);

		//! Update current Kalman Filter of road center location and color
		void updateUKF(Point2D<int> midPoint,PixRGB<byte>roadColor);

		//! Open another window for debug display
		void debugWin(Image<PixRGB<byte> >disp,std::string title = "Debug Win");

		//! Handle User Keyboar/Mouse event
		void handleUserEvent();

		//! Display Function
		void displayCanny();
		void displayVanishingPoint(Dims dims);
		void displayCenterSurround(Dims dims);
		void displayNavigationError
			(double error, Image<PixRGB<byte> > &roadFindingMap, Image<PixRGB<byte> > &superPixelMap);

		//! Robot Driving Function
		void updateMotor(double tran, double rot);

		/////////////////////////////////////////////////////////////////////
		// class member variables
		/////////////////////////////////////////////////////////////////////

		//! Input source 
		imageDB itsImageDB;
		roadRegionDB itsRoadRegionDB;

		//! Input/Output Images
		nub::soft_ref<OutputFrameSeries> itsOfs;
		Image<PixRGB<byte> > itsCurrImg;
		Image<PixRGB<byte> > itsProcImg;
		Image<PixRGB<byte> > itsDispImg;
		Image<PixRGB<byte> > itsRawSuperPixelImg;
		Image<PixRGB<byte> > itsRawRoadSuperPixelImg;
		Image<PixRGB<byte> > itsGroundTruthImg;
		int itsCurrImgID;
		int itsPrevProcImgID;


		//! UKF parameters
		SPFilter itsSpf;//SuperPixel UKF
		Image<double> itszNoise;
		double itsZprob;

		//! Algorithm Modules
		rutz::shared_ptr<VanishingPointDetector> itsVanishingPointDetector;
  //rutz::shared_ptr<CenterSurroundHistogramRoadRecognition> itsCSHistRoadRec;
		rutz::shared_ptr<AppearanceContrastTrailRecognizer> itsCSHistRoadRec;
		rutz::shared_ptr<SuperPixelRoadSegmentor> itsSuperPixelRoadSegmentor;

		rutz::shared_ptr<RoadGenerator> itsRoadGenerator;
		//  nub::soft_ref<SaliencyMT> itsSmt;

		//! Road parameters
		Point2D<int> itsEstMiddlePoint;//Current road middle point
		Point2D<int> itsEstMiddlePointFiltered;//UKF filted road millde point
		Point2D<int> itsHighMiddlePoint;//p1,get from fitLine of itsMiddlePoint
		PixRGB<byte>  itsRoadColor;//Current road color
		PixRGB<byte>  itsRoadColorFiltered;//UKF filted road color
		float itsRoadColorDiff;//Current road color difference from filted road color
		PixRGB<byte> itsRoadColorDiffSub;//Color diff for each color channel
		int itsMiddlePoint[5];//Bottom road middle point for line fitting

		//! SuperPixel Road finding flags
		bool itsUseFloatWindow;

		//Feature maps, not in use
		std::vector<Image<float> >itsMapChannels;//for superpixel multi channel test
		Image<float> itsSalMap;//for superpixel  channel test
		Image<float> itsIntMap;//for superpixel  channel test
		Image<float> itsRgMap;//for superpixel  channel test
		Image<float> itsByMap;//for superpixel  channel test

		//! Ice related and motor command
		IceUtil::Mutex its_Curr_Img_mutex;        //!< locking log filename
		IceUtil::Mutex its_Curr_Mtr_mutex;        //!< locking log filename
		double itsRcTransSpeed;
		double itsRcRotSpeed;
		int itsRemoteMode;

		//! Program mode flag
		bool itsWaitScreen;
		bool itsGroundTruthMode;//for gt labeling
		bool itsGroundTruthOnlyMode;//only play gt frame
		bool itsSyntheticRoadMode;//read image from RoadGenerator
		bool itsUsingVPD;//using vanishing point algorithm
		bool itsUsingCSH;//using center surround algorithm

		//! Ground Truth Parameters
		int itsGroundTruthCounter;//count how many times we enter gt mode 
		int itsLabeledGroundTruthCounter;//Count how many frame get labeled
		int itsPlayedGroundTruthCounter;//show how many ground truth image already played
		int itsGroundTruthErrorSum;//for average gt error

		//! File I/O name
		std::string itsConfFilename;
		std::string itsTestFilename;
		std::string itsTestVPFilename;
		std::string itsTestCSHFilename;
		std::string itsTestSPFilename;

		//! Debug Window
		rutz::shared_ptr<XWinManaged> itsWin;
		std::vector<rutz::shared_ptr<XWinManaged> > itsWinVec;
		uint itsDebugWinCounter;

		//! Timer
		Timer itsTimer;
		Timer itsGTimer;//to measure how long in the GT mode

		//! System Command Line Parameters
		OModelParam<bool>        itsGroundTruthModeOpt;
		OModelParam<std::string> itsAnnotaterOpt;
		OModelParam<int>         itsSpeedOpt;
		OModelParam<std::string> itsSourceDB;
		OModelParam<bool>        itsRealCameraModeOpt;

		// File I/O helper function
		// ######################################################################
		void writeToTestFile(std::string line)
		{
			FILE *rFile = fopen(itsTestFilename.c_str(), "at");
			if (rFile != NULL)
			{
				fputs(line.c_str(), rFile);
				fclose (rFile);
			}
			else LFATAL("can't append to file: %s", itsTestFilename.c_str());

		}
		// ######################################################################
		void writeToConfFile(std::string line)
		{
			FILE *rFile = fopen(itsConfFilename.c_str(), "at");
			if (rFile != NULL)
			{
				fputs(line.c_str(), rFile);
				fclose (rFile);
			}
			else LFATAL("can't append to file: %s", itsConfFilename.c_str());

		}
		// ######################################################################
		void writeToFile(std::string line,std::string file)
		{
			FILE *rFile = fopen(file.c_str(), "at");
			if (rFile != NULL)
			{
				fputs(line.c_str(), rFile);
				fclose (rFile);
			}
			else LFATAL("can't append to file: %s", itsConfFilename.c_str());

		}
		// Math calculation function
		// ######################################################################
		float colorDiff(PixRGB<byte> c1,PixRGB<byte> c2)
		{
			int r = c1.red() - c2.red();
			int g = c1.green() - c2.green();
			int b = c1.blue() - c2.blue();
			return sqrt(r*r+g*g+b*b);
		} 
		// ######################################################################
		PixRGB<byte> colorDiffSub(PixRGB<byte> c1,PixRGB<byte> c2)
		{
			int r = abs(c1.red() - c2.red());
			int g = abs(c1.green() - c2.green());
			int b = abs(c1.blue() - c2.blue());
			return PixRGB<byte>(r,g,b);
		} 
		// Weighted Average color
		// ######################################################################
		PixRGB<byte> colorAvg(PixRGB<byte> c1,PixRGB<byte> c2,float ratio1 = 0.5)
		{
			//int ratio1 = 8;//80%
			float ratio2 = 1.0 - ratio1;//20%

			int r = (c1.red()*ratio1 + c2.red()*ratio2);
			int g = (c1.green()*ratio1 + c2.green()*ratio2);
			int b = (c1.blue()*ratio1 + c2.blue()*ratio2);
			return PixRGB<byte>(r,g,b);
		} 
		// Given two lines, return intersection
		// ######################################################################
		Point2D<int> vanishPoint(Point2D<int> p1, Point2D<int> p2,Point2D<int> p3,Point2D<int> p4)
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
			if (abs(numera) < EPS && abs(numerb) < EPS && abs(denom) < EPS) {
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
};//end class RG_Lane
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

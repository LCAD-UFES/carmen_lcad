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
//#include "Image/OpenCVUtil.H"
#include "Robots/Beobot2/LaneFollowing/RG_Lane/RG_Lane.H"
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
#include "Ice/BeobotEvents.ice.H"
//#include "<sys/stat.h>"		//for system time

#include "SIFT/Keypoint.H"
#include "SIFT/VisualObject.H"
#include "SIFT/VisualObjectMatch.H"
#include "Transport/FrameInfo.H"

#include "Ice/IceImageUtils.H"

#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"

#include "GUI/DebugWin.H" // for SHOWIMG()

#include <map>
#include <pthread.h>



#define FOLDER 	"../data/logs"
//#define DBNAME	"Lane"
//#define DBNAME	"VSS"
#define HNB	  "indoor"
#define SSL	  "SSL"
#define EQUAD	"Equad"
#define ANF   "outdoor"
#define ICRA  "ICRA2012"
#define DEFAULT_DBNAME EQUAD 

//#define DBNAME	"post"
//#define DBNAME	"post"
#define DEFAULT_IMAGE_PREFIX  "image_0000000000"
#define DEFAULT_IMAGE_POSTFIX ".ppm"

#define SUPERPIXEL_DOWNSIZE_FACTOR   4
#define CSH_DOWNSIZE_FACTOR          2

#define SEARCH_WINDOW_W 20
#define SEARCH_WINDOW_H 5
#define SEARCH_WINDOW_BOTTOM 1 //1 pixel above image
#define GROUND_TRUTH_MODE false
#define GROUND_TRUTH_FPS 1 //
#define GROUND_TRUTH_ONLY false
#define SYNTHETIC_ROAD_MODE false
#define COLOR_WHITE PixRGB<byte>(255,255,255)

#define USING_VPD false 
#define USING_CSH false

#define ANFSTD 6.92632622334715 //std error for sp+vp
#define ANFMEN 6.91860465116279 //mean error for sp+vp

#define ANFSTDSP 9.04950879837968 //std error for sp only
#define ANFMENSP 7.29715762273902 //mean error for sp only

#define ANFSTDVP 19.0542245321467 //std error for vp only
#define ANFMENVP 12.0292397660819 //mean error for vp only

const ModelOptionCateg MOC_RGLANE = {
        MOC_SORTPRI_3, "Region Growing Related Options" };

const ModelOptionDef OPT_GROUNDTRUTHMODE =
{ MODOPT_ARG(bool), "GROUNDTRUTHMODE", &MOC_RGLANE, OPTEXP_CORE,
        "Enter program as a ground truth labeling tool",
        "gt-mode", '\0', "true|false", "false"};
const ModelOptionDef OPT_ANNOTATER=
{ MODOPT_ARG(std::string), "annotater", &MOC_RGLANE, OPTEXP_CORE,
        "Annotater name for groundtruth labeling",
        "name", '\0', "<string>", "NULL"};
const ModelOptionDef OPT_SPEED=
{ MODOPT_ARG(int), "play speed", &MOC_RGLANE, OPTEXP_CORE,
        "Default play speed",
        "speed", '\0', "int", "10"};
const ModelOptionDef OPT_SOURCE=
{ MODOPT_ARG(std::string), "source", &MOC_RGLANE, OPTEXP_CORE,
        "Default database source",
        "db", '\0', "<string>", "Default"};

const ModelOptionDef OPT_REALCAMERAMODE=
{ MODOPT_ARG(bool), "Real Camera Mode", &MOC_RGLANE, OPTEXP_CORE,
        "Using real camera image form BeoCamera",
        "real-camera", '\0', "true|false", "false"};
// ######################################################################
RG_Lane::RG_Lane(OptionManager& mgr,
                 const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
//  itsSmt(new SaliencyMT(mgr,descrName,tagName)),
  itsOfs(new OutputFrameSeries(mgr)),
  //itsCSHistRoadRec(new CenterSurroundHistogramRoadRecognition()),
  itsCSHistRoadRec(new AppearanceContrastTrailRecognizer()),
  itsRoadGenerator(new RoadGenerator(80,60)),
  itsTimer(1000000),//microseconds
  itsGTimer(1),//1 tick per sec
	itsGroundTruthModeOpt(&OPT_GROUNDTRUTHMODE, this, 0),
	itsAnnotaterOpt(&OPT_ANNOTATER, this, 0),
	itsSpeedOpt(&OPT_SPEED, this, 0),
	itsSourceDB(&OPT_SOURCE, this, 0),
	itsRealCameraModeOpt(&OPT_REALCAMERAMODE, this, 0)
{
  setModelParamVal("InputFameDims",Dims(320,240),MC_RECURSE|MC_IGNORE_MISSING);
//  addSubComponent(itsSmt);
  itsRoadColor = PixRGB<byte>(0,0,0); 
  itsRoadColorFiltered = PixRGB<byte>(0,0,0); 
  itsMiddlePoint[0] = 0;
  itsMiddlePoint[1] = 0;
  itsMiddlePoint[2] = 0;
  itsMiddlePoint[3] = 0;
  itsMiddlePoint[4] = 0;
  itsRoadColorDiff = 0.0;
  itsEstMiddlePoint = Point2D<int>(0,0);
  itsEstMiddlePointFiltered = Point2D<int>(0,0);
  itsRoadColorDiffSub = PixRGB<byte>(0,0,0);
  itsOfs->addFrameDest("display");//Add default --out=display
  addSubComponent(itsOfs);
  itsWaitScreen = false;
  itsGroundTruthMode = GROUND_TRUTH_MODE;
  itsGroundTruthOnlyMode = GROUND_TRUTH_ONLY;
  itsSyntheticRoadMode = SYNTHETIC_ROAD_MODE;
	itsUsingVPD = USING_VPD;
	itsUsingCSH = USING_CSH;
  itsGroundTruthCounter = 0;
  itsPlayedGroundTruthCounter = 0;
  itsLabeledGroundTruthCounter= 0;
  itsGroundTruthErrorSum = 0;
  itsUseFloatWindow = true;
	itsDebugWinCounter = 0;

  itszNoise = Image<double> (5,5,ZEROS);
  double posVar=10.0;
  double roadColorVar=25.0;
  itszNoise.setVal(0,0,posVar*posVar);
  itszNoise.setVal(1,1,posVar*posVar);
  itszNoise.setVal(2,2,roadColorVar*roadColorVar);
  itszNoise.setVal(3,3,roadColorVar*roadColorVar);
  itszNoise.setVal(4,4,roadColorVar*roadColorVar);

  itsWin.reset();
	itsTimer.reset();
	itsGTimer.reset();
	itsGTimer.pause();
  itsSpf = SPFilter();	
}

// ######################################################################
void RG_Lane::switchDB(const std::string& dbname)
{
  //reset UKF and VPD	
  itsVanishingPointDetector = 
    rutz::shared_ptr<VanishingPointDetector>(new VanishingPointDetector());	
	itsSuperPixelRoadSegmentor =
    rutz::shared_ptr<SuperPixelRoadSegmentor>(new SuperPixelRoadSegmentor());	

  itsSpf = SPFilter();	


	if(itsRealCameraModeOpt.getVal()){
		return;	
	}
	if(dbname.compare("RoadGenerator") == 0)			
	{
		itsSyntheticRoadMode = true;
	}else
	{

		itsSyntheticRoadMode = false;
		// load image db
		LINFO("load db");
		itsConfFilename = 
			std::string(sformat("%s/%s/%s.conf",FOLDER,dbname.c_str(),dbname.c_str()));
		itsTestFilename = 
			std::string(sformat("%s/%s/%s.test",FOLDER,dbname.c_str(),dbname.c_str()));
		itsTestVPFilename = 
			std::string(sformat("%s/%s/%s.VP.test",FOLDER,dbname.c_str(),dbname.c_str()));
		itsTestCSHFilename = 
			std::string(sformat("%s/%s/%s.CSH.test",FOLDER,dbname.c_str(),dbname.c_str()));
		itsTestSPFilename = 
			std::string(sformat("%s/%s/%s.SP.test",FOLDER,dbname.c_str(),dbname.c_str()));
		openDB(dbname);  

		// get the time of day
		time_t rawtime; struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		char buffer [80];
		strftime (buffer,80, "%Y_%m_%d__%H_%M_%S",timeinfo);
		std::string startTime(buffer);

		std::string header =  
			std::string(sformat("\n###########Start %s for %s###########\n", 
						startTime.c_str(),dbname.c_str()));
		if(itsGroundTruthModeOpt.getVal())
		{
			LINFO("Default is in GT mode\n\n\n");	
			LINFO("Annotater name is %s",itsAnnotaterOpt.getVal().c_str());
			LINFO("GT Speed is %d",itsSpeedOpt.getVal());
			itsGroundTruthMode = true;
			itsImageDB.setFps(itsSpeedOpt.getVal());

			header =  std::string(sformat("\n###########Start %s for %s : Annotater %s ###########\n", 
							startTime.c_str(),dbname.c_str(),itsAnnotaterOpt.getVal().c_str()));
			writeToConfFile(header);	
			
		}
		writeToTestFile(header);
		writeToFile(header,itsTestVPFilename);
		writeToFile(header,itsTestSPFilename);
		writeToFile(header,itsTestCSHFilename);

		


	}

	//reset GroundTruth Data
	itsGroundTruthErrorSum = 0.0;
	itsGroundTruthCounter = 0;
  itsPlayedGroundTruthCounter = 0;
  itsLabeledGroundTruthCounter= 0;
}

// ######################################################################
void RG_Lane::openDB(const std::string& dbname)
{
  FILE *dbconf = fopen(itsConfFilename.c_str(),"r");
  if(dbconf == NULL){
    LFATAL("Can not open file: %s",itsConfFilename.c_str());
  }else{
    char line[512];
    while(fgets(line,sizeof line,dbconf)!= NULL)
      {
        //LINFO("Got line[%s]",line);
        //Skip the line start at#
        if(line[0] != '#')
          {
            char subdb[256],prefix[256],postfix[256];
            if(line[0] == 'G'){
              //Get ground truth data

              //GT:ID 23596,VP 226,122,BP 201,240,LBP 56,240,RBP 345,240,npx 202
              int img_id,vpi,vpj,bpi,bpj,lbpi,lbpj,rbpi,rbpj,npx;
              int ret = 
                sscanf(line,"GT:ID %d,VP %d,%d,BP %d,%d,LBP %d,%d,RBP %d,%d,npx %d",
                       &img_id,&vpi,&vpj,&bpi,&bpj,&lbpi,&lbpj,&rbpi,&rbpj,&npx);

              if(ret == 10){

                roadRegion gtRg = roadRegion(img_id,
                                             Point2D<int> (vpi,vpj),
                                             Point2D<int> (bpi,bpj),
                                             Point2D<int> (lbpi,lbpj),
                                             Point2D<int> (rbpi,rbpj),
                                             Point2D<int> (npx,0),
                                             Dims(320,240));
                itsRoadRegionDB.add(gtRg);
                //LINFO(gtRg.toString().c_str());
              }

            }else if(line[0] == 'D'){
              //Get Don't Care Frame 

              //DR:ID 23596
              int img_id;
              int ret = sscanf(line,"DR:ID %d", &img_id);

              if(ret == 1){

                roadRegion gtRg = roadRegion(img_id, Dims(320,240));
                itsRoadRegionDB.add(gtRg);
                //LINFO(gtRg.toString().c_str());
              }

            }else{
              int start,end;
              int ret = sscanf(line,"%s %d %d %s %s",subdb,&start,&end,prefix,postfix);
              //sscanf(line,"%s %d %d",subdb,&start,&end);
              if(ret == 5){
                LINFO("Got [%s],start[%d],end[%d]",subdb,start,end);
                std::string path = 
                  sformat("%s/%s/%s/%s",FOLDER,dbname.c_str(),subdb,prefix);
                imageDB tmpDB(path,postfix,start,end);
                itsRoadRegionDB = roadRegionDB(start,end);
                itsImageDB = tmpDB;
              }else if(ret == 3){
                //use default prefix & postfix

                LINFO("Got [%s],start[%d],end[%d]",subdb,start,end);
                std::string path = 
                  sformat("%s/%s/%s/%s",FOLDER,dbname.c_str(),subdb,DEFAULT_IMAGE_PREFIX);
                imageDB tmpDB(path,DEFAULT_IMAGE_POSTFIX,start,end);
                itsRoadRegionDB = roadRegionDB(start,end);
                itsImageDB = tmpDB;
              }
              //else LINFO("Can't not read: %s",line);
            }
          }			
      }
    itsImageDB.setRoadRegionDB(itsRoadRegionDB);
  }
}

// ######################################################################
RG_Lane::~RG_Lane()
{

	LINFO("Program End GT Count %d",itsLabeledGroundTruthCounter);	

	float gtt = itsGTimer.get();
	int hours = gtt/3600;
	int minutes = (gtt - hours*3600)/60;
	int seconds = gtt - hours*3600 - minutes*60;
  float secPerFrame = (itsLabeledGroundTruthCounter == 0) ? 0 : gtt/(itsLabeledGroundTruthCounter);//count how many sec to label one frame
	// get the time of day
	time_t rawtime; struct tm * timeinfo;
	time ( &rawtime );
	timeinfo = localtime ( &rawtime );
	char buffer [80];
	strftime (buffer,80, "%Y_%m_%d__%H_%M_%S",timeinfo);
	std::string startTime(buffer);
	std::string header =  
		std::string(sformat("###########End %s ######GT %d frames,%2dH:%2dM:%2dS,SPF:%f\n",
		startTime.c_str(),itsLabeledGroundTruthCounter,hours,minutes,seconds,secPerFrame));
		writeToConfFile(header);	
		writeToFile(header,itsTestCSHFilename);	
}

// ######################################################################
void RG_Lane::start1()
{ 
  if(itsRealCameraModeOpt.getVal()){
			itsUsingVPD = true;	
	} 
	
	LINFO("User input DB %s",itsSourceDB.getVal().c_str());
	if(itsSourceDB.getVal().compare("Default") == 0)			
		switchDB(DEFAULT_DBNAME);
	else
		switchDB(itsSourceDB.getVal());

}

// ######################################################################
void RG_Lane::registerTopics()
{
  // subscribe to all sensor data
  this->registerSubscription("CameraMessageTopic");
  this->registerSubscription("MotorMessageTopic");
  this->registerPublisher("MotorRequestTopic");
}

// ######################################################################
void RG_Lane::evolve()
{
	// if simulation  
	if(!itsWaitScreen)
	{      
		if(!itsRealCameraModeOpt.getVal())loadFrame();//Load frame from file instead of camera

		// find the road in front of the robot
		findRoad();



		if(itsDispImg.initialized())
		{
			itsOfs->writeRGB(itsDispImg, "display", FrameInfo("RG_nav",SRC_POS));

			//save screen shot to file
			//std::string saveFName(sformat("icra/%s/image_%05d.ppm",DEFAULT_DBNAME,itsImageDB.getCurrentImageID()));
			//Raster::WriteRGB(itsDispImg, saveFName);
		}
		itsDebugWinCounter = 0;//reset	
	}
	handleUserEvent();

	//itsOfs->writeRGB(itsDispImg, "RG_nav", FrameInfo("RG_nav",SRC_POS));
}
// ######################################################################
void RG_Lane::handleUserEvent()
{
	//handle clicks
	const nub::soft_ref<ImageDisplayStream> ids =
		itsOfs->findFrameDestType<ImageDisplayStream>();

	const rutz::shared_ptr<XWinManaged> uiwin =
		ids.is_valid()
		? ids->getWindow("display")
		: rutz::shared_ptr<XWinManaged>();
	int skipCounter = 0;
	if (uiwin.is_valid())
 {
		//Enter ground truth labeling mode by every GROUND_TRUTH_FPS frames
		if(itsGroundTruthMode)
		{
			LINFO("I am in gt mode");

			//forward to unlabeled frame
			while(itsRoadRegionDB.find(itsImageDB.getCurrentImageID()))
			{
				itsImageDB.nextImg();
				if((skipCounter++) % 100 == 0)
								LINFO("Skip.....Frame %5d",itsImageDB.getCurrentImageID());
			}
			if(!itsRoadRegionDB.find(itsImageDB.getCurrentImageID()))//find GT and DR frame
			{
				LINFO("Current region not in db..enable labeling tool gui");
				if((itsGroundTruthCounter % GROUND_TRUTH_FPS) == 0)
				{
					itsWaitScreen = true;
					//itsGroundTruthMode = true;
					groundTruthMode(uiwin);		
				}
				itsGroundTruthCounter++;
			}else{

				LINFO("Current region already in db..skip");
			}
		}
		int key = uiwin->getLastKeyPress();

		if(key != -1)
						LINFO("key %i", key);
		switch(key)
		{
			case -1:
				break;
			case 13: //g (MAC)
			case 42: //g label ground
				itsWaitScreen = true;
				itsGroundTruthMode = true;
				groundTruthMode(uiwin);		
				break;
			case 11: //f (mac)
			case 41: //f forward faster
				itsImageDB.forwardFaster();
				break;
			case 10: //d (mac) 
			case 40: //d forward slower	
				itsImageDB.forwardSlower();
				break;
			case 9:  //s (mac) 
			case 39: //s flip play direction 
				itsImageDB.flipDirection();
				break;
			case  8: //a (mac) 
			case 38: //a forward normal
				itsImageDB.forwardNormal();
				break;
			case 12: //h (mac) 
			case 43: //h switch labeling ground truth mode 
				LINFO("Enter to Ground Truth Mode ");
				itsGroundTruthMode =!itsGroundTruthMode;
				break;
				//case 12: //j (mac) 
			case 44: //j play only ground truth image
			case 46: //j (mac) 
				itsGroundTruthOnlyMode =!itsGroundTruthOnlyMode;;
				break;
			case 57: //space (mac)(n on PC) 
			case 65: //space pause/resume
				itsWaitScreen =! itsWaitScreen;
				break;
			case 14: //z (mac)
			case 52: //z switch to Anf
				switchDB(ANF);
				break;
			case 15: //x switch to SSL 
			case 53: //x switch to SSL 
				switchDB(SSL);
				break;
			case 16: //c (mac) 
			case 54: //c switch to Equad
				switchDB(EQUAD);
				break;
			case 17: //v (mac)
			case 55: //v switch to HNB 
				switchDB(HNB);
				break;
			case 19: //b (mac) 
			case 56: //b switch to RoadGenerator 
				switchDB("RoadGenerator");
				break;
			case 20: //q (mac) 
			case 24: //q Turn on/off VPD 
				itsUsingVPD =! itsUsingVPD;
				break;
			case 21: //w (mac) 
			case 25: //w Turn on/off CSH 
				itsUsingCSH =! itsUsingCSH;
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
		} 
	}
}	
// ######################################################################
void RG_Lane::loadFrame()
{
  //LINFO("Start Load Frame");
  // load teach image data
  //itsTimer.reset();
	if(itsSyntheticRoadMode)
	{
      //LINFO("Using synthetic road generator");
      itsCurrImg = itsRoadGenerator->getRoad(); 
		
	}
  else if(itsGroundTruthOnlyMode)
    {
      //LINFO("In GT only mode, find next gt img");
      itsCurrImg =  itsImageDB.nextGroundTruthImg();
    }
  else
    {
      //LINFO("Read next image");
      itsCurrImg =  itsImageDB.nextImg();      
    }


  if(itsCurrImg.getDims() != Dims(320,240))
    {
      //LINFO("Image size is not 320x240,rescale");
      itsCurrImg = rescale(itsCurrImg,Dims(320,240)) ;
    }
		
  //LDEBUG("Time for Load %f", itsTimer.get()/1000.0);
}

// ######################################################################
// ######################################################################
void RG_Lane::groundTruthMode(const rutz::shared_ptr<XWinManaged> uiwin){
	itsGTimer.resume();//start count time
  int clickCount = 0;
  int w = itsCurrImg.getWidth();
  int h = itsCurrImg.getHeight();
  Point2D<int> l1,l2,r1,r2,r3,vp,mp,bp,lbp,rbp;

  //left/right edge point, left/right/middle navigationabe point
  Point2D<int> lep,rep,lnp,rnp,mnp;

  Point2D<int> bl1 = Point2D<int>(0,2*h);//Image bottom line left
  Point2D<int> bl2 = Point2D<int>(w,2*h);//Image bottom line right
  Point2D<int> offset_2x = Point2D<int>(0,0);
  Image<PixRGB<byte> > tempImage;

  tempImage.resize(std::max(itsDispImg.getWidth(),w*3),
                   std::max(itsDispImg.getHeight(),h*2), ZEROS);
  inplacePaste(tempImage, itsDispImg, offset_2x);
  char buffer[200];
  bool save = false;
  bool dontcare = false;
  sprintf(buffer,"Ground Truth Mode");
  writeText(tempImage,Point2D<int>(0,h*2-20),
            buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),
            SimpleFont::FIXED(12));
  inplacePaste(tempImage, rescale(itsCurrImg,itsCurrImg.getDims()*2), offset_2x);

  sprintf(buffer,"Please Click Left     Top Edge");
  writeText(tempImage,Point2D<int>(0,10),buffer,PixRGB<byte>(255,255,255),
            PixRGB<byte>(0,0,255),SimpleFont::FIXED(20));
  itsOfs->writeRGB(tempImage, "display", FrameInfo("RG_nav",SRC_POS));

  //Big Ass Cancel Button
  drawFilledRect(tempImage, 
                 Rectangle(Point2D<int>(w*2+15,15), 
                           Dims(w-30, h/2-15)), PixRGB<byte>(0,255,0));//Skip
  sprintf(buffer,"Skip");
  writeText(tempImage,Point2D<int>(w*2.4,20),buffer,PixRGB<byte>(255,255,255),
            PixRGB<byte>(0,255,0),SimpleFont::FIXED(20));

  drawFilledRect(tempImage, 
                 Rectangle(Point2D<int>(w*2+15,h*0.5), 
                           Dims(w-30, h/2 - 15)), PixRGB<byte>(255,255,0));//Exit
  sprintf(buffer,"Exit");
  writeText(tempImage,Point2D<int>(w*2.4,h*0.5+20),buffer,
            PixRGB<byte>(255,255,255),PixRGB<byte>(255,128,0),SimpleFont::FIXED(20));

  drawFilledRect(tempImage, 
                 Rectangle(Point2D<int>(w*2+15,h*2.5+25), 
                           Dims(w-30, h/3-15)), PixRGB<byte>(0,255,0));//Don't Care
  sprintf(buffer,"Don't Care");
  writeText(tempImage,Point2D<int>(w*2.2,h*2.5+45),buffer,PixRGB<byte>(255,255,255),
            PixRGB<byte>(0,255,0),SimpleFont::FIXED(20));

  inplacePaste(tempImage, getMeterImage(itsCurrImg.getDims()), Point2D<int>(0,h*2));
  while(clickCount < 5){
		float gtt = itsGTimer.get();
		int hours = gtt/3600;
		int minutes = (gtt - hours*3600)/60;
		int seconds = gtt - hours*3600 - minutes*60;
		sprintf(buffer,"%2d:%2d:%2d",hours,minutes,seconds);		
		writeText(tempImage,Point2D<int>(w*2,h*2),buffer,
				PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(20));
		sprintf(buffer,"Annotater: %s",itsAnnotaterOpt.getVal().c_str());
		writeText(tempImage,Point2D<int>(w*2,h*2+50),buffer,
				PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(10));

    if (uiwin.is_valid())
      {
        int key = uiwin->getLastKeyPress();

        switch(key)
          {
          case -1:
            break;
          case 42: //g label ground
            itsWaitScreen = true;
            itsGroundTruthMode = true;
            groundTruthMode(uiwin);		
            break;
          case 41: //f forward faster
            itsImageDB.forwardFaster();
            break;
          case 40: //d forward slower	
            itsImageDB.forwardSlower();
            break;
          case 39: //s flip play direction 
            itsImageDB.flipDirection();
            break;
          case 38: //a forward normal
            itsImageDB.forwardNormal();
            break;
          case 43: //h exit ground truth mode 
            itsGroundTruthMode =!itsGroundTruthMode;;
            break;
          case 65: //space pause/resume
            itsWaitScreen =! itsWaitScreen;
            break;
          default:		
            LINFO("key %i", key);
            break;
          }

        Point2D<int> pos = uiwin->getLastMouseClick();
        if (pos.isValid())
          {
            PixRGB<byte> pixColor = itsDispImg.getVal(pos);
            LINFO("Mouse Click (%d %d) Color (%d,%d,%d) Click %d", 
                  pos.i,pos.j,pixColor.red(),pixColor.green(),pixColor.blue(),clickCount);
			

            //Check right skip & Exit botton
            if((pos.i >=w*2+15 && pos.i<= w*3-15 && pos.j >= 15 && pos.j <= h-15  )){

              //click exit
              if((pos.i >=w*2+15 && pos.i<= w*3-15 && pos.j >= h*0.5 && pos.j <= h-15  )){
                itsGroundTruthMode = false;
                itsWaitScreen = false;
								LINFO("Exit");
              }
              tempImage =  itsDispImg;
              LINFO("Skip");
              clickCount = 10;
            }
						//Check don't care button
            if((pos.i >=w*2+15 && pos.i<= w*3-15 && pos.j >= h*2.5+15 && pos.j <= h*3-15  )){
              LINFO("Don't Care");
              tempImage =  itsDispImg;
							dontcare = true;
              clickCount = 10;
						}
            switch(clickCount){
            case 0:
              l1 = pos;
              sprintf(buffer,"Please Click Left  Bottom Edge");
              writeText(tempImage,Point2D<int>(w*0,10),buffer,
                        PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,255),
                        SimpleFont::FIXED(20));
              break;
            case 1:
              l2 = pos;
              drawLine(tempImage,l1,l2,PixRGB<byte>(0,255,0),2);//screen center -Green line 
              sprintf(buffer,"Please Click Right    Top Edge");
              writeText(tempImage,Point2D<int>(w*0,10),buffer,
                        PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,255),
                        SimpleFont::FIXED(20));
              break;
            case 2: 
              sprintf(buffer,"Please Click Right Bottom Edge");
              writeText(tempImage,Point2D<int>(w*0,10),buffer,
                        PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,255),
                        SimpleFont::FIXED(20));
              r1 = pos;
              break;
            case 3: 
              r2 = pos;
              drawLine(tempImage,r1,r2,PixRGB<byte>(0,255,0),2);//screen center -Green line 
              vp = vanishPoint(l1,l2,r1,r2);
              drawCircle(tempImage,vp,5,PixRGB<byte>(20,50,255),10);
						
              r3 = vanishPoint(l2,Point2D<int>(l2.i+1,l2.j),r1,r2);//Find r3 point with same y of left line
              mp = (l2+r3)/2;//bottom of middle point 
              //drawCircle(tempImage,mp,10,PixRGB<byte>(20,50,255));
			
              //middle point of image bottom edge
              bp = vanishPoint(bl1,bl2,vp,mp);
              //drawCircle(tempImage,bp,10,PixRGB<byte>(20,50,255));

              lbp = vanishPoint(l1,l2,bl1,bl2);
              if(lbp.i < 0){
                //if left point is out of image left edge
                lep = vanishPoint(vp,lbp,Point2D<int>(0,0),Point2D<int>(0,2*h));//Left edge point :Point on image left edge
								
                drawLine(tempImage,vp,lep,PixRGB<byte>(0,255,0),2);//screen center -Green line 
                drawLine(tempImage,lep,Point2D<int>(0,2*h),PixRGB<byte>(0,255,0),2);//screen center -Green line 
                lnp = Point2D<int>(lep.i,2*h);
              }else{
							
                lnp = lbp;
                drawLine(tempImage,vp,lbp,PixRGB<byte>(0,255,0),2);//screen center -Green line 
              }

              rbp = vanishPoint(r1,r2,bl1,bl2);
              if(rbp.i > 2*w){
                //if right point is out of image right edge
                rep = vanishPoint(vp,rbp,Point2D<int>(2*w,0),Point2D<int>(2*w,2*h));//Left edge point :Point on image left edge
                drawLine(tempImage,vp,rep,PixRGB<byte>(0,255,0),2);//screen center -Green line 
                drawLine(tempImage,rep,Point2D<int>(2*w,2*h),PixRGB<byte>(0,255,0),2);//screen center -Green line 
                rnp = Point2D<int>(rep.i,2*h);
              }else{
                rnp = rbp;	
                drawLine(tempImage,vp,rbp,PixRGB<byte>(0,255,0),2);//screen center -Green line 
              }

              mnp = (lnp+rnp)/2;
 
              drawLine(tempImage,vp,bp,PixRGB<byte>(128,128,0),2);//groudtruth middle line 
              drawLine(tempImage,vp,mnp,PixRGB<byte>(255,255,0),2);//navigatable line 

              //Draw Buttons
              sprintf(buffer,"                                ");
              writeText(tempImage,Point2D<int>(w*0,0),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(20));
              writeText(tempImage,Point2D<int>(w*0,10),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(20));
              sprintf(buffer,"Save");
              writeText(tempImage,Point2D<int>(w*0.7,10),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,255,0),SimpleFont::FIXED(20));
              drawFilledRect(tempImage, Rectangle(Point2D<int>(w*2+15,h+15), Dims(w-30, h-30)), PixRGB<byte>(0,255,0));
              sprintf(buffer,"Save");
              writeText(tempImage,Point2D<int>(w*2.4,1.5*h-20),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,255,0),SimpleFont::FIXED(20));
              sprintf(buffer,"Redo");
              writeText(tempImage,Point2D<int>(w*1.0,10),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,255,0),SimpleFont::FIXED(20));
              sprintf(buffer,"Skip");
              writeText(tempImage,Point2D<int>(w*1.3,10),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,255,0),SimpleFont::FIXED(20));
						
              break;
            case 4:
		
              //redo
              if(pos.j < 55 || (pos.i >= w*2+15)){

                //handle button click
                if((pos.i >= (w*0.7-10) && pos.i < (w*1-10))||//top small save
                   (pos.i >= (w*2+15)   && pos.i <= w*3-15 && pos.j >= h+15 && pos.j <= 2*h-15  ))//right big save
                  {
                    //LINFO("Save");
                    clickCount = 6;
                    save = true;
                  }else if(pos.i >= (w*1-10) && pos.i < (w*1.3-10)){
                  //LINFO("Redo");
								
                  clickCount = -1;
                  tempImage =  itsDispImg;
                  inplacePaste(tempImage, rescale(itsCurrImg,itsCurrImg.getDims()*2), offset_2x);
                  sprintf(buffer,"Ground Truth Mode");
                  writeText(tempImage,Point2D<int>(0,h*2-20),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(20));
                  sprintf(buffer,"Please Click Left     Top Edge");
                  writeText(tempImage,Point2D<int>(0,10),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,255),SimpleFont::FIXED(20));
                }else if((pos.i >= (w*1.3-10) && pos.i < (w*1.6-10))//top small skip
                         ){
							
                  clickCount = 6;//exit click
                  tempImage =  itsDispImg;
                  //LINFO("Skip");
                }else{

                  clickCount = 3;//do notthing
                }

              }else{
                clickCount = 3;//do notthing
              }		
              break;
            default:
              break;
            }
            clickCount++;
          }  
      }
    itsOfs->writeRGB(tempImage, "display", FrameInfo("RG_nav",SRC_POS));
  }


  if(dontcare)
	{
    std::string trainGT = 
      std::string(sformat("DR:ID %d\n", itsImageDB.getCurrentImageID()));	
    LINFO("%s",trainGT.c_str());
    writeToConfFile(trainGT);
    roadRegion newRg = 
      roadRegion(itsImageDB.getCurrentImageID(), Dims(320,240)); 
    itsRoadRegionDB.add(newRg);

	}else if(save){
		itsLabeledGroundTruthCounter++;
    //Convert coords to 1x size
    Point2D<int> nvp,nbp,nlbp,nrbp,offset_1x,np,nmnp,cp;
    nvp = (vp - offset_2x)/2;
    nbp = (bp - offset_2x)/2;
    nlbp = (lbp - offset_2x)/2;
    nrbp = (rbp - offset_2x)/2;
    nmnp = (mnp - offset_2x)/2;

    //Conpute navigation point, 8 pixel above bottom
    np = vanishPoint(Point2D<int>(0,h-8),Point2D<int>(w,h-8),nvp,nbp);
    cp = vanishPoint(Point2D<int>(0,h-8),Point2D<int>(w,h-8),nvp,nmnp);

    std::string trainGT = 
      std::string(sformat("GT:ID %d,VP %d,%d,BP %d,%d,LBP %d,%d,"
                          "RBP %d,%d,npx %d,cpx %d\n",
                          itsImageDB.getCurrentImageID(),
                          nvp.i,nvp.j,nbp.i,nbp.j,nlbp.i,nlbp.j,
                          nrbp.i,nrbp.j,np.i,cp.i));	
    LINFO("%s",trainGT.c_str());
    writeToConfFile(trainGT);
    std::string testResult = 
      std::string(sformat("GT:ID %d,PT %f,VP %d,%d,BP %d,%d,LBP %d,%d,"
                          "RBP %d,%d,npx %d,est %d,error %d,error2 %d\n",
                          itsImageDB.getCurrentImageID(),
                          itsImageDB.getPercentage(),
                          nvp.i,nvp.j,nbp.i,nbp.j,nlbp.i,nlbp.j,nrbp.i,nrbp.j,np.i,
                          itsEstMiddlePoint.i,
                          np.i-itsEstMiddlePoint.i,
                          cp.i-itsEstMiddlePoint.i));	
    writeToTestFile(testResult);


    LINFO("GT - EST %d, Error 2 %d",
          np.i - itsEstMiddlePoint.i,cp.i - itsEstMiddlePoint.i);	

    tempImage = itsDispImg;
    offset_1x = Point2D<int>(2*w,h);

    roadRegion newRg = 
      roadRegion(itsImageDB.getCurrentImageID(), nvp, nbp,nlbp,nrbp, 
                 Point2D<int> (np.i,0), Dims(320,240)); 
    itsRoadRegionDB.add(newRg);

    //Draw line on orginal image
    itsGroundTruthImg = newRg.drawRoadRegion(itsGroundTruthImg);

    inplacePaste(tempImage, itsGroundTruthImg, Point2D<int>(2*w, h));
  }

  if(itsGroundTruthMode){
    //Resume the video right after saving
    itsWaitScreen = false;
  }
  //Return back to normal mode with labeled data
  itsOfs->writeRGB(tempImage, "display", FrameInfo("RG_nav",SRC_POS));
	itsGTimer.pause();
}

// ######################################################################
Image<PixRGB<byte> > RG_Lane::getSuperPixel(Image<PixRGB<byte> > img)
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
      if(itsRoadColor == PixRGB<byte>(0,0,0) || itsRoadColorFiltered == PixRGB<byte>(0,0,0)){	
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
        float min = colorDiff(itsRoadColorFiltered,color_map[0]);//use UKF color
        for(int i = 1; i < (int)color_map_count.size() ; i++)
          {
            float cd = colorDiff(itsRoadColorFiltered,color_map[i]);
            int rs = color_size_map[i];//region size
            //LINFO("Road Region Size %d",rs);
            if(cd < min && rs > 100)
              {
                min = cd;
                min_index = i;			
              }
          }
        itsRoadColorDiff = colorDiff(itsRoadColorFiltered,color_map[min_index]);

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
      itsRoadColorDiff = colorDiff(itsRoadColorFiltered,color_map[0]);
      itsRoadColorDiffSub = colorDiffSub(itsRoadColorFiltered,color_map[0]);

      //if((itsRoadColorDiff < 50.0 && color_map[0].green() > 150)||itsRoadColor == PixRGB<byte>(0,0,0)){//for outdoor grass
      //if((itsRoadColorDiff < 90.0 && color_map[0].green()<150)||itsRoadColor == PixRGB<byte>(0,0,0)){//indoor
      if((itsRoadColorDiff < 90.0)||itsRoadColor == PixRGB<byte>(0,0,0)){//general, high fail rate
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
            output.setVal(x,y,PixRGB<byte>(255,0,0));
            //LINFO("Found road pixel============");
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
            //					output.setVal(x,y,c);
          }
        }
    }
  if(!output.initialized())
    return img;

  //LINFO("Finish Road Finding");
  itsRawRoadSuperPixelImg = hybrid;
  return output;
}

// ######################################################################
void RG_Lane::findRoad()
{
  // get the current image
  its_Curr_Img_mutex.lock();
  Image<PixRGB<byte> > currImage = itsCurrImg;
  its_Curr_Img_mutex.unlock();

  if(!currImage.initialized()) return;

  // display
  int w = currImage.getWidth();
  int h = currImage.getHeight();
  Dims dims = currImage.getDims();
  itsDispImg.resize(w*3, 3*h, ZEROS);

  // 80x60
  Image<PixRGB<byte> > downSizeImg  = rescale(currImage,dims/SUPERPIXEL_DOWNSIZE_FACTOR);  

  Image<PixRGB<byte> > downSizeImg2 = rescale(currImage,dims/CSH_DOWNSIZE_FACTOR);  

  // CENTER SURROUND HISTOGRAM ROAD RECOGNITION:
  //if(itsUsingCSH) itsCSHistRoadRec->setImage(downSizeImg);  // OLD CODE
  if(itsUsingCSH) itsCSHistRoadRec->computeRoad(downSizeImg2);  

  // rescale input image
  Image<PixRGB<byte> >vpimg = currImage;

  if(itsUsingVPD)
    {
      if(w >= 320 ||w/h != 320/240)
        {
          LINFO("rescale input to 160/120");
          vpimg = rescale(currImage,160,120);
        }

      // /Compute Vanishing Point and find road        
      itsVanishingPointDetector->updateRoadColor(itsRoadColorFiltered);//FIXXX, don't know it works or not,kind work
      itsVanishingPointDetector->updateImage(vpimg);
      displayVanishingPoint(dims);
    }

  // CENTER SURROUND HISTOGRAM ROAD RECOGNITION:
  if(itsUsingCSH) displayCenterSurround(dims);


  // check for vpd result reliability
  // if it's above threshold
  if(itsUsingVPD)
    {
      if(itsVanishingPointDetector->getRoadColor() != PixRGB<byte>(0,0,0))
        {
          int vbx = itsVanishingPointDetector->getEstMiddlePoint().i;
          PixRGB<byte> vrc = itsVanishingPointDetector->getRoadColor();
          int rf = vrc.red();
          int gf = vrc.green();
          int bf = vrc.blue();

          LINFO("Vanishing Road Center %d, Road Color (%d,%d,%d)",vbx,rf,gf,bf);
          //updateUKF(itsVanishingPointDetector->getEstMiddlePoint(),itsVanishingPointDetector->getRoadColor());
          updateUKF(itsEstMiddlePointFiltered,itsVanishingPointDetector->getRoadColor());
      
          // display 
          char buffer[256]; 
          sprintf(buffer,"Using VPD %e",itsZprob);
          writeText(itsDispImg,Point2D<int>(2*w,3*h-20),buffer,
                    PixRGB<byte>(0,255,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(10));
        }
    }


  // REGION GROWING:
  // Compute Superpixel and find road region. Resize to original for display only
  Image<PixRGB<byte> > superPixelMap = rescale(getSuperPixel(downSizeImg), w,h);
  
  Image<PixRGB<byte> > roadFindingMap(w, h, ZEROS);
  //LINFO("superpixel size1 w %d h %d",superPixelMap.getWidth(),superPixelMap.getHeight());

  // Estimate Middle of Road
  if(superPixelMap.initialized())
    {
      //LINFO("SP initialized,predict UKF");
      
      // predict UKF
      itsSpf.predictState();
      //LINFO("SP inited,predict observation UKF");
      itsSpf.predictObservation(itszNoise);
      
      //LINFO("SP inited,predict UKF done");
      
      //LINFO("Find road color");
      std::vector<Point2D<int> > points;

      for(int y = 0 ; y < superPixelMap.getHeight() ; y ++)
        {
          int middle = 0, pixCount = 0;
          for(int x = 0 ; x < superPixelMap.getWidth() ; x ++)
            {
              PixRGB<byte> s = superPixelMap.getVal(x,y);

              //Find ave of middle pixel
              if(s.red()==255 && s.green()==0 && s.blue()==0)
                {
                  middle+=x; pixCount++;
                }
              roadFindingMap.setVal(x,y,s);//copy pixel from superPixelMap
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

      //LINFO("Do Middle Line finder, point size %d",(int)points.size());
      roadFindingMap = getGroundTruthData(roadFindingMap);

      drawLine
        (roadFindingMap,Point2D<int>(w/2,0),
         Point2D<int>(w/2,h),PixRGB<byte>(0,255,0),1);//Green line

      if(points.size() > 1)
        {
          Point2D<int> p1,p2;
          fitLine(points,p1,p2);
          //char buffer[20];

          //LINFO("Compute Navigation Error");
          //Compute Navigation Error
          itsEstMiddlePoint = Point2D<int>((p1.i+p2.i)/2,h-8);
          itsHighMiddlePoint = p1;

          //update UKF
					int currMd= itsEstMiddlePoint.i;
					int rf = itsRoadColor.red();
					int gf = itsRoadColor.green();
					int bf = itsRoadColor.blue();
					LINFO("Current SuperPixel Road Middle Point %d Color (%d,%d,%d)",currMd,rf,gf,bf);

          updateUKF(itsEstMiddlePoint,itsRoadColor);

          drawLine(roadFindingMap,p1,p2,PixRGB<byte>(255,255,0),3);//Yellow Middle line

          //LINFO("Compute Navigation Error Done");
        }//extra,remove it later
      itsSpf.getState(itsEstMiddlePointFiltered,itsRoadColorFiltered);
			int fvbx = itsEstMiddlePointFiltered.i;
			int rf = itsRoadColorFiltered.red();
			int gf = itsRoadColorFiltered.green();
			int bf = itsRoadColorFiltered.blue();
			LINFO("Check UKF road md %d, (%d,%d,%d)",fvbx,rf,gf,bf);
			if(fvbx > w||(rf==0 && gf==0 && bf ==0)){
				LINFO("UKF screw up......FIXXX it");
				itsEstMiddlePointFiltered = itsEstMiddlePoint;
				itsRoadColorFiltered = itsRoadColor;
				
			}




			//drawCircle(roadFindingMap,itsEstMiddlePointFiltered,15,PixRGB<byte>(20,50,255),3);//Final middle center - blue point
			//debugWin(roadFindingMap,"Road Finding Map");

      // update motor
      double error =  w/2 - itsEstMiddlePointFiltered.i;
      updateMotor(0.5,1*(error/100));
      displayNavigationError(error, roadFindingMap, superPixelMap);
      /*
      //load gt data

      //error < 0 turn right
      //error > 0 turn left
      }else//if points size < 1 
      {
      updateMotor(0.0,0.0);
      }
      */
    }

  inplacePaste(itsDispImg,roadFindingMap, Point2D<int>(w, 0));
  inplacePaste(itsDispImg, itsGroundTruthImg, Point2D<int>(2*w, 0));
  inplacePaste(itsDispImg, getMeterImage(dims), Point2D<int>(0,0));
  inplacePaste(itsDispImg, currImage, Point2D<int>(0, h));
  //superpixel initialized image
  //LINFO("End of Draw State");
}
// ######################################################################
void RG_Lane::updateUKF(Point2D<int> midPoint,PixRGB<byte>roadColor)
{
	
  //update UKF
  Image<double> z(1,5,ZEROS);
  z[0] = midPoint.i;
  z[1] = midPoint.j;
  z[2] = roadColor.red();
  z[3] = roadColor.green();
  z[4] = roadColor.blue();
	LINFO("UKF road(%d,%d) color(%d,%d,%d)",(int)z[0],(int)z[1],(int)z[2],(int)z[3],(int)z[4]);

  itsZprob = itsSpf.getLikelihood(z,Image<double>());
  //LINFO("Z prob %e",itsZprob);
  itsSpf.update(z,itszNoise);
}
// ######################################################################
Image<PixRGB<byte> > RG_Lane::getGroundTruthData(Image<PixRGB<byte> > roadFindingMap)
{
  PixRGB<byte> darkGreen = PixRGB<byte>(34,139,34);
  PixRGB<byte> green = PixRGB<byte>(0,255,0);
  int w = itsCurrImg.getWidth();
  char buffer[255];

  if(itsGroundTruthOnlyMode)
    sprintf(buffer,"Play Ground Truth Only");
  else	
    sprintf(buffer,"Ground truth");
  itsGroundTruthImg = itsCurrImg;//FIXXX
  writeText(itsGroundTruthImg,Point2D<int>(0,0),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

  if(!itsRealCameraModeOpt.getVal())
    {
			int gtError;
			//Get road parameters from generator
			if(itsSyntheticRoadMode){
					sprintf(buffer,"Left Ang %3d Right Ang %3d",itsRoadGenerator->getLeftAngle(),itsRoadGenerator->getRightAngle());
					writeText(itsGroundTruthImg,Point2D<int>(100,0),buffer, PixRGB<byte>(0,255,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
					float scale = itsCurrImg.getDims().w() /itsRoadGenerator->getGT().getDims().w();
					Point2D<int> rgvp = (itsRoadGenerator->getGT().getVp());
					rgvp*=scale;
					sprintf(buffer,"(%3d,%3d)",rgvp.i,rgvp.j);
					writeText(itsGroundTruthImg,rgvp - Point2D<int>(0,20),buffer, PixRGB<byte>(0,255,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));


				//Get road parameters from files(created by human annotater)
			}else{

				//Check ground truth db. If we have data, than we draw it
				roadRegion rTest;
				//LINFO("Try find GT data for img %d",itsImageDB.getCurrentImageID());
				if(itsRoadRegionDB.find(itsImageDB.getCurrentImageID(),rTest))
				{
					itsGroundTruthImg = (rTest.drawRoadRegion(itsGroundTruthImg));
					roadFindingMap = rTest.drawRoadRegion(roadFindingMap);

					//write combind result
					writeToTestFile(rTest.output(itsImageDB.getPercentage(),
								itsEstMiddlePointFiltered.i));

					//write vanishing point test result
					if(itsUsingVPD )
					{
						writeToFile(rTest.output(itsImageDB.getPercentage(),
									itsVanishingPointDetector->getEstMiddlePoint().i),itsTestVPFilename);
					}
					//write region detection test result
					writeToFile(rTest.output(itsImageDB.getPercentage(),
								itsEstMiddlePoint.i),itsTestSPFilename);

					// write output of Rasmussen's algorithm
					if(itsUsingCSH)
					{
									writeToFile(rTest.output(itsImageDB.getPercentage(),
																					itsCSHistRoadRec->getMiddlePoint().i*CSH_DOWNSIZE_FACTOR),itsTestCSHFilename);
					}

//					float sp = abs(rTest.getError2(itsEstMiddlePoint.i ));
//					float vp = abs(rTest.getError2(itsVanishingPointDetector->getEstMiddlePoint().i));
//					float spvp = abs(rTest.getError2(itsEstMiddlePointFiltered.i));


					//ANF vp 6 27 sp 3 16
					//HNB vp 5 15 sp 1 6
					//SSL vp 20 75 sp 17 70
					//EQ  vp15 50 sp 13 40

//					if(spvp > vp  && vp > sp){
//							LINFO("\n\nTotal is bad than SP  %f, sp < vp%f  good...total is %f\n\n",sp,vp,spvp);
//							//Raster::waitForKey();
//							itsWaitScreen = !itsWaitScreen;
//					}
//					if(vp <15  && sp > 40){
//							LINFO("\n\n\n SP good %f VP  bad %f ...total is %f\n",sp,vp,spvp);
//							itsWaitScreen = !itsWaitScreen;
//					}
					gtError = rTest.getError2(itsEstMiddlePointFiltered.i);

					itsPlayedGroundTruthCounter++;//keep tracked how many ground truth image we played



					drawLine(roadFindingMap,Point2D<int>(w/2,20),
							Point2D<int>(w/2+gtError,20),green,2);//
					itsGroundTruthErrorSum += abs(gtError);

					sprintf(buffer,"G");
					//float averageGtError = itsGroundTruthErrorSum/itsPlayedGroundTruthCounter;
					if(gtError < 0)
					{
						writeText(roadFindingMap,Point2D<int>(w/2+2,20-4),buffer,
								green,PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
					}
					else
					{
						writeText(roadFindingMap,Point2D<int>(w/2-8,20-4),buffer,
								green,PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

					}
					//sprintf(buffer,"cur %2d,avg %2.1f",gtError,averageGtError);
					//writeText(roadFindingMap,Point2D<int>(w/2-8,30),buffer,
					//		PixRGB<byte>(0,255,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
					//sprintf(buffer,"sum %2d,cnt %2d",itsGroundTruthErrorSum,
					//		itsPlayedGroundTruthCounter);
					//writeText(roadFindingMap,Point2D<int>(w/2-8,45),buffer,
					//		PixRGB<byte>(0,255,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

				}
			}
    }//real camera mode
  return roadFindingMap;
}

// #######################################	
Image<PixRGB<byte> > RG_Lane:: getMeterImage(Dims dims)
{
  Image<PixRGB<byte> > meterImg(dims, ZEROS);
  int w = dims.w();
  char buffer[255];

  //draw road color UKF
  sprintf(buffer,"Zprob %3.3e",itsZprob);
  writeText
    (meterImg,Point2D<int>(165,165),buffer,
     PixRGB<byte>(255,0,0),PixRGB<byte>(0,0,0),
     SimpleFont::FIXED(8));

  drawFilledRect(meterImg, Rectangle(Point2D<int>(0,50), Dims(100, 50)), itsRoadColor);//Current SP roadColor
  sprintf(buffer,"SP Road %d,%d,%d",itsRoadColor.red(),itsRoadColor.green(),itsRoadColor.blue());
  writeText(meterImg,Point2D<int>(0,55),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));


  drawFilledRect(meterImg, Rectangle(Point2D<int>(w/2+10,50), Dims(100, 100)), itsRoadColorFiltered);//Current UKF road Color
  sprintf(buffer,"UKF %d,%d,%d",itsRoadColorFiltered.red(),itsRoadColorFiltered.green(),itsRoadColorFiltered.blue());
  writeText(meterImg,Point2D<int>(165,55),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

  sprintf(buffer,"MidUKF %3d,%3d",itsEstMiddlePointFiltered.i,itsEstMiddlePointFiltered.j);
  writeText(meterImg,Point2D<int>(165,35),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));


  sprintf(buffer,"Middle %3d,%3d",itsEstMiddlePoint.i,itsEstMiddlePoint.j);
  writeText(meterImg,Point2D<int>(5,35),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
  drawLine(meterImg,itsHighMiddlePoint,itsEstMiddlePointFiltered,PixRGB<byte>(0,255,0),2);//from Superpixel UKF


	
	if(itsUsingVPD)
	{
		PixRGB<byte> vr = itsVanishingPointDetector->getRoadColor();
		drawFilledRect(meterImg, Rectangle(Point2D<int>(0,105), Dims(100, 50)), vr);//Current VP roadColor
		sprintf(buffer,"VP Road %d,%d,%d",vr.red(),vr.green(),vr.blue());
		writeText(meterImg,Point2D<int>(0,105),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

		drawLine(meterImg,itsHighMiddlePoint,itsVanishingPointDetector->getEstMiddlePoint(),PixRGB<byte>(0,0,255),2);//from VPD UKF
	}
  if(!itsRealCameraModeOpt.getVal())
    {

      drawFilledRect(meterImg, Rectangle(Point2D<int>(0,0), Dims(w, 23)), PixRGB<byte>(128,128,128));
      drawFilledRect(meterImg, Rectangle(Point2D<int>(0,0), Dims(w*itsImageDB.getPercentage(), 23)), PixRGB<byte>(0,255,128));
      int ptg = itsImageDB.getPercentage()*100;
      sprintf(buffer,"ID:%5d Fps:%d Time:%4.2fs %d%%",itsImageDB.getCurrentImageID(),itsImageDB.getFps(),
              itsImageDB.getCurrentTime(),ptg);

      writeText(meterImg,Point2D<int>(0,0),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
      sprintf(buffer,"Switch DB");
      writeText(meterImg,Point2D<int>(0,122),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));
      sprintf(buffer,"Z:Anf");
      writeText(meterImg,Point2D<int>(0,130),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));
      sprintf(buffer,"X:SSL");
      writeText(meterImg,Point2D<int>(0,138),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));
      sprintf(buffer,"C:Equad");
      writeText(meterImg,Point2D<int>(0,146),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));
      sprintf(buffer,"V:HNB");
      writeText(meterImg,Point2D<int>(0,154),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));
      sprintf(buffer,"B:RoadGen");
      writeText(meterImg,Point2D<int>(0,162),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

      sprintf(buffer,"F:Play Faster>>>");
      writeText(meterImg,Point2D<int>(0,170),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));
      sprintf(buffer,"D:Play Slower>");
      writeText(meterImg,Point2D<int>(0,178),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));
      sprintf(buffer,"S:Forward/Backward");
      writeText(meterImg,Point2D<int>(0,186),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));
      sprintf(buffer,"A:Default Play Speed");
      writeText(meterImg,Point2D<int>(0,194),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));
      sprintf(buffer,"Space:Pause/Resume");
      writeText(meterImg,Point2D<int>(0,202),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

      sprintf(buffer,"H:Edit GT");
      writeText(meterImg,Point2D<int>(0,214),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));
      sprintf(buffer,"J:Play GT");
      writeText(meterImg,Point2D<int>(0,222),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

    }//real camera mode

	if(itsUsingVPD) sprintf(buffer,"Q:VPD Off");
	else sprintf(buffer,"Q:VPD On");
	writeText(meterImg,Point2D<int>(165,214),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

	if(itsUsingCSH)sprintf(buffer,"W:CSH Off");
	else sprintf(buffer,"W:CSH On");
	writeText(meterImg,Point2D<int>(165,222),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

	float procTime = itsTimer.get()/1000.0F;	itsTimer.reset();
	sprintf(buffer,"Tot:%6.3fms",procTime);
	writeText(meterImg,Point2D<int>(245,0),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
	
  return meterImg;	
}
// ######################################################################
void RG_Lane:: fitLine(std::vector<Point2D<int> > points, Point2D<int>&p1,Point2D<int>&p2){
  float *line = new float[4];
  //float linearity = 0.0f;
  CvMemStorage* storage = cvCreateMemStorage(0);
  //	CvSeq* point_seq = cvCreateSeq( CV_32FC2, sizeof(CvSeq), sizeof(CvPoint2D32f), storage );
  CvPoint* cvPoints = (CvPoint*)malloc(points.size()*sizeof(Point2D<int>));
  for(uint i = 0; i < points.size(); i++){
    int x = points.at(i).i;
    int y = points.at(i).j;
    //cvSeqPush(point_seq,cvPoint2D32f(x,y));
    cvPoints[i].x = x;
    cvPoints[i].y = y;
  }
  //	linearity = myLinearity(point_seq);
  CvMat point_mat = cvMat(1,points.size(), CV_32SC2, cvPoints);
  cvFitLine(&point_mat, CV_DIST_L2,0, 0.01,0.01,line);
  //LINFO("v=(%f,%f),vy/vx=%f,(x,y)=(%f,%f), Linearity=%f\n",line[0],line[1],line[1]/line[0],line[2],line[3],linearity);
  cvReleaseMemStorage(&storage);

  double a, b, c, d, e, f,x0,y0,x1,y1;              // y = a + b*x, b is slop
  b = line[1]/ line[0];                  // y = c + d*x
  a = line[3]- b*line[2];                // y = e + f*x
  d = -1/b;
  c = points[0].j - d*points[0].i;
  f = d;
  e = points[points.size()-1].j - f*points[points.size()-1].i;

  x0 = (a-c)/(d-b);                  // x = (a-c)/(d-b)
  y0 = c+d*x0;                   // y = a + b*x
  x1 = (a-e)/(f-b);
  y1 = e+f*x1;

  p1.i = (int)x0;              
  p1.j = (int)y0;           
  p2.i = (int)x1;
  p2.j = (int)y1;
}

// ######################################################################
void RG_Lane:: updateMessage 
(const RobotSimEvents::EventMessagePtr& eMsg, const Ice::Current&)
{
  Timer tim(1000000); tim.reset();
  // camera message
  if(eMsg->ice_isA("::BeobotEvents::CameraMessage"))
    {
      // store the image
      BeobotEvents::CameraMessagePtr cameraMsg =
        BeobotEvents::CameraMessagePtr::dynamicCast(eMsg);

      int currRequestID = cameraMsg->RequestID;
      Image<PixRGB<byte> > img = Ice2Image<PixRGB<byte> >(cameraMsg->image);

      LDEBUG("Got a CameraMessage with Request ID = %d", currRequestID);
      LINFO("Got a CameraMessage with Request ID = %d", currRequestID);

      its_Curr_Img_mutex.lock();
      if(img.getDims() != Dims(320,240))
        {
          itsCurrImg = rescale(img,Dims(320,240)) ;
        }
      else
        {
          itsCurrImg = img; 
        }

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

  //LINFO("\n\nhow long: %f\n\n", tim.get()/1000.0F);
}
// ######################################################################
void RG_Lane::updateMotor(double tran, double rot)
{
  BeobotEvents::MotorRequestPtr msg = new BeobotEvents::MotorRequest;
  msg->transVel = tran;
  msg->rotVel   = rot;
  this->publish("MotorRequestTopic", msg);
  if(itsRealCameraModeOpt.getVal()){
    LINFO("[%d] Publish motor request Trans %f Rotation %f",
          itsPrevProcImgID,tran,rot);
  }
}

// ######################################################################
void RG_Lane::displayVanishingPoint(Dims dims)
{
  uint w = dims.w();
  uint h = dims.h();
	char buffer[200];
  // display results
  Image<float> confMap = itsVanishingPointDetector->getConfidenceMapImg();
  Image<PixRGB<byte> > confColorMap = itsVanishingPointDetector->getConfidenceMapImg(0.3);
  //Image<int> indexMap = itsVanishingPointDetector->getIndexMap();
  Image<float> voteMap = itsVanishingPointDetector->getVoteMapImg();



  //Image<PixRGB<byte> > oriSet = itsVanishingPointDetector->getOrientationSetDisplay();//FIXXX
  Image<PixRGB<byte> > outputIma = itsVanishingPointDetector->getOutputImage();
  Image<PixRGB<byte> > outputFiltedIma = itsVanishingPointDetector->getFilteredImage();
	Image<PixRGB<byte> > dispVoteMap = rescale((Image<PixRGB<byte> >)toRGB(normalizeFloat(voteMap, true)),dims);
  Image<float> searchMemoryMap = itsVanishingPointDetector->getSearchMemoryMap();

	int memSize = itsVanishingPointDetector->getMaskSize();
	int imaSize = confMap.getWidth()*confMap.getHeight();
	float memRatio = ((float)memSize/(float)imaSize)*100;

  sprintf(buffer,"Mask Size %4d/%4d %4.2f%%",memSize,imaSize,memRatio);
  writeText(dispVoteMap,Point2D<int>(0,0),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

  inplacePaste(itsDispImg, rescale(outputFiltedIma, dims), Point2D<int>(0, 2*h));
  inplacePaste(itsDispImg, rescale(outputIma, dims), Point2D<int>(w, 2*h));
  inplacePaste(itsDispImg, dispVoteMap, Point2D<int>(2*w, 2*h));
	//debugWin( dispVoteMap,"Vote Map");
	//debugWin( rescale((Image<PixRGB<byte> >)toRGB(normalizeFloat(confMap, true)),dims),"Conf Map");
	//debugWin( rescale((Image<PixRGB<byte> >)toRGB(normalizeFloat(searchMemoryMap, true)),dims),"Search Templete");
	//debugWin( rescale(outputFiltedIma,dims),"VP Output");
	//debugWin( rescale(confColorMap,dims),"Color Conf");
  //LINFO("Check Use VPD or not");
}

// ######################################################################
void RG_Lane::displayCenterSurround(Dims dims)
{
  uint w = dims.w();
  uint h = dims.h();

  // display results
  Image<PixRGB<byte> > result = itsCSHistRoadRec->getDisplayImage();
  Image<PixRGB<byte> > kmeans = itsCSHistRoadRec->getKMeansDisplayImage();

  inplacePaste(itsDispImg, rescale(kmeans, dims), Point2D<int>(1*w, 2*h));
  inplacePaste(itsDispImg, rescale(result, dims), Point2D<int>(2*w, 2*h));
}
// ######################################################################
void RG_Lane::displayNavigationError
(double error, Image<PixRGB<byte> > &roadFindingMap, Image<PixRGB<byte> > &superPixelMap)
{
  char buffer[256]; 
  
  Dims dims = roadFindingMap.getDims();
  uint w = dims.w();
  uint h = dims.h();
  itsRawSuperPixelImg = rescale(itsRawSuperPixelImg, dims);
  sprintf(buffer,"RAW Region Segment");
  writeText(itsRawSuperPixelImg,Point2D<int>(0,0),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
  inplacePaste(itsDispImg,itsRawSuperPixelImg, Point2D<int>(w*2, h));
      
  itsRawRoadSuperPixelImg = rescale(itsRawRoadSuperPixelImg, dims);
  sprintf(buffer,"Road SuperPixel");
  writeText(itsRawSuperPixelImg,Point2D<int>(0,0),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
  inplacePaste(itsDispImg, itsRawRoadSuperPixelImg  , Point2D<int>(w, h));
      
	LINFO("hi3, error %f",error);

  PixRGB<byte> darkGreen = PixRGB<byte>(34,139,34);
  PixRGB<byte> green  = PixRGB<byte>(0,255,0);
  PixRGB<byte> orange = PixRGB<byte>(255,165,0);
  PixRGB<byte> blue   = PixRGB<byte>(0,0,255);

  sprintf(buffer,"Road Finding Map");
  writeText(roadFindingMap,Point2D<int>(5,0),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
  //Draw float window in orange fixed window in blue
  int winTopLeftY = h -(SEARCH_WINDOW_BOTTOM+SEARCH_WINDOW_H)*SUPERPIXEL_DOWNSIZE_FACTOR; // h - 24
  int winWidth  =  SEARCH_WINDOW_W * SUPERPIXEL_DOWNSIZE_FACTOR;
  int winHeight =  SEARCH_WINDOW_H * SUPERPIXEL_DOWNSIZE_FACTOR;
	int winTopLeftX = 0;
	if(itsUseFloatWindow)
	{
				winTopLeftX = itsEstMiddlePoint.i - (SEARCH_WINDOW_W/2*SUPERPIXEL_DOWNSIZE_FACTOR); //current center -40 float center location
				if(winTopLeftX < 0) winTopLeftX = 0;
				if(winTopLeftX + winWidth > (int)w) winWidth = w - winTopLeftX -1;
				drawFilledRect(roadFindingMap, Rectangle(Point2D<int>(winTopLeftX,winTopLeftY), Dims(winWidth, winHeight)), blue);//
//				Raster::waitForKey();   
					
	}else{
				winTopLeftX = w/2-(SEARCH_WINDOW_W/2*SUPERPIXEL_DOWNSIZE_FACTOR); //w/2 -40 fixed center location
				drawRect(roadFindingMap, Rectangle(Point2D<int>(winTopLeftX,winTopLeftY), Dims(winWidth, winHeight)), orange);//
	}



  drawLine(roadFindingMap,Point2D<int>(w/2,10),Point2D<int>(w/2+error,10),darkGreen,2);//screen center -Green line 
  sprintf(buffer,"C");
  if(error < 0)
    {
      writeText(roadFindingMap,Point2D<int>(w/2+2,4),buffer,darkGreen,PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
    }else{
    writeText(roadFindingMap,Point2D<int>(w/2-8,4),buffer,darkGreen,PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
  }
  sprintf(buffer,"%1.2f",error);
  writeText(roadFindingMap,Point2D<int>(w-40,4),buffer,PixRGB<byte>(0,255,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

  //sprintf(buffer,"DF %1.2f (%d,%d,%d)",itsRoadColorDiff,itsRoadColorDiffSub.red(),itsRoadColorDiffSub.green(),itsRoadColorDiffSub.blue());
  //writeText(roadFindingMap,Point2D<int>(0,h),buffer,PixRGB<byte>(255,255,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

  sprintf(buffer,"Road Map");
  writeText(roadFindingMap,Point2D<int>(0,h),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
}
// ######################################################################
void RG_Lane::debugWin(Image<PixRGB<byte> >disp, std::string title)
{
	//rutz::shared_ptr<XWinManaged> itsWin;
	 itsDebugWinCounter ++;
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
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

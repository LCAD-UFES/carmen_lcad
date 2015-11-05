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
#include "Robots/Beobot2/LaneFollowing/RG_Lane/RG_Lane2.H"
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
#include <fstream> //for ifstream


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

#define CSH_DOWNSIZE_FACTOR          2

#define GROUND_TRUTH_MODE false
#define GROUND_TRUTH_FPS 1 //
#define GROUND_TRUTH_ONLY false 
#define SYNTHETIC_ROAD_MODE false
#define COLOR_WHITE PixRGB<byte>(255,255,255)
#define COLOR_BLACK PixRGB<byte>(0,0,0)
#define COLOR_NAVY  PixRGB<byte>(0,0,128) //for vp
#define COLOR_RED   PixRGB<byte>(255,0,0) //for sp
#define COLOR_GOLD	PixRGB<byte>(255,215,0) //for csh
#define COLOR_GREEN PixRGB<byte>(0,255,0)

#define COLOR_HOTPINK   PixRGB<byte>(255,105,180) 
#define COLOR_PURPLE    PixRGB<byte>(160,32,240)
#define COLOR_DARKGREEN PixRGB<byte>(0,100,0)

#define USING_VPD	true 
#define USING_CSH false


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

const ModelOptionDef OPT_RECORD=
{ MODOPT_ARG(std::string), "Record", &MOC_RGLANE, OPTEXP_CORE,
        "Recoard all screen shot in to specify path",
        "record", '\0', "<string>", "none"};
const ModelOptionDef OPT_STARTID=
{ MODOPT_ARG(int), "start frame", &MOC_RGLANE, OPTEXP_CORE,
        "Default start frame id",
        "id", '\0', "int", "-1"};
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
  itsSuperPixelRoadSegmenter(new SuperPixelRoadSegmenter()),
  itsRoadGenerator(new RoadGenerator(80,60)),
  itsTimer(1000000),//microseconds
  itsGTimer(1),//1 tick per sec
	itsGroundTruthModeOpt(&OPT_GROUNDTRUTHMODE, this, 0),
	itsAnnotaterOpt(&OPT_ANNOTATER, this, 0),
	itsSpeedOpt(&OPT_SPEED, this, 0),
	itsStartIdOpt(&OPT_STARTID, this, 0),
	itsSourceDB(&OPT_SOURCE, this, 0),
	itsRecordPath(&OPT_RECORD, this, 0),
	itsRealCameraModeOpt(&OPT_REALCAMERAMODE, this, 0)
{
  setModelParamVal("InputFameDims",Dims(320,240),MC_RECURSE|MC_IGNORE_MISSING);
//  addSubComponent(itsSmt);
  itsRoadColor = PixRGB<byte>(0,0,0); 
  itsRoadColorFiltered = PixRGB<byte>(0,0,0); 
  itsRoadColorDiff = 0.0;
  itsEstMiddlePoint = Point2D<int>(0,0);
  itsEstMiddlePointFiltered = Point2D<int>(0,0);

	itsSuperPixelError 		 = 999;
	itsVanishingPointError = 999;
	itsCenterSurroundError = 999;
	itsSpVpError 					 = 999;
	itsScreenGroundTruth   = 999;
	


  itsOfs->addFrameDest("display");//Add default --out=display
  addSubComponent(itsOfs);
  itsWaitScreen = false;
  itsNextFrame = false;
  itsPreviousFrame = false;
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

  itsWin.reset();
	itsTimer.reset();
	itsGTimer.reset();
	itsGTimer.pause();

	itsIcestormIP.setVal("bx7");//set default icestorm-ip to bx7	


  itsRoadFilter= RoadFilter();	
  itszNoise = Image<double> (4,4,ZEROS);

  // Initial noise matrix
  itszNoise = Image<double> (5,5,ZEROS);
  double posVar=10.0;
  double roadColorVar=25.0;
  itszNoise.setVal(0,0,posVar*posVar);
  itszNoise.setVal(1,1,posVar*posVar);
  itszNoise.setVal(2,2,roadColorVar*roadColorVar);
  itszNoise.setVal(3,3,roadColorVar*roadColorVar);
  itszNoise.setVal(4,4,roadColorVar*roadColorVar);

}

// ######################################################################
void RG_Lane::switchDB(const std::string& dbname)
{
  //reset UKF and VPD	
  itsVanishingPointDetector = 
    rutz::shared_ptr<VanishingPointDetector>(new VanishingPointDetector());	

  itsSuperPixelRoadSegmenter= 
    rutz::shared_ptr<SuperPixelRoadSegmenter>(new SuperPixelRoadSegmenter());	

	itsCSHistRoadRec = 
    rutz::shared_ptr<AppearanceContrastTrailRecognizer>(new AppearanceContrastTrailRecognizer());	

  itsRoadFilter = RoadFilter();	


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
		itsTestKonsFilename = 
			std::string(sformat("%s/%s/%s.KONS.test",FOLDER,dbname.c_str(),dbname.c_str()));
		itsTestKonsImgFolder = 
			std::string(sformat("%s/%s/KONS",FOLDER,dbname.c_str()));
		openDB(dbname);  
		openKong(dbname);  
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
void RG_Lane::openKong(const std::string& dbname)
{

  FILE *dbconf = fopen(itsTestKonsFilename.c_str(),"r");
  if(dbconf == NULL){
    LINFO("Can not open Kong's file: %s",itsTestKonsFilename.c_str());
  }else{
    char line[512];
    while(fgets(line,sizeof line,dbconf)!= NULL)
      {
        //LINFO("Got line[%s]",line);
        //Skip the line start at#
        if(line[0] != '#')
          {
            if(line[0] == 'G')
						{
              //Get ground truth data
							//GT:ID 23317,WIDTH 320,HEIGHT 240,VP 177,121,LP 47,237,RP 320,188,LANG 137.50,RANG 25.00,CPUTIME 43.21
              int img_id,vpi,vpj,lbpi,lbpj,rbpi,rbpj,npx,w,h;
							float lang,rang,cputime;
              int ret = 
                sscanf(line,"GT:ID %d,WIDTH %d,HEIGHT %d,VP %d,%d,LP %d,%d,RP %d,%d,LANG %f,RANG %f,CPUTIME %f",
                       &img_id,&w,&h,&vpi,&vpj,&lbpi,&lbpj,&rbpi,&rbpj,&lang,&rang,&cputime);

              if(ret == 12){

								Point2D<int> lbp = computeBottomPoint(Point2D<int>(vpi,vpj),lang,Dims(w,h));
								Point2D<int> rbp = computeBottomPoint(Point2D<int>(vpi,vpj),rang,Dims(w,h));
								Point2D<int> bp = (lbp+rbp)/2	;
								npx = bp.i;
                roadRegion gtRg = roadRegion(img_id,
                                             Point2D<int> (vpi,vpj),
                                             bp,
                                             lbp,
                                             rbp,
                                             Point2D<int> (npx,0),
                                             Dims(w,h));
                itsRoadRegionKONG.add(gtRg);
										
                //LINFO(gtRg.toString().c_str());
              }
							
      	}//if == 'G'
  		}//if not #
//    itsImageDB.setRoadRegionDB(itsRoadRegionDB);

		}//whiel line end

	}//if file opened
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
                itsRoadRegionKONG = roadRegionDB(start,end);
                itsImageDB = tmpDB;
              }else if(ret == 3){
                //use default prefix & postfix

                LINFO("Got [%s],start[%d],end[%d]",subdb,start,end);
                std::string path = 
                  sformat("%s/%s/%s/%s",FOLDER,dbname.c_str(),subdb,DEFAULT_IMAGE_PREFIX);
                imageDB tmpDB(path,DEFAULT_IMAGE_POSTFIX,start,end);
                itsRoadRegionDB = roadRegionDB(start,end);
                itsRoadRegionKONG = roadRegionDB(start,end);
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
void RG_Lane::start3()
{
		if(itsImageDB.seek(itsStartIdOpt.getVal())){
			//if we have set start frame, seek to that frame and pause there
			LINFO("Start at frame %d\n\n\n",itsStartIdOpt.getVal());	
			//itsWaitScreen = true;
		}	

}

// ######################################################################
void RG_Lane::registerTopics()
{
  // subscribe to all sensor data
  this->registerSubscription("CameraMessageTopic");
  this->registerSubscription("MotorMessageTopic");

  this->registerPublisher("MotorRequestTopic");
  this->registerPublisher("TraversalMapMessageTopic");
}

// ######################################################################
void RG_Lane::evolve()
{
  bool processNewFrame = false;

  // if simulation  
  if(!itsRealCameraModeOpt.getVal())
    {
      // if not waiting for user command to resume
      if(!itsWaitScreen || itsNextFrame || itsPreviousFrame)
        {      
          itsNextFrame = false;//turn off after one frame
          if(itsImageDB.seeked(itsStartIdOpt.getVal()))	
            itsWaitScreen = true;
          
          //Load frame from file
          LINFO("Sim Load Frame");
          loadFrame();

          processNewFrame = true;
        }
    }
  // real camera situation
  else
    {
      its_Curr_Img_mutex.lock();
      int currImgID = itsCurrImgID;
      its_Curr_Img_mutex.unlock();

      if(itsPrevProcImgID != currImgID) processNewFrame = true;
    }

  // only process if there is a new frame
  if(processNewFrame)
    {
      // find the road in front of the robot
      findRoad();

      if(itsDispImg.initialized())
        {
          LINFO("displaying image");
          itsOfs->writeRGB(itsDispImg, "display", FrameInfo("RG_nav",SRC_POS));

          //save screen shot to file
          if(itsRecordPath.getVal().compare("none") != 0){
            if(itsRecordPath.getVal().compare("default") == 0){
              saveScreen();
            }else{
              saveScreen(itsRecordPath.getVal());
            }
          }
        }
      itsDebugWinCounter = 0;//reset	

      if(itsPreviousFrame){
        itsImageDB.flipDirection();
        itsPreviousFrame = false;
      }
    }

 // check user interface
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
        case 133: //arrow down(mac)
        case 116: //arrow down
          //next frame
          itsNextFrame = true;
          break;
        case 134: //arrow up(mac) 
        case 111: //arrow up 
          //previous frame
          itsImageDB.flipDirection();
          itsPreviousFrame = true;
          break;
          //case 39: //o (mac),conflict with Linux 's' key
        case 32: //o Output Screen Shot
          LINFO("Save screen shot");
          saveScreen();

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
void RG_Lane::saveScreen(std::string folder)
{

  LINFO("Saving ScreenShot To image");
  //save screen shot to file
  std::string saveFName(sformat("%s/%s_image_%05d_Screen.ppm",folder.c_str(),itsSourceDB.getVal().c_str(),itsImageDB.getCurrentImageID()));
  if(itsDispImg.initialized())
    Raster::WriteRGB(itsDispImg, saveFName);
  else
    LINFO("No disp Img to save");

  std::string saveSpName(sformat("%s/%s_image_%05d_SP.ppm",	  folder.c_str(),itsSourceDB.getVal().c_str(),itsImageDB.getCurrentImageID()));
  if(itsSpOutputImg.initialized())
    Raster::WriteRGB(itsSpOutputImg,saveSpName);
  else
    LINFO("No sp Img to save");

  std::string saveVpName(sformat("%s/%s_image_%05d_VP.ppm",		folder.c_str(),itsSourceDB.getVal().c_str(),itsImageDB.getCurrentImageID()));
  if(itsVpOutputImg.initialized())
    Raster::WriteRGB(itsVpOutputImg,saveVpName);
  else
    LINFO("No vp Img to save");

  std::string saveRasName(sformat("%s/%s_image_%05d_Ras.ppm",		folder.c_str(),itsSourceDB.getVal().c_str(),itsImageDB.getCurrentImageID()));
  if(itsRasOutputImg.initialized())
    Raster::WriteRGB(itsRasOutputImg,saveRasName);
  else
    LINFO("No Ras Img to save");

  std::string saveKmeanName(sformat("%s/%s_image_%05d_Kmean.ppm",		folder.c_str(),itsSourceDB.getVal().c_str(),itsImageDB.getCurrentImageID()));
  if(itsKmeanOutputImg.initialized())
    Raster::WriteRGB(itsKmeanOutputImg,saveKmeanName);
  else
    LINFO("No Kmean Img to save");

  std::string saveKongName(sformat("%s/%s_image_%05d_Kong.ppm",	folder.c_str(),itsSourceDB.getVal().c_str(),itsImageDB.getCurrentImageID()));
  if(itsKongOutputImg.initialized())
    Raster::WriteRGB(itsKongOutputImg,saveKongName);
  else
    LINFO("No Kong Img to save");

  std::string saveTotalName(sformat("%s/%s_image_%05d_Total.ppm",	folder.c_str(),itsSourceDB.getVal().c_str(),itsImageDB.getCurrentImageID()));
  if(itsTotalOutputImg.initialized())
    Raster::WriteRGB(itsTotalOutputImg,saveTotalName);
  else
    LINFO("No Total Img to save");

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
    np = vanishPoint(Point2D<int>(0,h-8),Point2D<int>(w,h-8),nvp,nbp);// original center point
    cp = vanishPoint(Point2D<int>(0,h-8),Point2D<int>(w,h-8),nvp,nmnp);// screen crop center point

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
    itsGroundTruthImg = newRg.drawRoadRegion(itsCurrImg);

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
void RG_Lane::findRoad()
{
				
			LINFO("suck 0");
				// get the current image
				its_Curr_Img_mutex.lock();
				Image<PixRGB<byte> > currImage = itsCurrImg;
                                int currImgID =  itsCurrImgID; 
				its_Curr_Img_mutex.unlock();

			LINFO("suck 1");
				if(!currImage.initialized()) return;

			LINFO("suck 2");
				// display
				int w = currImage.getWidth();
				int h = currImage.getHeight();
				Dims dims = currImage.getDims();
				itsDispImg.resize(w*5, 2*h, ZEROS);

			LINFO("suck 3");
				//////////////////////////////////
				//Compute Center Surround Features
				//////////////////////////////////
				Image<PixRGB<byte> > downSizeImg2 = rescale(currImage,dims/CSH_DOWNSIZE_FACTOR);  
			LINFO("suck 4");
				// CENTER SURROUND HISTOGRAM ROAD RECOGNITION:
				//if(itsUsingCSH) itsCSHistRoadRec->setImage(downSizeImg);  // OLD CODE
				if(itsUsingCSH) itsCSHistRoadRec->computeRoad(downSizeImg2);  

			LINFO("suck 5");
				//////////////////////////////////
				//Compute Vanishing Point Features
				//////////////////////////////////
				// rescale input image
				Image<PixRGB<byte> >vpimg = currImage;

			LINFO("suck 6");
				if(itsUsingVPD)
				{
								if(w >= 320 ||w/h != 320/240)
								{
												LINFO("rescale input to 160/120");
												vpimg = rescale(currImage,160,120);
								}
								// /Compute Vanishing Point and find road        
								//itsVanishingPointDetector->updateRoadColor(itsRoadColorFiltered);//FIXXX, don't know it works or not,kind work
								itsVanishingPointDetector->updateImage(vpimg);
								itsVanishingPointDetector->computeOutputImage(dims);//just compute the output image
				}
			LINFO("suck 7");
				// CENTER SURROUND HISTOGRAM ROAD RECOGNITION:
				if(itsUsingCSH) displayCenterSurround(dims);

			LINFO("suck 8");
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
												updateUKF(itsVanishingPointDetector->getEstMiddlePoint(),itsVanishingPointDetector->getRoadColor());
												//updateUKF(itsEstMiddlePointFiltered,itsVanishingPointDetector->getRoadColor());

												// display 
												//char buffer[256]; 
												//sprintf(buffer,"Using VPD %e",itsZprob);
												//writeText(itsDispImg,Point2D<int>(2*w,3*h-20),buffer,
												//PixRGB<byte>(0,255,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(10));
								}
				}

			LINFO("suck 9");


				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////
				//KONG's Vanishing

				///////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////

			if(!itsRealCameraModeOpt.getVal())
			{
				roadRegion kongRr;
				LINFO("Try find Kong data for img %d",itsImageDB.getCurrentImageID());
				if(itsRoadRegionKONG.find(itsImageDB.getCurrentImageID(),kongRr))
				{
					//LINFO("Got Kongs Data");
					Image<PixRGB<byte> > kongImg = kongRr.drawRoadRegion(currImage); 
					itsKongOutputImg = kongImg;//saving output 
					char buffer[200];
					sprintf(buffer,"Kong's Output");
					writeText(kongImg,Point2D<int>(5,0),buffer,COLOR_HOTPINK,COLOR_BLACK,SimpleFont::FIXED(8));
					std::string kongImgPath = 
						std::string(sformat("%s/%dvp1VotingMap_ini.jpg",itsTestKonsImgFolder.c_str(),itsImageDB.getCurrentImageID()));

					//check vote image file exist or not
					if(ifstream(kongImgPath.c_str())){
						Image<PixRGB<byte> > kongVoteImg =	rescale(Raster::ReadRGB(kongImgPath.c_str()),currImage.getDims());
						inplacePaste(itsDispImg, kongVoteImg, Point2D<int>(w, 0));
					}
					inplacePaste(itsDispImg, kongImg, Point2D<int>(2*w, 0));

				}
			}

				LINFO("suck 10");
				///////////////////////////////////////////////////////////////////////////////
				// REGION GROWING by Superpixel:
				// Compute Superpixel and find road region. Resize to original for display only
				////////////////////////////////////////////////////////////////////////////////
				itsSuperPixelRoadSegmenter->updateImage(currImage);
				Image<PixRGB<byte> > superPixelMap  = itsSuperPixelRoadSegmenter->getSuperPixelMap();
				itsRawSuperPixelImg = itsSuperPixelRoadSegmenter->getRawSuperPixelImg();
				itsRawRoadSuperPixelImg = itsSuperPixelRoadSegmenter->getRawRoadSuperPixelImg();

				Image<PixRGB<byte> > roadFindingMap = itsSuperPixelRoadSegmenter->getRoadFindingMap();
				itsSpOutputImg = roadFindingMap;//saving out;
				itsEstMiddlePoint = itsSuperPixelRoadSegmenter->getMiddlePoint();//FIXXX 
				itsRoadColor			= itsSuperPixelRoadSegmenter->getRoadColor();
				itsHighMiddlePoint = itsSuperPixelRoadSegmenter->getHighMiddlePoint();//p1;
				//updateUKF(itsEstMiddlePoint,itsRoadColor);//update UKF using sp's road color
				updateUKF(itsEstMiddlePoint,itsRoadColorFiltered);//do not use sp's road color,FIXXXX if not work
				//itsRoadColorFiltered = itsRoadColor;//the way to disable UKF
				//itsEstMiddlePointFiltered= itsEstMiddlePoint;//the way to disable UKF


				//feed road color filted info back to Superpixel Road Segmenter			
				itsSuperPixelRoadSegmenter->updateRoadColor(itsRoadColorFiltered);

	
				LINFO("hellooo ");
				// update motor

				

				double controlError =  w/2 - itsEstMiddlePointFiltered.i;//for the navigation error,it compare to screen center, not compare to groundtruth


				updateMotor(0.5,1*(controlError/100));
				LINFO("hellooo :control error is %f",controlError);


				itsGroundTruthImg = getGroundTruthData(itsCurrImg);
				displayNavigationError(controlError, roadFindingMap, superPixelMap);
				displayVanishingPoint(dims);
				inplacePaste(itsDispImg, getMeterImage(dims), Point2D<int>(0,0));
				inplacePaste(itsDispImg,roadFindingMap, Point2D<int>(0, h));
				inplacePaste(itsDispImg, itsGroundTruthImg, Point2D<int>(3*w, 0));
				Image<PixRGB<byte> > ri(toRGB(Image<float>(itsSuperPixelRoadSegmenter->getRoadIndexMap())));
				inplacePaste(itsDispImg, ri, Point2D<int>(0, h+10));


				LINFO("send traversal map");
				sendTraversalMap(itsSuperPixelRoadSegmenter->getRoadIndexMap());
         
				// update the latest image being processed
				itsPrevProcImgID = currImgID;

}
// ######################################################################
void RG_Lane::updateUKF(Point2D<int> midPoint,PixRGB<byte>roadColor)
{
	
	LINFO("hellooo2 ");
	itsRoadFilter.predictState();
	itsRoadFilter.predictObservation(itszNoise);
  //update UKF
  Image<double> z(1,5,ZEROS);
  z[0] = midPoint.i;
  z[1] = midPoint.j;
  z[2] = roadColor.red();
  z[3] = roadColor.green();
  z[4] = roadColor.blue();
	LINFO("UKF road(%d,%d) color(%d,%d,%d)",(int)z[0],(int)z[1],(int)z[2],(int)z[3],(int)z[4]);

				LINFO("hellooo3 ");
	
	if(midPoint.i == 0 && midPoint.j == 0) return;//skip 0,0

  itsZprob = itsRoadFilter.getLikelihood(z,Image<double>());
				LINFO("hellooo4 ");
  //LINFO("Z prob %e",itsZprob);
  itsRoadFilter.update(z,itszNoise);
				LINFO("hellooo5 ");
	itsRoadFilter.getState(itsEstMiddlePointFiltered,itsRoadColorFiltered);
}
// ######################################################################
Image<PixRGB<byte> > RG_Lane::getGroundTruthData(Image<PixRGB<byte> > inputImage)
{
  PixRGB<byte> darkGreen = PixRGB<byte>(34,139,34);
  PixRGB<byte> green = PixRGB<byte>(0,255,0);
  char buffer[255];

  Image<PixRGB<byte> > outputImage = inputImage;

  if(!itsRealCameraModeOpt.getVal())
    {
      int gtError;
      int scale = 2;//640x480
      int w = inputImage.getWidth()*scale;
      int h = inputImage.getHeight()*scale;
      //Get road parameters from generator
      if(itsSyntheticRoadMode){
        float scales = inputImage.getDims().w() /itsRoadGenerator->getGT().getDims().w();
        Point2D<int> rgvp = (itsRoadGenerator->getGT().getVp());
        rgvp*=scales;
        sprintf(buffer,"(%3d,%3d)",rgvp.i,rgvp.j);
        writeText(outputImage,rgvp - Point2D<int>(0,20),buffer, PixRGB<byte>(0,255,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
        sprintf(buffer,"Left Ang %3d Right Ang %3d",itsRoadGenerator->getLeftAngle(),itsRoadGenerator->getRightAngle());
        writeText(outputImage,Point2D<int>(100,0),buffer, PixRGB<byte>(0,255,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));


        //Get road parameters from files(created by human annotater)
      }else{

        //Check ground truth db. If we have data, than we draw it
        roadRegion rTest;
        //LINFO("Try find GT data for img %d",itsImageDB.getCurrentImageID());
        if(itsRoadRegionDB.find(itsImageDB.getCurrentImageID(),rTest))
          {








            //					itsGroundTruthImg = (rTest.drawRoadRegion(itsGroundTruthImg));
            itsTotalOutputImg = rTest.drawRoadRegion(inputImage,1,itsEstMiddlePointFiltered);//draw a line from GT_VP to bottom est road center

            outputImage= rTest.drawRoadRegion(inputImage,scale);//2x bigger ,640x480
            int w = outputImage.getWidth();
            int h = outputImage.getHeight();

            //draw sp+vp dot on gt image in purple
            drawCircle(outputImage,itsEstMiddlePointFiltered*scale,5,COLOR_PURPLE,5);
            sprintf(buffer,"T");
            writeText(outputImage,itsEstMiddlePointFiltered*scale,buffer,COLOR_WHITE,COLOR_PURPLE,SimpleFont::FIXED(8),false,ANCHOR_CENTER);

            //draw sp dot on gt image in red
            drawCircle(outputImage,itsEstMiddlePoint*scale,5,COLOR_RED,5);
            sprintf(buffer,"S");
            writeText(outputImage,itsEstMiddlePoint*scale,buffer,COLOR_WHITE,COLOR_RED,SimpleFont::FIXED(8),false,ANCHOR_CENTER);

            drawLine(outputImage,Point2D<int>(w/2,0),Point2D<int>(w/2,h),COLOR_DARKGREEN,1);//draw center line in dark green


            //write combind result
            writeToTestFile(rTest.output(itsImageDB.getPercentage(),
                                         itsEstMiddlePointFiltered.i));
            //get sp only error to ground truth
            itsSuperPixelError = rTest.getError2(itsEstMiddlePoint.i);		
					
            //get sp+vp error 
            itsSpVpError = rTest.getError2(itsEstMiddlePointFiltered.i);
            itsScreenGroundTruth = rTest.getScreenGroundTruth();
            sprintf(buffer,"GT:%3d",itsScreenGroundTruth);
            writeText(outputImage,Point2D<int>(w-100,10),buffer, COLOR_GREEN,COLOR_BLACK,SimpleFont::FIXED(10));
            drawLine(outputImage,Point2D<int>(itsScreenGroundTruth*scale,0),Point2D<int>(itsScreenGroundTruth*scale,h),COLOR_GREEN,1);//draw screen gt in green
            sprintf(buffer,"G");
            writeText(outputImage,Point2D<int>(itsScreenGroundTruth*scale,0),buffer,COLOR_BLACK,COLOR_GREEN,SimpleFont::FIXED(8));

            //write vanishing point test result
            if(itsUsingVPD )
              {
                writeToFile(rTest.output(itsImageDB.getPercentage(),
                                         itsVanishingPointDetector->getEstMiddlePoint().i),itsTestVPFilename);
                itsVanishingPointError = rTest.getError2(itsVanishingPointDetector->getEstMiddlePoint().i);
                LINFO("itsVanishingPointError %d",itsVanishingPointError);
                drawCircle(outputImage,itsVanishingPointDetector->getEstMiddlePoint()*scale -Point2D<int>(0,10),5,COLOR_NAVY,5);
                sprintf(buffer,"V");
                writeText(outputImage,itsVanishingPointDetector->getEstMiddlePoint()*scale - Point2D<int>(-2,7),buffer,COLOR_WHITE,COLOR_NAVY,SimpleFont::FIXED(8),false,ANCHOR_CENTER);
              }
            //write region detection test result
            writeToFile(rTest.output(itsImageDB.getPercentage(),
                                     itsEstMiddlePoint.i),itsTestSPFilename);

            // write output of Rasmussen's algorithm
            if(itsUsingCSH)
              {
                writeToFile(rTest.output(itsImageDB.getPercentage(),
                                         itsCSHistRoadRec->getMiddlePoint().i*CSH_DOWNSIZE_FACTOR),itsTestCSHFilename);
                itsCenterSurroundError = rTest.getError2(itsCSHistRoadRec->getMiddlePoint().i*CSH_DOWNSIZE_FACTOR);
                drawCircle(outputImage,itsCSHistRoadRec->getMiddlePoint()*CSH_DOWNSIZE_FACTOR*scale - Point2D<int>(0,10),5,COLOR_GOLD,5);
                sprintf(buffer,"C");
                writeText(outputImage,itsCSHistRoadRec->getMiddlePoint()*CSH_DOWNSIZE_FACTOR*scale  - Point2D<int>(-2,7),buffer,COLOR_NAVY,COLOR_GOLD,SimpleFont::FIXED(8),false,ANCHOR_CENTER);
              }

            //write outout of Kong's algorithm
            if(!itsRealCameraModeOpt.getVal())
              {
                roadRegion kongRr;
                if(itsRoadRegionKONG.find(itsImageDB.getCurrentImageID(),kongRr))
                  {

                    int cp = kongRr.getScreenGroundTruth();//LINFO("Kong's CP is %d, image h is %d",cp,h);
                    drawCircle(outputImage,Point2D<int>(cp*scale,h-10) ,5,COLOR_HOTPINK,5);
                    sprintf(buffer,"K");
                    writeText(outputImage,Point2D<int>(cp*scale,h-10),buffer,COLOR_WHITE,COLOR_HOTPINK,SimpleFont::FIXED(8),false,ANCHOR_CENTER);

                  }
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
            sprintf(buffer,"TotErr:%3d",gtError);
            writeText(outputImage,Point2D<int>(w-120,30),buffer, COLOR_GREEN,COLOR_BLACK,SimpleFont::FIXED(10));

            itsPlayedGroundTruthCounter++;//keep tracked how many ground truth image we played


					
            drawLine(outputImage,Point2D<int>(w/2,20),
                     Point2D<int>(w/2+gtError*scale,20),green,2);//
            itsGroundTruthErrorSum += abs(gtError);

            sprintf(buffer,"C");
            //float averageGtError = itsGroundTruthErrorSum/itsPlayedGroundTruthCounter;
            if(gtError < 0)
              {
                writeText(outputImage,Point2D<int>(w/2+2,20-4),buffer,
                          green,PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
              }
            else
              {
                writeText(outputImage,Point2D<int>(w/2-8,20-4),buffer,
                          green,PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

              }
            //if(fabs(gtError) > 60) itsWaitScreen = true;///FIXXXXXXXX ...just for testing

            //sprintf(buffer,"cur %2d,avg %2.1f",gtError,averageGtError);
            //writeText(roadFindingMap,Point2D<int>(w/2-8,30),buffer,
            //		PixRGB<byte>(0,255,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
            //sprintf(buffer,"sum %2d,cnt %2d",itsGroundTruthErrorSum,
            //		itsPlayedGroundTruthCounter);
            //writeText(roadFindingMap,Point2D<int>(w/2-8,45),buffer,
            //		PixRGB<byte>(0,255,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));



          }else{//if find gt in database
          //if no gt found,just scale it
          outputImage = rescale(inputImage,inputImage.getDims()*scale);
          itsSuperPixelError 		 = 999;
          itsVanishingPointError = 999;
          itsCenterSurroundError = 999;
          itsSpVpError					 = 999;
          itsScreenGroundTruth   = 999;
        }
      }//if not Synthetic Road Mode
      if(itsGroundTruthOnlyMode)
        sprintf(buffer,"Play Ground Truth Only");
      else	
        sprintf(buffer,"Ground truth");
      writeText(outputImage,Point2D<int>(0,0),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
      LINFO("Groundtruth image size %d %d",w,h);
    }//real camera mode
  return outputImage;
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
  drawLine(meterImg,itsHighMiddlePoint,itsEstMiddlePointFiltered,PixRGB<byte>(0,255,0),2);//from Superpixel UKF,Green


	
	if(itsUsingVPD)
	{
		PixRGB<byte> vr = itsVanishingPointDetector->getRoadColor();
		drawFilledRect(meterImg, Rectangle(Point2D<int>(0,105), Dims(100, 50)), vr);//Current VP roadColor
		sprintf(buffer,"VP Road %d,%d,%d",vr.red(),vr.green(),vr.blue());
		writeText(meterImg,Point2D<int>(0,105),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

		drawLine(meterImg,itsHighMiddlePoint,itsVanishingPointDetector->getEstMiddlePoint(),PixRGB<byte>(0,0,255),2);//from VPD UKF,blue
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

      sprintf(buffer,"H:Edit GT   UP:Last Frame");
      writeText(meterImg,Point2D<int>(0,214),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));
      sprintf(buffer,"J:Play GT   DN:Next Frame ");
      writeText(meterImg,Point2D<int>(0,222),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

    }//real camera mode

	if(itsUsingVPD) sprintf(buffer,"Q:VPD Off");
	else sprintf(buffer,"Q:VPD On");
	writeText(meterImg,Point2D<int>(165,214),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

	if(itsUsingCSH)sprintf(buffer,"W:CSH Off");
	else sprintf(buffer,"W:CSH On");
	writeText(meterImg,Point2D<int>(165,222),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));

	sprintf(buffer,"O:Save Screenshot");
	writeText(meterImg,Point2D<int>(165,186),buffer,PixRGB<byte>(255,128,128),PixRGB<byte>(0,0,0),SimpleFont::FIXED(6));
	float procTime = itsTimer.get()/1000.0F;	itsTimer.reset();
	sprintf(buffer,"Tot:%6.3fms",procTime);
	writeText(meterImg,Point2D<int>(245,0),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
	
  return meterImg;	
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
void RG_Lane::sendTraversalMap(Image<int> tmap)
{
  // FIXXXXXX: This should just be a float map
  Image<float> ftmap(tmap);

  BeobotEvents::TraversalMapMessagePtr msg = 
    new BeobotEvents::TraversalMapMessage;
  msg->tmap      = Image2Ice(ftmap);
  msg->RequestID = itsPrevProcImgID;
  this->publish("TraversalMapMessageTopic", msg);
  LINFO("[%d] Publish Traversal Map Message", itsPrevProcImgID);
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
  PixRGB<byte> green = PixRGB<byte>(0,255,0);
  // display results
  Image<float> confMap = itsVanishingPointDetector->getConfidenceMapImg();
  Image<PixRGB<byte> > confColorMap = itsVanishingPointDetector->getConfidenceMapImg(0.3);
  //Image<int> indexMap = itsVanishingPointDetector->getIndexMap();
  Image<float> voteMap = itsVanishingPointDetector->getVoteMapImg();



  //Image<PixRGB<byte> > oriSet = itsVanishingPointDetector->getOrientationSetDisplay();//FIXXX


  Image<PixRGB<byte> > outputIma = itsVanishingPointDetector->getOutputImage();
	itsVpOutputImg = outputIma;//saving output
  sprintf(buffer,"Vanishing Point Output");
  writeText(outputIma,Point2D<int>(0,0),buffer,COLOR_NAVY,COLOR_WHITE,SimpleFont::FIXED(8));
  drawLine(outputIma,Point2D<int>(w/2,0),Point2D<int>(w/2,h),COLOR_DARKGREEN,1);//screen center - darkGreen line 
  sprintf(buffer,"C");
  writeText(outputIma,Point2D<int>(w/2,0),buffer,COLOR_WHITE,COLOR_DARKGREEN,SimpleFont::FIXED(8));
	//write error number on top right
	if(itsVanishingPointError!= 999){
		LINFO("\n\n\n\nin side : itsVanishingPointError %d\n\n\n",itsVanishingPointError);
		//Raster::waitForKey();
		sprintf(buffer,"VError:%d",itsVanishingPointError);
		
		writeText(outputIma,Point2D<int>(w-80,4),buffer,PixRGB<byte>(0,255,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

		sprintf(buffer,"Loc: %d", itsVanishingPointDetector->getEstMiddlePoint().i);
		writeText(outputIma,Point2D<int>(w-80,24),buffer,PixRGB<byte>(0,255,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

		drawLine(outputIma,Point2D<int>(itsScreenGroundTruth,0),Point2D<int>(itsScreenGroundTruth,h),green,1);//screen gt -Green line 
		sprintf(buffer,"G");
		writeText(outputIma,Point2D<int>(itsScreenGroundTruth,0),buffer,COLOR_GOLD,COLOR_GREEN,SimpleFont::FIXED(8));
		itsVanishingPointError = 999;
	}
  inplacePaste(itsDispImg, outputIma, Point2D<int>(w, h));


  Image<PixRGB<byte> > outputFiltedIma = itsVanishingPointDetector->getFilteredImage();
	Image<PixRGB<byte> > dispVoteMap = rescale((Image<PixRGB<byte> >)toRGB(normalizeFloat(voteMap, true)),dims);
  Image<float> searchMemoryMap = itsVanishingPointDetector->getSearchMemoryMap();

	int memSize = itsVanishingPointDetector->getMaskSize();
	int imaSize = confMap.getWidth()*confMap.getHeight();
	float memRatio = ((float)memSize/(float)imaSize)*100;

  sprintf(buffer,"Mask Size %4d/%4d %4.2f%%",memSize,imaSize,memRatio);
  writeText(dispVoteMap,Point2D<int>(0,0),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

  //inplacePaste(itsDispImg, rescale(outputFiltedIma, dims), Point2D<int>(0, 2*h));

	//LINFO("Place vote map");
  inplacePaste(itsDispImg, dispVoteMap, Point2D<int>(w, 0));
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

  char buffer[256]; 
  PixRGB<byte> green  = PixRGB<byte>(0,255,0);
  // display results

	//get road template then convert to road region for drawing
	RoadTemplate rt = itsCSHistRoadRec->getHighestRoadTemplate();	
	//compute the scale from road template to 320x240
	int scale = w/rt.getDims().w();
	//convert RoadTemplate to roadRegion
	roadRegion rTest(0,rt.getVanishingPoint()*scale,Point2D<int> (0,0),rt.getLeftPoint()*scale,rt.getRightPoint()*scale,Point2D<int> (0,0),dims);
	LINFO("roadRegion %s,scale %d",rTest.toString().c_str(),scale);

  Image<PixRGB<byte> > cresult = rescale(itsCSHistRoadRec->getDisplayImage(),dims);
	itsRasOutputImg = cresult;//saving output
  Image<PixRGB<byte> > result = cresult;//rTest.drawRoadRegion(cresult);
  Image<PixRGB<byte> > kmeans = rescale(itsCSHistRoadRec->getKMeansDisplayImage(),dims);
	itsKmeanOutputImg = kmeans;//saving output

  sprintf(buffer,"Center Surround Output");
  writeText(result,Point2D<int>(5,0),buffer,COLOR_GOLD,COLOR_BLACK,SimpleFont::FIXED(8));

  drawLine(result,Point2D<int>(w/2,0),Point2D<int>(w/2,h),COLOR_DARKGREEN,1);//screen center - darkGreen line 
  sprintf(buffer,"C");
  writeText(result,Point2D<int>(w/2,0),buffer,COLOR_GOLD,COLOR_DARKGREEN,SimpleFont::FIXED(8));
	//write error number on top right
	if(itsCenterSurroundError != 999){
		sprintf(buffer,"CError:%d",itsCenterSurroundError);
		writeText(result,Point2D<int>(w-80,4),buffer,PixRGB<byte>(0,255,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
		drawLine(result,Point2D<int>(itsScreenGroundTruth,0),Point2D<int>(itsScreenGroundTruth,h),green,1);//screen gt -Green line 
		sprintf(buffer,"G");
		writeText(result,Point2D<int>(itsScreenGroundTruth,0),buffer,COLOR_GOLD,COLOR_GREEN,SimpleFont::FIXED(8));

		sprintf(buffer,"Loc: %d", itsCSHistRoadRec->getMiddlePoint().i*CSH_DOWNSIZE_FACTOR);
		writeText(result,Point2D<int>(w-80,24),buffer,PixRGB<byte>(0,255,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
		itsCenterSurroundError = 999;
	}
  inplacePaste(itsDispImg, kmeans , Point2D<int>(2*w, 0));
  inplacePaste(itsDispImg, result , Point2D<int>(2*w, h));
}
// ######################################################################
void RG_Lane::displayNavigationError
(double controlError, Image<PixRGB<byte> > &roadFindingMap, Image<PixRGB<byte> > &superPixelMap)
{
  char buffer[256]; 
  int hi = 0;
  Dims dims = roadFindingMap.getDims();
  uint w = dims.w();
  uint h = dims.h();
//  itsRawSuperPixelImg = rescale(itsRawSuperPixelImg, dims);
//  sprintf(buffer,"RAW Region Segment");
//  writeText(itsRawSuperPixelImg,Point2D<int>(0,0),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
//  inplacePaste(itsDispImg,itsRawSuperPixelImg, Point2D<int>(0, h));
      
//  itsRawRoadSuperPixelImg = rescale(itsRawRoadSuperPixelImg, dims);
//  sprintf(buffer,"Raw Road SuperPixel");
//  writeText(itsRawRoadSuperPixelImg,Point2D<int>(0,0),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
//  inplacePaste(itsDispImg, itsRawRoadSuperPixelImg  , Point2D<int>(w, h));
      
	LINFO("hi3, error %f",controlError);

  PixRGB<byte> darkGreen = PixRGB<byte>(34,139,34);
  PixRGB<byte> green  = PixRGB<byte>(0,255,0);
  PixRGB<byte> orange = PixRGB<byte>(255,165,0);
  PixRGB<byte> blue   = PixRGB<byte>(0,0,255);

	LINFO("hihi %d",hi++);
  sprintf(buffer,"Super Pixel Output");
  writeText(roadFindingMap,Point2D<int>(5,0),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
//  //Draw float window in orange fixed window in blue
//  int winTopLeftY = h -(SEARCH_WINDOW_BOTTOM+SEARCH_WINDOW_H)*SUPERPIXEL_DOWNSIZE_FACTOR; // h - 24
//  int winWidth  =  SEARCH_WINDOW_W * SUPERPIXEL_DOWNSIZE_FACTOR;
//  int winHeight =  SEARCH_WINDOW_H * SUPERPIXEL_DOWNSIZE_FACTOR;
//	int winTopLeftX = 0;
//	if(itsUseFloatWindow)
//	{
//				winTopLeftX = itsEstMiddlePoint.i - (SEARCH_WINDOW_W/2*SUPERPIXEL_DOWNSIZE_FACTOR); //current center -40 float center location
//				if(winTopLeftX < 0) winTopLeftX = 0;
//				if(winTopLeftX + winWidth > (int)w) winWidth = w - winTopLeftX -1;
//				drawFilledRect(roadFindingMap, Rectangle(Point2D<int>(winTopLeftX,winTopLeftY), Dims(winWidth, winHeight)), blue);//
////				Raster::waitForKey();   
//					
//	}else{
//				winTopLeftX = w/2-(SEARCH_WINDOW_W/2*SUPERPIXEL_DOWNSIZE_FACTOR); //w/2 -40 fixed center location
//				drawRect(roadFindingMap, Rectangle(Point2D<int>(winTopLeftX,winTopLeftY), Dims(winWidth, winHeight)), orange);//
//	}



	LINFO("hihi %d",hi++);
  //drawLine(roadFindingMap,Point2D<int>(w/2,10),Point2D<int>(w/2+error,10),darkGreen,2);//screen center -Green line 

	sprintf(buffer,"G");
	writeText(roadFindingMap,Point2D<int>(itsScreenGroundTruth,0),buffer,COLOR_GOLD,COLOR_GREEN,SimpleFont::FIXED(8));
  drawLine(roadFindingMap,Point2D<int>(itsScreenGroundTruth,0),Point2D<int>(itsScreenGroundTruth,h),COLOR_GREEN,1);//screen groundtruth


  drawLine(roadFindingMap,Point2D<int>(w/2,0),Point2D<int>(w/2,h),COLOR_DARKGREEN,1);//screen center - darkGreen line 
  sprintf(buffer,"C");
  if(controlError < 0)
    {
      writeText(roadFindingMap,Point2D<int>(w/2+2,4),buffer,darkGreen,PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
    }else{
    writeText(roadFindingMap,Point2D<int>(w/2-8,4),buffer,darkGreen,PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
  }

	if(itsSuperPixelError != 999){		
		sprintf(buffer,"SError:%d",itsSuperPixelError);
		writeText(roadFindingMap,Point2D<int>(w-80,4),buffer,PixRGB<byte>(0,255,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
			sprintf(buffer,"Loc: %d", itsEstMiddlePoint.i);
		writeText(roadFindingMap,Point2D<int>(w-80,24),buffer,PixRGB<byte>(0,255,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
		itsSuperPixelError = 999;
	}


  //sprintf(buffer,"DF %1.2f (%d,%d,%d)",itsRoadColorDiff,itsRoadColorDiffSub.red(),itsRoadColorDiffSub.green(),itsRoadColorDiffSub.blue());
  //writeText(roadFindingMap,Point2D<int>(0,h),buffer,PixRGB<byte>(255,255,0),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

  sprintf(buffer,"Road Map");
  writeText(roadFindingMap,Point2D<int>(0,h),buffer,PixRGB<byte>(255,255,255),PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
}
// ######################################################################
void RG_Lane::debugWin(Image<PixRGB<byte> >disp, std::string title)
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
Point2D<int> RG_Lane::computeBottomPoint(Point2D<int> point,float angle,Dims d)
{
	//tan(a) = (H - Y1) / (X2 - X1)
	//X2 = ((H-Y1) / tan(a) ) + x1
	//Angle from right to left, ex 45 deg is from middle to right bottom
  if(angle < 0){
		LINFO("Angle %3.1f is negative, add 180.00 to %3.0f",angle,angle+180);
		angle += 180;
	}
	if(angle == 0.0){
		LINFO("Angle %3.1f is too small, set to 5.00",angle);
		angle = 5.0;
	}
	int x1 = point.i,y1 = point.j,y2 = d.h();
	float x2 = ((y2 - y1) / tan((angle/180.0)*M_PI))+x1;
	//LINFO("x1,y1=(%d,%d), x2,y2=(%d,%d),angle %f",x1,y1,(int)x2,y2,angle);
	return Point2D<int>(x2,y2);
	
}


  // setupLaneRecognition();



// Image<float> calib_roadmap = 
//   applyCoordinateTransform(traversalMap, itsCalibrationMatrix,
//                       Point2D<float>(0,BEOBOT2_ROBOT_WIDTH/2);  
//                       );

// // ######################################################################
// void BeoNavigator::setupGroundMappingCalibration()
// { 
//   uint w = 80;
//   uint h = 60; 
//   float scale = 8.0F;
//   Image<float> traversableMap(w,h, ZEROS);

//   // front middle of the robot is the real world origin  
//   std::vector<Point2D<float> > imgCoord;
//   std::vector<Point2D<float> > grndCoord;

//   FILE *fp;  char inLine[200];
//   std::string fName("src/Robots/Beobot2/Navigation/camera_calib2.txt"); 
//   LINFO("calibration file: %s",fName.c_str());
//   if((fp = fopen(fName.c_str(),"rb")) == NULL) LFATAL("not found");
//   while(fgets(inLine, 200, fp) != NULL)
//     {
//       // get the calibration points
//       float i, j, x, y;
//       sscanf(inLine, "%f %f %f %f", &i, &j, &x, &y);
//       LINFO("%7.2f %7.2f -> [%6.2f %6.2f] --> [%7.2f %7.2f]", 
//             i,j, i/scale,j/scale, x,y);
//       imgCoord.push_back (Point2D<float>(i/scale, j/scale));   
//       grndCoord.push_back(Point2D<float>(x,y));
//     }
//   fclose(fp);


//   // drawLine
//   //   (traversableMap, 
//   //    Point2D<int>( int(336/scale), int(387/scale)), 
//   //    Point2D<int>( int(335/scale), int(474/scale)), 255.0F);

//   // drawLine
//   //   (traversableMap, 
//   //    Point2D<int>( int(219/scale), int(371/scale)), 
//   //    Point2D<int>( int(118/scale), int(479/scale)), 255.0F);


//   // if(itsWin.is_invalid())
//   //   itsWin.reset(new XWinManaged(Dims(4*w, 4*h), 0, 0, "Calib"));
//   // else itsWin->setDims(Dims(4*w, 4*h));
//   // itsWin->drawImage(zoomXY(traversableMap, 4),0,0); 
//   // Raster::waitForKey();


//   itsCalibrationMatrix = calibrate(imgCoord, grndCoord);  
//   // Image<float> gmap = applyTraversableMap(traversableMap, itsCalibrationMatrix);

//   // double gW, gH; uint   nH, nV;
//   // its_Local_Map_mutex.lock();
//   // itsLocalMap->getMapDimension(gW, gH, nH, nV);
//   // its_Local_Map_mutex.unlock();
//   // uint s = 10;

//   // if(itsWin.is_invalid())
//   //   itsWin.reset(new XWinManaged(Dims(s*nH, s*nV), 0, 0, "Calib"));
//   // else itsWin->setDims(Dims(s*nH, s*nV));
//   // itsWin->drawImage(zoomXY(gmap, s),0,0); 

//   // Raster::waitForKey();
// }

// // ######################################################################
// Image<float> BeoNavigator::calibrate
// (std::vector<Point2D<float> > imgCoord, 
//  std::vector<Point2D<float> > grndCoord)
// { 
//   uint n = imgCoord.size();
  
//   Image<float> A(8, 2*n, ZEROS);
//   Image<float> B(1, 2*n, ZEROS);
    
//   for(uint i = 0; i < n; i++)
//     {
//       float x = grndCoord[i].i;
//       float y = grndCoord[i].j;
//       float u = imgCoord[i].i;
//       float v = imgCoord[i].j;
        
//       A.setVal(0, i*2  , u   );
//       A.setVal(1, i*2  , v   );
//       A.setVal(2, i*2  , 1.0F);
//       A.setVal(6, i*2  , -x*u);
//       A.setVal(7, i*2  , -x*v);

//       A.setVal(3, i*2+1, u   );
//       A.setVal(4, i*2+1, v   );
//       A.setVal(5, i*2+1, 1.0F);
//       A.setVal(6, i*2+1, -y*u);
//       A.setVal(7, i*2+1, -y*v);

//       B.setVal(0, i*2  , x);
//       B.setVal(0, i*2+1, y);
//     }

//   Image<float> A_t = transpose(A); 
//   Image<float> aa = matrixInv(matrixMult(A_t,A));
//   Image<float> bb = matrixMult(A_t,B);
//   Image<float> X  = matrixMult(aa,bb); 

//   Image<float> cmatrix(3,3, ZEROS);
//   cmatrix.setVal(0,0, X.getVal(0,0));
//   cmatrix.setVal(1,0, X.getVal(0,1));
//   cmatrix.setVal(2,0, X.getVal(0,2));
//   cmatrix.setVal(0,1, X.getVal(0,3));
//   cmatrix.setVal(1,1, X.getVal(0,4));
//   cmatrix.setVal(2,1, X.getVal(0,5));
//   cmatrix.setVal(0,2, X.getVal(0,6));
//   cmatrix.setVal(1,2, X.getVal(0,7));
//   cmatrix.setVal(2,2, 1.0);

//   // float a = X.getVal(0,0);
//   // float b = X.getVal(0,1);
//   // float c = X.getVal(0,2);
//   // float d = X.getVal(0,3);
//   // float e = X.getVal(0,4);
//   // float f = X.getVal(0,5);
//   // float g = X.getVal(0,6);
//   // float h = X.getVal(0,7);

//   // float u = 55;
//   // float v = 30;
//   // float x =  900;
//   // float y = 1800;
//   // float ans_u = u*a + v*b + c - x*u*g - x*v*h;  
//   // float ans_v = u*d + v*e + f - y*u*g - y*v*h;  

//   // LINFO("%f %f %f %f --> %f %f", u,v, x,y, ans_u, ans_v);

//   // float ans_x = a*u + b*v + c;
//   // float ans_y = d*u + e*v + f;
//   // float ans_z = g*u + h*v + 1;

//   // LINFO("%f %f %f %f --> %f %f %f", u,v, x,y, ans_x/ans_z, ans_y/ans_z, ans_z);

//   return cmatrix;
// }

// // ######################################################################
// Image<float> BeoNavigator::applyCoordinateTransform
// (Image<float> traversableMap, Image<float> cmatrix, 
//  Point2D<float> sensorOriginLocation)
// { 
//   uint w = traversableMap.getWidth();
//   uint h = traversableMap.getHeight();

//   // get local map dimensions
//   double gW, gH; uint   nH, nV;
//   its_Local_Map_mutex.lock();
//   itsLocalMap->getMapDimension(gW, gH, nH, nV);
//   its_Local_Map_mutex.unlock();
//   float lmapW = gW*nH;
//   float lmapH = gH*nV;

//   float gW_2 = gW/2;
//   float gH_2 = gH/2;

//   // robot front 
//   float rfX = lmapW/2 + sensorOriginLocation.i;
//   float rfY = lmapH/2 + sensorOriginLocation.j;

//   //LINFO("map: %f x %d = %f, %f x %d = %f : rfX: %f rfY = %f", 
//   //      gW, nH, lmapW, gH, nV, lmapH, rfX, rfY);

//   Image<float> totalMap(nH,nV, ZEROS);
//   Image<float> countMap(nH,nV, ZEROS);

//   Image<float> pt(1,3, ZEROS);
//   for(uint j = 0; j < h; j++)
//     for(uint i = 0; i < w; i++)
//       {
//         // road is labeled 1.0
//         float label = traversableMap.getVal(i,j);
//         if(label == 0.0F) continue;

//         float val = 1.0F;

//         pt.setVal(0,0, i);
//         pt.setVal(0,1, j);
//         pt.setVal(0,2, 1);

//         Image<float> res =  matrixMult(cmatrix,pt);
//         float z = res.getVal(0,2);
//         float x = res.getVal(0,0)/z;
//         float y = res.getVal(0,1)/z;
                
//         // convert to local map pixel coord
//         // assume 0,0 is robot front        
//         float cx = -y + rfX;
//         float cy = -x + rfY;

//         //float value = traversableMap.getVal(i,j);
//         //if(value > 0.0)
//         //if(i == 22 && j == 52)
//         //  LINFO("%d %d -> %7.2f %7.2f ==> %7.2f %7.2f", i,j, x,y, cx,cy);

//         if(cx >= 0.0F && cx < lmapW &&
//            cy >= 0.0F && cy < lmapH   )
//           {
//             uint ii = (cx+gW_2)/lmapW*nH;
//             uint jj = (cy+gH_2)/lmapH*nV;

//             //if(x < 0)
//             //  LINFO("%d %d -> %7.2f %7.2f [%7.2f]==> %7.2f %7.2f -> %d %d", 
//             //        i,j, x,y,z, cx,cy, ii,jj);

//             if(totalMap.coordsOk(Point2D<int>(ii,jj)))
//               {
//                 totalMap.setVal(ii,jj,val);
//                 uint count = countMap.getVal(ii,jj);
//                 countMap.setVal(ii,jj, count+1);
//                 //LINFO("%3d %3d: %10.3f %10.3f->%10.3f %10.3f->%d %d", 
//                 //      i,j, x,y, cx,cy, ii,jj );

//                 // FIXXX may need more blurring 
//                 //       when using real data
//               }
//           }
//       }

//   Image<float> gMap(nH,nV, ZEROS);
//   for(uint i = 0; i < nH; i++)
//     for(uint j = 0; j < nV; j++)
//       {
//         float count = countMap.getVal(i,j);
//         if(count != 0.0)
//           {
//             //float val = totalMap.getVal(i,j);
//             //if(val > 0.0) LINFO("[%3d %3d]: %f", i,j, val/count);
//             gMap.setVal(i,j, 255.0F);//val/count);
//           }
//       }

//   return gMap;
// }

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

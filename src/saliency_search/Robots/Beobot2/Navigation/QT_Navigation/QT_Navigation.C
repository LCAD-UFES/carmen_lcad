/*!@file Robots2/Beobot2/Navigation/QT_Navigation/QT_Navigation.C */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/QT_Navigation.C
// $ $Id: QT_Navigation.C 13084 2010-03-30 02:42:00Z kai $
//
//////////////////////////////////////////////////////////////////////////

#include "Robots/Beobot2/Navigation/QT_Navigation/QT_Navigation.H"
#include "Ice/BeobotEvents.ice.H"

#include "Raster/Raster.H"
#include "Util/sformat.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "Image/ColorOps.H"   // for luminance()
#include "Image/ShapeOps.H"   // for rescale()
#include "Image/MathOps.H"    // for stdev()
#include "Image/MatrixOps.H"  // for matrixMult()
#include "Image/CutPaste.H"   // for inplacePaste()

#include "Util/Timer.H"

#include "SIFT/Keypoint.H"
#include "SIFT/VisualObject.H"
#include "SIFT/VisualObjectMatch.H"
#include "Transport/FrameInfo.H"

#include "Ice/IceImageUtils.H"

#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"


#define  TEACH_FOLDER		     "../data/logs/2010_02_26__18_32_21/image_0000000000"
#define  TEACH_START_NUMBER        14363 
#define  TEACH_END_NUMBER          15038 


//#define  TEACH_FOLDER                   "../data/logs/2009_11_12__17_30_33/image_0000000000"
//#define  TEACH_START_NUMBER        71449
//#define  TEACH_END_NUMBER          75900
#define  REPLAY_FOLDER                   "../data/logs/2010_02_26__18_33_26/image_0000000000"
#define  REPLAY_START_NUMBER       16298
#define  REPLAY_END_NUMBER         17136

#define  DFRAME                    (TEACH_START_NUMBER - REPLAY_START_NUMBER)
#define  IMAX                      1.0
#define  IMIN                      0.0

const ModelOptionCateg MOC_QT_Navigation =
  { MOC_SORTPRI_3, "QT Navigation Related Options" };
const ModelOptionDef OPT_RealCamera =
  { MODOPT_ARG(bool), "RealCamera", &MOC_QT_Navigation, OPTEXP_CORE,
    "Do we want use real camera from BeoCamera? Default will load from image files.",
    "camera", '\0', "true | false", "false" };

// ######################################################################
QT_Navigation::QT_Navigation(OptionManager& mgr,
                             const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsFftComputer(new GistEstimatorFFT(mgr)),
  itsOfs(new OutputFrameSeries(mgr)),
  itsMapImg(320,240,ZEROS),
  itsVisualObjectDB(new VisualObjectDB()),
  itsVisualObjectDB2(new VisualObjectDB()),
  itsFfn(new FeedForwardNetwork()),
  itsTimer(1000000),
  itsCurrImgID(-1),
  itsPrevProcImgID(-1),
  itsTeachImgID(TEACH_START_NUMBER +5 +300),//FIXX
  itsReplayImgID(REPLAY_START_NUMBER +300),//FIXX
  itsTransSpeed(0.0),
  itsRotSpeed(0.0),
  itsDirVector(0.0),
  itsDir(0),
  itsMilestoneErr(999.99),
  itsFalseErrRate(0.0),
  itsDirCount(0),
  itsPosition(0.0, 0.0, 0.0),
  itsError(0.0, 0.0, 0.0),
  itsRealCamera(&OPT_RealCamera, this, 0),
  itsEstop(true),
  itsReplayDBLoaded(false),
  itsIState(0.0),
  itsDState(0.0),
  itsPGain(1.0),
  itsIGain(0.01),
  itsDGain(0.0)
{
  addSubComponent(itsOfs);

  // load the teach Visual Object Database
  loadDB(sformat("%s.sift",TEACH_FOLDER));
}

// ######################################################################
void QT_Navigation::loadReplayDB()
{
  if(!itsReplayDBLoaded)
    {
      if(!itsRealCamera.getVal())
        {

          if(itsVisualObjectDB2->loadFrom
             (sformat("%s.sift",REPLAY_FOLDER), false))
            LINFO("Load Replay Database");
          else
            {
              LINFO("Can't load database");
              saveDB(REPLAY_START_NUMBER,
                     REPLAY_END_NUMBER,
                     REPLAY_FOLDER);
          }
        }
      itsReplayDBLoaded = true;
    }
}

// ######################################################################
void QT_Navigation::initFFN()
{ }

// ######################################################################
QT_Navigation::~QT_Navigation()
{ }

// ######################################################################
void QT_Navigation::start1()
{ }

// ######################################################################
void QT_Navigation::registerTopics()
{
  // subscribe to all sensor data
  this->registerSubscription("CameraMessageTopic");
  this->registerSubscription("MotorMessageTopic");
  this->registerPublisher("MotorRequestTopic");
}

// ######################################################################
void QT_Navigation::evolve()
{
  // if simulation
  if(!itsRealCamera.getVal())
    {
      navigation();
    }

  // if real camera
  else
    {
      // check if the current image is updated
      its_Curr_Img_mutex.lock();
      bool newImageFlag = (itsPrevProcImgID < itsCurrImgID);
      its_Curr_Img_mutex.unlock();

      // if so, process
      if(newImageFlag)
        {
          itsTimer.reset();
          its_Curr_Img_mutex.lock();
          itsProcImg = itsCurrImg;
          itsPrevProcImgID = itsCurrImgID;
          its_Curr_Img_mutex.unlock();
          navigation();
        }
    }

  drawState();
  itsOfs->writeRGB(itsDispImg, "QT_nav", FrameInfo("QT_nav",SRC_POS));
  if(itsEstop == false)
    {
      //Raster::waitForKey();
      //itsEstop = true;
    }
}

// ######################################################################
void QT_Navigation::recovery()
{
  int trytimes = 0;
  do{
    loadFrame();
    std::vector< rutz::shared_ptr<VisualObjectMatch> > matches;
    int nmatches =
      itsVisualObjectDB->getObjectMatches
      (itsReplayVo,matches,VOMA_SIMPLE,
       10U,
       0.5F, //keypoint distance score default 0.5F
       0.5F, //affine distance score default 0.5F
       1.0F, //minscore  default 1.0F
       4U, //min # of keypoint match
       100U, //keypoint selection thershold
       false //sort by preattentive
       );
    std::string objName;
    int closestFrame = 999999999;
    int tmpFrameID = itsTeachImgID;
    LINFO("Searching For Nearest Key Frame in the Database");
    if (nmatches > 0 )
      {
        for(int i = 0; i < nmatches; i++)
          {
            LINFO("Found [%d] frames matches Frame[%d]", nmatches, i);

            // so that we will have a ref to the last matches obj
            rutz::shared_ptr<VisualObject> obj;
            rutz::shared_ptr<VisualObjectMatch> vom;
            vom = matches[i];
            obj = vom->getVoTest();
            //      score = vom->getScore();
            //      nkeyp = vom->size();
            //      avgScore = vom->getKeypointAvgDist();
            //      affineAvgDist = vom->getAffineAvgDist();
            objName = obj->getName();
            int tmpID = atoi(objName.c_str());
            int dx = tmpID - itsTeachImgID;

            LINFO("Dist. from current teach frame[%d]=======new ID[%d]",
                  dx,tmpID);
            // find closest frame that matches
            //FIXX: we don't consider the match rate yet.
            if(dx < closestFrame)
              {
                closestFrame = dx;
                tmpFrameID = tmpID;
              }
          }
        LINFO("Recovery Found:TID[%d]RID[%d] Match[%d]",
              itsTeachImgID,itsReplayImgID,tmpFrameID);
        itsTeachImgID = tmpFrameID;
        itsEstop = true;
        itsTransSpeed = 0.8;
        loadFrame();
        computeSIFT();
      }
    else
      {
        LINFO("No Match Point,Try get new frame and try again");
        loadFrame();
        trytimes++;
      }
  }
  while(!itsEstop && trytimes < 10);
}

// ######################################################################
// The main function of navigation
void QT_Navigation::navigation()
{

  // load key frame from database
  // compute SIFT keypoint and matching
  loadFrame();
  //computeSIFT();

  // apply QT algo for direction
  double error = computeDirection2(itsTeachImgID);

        //When the mean error and stderr have too much difference
        //Then ignore the SIFT bogus match
        if(error != 0.0 && itsMatchList->size()!=0 &&
                        (itsMatchList->checkSIFTaffine() || itsFalseErrRate > 1.0)
                ){
                updateMotorPID(itsTransSpeed, itsRotSpeed,error);
                updateMotor(itsTransSpeed, itsRotSpeed);
                updateKeyframe(error);
                itsEstop = true;

      // update keyframe when reaching milestone
      updateKeyframe(error);
      itsEstop = true;
    }

  // when the mean error and stderr have too much difference
  // then ignore the SIFT bogus match
  else
    {
      itsEstop = false;
      LINFO("lost matching; start recover mechanism");
      itsTransSpeed = 0.0;
      itsRotSpeed = 0.0;
      updateMotor(itsTransSpeed, itsRotSpeed);
      //Raster::waitForKey();
      //recovery();
      //itsEstop = true;
    }
}

// ######################################################################
void QT_Navigation::loadDB(const std::string& path)
{
  // load the actual VisualObjectDatabase but don't load images
  if(itsVisualObjectDB->loadFrom(path,false))
    {
      LINFO("Load SIFT Database");
    }
  else
    {
      LINFO("SIFT Database %s not exist, create new", path.c_str());
      saveDB(TEACH_START_NUMBER,TEACH_END_NUMBER,TEACH_FOLDER);
    }
}

// ######################################################################
void QT_Navigation::saveDB(int start,int end,const std::string& path)
{
  itsVisualObjectDB.reset(new VisualObjectDB());
  for(int i = start; i < end; i++)
    {
      itsTimer.reset();
      std::string teachFileName(sformat("%s%05d.ppm",path.c_str(),i));
      Image< PixRGB<byte> > img = Raster::ReadRGB(teachFileName);
      rutz::shared_ptr<VisualObject> vo1
        (new VisualObject(sformat("%d",i),
                          sformat("%s%05d.png",path.c_str(),i),
                          rescale(img,img.getWidth()/2, img.getHeight()/2)));
      itsVisualObjectDB->addObject(vo1);
      LINFO("Time[%f] Compute SIFT for Frame %d/%d",
            itsTimer.get()/1000.0,i-start+1,end-start+1);
    }
  std::string teachDB(sformat("%s.sift",path.c_str()));
  //        itsVisualObjectDB->setName();
  itsVisualObjectDB->saveTo(teachDB);
}

// ######################################################################
void QT_Navigation::loadFrame()
{
  // load teach image data
  itsTimer.reset();
  std::string teachFileName
    (sformat("%s%05d.ppm", TEACH_FOLDER, itsTeachImgID));
  itsTeachImg = Raster::ReadRGB(teachFileName);
  itsTeachVo = itsVisualObjectDB->getObject
    (itsTeachImgID - TEACH_START_NUMBER);
  LINFO("Time for Computing SIFT on TeachImg %f",
        itsTimer.get()/1000.0);
  itsTimer.reset();

  // Load Current Replay image data

  // running from image files
  if(!itsRealCamera.getVal())
    {
      loadReplayDB();
      std::string replayFileName
        (sformat("%s%05d.ppm", REPLAY_FOLDER, itsReplayImgID));
      itsReplayImg = Raster::ReadRGB(replayFileName);

      itsReplayVo = itsVisualObjectDB2->
        getObject(itsReplayImgID - REPLAY_START_NUMBER);
      itsReplayImgID++;
    }

  // running from real camera
  else
    {
      itsReplayImg   = itsProcImg;
      itsReplayImgID = itsPrevProcImgID;
		//	LINFO("TeachVo Width = %d",itsTeachVo->getKeypointImage(1.0).getWidth());
		//	if(itsProcImg.getWidth()==itsTeachVo->getKeypointImage(1.0).getWidth()){
		//		itsReplayImg   = itsProcImg;	
		//	}else
		//	{
		//		itsReplayImg   = rescale(itsProcImg,itsTeachVo->getKeypointImage(1.0).getWidth(),
		//				itsTeachVo->getKeypointImage(1.0).getHeight());	
		//	}
      rutz::shared_ptr<VisualObject> vo2
        (new VisualObject
         ("1", "",
          rescale(itsReplayImg,itsReplayImg.getWidth()/2,
                  itsReplayImg.getHeight()/2)));
      itsReplayVo = vo2;
    }
  LINFO("Time for Computing SIFT on ReplayImg %f",
        itsTimer.get()/1000.0);
}

// ######################################################################
bool QT_Navigation::computeSIFT()
{
  itsTimer.reset();
  VisualObjectMatchAlgo voma(VOMA_SIMPLE);
  itsMatchList.reset(new VisualObjectMatch(itsTeachVo, itsReplayVo, voma));
  LINFO("=========================================");
  //LINFO("Found %u matches between Teach and Replay", match.size());

  // let's prune the matches:
  //uint np =
  itsMatchList->prune();
  //LINFO("Pruned %u outlier matches.", np);
  LINFO("Match size now is %u ", itsMatchList->size());

  // show our final affine transform:
  //        std::cerr<<match.getSIFTaffine();

  //LINFO("getKeypointAvgDist = %f", match.getKeypointAvgDist());
  //LINFO("getAffineAvgDist = %f", match.getAffineAvgDist());
  //LINFO("getScore = %f", match.getScore());
  if (itsMatchList->checkSIFTaffine() == false)
    LINFO("### Affine is too weird -- BOGUS MATCH");

  // record the time
  LINFO("Time for Matching SIFT %f",itsTimer.get()/1000.0);

  // if the sift point is valid, return true
  return (itsMatchList->size()!=0 && itsMatchList->checkSIFTaffine());
}

// ######################################################################
double QT_Navigation::computeDirection2(int teachID)
{
  int dirF = 0,dirL = 0,dirR = 0;
  rutz::shared_ptr<VisualObjectMatch> bestMatch;
  int bestMatchSize = -1;
  int bestID = 0;
  LINFO("Direction2 Start");

  // boundary condition
  if((teachID-5) < TEACH_START_NUMBER) teachID = TEACH_START_NUMBER+2;

  // looking 10 frame ahead to find best match frame
  for(int id = teachID-2; id < teachID+10; id++)
    {
      std::string teachFileName(sformat("%s%05d.ppm",TEACH_FOLDER,id));
      rutz::shared_ptr<VisualObject> vo1 =
        itsVisualObjectDB->getObject(id - TEACH_START_NUMBER);

      VisualObjectMatchAlgo voma(VOMA_SIMPLE);
      rutz::shared_ptr<VisualObjectMatch>
        match(new VisualObjectMatch(vo1, itsReplayVo, voma));
      match->prune();

      // =========================
      // find the best Keyframe
      float theta = 2.0, sx = 2.0, sy = 2.0, str = 0;

      // check if the match is valid
      if (match->checkSIFTaffine())
        {
          match->getSIFTaffine().decompose(theta, sx, sy, str);
        }
      int msize = match->size();
      if((msize > bestMatchSize && sx < 1.0 && sy < 1.0) ||
         (bestMatchSize == -1 && id==teachID))
        {
          bestMatchSize = msize;
          bestMatch = match;
          bestID = id;
        }
      LINFO("Best Match Size %d,",bestMatchSize);
      itsMatchList = bestMatch;

      int dirGain = 1; if(id == bestID) dirGain = 2;

      // compute the direction and find mean of error
      for (int i = 0; i < msize; i ++)
        {
          rutz::shared_ptr<Keypoint> refk = match->getKeypointMatch(i).refkp;
          rutz::shared_ptr<Keypoint> tstk = match->getKeypointMatch(i).tstkp;

          double ut = tstk->getX() - itsTeachImg.getWidth()/4;
          double ud = refk->getX() - itsTeachImg.getWidth()/4;

          if(ut > 0.0 && ud < 0.0)
            {
              dirR += dirGain;
            }
          else if(ut < 0.0 && ud > 0.0)
            {
              dirL += dirGain;
            }
          else if(ut > 0.0 && ut > ud)
            {
              dirR += dirGain;
            }
          else if(ut < 0.0 && ut < ud)
            {
              dirL += dirGain;
            }
          else
            {
              dirF +=dirGain;
            }
        }
    }//end for

  //=========================
  double stdx = 0.0,stdy = 0.0;
  double avgdx = 0.0,avgdy = 0.0;

  // compute the std error
  for (int i = 0; i < bestMatchSize; i ++)
    {
      // LINFO("Loading best match point %d",i);
      rutz::shared_ptr<Keypoint> refk = bestMatch->getKeypointMatch(i).refkp;
      rutz::shared_ptr<Keypoint> tstk = bestMatch->getKeypointMatch(i).tstkp;
      double dx = fabs(tstk->getX() - refk->getX())+ 0.000001;
      stdx += dx * dx;
      avgdx+= dx;

      double dy = fabs(tstk->getY() - refk->getY())+ 0.000001;
      stdy += dy * dy;
      avgdy+= dy;
      LINFO("Error! i=[%d] BestMatchSize[%d] "
            "avgdx[%f] avgdy[%f],dx[%f], dy[%f]",
            i,bestMatchSize,avgdx,avgdy,dx,dy);
    }
  if(bestMatchSize != 0)
    {
      avgdx/= bestMatchSize;
      avgdy/= bestMatchSize;
      stdx -= (avgdx*avgdx);
      stdy -= (avgdy*avgdy);
      stdx /= bestMatchSize;
      stdx  = sqrt(stdx);
      stdy /= bestMatchSize;
      stdy  = sqrt(stdy);
      LINFO("Error! BestMatchSize[%d]  avgdx[%f] avgdy[%f]",
            bestMatchSize,avgdx,avgdy);
    }
  else
    {
      avgdx = 0;
      avgdy = 0;
      stdx  = 0;
      stdy  = 0;
    }

  LINFO("stdx %f stdy %f",stdx,stdy);
  uint stdcount = 0;
  double newAvgX = 0.0,newAvgY = 0.0;

  // Pick one stddev error as the new error data
  for (int i = 0; i < bestMatchSize; i ++)
    {
      rutz::shared_ptr<Keypoint> refk = bestMatch->getKeypointMatch(i).refkp;
      rutz::shared_ptr<Keypoint> tstk = bestMatch->getKeypointMatch(i).tstkp;
      double dx = fabs(tstk->getX() - refk->getX())+ 0.000001;
      double dy = fabs(tstk->getY() - refk->getY())+ 0.000001 ;
      double ddx = fabs(dx - avgdx);
      double ddy = fabs(dy - avgdy);

      if(ddx < stdx && ddy < stdy)
        {
          newAvgX += dx;
          newAvgY += dy;
          stdcount++;
        }
    }

  if(stdcount != 0)
    {
      newAvgX/= stdcount;
      newAvgY/= stdcount;
    }
  double newAvgerr = sqrt(newAvgX*newAvgX + newAvgY*newAvgY);
  double avgerr = sqrt(avgdx*avgdx + avgdy*avgdy);
  double angle = (newAvgX*(16.0/33.0));
  itsError.x = newAvgX;
  itsError.y = newAvgY;
  itsError.z = newAvgerr;
  //FIXXX
  if(newAvgerr == 0.0)
    {
      //Raster::waitForKey();
      newAvgerr = avgerr;
      itsFalseErrRate = 0;
    }
  else
    {
      itsFalseErrRate = avgerr/newAvgerr;
    }
  LINFO("False Error Rate %1.3f",itsFalseErrRate);
  if(itsFalseErrRate == 0.0)
    {
      LINFO("Error!!!! stdcount[%d]  avgerr[%f]",
            stdcount,avgerr);
      LINFO("Error!!!! bestMatchSize[%d]  stdx[%f] stdy[%f]",
            bestMatchSize,stdx,stdy);
      //Raster::waitForKey();
    }

  // reset target keyframe if we found better match
  if(bestID > itsTeachImgID && newAvgerr < 10.0)
    itsTeachImgID = bestID;

  // compute the forward speed based on scale
  if (bestMatch->checkSIFTaffine())//Check if the match is valid
    {
      float theta = 0.0 , sx = 0.0, sy = 0.0, str = 0.0 ;
      bestMatch->getSIFTaffine().decompose(theta, sx, sy, str);
      //if the size difference is large(sx/sy << 1.0),we can speed up
      double rms = 1.0 -(sqrt(sx*sx+sy*sy)/sqrt(2));

      double gain = 1.0;
      if(rms > 0.0)
        itsTransSpeed = 0.8 + rms * gain;
      if(itsTransSpeed > 1.0)itsTransSpeed = 1.0;
      if(itsTransSpeed < 0.0)itsTransSpeed = 0.0;
    }
  else
    {
      itsTransSpeed = 0.8;
    }

  // compute turning speed
  if(dirF >= abs(dirR + dirL))
    {
      updatePosition(0);
      itsRotSpeed = 0.4;
      if(itsDir == 0)//same heading since last time
        {
          itsDirCount++;
        }
      else
        {
          itsDirCount = 0;//reset direction count
          itsDir = 0;//reset direction
        }
    }
  else if(dirR > dirL)
    {
      updatePosition(angle);
      //Average the speed
      itsRotSpeed -= newAvgX/100.0;
      itsRotSpeed /= 2.0;
      if(itsDir == 1)//same heading since last time
        {
          itsDirCount++;
        }
      else
        {
          itsDirCount = 0;//reset direction count
          itsDir = 1;//reset direction
        }
    }
  else
    {
      updatePosition(-1*angle);
      //Average the speed
      itsRotSpeed += newAvgX/100.0+1.0;
      itsRotSpeed /= 2.0;
      if(itsDir == -1)//same heading since last time
        {
          itsDirCount++;
        }else{
        itsDirCount = 0;//reset direction count
        itsDir = -1;//reset direction
      }
    }
  return newAvgerr;
}

// ######################################################################
double QT_Navigation::computeDirection()
{
  itsTimer.reset();
  int dirF = 0,dirL = 0,dirR = 0;
  double avgdx = 0.0,avgdy = 0.0;
  uint msize = itsMatchList->size();

  // compute the direction and find mean of error
  for (uint i = 0; i < msize; i ++)
    {
      rutz::shared_ptr<Keypoint> refk =
        itsMatchList->getKeypointMatch(i).refkp;
      rutz::shared_ptr<Keypoint> tstk =
        itsMatchList->getKeypointMatch(i).tstkp;

      double ut = tstk->getX() - itsTeachImg.getWidth()/4;
      double ud = refk->getX() - itsTeachImg.getWidth()/4;
      double dx = fabs(ut - ud);
      double dy = fabs(tstk->getY() - refk->getY());
      avgdx += dx;
      avgdy += dy;
      std::string dir;

      if(ut > 0.0 && ud < 0.0)
        {
          dir = std::string("RIGHT");
          dirR ++;
        }
      else if(ut < 0.0 && ud > 0.0)
        {
          dir = std::string("LEFT");
          dirL ++;
        }
      else if(ut > 0.0 && ut > ud  )
        {
          dir = std::string("RIGHT");
          dirR ++;
        }
      else if(ut < 0.0 && ut < ud)
        {
          dir = std::string("LEFT");
          dirL ++;
        }
      else
        {
          dir = std::string("Forward");
          dirF ++;
        }
      LDEBUG("MatchPoint[%d] Teach(%f,%f) Replay(%f,%f), "
             "ud:[%f],ut:[%f],dx:[%f],dy:[%f] Turn %s",
             i,
             (refk->getX()),
             (refk->getY()),
             (tstk->getX()),
             (tstk->getY()),
             ud,ut,dx,dy,dir.c_str()
             );
    }
  avgdx/= msize;
  avgdy/= msize;
  double stdx = 0.0,stdy = 0.0;

  // compute the std error
  for (uint i = 0; i < msize; i ++)
    {
      rutz::shared_ptr<Keypoint> refk =
        itsMatchList->getKeypointMatch(i).refkp;
      rutz::shared_ptr<Keypoint> tstk =
        itsMatchList->getKeypointMatch(i).tstkp;
      double ddx = fabs(tstk->getX() - refk->getX()) - avgdx;
      stdx += ddx * ddx;
      double ddy = fabs(tstk->getY() - refk->getY()) - avgdy;
      stdy += ddy * ddy;
    }
  stdx /= msize;
  //stdx = sqrt(stdx);
  stdy /= msize;
  //stdy = sqrt(stdy);
  uint stdcount = 0;
  double newAvgX = 0.0,newAvgY = 0.0;

  // pick one std error as the new error data
  for (uint i = 0; i < msize; i ++)
    {
      rutz::shared_ptr<Keypoint> refk =
        itsMatchList->getKeypointMatch(i).refkp;
      rutz::shared_ptr<Keypoint> tstk =
        itsMatchList->getKeypointMatch(i).tstkp;
      double dx = fabs(tstk->getX() - refk->getX()) ;
      double dy = fabs(tstk->getY() - refk->getY()) ;
      double ddx = fabs(dx - avgdx);
      double ddy = fabs(dy - avgdy);

      if(ddx < sqrt(stdx) && ddy < sqrt(stdy))
        {
          newAvgX += dx;
          newAvgY += dy;
          stdcount++;
        }
    }
  if(stdcount != 0)
    {
      newAvgX/=stdcount;
      newAvgY/=stdcount;
    }

  double stdeverr = sqrt(stdx+stdy);
  double avgerr = sqrt(avgdx*avgdx + avgdy*avgdy);
  double newAvgerr = sqrt(newAvgX*newAvgX + newAvgY*newAvgY);
  double df = DFRAME - (itsTeachImgID - itsReplayImgID );
  double angle = (newAvgX*(56.0/33.0))/df;
  //double angle = (newAvgX*(16.0/33.0));
  LINFO("StdError[%f],StdErrX[%f],StdErrY[%f],Frame[%1.0f]",
        stdeverr,sqrt(stdx),sqrt(stdy),df);
  LINFO("AvgErr[%f],AvgX[%f],AvgY[%f]",avgerr,avgdx,avgdy);
  LINFO("StdAvgErr[%f],StdAvgX[%f],StdAvgY[%f]",newAvgerr,newAvgX,newAvgY);
  LINFO("Forward[%d], Right[%d], Left[%d] Angle[%f]",dirF,dirR,dirL,angle);
  itsError.x = newAvgX;
  itsError.y = newAvgY;
  itsError.z = newAvgerr;
  //FIXXX
  if(newAvgerr == 0.0)
    {
      //Raster::waitForKey();
      newAvgerr = avgerr;
    }

  itsFalseErrRate = avgerr/newAvgerr;

  LINFO("Time for QT Navigation %f",itsTimer.get()/1000.0);
  return newAvgerr;
}

// ######################################################################
void QT_Navigation::updateKeyframe(double error)
{
  LINFO("itsMerror %f,new error %f", itsMilestoneErr, error);

  // check if the match is valid
  if (itsMatchList->checkSIFTaffine())
    {
      float theta, sx, sy, str;
      itsMatchList->getSIFTaffine().decompose(theta, sx, sy, str);
      LINFO("SIFT SCALE SX[%f] SY[%f] Theta[%f] Str[%f]",sx,sy,theta,str);

      //Check the frame reach the milestone image or not
      //if(itsMilestoneErr > error && (sx <1.0&&sy <1.0))
      //        itsMilestoneErr = error;
      //else
      //{
      //FIXXX: Hack,Single threshold will not work in all environment
      if(((sx >=0.9 && sy >=0.9) && error <50.0 ) && itsMatchList->size()>2){
        itsMilestoneErr = 999.99;
        itsTeachImgID += 20;//FIXXX when turning sharp,slow down the forward rate
        LINFO("*********=========Switch Keyframe=========**********");
      }
      //}
    }
}

//Given Current image frame,it will try to find proper keyframe for navation
//
// ######################################################################
int QT_Navigation::computeEntropy()
{
  return 1;
}

// ######################################################################
void QT_Navigation::drawState()
{
  uint w = 320, h = 240;

  itsDispImg.resize(w*2, 3*h, NO_INIT);
  // original image
  inplacePaste(itsDispImg, itsProcImg, Point2D<int>(0, 0));

  its_Curr_Img_mutex.lock();
  bool newImageFlag = (itsPrevProcImgID < itsCurrImgID);
  its_Curr_Img_mutex.unlock();
  if(!itsRealCamera.getVal() || newImageFlag){
    char buffer[128];
    sprintf(buffer,"img%5d",itsTeachImgID);
    Image< PixRGB<byte> > tmpImg = itsTeachImg;
    writeText(tmpImg, Point2D<int>(5,5),
              buffer, PixRGB<byte>(255,255,255),
              PixRGB<byte>(0,0,0), SimpleFont::FIXED(8));
    inplacePaste(itsDispImg, tmpImg, Point2D<int>(0, h));

		//Check the both input image have same size
//		if(itsReplayImg.getWidth() == w)
//		{
			tmpImg = itsReplayImg;	
//		}else{
//			tmpImg = rescale(itsReplayImg,w,h);
//		}
    sprintf(buffer,"img%5d",itsReplayImgID);
    writeText(itsReplayImg, Point2D<int>(5,5),
              buffer, PixRGB<byte>(255,255,255),
              PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));

    inplacePaste(itsDispImg, tmpImg, Point2D<int>(0, 2*h));
    inplacePaste(itsDispImg, itsTeachVo->getKeypointImage(1.0) ,
                 Point2D<int>(w, h));
    inplacePaste(itsDispImg, itsReplayVo->getKeypointImage(1.0),
                 Point2D<int>(w, h*1.5));
    if(itsEstop)
      {
        Image< PixRGB<byte> > mimg = itsMatchList->getMatchImage(1.0F);
        inplacePaste(itsDispImg, mimg, Point2D<int>(w*1.5, h));
      }
    double mapScale = 0.20;
    Point2D<int> drawPos
      (int(itsPosition.x*mapScale + itsMapImg.getWidth()/2),
       int(itsPosition.y*mapScale + itsMapImg.getHeight()/2));
    if(itsMapImg.coordsOk(drawPos))
      itsMapImg.setVal(drawPos, PixRGB<byte>(0,255,0));
    inplacePaste(itsDispImg, itsMapImg, Point2D<int>(w, h*2));
    inplacePaste(itsDispImg, drawInfoImg(), Point2D<int>(w, 0));
  }
}

// ######################################################################
// Only for display current heading
Image<PixRGB<byte> > QT_Navigation::drawInfoImg()
{
  char buffer[128];
  Image<PixRGB<byte> > dirImg(320,240,ZEROS);

  if(itsDir == 0)
    {
      sprintf(buffer, "|||");
    }
  else if(itsDir > 0)
    {
      sprintf(buffer, "-->  ");
    }
  else
    {
      sprintf(buffer, "<--  ");
    }
  writeText(dirImg, Point2D<int>(dirImg.getWidth()/2-20,
                                 dirImg.getHeight()/2-20),
            buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),
            SimpleFont::FIXED(20));

  sprintf(buffer, "[%d]",itsDirCount);
  writeText(dirImg, Point2D<int>(dirImg.getWidth()/2-10,
                                 dirImg.getHeight()/2+40),
            buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),
            SimpleFont::FIXED(20));

  sprintf(buffer, "False Error Rate:[%1.3f]",itsFalseErrRate);
  writeText(dirImg, Point2D<int>(10,10),
            buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),
            SimpleFont::FIXED(8));
  if(itsEstop)
    {
      sprintf(buffer, "Match:[%d]",itsMatchList->size());
      writeText(dirImg, Point2D<int>(210,10),
                buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),
                SimpleFont::FIXED(8));
    }
  sprintf(buffer, "Error:[%1.3f] X:[%1.3f] Y:[%1.3f]",
          itsError.z,itsError.x,itsError.y);
  writeText(dirImg, Point2D<int>(10,25),
            buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),
            SimpleFont::FIXED(8));

  // check if the match is valid
  if (itsEstop && itsMatchList->checkSIFTaffine())
    {
      float theta, sx, sy, str;
      itsMatchList->getSIFTaffine().decompose(theta, sx, sy, str);
      sprintf(buffer, "Affine T:[%1.3f] X:[%1.3f] Y:[%1.3f]",theta,sx,sy);
      writeText(dirImg, Point2D<int>(10,40),
                buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),
                SimpleFont::FIXED(8));
    }
  sprintf(buffer, "Motor Speed :[%1.3f] Motor Rot:[%1.3f]",
          itsTransSpeed, itsRotSpeed);
  writeText(dirImg, Point2D<int>(10,55),
            buffer, PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),
            SimpleFont::FIXED(8));
  return dirImg;
}

// ######################################################################
void QT_Navigation::updatePosition(double turn)
{
  itsDirVector += turn;
  if(itsDirVector > 360.0) itsDirVector = 0.0;
  if(itsDirVector < 0.0) itsDirVector = 360.0;

  itsPosition.x -= sin(itsDirVector*(M_PI/180.0));
  itsPosition.y -= cos(itsDirVector*(M_PI/180.0));
}

// ######################################################################
void QT_Navigation::updateMessage
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
void QT_Navigation::updateMotorPID(double tran, double rot,double error)
{
  error/=100.0;
  if(itsDir == 1)
    error *= -1.0;
  double pTerm,iTerm,dTerm;
  pTerm = itsPGain * error;
  itsIState += error;
  if(itsIState > IMAX)
    itsIState = IMAX;
  else if(itsIState < IMIN)
    itsIState = IMIN;
  iTerm = itsIGain * itsIState;
  dTerm = itsDGain * (rot - itsDState);
  itsDState = rot;
  double pid = pTerm + iTerm - dTerm;

  LINFO("P[%1.2f] I[%1.2f] D[%1.2f], Istate[%1.2f] DState[%1.2f]",
        pTerm,iTerm,dTerm,itsIState,itsDState);
  LINFO("pid[%1.2f],rot[%1.2f]",pid,rot);
  updateMotor(tran,pid);
}

// ######################################################################
void QT_Navigation::updateMotor(double tran, double rot)
{
  BeobotEvents::MotorRequestPtr msg = new BeobotEvents::MotorRequest;
  msg->transVel = tran;
  msg->rotVel   = rot;
  this->publish("MotorRequestTopic", msg);
  LINFO("[%d] Publish motor request Trans %f Rotation %f",
        itsPrevProcImgID,tran,rot);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

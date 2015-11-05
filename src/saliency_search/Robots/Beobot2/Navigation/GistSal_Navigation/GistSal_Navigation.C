/*!@file Robots2/Beobot2/Navigation/GistSal_Navigation/GistSal_Navigation.C */
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
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/GistSal_Navigation.C
// $ $Id: GistSal_Navigation.C 15310 2012-06-01 02:29:24Z itti $
//
//////////////////////////////////////////////////////////////////////////

#include "Robots/Beobot2/Navigation/GistSal_Navigation/GistSal_Navigation.H"
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

#define IMAX                      1.0
#define IMIN                      0.0
#define LOST_TIME_LIMIT            6500000
#define ROT_TIME_LIMIT             3000000
#define JUNCTION_START_DIST       8.0
#define REVERSE_COUNT_MAX         10
#define INVALID_ANGLE             -100.0
#define INVALID_DISTANCE          -200.0

#define C_PROCEDURE_INACTIVE      0
#define C_PROCEDURE_ACTIVE        1
#define C_PROCEDURE_CONCLUDING    2

#define BASE_ROT                  .2

// ######################################################################
GistSal_Navigation::GistSal_Navigation(OptionManager& mgr,
                                       const std::string& descrName,
                                       const std::string& tagName)
  :
  RobotBrainComponent(mgr, descrName, tagName),
  itsOfs(new OutputFrameSeries(mgr)),
  itsTimer(1000000),
  itsCurrImgID(-1),
  itsPrevProcImgID(-1),
  itsTransSpeed(0.0),
  itsRotSpeed(0.0),
  itsLastSetTransSpeed(0.0),
  itsLastSetRotSpeed(0.0),
  itsDir(0),
  itsIState(0.0),
  itsDState(0.0),
  itsPGain(1.0),
  itsIGain(0.01),
  itsDGain(0.0),
  itsReverseCount(0),
  itsReverseDir(false),
  itsDistToGoal(-1.0),
  itsLastFoundTimer(1000000),
  itsSearchTracker(new SalientRegionTracker(mgr)),
  itsNewMovementTracker(new SalientRegionTracker(mgr)),
  itsCurrMovementTracker(new SalientRegionTracker(mgr))
{
  addSubComponent(itsOfs);

  // tracker
  addSubComponent(itsSearchTracker);
  addSubComponent(itsNewMovementTracker);
  addSubComponent(itsCurrMovementTracker);
  itsResetSearchTracker = true;

  // assume the robot is not moving
  // until after a landmark is recognized
  itsTurnAngle    =  0.0;
  itsJunctionDist = -1.0;
  itsLastFoundTimer.reset();

  // FIX: HACK IT FIRST TO JUST GO HOME
  itsDesiredSegmentLocation       = 3;
  itsDesiredSegmentLengthTraveled = 0.95;
  itsInGoal = false;

  // this initialization is meaningless
  itsCurrentSegmentLocation       = 0;
  itsCurrentSegmentLengthTraveled = 0.0;

  itsCornerProcStatus = C_PROCEDURE_INACTIVE;
}

// ######################################################################
void GistSal_Navigation::setEnvironment
(rutz::shared_ptr<Environment> env)
{
  itsEnvironment = env;

  // from its environment: topological map
  itsTopologicalMap = env->getTopologicalMap();

  // from its environment: visual landmark database
  itsLandmarkDB = env->getLandmarkDB();
}

// ######################################################################
GistSal_Navigation::~GistSal_Navigation()
{ }

// ######################################################################
void GistSal_Navigation::start1()
{ }

// ######################################################################
void GistSal_Navigation::registerTopics()
{
  // subscribe to gist, saliency, localizer result, and motor topics
  this->registerSubscription("LandmarkTrackMessageTopic");
  this->registerSubscription("LandmarkDBSearchResultMessageTopic");
  this->registerSubscription("CurrentLocationMessageTopic");
  this->registerSubscription("MotorMessageTopic");
  this->registerSubscription("CornerMotorRequestTopic");

  this->registerPublisher("CornerLocationMessageTopic");
  this->registerPublisher("MotorRequestTopic");
}

// ######################################################################
void GistSal_Navigation::updateMessage
(const RobotSimEvents::EventMessagePtr& eMsg, const Ice::Current&)
{
  Timer timer(1000000);

  // input image, saliency, and gist features message
  if(eMsg->ice_isA("::BeobotEvents::LandmarkTrackMessage"))
    {
      // store the image
      BeobotEvents::LandmarkTrackMessagePtr lTrackMsg =
        BeobotEvents::LandmarkTrackMessagePtr::dynamicCast(eMsg);

      int currRequestID = lTrackMsg->RequestID;
      Image<PixRGB<byte> > img = Ice2Image<PixRGB<byte> >(lTrackMsg->currIma);

      LINFO("Got an lTrackMessage with Request ID = %d", currRequestID);

      // get the recieved message
      its_input_mutex.lock();
      itsCurrImg   = img;
      itsCurrImgID = currRequestID;

      // get the salient region information
      uint inputSize = lTrackMsg->salientRegions.size();
      itsCurrSalPoints.clear();
      itsCurrSalRects.clear();
      for(uint i = 0; i < inputSize; i++)
        {
          BeobotEvents::SalientRegion salReg = lTrackMsg->salientRegions[i];
          LDEBUG("M[%4d] sp[%4d,%4d] rect[%4d,%4d,%4d,%4d]",
                 i, salReg.salpt.i, salReg.salpt.j,
                 salReg.objRect.tl.i, salReg.objRect.tl.j,
                 salReg.objRect.br.i, salReg.objRect.br.j);

          Point2D<int> salpt(salReg.salpt.i, salReg.salpt.j);
          itsCurrSalPoints.push_back(salpt);

          Rectangle rect = Rectangle::tlbrO
              (salReg.objRect.tl.j, salReg.objRect.tl.i,
               salReg.objRect.br.j, salReg.objRect.br.i);
          itsCurrSalRects.push_back(rect);
       }

      // get the conspicuity maps
      itsCurrCmap.clear();
      itsCurrCmap.reset(NUM_CHANNELS);
      for(uint i = 0; i < NUM_CHANNELS; i++)
        itsCurrCmap[i] = Ice2Image<float>(lTrackMsg->conspicuityMaps[i]);

      its_input_mutex.unlock();

      LDEBUG("[input time: %f ms]\n\n", timer.get()/1000.0f);
    }

  // landmark database match found to be tracked
  else if(eMsg->ice_isA("::BeobotEvents::LandmarkDBSearchResultMessage"))
    {
      BeobotEvents::LandmarkDBSearchResultMessagePtr dbrMsg =
        BeobotEvents::LandmarkDBSearchResultMessagePtr::dynamicCast(eMsg);

      int currRequestID = dbrMsg->RequestID;

      LINFO("Got a LandmarkDBSearchResultMessage with Request ID = %d",
            currRequestID);

      its_tracker_mutex.lock();
      its_Curr_Dbr_mutex.lock();
      itsCurrImgID = currRequestID;

      // make sure there is at least 1 to be tracked
      uint numRecognized = dbrMsg->matches.size();

      // check for asynchrony
      if(numRecognized > 0)
        {
          uint mIndex = dbrMsg->matches[numRecognized - 1].inputSalRegID;
          if(mIndex >= itsSearchTracker->getNumTrackedPoints())
            {
              LINFO("\n\n\nASYNCHRONIZATION HAS OCCURED: %d %d\n\n",
                    mIndex, itsSearchTracker->getNumTrackedPoints());
              numRecognized = 0;

              // NOTE: need better solution to fix this problem
            }
        }

      // if we have a new recognized salient region
      if(numRecognized > 0)
        {
          itsNewMovementTracker->clear();

          itsNewMovementTracker->reset(numRecognized);
          itsNewMovementTracker->setCurrInputImage
            (itsSearchTracker->getCurrInputImage());
          itsNewMovementTracker->setOriginalInputImage
            (itsSearchTracker->getOriginalInputImage());

          // MOVE found Landmark from  SearchTracker to NewTracker
          itsNewLandmarkDBMatches.clear();
          for(uint i = 0; i < dbrMsg->matches.size(); i++)
            {
              uint index = dbrMsg->matches[i].inputSalRegID;
              itsNewMovementTracker->move(itsSearchTracker, index);

              // get the salient region of the landmark
              // that are just identified
              rutz::shared_ptr<LandmarkDBMatch> ldbm(new LandmarkDBMatch());
              GSlocJobData dbmi(index,
                                dbrMsg->matches[i].dbSegNum,
                                dbrMsg->matches[i].dbLmkNum,
                                dbrMsg->matches[i].dbVOStart,
                                dbrMsg->matches[i].dbVOEnd);
              ldbm->dbi = dbmi;
              itsNewLandmarkDBMatches.push_back(ldbm);

              LDEBUG("DB Match sent: %d: (%d %d - %d %d)",
                     itsNewLandmarkDBMatches[i]->dbi.objNum,
                     itsNewLandmarkDBMatches[i]->dbi.segNum,
                     itsNewLandmarkDBMatches[i]->dbi.lmkNum,
                     itsNewLandmarkDBMatches[i]->dbi.voStartNum,
                     itsNewLandmarkDBMatches[i]->dbi.voEndNum);
            }
        }

      // we can now reset the search tracker
      itsResetSearchTracker = true;

      its_Curr_Dbr_mutex.unlock();
      its_tracker_mutex.unlock();
      LDEBUG("Found %d new landmarks in %f ms \n\n",
             numRecognized, timer.get()/1000.0f);
    }

  // current location message
  else if(eMsg->ice_isA("::BeobotEvents::CurrentLocationMessage"))
    {
      BeobotEvents::CurrentLocationMessagePtr clMsg =
        BeobotEvents::CurrentLocationMessagePtr::dynamicCast(eMsg);
      LDEBUG("Got a CurrentLocationMessage with Request ID = %d"
             ": loc(%d, %f)",
             clMsg->RequestID, clMsg->segNum, clMsg->lenTrav);
      its_Curr_Loc_mutex.lock();
      itsCurrentSegmentLocation = clMsg->segNum;
      itsCurrentSegmentLengthTraveled = clMsg->lenTrav;
      its_Curr_Loc_mutex.unlock();
    }

  // motor message
  else if(eMsg->ice_isA("::BeobotEvents::MotorMessage"))
    {
      BeobotEvents::MotorMessagePtr mtrMsg =
        BeobotEvents::MotorMessagePtr::dynamicCast(eMsg);
      its_Curr_Mtr_mutex.lock();
      itsRemoteMode = mtrMsg->rcMode;
      itsRcTransSpeed = mtrMsg->rcTransVel;
      itsRcRotSpeed = mtrMsg->rcRotVel;
      its_Curr_Mtr_mutex.unlock();

      LDEBUG("Got a MotorMessage with Request ID = %d: RC Trans %f, Rot %f",
             mtrMsg->RequestID, itsRcTransSpeed, itsRcRotSpeed);
    }

  // corner message
  else if(eMsg->ice_isA("::BeobotEvents::CornerMatchMessage"))
    {
      BeobotEvents::CornerMatchMessagePtr cmmMsg =
        BeobotEvents::CornerMatchMessagePtr::dynamicCast(eMsg);

      its_Curr_CornerMtr_mutex.lock();
      itsCornerTransSpeed  = cmmMsg->transVel;
      itsCornerRotSpeed    = cmmMsg->rotVel;

      if(cmmMsg->status      == C_PROCEDURE_INACTIVE)
        itsCornerProcStatus   = C_PROCEDURE_INACTIVE;
      else if(cmmMsg->status == C_PROCEDURE_ACTIVE)
        itsCornerProcStatus   = C_PROCEDURE_ACTIVE;
      else if(cmmMsg->status == C_PROCEDURE_CONCLUDING)
        itsCornerProcStatus   = C_PROCEDURE_CONCLUDING;

      // FIXXX_TR2011 get the coordinate matches as well
      itsCornerMatchCoordinates.clear();
      uint matchSize = cmmMsg->matches.size();
      for(uint i = 0; i < matchSize; i++)
        {
          BeobotEvents::MatchCoord match = cmmMsg->matches[i];
          LDEBUG("match[%d]:[%4f,%4f],[%4f,%4f]", i,
                 match.targetCoord.i, match.targetCoord.j,
                 match.dbCoord.i,     match.dbCoord.j);

          MatchCoordinate cmatch
            (Point2D<double>(match.targetCoord.i, match.targetCoord.j),
             Point2D<double>(match.dbCoord.i,     match.dbCoord.j));
          itsCornerMatchCoordinates.push_back(cmatch);
        }      

      its_Curr_CornerMtr_mutex.unlock();
      LINFO("Got a CornerMotorRequest Trans %f, Rot %f",
            itsCornerTransSpeed, itsCornerRotSpeed);
    }
}

// ######################################################################
void GistSal_Navigation::evolve()
{
  // check if the current image is updated
  its_input_mutex.lock();
  bool newInputFlag = (itsPrevProcImgID < itsCurrImgID);
  its_input_mutex.unlock();

  // if so, process
  if(newInputFlag)
    {
      itsTimer.reset();

      its_tracker_mutex.lock();
      its_Curr_Dbr_mutex.lock();
      its_Curr_CornerMtr_mutex.lock();

      its_input_mutex.lock();
      itsProcImg = itsCurrImg;
      itsPrevProcImgID = itsCurrImgID;
      std::vector<Point2D<int> > salPoints = itsCurrSalPoints;
      ImageSet<float> cmap = itsCurrCmap;
      std::vector<Rectangle> rects = itsCurrSalRects;
      its_input_mutex.unlock();

      // feed conspicuity maps to the trackers
      LINFO("\n NEW IMAGE search tracker: reset: %d ", itsResetSearchTracker);
      // FIX !!!
      std::vector<rutz::shared_ptr<VisualObject> > fakeVO;
      itsSearchTracker->input
        (itsProcImg, cmap, itsResetSearchTracker,
         salPoints, rects, fakeVO);
      LINFO("\n movement tracker");
      itsCurrMovementTracker->input
        (itsProcImg, cmap, false,
         salPoints, rects, fakeVO);
      itsNewMovementTracker->input
        (itsProcImg, cmap, false,
         salPoints, rects, fakeVO);

      // so that we only reset once per request
      itsResetSearchTracker = false;

      // FOR MOVEMENT

      // check for nearness to junctions/intersections
      bool nearJunction = estimateProgressToDestination();

      // NOTE: BIAS IGNORED

      // if near junction && not in corner procedure
      if(nearJunction &&
         itsCornerProcStatus == C_PROCEDURE_INACTIVE)
        {
          // start the turning procedure
          // send out which corner to cue from
          BeobotEvents::CornerLocationMessagePtr msg =
            new BeobotEvents::CornerLocationMessage;
          msg->cornerLocation = itsCurrentSegmentLocation;
          this->publish("CornerLocationMessageTopic", msg);
          LINFO("Publishing corner location %d", msg->cornerLocation);
        }

      // if in the middle of corner procedure
      if(itsCornerProcStatus == C_PROCEDURE_ACTIVE)
        {
          itsTransSpeed = itsCornerTransSpeed;
          itsRotSpeed   = itsCornerRotSpeed;

          // FIXXX_TR2011 Set the corner coordinate matches 
          // to be the utilized coordinate matches
          itsMatchCoordinates = itsCornerMatchCoordinates;
          itsLastFoundTimer.reset();
        }

      if(itsCornerProcStatus == C_PROCEDURE_CONCLUDING ||
         itsCornerProcStatus == C_PROCEDURE_INACTIVE     )
        {
          // get the recognized salient regions
          uint64 kpst = itsTimer.get();
          bool foundSomething = (getKeypointMatches() != 0);
          //if(foundSomething) itsLastFoundTimer.reset();

          LDEBUG("[KP time: %f]\n\n", (itsTimer.get() - kpst)/1000.0f);
          if(itsCurrLandmarkDBMatches.size() != 0)
            {
              // extract the error difference
              computeErrorDifference();

              // last found timer can be reset
              itsLastFoundTimer.reset();
              itsReverseCount = 0;
              //itsReverseDir = false;

              itsCornerProcStatus = C_PROCEDURE_INACTIVE;
            }
          else
            {
              if(itsCornerProcStatus == C_PROCEDURE_CONCLUDING)
                {
                  itsTransSpeed = itsCornerTransSpeed;
                  itsRotSpeed   = itsCornerRotSpeed;

                  // FIXXX_TR2011 Set the corner coordinate matches
                  // to be the utilized coordinate matches
                  itsMatchCoordinates = itsCornerMatchCoordinates;

                  itsLastFoundTimer.reset();
                }
              else
                {
                  // if there are previous landmark 
                  // that is being tracked
                  if(itsRotSpeed != 0.0)
                    {
                      // FIXXX_2011 just a hack to make things work for now
                      LINFO("\n\n\n\n DECAY PREVIOUS COMMAND \n\n\n\n");
                      
                      double decay = -.1 * itsLastSetRotSpeed; 
                      itsRotSpeed += decay;                   
                    }

                  // slow down - hope we find new regions
                  itsTransSpeed = 0.6;

                  //if(itsLastFoundTimer.get() > ROT_TIME_LIMIT
                  //   && !itsReverseDir)
                  //  {
                  //    itsRotSpeed = -1.0 * itsRotSpeed;
                  //    itsReverseDir = true;
                  //  }
                  //itsDistToGoal = -1.0;

                  // FIXXX_TR2011 set the feature matches num to 0
                  itsMatchCoordinates.clear();
                }
            }

          // if we have not found anything for some time
          if(itsLastFoundTimer.get() > LOST_TIME_LIMIT)
            recover(foundSomething);
        }

      // use the error difference to drive
      navigation();

      its_Curr_CornerMtr_mutex.unlock();
      its_Curr_Dbr_mutex.unlock();
      its_tracker_mutex.unlock();

      // draw the state of the surroundings
      drawState();
      if(itsDispImg.getWidth() > 0 && itsDispImg.getHeight() > 0)
        itsOfs->writeRGB(itsDispImg, " ><\" GistSal_nav",
                         FrameInfo("GistSal_nav",SRC_POS));
      
      LINFO("[time: %f]\n\n\n\n", itsTimer.get()/1000.0f);
    }
}

// ######################################################################
// computing visual error difference
uint GistSal_Navigation::getKeypointMatches()
{
  Timer tim(1000000);
  uint nCount, cCount;

  // check if there are new DBmatches
  //uint oldNcount = itsNewLandmarkDBMatches.size();
  //uint oldCcount = itsCurrLandmarkDBMatches.size();

  // check if the new regions can be projected forward
  nCount = projectForward(itsNewLandmarkDBMatches,
                          itsNewMovementTracker);
  if(nCount == 0)
    {

      // No
      LINFO("\n\n\n New Project forward FAIL: %d time: %f\n\n\n",
            nCount, tim.get()/1000.0f);

      // stick with the old ones
      cCount = projectForward(itsCurrLandmarkDBMatches,
                              itsCurrMovementTracker);

      LINFO("\n\n\n Old Project forward ?   : %d time: %f\n\n\n",
            cCount, tim.get()/1000.0f);
    }
  else
    {
      // project forward succeed
      //LINFO("\n\n\n New Project forward WORK: %d time: %f\n\n\n",
      //      nCount, tim.get()/1000.0f);

      // move new regions to become current ones
      itsCurrLandmarkDBMatches = itsNewLandmarkDBMatches;

      itsCurrMovementTracker->clear();
      itsCurrMovementTracker->reset(nCount);
      itsCurrMovementTracker->setCurrInputImage
        (itsNewMovementTracker->getCurrInputImage());
      itsCurrMovementTracker->setOriginalInputImage
        (itsNewMovementTracker->getOriginalInputImage());
      for(uint i = 0; i < nCount; i++)
        itsCurrMovementTracker->move(itsNewMovementTracker, i);
    }
  cCount = itsCurrLandmarkDBMatches.size();
  LDEBUG("Project forward END: %d time: %f",
        cCount, tim.get()/1000.0f);

  // DON'T STOP ON THE WRONG SCALE

  // check if there are new DBmatches
  uint count = itsNewLandmarkDBMatches.size();

  // if not keep the old DBmatches
  if (count == 0)
    {
      count = itsCurrLandmarkDBMatches.size();
      // for(uint i = 0; i < itsCurrLandmarkDBMatches.size(); i++)
      //   {

      //     int index = itsLandmarkDB->getLandmark
      //       (itsCurrLandmarkDBMatches[i]->dbi.segNum,
      //        itsCurrLandmarkDBMatches[i]->dbi.lmkNum)->match
      //       (vo, cmatch,
      //        itsCurrLandmarkDBMatches[i]->dbi.voStartNum,
      //        -1, 10.0F, .75F, 2.5F, 4, M_PI/4, 1.25F);


      //     SIFTaffine aff =
      //       (itsCurrLandmarkDBMatches[i])->matchRes->getSIFTaffine();
      //     float theta, sx, sy, str;
      //     aff.decompose(theta, sx, sy, str);
      //     if(sx > 1.25F || sx < 0.8F)
      //       if(count > 0) count--;
      //   }
    }

// int index = itsLandmarkDB->getLandmark
//         (landmarkDBMatches[i]->dbi.segNum,
//          landmarkDBMatches[i]->dbi.lmkNum)->match
//         (vo, cmatch, landmarkDBMatches[i]->dbi.voStartNum,
//          -1, 10.0F, .75F, 2.5F, 4, M_PI/4, 1.25F);

  return count;
}

// ######################################################################
uint GistSal_Navigation::projectForward
(std::vector<rutz::shared_ptr<LandmarkDBMatch> > &landmarkDBMatches,
 nub::soft_ref<SalientRegionTracker> tracker)
{
  // get the found database matches
  std::vector<rutz::shared_ptr<LandmarkDBMatch> > tempLandmarkDBMatches;

  LDEBUG("start: %" ZU , landmarkDBMatches.size());

  for(uint i = 0; i < landmarkDBMatches.size(); i++)
    {
      // get the tracked salient region rectangle
      Rectangle rect = tracker->getCurrTrackedROI(i);

      // create a Visual Object
      Point2D<int> offset= rect.topLeft();
      Image<PixRGB<byte> > objImg = crop(itsCurrImg, rect);
      std::string iName("iName");
      std::string ifName = iName + std::string(".png");
      rutz::shared_ptr<VisualObject>
        vo(new VisualObject(iName, ifName, objImg));
      //vo(new VisualObject(iName, ifName, objImg, salpt - offset));

      // project forward with the database found
      LDEBUG("DB Match sent: %d: (%d %d - %d %d)",
             landmarkDBMatches[i]->dbi.objNum,
             landmarkDBMatches[i]->dbi.segNum,
             landmarkDBMatches[i]->dbi.lmkNum,
             landmarkDBMatches[i]->dbi.voStartNum,
             landmarkDBMatches[i]->dbi.voEndNum);

      rutz::shared_ptr<VisualObjectMatch> cmatch;
      int index = itsLandmarkDB->getLandmark
        (landmarkDBMatches[i]->dbi.segNum,
         landmarkDBMatches[i]->dbi.lmkNum)->match
        (vo, cmatch, landmarkDBMatches[i]->dbi.voStartNum,
         -1, 10.0F, .75F, 2.5F, 4, M_PI/4, 1.25F);
      landmarkDBMatches[i]->dbi.voStartNum = index;
      landmarkDBMatches[i]->dbi.voEndNum   = index;

      // set successful projected forward landmarks
      if(index != -1)
        {
          //LINFO("Project Forward succeed: %d\n", index);

          landmarkDBMatches[i]->vo = vo;

          // need to store this to release mutex during drawState()
          Point2D<int> pt =
            itsLandmarkDB->getLandmark
            (landmarkDBMatches[i]->dbi.segNum,
             landmarkDBMatches[i]->dbi.lmkNum)
            ->getOffsetCoords
            (landmarkDBMatches[i]->dbi.voStartNum);

          landmarkDBMatches[i]->voOffset = offset;
          landmarkDBMatches[i]->dbOffset = pt;
          landmarkDBMatches[i]->matchRes = cmatch;

          tempLandmarkDBMatches.push_back(landmarkDBMatches[i]);

          LINFO("project forward succeed %d[%d]", i, index);
        }
      else
        LINFO("Project Forward fail: %d\n", index);
    }

  // only switch when there is a new
  //if(tempFoundDBmatches.size() != 0)
  landmarkDBMatches = tempLandmarkDBMatches;

  return uint(landmarkDBMatches.size());
}

// ######################################################################
// computing visual error difference
void GistSal_Navigation::computeErrorDifference()
{
  // boundary condition
  //if((teachID-5) < TEACH_START_NUMBER) teachID = TEACH_START_NUMBER+2;

  // looking 10 frame ahead to find best match frame
  //for(int id = teachID-2; id < teachID+10; id++)
  // =========================
  //   find the best Keyframe
  //   float theta = 2.0, sx = 2.0, sy = 2.0, str = 0;
  //   match->getSIFTaffine().decompose(theta, sx, sy, str);

  int dirF = 0,dirL = 0,dirR = 0;

  uint w = itsProcImg.getWidth();
  uint h = itsProcImg.getHeight();

  // mean and standard error
  //double stdx  = 0.0, stdy  = 0.0;
  double avgdx = 0.0, avgdy = 0.0;

  uint nLDBMatch = itsCurrLandmarkDBMatches.size();
  uint totalMatchSize = 0;
  itsMatchCoordinates.clear();

  for(uint i = 0; i < nLDBMatch; i++)
    totalMatchSize += itsCurrLandmarkDBMatches[i]->matchRes->size();

  // go through each identified landmark
  for(uint i = 0; i < nLDBMatch; i++)
    {
      int dirGain = 1;

      rutz::shared_ptr<LandmarkDBMatch> ldbm =
        itsCurrLandmarkDBMatches[i];
      rutz::shared_ptr<VisualObjectMatch> matchRes =
        ldbm->matchRes;

      Point2D<int> obOffset = ldbm->voOffset;
      Point2D<int> dbOffset = ldbm->dbOffset;

      bool isODmatch =
        (ldbm->vo == matchRes->getVoRef());

      // compute the direction and find mean of error
      for (uint j = 0; j < matchRes->size(); j++)
        {
          rutz::shared_ptr<Keypoint> obkp; // input    visual object
          rutz::shared_ptr<Keypoint> dbkp; // database visual object
          if(isODmatch)
            {
              obkp = matchRes->getKeypointMatch(j).refkp;
              dbkp = matchRes->getKeypointMatch(j).tstkp;
            }
          else
            {
              obkp = matchRes->getKeypointMatch(j).tstkp;
              dbkp = matchRes->getKeypointMatch(j).refkp;
            }

          // to shift origin to the middle of the image
          double ud = dbkp->getX() + dbOffset.i - w/2;
          double ut = obkp->getX() + obOffset.i - w/2;

          double vd = dbkp->getY() + dbOffset.j - h/2;
          double vt = obkp->getY() + obOffset.j - h/2;

          double dx = fabs(ut - ud);
          double dy = fabs(vt - vd);

          // calculate standard deviation and average
          //double stdx  = 0.0;
          //double stdy  = 0.0;
          avgdx += dx;
          avgdy += dy;

          // FIXXX_TR2011 set as features to go to LocalMapNavigator
          itsMatchCoordinates.push_back
            (MatchCoordinate(Point2D<double>(ut,vt), 
                             Point2D<double>(ud,vd)));

          // calculate direction
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
    }
  LDEBUG("dirL: %4d dirF: %4d dirR: %4d", dirL, dirF, dirR);
  avgdx /= totalMatchSize;
  avgdy /= totalMatchSize;
  LDEBUG("avgdx %f avgdy %f", avgdx, avgdy);

  // calculating standard deviation
  // in the loop above-> stdx += dx * dx;
  // after loop -> stdx -= (avgdx*avgdx);
  //               stdx /= bestMatchSize;
  //               stdx  = sqrt(stdx);

  itsXError = avgdx;
  itsYError = avgdy;
  itsError  = sqrt(avgdx*avgdx + avgdy*avgdy);

  // compute the forward speed based on scale
  // if it's much smaller than the one on DB: drive fast
  //  bestMatch->getSIFTaffine().decompose(theta, sx, sy, str);
  //
  //  //if the size difference is large(sx/sy << 1.0),we can speed up
  //  double rms = 1.0 -(sqrt(sx*sx+sy*sy)/sqrt(2));

  // double gain = 1.0;
  //    if(rms > 0.0)
  //     itsTransSpeed = 0.8 + rms * gain;
  //   if(itsTransSpeed > 1.0)itsTransSpeed = 1.0;
  //   if(itsTransSpeed < 0.0)itsTransSpeed = 0.0;

  // just use the same speed, for now
  itsTransSpeed = 0.8;

  // compute turning speed
  if(dirF >= abs(dirR + dirL))
    { itsRotSpeed = 0.0; itsDir = 0; }
  else if(dirR > dirL)
    {
      //itsXError -= 55.0;
      //if(itsXError < 0.0) itsXError = 0;

      if(itsXError > 50.0)
        itsRotSpeed = -0.40;
      else
        itsRotSpeed = -0.20 + itsXError/50.0 * -0.20;

      itsDir = 1.0;
    }
  else
    {
      //itsXError += 25.0;
      //if(itsXError < 0.0) itsXError = 0;

      if(itsXError > 50.0)
        itsRotSpeed = 0.60;
      else
        itsRotSpeed = 0.30 + itsXError/50.0 * 0.30;

      itsDir = -1.0;
    }

  itsLastSetTransSpeed = itsTransSpeed;
  itsLastSetRotSpeed   = itsRotSpeed;
  LDEBUG("Speed: [trans: %7.3f, rot: %7.3f] Error: %f",
         itsTransSpeed, itsRotSpeed, itsError);
}

// ######################################################################
bool GistSal_Navigation::estimateProgressToDestination()
{
  bool nearJunction = false;

  its_Curr_Loc_mutex.lock();
  uint  currSegNum  = itsCurrentSegmentLocation;
  float currLenTrav = itsCurrentSegmentLengthTraveled;
  uint  desSegNum   = itsDesiredSegmentLocation;
  float desLenTrav  = itsDesiredSegmentLengthTraveled;
  its_Curr_Loc_mutex.unlock();

  // check the moves for the robot to go to the goal location
  float dist =
    itsTopologicalMap->getPath
    (currSegNum, currLenTrav, desSegNum, desLenTrav, itsMoves);
  // real distance in ft.
  dist *= itsTopologicalMap->getMapScale();
  itsDistToGoal = dist;

  // have a turn around command (if necessary) as well
  uint msize = itsMoves.size();
  for(uint i = 0; i < msize; i++)
    LDEBUG("Moves[%d]: %d", i, itsMoves[i]);

  itsJunctionDist = INVALID_DISTANCE;

  // if we are at goal location
  if(dist < 3.5 || msize == 0)
    {
      // tell the robot to stop
      itsInGoal     = true;
      itsTransSpeed = 0.0;
      itsRotSpeed   = 0.0;
      return false;
    }
  else itsInGoal = false;

  // we actually only need to look one step ahead
  if(itsMoves[0] == TOPOMAP_TURN_AROUND)
    {
      // turn around
      itsTurnAngle = M_PI;
      LDEBUG("1 Move: TURN AROUND");

      // this is probably a blocking open-loop operation

      // but also need to be careful about
      // not overcommitting too quickly

    }
  else if(itsMoves[0] == TOPOMAP_MOVE_FORWARD)
    {
      // if the goal is in the current and the next edge
      if(msize == 1)
        {
          LDEBUG("1 Move: almost there just MOVE FORWARD");

          // calculate speed according to distance left
          double speed = 0.8;
          if(dist < 10.0 && dist > 0.0) speed *= dist/10.0;
          itsTransSpeed = speed;

          itsTurnAngle    = 0.0;
          itsJunctionDist = -1111.0;
        }
      // else we need to figure out how far the next node is
      else
        {
          // get the current edge
          float ds, de;
          rutz::shared_ptr<Edge> e1 =
            itsTopologicalMap->getEdge(currSegNum, currLenTrav, ds, de);

          // distance to junction in ft.
          float jdist = de * itsTopologicalMap->getMapScale();
          itsJunctionDist = jdist;
          LDEBUG("JDIST: %f", jdist );

          // if we are far from the junction
          if(jdist > JUNCTION_START_DIST)
            {
              // move with speed according to how far from junction
              // calculate speed according to distance left
              //float speed = 0.8;
              //if(jdist > 5.0 && jdist > 0.0)
              //  speed = speed - .6*(5.0 - jdist)/5.0;
              //              itsTransSpeed = speed;

              itsTurnAngle = 0.0;
            }
          else
            {
              // get the next edge
              rutz::shared_ptr<Edge> e2 =
                itsTopologicalMap->getEdge(itsMoves[1]);

              // calculate the degree amount for the turn as well
              float angle  = itsTopologicalMap->getAngle(e1,e2);
              float dangle = angle*180.0/M_PI;
              LDEBUG("ANGLE: %f or %f", angle, dangle);

              itsTurnAngle   = angle;
              double addRot  = 0.0;
              double fdangle = fabs(dangle);

              // don't add rotation if the angle is less than 30 degrees
              // keep going with the visual feedback
              if(fdangle < 30.0)
                addRot = 0.0;
              // else if within  30 to 150 (to the right)
              else if(dangle >  30.0 && dangle <  150.0)
                addRot = -0.3;
              // else if going hard right
              else if(dangle >= 150.0)
                addRot = -0.6;
              // else if within  -30 to -150 (to the left)
              else if(dangle < -30.0 && dangle > -150.0)
                addRot = 0.3;
              // else if going hard left
              else if(dangle <= -150.0)
                addRot = 0.6;

              LDEBUG("\n\n\naddRot: %f dangle: %f\n\n\n\n", addRot, dangle);

              // IMPORTANT: need more visual feedback here!!!
              // check with the inputs that are matched
              // see if they are from the next segment

              // MOTOR COMMAND
              itsTransSpeed = 0.8;
              itsRotSpeed  += addRot;

              nearJunction = true;
            }
        }
    }

  return nearJunction;

  // QUESTION: how do we infuse this with other motor command

  // train gist for distance to junction
  // -> recognizing intersection

  // NOTE: put all these information to the display
}

// ######################################################################
// The main function for recovery
void GistSal_Navigation::recover(bool foundSomething)
{
  itsTurnAngle    = INVALID_ANGLE;
  itsJunctionDist = INVALID_DISTANCE;

  // pan around
  itsTransSpeed = 0.0;

  //uint64 time = itsLastFoundTimer.get();
  //if(((time/8000000)%2) == 1)
  //itsRotSpeed   = -0.45;
  //else

  if(foundSomething && itsReverseCount != 0)
    {
      itsRotSpeed = 0.0;
      if(itsReverseCount > 0) itsReverseCount--;
    }
  else if(foundSomething || itsReverseCount > 0)
    {
      itsRotSpeed = -BASE_ROT;

      itsReverseCount++;
      if(itsReverseCount > REVERSE_COUNT_MAX)
        itsReverseCount = 0;
    }
  else
    itsRotSpeed   = BASE_ROT; // spin LEFT

  LINFO("\n\n\n\n\n recovering \n\n\n\n\n\n");
}

// ######################################################################
// The main function of navigation
void GistSal_Navigation::navigation()
{
  // update motor command
  //updateMotorPID(itsTransSpeed, itsRotSpeed);

  // FIXXX_TR2011 !!!!!!!!!!!!!!!!!! BYPASS FOR NOW !!!!!!!!!!!!!!!!!!!!!!!!
  //updateMotor(itsCornerTransSpeed, itsCornerRotSpeed);
  //updateMotor(itsTransSpeed, itsRotSpeed);

  // FIXXX_2011 send the command as well as the features
  BeobotEvents::GSNavMatchMessagePtr msg = 
    new BeobotEvents::GSNavMatchMessage;
  msg->transVel = itsTransSpeed;
  msg->rotVel   = itsRotSpeed;

  uint numMatches = itsMatchCoordinates.size();
  for(uint i = 0; i < numMatches; i++)
    {
      BeobotEvents::MatchCoord match;
      match.targetCoord.i = itsMatchCoordinates[i].itsTargetCoord.i;
      match.targetCoord.j = itsMatchCoordinates[i].itsTargetCoord.j;
      match.dbCoord.i     = itsMatchCoordinates[i].itsDbCoord.i;
      match.dbCoord.j     = itsMatchCoordinates[i].itsDbCoord.j;
     
      msg->matches.push_back(match);
    }

  this->publish("GSNavMatchMessageTopic", msg);
  LINFO("[%d] Publish GSNav match message Trans %f Rotation %f",
        itsPrevProcImgID, itsTransSpeed, itsRotSpeed);

  // FIXXX_2011 wipe the features to 0 to ensure that they are only sent once
  itsMatchCoordinates.clear();
}

// ######################################################################
void GistSal_Navigation::drawState()
{
  uint w = itsProcImg.getWidth();
  uint h = itsProcImg.getHeight();

  itsDispImg.resize(5*w, 3*h, NO_INIT);

  for(uint i = 0; i < itsCurrLandmarkDBMatches.size(); i++)
    {
      //Image< PixRGB<byte> > mImg = itsMatchRes[i]->getMatchImage(1.0F);
      //inplacePaste(itsDispImg, mImg, Point2D<int>(w*i, 0));

      rutz::shared_ptr<LandmarkDBMatch> ldbm =
        itsCurrLandmarkDBMatches[i];

      rutz::shared_ptr<VisualObjectMatch> matchRes =
        ldbm->matchRes;

      Dims d(w,h);
      Point2D<int> objOffset1 = ldbm->voOffset;
      Point2D<int> objOffset2 = ldbm->dbOffset;

      bool isODmatch = (ldbm->vo ==  matchRes->getVoRef());
      bool isDOmatch = (ldbm->vo ==  matchRes->getVoTest());

      Image< PixRGB<byte> > result;
      if(isODmatch)
        {
          // LINFO("[%d %d]    O[%d %d wh: %d %d] D[%d %d wh: %d %d]",
          //       d.w(), d.h(),
          //       objOffset1.i, objOffset1.j,
          //       itsMatchRes[i]->getVoRef()->getImage().getWidth(),
          //       itsMatchRes[i]->getVoRef()->getImage().getHeight(),
          //       objOffset2.i, objOffset2.j,
          //       itsMatchRes[i]->getVoTest()->getImage().getWidth(),
          //       itsMatchRes[i]->getVoTest()->getImage().getHeight()  );

          result = matchRes->getMatchImage(d, objOffset1, objOffset2);
        }
      else if(isDOmatch)
        {
          // LINFO("[%d %d]2   O[%d %d wh: %d %d] D[%d %d wh: %d %d]",
          //       d.w(), d.h(),
          //       objOffset1.i, objOffset1.j,
          //       itsMatchRes[i]->getVoTest()->getImage().getWidth(),
          //       itsMatchRes[i]->getVoTest()->getImage().getHeight(),
          //       objOffset2.i, objOffset2.j,
          //       itsMatchRes[i]->getVoRef()->getImage().getWidth(),
          //       itsMatchRes[i]->getVoRef()->getImage().getHeight()  );

          result = matchRes->getMatchImage(d, objOffset2, objOffset1);
        }
      else{LFATAL("boom. Object neither ref nor tst."); }

      std::string ntext(sformat("[%d]", i));
      if(isODmatch) ntext += std::string("OD");
      else ntext += std::string("OD");
      inplacePaste(itsDispImg, result, Point2D<int>(i*w,0));
      writeText(itsDispImg, Point2D<int>(w*i,0), ntext.c_str());

      SIFTaffine aff = matchRes->getSIFTaffine();
      float theta, sx, sy, str;
      aff.decompose(theta, sx, sy, str);
      std::string ntext2(sformat("[%6.3f, %6.3f", sx, sy));
      writeText(itsDispImg, Point2D<int>(w*i,20), ntext2.c_str());
    }

  inplacePaste(itsDispImg, drawInfoImg(), Point2D<int>(0, 2*h));
}

// ######################################################################
// display current heading information
Image<PixRGB<byte> > GistSal_Navigation::drawInfoImg()
{
  uint w = itsProcImg.getWidth();
  uint h = itsProcImg.getHeight();

  Image<PixRGB<byte> > dirImg(5*w, h,ZEROS);

  std::string text;
  if(itsDir == 0)     { text = sformat("Dir: |||  "); }
  else if(itsDir > 0) { text = sformat("Dir: -->  "); }
  else                { text = sformat("Dir: <--  "); }

  uint totalMatchSize = 0;
  uint nLDBMatch = itsCurrLandmarkDBMatches.size();
  for(uint i = 0; i < nLDBMatch; i++)
    totalMatchSize += itsCurrLandmarkDBMatches[i]->matchRes->size();
  text += sformat("#kp[%4d]", totalMatchSize);

  text += sformat("lDiff:[%8.3f] X:[%8.3f] Y:[%8.3f] ",
                  itsError, itsXError, itsYError);

  text += sformat("Trans[%8.3f] Rot[%8.3f]",
                  itsTransSpeed, itsRotSpeed);

  writeText(dirImg, Point2D<int>(0,0), text.c_str(),
            PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),
            SimpleFont::FIXED(8));

  std::string text2;
  its_Curr_Loc_mutex.lock();
  uint  currSegNum  = itsCurrentSegmentLocation;
  float currLenTrav = itsCurrentSegmentLengthTraveled;
  uint  desSegNum   = itsDesiredSegmentLocation;
  float desLenTrav  = itsDesiredSegmentLengthTraveled;
  its_Curr_Loc_mutex.unlock();

  text2 += sformat("C[%3d, %6.3f] -> ", currSegNum, currLenTrav);
  text2 += sformat("D[%3d, %6.3f]: ",  desSegNum,  desLenTrav);
  text2 += sformat("dist:[%8.3f]: ",  itsDistToGoal);
  text2 += sformat("M:[%" ZU "]: [", itsMoves.size());
  for(uint i = 0; i < itsMoves.size(); i++)
    text2 += sformat(" %d", itsMoves[i]);

  if(itsTurnAngle == INVALID_ANGLE)
    text2 += sformat("] tAng: \">< ");
  else
    text2 += sformat("] tAng: %7.3f ", itsTurnAngle*180.0/M_PI);
  text2 += sformat("jDist: %8.3f", itsJunctionDist);

  writeText(dirImg, Point2D<int>(0,20), text2.c_str(),
            PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),
            SimpleFont::FIXED(8));

  std::string text3;
  if(itsInGoal) text3 += sformat("IN GOAL     ");
  else          text3 += sformat("NOT IN GOAL ");
  uint64 tim =  itsLastFoundTimer.get();
  float lftime = tim/1000.0;
  text3 += sformat("%f ", lftime);
  if(tim > LOST_TIME_LIMIT)
    text3 += sformat("- OVER LOST TIME LIMIT. ");
  text3 += sformat("STATUS: %3d", itsCornerProcStatus);

  writeText(dirImg, Point2D<int>(0,40), text3.c_str(),
            PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0),
            SimpleFont::FIXED(8));

  return dirImg;
}

// ######################################################################
void GistSal_Navigation::updateMotorPID(double tran, double rot,double error)
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
void GistSal_Navigation::updateMotor(double tran, double rot)
{
  ////////////////////// HACK FIXXXXXXXXXXXX///////////////////
  //tran = 0.2;
  //rot  = 0.0;
  ////////////////////// HACK FIXXXXXXXXXXXX///////////////////

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

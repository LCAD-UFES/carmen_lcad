/*!@file Robots2/Beobot2/Localization/Beobot2_GistSalLocalizerMaster.C
  Vision robot localization using a combination saliency and gist.
  Run app-Beobot2GistSalMaster at X1 to run Gist-Saliency model
  Run app-Beobot2_GistSalLocalizerMaster at X1 to run SIFT recognition master    
  Run app-Beobot2_GistSalLocalizerWorker at X[2 ... 8] to run SIFT recognition workers    
  see Siagian_Itti09tr                                                  */
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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Beobot2/Localization/Beobot2_GistSalLocalizerMaster.C $
// $Id: Beobot2_GistSalLocalizerMaster.C 15441 2012-11-14 21:28:03Z kai $
//

#include "Robots/Beobot2/Localization/Beobot2_GistSalLocalizerMaster.H"

#include "Image/MathOps.H"      // for inPlaceNormalize()
#include "Image/CutPaste.H"     // for inplacePaste()
#include "Image/DrawOps.H"      // for drawing
#include "Image/ShapeOps.H"     // for decXY()
#include "Image/MatrixOps.H"    // for matrixMult(), matrixInv(), etc

#include "Beobot/beobot-GSnav-def.H"
#include "Beobot/GSnavResult.H"

#include "Robots/Beobot2/BeoCommon.H"


#ifndef BEOBOT2_GISTSALLOCALIZERMASTERI_C
#define BEOBOT2_GISTSALLOCALIZERMASTERI_C

// number of particles used
#define NUM_PARTICLES          100

// maximum allowable localization error (in unit map)
#define MAX_LOC_ERROR          5.0

// standard deviation for odometry error (in meter)
#define STD_ODO_ERROR          0.02

// standard deviation for length traveled error (in ltrav [0.0 ... 1.0])
#define STD_LTRAV_ERROR        0.02

#define GIST_PRIORITY_WEIGHT   0.5
#define SAL_PRIORITY_WEIGHT    0.2
#define LOCN_PRIORITY_WEIGHT   0.3

// ######################################################################
Beobot2_GistSalLocalizerMasterI::Beobot2_GistSalLocalizerMasterI
(OptionManager& mgr,const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsSegmentBeliefHistogram(new Histogram()),
  itsSearchTimer(1000000),
  itsAbort(false),
  itsInputFnum(-1),
  itsSearchInputFnum(-1)
{
  // default start and ground truth location:
  //  the two formats are not the same
  itsSegmentLocation       = 0;
  itsSegmentLengthTraveled = 0.0F;
  itsLocation              = Point2D<int>(-1,-1);
  itsSnumGT                = 0;
  itsLtravGT               = 0.0;

  itsTimes.clear();

  itsNumWorkers     = 1;
  itsNumBusyWorkers = 0;

  itsSavePrefix = "";

  // FIXXX make sure this is inputted from odometry
  itsRobotDx = 0.0F;
  itsRobotDy = 0.0F;

  itsStopSent = false;
}

// ######################################################################
void Beobot2_GistSalLocalizerMasterI::start1()
{
  uint w = 160; uint h = 120;
  itsResultWin.reset
    (new XWinManaged(Dims(5*w, 3*h), 0, h+30, "Result Window" ));
  //itsInputWin.reset
  //  (new XWinManaged(Dims(w, h), 5*w+30, h+30, "Input Window" ));
}

// ######################################################################
Beobot2_GistSalLocalizerMasterI::~Beobot2_GistSalLocalizerMasterI()
{ }

// ######################################################################
void Beobot2_GistSalLocalizerMasterI::registerTopics()
{
  this->registerSubscription("GistSalMessageTopic");
  this->registerSubscription("LandmarkMatchResultMessageTopic");
  this->registerSubscription("LandmarkSearchStatMessageTopic");
  this->registerSubscription("SearchDoneMessageTopic");
  this->registerSubscription("AbortMessageTopic");

  this->registerSubscription("AccumulatedOdometryMessageTopic");
  this->registerSubscription("CurrentLocationResetMessageTopic");
  this->registerSubscription("GoalLocationRequestTopic");

  this->registerPublisher("LandmarkSearchQueueMessageTopic");
  this->registerPublisher("CancelSearchMessageTopic");
  this->registerPublisher("NextFrameMessageTopic");
  this->registerPublisher("LandmarkTrackMessageTopic");
  this->registerPublisher("CurrentLocationMessageTopic");
  this->registerPublisher("LandmarkDBSearchResultMessageTopic");
  this->registerPublisher("VisionLocalizationMessageTopic");
}

// ######################################################################
void Beobot2_GistSalLocalizerMasterI::evolve()
{ 
  // FIXXX: move something here

  // send out results
}

// ######################################################################
void Beobot2_GistSalLocalizerMasterI::updateMessage
(const RobotSimEvents::EventMessagePtr& eMsg, const Ice::Current&)
{
  Timer timer(1000000);

  // Get a gist-sal message
  if(eMsg->ice_isA("::BeobotEvents::GistSalMessage"))
  {
    its_fnum_mutex.lock();
    int inputFnum = itsInputFnum;
    its_fnum_mutex.unlock();

    // process previous results
    if(inputFnum != -1)
      {
        // update the belief using individual object
        // NOTE: this is for individual object updating
        LDEBUG("Saving belief[%d]", inputFnum);
        its_particles_mutex.lock();
        its_results_mutex.lock();

        objectUpdateBelief();
        updateBelief();

        // send current location message right away
        BeobotEvents::CurrentLocationMessagePtr clmsg =
          new BeobotEvents::CurrentLocationMessage;
        clmsg->RequestID = inputFnum;
        clmsg->segNum    = itsSegmentLocation;
        clmsg->lenTrav   = itsSegmentLengthTraveled;
        LINFO("Publishing CurrentLocationMessage with ID: %d [%d,%f]",
              inputFnum, clmsg->segNum, clmsg->lenTrav);
        publish("CurrentLocationMessageTopic", clmsg);

        // TIME: takes 10 - 20 ms
        saveLocalizerResults();

        its_results_mutex.unlock();
        its_particles_mutex.unlock();
      }

    BeobotEvents::GistSalMessagePtr gistSalMsg =
      BeobotEvents::GistSalMessagePtr::dynamicCast(eMsg);

    // Get the current request ID
    int currRequestID = gistSalMsg->RequestID;

    // TIME: takes 4ms
    //Image<PixRGB<byte> > currImg = 
    //  Ice2Image<PixRGB<byte> >(gistSalMsg->currIma);  
    //itsInputWin->drawImage(currImg,0,0);

    its_fnum_mutex.lock();
    itsInputFnum = currRequestID;
    its_fnum_mutex.unlock();
    LINFO("Got a GSMessage with Request ID = %d", currRequestID);

    // get the gist
    its_gist_mutex.lock();
    uint gsize = gistSalMsg->gistFeatures.size();
    itsInputGist.resize(1, gsize, NO_INIT);
    Image<double>::iterator aptr = itsInputGist.beginw();
    for(uint i = 0; i < gsize; i++)
      *aptr++ = gistSalMsg->gistFeatures[i];

    // TIME: takes 0.4ms
    // calculate the segment prediction
    itsSegmentHistogram = itsEnvironment->classifySegNum(itsInputGist);
    its_gist_mutex.unlock();












    // FIX: HACK for ground truth and robot movement
    getGroundTruth(currRequestID, itsSnumGT, itsLtravGT,
                   itsRobotDx, itsRobotDy);












    // apply action model and segment observation model
    its_particles_mutex.lock();
    //its_gist_mutex.unlock();
    actionUpdateBelief();
    segmentUpdateBelief();

    //its_gist_mutex.unlock();
    its_particles_mutex.unlock();

    its_fnum_mutex.lock();
    inputFnum           = itsInputFnum;
    int searchInputFnum = itsSearchInputFnum;
    its_fnum_mutex.unlock();

    // if timer goes past the time limit and robot is moving
    // FIX: maybe we still discard search even if robot is stationary
    if((inputFnum - searchInputFnum) > SEARCH_TIME_LIMIT &&
       !itsStopSent)
      {
        LINFO("%d STOP SEARCH", inputFnum);
        BeobotEvents::CancelSearchMessagePtr msg =
          new BeobotEvents::CancelSearchMessage;

        LINFO("Publishing CancelSearchMessage");
        publish("CancelSearchMessageTopic", msg);

        // sleep for 10 ms
        usleep(10000);

        itsStopSent = true;
      }

    // check if all workers are not running
    its_num_busy_workers_mutex.lock();
    bool noBusyWorkers = (itsNumBusyWorkers == 0);
    its_num_busy_workers_mutex.unlock();

    // send tracker info right away
    BeobotEvents::LandmarkTrackMessagePtr tmsg =
      new BeobotEvents::LandmarkTrackMessage;
    tmsg->currIma         = gistSalMsg->currIma;
    tmsg->conspicuityMaps = gistSalMsg->conspicuityMaps;
    tmsg->salientRegions  = gistSalMsg->salientRegions;
    tmsg->RequestID       = currRequestID;
    tmsg->resetTracker    = noBusyWorkers;
    LINFO("Publishing ltrackMessage with ID: %d at %f ms",
          inputFnum, timer.get()/1000.0);
    publish("LandmarkTrackMessageTopic", tmsg);

    if(noBusyWorkers)
      {
        itsStopSent = false;

        itsSearchTimer.reset();
        its_num_busy_workers_mutex.lock();
        itsNumBusyWorkers    = itsNumWorkers;
        its_num_busy_workers_mutex.unlock();

        its_fnum_mutex.lock();
        itsSearchInputFnum = currRequestID;
        its_fnum_mutex.unlock();
        LINFO("[%6d] NEW salregs", currRequestID);

        BeobotEvents::LandmarkSearchQueueMessagePtr smsg =
          new BeobotEvents::LandmarkSearchQueueMessage;

        // get the input image
        its_input_info_mutex.lock();
        itsInputImage = Ice2Image<PixRGB<byte> >(gistSalMsg->currIma);

        if((itsResultWin->getDims().w()/5) != itsInputImage.getWidth() ||
           (itsResultWin->getDims().h()/3) != itsInputImage.getHeight()  )
          itsResultWin->setDims
            (Dims(5*itsInputImage.getWidth(), 3*itsInputImage.getHeight()));

        // pass the image as well
        smsg->currIma = gistSalMsg->currIma;

        // get the salient region information
        itsInputVO.clear();
        itsInputObjOffset.clear();
        uint inputSize = gistSalMsg->salientRegions.size();
        for(uint i = 0; i < inputSize; i++)
          {
            BeobotEvents::SalientRegion salReg = gistSalMsg->salientRegions[i];
            LDEBUG("M[%4d] sp[%4d,%4d] rect[%4d,%4d,%4d,%4d]",
                   i, salReg.salpt.i, salReg.salpt.j,
                   salReg.objRect.tl.i, salReg.objRect.tl.j,
                   salReg.objRect.br.i, salReg.objRect.br.j);

            // print the pre-attentive feature vector
            std::vector<float> features;
            uint fsize = salReg.salFeatures.size();
            for(uint j = 0; j < fsize; j++)
              features.push_back(salReg.salFeatures[j]);

            Point2D<int> salpt(salReg.salpt.i, salReg.salpt.j);
            Point2D<int> offset( salReg.objRect.tl.i, salReg.objRect.tl.j);
            Rectangle rect = Rectangle::tlbrO
              (salReg.objRect.tl.j, salReg.objRect.tl.i,
               salReg.objRect.br.j, salReg.objRect.br.i);
            itsInputObjOffset.push_back(offset);

            // create a visual object for the salient region
            Image<PixRGB<byte> > objImg = crop(itsInputImage, rect);

            std::string testRunFPrefix("testRunFPrefix");
            std::string iname("iname");
            std::string saveFilePath("saveFilePath");

            std::string
              iName(sformat("%s_SAL_%07d_%02d",
                            testRunFPrefix.c_str(), currRequestID, i));
            std::string ifName = iName + std::string(".png");
            ifName = saveFilePath + ifName;
            rutz::shared_ptr<VisualObject>
              vo(new VisualObject
                 (iName, ifName, objImg, salpt - offset, features,
                  std::vector< rutz::shared_ptr<Keypoint> >(), false, false));
            itsInputVO.push_back(vo);

            LDEBUG("[%d] image[%d]: %s sal:[%d,%d] offset:[%d,%d]",
                   currRequestID, i, iName.c_str(),
                   (salpt - offset).i, (salpt - offset).j,
                   offset.i, offset.j);

            // Push the salient region onto the job queue
            smsg->salientRegions.push_back(salReg);
          }
        its_input_info_mutex.unlock();

        // call the search priority function
        its_job_queue_mutex.lock();
        //its_input_info_mutex.lock();
        itsJobQueue.clear();
        setSearchPriority();
        //its_input_info_mutex.unlock();

        // we will prioritize using saliency in the search loop
        // FIX: funky project forward stuff

        // store the jobs to the message
        std::list<GSlocJobData>::iterator itr = itsJobQueue.begin();
        uint count = 0;
        while (itr != itsJobQueue.end())
          {
            // Create a job queue
            BeobotEvents::LandmarkSearchJob tempJob;
            tempJob.inputSalRegID  = (*itr).objNum;
            tempJob.dbSegNum       = (*itr).segNum;
            tempJob.dbLmkNum       = (*itr).lmkNum;
            tempJob.dbVOStart      = (*itr).voStartNum;
            tempJob.dbVOEnd        = (*itr).voEndNum;

            //LINFO("[%5d] match obj[%d] lDB[%3d][%3d]:[%3d,%3d]", count,
            //       (*itr).objNum, (*itr).segNum, (*itr).lmkNum,
            //       (*itr).voStartNum,(*itr).voEndNum);

            smsg->jobs.push_back(tempJob);
            itr++; count++;
          }
        its_job_queue_mutex.unlock();

        // resize the result storage and information for when to quit
        its_results_mutex.lock();
        itsMatchFound.clear();
        itsVOmatchImage.clear();    itsVOmatchImage.resize(inputSize);
        itsLmkMatch.clear();        itsLmkMatch.resize(inputSize);
        itsSegNumMatch.clear();     itsSegNumMatch.resize(inputSize);
        itsLmkNumMatch.clear();     itsLmkNumMatch.resize(inputSize);
        itsVobNumMatch.clear();     itsVobNumMatch.resize(inputSize);
        itsLenTravMatch.clear();    itsLenTravMatch.resize(inputSize);
        itsNumObjectSearch.clear(); itsNumObjectSearch.resize(inputSize);
        for(uint i = 0; i < inputSize; i++) itsMatchFound.push_back(false);
        for(uint i = 0; i < inputSize; i++) itsNumObjectSearch[i] = 0;

        itsNumJobs           = count;
        itsNumJobsProcessed  = 0;
        itsLastSuccessfulJob = 0;
        itsNumObjectFound    = 0;
        its_results_mutex.unlock();
        smsg->RequestID = currRequestID;

        LINFO("Publishing lsqMessage[%d] with ID: %d", count, currRequestID);
        publish("LandmarkSearchQueueMessageTopic", smsg);
      }

      float time = timer.get()/1000.0F;
      LINFO("Time[%6d]: %f\n", currRequestID, time);
  }

  // Get a landmark match results
  else if(eMsg->ice_isA("::BeobotEvents::LandmarkMatchResultMessage"))
  {
    BeobotEvents::LandmarkMatchResultMessagePtr lmrMsg =
      BeobotEvents::LandmarkMatchResultMessagePtr::dynamicCast(eMsg);

    //Get the current request ID
    int currRequestID = lmrMsg->RequestID;

    BeobotEvents::LandmarkSearchJob tempJob = lmrMsg->matchInfo;
    LINFO("Got an lmrMessage");

    // matched landmark identity
    LINFO("-> found match[%d]: with itsLandmarkDB[%d][%d]",
          tempJob.inputSalRegID, tempJob.dbSegNum, tempJob.dbLmkNum);

    // location of the salient region found
    // used to estimate robot location
    its_results_mutex.lock();
    if(!itsMatchFound[tempJob.inputSalRegID])
      itsNumObjectFound++;
    itsMatchFound[tempJob.inputSalRegID]   = true;
    itsSegNumMatch[tempJob.inputSalRegID]  = tempJob.dbSegNum;
    itsLmkNumMatch[tempJob.inputSalRegID]  = tempJob.dbLmkNum;
    itsVobNumMatch[tempJob.inputSalRegID]  = tempJob.dbVOStart;
    itsLenTravMatch[tempJob.inputSalRegID] = lmrMsg->lenTravMatch;
    itsVOmatchImage[tempJob.inputSalRegID] =
      Ice2Image<PixRGB<byte> >(lmrMsg->voMatchImage);
    its_results_mutex.unlock();

    float time = timer.get()/1000.0F;
    LINFO("::BeobotEvents::LandmarkMatchResultMessage Time[%6d]: %f\n", 
          currRequestID, time);
  }

  // Got a landmark search stat
  else if(eMsg->ice_isA("::BeobotEvents::LandmarkSearchStatMessage"))
  {
    BeobotEvents::LandmarkSearchStatMessagePtr lssMsg =
      BeobotEvents::LandmarkSearchStatMessagePtr::dynamicCast(eMsg);

    // Get the current request ID
    //int currRequestID = lssMsg->RequestID;

    LDEBUG("Got an lssMessage with Request ID = %d [%d,%d]",
           lssMsg->RequestID, lssMsg->inputSalRegID, lssMsg->numObjSearch);

    // update
    its_results_mutex.lock();
    itsNumObjectSearch[lssMsg->inputSalRegID] += lssMsg->numObjSearch;
    if(lssMsg->found) itsLastSuccessfulJob = itsNumJobsProcessed;
    itsNumJobsProcessed++;

    // update last successful job count
    int dlast = itsNumJobsProcessed - itsLastSuccessfulJob;

      // stop after matching 3 of 5
      // check if the number of matches since last successful one
      //   is bigger than a percentage of the queue size
    bool earlyExit =
      itsNumObjectFound > 2 ||
        (itsNumObjectFound == 2 && dlast > (int)(0.05 * itsNumJobs)) ||
        (itsNumObjectFound == 1 && dlast > (int)(0.10 * itsNumJobs)) ||
        (itsNumObjectFound == 0 && dlast > (int)(0.30 * itsNumJobs));
    its_results_mutex.unlock();

    its_job_queue_mutex.lock();
    uint njobs = itsJobQueue.size();
    if(njobs > 0 && earlyExit)
      {
        its_results_mutex.lock();
        LINFO("EE: %d [found: %d, dlast: %d] clear: %d,"
              " jobs processed: %d/%d = %f",
              earlyExit, itsNumObjectFound, dlast, njobs,
              itsNumJobsProcessed, itsNumJobs,
              (float)itsNumJobsProcessed/itsNumJobs);
        its_results_mutex.unlock();

        // send clear jobs message
        itsJobQueue.clear();
        BeobotEvents::CancelSearchMessagePtr msg =
          new BeobotEvents::CancelSearchMessage;

        LINFO("Publishing CancelSearchMessage");
        publish("CancelSearchMessageTopic", msg);
      }
    its_job_queue_mutex.unlock();
  }

  // Got a search-is-done message
  else if(eMsg->ice_isA("::BeobotEvents::SearchDoneMessage"))
  {
    // check how many workers are done
    its_num_busy_workers_mutex.lock();
    if(itsNumBusyWorkers > 0) itsNumBusyWorkers--;
    bool noBusyWorkers = (itsNumBusyWorkers == 0);
    its_num_busy_workers_mutex.unlock();
    if(noBusyWorkers)
      {
        // store the search time
        float loopTime = itsSearchTimer.get()/1000.0F;
        itsTimes.push_back(loopTime);

        its_fnum_mutex.lock();
        int inputFnum       = itsInputFnum;
        int searchInputFnum = itsSearchInputFnum;
        its_fnum_mutex.unlock();

        LINFO("[%5d] Time: %f", inputFnum, loopTime);

        BeobotEvents::NextFrameMessagePtr msg =
          new BeobotEvents::NextFrameMessage;

        LINFO("Publishing NextFrameMessage");
        publish("NextFrameMessageTopic", msg);

        // inform navigation which landmarks are identified
        // to be used for navigation
        BeobotEvents::LandmarkDBSearchResultMessagePtr lmsg =
          new BeobotEvents::LandmarkDBSearchResultMessage;
        lmsg->RequestID = searchInputFnum;

        // send the found matches to tracker for movement
        its_results_mutex.lock();
        for(uint i = 0; i < itsMatchFound.size(); i++)
          {
            if(itsMatchFound[i])
              {
                BeobotEvents::LandmarkSearchJob match;
                match.inputSalRegID = i;
                match.dbSegNum      = itsSegNumMatch[i];
                match.dbLmkNum      = itsLmkNumMatch[i];
                match.dbVOStart     = itsVobNumMatch[i];
                match.dbVOEnd       = itsVobNumMatch[i];
                lmsg->matches.push_back(match);

                LDEBUG("sending [%d - %d %d - %d %d]",
                       match.inputSalRegID,
                       match.dbSegNum, match.dbLmkNum,
                       match.dbVOStart, match.dbVOEnd);
              }
          }
        its_results_mutex.unlock();

        LINFO("[%3d] Publishing LandmarkDBSearchResultMessage",
              lmsg->RequestID );
        publish("LandmarkDBSearchResultMessageTopic", lmsg);

        // publish vision localization results
        BeobotEvents::VisionLocalizationMessagePtr vlmsg =
          new BeobotEvents::VisionLocalizationMessage;
        vlmsg->RequestID = searchInputFnum;
        vlmsg->segnum    = itsSegmentLocation;
        vlmsg->lentrav   = itsSegmentLengthTraveled;
        vlmsg->x         = itsLocation.i;
        vlmsg->y         = itsLocation.j;          

        LINFO("[%3d] Publishing VisionLocalizationMessage",
              searchInputFnum);
        publish("VisionLocalizationMessageTopic", vlmsg);

        // display the search loop statistics
        if(itsAbort)
          {
            double meanTime, minTime, maxTime, stdevTime;
            Image<double> t1(1, itsTimes.size(),NO_INIT);
            for(uint i = 0; i < itsTimes.size(); i++)
              t1.setVal(0, i, double(itsTimes[i]));

            meanTime = mean(t1);
            getMinMax(t1, minTime, maxTime);
            stdevTime = stdev(t1);

            LINFO("Time: %f (%f - %f) std: %f",
                  meanTime, minTime, maxTime, stdevTime);
          }        
      }
  }

  // Got an abort message
  else if(eMsg->ice_isA("::BeobotEvents::AbortMessage"))
  {
    itsAbort = true;

    // get and store robot location & error, and loop time
    its_particles_mutex.lock();
    its_results_mutex.lock();
    objectUpdateBelief();

    // update the belief using all the found salient regions
    updateBelief();

    saveLocalizerResults();
    its_results_mutex.unlock();
    its_particles_mutex.unlock();

    // send clear jobs message
    its_job_queue_mutex.lock();
    itsJobQueue.clear();
    its_job_queue_mutex.unlock();
    BeobotEvents::CancelSearchMessagePtr msg =
      new BeobotEvents::CancelSearchMessage;

    LINFO("Publishing CancelSearchMessage");
    publish("CancelSearchMessageTopic", msg);

    // print the results summary
    GSnavResult r1;
    r1.read(itsSavePrefix, itsTopologicalMap->getSegmentNum());
    r1.createSummaryResult();

    sleep(5);
    exit(0);
  }

  // process goal location for turning
  else if(eMsg->ice_isA("::BeobotEvents::GoalLocationRequest"))
    {
      // we got new goal location request                     
      BeobotEvents::GoalLocationRequestPtr glrMsg =
        BeobotEvents::GoalLocationRequestPtr::dynamicCast(eMsg);      

      if(glrMsg->goalType == BEONAVIGATOR_TURN_GOAL)
        {
          Point2D<int> goalNode(glrMsg->goalLoc.i, glrMsg->goalLoc.j);
          uint  snum  = glrMsg->snum;
          float ltrav = glrMsg->ltrav;

          LINFO("Turn[%4d %4d] to (%d, %f)", 
                goalNode.i, goalNode.j, snum, ltrav);
          initParticles(snum, ltrav, 0.0F);
        }
    }

  // accumulated odometry information
  else if(eMsg->ice_isA("::BeobotEvents::AccumulatedOdometryMessage"))
    {
      // we got new goal location request                     
      BeobotEvents::AccumulatedOdometryMessagePtr aoMsg =
        BeobotEvents::AccumulatedOdometryMessagePtr::dynamicCast(eMsg);      

      float acc_odo_dist = aoMsg->AccOdo;
      LINFO("AO distance %f", acc_odo_dist);
      accumulatedOdometryUpdateBelief(acc_odo_dist);
      updateBelief();
    }

  // reset location 
  else if(eMsg->ice_isA("::BeobotEvents::CurrentLocationResetMessage"))
    {
      // we got new goal location request                     
      BeobotEvents::CurrentLocationResetMessagePtr clrMsg =
        BeobotEvents::CurrentLocationResetMessagePtr::dynamicCast(eMsg);      

      initParticles(clrMsg->snum, clrMsg->ltrav, 0.0F);
      LINFO("CLR to: (%d, %f)", clrMsg->snum, clrMsg->ltrav);
    }

  // this is for checking if we need to reset search for the workers
  // for failed tracking or time is up
}

// ######################################################################
void Beobot2_GistSalLocalizerMasterI::setEnvironment
(rutz::shared_ptr<Environment> env)
{
  itsEnvironment = env;

  //! from its environment: topological map
  itsTopologicalMap = env->getTopologicalMap();

  //! from its environment: visual landmark database
  itsLandmarkDB = env->getLandmarkDB();
}

// ######################################################################
void Beobot2_GistSalLocalizerMasterI::setNumWorkers(uint numWorkers)
{
  itsNumWorkers = numWorkers;
}

// ######################################################################
void Beobot2_GistSalLocalizerMasterI::initParticles
(uint snum, float ltrav, float variance)
{
  uint nsegment = itsTopologicalMap->getSegmentNum();
  itsSegmentBeliefHistogram->resize(nsegment);
  LINFO("number of segment : %d", nsegment);

  itsBeliefParticles.clear();
  itsBeliefLocations.clear();

  LINFO("create particle: %d %f", snum, ltrav);

  // create initial particle
  for(uint i = 0; i < NUM_PARTICLES; i++)
    {
      // add the 
      if(variance != 0.0)
        {
          ltrav += (2.0*(-.5F +rand()/float(RAND_MAX + 1.0))) * variance;
          if      (ltrav < 0.0F) ltrav = 0.0F;
          else if (ltrav > 1.0F) ltrav = 1.0F;
        }

      itsBeliefParticles.push_back(LocParticle(snum, ltrav));

      // fill in the locations as well
      Point2D<int> loc = itsTopologicalMap->getLocation(snum, ltrav);
      itsBeliefLocations.push_back(loc);

      LDEBUG("particle[%4u]: (%3u, %10.6f) = (%4d %4d)",
             i, snum, ltrav, loc.i, loc.j);
    }
}

// ######################################################################
void Beobot2_GistSalLocalizerMasterI::initParticles(std::string belFName)
{
  uint nsegment = itsTopologicalMap->getSegmentNum();
  itsSegmentBeliefHistogram->resize(nsegment);
  LINFO("number of segment : %d", nsegment);

  itsBeliefParticles.clear();
  itsBeliefLocations.clear();

  // check if the file does not exist or it's a blank entry
  FILE *fp; if((fp = fopen(belFName.c_str(),"rb")) == NULL)
    {
      LINFO("Belief file %s not found", belFName.c_str());
      LINFO("create random particles");

      // create initial random particles
      for(uint i = 0; i < NUM_PARTICLES; i++)
        {
          float t  = rand()/(RAND_MAX + 1.0);
          float t2 = rand()/(RAND_MAX + 1.0);

          uint  snum  = uint ((0)    + ((nsegment) * t ));
          float ltrav = float((0.0F) + ((1.0F    ) * t2));
          itsBeliefParticles.push_back(LocParticle(snum, ltrav));
        }
    }
  else
    {
      LINFO("Belief file %s found", belFName.c_str());

      // get the particles
      for(uint i = 0; i < NUM_PARTICLES; i++)
        {
          char inLine[200]; fgets(inLine, 200, fp);
          uint snum; float ltrav;
          sscanf(inLine, "%d %f", &snum, &ltrav);
          itsBeliefParticles.push_back(LocParticle(snum, ltrav));
        }
      Raster::waitForKey();
    }

  // fill in the locations as well
  for(uint i = 0; i < NUM_PARTICLES; i++)
    {
      uint  snum  = itsBeliefParticles[i].segnum;
      float ltrav = itsBeliefParticles[i].lentrav;

      // convert to Point2D<int>
      Point2D<int> loc = itsTopologicalMap->getLocation(snum, ltrav);
      itsBeliefLocations.push_back(loc);

      LDEBUG("particle[%4u]: (%3u, %10.6f) = (%4d %4d)",
             i, snum, ltrav, loc.i, loc.j);
    }
}

// // ######################################################################
// std::vector<LocParticle> Beobot2_GistSalLocalizerMasterI::getBeliefParticles()
// {
//   std::vector<LocParticle> beliefParticles(itsBeliefParticles.size());

//   pthread_mutex_lock(&particleLock);
//   for(uint i = 0; i < itsBeliefParticles.size(); i++)
//     {
//       beliefParticles[i] =
//         LocParticle(itsBeliefParticles[i].segnum,
//                    itsBeliefParticles[i].lentrav);
//     }
//   pthread_mutex_unlock(&particleLock);

//   return beliefParticles;
// }

// ######################################################################
//! set the search priority for landmark DB
void Beobot2_GistSalLocalizerMasterI::setSearchPriority()
{
  // search priority is:
  // GIST_PRIORITY_WEIGHT * segment priority +
  // LOCN_PRIORITY_WEIGHT * current location priority +
  // SAL_PRIORITY_WEIGHT * saliency priority
  //   (sal is done in the search loop)

  Timer timer(1000000);
  // create jobs for each landmark - object combination
  for(uint i = 0; i < itsEnvironment->getNumSegment(); i++)
    {
      uint nlmk = itsLandmarkDB->getNumSegLandmark(i);
      LDEBUG("itsLandmarkDB[%d]: %d", i, nlmk);
      for(uint j = 0; j < nlmk; j++)
        {
          // check each object
          for(uint l = 0; l < itsInputVO.size(); l++)
            {
              uint nObj = itsLandmarkDB->getLandmark(i,j)->numObjects();
              uint k = 0;
              while(k < nObj)
                {
                  uint k2 = k + N_OBJECT_BLOCK - 1;
                  if(k2 > nObj-1) k2 = nObj - 1;
                  itsJobQueue.push_back(GSlocJobData(l, i, j, k, k2));
                  LDEBUG("match obj[%d] lDB[%3d][%3d]:[%3d,%3d]",
                         l, i,j,k,k2);
                  k = k2 + 1;
                }
            }
        }
    }
  LINFO("setting jobs %11.5f", timer.get()/1000.0F);

  // FIX: GOOD LANDMARKS ARE FOUND IN MULTIPLE RUNS !

  // set the order of search to random values
  // not actually used, just for baseline to put in ICRA08
  //addRandomPriority();

  // FIX: always load the last 10 matched landmarks first
  // we do this by adding a value of 3.0

  // add the segment priority
  timer.reset();
  //its_gist_mutex.unlock();
  addSegmentPriority();
  //its_gist_mutex.unlock();
  LINFO("segment      %11.5f", timer.get()/1000.0F);

  // add the current location priority
  timer.reset();
  addLocationPriority();
  LINFO("location     %11.5f", timer.get()/1000.0F);

  // BIG NOTE:
  // doing saliency prioritization is slow
  // has to be done outside the input loop

  // add the saliency priority
  timer.reset();
  addSaliencyPriority();
  LINFO("sal prior    %11.5f", timer.get()/1000.0F);

  // push the jobs based on the biasing values
  timer.reset();
  itsJobQueue.sort();
  LINFO("sort         %11.5f", timer.get()/1000.0F);

  // print the priority values
  std::list<GSlocJobData>::iterator itr = itsJobQueue.begin();
  uint count = 0;
  while (itr != itsJobQueue.end())
    {
      LDEBUG("[%5d] pval[%3d][%3d][%3d]: %f + %f + %f = %f", count,
             (*itr).segNum, (*itr).lmkNum, (*itr).objNum,
             (*itr).segVal, (*itr).salVal, (*itr).locVal,
             (*itr).pVal);
      itr++; count++;
    }
}

// ######################################################################
void Beobot2_GistSalLocalizerMasterI::addRandomPriority()
{
  // add a random value to the priority value
  std::list<GSlocJobData>::iterator itr = itsJobQueue.begin();
  while (itr != itsJobQueue.end())
    {
      // flip the value
      float val = float(rand()/(RAND_MAX + 1.0));
      (*itr).pVal += val;
      itr++;
    }
}

// ######################################################################
void Beobot2_GistSalLocalizerMasterI::addSegmentPriority()
{
  // add the segment value to the priority value
  std::list<GSlocJobData>::iterator itr = itsJobQueue.begin();
  while (itr != itsJobQueue.end())
    {
      // flip the value
      float val = GIST_PRIORITY_WEIGHT *
        (1.0 - itsSegmentHistogram->getValue((*itr).segNum));

      (*itr).pVal += val;
      (*itr).segVal = val;
      itr++;
    }
}

// ######################################################################
void Beobot2_GistSalLocalizerMasterI::addSaliencyPriority()
{
  uint nObj = itsInputVO.size();

  // go through each landmark - object combination
  Timer timer(1000000);
  uint nSeg = itsEnvironment->getNumSegment();
  std::vector<std::vector<std::vector<float> > > salVal(nSeg);
  for(uint i = 0; i < itsEnvironment->getNumSegment(); i++)
    {
      uint nlmk = itsLandmarkDB->getNumSegLandmark(i);
      salVal[i].resize(nlmk);
      for(uint j = 0; j < nlmk; j++)
        {
          // check each object
          salVal[i][j].resize(nObj);
          for(uint k = 0; k < nObj; k++)
            {
              LDEBUG("sal seg[%3d] lmk[%3d] obj[%3d]", i,j,k);
              salVal[i][j][k] = SAL_PRIORITY_WEIGHT *
                itsLandmarkDB->getLandmark(i,j)
                ->matchSalientFeatures(itsInputVO[k]);
            }

          // display the database landmark object
          //Image<PixRGB<byte> > img = itsLandmarkDB->getLandmark(i,j)
          //  ->getObject(0)->getSalAndKeypointImage();
          //itsWin->drawImage(img, 0,0);
          //Raster::waitForKey();
        }
    }
  LDEBUG("compute saliency dist %11.5f", timer.get()/1000.0F);

  // add the saliency value to the priority value
  std::list<GSlocJobData>::iterator itr = itsJobQueue.begin();
  while (itr != itsJobQueue.end())
    {
      float t = (*itr).pVal;
      float val = salVal[(*itr).segNum][(*itr).lmkNum][(*itr).objNum];
      (*itr).pVal += val;
      (*itr).salVal = val;

      LDEBUG("pval[%3d][%3d][%3d]: %f + %f = %f",
             (*itr).segNum, (*itr).lmkNum, (*itr).objNum, t, val, (*itr).pVal);
      itr++;
    }
}

// ######################################################################
void Beobot2_GistSalLocalizerMasterI::addLocationPriority()
{
  // normalizer: setup weight and sigma for decision boundary
  Dims mDims = itsTopologicalMap->getMapDims();
  Point2D<int> brMap(mDims.w(), mDims.h());
  float mDiag = brMap.distance(Point2D<int>(0,0));
  float sigma = .1 * mDiag;
  LDEBUG("map diagonal: %f -> sigma: %f",  mDiag, sigma);
  LDEBUG("curr loc: %d, %f ", itsSegmentLocation, itsSegmentLengthTraveled);

  // get the distance to all landmarks from current belief location
  uint nSeg =itsEnvironment->getNumSegment();
  std::vector<std::vector<float> > locVal(nSeg);
  for(uint i = 0; i < itsEnvironment->getNumSegment(); i++)
    {
      uint nlmk = itsLandmarkDB->getNumSegLandmark(i);
      locVal[i].resize(nlmk);
      for(uint j = 0; j < nlmk; j++)
        {
          std::pair<float,float> locRange =
            itsLandmarkDB->getLocationRange(i,j);
          LDEBUG("lmk[%3d][%3d]: (%f,%f)", i,j,
                 locRange.first, locRange.second);

          // if the location is within the range of the landmark
          if(itsSegmentLocation == i &&
             itsSegmentLengthTraveled >= locRange.first &&
             itsSegmentLengthTraveled <= locRange.second  )
            {
              locVal[i][j] = 0.0;
              LDEBUG("dist[%d][%d]: within -> %f",i,j, locVal[i][j]);
            }
          else
            {
              // get distance to the first seen location
              float fdist = itsTopologicalMap->
                getDistance(itsSegmentLocation, itsSegmentLengthTraveled,
                            i, locRange.first);

              // get distance to the last seen location
              float ldist = itsTopologicalMap->
                getDistance(itsSegmentLocation, itsSegmentLengthTraveled,
                            i, locRange.second);

              // get the minimum distance
              float dist = std::min(fdist,ldist);

              LDEBUG("f - l: %d [%f -> %f][%f -> %f]",
                     i, locRange.first, fdist, locRange.second, ldist);

              // get distances to nodes in between
              std::vector<std::pair<uint,float> > betLocs =
                itsTopologicalMap->getNodeLocationsInInterval
                (i, locRange.first, locRange.second);
              for(uint k = 0; k < betLocs.size(); k++)
                {
                  float bdist = itsTopologicalMap->
                    getDistance(itsSegmentLocation, itsSegmentLengthTraveled,
                                i, betLocs[k].second);
                  if(dist > bdist) dist = bdist;

                  LDEBUG("bet: %d [%f -> %f]", i, betLocs[k].second, bdist);
                }

              // normalize gaussian val to 1.0 invert the values
              locVal[i][j] = LOCN_PRIORITY_WEIGHT *
                (1.0 - pow(M_E, -dist*dist/(2.0*sigma*sigma)));
              LDEBUG("dist[%d][%d]: %f ->%f",i,j, dist, locVal[i][j]);
            }
        }
    }

  // add the location value to the priority value
  std::list<GSlocJobData>::iterator itr = itsJobQueue.begin();
  while (itr != itsJobQueue.end())
    {
      float t = (*itr).pVal;
      float val = locVal[(*itr).segNum][(*itr).lmkNum];
      (*itr).pVal += val;
      (*itr).locVal = val;

      LDEBUG("pval[%3d][%3d][%3d]: %f -> %f",
             (*itr).segNum, (*itr).lmkNum, (*itr).objNum, t, (*itr).pVal);
      itr++;
    }
}

// ######################################################################
rutz::shared_ptr<Histogram>
Beobot2_GistSalLocalizerMasterI::getSegmentBeliefHistogram()
{
  itsSegmentBeliefHistogram->clear();

  for(uint i = 0; i < NUM_PARTICLES; i++)
    {
      itsSegmentBeliefHistogram->
        addValue(itsBeliefParticles[i].segnum, 1.0F);
    }

  //! print the histogram profile
  uint nsegment = itsTopologicalMap->getSegmentNum();
  for(uint i = 0; i < nsegment; i++)
    LDEBUG("[%d]: %d", i, uint(itsSegmentBeliefHistogram->getValue(i)));
  return itsSegmentBeliefHistogram;
}

// ######################################################################
void Beobot2_GistSalLocalizerMasterI::saveLocalizerResults()
{
  uint  snumRes = 0; float ltravRes = -1.0F;
  float error = 0.0F;

  its_fnum_mutex.lock();
  int index = itsInputFnum;
  its_fnum_mutex.unlock();

  if(itsLtravGT != -1.0F)
    {
      //Point2D<float> p = 
      itsEnvironment->getLocationFloat(itsSnumGT, itsLtravGT);

      // check the ground truth
      //float xGT = p.i; float yGT = p.j;
      snumRes  = itsSegmentLocation;
      ltravRes = itsSegmentLengthTraveled;
      error = itsTopologicalMap->
        getDistance(itsSnumGT, itsLtravGT, snumRes, ltravRes);
      LDEBUG("Ground Truth [%d %f] vs [%d %f]: error: %f",
             itsSnumGT, itsLtravGT, snumRes, ltravRes, error);
    }

  bool saveDisplay = true;
  if(saveDisplay)
    {
      // TIME: takes 10 - 20ms
      Image<PixRGB<byte> > dispIma(getDisplayImage());  
      itsResultWin->setTitle(sformat("%d", index).c_str());
      itsResultWin->drawImage(dispIma, 0, 0);

      // save it to be mpeg encoded
      std::string saveFName =  itsSavePrefix + sformat("_RES_%07d.ppm", index);
      LDEBUG("saving: %s",saveFName.c_str());
      //Raster::WriteRGB(dispIma,saveFName);

      // save the particle locations
      // in case we need to restart in the middle
      std::string belFName = itsSavePrefix + sformat("_bel_%07d.txt", index);
      FILE *bfp; LDEBUG("belief file: %s", belFName.c_str());
      if((bfp = fopen(belFName.c_str(),"wt")) == NULL)
        LFATAL("Belief file: %s not found", belFName.c_str());
      for(uint i = 0; i < itsBeliefParticles.size(); i++)
        {
          std::string bel = sformat("%d %f ",
                                    itsBeliefParticles[i].segnum,
                                    itsBeliefParticles[i].lentrav);
          bel += std::string("\n");
          fputs(bel.c_str(), bfp);
        }
      fclose (bfp);
    }  

  // save result in a file by appending to the file
  std::string resFName = itsSavePrefix + sformat("_results.txt");
  FILE *rFile = fopen(resFName.c_str(), "at");
  if (rFile != NULL)
    {
      LDEBUG("saving result to %s", resFName.c_str());
      std::string line =
        sformat("%5d %3d %8.5f %3d %8.5f %10.6f",
                index, itsSnumGT, itsLtravGT, snumRes, ltravRes, error);

      its_num_busy_workers_mutex.lock();
      bool noBusyWorkers = (itsNumBusyWorkers == 0);
      its_num_busy_workers_mutex.unlock();

      if(noBusyWorkers)
        {
          uint ninput = itsMatchFound.size();
          line += sformat(" %d", ninput);
          for(uint i = 0; i < ninput; i++)
            line += sformat("%3d %6d", int(itsMatchFound[i]),
                            itsNumObjectSearch[i]);
        }
      else line += sformat(" 0");

      LINFO("%s", line.c_str());
      line += std::string("\n");

      fputs(line.c_str(), rFile);
      fclose (rFile);
    }
  else LINFO("can't create file: %s", resFName.c_str());
}

// ######################################################################
void Beobot2_GistSalLocalizerMasterI::updateBelief()
{
  // set the most likely location
  setLocation();
}

// ######################################################################
void Beobot2_GistSalLocalizerMasterI::actionUpdateBelief()
{
  std::vector<int> stotal(itsTopologicalMap->getSegmentNum());
  for(uint i = 0; i < stotal.size(); i++) stotal[i] = 0;

  // apply the motor movement + noise to each particles
  for(uint i = 0; i < NUM_PARTICLES; i++)
    {
      uint  snum  = itsBeliefParticles[i].segnum;
      float ltrav = itsBeliefParticles[i].lentrav;
      LDEBUG("particle[%d]: %d, %f", i, snum, ltrav);

      // apply the motor command
      float nltrav = ltrav + itsRobotDx;

      //      if(itsRobotDx != 0.0F)
        {
          // assume std odometry error of .02ft
          float mscale = itsTopologicalMap->getMapScale();
          float err    = STD_ODO_ERROR/mscale;
          
          // add noise from a Gaussian distribution (Box-Muller method)
          float r1 = float(rand()/(RAND_MAX + 1.0));
          float r2 = float(rand()/(RAND_MAX + 1.0));
          double r = err * sqrt( -2.0 * log(r1));
          double phi = 2.0 * M_PI * r2;
          nltrav += (r * cos(phi));
          
          // if nltrav is now improper
          // FIX: NEED THE SPILL OVER EFFECT
          if(nltrav < 0.0F) nltrav = 0.0F;
          if(nltrav > 1.0F) nltrav = 1.0F;
        }

      itsBeliefParticles[i].lentrav = nltrav;
      //stotal[snum]++;

      // convert to Point2D<int>
      Point2D<int> loc = itsTopologicalMap->
        getLocation(itsBeliefParticles[i].segnum,
                    itsBeliefParticles[i].lentrav);
      itsBeliefLocations[i] = loc;
    }
}

// ######################################################################
void Beobot2_GistSalLocalizerMasterI::segmentUpdateBelief()
{
  // accweight is for easy calculation for choosing a random particle
  std::vector<float> weight(NUM_PARTICLES);
  std::vector<float> accweight(NUM_PARTICLES);

  // for each particles
  float accw = 0.0F;
  for(uint i = 0; i < NUM_PARTICLES; i++)
    {
      uint  snum  = itsBeliefParticles[i].segnum;
      float ltrav = itsBeliefParticles[i].lentrav;
      LDEBUG("particle[%4d]: %d, %f", i, snum, ltrav);

      // get likelihood score
      // score(snum)/sum(score(all segments) * score(snum)
      // with the denominator taken out
      // + 0.25 for the 50% memory of previous time
      float pscore = itsSegmentHistogram->getValue(snum);
      float score  = (pscore * pscore) + 0.25f;
      LDEBUG("score: %f * %f = %f", pscore, pscore, score);

      // update weight
      weight[i] = score;
      accw += score;
      accweight[i] = accw;
    }
  for(uint i = 0; i < NUM_PARTICLES; i++)
    LDEBUG("p[%4d]: w: %f %f ",i, weight[i], accweight[i]);
  LDEBUG("accw: %f",accw);

  // add 1% noise weight
  accw *= 1.01F; LDEBUG("accw+ noise: %f",accw);

  // weighted resample NUM_PARTICLES particles
  std::vector<LocParticle> tbelief(NUM_PARTICLES);
  uint nsegment = itsTopologicalMap->getSegmentNum();
  std::vector<int> stotal(nsegment);
  for(uint i = 0; i < stotal.size(); i++) stotal[i] = 0;

  for(uint i = 0; i < NUM_PARTICLES; i++)
    {
      // draw a random value between 0 and accw
      float rval  = accw * float(rand()/(RAND_MAX + 1.0));

      // get the random index
      uint sind = NUM_PARTICLES;
      for(uint j = 0; j < NUM_PARTICLES; j++)
        if(rval < accweight[j]) { sind = j; j = NUM_PARTICLES; }
      LDEBUG("rval: %f -> %d", rval, sind);

      // if need to create a random particle
      if(sind == NUM_PARTICLES)
        {
          // create initial random particles
          float t  = rand()/(RAND_MAX + 1.0);
          float t2 = rand()/(RAND_MAX + 1.0);

          uint  snum  = uint ((0)    + ((nsegment) * t ));
          float ltrav = float((0.0F) + ((1.0F    ) * t2));
          tbelief[i] = LocParticle(snum, ltrav);
          stotal[snum]++;
          LDEBUG("rand particle[%d]: (%d, %f)", i, snum, ltrav);
        }
      else
        {
          tbelief[i] = itsBeliefParticles[sind];
          stotal[itsBeliefParticles[sind].segnum]++;
          LDEBUG("old  particle[%d]", sind);
        }
   }

  for(uint i = 0; i < stotal.size(); i++) LDEBUG("seg[%d]: %d",i, stotal[i]);

  // copy all the particles to our belief
  for(uint i = 0; i < NUM_PARTICLES; i++)
    {
      itsBeliefParticles[i] = tbelief[i];

      // convert to Point2D<int>
      Point2D<int> loc = itsTopologicalMap->
        getLocation(tbelief[i].segnum, tbelief[i].lentrav);
      itsBeliefLocations[i] = loc;
    }
}

// ######################################################################
void Beobot2_GistSalLocalizerMasterI::objectUpdateBelief()
{
  // make sure at least 1 object is found
  uint c = 0;
  for(uint i = 0; i < itsMatchFound.size(); i++) if(itsMatchFound[i]) c++;

  // if there are no matches update accordingly
  if(c == 0) { noObjectUpdateBelief(); return; }

  // setup weight and sigma for decision boundary
  Dims mDims = itsTopologicalMap->getMapDims();
  Point2D<int> brMap(mDims.w(), mDims.h());
  float mDiag = brMap.distance(Point2D<int>(0,0));
  float sigma = .05*mDiag;
  LDEBUG("map diagonal: %f -> sigma: %f", mDiag, sigma);

  // accweight is for easy calculation for choosing a random particle
  std::vector<float> weight(NUM_PARTICLES);
  std::vector<float> accweight(NUM_PARTICLES);

  // for each particles
  float accw = 0.0F;
  for(uint i = 0; i < NUM_PARTICLES; i++)
    {
      uint  snum  = itsBeliefParticles[i].segnum;
      float ltrav = itsBeliefParticles[i].lentrav;
      LDEBUG("particle[%d]: %d, %f", i, snum, ltrav);

      // go through each object found
      float pObjObs = 1.0;
      for(uint index = 0; index < itsMatchFound.size(); index++)
        {
          if(itsMatchFound[index])
            {
              // get the location of the object found
              uint  snumMatch  = itsSegNumMatch[index];
              float ltravMatch = itsLenTravMatch[index];
              LDEBUG("Match[%d]: [%d %f]", index, snumMatch, ltravMatch);

              // get the distance between the two points
              float dist = itsTopologicalMap->
                getDistance(snum, ltrav, snumMatch, ltravMatch);





              // original match from 
              float pOMatch = 1.0/(sigma * sqrt(2.0 * M_PI)) *
                pow(M_E, -dist*dist/(2.0*sigma*sigma));
              pObjObs *= pOMatch;








              LDEBUG("dist: %f -> pOMatch: %f -> %f", dist, pOMatch, pObjObs);
            }
        }

      // update weight
      weight[i] = pObjObs;
      accweight[i] = weight[i] + accw;
      accw = accweight[i];
    }



  LDEBUG("accw: %f",accw);
  // add 1% weight - because objects are more exacting WAS 20%
  accw *= 1.01F;
  LDEBUG("accw+ noise: %f",accw);

  // weighted resample NUM_PARTICLES particles
  std::vector<LocParticle> tbelief;
  std::vector<int> stotal(itsTopologicalMap->getSegmentNum());
  for(uint i = 0; i < stotal.size(); i++) stotal[i] = 0;

  uint nsegment = itsTopologicalMap->getSegmentNum();

  for(uint i = 0; i < NUM_PARTICLES; i++)
    LDEBUG("p[%d]: %f %f ",i, weight[i], accweight[i]);

  for(uint i = 0; i < NUM_PARTICLES; i++)
    {
      // draw a random value between 0 and accw
      float rval  = accw * float(rand()/(RAND_MAX + 1.0));

      // get the random index
      uint sind = NUM_PARTICLES;
      for(uint j = 0; j < NUM_PARTICLES; j++)
        if(rval < accweight[j]) { sind = j; j = NUM_PARTICLES; }
      LDEBUG("rval: %f -> %d", rval, sind);

      // if need to create a random particle
      if(sind == NUM_PARTICLES)
        {
          // create initial random particles
          float t  = rand()/(RAND_MAX + 1.0);
          float t2 = rand()/(RAND_MAX + 1.0);

          uint  snum  = uint ((0)    + ((nsegment) * t ));
          float ltrav = float((0.0F) + ((1.0F    ) * t2));
          tbelief.push_back(LocParticle(snum, ltrav));
          stotal[snum]++;
          LDEBUG("rand particle[%d]: (%d, %f)", i, snum, ltrav);
        }
      else
        {
          tbelief.push_back(itsBeliefParticles[sind]);
          stotal[itsBeliefParticles[sind].segnum]++;
          LDEBUG("old  particle[%d]", sind);
        }
    }

  for(uint i = 0; i < stotal.size(); i++) LDEBUG("[%d]: %d",i, stotal[i]);

  // copy all the particles to our belief
  for(uint i = 0; i < NUM_PARTICLES; i++)
    {
      itsBeliefParticles[i] = tbelief[i];

      // convert to Point2D<int>
      Point2D<int> loc = itsTopologicalMap->
        getLocation(tbelief[i].segnum, tbelief[i].lentrav);
      itsBeliefLocations[i] = loc;
    }
}

// ######################################################################
void Beobot2_GistSalLocalizerMasterI::noObjectUpdateBelief()
{
  // check timer of last matches
  
  // start adding randomness after awhile


}

// ######################################################################
void Beobot2_GistSalLocalizerMasterI::accumulatedOdometryUpdateBelief
(float acc_odo_dist)
{
  // accweight is for easy calculation for choosing a random particle
  std::vector<float> weight(NUM_PARTICLES);
  std::vector<float> accweight(NUM_PARTICLES);

  // for each particles
  float accw = 0.0F;
  for(uint i = 0; i < NUM_PARTICLES; i++)
    {
      uint  snum  = itsBeliefParticles[i].segnum;
      float ltrav = itsBeliefParticles[i].lentrav;
      LINFO("particle[%d]: %d, %f", i, snum, ltrav);

      float slen = itsTopologicalMap->getSegmentLength(snum);
      float acc_odo_ltrav = acc_odo_dist/slen;

      float pAccOdo = 0.2;     

      float slack_ltrav = 5.0/slen;
      if(slack_ltrav < .25) slack_ltrav = .25;

      LDEBUG("slen: %f aoltrav: %f slacktrav: %f", 
             slen, acc_odo_ltrav, slack_ltrav);

      if(acc_odo_ltrav > (1.0F+slack_ltrav)) 
        {
          // don't add anymore weight
        }
      else
        {
          if(acc_odo_ltrav > 1.0) acc_odo_ltrav = 1.0;          

          // std. deviation is around 1 meter
          float sigma = 1.0;
          

          LDEBUG("[%d %f]:[%d %f]", snum, ltrav, snum, acc_odo_ltrav);
          
          // get the distance between the two points
          float dist = itsTopologicalMap->
            getDistance(snum, ltrav, snum, acc_odo_ltrav);
      
          // original match from 
          pAccOdo += 1.0/(sigma * sqrt(2.0 * M_PI)) *
            pow(M_E, -dist*dist/(2.0*sigma*sigma));

          LINFO("dist: %f -> pAccOdo -> %f", dist, pAccOdo);
        }

      // update weight
      weight[i] = pAccOdo;
      accweight[i] = weight[i] + accw;
      accw = accweight[i];
    }


  LDEBUG("accw: %f",accw);

  // add 1% weight - because objects are more exacting
  // was random when no objects wa
  accw *= 1.01F;
  LDEBUG("accw+ noise: %f",accw);

  // weighted resample NUM_PARTICLES particles
  std::vector<LocParticle> tbelief;
  std::vector<int> stotal(itsTopologicalMap->getSegmentNum());
  for(uint i = 0; i < stotal.size(); i++) stotal[i] = 0;

  uint nsegment = itsTopologicalMap->getSegmentNum();

  for(uint i = 0; i < NUM_PARTICLES; i++)
    LDEBUG("p[%d]: %f %f ",i, weight[i], accweight[i]);

  for(uint i = 0; i < NUM_PARTICLES; i++)
    {
      // draw a random value between 0 and accw
      float rval  = accw * float(rand()/(RAND_MAX + 1.0));

      // get the random index
      uint sind = NUM_PARTICLES;
      for(uint j = 0; j < NUM_PARTICLES; j++)
        if(rval < accweight[j]) { sind = j; j = NUM_PARTICLES; }
      LDEBUG("rval: %f -> %d", rval, sind);

      // if need to create a random particle
      if(sind == NUM_PARTICLES)
        {
          // create initial random particles
          float t  = rand()/(RAND_MAX + 1.0);
          float t2 = rand()/(RAND_MAX + 1.0);

          uint  snum  = uint ((0)    + ((nsegment) * t ));
          float ltrav = float((0.0F) + ((1.0F    ) * t2));
          tbelief.push_back(LocParticle(snum, ltrav));
          stotal[snum]++;
          LDEBUG("rand particle[%d]: (%d, %f)", i, snum, ltrav);
        }
      else
        {
          tbelief.push_back(itsBeliefParticles[sind]);
          stotal[itsBeliefParticles[sind].segnum]++;
          LDEBUG("old  particle[%d]", sind);
        }
    }

  for(uint i = 0; i < stotal.size(); i++) LDEBUG("[%d]: %d",i, stotal[i]);

  // copy all the particles to our belief
  for(uint i = 0; i < NUM_PARTICLES; i++)
    {
      itsBeliefParticles[i] = tbelief[i];

      // convert to Point2D<int>
      Point2D<int> loc = itsTopologicalMap->
        getLocation(tbelief[i].segnum, tbelief[i].lentrav);
      itsBeliefLocations[i] = loc;
    }

}

// ######################################################################
void Beobot2_GistSalLocalizerMasterI::objectUpdateBelief(uint index)
{
  // make sure this object is found
  if(!itsMatchFound[index]) return;

  // setup weight and sigma for decision boundary
  Dims mDims = itsTopologicalMap->getMapDims();
  Point2D<int> brMap(mDims.w(), mDims.h());
  float mDiag = brMap.distance(Point2D<int>(0,0));
  float sigma = .05*mDiag;
  LDEBUG("map diagonal: %f -> sigma: %f", mDiag, sigma);

  // accweight is for easy calculation for choosing a random particle
  std::vector<float> weight(NUM_PARTICLES);
  std::vector<float> accweight(NUM_PARTICLES);

  // for each particles
  float accw = 0.0F;
  for(uint i = 0; i < NUM_PARTICLES; i++)
    {
      uint  snum  = itsBeliefParticles[i].segnum;
      float ltrav = itsBeliefParticles[i].lentrav;
      LDEBUG("particle[%d]: %d, %f", i, snum, ltrav);

      // get the location of the object found
      uint  snumMatch  = itsSegNumMatch[index];
      float ltravMatch = itsLenTravMatch[index];
      LDEBUG("Match[%d]: [%d %f]", index, snumMatch, ltravMatch);

      // get the distance between the two points
      float dist = itsTopologicalMap->
        getDistance(snum, ltrav, snumMatch, ltravMatch);

      float pObjObs = 1.0/(sigma * sqrt(2.0 * M_PI)) *
        pow(M_E, -dist*dist/(2.0*sigma*sigma));
      LDEBUG("dist: %f -> %f", dist, pObjObs);

      // update weight
      weight[i] = pObjObs;
      accweight[i] = weight[i] + accw;
      accw = accweight[i];
    }

  LDEBUG("accw: %f",accw);
  // add 20% weight - because objects are more exacting
  accw *= 1.20F;
  LDEBUG("accw+ noise: %f",accw);

  // weighted resample NUM_PARTICLES particles
  std::vector<LocParticle> tbelief;
  std::vector<int> stotal(itsTopologicalMap->getSegmentNum());
  for(uint i = 0; i < stotal.size(); i++) stotal[i] = 0;

  uint nsegment = itsTopologicalMap->getSegmentNum();

  for(uint i = 0; i < NUM_PARTICLES; i++)
    LDEBUG("p[%d]: %f %f ",i, weight[i], accweight[i]);

  for(uint i = 0; i < NUM_PARTICLES; i++)
    {
      // draw a random value between 0 and accw
      float rval  = accw * float(rand()/(RAND_MAX + 1.0));

      // get the random index
      uint sind = NUM_PARTICLES;
      for(uint j = 0; j < NUM_PARTICLES; j++)
        if(rval < accweight[j]) { sind = j; j = NUM_PARTICLES; }
      LDEBUG("rval: %f -> %d", rval, sind);

      // if need to create a random particle
      if(sind == NUM_PARTICLES)
        {
          // create initial random particles
          float t  = rand()/(RAND_MAX + 1.0);
          float t2 = rand()/(RAND_MAX + 1.0);

          uint  snum  = uint ((0)    + ((nsegment) * t ));
          float ltrav = float((0.0F) + ((1.0F    ) * t2));
          tbelief.push_back(LocParticle(snum, ltrav));
          stotal[snum]++;
          LDEBUG("rand particle[%d]: (%d, %f)", i, snum, ltrav);
        }
      else
        {
          tbelief.push_back(itsBeliefParticles[sind]);
          stotal[itsBeliefParticles[sind].segnum]++;
          LDEBUG("old  particle[%d]", sind);
        }
    }

  for(uint i = 0; i < stotal.size(); i++) LDEBUG("[%d]: %d",i, stotal[i]);

  // copy all the particles to our belief
  for(uint i = 0; i < NUM_PARTICLES; i++)
    {
      itsBeliefParticles[i] = tbelief[i];

      // convert to Point2D<int>
      Point2D<int> loc = itsTopologicalMap->
        getLocation(tbelief[i].segnum, tbelief[i].lentrav);
      itsBeliefLocations[i] = loc;
    }
}

// ######################################################################
void Beobot2_GistSalLocalizerMasterI::setLocation()
{
  // for each point
  float maxscore = 0.0F;
  for(uint i = 0; i < itsBeliefLocations.size(); i++)
    {
      // get distances with the neighbors closer than MAX_ERROR_DIST
      float score = 0.0F;
      Point2D<int> a = itsBeliefLocations[i];
      uint aseg = itsBeliefParticles[i].segnum;
      for(uint j = 0; j < itsBeliefLocations.size(); j++)
        {
          Point2D<int> b = itsBeliefLocations[j];
          float dist = a.distance(b);

          uint bseg = itsBeliefParticles[j].segnum;
          float cscore = 0.0; float sthresh = MAX_LOC_ERROR/2.0;
          if(dist < sthresh) // (0.0  ... 2.5] -> (1.0 -> 0.8]
            {
              cscore = (1.0  - (dist - 0.0)/sthresh * 0.2);
            }
          else if(dist < sthresh*2) // 2.5 ... 5.0] -> [0.8 ... 0.2]
            {
              cscore = (0.8 - (dist - sthresh)/sthresh * 0.6);
            }
          if(aseg != bseg) cscore *= .5;
          score += cscore;
        }

      // update max location
      if(score > maxscore)
        {
          maxscore = score;
          itsLocation = itsBeliefLocations[i];
          itsSegmentLocation = itsBeliefParticles[i].segnum;
          itsSegmentLengthTraveled = itsBeliefParticles[i].lentrav;
        }
    }

  LDEBUG("max score: %f: (%d, %d) = [%d %f]", maxscore,
         itsLocation.i, itsLocation.j,
         itsSegmentLocation, itsSegmentLengthTraveled);
}

// ######################################################################
Image<PixRGB<byte> >
Beobot2_GistSalLocalizerMasterI::getSalImage
( Image<PixRGB<byte> > ima,
  std::vector<rutz::shared_ptr<VisualObject> > inputVO,
  std::vector<Point2D<int> > objOffset,
  std::vector<bool> found)
{
  int w = ima.getWidth();  int h = ima.getHeight();
  Image<PixRGB<byte> > dispIma(w,h,ZEROS);
  inplacePaste(dispIma,ima, Point2D<int>(0,0));

  // display the salient regions
  // and indicate if each is found or not
  LDEBUG("number of input objects: %" ZU , inputVO.size());
  for(uint i = 0; i < inputVO.size(); i++)
    {
      Rectangle r(objOffset[i], inputVO[i]->getImage().getDims());
      Point2D<int> salpt = objOffset[i] + inputVO[i]->getSalPoint();
      if(!found[i])
        {
          drawRect(dispIma,r,PixRGB<byte>(255,0,0));
          drawDisk(dispIma, salpt, 3, PixRGB<byte>(255,0,0));
        }
      else
        {
          drawRect(dispIma,r,PixRGB<byte>(0,255,0));
          drawDisk(dispIma, salpt, 3, PixRGB<byte>(0,255,0));
        }
      LDEBUG("found: %d", int(found[i]));

      std::string ntext(sformat("%d", i));
      writeText(dispIma, objOffset[i] + inputVO[i]->getSalPoint(),
                ntext.c_str());
    }

  return dispIma;
}


// ######################################################################
Image<PixRGB<byte> >
Beobot2_GistSalLocalizerMasterI::getBeliefImage(uint w, uint h, float &scale)
{
  Image< PixRGB<byte> > res(w,h,ZEROS);

  // get the map from the topolagical map class
  Image< PixRGB<byte> > mapImg = itsTopologicalMap->getMapImage(w, h);
  Dims d = itsTopologicalMap->getMapDims();

  // add the particles on the map
  scale = float(mapImg.getWidth())/float(d.w());
  for(uint i = 0; i < itsBeliefParticles.size(); i++)
    {
      // get the point
      Point2D<int> loc(itsBeliefLocations[i].i * scale,
                       itsBeliefLocations[i].j * scale );
      LDEBUG("point: %d %d", loc.i, loc.j);

      drawDisk(mapImg, loc, 2, PixRGB<byte>(0,255,255));
    }

  // draw circle to the most likely location of the object
  Point2D<int> loc_int(itsLocation.i*scale, itsLocation.j*scale);
  drawDisk(mapImg, loc_int, 6, PixRGB<byte>(0,0,255));
  drawCircle(mapImg, loc_int, int((MAX_LOC_ERROR/4)*scale),
             PixRGB<byte>(0,255,0), 2);
  drawCircle(mapImg, loc_int, int(MAX_LOC_ERROR*scale),
             PixRGB<byte>(0,0,255), 2);

  inplacePaste(res, mapImg, Point2D<int>(0,0));

  // get the segment belief histogram
  rutz::shared_ptr<Histogram> shist = getSegmentBeliefHistogram();

  // check which side the histogram is going to be appended to
  uint wslack = w - mapImg.getWidth();
  uint hslack = h - mapImg.getHeight();
  if(hslack >= wslack)
    {
      Image<byte> sHistImg =
        shist->getHistogramImage(w, hslack, 0.0F, float(NUM_PARTICLES));
      inplacePaste(res, Image<PixRGB<byte> >(sHistImg),
                   Point2D<int>(0,mapImg.getHeight()));
    }
  else
    {
      Image<byte> sHistImg =
        shist->getHistogramImage(h, wslack, 0.0F, float(NUM_PARTICLES));

      Image<PixRGB<byte> >
        t = Image<PixRGB<byte> >(flipHoriz(transpose(sHistImg)));
      inplacePaste(res, t, Point2D<int>(mapImg.getWidth(), 0));
    }
  return res;
}


// ######################################################################
Image<PixRGB<byte> > Beobot2_GistSalLocalizerMasterI::getDisplayImage()
{
  // if search is not done don't need to get those information
  Image<PixRGB<byte> > ima = itsInputImage;
  uint w = ima.getWidth(); uint h = ima.getHeight();
  Image<PixRGB<byte> > dispIma(5*w,3*h,ZEROS);

  uint ninput = itsInputVO.size();
  std::vector<bool> mfound(ninput);
  std::vector<rutz::shared_ptr<VisualObject> > iObject(ninput);
  std::vector<Point2D<int> > iOffset(ninput);
  for(uint i = 0; i < ninput; i++)
    {
      mfound[i]     = itsMatchFound[i];
      iObject[i]    = itsInputVO[i];
      iOffset[i]    = itsInputObjOffset[i];
    }

  // display the results
  Image<PixRGB<byte> > salIma = getSalImage(ima, iObject, iOffset, mfound);
  inplacePaste(dispIma, zoomXY(salIma), Point2D<int>(0,0));

  // display the gist histogram
  Image<byte> gistHistImg =
    itsSegmentHistogram->getHistogramImage(w*3, h, 0.0F, 1.0F);
  inplacePaste(dispIma, Image<PixRGB<byte> >(gistHistImg), Point2D<int>(0,2*h));

  // display the localization belief
  float scale;
  Image<PixRGB<byte> > beliefImg = getBeliefImage(w*2, h*3, scale);

  // draw the ground truth
  Point2D<float> pgt = itsEnvironment->getLocationFloat(itsSnumGT, itsLtravGT);
  float xGT = pgt.i, yGT = pgt.j;
  if(itsLtravGT != -1.0F)
    {
      Point2D<int> loc(int(xGT*scale + .5), int(yGT*scale + .5));
      LINFO("Ground Truth disp %f %f -> %d %d", xGT, yGT, loc.i, loc.j);
      drawDisk(beliefImg,loc, 4, PixRGB<byte>(255,0,0));
    }

  // show where the objects indicate its position to be
  uint numObjectFound = 0;
  for(uint i = 0; i < ninput; i++)
    {
      if(mfound[i])
        {
          numObjectFound++;
          uint  snum  = itsSegNumMatch[i];
          float ltrav = itsLenTravMatch[i];
          Point2D<float> p = itsEnvironment->getLocationFloat(snum, ltrav);
          float x = p.i, y = p.j;
          Point2D<int> loc(int(x*scale + .5), int(y*scale + .5));
          LINFO("obj[%d] res: %f %f -> %d %d",i, x, y, loc.i, loc.j);
          drawDisk(beliefImg, loc, 3, PixRGB<byte>(255,255,0));
        }
    }
  inplacePaste(dispIma, beliefImg, Point2D<int>(3*w,0));

  // display a found object found
  uint fcount = 0;
  for(uint i = 0; i < ninput; i++)
    {
      if(mfound[i] && fcount == 0)
        {
          fcount++;

          //display the first object match found
          Image<PixRGB<byte> >matchIma = itsVOmatchImage[i];
          std::string ntext(sformat("object[%d]", i));
          writeText(matchIma, Point2D<int>(0,0), ntext.c_str());
          inplacePaste(dispIma, matchIma, Point2D<int>(2*w,0));
        }
    }
  if(fcount == 0) writeText(dispIma, Point2D<int>(2*w,0),"no objects found");

  return dispIma;
}


// ######################################################################
void Beobot2_GistSalLocalizerMasterI::getGroundTruth
(uint fNum, uint &snumGT, float &ltravGT, float &dx, float &dy)
{
  uint sfnum [4] = { 2363,  567, 2361,  571 };
  uint ssfnum[4] = {    0, 2363, 2930, 5291 };

  int afnum = 0;
  for(uint i = 0; i < 4; i++)
    {
      if((fNum >= ssfnum[i]) && (fNum < ssfnum[i]+sfnum[i]))
        {
          afnum = fNum - ssfnum[i];
          dx = 1.0/float(sfnum[i]); dy = 0.0F;
          snumGT = i; ltravGT = float(afnum)/float(sfnum[i]);
          break;
        }
    }

  // add random error to the robot movement
  dx = 0.0F; //dx = 1.0/866.0;
  dy = 0.0F; //FIX
}

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

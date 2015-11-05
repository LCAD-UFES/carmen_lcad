/*!@file Robots2/Beobot2/Localization/Beobot2_GistSalLocalizerWorker.C
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Beobot2/Localization/Beobot2_GistSalLocalizerWorker.C $
// $Id: Beobot2_GistSalLocalizerWorker.C 15441 2012-11-14 21:28:03Z kai $
//


#include "Robots/Beobot2/Localization/Beobot2_GistSalLocalizerWorker.H"

#include "Image/MathOps.H"      // for inPlaceNormalize()
#include "Image/CutPaste.H"     // for inplacePaste()
#include "Image/DrawOps.H"      // for drawing
#include "Image/ShapeOps.H"     // for decXY()
#include "Image/MatrixOps.H"    // for matrixMult(), matrixInv(), etc

#ifndef BEOBOT2_GISTSALLOCALIZERWORKERI_C
#define BEOBOT2_GISTSALLOCALIZERWORKERI_C

// ######################################################################
Beobot2_GistSalLocalizerWorkerI::Beobot2_GistSalLocalizerWorkerI
(OptionManager& mgr,const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsInputFnum(-1),
  itsLastSearchDone(-1)
{
  itsWorkerIndex = 0;
  itsNumWorkers  = 1;
  itsEmptyQueue  = false;
}

// ######################################################################
void Beobot2_GistSalLocalizerWorkerI::start1()
{
  //uint w = 160; uint h = 120;
  //itsInputWin.reset(new XWinManaged(Dims(w, h), 0, 2*(h+30),
  //                                  "GSL_W:Input" ));
}

// ######################################################################
Beobot2_GistSalLocalizerWorkerI::~Beobot2_GistSalLocalizerWorkerI()
{ }

// ######################################################################
void Beobot2_GistSalLocalizerWorkerI::registerTopics()
{
  this->registerSubscription("LandmarkSearchQueueMessageTopic");
  this->registerSubscription("LandmarkMatchResultMessageTopic");
  this->registerSubscription("CancelSearchMessageTopic");

  this->registerPublisher("LandmarkMatchResultMessageTopic");
  this->registerPublisher("LandmarkSearchStatMessageTopic");
  this->registerPublisher("SearchDoneMessageTopic");
}

// ######################################################################
void Beobot2_GistSalLocalizerWorkerI::evolve()
{
  // always check if we are emptying queue first
  its_job_queue_mutex.lock();
  if(itsEmptyQueue)
    {
      itsJobQueue.clear();
      itsEmptyQueue = false;

      if(itsLastSearchDone < itsInputFnum)
        {
          BeobotEvents::SearchDoneMessagePtr msg =
            new BeobotEvents::SearchDoneMessage;

          LINFO("Publishing SearchDoneMessage");
          publish("SearchDoneMessageTopic", msg);
          itsLastSearchDone = itsInputFnum;
        }
    }
  bool jobQueueEmpty = itsJobQueue.empty();
  its_job_queue_mutex.unlock();

  if (!jobQueueEmpty)
    {
      // get the next job
      its_job_queue_mutex.lock();
      bool hasJob = false;
      GSlocJobData cjob(0, 0, 0, 0, 0);

      while(!hasJob && !itsJobQueue.empty())
        {
          // pop jobs itsNumWorkers at a time
          for(uint i = 0; i < itsNumWorkers; i++)
            {
              if(i == itsWorkerIndex)
                { cjob = itsJobQueue.front(); hasJob = true; }
              if(!itsJobQueue.empty())
                itsJobQueue.pop_front();
              else
                i = itsNumWorkers;
            }

          // if the current object has been found, skip
          its_results_mutex.lock();
          if(itsMatchFound[cjob.objNum]) hasJob = false;
          its_results_mutex.unlock();
        }
      its_job_queue_mutex.unlock();

      if(hasJob) compute(cjob);
      if(itsJobQueue.empty()) itsEmptyQueue = true;
    }
  else itsEmptyQueue = false;
}

// ######################################################################
void Beobot2_GistSalLocalizerWorkerI::updateMessage
(const RobotSimEvents::EventMessagePtr& eMsg, const Ice::Current&)
{
  // Get a gist-sal message
  if(eMsg->ice_isA("::BeobotEvents::LandmarkSearchQueueMessage"))
  {
    BeobotEvents::LandmarkSearchQueueMessagePtr lsqMsg =
      BeobotEvents::LandmarkSearchQueueMessagePtr::dynamicCast(eMsg);

    //Get the current request ID
    int currRequestID = lsqMsg->RequestID;
    itsInputFnum = currRequestID;

    LINFO("Got an lsqMessage with Request ID = %d", currRequestID);

    // get the inputImage
    its_input_info_mutex.lock();
    itsInputImage = Ice2Image<PixRGB<byte> >(lsqMsg->currIma);
    //itsInputWin->setTitle(sformat("WM: %d",itsInputFnum).c_str());
    //itsInputWin->drawImage(itsInputImage, 0, 0);

    // get the salient region information
    itsInputVO.clear();
    itsVOKeypointsComputed.clear();
    itsInputObjOffset.clear();
    uint inputSize = lsqMsg->salientRegions.size();
    for(uint i = 0; i < inputSize; i++)
      {
        BeobotEvents::SalientRegion salReg = lsqMsg->salientRegions[i];
        LDEBUG("W[%4d] sp[%4d,%4d] rect[%4d,%4d,%4d,%4d]",
               i, salReg.salpt.i, salReg.salpt.j,
               salReg.objRect.tl.i, salReg.objRect.tl.j,
               salReg.objRect.br.i, salReg.objRect.br.j);

        // print the pre-attentive feature vector
        std::vector<float> features;
        uint fsize = salReg.salFeatures.size();
        for(uint j = 0; j < fsize; j++)
          {
            features.push_back(salReg.salFeatures[j]);
            LDEBUG("[%4d]:%7f", j, salReg.salFeatures[j]);
          }

        Point2D<int> salpt(salReg.salpt.i, salReg.salpt.j);
        Point2D<int> offset( salReg.objRect.tl.i, salReg.objRect.tl.j);
        Rectangle rect = Rectangle::tlbrO
          (salReg.objRect.tl.j, salReg.objRect.tl.i,
           salReg.objRect.br.j, salReg.objRect.br.i);

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
        itsVOKeypointsComputed.push_back(false);
        itsInputObjOffset.push_back(offset);

        LDEBUG("[%d] image[%d]: %s sal:[%d,%d] offset:[%d,%d]",
               currRequestID, i, iName.c_str(),
               (salpt - offset).i, (salpt - offset).j,
               offset.i, offset.j);
      }
    its_input_info_mutex.unlock();

    its_results_mutex.lock();
    itsMatchFound.clear();
    itsVOmatch.clear();         itsVOmatch.resize(inputSize);
    itsLmkMatch.clear();        itsLmkMatch.resize(inputSize);
    itsSegNumMatch.clear();     itsSegNumMatch.resize(inputSize);
    itsLenTravMatch.clear();    itsLenTravMatch.resize(inputSize);
    itsNumObjectSearch.clear(); itsNumObjectSearch.resize(inputSize);
    for(uint i = 0; i < inputSize; i++) itsMatchFound.push_back(false);
    for(uint i = 0; i < inputSize; i++) itsNumObjectSearch[i] = 0;
    itsNumJobsProcessed = 0;
    its_results_mutex.unlock();

    // fill the job queue
    its_job_queue_mutex.lock();
    itsJobQueue.clear();
    uint njobs = lsqMsg->jobs.size();
    for(uint i = 0; i < njobs; i++)
      {
        BeobotEvents::LandmarkSearchJob tempJob = lsqMsg->jobs[i];
        itsJobQueue.push_back
          (GSlocJobData(tempJob.inputSalRegID,
                        tempJob.dbSegNum,
                        tempJob.dbLmkNum,
                        tempJob.dbVOStart,
                        tempJob.dbVOEnd));
      }

    // print the job queue
    std::list<GSlocJobData>::iterator itr = itsJobQueue.begin();
    uint count = 0;
    while (itr != itsJobQueue.end())
      {
        LDEBUG("[%5d] match obj[%d] lDB[%3d][%3d]:[%3d,%3d]", count,
               (*itr).objNum, (*itr).segNum, (*itr).lmkNum,
               (*itr).voStartNum,(*itr).voEndNum);
        itr++; count++;
      }
    its_job_queue_mutex.unlock();
  }

  // Got a landmark match results - stop searching for that salient region
  else if(eMsg->ice_isA("::BeobotEvents::LandmarkMatchResultMessage"))
  {
    BeobotEvents::LandmarkMatchResultMessagePtr lmrMsg =
      BeobotEvents::LandmarkMatchResultMessagePtr::dynamicCast(eMsg);

    //Get the current request ID
    //int currRequestID = gistSalMsg->RequestID;

    BeobotEvents::LandmarkSearchJob tempJob = lmrMsg->matchInfo;
    LINFO("Got an lmrMessage");

    LINFO("LMR -> found match[%d]: with itsLandmarkDB[%d][%d]",
          tempJob.inputSalRegID, tempJob.dbSegNum, tempJob.dbLmkNum);

    its_results_mutex.lock();
    if(!itsMatchFound[tempJob.inputSalRegID])
      {
        itsMatchFound[tempJob.inputSalRegID]   = true;
        itsSegNumMatch[tempJob.inputSalRegID]  = lmrMsg->segNumMatch;
        itsLenTravMatch[tempJob.inputSalRegID] = lmrMsg->lenTravMatch;
      }
    its_results_mutex.unlock();
  }

  else if(eMsg->ice_isA("::BeobotEvents::CancelSearchMessage"))
    {
      its_job_queue_mutex.lock();
      itsEmptyQueue = true;
      its_job_queue_mutex.unlock();

      its_results_mutex.lock();
      LINFO("CancelSearchMessage: %d processed here", itsNumJobsProcessed);
      its_results_mutex.unlock();
    }
}

// ######################################################################
void Beobot2_GistSalLocalizerWorkerI::setEnvironment
(rutz::shared_ptr<Environment> env)
{
  itsEnvironment = env;

  //! from its environment: topological map
  itsTopologicalMap = env->getTopologicalMap();

  //! from its environment: visual landmark database
  itsLandmarkDB = env->getLandmarkDB();
}

// ######################################################################
void Beobot2_GistSalLocalizerWorkerI::setWorkerInformation
(uint index, uint totalNumWorkers)
{
  itsWorkerIndex = index;
  itsNumWorkers = totalNumWorkers;
}

// ######################################################################
void Beobot2_GistSalLocalizerWorkerI::compute(GSlocJobData cjob)
{
  LDEBUG("T[%4d] match object[%d] itsLandmarkDB[%d][%d]: [ %d, %d ]",
         itsWorkerIndex, cjob.objNum, cjob.segNum, cjob.lmkNum,
         cjob.voStartNum, cjob.voEndNum);

  its_input_info_mutex.lock();
  uint nvo = itsInputVO.size();
  its_input_info_mutex.unlock();

  if(nvo <= uint(cjob.objNum) ||
     itsLandmarkDB->getNumSegment() <= uint(cjob.segNum) ||
     itsLandmarkDB->getNumSegLandmark(cjob.segNum) <= uint(cjob.lmkNum) ||
     cjob.voStartNum < 0 ||
     itsLandmarkDB->getLandmark(cjob.segNum, cjob.lmkNum)->numObjects()
     <= uint(cjob.voStartNum) ||
     cjob.voEndNum < 0 ||
     itsLandmarkDB->getLandmark(cjob.segNum, cjob.lmkNum)->numObjects()
     <= uint(cjob.voEndNum))
    {
      LINFO("Invalid job[%4d] object[%d] itsLandmarkDB[%d][%d]: [ %d, %d ]",
             itsWorkerIndex, cjob.objNum, cjob.segNum, cjob.lmkNum,
             cjob.voStartNum, cjob.voEndNum);
      return;
    }

  // make sure the VO keypoints are computed
  its_input_info_mutex.lock();
  if(!itsVOKeypointsComputed[cjob.objNum])
    {
      itsInputVO[cjob.objNum]->computeKeypoints();
      itsVOKeypointsComputed[cjob.objNum] = true;
    }
  its_input_info_mutex.unlock();

  // match the object with the range of database
  // we limit the maxScale range to [2/3 ... 3/2] by entering 1.5
  // Now (2/10/2010):
  // we limit the maxScale range to [3/4 ... 4/3] by entering 1.33
  its_input_info_mutex.lock();
  rutz::shared_ptr<VisualObjectMatch> cmatch;
  int ind = itsLandmarkDB->getLandmark(cjob.segNum, cjob.lmkNum)->
    match(itsInputVO[cjob.objNum],
          cmatch, cjob.voStartNum, cjob.voEndNum,
          15.0F, 0.5F, 2.5F, 4, M_PI/4, 1.33F, .25F);
  its_input_info_mutex.unlock();

  // if match is found
  uint nObjSearch = 0;
  if(ind != -1)
    {
      LINFO("-> found match[%d]: %s with itsLandmarkDB[%d][%d]\n %s : %s",
            cjob.objNum, itsInputVO[cjob.objNum]->getName().c_str(),
            cjob.segNum, cjob.lmkNum,
            cmatch->getVoRef()->getName().c_str(),
            cmatch->getVoTest()->getName().c_str());
      nObjSearch = (ind - cjob.voStartNum + 1);

      // store the match information
      // since there is a possibility that there are concurrent threads
      // that also just found it we will choose the first match
      its_results_mutex.lock();
      itsVOmatch[cjob.objNum] = cmatch;
      itsLmkMatch[cjob.objNum] =
        GSlocJobData(cjob.objNum, cjob.segNum, cjob.lmkNum, ind, ind);
      itsSegNumMatch[cjob.objNum] = cjob.segNum;
      itsLenTravMatch[cjob.objNum] =
        itsLandmarkDB->getLenTrav(cjob.segNum, cjob.lmkNum, ind);
      itsMatchFound[cjob.objNum] = true;
      its_results_mutex.unlock();

      // send the result back to GSLoc_master
      BeobotEvents::LandmarkMatchResultMessagePtr msg =
        new BeobotEvents::LandmarkMatchResultMessage;

      msg->RequestID = itsInputFnum;
      msg->voMatchImage =
        Image2Ice(getMatchImage(cjob.objNum, itsInputImage.getDims()));
      BeobotEvents::LandmarkSearchJob tempJob;
      tempJob.inputSalRegID  = cjob.objNum;
      tempJob.dbSegNum       = cjob.segNum;
      tempJob.dbLmkNum       = cjob.lmkNum;
      tempJob.dbVOStart      = ind;
      tempJob.dbVOEnd        = ind;
      msg->matchInfo = tempJob;

      // location of the matched database salient region
      msg->segNumMatch = cjob.segNum;
      msg->lenTravMatch = itsLenTravMatch[cjob.objNum];

      LINFO("Publishing lmrMessage with ID: %d[%4d,%10.7f]",
            itsInputFnum, msg->segNumMatch, msg->lenTravMatch);
      publish("LandmarkMatchResultMessageTopic", msg);
    }
  else
    {
      nObjSearch = cjob.voEndNum - cjob.voStartNum + 1;
    }

    BeobotEvents::LandmarkSearchStatMessagePtr msg =
      new BeobotEvents::LandmarkSearchStatMessage;

    msg->RequestID     = itsInputFnum;
    msg->inputSalRegID = cjob.objNum;
    msg->numObjSearch  = nObjSearch;
    msg->found         = (ind != -1);

    LDEBUG("Publishing LandmarkSearchStatMessage");
    publish("LandmarkSearchStatMessageTopic", msg);

    its_results_mutex.lock();
    itsNumJobsProcessed++;
    its_results_mutex.unlock();
}

// ######################################################################
Image<PixRGB<byte> >
Beobot2_GistSalLocalizerWorkerI::getMatchImage(uint index, Dims d)
{
  Image< PixRGB<byte> > result;

  its_results_mutex.lock();
  uint size = itsMatchFound.size();
  its_results_mutex.unlock();

  ASSERT(index < size);
  Point2D<int> objOffset1 = itsInputObjOffset[index];
  Point2D<int> objOffset2 =
    itsLandmarkDB->getLandmark(itsLmkMatch[index].segNum,
                               itsLmkMatch[index].lmkNum)
    ->getOffsetCoords(itsLmkMatch[index].voStartNum);

  bool isODmatch = (itsInputVO[index] == itsVOmatch[index]->getVoRef());
  bool isDOmatch = (itsInputVO[index] == itsVOmatch[index]->getVoTest());

  if(isODmatch)
    result = itsVOmatch[index]->getMatchImage(d, objOffset1, objOffset2);
  else if(isDOmatch)
    result = itsVOmatch[index]->getMatchImage(d, objOffset2, objOffset1);
  else
    {
      LINFO("obj[%d] %s : %s, %s",
            index, itsInputVO[index]->getName().c_str(),
            itsVOmatch[index]->getVoRef()->getName().c_str(),
            itsVOmatch[index]->getVoTest()->getName().c_str());
      LFATAL("object neither ref nor tst");
    }

  return result;
}

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

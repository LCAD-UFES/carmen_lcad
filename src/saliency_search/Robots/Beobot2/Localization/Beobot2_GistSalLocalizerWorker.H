/*!@file Robots2/Beobot2/Localization/Beobot2_GistSalLocalizerWorker.H
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Beobot2/Localization/Beobot2_GistSalLocalizerWorker.H $
// $Id: Beobot2_GistSalLocalizerWorker.H 15425 2012-11-02 21:19:53Z siagian $
//

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"

#include "GUI/XWinManaged.H"

#include "Beobot/GSlocalizer.H"
#include "Beobot/Landmark.H"
#include "SIFT/Histogram.H"
#include "Beobot/LocParticle.H"
#include "Beobot/Environment.H"
#include "Util/Timer.H"



#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/BeobotEvents.ice.H"
#include "Ice/IceImageUtils.H"
#include <IceUtil/Thread.h>

#ifndef BEOBOT2_GISTSALLOCALIZERWORKERI_H
#define BEOBOT2_GISTSALLOCALIZERWORKERI_H

class Beobot2_GistSalLocalizerWorkerI : public RobotBrainComponent
{
public:

  Beobot2_GistSalLocalizerWorkerI(OptionManager& mgr,
      const std::string& descrName = "Beobot2_GistSalLocalizerWorker",
      const std::string& tagName = "Beobot2_GistSalLocalizerWorker");

  ~Beobot2_GistSalLocalizerWorkerI();

  //! Get started. See ModelComponent.
  virtual void start1();

  virtual void evolve();

  //!Get a message
  virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
      const Ice::Current&);

  virtual void registerTopics();

//   //! set the prefix of file to save data  - has to be done
//   void setSavePrefix(std::string prefix);

  //! set the environment - has to be done
  void setEnvironment(rutz::shared_ptr<Environment> env);

  //! set worker index and total number of workers
  void setWorkerInformation(uint index, uint totalNumWorkers);

  //@}

private:

//   // ######################################################################
//   /*! @name private functions */
//   //@{

  //! compute match
  void compute(GSlocJobData cjob);

  //! get the match image
  Image<PixRGB<byte> > getMatchImage(uint index, Dims d);

//   //! get the match
//   GSlocJobData getMatch(uint index);

//   //@}

//   //!  file prefix to save data
//   std::string itsSavePrefix;

  //! all the environment information
  rutz::shared_ptr<Environment> itsEnvironment;

  //! from its environment: topological map
  rutz::shared_ptr<TopologicalMap> itsTopologicalMap;

  //! from its environment: visual landmark database
  rutz::shared_ptr<LandmarkDB> itsLandmarkDB;

  //! the input image, visual objects, and gist
  Image<PixRGB<byte> > itsInputImage;
  std::vector<rutz::shared_ptr<VisualObject> > itsInputVO;
  std::vector<bool> itsVOKeypointsComputed;
  std::vector<Point2D<int> > itsInputObjOffset;
  //  Image<double> itsInputGist;


//   int itsInputFnum;
//   int itsSearchInputFnum;

//   //! ground truth information - default to (0,0.0)
//   uint  itsSnumGT;
//   float itsLtravGT;

//   //! the current robot movement
//   float itsRobotDx;
//   float itsRobotDy;

//   //! segment histogram from the classifier
//   rutz::shared_ptr<Histogram> itsSegmentHistogram;

  //! result of search
  //std::vector<Image<PixRGB<byte> > > itsVOmatchImage;
  std::vector<rutz::shared_ptr<VisualObjectMatch> > itsVOmatch;
  std::vector<GSlocJobData> itsLmkMatch;
  std::vector<uint> itsSegNumMatch;
  std::vector<float> itsLenTravMatch;
  std::vector<bool> itsMatchFound;
  std::vector<uint> itsNumObjectSearch;

//   //! resulting geographic location
//   uint itsSegmentLocation;
//   float itsSegmentLengthTraveled;
//   Point2D<int> itsLocation;

//   //! job queue and number of jobs to do
//   //! Note: they are on jobLock
  std::list<GSlocJobData> itsJobQueue;
//   bool itsIsQueueSorted;         //!< note if the queue is sorted
  uint itsNumJobsProcessed;      //!< number of jobs that has been processed
//   uint itsLastSuccessfulJob;     //!< job index last found
//   uint itsNumObjectFound;        //!< number of objects found
//   uint itsNumJobs;               //!< original number of jobs
   bool itsEmptyQueue;            //!< stop search request

//   //! segment histogram from the belief particles
//   rutz::shared_ptr<Histogram> itsSegmentBeliefHistogram;

//   //! especially for input
//   bool itsOutputReady2;

  //! locks
  IceUtil::Mutex its_job_queue_mutex;  //!< locking jobQueue
//   pthread_mutex_t fnumLock;      //!< locking frame number
//   pthread_mutex_t or2Lock;       //!< locking itsOutputReady2
//   pthread_mutex_t stopSearchLock;//!< locking stop search
  IceUtil::Mutex its_results_mutex;  //!< locking results
  IceUtil::Mutex its_input_info_mutex;  //!< locking the input information
//   pthread_mutex_t particleLock;  //!< locking belief particles

  rutz::shared_ptr<XWinManaged> itsInputWin;

//   rutz::shared_ptr<Timer> itsTimer;

  int itsInputFnum;
  int itsLastSearchDone;

  uint itsWorkerIndex;
  uint itsNumWorkers;
};

// ######################################################################
// Implementation for Beobot2_GisSalLocalizerWorker inline functions
// ######################################################################


#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

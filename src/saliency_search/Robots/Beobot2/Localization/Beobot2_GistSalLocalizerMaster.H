/*!@file Robots2/Beobot2/Localization/Beobot2_GistSalLocalizerMaster.H
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Beobot2/Localization/Beobot2_GistSalLocalizerMaster.H $
// $Id: Beobot2_GistSalLocalizerMaster.H 15441 2012-11-14 21:28:03Z kai $
//


#ifndef BEOBOT2_GISTSALLOCALIZERMASTERI_H
#define BEOBOT2_GISTSALLOCALIZERMASTERI_H

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "GUI/XWinManaged.H"

#include "Beobot/GSlocalizer.H"
#include "Beobot/Landmark.H"
#include "SIFT/Histogram.H"
#include "Beobot/LocParticle.H"
#include "Beobot/Environment.H"
#include "Util/Timer.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/BeobotEvents.ice.H"
#include "Ice/IceImageUtils.H"
#include <IceUtil/Thread.h>


class Beobot2_GistSalLocalizerMasterI : public RobotBrainComponent
{
public:

  // ######################################################################
  /*! @name Constructors and Destructors */
  //@{

  Beobot2_GistSalLocalizerMasterI(OptionManager& mgr,
      const std::string& descrName = "Beobot2_GistSalLocalizerMaster",
      const std::string& tagName = "Beobot2_GistSalLocalizerMaster");

  ~Beobot2_GistSalLocalizerMasterI();

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

  //@}

//   // ######################################################################
//   //! @name Access functions
//   //@{

//   //! get number of objects compared in the search
//   uint getNumObjectSearch(uint index);

//   //! get the environment information
//   rutz::shared_ptr<Environment> getEnvironment();

//   //! set the window to display results
//   void setWindow(rutz::shared_ptr<XWinManaged> inputWin);

//   //! get the input image
//   Image<PixRGB<byte> > getInputImage();

//   //! get the number of objects inputted
//   uint getNumInputObject();

//   //! get the visual object that we try to match
//   rutz::shared_ptr<VisualObject> getInputVO(uint index);

//   //! get the input gist
//   Image<double> getInputGist();

//   //! get the visual object match for the found object
//   rutz::shared_ptr<VisualObjectMatch> getVOmatch(uint index);

//   //! get the segment number of the object match found
//   uint getSegmentNumberMatch(uint index);

//   //! get the length traveled of the object match found
//   float getLengthTraveledMatch(uint index);

//   //! get the object offset of the visual object
//   //! that we try to match
//   Point2D<int> getInputObjOffset(uint index);

//   //! get the last input frame number
//   int getInputFnum();

//   //! get the last input frame number where search is started
//   int getSearchInputFnum();

//   //! get the segment histogram from the segment classifier
//   rutz::shared_ptr<Histogram> getSegmentHistogram();

//   //! get our geographical location
//   Point2D<int> getLocation();

//   //! get our segment location
//   uint getSegmentLocation();

//   //! get the length traveled within the segment
//   float getSegmentLengthTraveled();

//   //! set ground truth
//   void setGroundTruth(uint snum, float ltrav);

//   //! get ground truth
//   void getGroundTruth(uint &snum, float &ltrav);

  //@}

  // ######################################################################
  /*! @name member functions */
  //@{

  //! initialize the localization particles
  void initParticles(std::string belFName = std::string(""));

  void initParticles(uint snum, float ltrav, float variance = 0.0F);

//   //! get the belief particles (usually for recovering crashes)
//   std::vector<LocParticle> getBeliefParticles();

//   //! check if the serach is finished
//   bool outputReady();

//   //! return the result of the matching search
//   bool isMatchFound(uint index);

//   //! input the image, visual object and gist feature for search
//   //! also add the odometry change
  void input();
//   void input(Image<PixRGB<byte> > ima,
//              std::vector<rutz::shared_ptr<VisualObject> > inputVO,
//              std::vector<Point2D<int> > inputObjOffset,
//              int inputFnum, Image<double> cgist,
//              float dx = -1.0F, float dy = -1.0F);

  //! the belief histogram for segment only localization
  rutz::shared_ptr<Histogram> getSegmentBeliefHistogram();

//   //! stop search by cleaning up the queue
//   //! NOTE: this is a hard stop (blocking operation)
//   //!       may take time (500ms, or even longer)
//   void stopSearch();

//   //! stop search by flipping a stop-search bit
//   //! NOTE: this is a soft/non-blocking operation
//   void stopSearch2();

  //! update belief using the input just processed
  //! update our likely location
  void updateBelief();

  //! move the object from the previous location
  void actionUpdateBelief();

  //! update belief using the segment prediction
  void segmentUpdateBelief();

  //! update belief using all the objects found
  void objectUpdateBelief();

  //! update belief using object 'index'
  void objectUpdateBelief(uint index);

  //! update belief when there is no object
  void noObjectUpdateBelief();

  //! update belief using accumulated odometry
  void accumulatedOdometryUpdateBelief(float acc_odo_dist);

  //! set the most likely location
  void setLocation();

  //! save the localizer results
  void saveLocalizerResults();

  //! get the belief image (it is put on top of a map)
  Image<PixRGB<byte> > getBeliefImage(uint w, uint h, float &scale);

  Image<PixRGB<byte> > getDisplayImage();
  Image<PixRGB<byte> > getSalImage
  (Image<PixRGB<byte> > ima,
   std::vector<rutz::shared_ptr<VisualObject> > inputVO,
   std::vector<Point2D<int> > objOffset,
   std::vector<bool> found);

  // set the number of workers that will be
  void setNumWorkers(uint numWorkers);

  inline void setSavePrefix(std::string prefix);

private:


  // ######################################################################
  /*! @name private functions */
  //@{

  //! set the segment and object search priority for landmark DB
  void setSearchPriority();

  //! add the search order preference randomly
  void addRandomPriority();

  //! add the search order preference based on segment
  void addSegmentPriority();

  //! add the search order preference based on saliency match
  void addSaliencyPriority();

  //! add the search order preference based on current belief location
  void addLocationPriority();

  //! hacky way to feed in ground truth
  void getGroundTruth
  (uint fNum, uint &snumGT, float &ltravGT, float &dx, float &dy);



//   //! get the match
//   GSlocJobData getMatch(uint index);

//   //@}

//   //!  file prefix to save data
//   std::string itsSavePrefix;

  //! localization particles for beliefs
  std::vector<LocParticle> itsBeliefParticles;
  std::vector<Point2D<int> > itsBeliefLocations;

  //! all the environment information
  rutz::shared_ptr<Environment> itsEnvironment;

  //! from its environment: topological map
  rutz::shared_ptr<TopologicalMap> itsTopologicalMap;

  //! from its environment: visual landmark database
  rutz::shared_ptr<LandmarkDB> itsLandmarkDB;

  //! the input image, visual objects, and gist
  Image<PixRGB<byte> > itsInputImage;
  std::vector<rutz::shared_ptr<VisualObject> > itsInputVO;
//   std::vector<bool> itsVOKeypointsComputed;
  std::vector<Point2D<int> > itsInputObjOffset;
  Image<double> itsInputGist;

  //! ground truth information - default to (0,0.0)
  uint  itsSnumGT;
  float itsLtravGT;

  //! the current robot movement
  float itsRobotDx;
  float itsRobotDy;

  //! segment histogram from the classifier
  rutz::shared_ptr<Histogram> itsSegmentHistogram;

  //! result of search
  std::vector<Image<PixRGB<byte> > > itsVOmatchImage;
  std::vector<GSlocJobData> itsLmkMatch;
  std::vector<uint> itsSegNumMatch;
  std::vector<uint> itsLmkNumMatch;
  std::vector<uint> itsVobNumMatch;
  std::vector<float> itsLenTravMatch;
  std::vector<bool> itsMatchFound;
  std::vector<uint> itsNumObjectSearch;

  //! resulting geographic location
  uint itsSegmentLocation;
  float itsSegmentLengthTraveled;
  Point2D<int> itsLocation;

  //! job queue and number of jobs to do
  //! Note: they are on jobLock
  std::list<GSlocJobData> itsJobQueue;
//   bool itsIsQueueSorted;         //!< note if the queue is sorted
  uint itsNumJobsProcessed;      //!< number of jobs that has been processed
  uint itsLastSuccessfulJob;     //!< job index last found
  uint itsNumObjectFound;        //!< number of objects found
  uint itsNumJobs;               //!< original number of jobs
//   bool itsStopSearch;            //!< stop search request

  uint itsNumWorkers;            //!< the number of threads that are working
  uint itsNumBusyWorkers;

  //! segment histogram from the belief particles
  rutz::shared_ptr<Histogram> itsSegmentBeliefHistogram;

//   //! especially for input
//   bool itsOutputReady2;

  std::string                     itsSavePrefix;

  //! locks
  IceUtil::Mutex its_job_queue_mutex;        //!< locking jobQueue
  IceUtil::Mutex its_fnum_mutex;             //!< locking frame number
  IceUtil::Mutex its_num_busy_workers_mutex; //!< locking itsOutputReady2
  IceUtil::Mutex its_results_mutex;          //!< locking results
  IceUtil::Mutex its_input_info_mutex;       //!< locking the input information
  IceUtil::Mutex its_gist_mutex;             //!< locking gist features
  IceUtil::Mutex its_particles_mutex;        //!< locking belief particles

  rutz::shared_ptr<XWinManaged> itsInputWin;
  rutz::shared_ptr<XWinManaged> itsResultWin;

  Timer itsSearchTimer;
  std::vector<float> itsTimes;
  bool itsAbort;

  int itsInputFnum;
  int itsSearchInputFnum;
  bool itsStopSent;
};

// ######################################################################
// Implementation for Beobot2_GisSalLocalizerMaster inline functions
// ######################################################################
inline void Beobot2_GistSalLocalizerMasterI::setSavePrefix(std::string prefix)
{ itsSavePrefix = prefix; }

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

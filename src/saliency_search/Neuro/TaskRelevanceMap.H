/*!@file Neuro/TaskRelevanceMap.H Class declarations for task-relevance map class */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/TaskRelevanceMap.H $
// $Id: TaskRelevanceMap.H 13648 2010-07-07 01:32:32Z jshen $
//

#ifndef TASKRELEVANCEMAP_H_DEFINED
#define TASKRELEVANCEMAP_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Channels/InputFrame.H"
#include "Image/LevelSpec.H"
#include "Image/ImageCache.H"
#include "Neuro/NeuroSimEvents.H"
#include "Simulation/SimModule.H"
#include "Simulation/SimEvents.H"
#include "Media/TestImages.H"
// add class definitions for Object Recognition
//class TestImages;

typedef TestImages::SceneData Scene;
typedef TestImages::ObjData Object;

// ######################################################################
//! The task-relevance map base class
// ######################################################################
/*! This is a 2D task-relevance map. It is just a base class with
  virtual function definitions. Various implementations are available
  below.  The TRM is a topographic map where a value of zero indicates
  that a location is not relevant to the task at hand, while a value
  of 1.0 indicates neutral relevance and values higher than 1.0
  indicate high relevance. */
class TaskRelevanceMap : public SimModule
{
public:
  // ######################################################################
  //! @name Constructor, destructor, and reset
  //@{

  //! Ininitialized constructor
  /*! The map will be resized and initialized the first time input() is
    called */
  TaskRelevanceMap(OptionManager& mgr,
                   const std::string& descrName = "Task-Relevance Map",
                   const std::string& tagName = "TaskRelevanceMap");

  //! Destructor
  virtual ~TaskRelevanceMap();

  //@}

private:
  // forbid assignment and copy-construction:
  TaskRelevanceMap& operator=(const TaskRelevanceMap& sm);
  TaskRelevanceMap(const TaskRelevanceMap& sm);
};

// ######################################################################
//! TaskRelevanceMap configurator
// ######################################################################
/*! This will export the --trm-type=XX command-line option and will
  instantiate a TRM of the desired type as the option gets assigned a
  value. As this happens, new options may become available in the
  command-line. To see them, use --help AFTER you have chosen the type
  to use. The current TRM may be retrieved using getTRM(). */
class TaskRelevanceMapConfigurator : public ModelComponent
{
public:
  //! Constructor
  TaskRelevanceMapConfigurator(OptionManager& mgr,
                               const std::string& descrName =
                               "Task-Relevance Map Configurator",
                               const std::string& tagName =
                               "TaskRelevanceMapConfigurator");

  //! destructor
  virtual ~TaskRelevanceMapConfigurator();

  //! Get the chosen TRM
  /*! You should call this during start() of the ModelComponent that
    needs the TRM. CAUTION: this may be a rutz::shared_ptr(NULL) if no
    TRM type has been chosen. */
  nub::ref<TaskRelevanceMap> getTRM() const;

protected:
  OModelParam<std::string> itsTRMtype; //!< type of map

  //! Intercept people changing our ModelParam
  /*! See ModelComponent.H; as parsing the command-line or reading a
    config file sets our name, we'll also here instantiate a
    controller of the proper type (and export its options) */
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

private:
  nub::ref<TaskRelevanceMap> itsTRM; // the map
};


// ######################################################################
//! Stub implementation of TaskRelevanceMap
// ######################################################################
class TaskRelevanceMapStub : public TaskRelevanceMap
{
public:
  //! Constructor
  TaskRelevanceMapStub(OptionManager& mgr,
                       const std::string& descrName = "Task-Relevance Map Stub",
                       const std::string& tagName = "TaskRelevanceMapStub");

  //! Destructor
  virtual ~TaskRelevanceMapStub();
};

// ######################################################################
//! Partial implementation of TaskRelevanceMap for further derivation
// ######################################################################
class TaskRelevanceMapAdapter : public TaskRelevanceMap
{
public:
  //! Constructor
  TaskRelevanceMapAdapter(OptionManager& mgr,
                          const std::string& descrName,
                          const std::string& tagName);

  //! Destructor
  virtual ~TaskRelevanceMapAdapter();

protected:
  //! Callback for when a new input frame is available from the retina
  SIMCALLBACK_DECLARE(TaskRelevanceMapAdapter, SimEventRetinaImage);

  //! Callback for when a new eye activity is going on
  SIMCALLBACK_DECLARE(TaskRelevanceMapAdapter, SimEventSaccadeStatusEye);

  //! Callback for every clock tick, to run our diff equations in integrate()
  SIMCALLBACK_DECLARE(TaskRelevanceMapAdapter, SimEventClockTick);

  //! Callback for every time we should save our outputs
  SIMCALLBACK_DECLARE(TaskRelevanceMapAdapter, SimEventSaveOutput);

  //! LevelSpec for scale of saliency map
  OModelParam<LevelSpec> itsLevelSpec;

  //! Save our internals when saveResults() is called?
  OModelParam<bool> itsSaveResults;

  //! Save our internals
  /*! This implementation just saves the map with a file name prefix
    of "TRM". */
  virtual void save1(const ModelComponentSaveInfo& sinfo);

  //! Get reset. See ModelComponent
  /*! In the adapter, we just free itsMap. */
  virtual void reset1();

  // ### Derived classes will need to implement those:

  //! Called when a new input frame is available from the Retina
  virtual void inputFrame(const InputFrame& f) = 0;

  //! Some TRM implementations will reset their memory upon saccade
  virtual void saccadicSuppression(const bool on) = 0;

  //! Integrate for one time step
  virtual void integrate(SimEventQueue& q) = 0;

  //! This map contains the relevance values.
  /*! The TaskRelevanceMap adapter will initialize this to the correct
    size and all-ones as soon as an input frame is received. As long
    as itsMap is not initialized, derived classes should just do
    nothing. Derived classes should update itsMap with the new
    computed relevance values, as this is the one which will get
    saved, displayed, posted, etc. */
  Image<float> itsMap;
};

// ######################################################################
//! The standard task-relevance map
// ######################################################################
/*! This is our current standard TRM implementation. It is very
  incomplete as there is no good way to actually change relevance of
  anything. It will need in future work to be connected to an agent
  that computes the relevance of fixated locations and feeds that to
  the TRM. */
class TaskRelevanceMapStd : public TaskRelevanceMapAdapter
{
public:
  //! Uninitialized constructor
  TaskRelevanceMapStd(OptionManager& mgr, const std::string& descrName =
                      "Task-Relevance Map Std",
                      const std::string& tagName =
                      "TaskRelevanceMapStd");

  //! Destructor
  virtual ~TaskRelevanceMapStd();

protected:
  //! Learn the TRM for the given task by inspecting several scenes
  OModelParam<bool> itsLearnTRM;

  //! Learn and update the TRM as we inspecting several scenes
  OModelParam<bool> itsLearnUpdateTRM;

  //! If an image is specified with this option, then use it to bias the TRM
  OModelParam<std::string> itsBiasTRM;

  //! Called when a new input frame is available from the Retina
  virtual void inputFrame(const InputFrame& f);

  //! Some TRM implementations will reset their memory upon saccade
  virtual void saccadicSuppression(const bool on);

  //! Integrate for one time step
  virtual void integrate(SimEventQueue& q);
};

// ######################################################################
//! A killStatic task-relevance map
// ######################################################################
/*! This is a very simple TRM where image locations that do not change
  much over time see theri relevance progressively decrease. */
class TaskRelevanceMapKillStatic : public TaskRelevanceMapAdapter
{
public:
  //! Uninitialized constructor
  TaskRelevanceMapKillStatic(OptionManager& mgr, const std::string& descrName =
                      "Task-Relevance Map KillStatic",
                      const std::string& tagName =
                      "TaskRelevanceMapKillStatic");

  //! Destructor
  virtual ~TaskRelevanceMapKillStatic();

protected:
  /*! Anything in the difference between current and cumulated
    previous frames (range 0..255) less than this will be considered
    static */
  OModelParam<float> itsKillStaticThresh;

  /*! Max killing achievable for a purely static location (should be
    in 0.0 .. 1.0 range) */
  OModelParam<float> itsKillStaticCoeff;

  //! Called when a new input frame is available from the Retina
  virtual void inputFrame(const InputFrame& f);

  //! Some TRM implementations will reset their memory upon saccade
  virtual void saccadicSuppression(const bool on);

  //! Integrate for one time step
  virtual void integrate(SimEventQueue& q);

  //! Save our internals
  /*! Call the TaskRelevanceMapAdapter version, plus save our static buf. */
  virtual void save1(const ModelComponentSaveInfo& sinfo);

  //! Get reset. See ModelComponent
  virtual void reset1();

private:
  // cumulative buffer of the luminance image, at the resolution of
  // the saliency map. This is a sliding weighted average of the
  // downscaled luminance of the past few input frames, and it is used
  // to determine how different the current input is from that
  // accumulated average, and in turn to set the task relevance map so
  // that more static regions are less relevant.
  Image<float> itsStaticBuff;
};

// ######################################################################
//! A KillN task-relevance map
// ######################################################################
/*! This is an alternative to IOR, in which we set the relevance of
  the last N attended locations to zero. */
class TaskRelevanceMapKillN : public TaskRelevanceMapAdapter
{
public:
  //! Uninitialized constructor
  TaskRelevanceMapKillN(OptionManager& mgr, const std::string& descrName =
                      "Task-Relevance Map KillN",
                      const std::string& tagName =
                      "TaskRelevanceMapKillN");

  //! Destructor
  virtual ~TaskRelevanceMapKillN();

protected:
  //! Number of inputs to compute over
  OModelParam<int> itsN;

  //! Called when a new input frame is available from the Retina
  virtual void inputFrame(const InputFrame& f);

  //! Some TRM implementations will reset their memory upon saccade
  virtual void saccadicSuppression(const bool on);

  //! Integrate for one time step
  virtual void integrate(SimEventQueue& q);

  //! Get started. See ModelComponent.
  virtual void start1();

  //! Get reset. See ModelComponent
  virtual void reset1();

private:
  ImageCacheMinMax<float> itsCache;
};

// ######################################################################
//! A GistClassify task-relevance map
// ######################################################################
/*! This is an alternative to KillStatic method, we use the gist vector to
 classify the frames into different categories and assign each frame a
 pre-defined TP map*/
class TaskRelevanceMapGistClassify : public TaskRelevanceMapAdapter
{
public:
  //! Uninitialized constructor
  TaskRelevanceMapGistClassify(OptionManager& mgr, const std::string& descrName =
                      "Task-Relevance Map GistClassify",
                      const std::string& tagName =
                      "TaskRelevanceMapGistClassify");

  //! Destructor
  virtual ~TaskRelevanceMapGistClassify();

protected:
  //! Number of inputs to compute over
  OModelParam<std::string> itsClusterCenterFileName; // the clustered center of the gist
  OModelParam<std::string> itsTemplateDir; // the dir of the pre-defined TD map
  OModelParam<int> itsPCADims;
  OModelParam<std::string> itsPCAMatrixName;
  OModelParam<int> itsCacheSize;
  OModelParam<int> itsUpdatePeriod;

  //! Called when a new input frame is available from the Retina
  virtual void inputFrame(const InputFrame& f);

  //! Some TRM implementations will reset their memory upon saccade
  virtual void saccadicSuppression(const bool on);

  //! Integrate for one time step
  virtual void integrate(SimEventQueue& q);

  // classify the current frame's gist to one category
  virtual void gistmatch(Image<float> gist);

  // get the PCA Matrix
  virtual void getPCAMatrix();

  // compute the distance between current gist and gist center
  virtual Image<float> computeGistDist(Image<float> currGist);

  // compute the TD map according to the gistDist
  virtual Image<float> getTDMap(Image<float> gistDist);

  //! Get reset. See ModelComponent
  virtual void reset1();

private:
  Image<float> itsGist;
  Image<float> itsPCAMatrix;
  int itsMatchCategory;
  Image<float> itsCurrentTDMap;
  Image<float> itsTmpTDMap;
  std::string itsTDMapName;
  int itsNumOfCategory;
  int itsUsedDims;
  bool itsMapComputedForCurrentFrame;

  ImageCacheAvg<float> itsTDMap;
  int itsFrame;

};

// ######################################################################
//!  A Tigs task-relevance map
// ######################################################################
/*! This is an alternative to GistClassify method, we use the gist vector to
 get the TD map from a learned Tigs Matrix (see Rob's CVPR 2007 paper) */

class TaskRelevanceMapTigs : public TaskRelevanceMapAdapter
{
public:
  //! Uninitialized constructor
  TaskRelevanceMapTigs(OptionManager& mgr, const std::string& descrName =
                      "Task-Relevance Map Tigs",
                      const std::string& tagName =
                      "TaskRelevanceMapTigs");

  //! Destructor
  virtual ~TaskRelevanceMapTigs();

protected:
  //! Number of inputs to compute over
  OModelParam<std::string> itsTigsMatrixName; // the trained Tigs Matrix file Name
  OModelParam<int> itsPCADims;
  OModelParam<std::string> itsPCAMatrixName;
  OModelParam<int> itsCacheSize;
  OModelParam<int> itsUpdatePeriod;

  //! Called when a new input frame is available from the Retina
  virtual void inputFrame(const InputFrame& f);

  //! Some TRM implementations will reset their memory upon saccade
  virtual void saccadicSuppression(const bool on);

  //! Integrate for one time step
  virtual void integrate(SimEventQueue& q);

  // get the PCA Matrix
  virtual void getPCAMatrix();

  // compute the TD map according to the gistDist
  virtual Image<float> getTDMap(Image<float> gistDist);

  // get the Trained Tigs Matrix
  virtual void getTigsMatrix();

  // decode the 1x300 vector to a 2D image
  virtual Image<float> decode(Image<float> tmp);

  //! Save our internals
  /*! Call the TaskRelevanceMapAdapter version, plus save our static buf. */
  virtual void save1(const ModelComponentSaveInfo& sinfo);

  //! Get reset. See ModelComponent
  virtual void reset1();

private:
  Image<float> itsGist;
  Image<float> itsPCAMatrix;
  Image<float> itsTigsMatrix;
  Image<float> itsCurrentTDMap;
  Image<float> itsTmpTDMap;
  int itsUsedDims;
  bool itsMapComputedForCurrentFrame;

  ImageCacheAvg<float> itsTDMap;
  int itsFrame;
};


// ######################################################################
//! A Tigs task-relevance map A Tigs combined image PCA task-relevance map
// ######################################################################
/*! This is an alternative to GistClassify method, we use the gist vector
  and pca image information as input to compute the TDmap from a learned
  TD map Matrix*/
class TaskRelevanceMapTigs2 : public TaskRelevanceMapAdapter
{
public:
  //! Uninitialized constructor
  TaskRelevanceMapTigs2(OptionManager& mgr, const std::string& descrName =
                        "Task-Relevance Map Tigs2",
                        const std::string& tagName =
                        "TaskRelevanceMapTigs2");

  //! Destructor
  virtual ~TaskRelevanceMapTigs2();

protected:
  //! Number of inputs to compute over
  OModelParam<std::string> itsTigsMatrixName; // the trained Tigs Matrix file Name
  OModelParam<int> itsGistPCADims;
  OModelParam<int> itsImgPCADims;
  OModelParam<std::string> itsGistPCAMatrixName;
  OModelParam<std::string> itsImgPCAMatrixName;
  OModelParam<int> itsCacheSize;
  OModelParam<int> itsUpdatePeriod;

  //! Called when a new input frame is available from the Retina
  virtual void inputFrame(const InputFrame& f);

  //! Some TRM implementations will reset their memory upon saccade
  virtual void saccadicSuppression(const bool on);

  //! Integrate for one time step
  virtual void integrate(SimEventQueue& q);

  // get the Gist PCA Matrix
  virtual void getGistPCAMatrix();

  // get the image PCA Matrix
  virtual void getImgPCAMatrix();

  // compute the TD map according to the gistDist
  virtual Image<float> getTDMap(Image<float> gistDist);

  // get the Trained Tigs Matrix
  virtual void getTigsMatrix();

  // decode the 1x300 vector to a 2D image
  virtual Image<float> decode(Image<float> tmp);

  //! Save our internals
  /*! Call the TaskRelevanceMapAdapter version, plus save our static buf. */
  virtual void save1(const ModelComponentSaveInfo& sinfo);

  //! Get reset. See ModelComponent
  virtual void reset1();

private:
  Image<float> itsGist;
  Image<float> itsGistPCAMatrix;
  Image<float> itsImgPCAMatrix;
  Image<float> itsTigsMatrix;
  Image<float> itsCurrentTDMap;
  Image<float> itsTmpTDMap;
  Image<float> itsCurrFrameVec;
  int itsUsedDims;
  bool itsMapComputedForCurrentFrame;

  ImageCacheAvg<float> itsTDMap;
  int itsFrame;
};

///// Task Relevance Map based on VLM input /////
class TaskRelevanceMapSocial : public TaskRelevanceMapAdapter
{
public:
  //! Uninitialized constructor
  TaskRelevanceMapSocial(OptionManager& mgr, const std::string& descrName =
                      "Task-Relevance Map Social",
                      const std::string& tagName =
                      "TaskRelevanceMapSocial");

  //! Destructor
  virtual ~TaskRelevanceMapSocial();

protected:
  //! Number of inputs to compute over
  //OModelParam<int> itsN;

  //! Called when a new input frame is available from the Retina
  virtual void inputFrame(const InputFrame& f);

  //! Some TRM implementations will reset their memory upon saccade
  virtual void saccadicSuppression(const bool on);

  //! Integrate for one time step
  virtual void integrate(SimEventQueue& q);

  //! Get started. See ModelComponent.
  virtual void start1();

  //! Get reset. See ModelComponent
  virtual void reset1();

private:
  OModelParam<std::string> itsXMLFName;
  rutz::shared_ptr<TestImages> itsObjectsInfo;
  std::vector<std::string> itsObjectsNames;
  uint itsNumObjects;

  uint itsFrame;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

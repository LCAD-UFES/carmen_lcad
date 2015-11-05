/*!@file HippocampusI.H maintains the current thought location of the robot */


//////////////////////////////////////////////////////////////////// //
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
// Primary maintainer for this file: Lior Elazary <lelazary@yahoo.com>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/RobotBrain/HippocampusI.H $
// $Id: HippocampusI.H 12281 2009-12-17 09:00:36Z itti $
//

#ifndef HIPPOCAMPUSI_H_DEFINED
#define HIPPOCAMPUSI_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "GUI/XWinManaged.H"
#include "GUI/ImageDisplayStream.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/IceImageUtils.H"
#include "Ice/SimEventsUtils.H"
#include <IceUtil/Thread.h>

class HippocampusI : public ModelComponent, public RobotBrainObjects::Hippocampus, public IceUtil::Thread
{

public:


  struct Landmark
  {
    std::string name;
    double posX;
    double posY;
    Image<double> cov;

    Landmark(std::string n, double x, double y,
        Image<double>& c) : name(n), posX(x), posY(y), cov(c) {};

    Landmark() : name(""), posX(-1.0), posY(-1.0), cov(Image<double>(2,2,ZEROS)) {};
  };

  struct Particle
  {
    float x;
    float y;
    float theta;
    std::map<std::string, Landmark> landmarksDB;
    float w;
  };


  HippocampusI(OptionManager& mgr,
          const std::string& descrName = "Hippocampus",
          const std::string& tagName = "Hippocampus");
  ~HippocampusI();

  virtual void run();


  ////!Get the current state
  //PosState getCurrentState();


  ////!Update the state from the motion model
  //void updateState(const float xPos, const float yPos, const float ori);


  //Image<PixRGB<byte> > getCurrentMap();

  //!Update the current map
  //void updateMap();

  void evolve(); //Our evolve function

  //!Get a message from another part of the brain
  virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
      const Ice::Current&);

  //!Initalize the object
  void init(Ice::CommunicatorPtr ic, Ice::ObjectAdapterPtr adapter);


  //! Display an image representation of our current belief state
  //! ie. Draw all of our particles, landmarks, etc onto an image and display it
  void displayMap();

  //! Use a roulette wheel to probabilistically duplicate particles with high weights,
  //! and discard those with low weights
  void resampleParticles();

  //! Assign weights to each particle to judge how likely a particle is to
  //! observe the currently observed set of landmarks
  void updateParticleObservation(RobotSimEvents::LandmarksMessagePtr landmarksMsg);

  //! Assign weights to each particle to judge how likely a particle is to
  //! observe the currently observed set of landmarks. use EKF to track the landmarks
  void updateParticleSlamObservation(RobotSimEvents::LandmarksMessagePtr landmarksMsg);


  //! Add a new landmark to the landmark database
  void addLandmark(Landmark lm);

  //! Move each particle according to the motion model, plus some uniform noise
  void updateParticleMotion(RobotSimEvents::MotionMessagePtr newMotion);



private:
  nub::soft_ref<OutputFrameSeries> itsOfs;
  Image<PixRGB<byte> > itsMap;
  RobotSimEvents::EventsPrx itsEventsPub;
  std::vector<SimEventsUtils::TopicInfo> itsTopicsSubscriptions;
  Ice::ObjectPrx itsObjectPrx; //Our Proxy
  RobotSimEvents::StateMessagePtr itsCurrentState;
  RobotSimEvents::GPSMessagePtr itsCurrentGPSPos;
  RobotSimEvents::MotionMessagePtr itsCurrentMotion;
  RobotSimEvents::LandmarksMessagePtr itsCurrentLandmarks;
  RobotSimEvents::StateMessagePtr itsLastLandmarkState;
//  std::vector<Landmark> itsLandmarks;
  std::vector<Particle> itsParticles;

  std::map<std::string, Landmark> itsLandmarkDB;
  std::map<std::string, Landmark> itsGTLandmarkDB;

  //A record of the sum of the euclidean distances between landmarks and
  //their groundtruths at every timestep of the algorithm.
  //-Used in display map
  std::vector<float> itsLmDistanceHistory;
  float itsLmDistanceMax;

  int     itsNumParticles;
  float   itsRangeVariance;
  float   itsBearingVariance;
  int     itsFrame;

  IceUtil::Mutex itsParticlesMutex;


};

#endif

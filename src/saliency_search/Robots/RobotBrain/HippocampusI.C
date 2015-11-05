/*!@file Hippocampus.C maintains the current thought location of the robot */
//This modules invovles in the perception

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/RobotBrain/HippocampusI.C $
// $Id: HippocampusI.C 10794 2009-02-08 06:21:09Z itti $
//

#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "Image/lapack.H"
#include "Image/MatrixOps.H"
#include "Image/MathOps.H"
#include "GUI/ImageDisplayStream.H"
#include "GUI/DebugWin.H"
#include "Robots/RobotBrain/HippocampusI.H"
#include <math.h>


// ######################################################################
HippocampusI::HippocampusI(OptionManager& mgr,
    const std::string& descrName, const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsCurrentState(new RobotSimEvents::StateMessage),
  itsCurrentGPSPos(new RobotSimEvents::GPSMessage),
  itsCurrentMotion(new RobotSimEvents::MotionMessage),
  itsCurrentLandmarks(new RobotSimEvents::LandmarksMessage),
  itsLastLandmarkState(new RobotSimEvents::StateMessage),
  itsLmDistanceMax(0),
  itsNumParticles(300),
  itsRangeVariance(50),
  itsBearingVariance(.5),
  itsFrame(0)
{

  itsOfs = nub::ref<OutputFrameSeries>(new OutputFrameSeries(mgr));
  addSubComponent(itsOfs);

  //Starting position
  itsCurrentState->xPos = 0;
  itsCurrentState->yPos = 0;
  itsCurrentState->orientation = 0;

  itsLastLandmarkState->xPos = itsCurrentState->xPos;
  itsLastLandmarkState->yPos = itsCurrentState->yPos;
  itsLastLandmarkState->orientation = itsCurrentState->orientation;


  itsCurrentGPSPos->xPos = 0;
  itsCurrentGPSPos->yPos = 0;
  itsCurrentGPSPos->orientation = 0;

  itsCurrentMotion->distance = 0;
  itsCurrentMotion->angle = 0;

  itsMap = Image<PixRGB<byte> > (800, 800, ZEROS);

  //Initialize the particles to be at (0,0,0) with equal weight
  itsParticles.resize(itsNumParticles);
  for(int i=0; i<itsNumParticles; i++)
  {
    itsParticles.at(i).x     = itsCurrentState->xPos;
    itsParticles.at(i).y     = itsCurrentState->yPos;
    itsParticles.at(i).theta = itsCurrentState->orientation;
    itsParticles.at(i).w     = 1/itsNumParticles;
  }

  //init the landmarks
  //addLandmark(Landmark(1,  609.6*2, 609.6*7));
  //addLandmark(Landmark(2,  609.6*1, 609.6*5));
  //addLandmark(Landmark(3,  609.6*1, 609.6*3));
  //addLandmark(Landmark(4,  609.6*2, 609.6*2));
  //addLandmark(Landmark(5,  609.6*3, 609.6*4));
  //addLandmark(Landmark(6,  609.6*4.1, 609.6*2.2));
  //addLandmark(Landmark(7,  609.6*5, 609.6*4));
  //addLandmark(Landmark(8,  609.6*6, 609.6*4));
  //addLandmark(Landmark(9,  609.6*8, 609.6*4));
  //addLandmark(Landmark(10, 609.6*8, 609.6*3));
  //addLandmark(Landmark(11, 609.6*8, 609.6*6));
  //addLandmark(Landmark(12, 609.6*6, 609.6*7));
  //addLandmark(Landmark(13, 609.6*4, 609.6*7));

  //addLandmark(Landmark(1,  609.6*2, 609.6*-1));
  //addLandmark(Landmark(2,  609.6*4, 609.6*2));
  //addLandmark(Landmark(3,  609.6*4, 609.6*0));
  //addLandmark(Landmark(4,  609.6*3, 609.6*-1));

  //11_19/Path1
  //addLandmark(Landmark("L1",  609.6*2, 609.6*-0.75));
  //addLandmark(Landmark("L2",  609.6*-1, 609.6*1));
  //addLandmark(Landmark("L3",  609.6*-1, 609.6*4.2));
  //addLandmark(Landmark("L4",  609.6*-0.3, 609.6*-1));
  //addLandmark(Landmark("L5",  609.6*1, 609.6*5.5));
  //addLandmark(Landmark("L6",  609.6*-1.2, 609.6*3));
  //addLandmark(Landmark("L7",  609.6*-1.2, 609.6*2));
  //addLandmark(Landmark("L8",  609.6*1, 609.6*4));
  //addLandmark(Landmark("L9",  609.6*2, 609.6*3));
  //addLandmark(Landmark("L10", 609.6*0, 609.6*4));
  //addLandmark(Landmark("L11", 609.6*4, 609.6*0));
  //addLandmark(Landmark("L12", 609.6*2.2, 609.6*4));
  //addLandmark(Landmark("L14", 609.6*2.9, 609.6*6));
  //addLandmark(Landmark("L13", 609.6*-1, 609.6*0));


  //11_21/Path1
  Image<double> cov(2,2,ZEROS);
  addLandmark(Landmark(string("L1"),  609.6*3,    609.6*-1,   cov));
  addLandmark(Landmark(string("L2"),  609.6*-1,   609.6*4,    cov));
  addLandmark(Landmark(string("L3"),  609.6*4,    609.6*0,    cov));
  addLandmark(Landmark(string("L4"),  609.6*-1.5, 609.6*2,    cov));
  addLandmark(Landmark(string("L5"),  609.6*3.5,  609.6*3,    cov));
  addLandmark(Landmark(string("L6"),  609.6*1,    609.6*4,    cov));
  addLandmark(Landmark(string("L7"),  609.6*3.5,  609.6*1,    cov));
  addLandmark(Landmark(string("L8"),  609.6*1,    609.6*5.5,  cov));
  addLandmark(Landmark(string("L9"),  609.6*-1.5, 609.6*3,    cov));
  addLandmark(Landmark(string("L10"), 609.6*2,    609.6*3,    cov));
  addLandmark(Landmark(string("L11"), 609.6*-0.8, 609.6*5,    cov));
  addLandmark(Landmark(string("L12"), 609.6*-0.5, 609.6*-1,   cov));
  addLandmark(Landmark(string("L13"), 609.6*1,    609.6*-1,   cov));
  addLandmark(Landmark(string("L14"), 609.6*2.5,  609.6*6,    cov));


}

// ######################################################################
HippocampusI::~HippocampusI()
{
  SimEventsUtils::unsubscribeSimEvents(itsTopicsSubscriptions, itsObjectPrx);
}


// ######################################################################
void HippocampusI::init(Ice::CommunicatorPtr ic, Ice::ObjectAdapterPtr adapter)
{

  Ice::ObjectPtr hippPtr = this;
  itsObjectPrx = adapter->add(hippPtr,
      ic->stringToIdentity("Hippocampus"));


  IceStorm::TopicPrx topicPrx;

  itsTopicsSubscriptions.push_back(SimEventsUtils::TopicInfo("GPSMessageTopic", topicPrx));
  itsTopicsSubscriptions.push_back(SimEventsUtils::TopicInfo("MotionMessageTopic", topicPrx));
  itsTopicsSubscriptions.push_back(SimEventsUtils::TopicInfo("LandmarksMessageTopic", topicPrx));

  SimEventsUtils::initSimEvents(ic, itsObjectPrx, itsTopicsSubscriptions);

  itsEventsPub = RobotSimEvents::EventsPrx::uncheckedCast(
      SimEventsUtils::getPublisher(ic, "StateMessageTopic")
      );

  IceUtil::ThreadPtr hippThread = this;
  hippThread->start();

  usleep(10000);
}


// ######################################################################
void HippocampusI::run()
{
  sleep(1);

  while(1)
  {
    evolve();
    usleep(100000);
  }
}

// ######################################################################
void HippocampusI::evolve()
{
  //calculate the new state from the particles (mean)
  float xPos = 0, yPos = 0, orientation = 0;
  for(int i=0; i<itsNumParticles; i++)
  {
    xPos += itsParticles.at(i).x;
    yPos += itsParticles.at(i).y;
    orientation += itsParticles.at(i).theta;
  }
  xPos        /= itsParticles.size();
  yPos        /= itsParticles.size();
  orientation /= itsParticles.size();

  itsCurrentState->xPos = xPos;
  itsCurrentState->yPos = yPos;
  itsCurrentState->orientation = orientation;

  LDEBUG("%f %f %f\n",
      itsCurrentState->xPos,
      itsCurrentState->yPos,
      itsCurrentState->orientation);

  itsEventsPub->updateMessage(itsCurrentState);
}

// ######################################################################
void HippocampusI::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg, const Ice::Current&)
{
  //Get a gps message
  if(eMsg->ice_isA("::RobotSimEvents::GPSMessage"))
  {
    RobotSimEvents::GPSMessagePtr gpsMsg = RobotSimEvents::GPSMessagePtr::dynamicCast(eMsg);
    itsCurrentGPSPos->xPos = gpsMsg->xPos;
    itsCurrentGPSPos->yPos = gpsMsg->yPos;
    itsCurrentGPSPos->orientation = gpsMsg->orientation;
  }

  if(eMsg->ice_isA("::RobotSimEvents::MotionMessage"))
  {
    RobotSimEvents::MotionMessagePtr motionMsg = RobotSimEvents::MotionMessagePtr::dynamicCast(eMsg);
    itsCurrentMotion->distance = motionMsg->distance;
    itsCurrentMotion->angle = motionMsg->angle;

    updateParticleMotion(itsCurrentMotion);
    itsFrame++;

   // printf("%i\t-1\t%f\t%f\n", itsFrame++,
   //     itsCurrentMotion->distance,
   //     itsCurrentMotion->angle);

  }

  if(eMsg->ice_isA("::RobotSimEvents::LandmarksMessage"))
  {

    RobotSimEvents::LandmarksMessagePtr landmarksMsg = RobotSimEvents::LandmarksMessagePtr::dynamicCast(eMsg);
    //updateParticleObservation(landmarksMsg);
    updateParticleSlamObservation(landmarksMsg);

    itsFrame++;
   // for(uint li=0; li<landmarksMsg->landmarks.size(); li++)
   // {

   //   printf("%i\t%s\t%f\t%f\n",itsFrame,
   //       landmarksMsg->landmarks[li].name.c_str(),
   //       landmarksMsg->landmarks[li].range,
   //       landmarksMsg->landmarks[li].bearing);
   // }

   /////// for(uint i=0; i<itsParticles.size(); i++)
   /////// {
   ///////   LINFO("%i %f\n",
   ///////       i, itsParticles[i].w);
   /////// }
    ////Save the current landmarks for latter display
    itsCurrentLandmarks->landmarks = landmarksMsg->landmarks;

    //find the max particle
    int maxP = 0; float maxVal=itsParticles[0].w;
    for(uint i=1; i<itsParticles.size(); i++)
    {
      if (itsParticles[i].w > maxVal)
      {
        maxVal = itsParticles[i].w;
        maxP = i;
      }
    }

    itsLandmarkDB = itsParticles[maxP].landmarksDB;


    displayMap();
    resampleParticles();

  }
  fflush(stdout);
}

// ######################################################################
void HippocampusI::updateParticleMotion(RobotSimEvents::MotionMessagePtr newMotion)
{
  /* Needed
     double time; //time for this newMotion, because each sampling is different
     double commandF; //forward or backward
     double commandS; //steer
     */
  double alpha[4] = {0.002, 0, 0.2, 0};

  // check out values
  //printf("%f %f %f %f %f\n", newMotion->distance, newMotion->angle,
  //    itsParticles.at(0).x, itsParticles.at(0).y, itsParticles.at(0).theta);

  //Move each particle according to the motion model, plus some noise
  if(abs(newMotion->angle)<10 && abs(newMotion->distance)<25) {
    itsParticlesMutex.lock();

    for(int i=0; i<itsNumParticles; i++)
    {
      //odometry data
      float theta = itsParticles.at(i).theta;
      float dx = newMotion->distance*cos(theta);
      float dy = newMotion->distance*sin(theta);

//      itsParticles.at(i).x     += dx;
//      itsParticles.at(i).y     += dy;
//      itsParticles.at(i).theta += newMotion->angle*M_PI/180;
//
      float rot1  = atan2(dy, dx) - theta;
      float trans = sqrt(pow(dx,2)+pow(dy,2));
      float rot2  = (theta+newMotion->angle*M_PI/180) - theta - rot1;

      //noise
      float nr1 = rot1  - randomDoubleFromNormal(alpha[0]*abs(rot1)+alpha[1]*abs(trans));      //rot1
      float ntr = trans - randomDoubleFromNormal(alpha[2]*abs(trans)+alpha[3]*abs(rot1+rot2)); //trans
      float nr2 = rot2  - randomDoubleFromNormal(alpha[0]*abs(rot2)+alpha[1]*abs(trans));      //rot2

      //new particle
      itsParticles.at(i).x     += ntr*cos(theta+nr1);
      itsParticles.at(i).y     += ntr*sin(theta+nr1);
      itsParticles.at(i).theta += nr1 + nr2;

      //Bound the angle to -PI <-> PI
      if(itsParticles.at(i).theta > M_PI)
      {
        itsParticles.at(i).theta -= 2*M_PI;

      }
      if(itsParticles.at(i).theta < M_PI)
      {
        itsParticles.at(i).theta += 2*M_PI;
      }
    }
    itsParticlesMutex.unlock();
  }

}

// ######################################################################
void HippocampusI::updateParticleSlamObservation(RobotSimEvents::LandmarksMessagePtr landmarksMsg)
{
  itsParticlesMutex.lock();
  float sum = 0;
  std::vector<Particle>::iterator particleIt =
    itsParticles.begin();

  for(; particleIt != itsParticles.end(); particleIt++)
  {
    for(uint li=0; li<landmarksMsg->landmarks.size(); li++)
    {

      std::string landmarkName = landmarksMsg->landmarks[li].name;
      float range = landmarksMsg->landmarks[li].range*1000;
      float bearing = landmarksMsg->landmarks[li].bearing;

      //Check to make sure we know about this landmark
      std::map<std::string, Landmark>::iterator landIt =
        particleIt->landmarksDB.find(landmarkName);

      //Landmark currLand = landIt->second;

      Image<double>  sensorNoise(2,2, ZEROS);
      sensorNoise.setVal(0,0, squareOf(500)); //Range noise
      sensorNoise.setVal(1,1, squareOf(5*M_PI/180)); //bearing noise


      //If we don't have the landmark in our database, then let's add it.
      if(landIt == particleIt->landmarksDB.end())
      {
        //initalize landmark mean as u_j = g-1(z,x);

        double landmarkXPos = particleIt->x +  range*cos(bearing + particleIt->theta);
        double landmarkYPos = particleIt->y +  range*sin(bearing + particleIt->theta);

        //initalize covariance
        double dx = landmarkXPos - particleIt->x;
        double dy = landmarkYPos - particleIt->y;
        double d2 = squareOf(dx) + squareOf(dy);
        double d = sqrt(d2);


        Image<double> G(2,2, NO_INIT);
        G.setVal(0,0, dx/d); G.setVal(1,0, dy/d);
        G.setVal(0,1, -dy/d2); G.setVal(1,1, dx/d2);

        Image<double> Ginv = matrixInv(G);


        Image<double> cov = matrixMult(matrixMult(Ginv,sensorNoise),transpose(Ginv));

        //add the landmark to the DB
        Landmark lm(landmarkName, landmarkXPos, landmarkYPos, cov);
        particleIt->landmarksDB[landmarkName] = lm;
        particleIt->w *= 0.00001;
      }
      else {

        //Use the Cholesky factorization for the EKF update

        //we have the landmark calculate the change in landmark position
        //predict the mesurment z=g(u,x)
        double landmarkXPos = landIt->second.posX;
        double landmarkYPos = landIt->second.posY;

        double dx = landmarkXPos - particleIt->x;
        double dy = landmarkYPos - particleIt->y;
        double d2 = squareOf(dx) + squareOf(dy);
        double d = sqrt(d2);

        double beliefRange = d;
        double beliefBearing = atan2(dy, dx) - particleIt->theta;

        //Limit the angle between -pi to pi
        if (beliefBearing < M_PI)
        {
          beliefBearing += 2*M_PI;
        }
        if (beliefBearing > M_PI)
        {
          beliefBearing -= 2*M_PI;
        }


        Image<double> G(2,2, NO_INIT);
        G.setVal(0,0, dx/d); G.setVal(1,0, dy/d);
        G.setVal(0,1, -dy/d2); G.setVal(1,1, dx/d2);

        Image<double> innov(1,2,NO_INIT);
        innov.setVal(0,0, range - beliefRange);
        innov.setVal(0,1, bearing - beliefBearing);


        Image<double> PHt = matrixMult(landIt->second.cov,transpose(G));
        Image<double> S = matrixMult(G, PHt) + sensorNoise;

        //Make symmetric
        S = S+transpose(S);
        S = S *0.5;

        Image<double> SChol = lapack::dpotrf(&S);
        SChol.setVal(0,1,0); //Set the bottom left to 0 ?

        Image<double> SCholInv = matrixInv(SChol);


        Image<double> W1 = matrixMult(PHt, SCholInv);
        Image<double> K = matrixMult(W1, transpose(SCholInv));


        Image<double> dLandmarkPos = matrixMult(K,innov);
        landIt->second.posX += dLandmarkPos.getVal(0,0);
        landIt->second.posY += dLandmarkPos.getVal(0,1);

        landIt->second.cov -= matrixMult(W1, transpose(W1));


        Image<double> Q = matrixMult(matrixMult(G, landIt->second.cov),transpose(G)) + sensorNoise;

        Image<double> diffSq = matrixMult(matrixMult(transpose(innov), matrixInv(Q)), innov);

        particleIt->w *= exp(-0.5*diffSq.getVal(0,0)) + 0.001;

      }
    }

    sum += particleIt->w;

  }

  //normalize
  for(uint i=0; i<itsParticles.size(); i++)
    itsParticles.at(i).w /= sum;

  itsParticlesMutex.unlock();
}

// ######################################################################
void HippocampusI::updateParticleObservation(RobotSimEvents::LandmarksMessagePtr landmarksMsg)
{
  itsParticlesMutex.lock();
  float sum = 0;
  for(int particle=0; particle<itsNumParticles; particle++)
  {
    for(uint li=0; li<landmarksMsg->landmarks.size(); li++)
    {
      //Check to make sure we know about this landmark
      std::map<std::string, Landmark>::iterator landIt = itsLandmarkDB.find(landmarksMsg->landmarks.at(li).name);

      //If we don't have the landmark in our database, then let's add it.
      if(landIt == itsLandmarkDB.end())
      {
        //addLandmark(landmarksMsg->landmarks.at(li));
        continue;
      }

      Landmark currLand = landIt->second;

      float bearing =  landmarksMsg->landmarks[li].bearing;
      float range   =  landmarksMsg->landmarks[li].range*1000;

      float beliefRange    = sqrt(pow(currLand.posY - itsParticles.at(particle).y, 2) +
                                  pow(currLand.posX - itsParticles.at(particle).x, 2));
      float beliefBearing  = atan2(currLand.posY - itsParticles.at(particle).y,
                                   currLand.posX - itsParticles.at(particle).x)
                             - itsParticles.at(particle).theta;

      if (beliefBearing < M_PI)
        beliefBearing += 2*M_PI;
      if (beliefBearing > M_PI)
        beliefBearing -= 2*M_PI;

      float innovRange   = range   - beliefRange;
      float innovBearing = bearing - beliefBearing;


      //TODO:Make this a 2D gaussian
      float prob = exp(-0.5*pow(innovRange,   2) / itsRangeVariance  ) + .0001;
      prob =   exp(-0.5*pow(innovBearing, 2) / itsBearingVariance) + .0001;

      itsParticles.at(particle).w = prob;
      //LINFO("%i %f\n", particle, prob);
    }
    sum += itsParticles.at(particle).w;
  }

  //normalize
  for(uint i=0; i<itsParticles.size(); i++)
    itsParticles.at(i).w /= sum;

  itsParticlesMutex.unlock();
}

// ######################################################################
void HippocampusI::addLandmark(Landmark lm)
{
  //Initalize the landmark position with a fixed range
  LINFO("Adding Landmark %s pos", lm.name.c_str());
  itsGTLandmarkDB[lm.name] = lm;
}



// ######################################################################
void HippocampusI::resampleParticles()
{
  //Use a roulette wheel to probabilistically duplicate particles with high weights,
  //and discard those with low weights
  itsParticlesMutex.lock();

  std::vector<Particle> newParticles;

  //Calculate a Cumulative Distribution Function for our particle weights
  std::vector<float> CDF;
  CDF.resize(itsParticles.size());

  CDF.at(0) = itsParticles.at(0).w;
  for(uint i=1; i<CDF.size(); i++)
    CDF.at(i) = CDF.at(i-1) + itsParticles.at(i).w;

  uint i = 0;
  float u = randomDouble()* 1.0/float(itsParticles.size());

  for(uint j=0; j < itsParticles.size(); j++)
  {
    while(u > CDF.at(i))
      i++;

    Particle p = itsParticles.at(i);
    p.w     = 1.0/float(itsParticles.size());

    newParticles.push_back(p);

    u += 1.0/float(itsParticles.size());
  }

  itsParticles = newParticles;

  itsParticlesMutex.unlock();
}

// ######################################################################
void HippocampusI::displayMap()
{
  int circRadius = 10;
  Image<PixRGB<byte> > tmpMap = itsMap;


 // //Graph the average distance between known landmarks, and their ground truthed positions
 // if(itsGTLandmarkDB.size() > 0) {
 //   std::map<std::string, Landmark>::iterator lIt = itsLandmarkDB.begin();
 //   float lmDistance = 0.0;
 //   for(;lIt != itsLandmarkDB.end(); lIt++)
 //   {
 //     //Make sure we have a ground truth of the found landmark
 //     std::map<std::string, Landmark>::iterator matchIt = itsGTLandmarkDB.find(lIt->second.name);
 //     if(matchIt != itsLandmarkDB.end())
 //     {
 //       lmDistance += sqrt(
 //           squareOf(matchIt->second.posX - lIt->second.posX) +
 //           squareOf(matchIt->second.posY - lIt->second.posY)
 //           );
 //     }
 //   }
 //   if(lmDistance > itsLmDistanceMax)
 //     itsLmDistanceMax = lmDistance;

 //   itsLmDistanceHistory.push_back(lmDistance);
 //
 //   Image<PixRGB<byte> > distanceGraph = linePlot(itsLmDistanceHistory, tmpMap.getWidth(), 150, 0, itsLmDistanceMax, "t");
 //   itsOfs->writeRGB(distanceGraph, "DistanceGraph", FrameInfo("DistanceGraph", SRC_POS));
 // }

  Image<float> probMap(itsMap.getDims(), ZEROS);

  //Draw the State on the map;
  float scale = 1.0/7.0;
  Point2D<int> offset((int)(609.6*3.0*scale), (int)(609.6*2.0*scale));

  Point2D<int> p1 = Point2D<int>(scale*609.6*0   , scale*609.6*0) + offset;
  Point2D<int> p2 = Point2D<int>(scale*609.6*3   , scale*609.6*0) + offset;
  Point2D<int> p3 = Point2D<int>(scale*609.6*3   , scale*609.6*5) + offset;
  Point2D<int> p4 = Point2D<int>(scale*609.6*-0.5, scale*609.6*5) + offset;
  Point2D<int> p5 = Point2D<int>(scale*609.6*-0.5, scale*609.6*0) + offset;

  drawLine(tmpMap, p1, p2, PixRGB<byte>(50,50,50));
  drawLine(tmpMap, p2, p3, PixRGB<byte>(50,50,50));
  drawLine(tmpMap, p3, p4, PixRGB<byte>(50,50,50));
  drawLine(tmpMap, p4, p5, PixRGB<byte>(50,50,50));
  drawLine(tmpMap, p5, p1, PixRGB<byte>(50,50,50));

  //Draw the particles on the map
  for(uint particle=0; particle<itsParticles.size(); particle++) {
    Point2D<int> particlePos( itsParticles.at(particle).x*scale,
        itsParticles.at(particle).y*scale);
    float prob = itsParticles.at(particle).w;
    if (probMap.coordsOk(offset + particlePos))
        probMap.setVal(offset + particlePos, prob);
  }

  //Draw the ground truth landmarks on the map
  std::map<std::string, Landmark>::iterator landIt = itsGTLandmarkDB.begin();
  for(;landIt != itsGTLandmarkDB.end(); ++landIt)
  {
    Point2D<int> lmPos(
        (*landIt).second.posX * scale,
        (*landIt).second.posY * scale
        );
    drawCross(tmpMap, lmPos + offset, PixRGB<byte>(0,100, 0));

  }


  //Draw the leading particle's landmarks on the map
  landIt = itsLandmarkDB.begin();
  for(;landIt != itsLandmarkDB.end(); ++landIt)
  {
    Point2D<int> lmPos(
        (*landIt).second.posX * scale,
        (*landIt).second.posY * scale
        );

    if(tmpMap.coordsOk(offset+lmPos) &&
        tmpMap.coordsOk(offset+lmPos+Point2D<int>(landIt->second.name.size()*10, circRadius*2.5)))
    {
      Image<PixRGB<byte> > labelImage(landIt->second.name.size()*10, circRadius*2.5, ZEROS);

      drawCircle(labelImage, Point2D<int>(circRadius,circRadius), circRadius, PixRGB<byte>(0,0,150),0);

      writeText(
          labelImage,
          Point2D<int>(circRadius/2,circRadius/2),
          landIt->second.name.c_str(),
          PixRGB<byte>(75,75,255),
          PixRGB<byte>(0,0,0),
          SimpleFont::FIXED(6),
          true
          );

      labelImage = flipVertic(labelImage);
      inplacePaste(tmpMap, labelImage, offset+lmPos-Point2D<int>(circRadius, circRadius));
    }
  }

  //draw the lines to the landmark
  for(uint li=0; li<itsCurrentLandmarks->landmarks.size(); li++)
  {
    float bearing = itsCurrentLandmarks->landmarks[li].bearing;
    float range = (itsCurrentLandmarks->landmarks[li].range*1000);

    float posX = itsCurrentState->xPos + (range*cos(itsCurrentState->orientation + bearing));
    float posY = itsCurrentState->yPos + (range*sin(itsCurrentState->orientation + bearing));

    drawLine(tmpMap,
        offset + Point2D<int>((int)(itsCurrentState->xPos*scale),
          ((int)(itsCurrentState->yPos*scale))),
        offset + Point2D<int>((int)(posX*scale), ((int)(posY*scale))),
        PixRGB<byte>(0,0,255));
  }

  Point2D<int> posOnMap((int)(itsCurrentState->xPos*scale),
      (int)( itsCurrentState->yPos*scale));

  //Draw the robot
  if (itsMap.coordsOk(offset + posOnMap))
  {
    itsMap.setVal(offset + posOnMap, PixRGB<byte>(255,0,0));
    drawDisk(tmpMap, offset + posOnMap, 10, PixRGB<byte>(100,200,100));

    Point2D<int> robotAng(int(cos(itsCurrentState->orientation)*9),
                          int(sin(itsCurrentState->orientation)*9));

    drawLine(tmpMap, offset + posOnMap, offset + posOnMap + robotAng, PixRGB<byte>(0,0,0), 2);
  }

  inplaceNormalize(probMap, 0.0F, 255.0F);

  //Print out the frame number and the believed robot position
  printf("%d %d %d ", itsFrame, posOnMap.i, posOnMap.j);
  std::map<std::string, Landmark>::iterator gtIt = itsGTLandmarkDB.begin();
  for(;gtIt != itsGTLandmarkDB.end(); gtIt++) {
    //Print out the name of this landmark
    printf("%s ", gtIt->second.name.c_str());

    //See if our current landmark DB knows about this ground truth
    std::map<std::string, Landmark>::iterator lIt = itsLandmarkDB.find(gtIt->second.name);

    if(lIt == itsLandmarkDB.end())
    {
      //If it does not, then output zeros for the position
      printf("0 0 ");
    }
    else
    {
      //If it does, then print out the believed position of this landmark
      printf("%f %f ", lIt->second.posX, lIt->second.posY);
    }
  }
  printf("\n");


  itsOfs->writeRGB(flipVertic(tmpMap), "Map", FrameInfo("Map", SRC_POS));
  itsOfs->writeRGB(flipVertic(toRGB(probMap)), "ProbMap", FrameInfo("Map", SRC_POS));
}


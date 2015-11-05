/**
* \file Localization.H
* \brief Contains declaration for Localization class
*
*        Notes : Things to be done
*                ->Make orientation relative to the reading taken at start instead of absolute
*                        just introduce a message at the start time
*                ->
*
*
*/

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Media/FrameSeries.H"

#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "GUI/XWinManaged.H"
#include "GUI/ImageDisplayStream.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Util/Types.H"      // for byte
#include <string>
#include <vector>
#include <math.h>


#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/IceImageUtils.H"
#include <IceUtil/Thread.h>

//////BOOST Libraries


#include <boost/random/linear_congruential.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

////////////

#ifndef LOCALIZATION_H
#define LOCALIZATION_H

/// \def BOT_SPEED 1
/// \brief Some defintions for parameters that we are going to use in equations for predictions and updates using sensor data
#define BOT_SPEED 1 //(pixels per 1 second)
#define INIT_PARTICLES 1000  //(Kick off with 1000 particles)
#define BETA 0.4 // max allowed depletion before resampling


/// \def NORMAL_DIST(MEAN,SIGMA,X)
/// \brief Equation of normal distribution
#define NORMAL_DIST(MEAN,SIGMA,X) ((1.0/(SIGMA*sqrt(2*M_PI)))*pow(M_E,(-1.0*(X-MEAN)*(X-MEAN))/(2.0*SIGMA*SIGMA)))
// Note: Fine tune these parameters to improve prediction performance
// For best results use this constants from experiments on sub

//Constants for forward movement

//Constants for rotation
#define M_DRIFT 0.35  // the mean rotation error per delta(theta) = 1
#define SIGMA_DRIFT 5 // standard deviation on 360 degree rotation
//
#define M_TRANS 0.35  // the mean translation per delta(trans) = 1
#define SIGMA_TRANS 5 // standard deviation on 1 unit translation
//

#include "pParticle.H"

/**
* \class Localization Localization.H "src/Robots/SeaBeeIII/Localization.H"
* \brief contains functions for tracking the position of the sub using particle filter technic
*
*/

class Localization : public RobotBrainComponent
{
private:
        /// \brief pointer to OutputFrameSeries class
        nub::soft_ref<OutputFrameSeries>          itsOfs;
        /// \brief whether last time sub was running or not?
        bool itsRunning;

        /// \brief the image where positions are written out
        Image<float> iMap;
        /// \brief will contain the obstacles on the map
        Image<float> originalMap;

        /// \brief The list of particles
        vector<class pParticle> pList;
        /// \brief number of particles in the image right now
        int itsNumParticles;
        /// \brief minimum number of particles to be kept in image
        int itsKeepParticles;

        /// \brief current estimate of the method
        class pParticle maxPoint;

        /// \brief Memory for the message data that we have received
        /// Timestamp for estimation of the speed
        time_t timestamp;

        /// \private int curOrientation
        /// \brief The last orientation that we received
        int curOrientation;

        /// \typedef typedef boost::minstd_rand  base_gen_type
        /// \brief BOOST library stuff
        typedef boost::minstd_rand  base_gen_type;
        typedef boost::normal_distribution<>  dist_type;
        typedef boost::variate_generator<base_gen_type&, dist_type>  rgen_type;

        base_gen_type generator;
        rgen_type generate_drift;
        rgen_type generate_trans;


public:

  Localization(OptionManager& mgr,
      const std::string& descrName = "Localization",
      const std::string& tagName = "Localization");

  ~Localization();

  virtual void evolve();

  //!Get a message
  virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
      const Ice::Current&);

  virtual void registerTopics();

  //

        void initPoints(const short int);
        float getNormalizer();
        float rand_N();
  //
  // Some of my functions

        void predict(RobotSimEvents::OrientationMessagePtr);

        void update(RobotSimEvents::ObstacleMessagePtr);

        void resample(int no =0);

        class pParticle& getCurrentEstimate();
  //
  // Debugging it

};


#endif

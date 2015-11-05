#include "Robots/SeaBeeIII/MapperI.H"

const ModelOptionCateg MOC_MapperI =
 { MOC_SORTPRI_3, "SeaBee Mapper Component Related Options" };

const ModelOptionDef OPT_NumParticles =
{ MODOPT_ARG(int), "NumParticles", &MOC_MapperI, OPTEXP_CORE,
  "Number of particles to keep track of for the particle filter.\n"
    "  Using more particles can make the position estimate more accurate\n"
    "  but will consume more cpu resources",
  "num-particles", '\0', "<int>", "500"};

const ModelOptionDef OPT_HeadingVariance =
{ MODOPT_ARG(float), "HeadingVariance", &MOC_MapperI, OPTEXP_CORE,
  "The amount of variance (in degrees) in the heading. Using a large number here will"
    "  spread out the particles laterally as movement occurs.",
  "heading-var", '\0', "<float>", "3"};

const ModelOptionDef OPT_SpeedVariance =
{ MODOPT_ARG(float), "SpeedVariance", &MOC_MapperI, OPTEXP_CORE,
  "The amount of variance in the speed (in feet/second). Using a large number here will"
    "  spread out the particles in the direction of movement as movement occurs.",
  "speed-var", '\0', "<float>", "3"};

// ######################################################################
MapperI::MapperI(int id, OptionManager& mgr,
    const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsNumParticles(&OPT_NumParticles, this, 0),
  itsHeadingVariance(&OPT_HeadingVariance, this, 0),
  itsSpeedVariance(&OPT_SpeedVariance, this, 0)
{
  //Fill the particle buffer with particles - each having
  //equal probability
  Particle initialParticle;
  initialParticle.pos = Point2D<float>(0,0);
  initialParticle.P = 1.0/itsNumParticles.getVal();
  itsParticles.resize(itsNumParticles.getVal(), initialParticle);
}

// ######################################################################
MapperI::~MapperI()
{

}

void MapperI::registerTopics()
{
  LINFO("Registering Mapper Message");
  this->registerPublisher("MapperMessageTopic");
  this->registerSubscription("BeeStemMessageTopic");
  this->registerSubscription("VisionObjectMessageTopic");
}

void MapperI::evolve()
{

}

// ######################################################################
void MapperI::moveParticles(float speed, float heading, uint64 elapsedTicks)
{
  //Grab a lock on the particles buffer
  IceUtil::Mutex::Lock lock(itsParticlesMutex);

  double elapsedSecs = double(elapsedTicks) / double(itsSampleTimer.ticksPerSec());

  //Move each particle according to the reported speed/heading, but shuffled
  //a bit by some gaussian noise
  vector<Particle>::iterator partIt;
  for(partIt = itsParticles.begin(); partIt != itsParticles.end(); partIt++)
  {

    //Consider this particle's heading and speed as drawn from a Gaussian
    //distribution with a mean around the reported heading and speed, but
    //with a variance specified by the user.
    float randHeading = heading + randomDoubleFromNormal(itsHeadingVariance.getVal());
    float randSpeed   = speed   + randomDoubleFromNormal(itsSpeedVariance.getVal());

    //Calculate the change in X and Y for this particle due to the new heading and speed
    float dx = randSpeed * elapsedSecs * cos(randHeading);
    float dy = randSpeed * elapsedSecs * sin(randHeading);

    //Add the change in position to the particles currently believed position
    partIt->pos += Point2D<float>(dx,dy);
  }
}

Point2D<float> MapperI::resolvePosition()
{
  Point2D<float> resolvedPos(0,0);

  //Grab a lock on the particles buffer
  IceUtil::Mutex::Lock lock(itsParticlesMutex);

  float totalProb = 0.0;

  //Weighted average of all particles' coordinates
  vector<Particle>::iterator partIt;
  for(partIt = itsParticles.begin(); partIt != itsParticles.end(); partIt++)
  {
    resolvedPos.i += partIt->pos.i * partIt->P;
    resolvedPos.j += partIt->pos.j * partIt->P;
    totalProb += partIt->P;
  }

  return resolvedPos / totalProb;

}

// ######################################################################
void MapperI::updateObservation(RobotSimEvents::SeaBeeObjectType obsObjType,
                       bool forwardCamera,
                       ImageIceMod::Point3DIce obsObjectPosition,
                       ImageIceMod::Point3DIce obsObjectVariance)
{
  //Grab a lock on the particles buffer
  IceUtil::Mutex::Lock lock(itsParticlesMutex);

  vector<Particle>::iterator partIt;
  for(partIt = itsParticles.begin(); partIt != itsParticles.end(); partIt++)
  {

    //Find the closest object of this type in our map
    MapObject matchedObject;
    float minDistance = -1;
    vector<MapObject>::iterator objIt;
    for(objIt = itsMap.begin(); objIt != itsMap.end(); objIt++)
      if(objIt->type == obsObjType)
      {
        float distance = objIt->pos.distance(partIt->pos);
        if(distance < minDistance || minDistance == -1)
          matchedObject = *objIt;
      }




  }
}

// ######################################################################
void MapperI::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{
  //Get a retina message
  if(eMsg->ice_isA("::RobotSimEvents::BeeStemMessage"))
  {
    RobotSimEvents::BeeStemMessagePtr ccMsg = RobotSimEvents::BeeStemMessagePtr::dynamicCast(eMsg);
    float heading = ccMsg->compassHeading;
    float speed = ccMsg->desiredSpeed;

    moveParticles(heading, speed, itsSampleTimer.getReset());
  }
  if(eMsg->ice_isA("::RobotSimEvents::VisionObjectMessage"))
  {
    RobotSimEvents::VisionObjectMessagePtr voMsg = RobotSimEvents::VisionObjectMessagePtr::dynamicCast(eMsg);

    RobotSimEvents::SeaBeeObjectType objType = voMsg->objectType;
    bool forwardCamera                       = voMsg->forwardCamera;
    ImageIceMod::Point3DIce objectPosition   = voMsg->objectPosition;
    ImageIceMod::Point3DIce objectVariance   = voMsg->objectVariance;

    updateObservation(objType, forwardCamera, objectPosition, objectVariance);
  }

}



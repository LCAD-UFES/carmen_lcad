#include "Component/ModelOptionDef.H"
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
#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Util/Timer.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/IceImageUtils.H"
#include <IceUtil/Thread.h>


class MapperI : public RobotBrainComponent
{
public:

  MapperI(int id, OptionManager& mgr,
      const std::string& descrName = "Mapper",
      const std::string& tagName   = "Mapper");

  ~MapperI();

  virtual void evolve();

  //!Get a message
  virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
      const Ice::Current&);

  virtual void registerTopics();

  //When a new heading/speed reading comes in, this function
  //will update the positions of all particles accordingly
  void moveParticles(float speed, float heading, uint64 elapsedTicks);

  //Inspect the particle vector and decide on a single position point
  //For now, we just average all particles together
  Point2D<float> resolvePosition();

  void updateObservation(RobotSimEvents::SeaBeeObjectType obsObjType,
                         bool forwardCamera,
                         ImageIceMod::Point3DIce obsObjectPosition,
                         ImageIceMod::Point3DIce obsObjectVariance);
typedef struct
  {
    Point2D<float>   pos;
    Point2D<float> variance;
    RobotSimEvents::SeaBeeObjectType type;

  } MapObject;

private:

  //Simple structure to keep track of the position estimates
  typedef struct
  {
    Point2D<float> pos;                     //Position of a particle

    double P;                               //Probability of the particle
  } Particle;

  std::vector<Particle> itsParticles;       //A vector of coordinates with associated probabilities indicating
                                            //a sparse probability distribution of our current location

  std::vector<MapObject> itsMap;             //A vector representing our knowledge of the map


  IceUtil::Mutex         itsParticlesMutex;  //A lock to make sure that only one thread accesses the particle
                                             //vector at a time

  Timer                   itsSampleTimer;    //A timer to keep track of when our last measurement was recieved -
                                             //Useful for calculating distances from speeds

  OModelParam<int>   itsNumParticles;
  OModelParam<float> itsHeadingVariance;
  OModelParam<float> itsSpeedVariance;
};




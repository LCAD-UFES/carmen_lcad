#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Robots/RobotBrain/RobotBrainComponent.H"

#include "Ice/IceImageUtils.H"
#include "Ice/BeobotEvents.ice.H"

#ifndef LANDMARKDBWORKERI_H
#define LANDMARKDBWORKERI_H

class LandmarkDBWorkerI : public RobotBrainComponent
{
public:

  LandmarkDBWorkerI(OptionManager& mgr,
      const std::string& descrName = "LandmarkDBWorker",
      const std::string& tagName = "LandmarkDBWorker");

  ~LandmarkDBWorkerI();

  virtual void evolve();

  //!Get a message
  virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
      const Ice::Current&);

  virtual void registerTopics();

private:
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

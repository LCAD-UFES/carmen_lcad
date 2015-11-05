

#include "Component/ModelManager.H"
#include "Component/OptionManager.H"

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

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/IceImageUtils.H"
#include <IceUtil/Thread.h>

#ifndef SEABEEINJECTOR_H
#define SEABEEINJECTOR_H

class SeaBeeInjector : public RobotBrainComponent
{
public:

  SeaBeeInjector(int id, OptionManager& mgr,
      const std::string& descrName = "MovementController",
      const std::string& tagName = "MovementController");

  ~SeaBeeInjector();

  virtual void evolve();

  //!Get a message
  virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
      const Ice::Current&);

  virtual void registerTopics();
        void send_message(RobotSimEvents::SeaBeeStateConditionMessagePtr msg, int s, int path, int a, int b, int c, int d, int e, int f, int g, int h, int i);

private:

  unsigned int itsCurrentState;
};

#endif

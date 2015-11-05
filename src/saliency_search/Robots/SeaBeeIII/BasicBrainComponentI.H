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

#ifndef BASICBRAINCOMPONENTI_H
#define BASICBRAINCOMPONENTI_H

class BasicBrainComponentI : public RobotBrainComponent
{
public:

  //!Constructor
  BasicBrainComponentI(int id, OptionManager& mgr,
      const std::string& descrName = "BasicBrainComponent",
      const std::string& tagName = "BasicBrainComponent");

  //!Destructor
  ~BasicBrainComponentI();

  //!Main run loop
  virtual void evolve();

  //!Get a message
  virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
      const Ice::Current&);

  //!Component registers itself as publisher/subscriber to topic(s)
  virtual void registerTopics();

};

#endif

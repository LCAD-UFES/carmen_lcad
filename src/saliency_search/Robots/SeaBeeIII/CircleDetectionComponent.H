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
#include "Robots/SeaBeeIII/VisionBrainComponentI.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/IceImageUtils.H"
#include <IceUtil/Thread.h>

#ifndef CIRCLEDETECTIONCOMPONENT_H
#define CIRCLEDETECTIONCOMPONENT_H


class CircleDetectionComponent : public VisionBrainComponentI
{
public:

  CircleDetectionComponent(int id, OptionManager& mgr,
      const std::string& descrName = "CircleDetectionComponent",
      const std::string& tagName = "CircleDetectionComponent");

  ~CircleDetectionComponent();

  virtual void updateFrame(Image<PixRGB<byte> > img, bool isFwdCamera);

  virtual void registerTopics();

private:


};

#endif

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
#include <IceUtil/Thread.h>

#ifndef BASICVISIONBRAINCOMPONENTI_H
#define BASICVISIONBRAINCOMPONENTI_H


class BasicVisionBrainComponentI : public VisionBrainComponentI
{
public:

  BasicVisionBrainComponentI(int id, OptionManager& mgr,
      const std::string& descrName = "BasicVisionBrainComponent",
      const std::string& tagName = "BasicVisionBrainComponent");

  ~BasicVisionBrainComponentI();

  virtual void updateFrame(Image<PixRGB<byte> > img, std::string cameraId);

  virtual void registerTopics();

private:


};

#endif

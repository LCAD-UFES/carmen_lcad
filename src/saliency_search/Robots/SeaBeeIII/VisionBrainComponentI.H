
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Component/OptionManager.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "GUI/XWinManaged.H"
#include "GUI/ImageDisplayStream.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "Image/Pixels.H"
#include "Robots/RobotBrain/RobotBrainComponent.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/IceImageUtils.H"
#include "Ice/RobotSimEvents.ice.H"
#include <IceUtil/Thread.h>

#ifndef VISIONBRAINCOMPONENTI_H
#define VISIONBRAINCOMPONENTI_H

class VisionBrainComponentI : public RobotBrainComponent
{
public:

  VisionBrainComponentI(OptionManager& mgr,
      const std::string& descrName = "VisionBrainComponent",
      const std::string& tagName = "VisionBrainComponent");

  ~VisionBrainComponentI();

  virtual void evolve();

  //!Get a message
  virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
      const Ice::Current&);

  virtual void registerVisionTopics();

  virtual void registerVisionPublisher(const std::string& MessageTopic);

  virtual bool publishVisionMsg(const::std::string& MessageTopic, RobotSimEvents::EventMessagePtr msg);

        //Guys, let's just decide on a single updateFrame, and get rid of this one... --Rand
  //virtual void updateFrame(Image<PixRGB<byte> > img, bool isFwdCamera) {};

  virtual void updateFrame(Image<PixRGB<byte> > img, std::string cameraID) = 0;

  std::string getCameraSourceName() { return itsCameraSource.getVal();};

protected:
  nub::soft_ref<OutputFrameSeries> itsOfs;
  int itsFrameCount;

private:

  OModelParam<std::string> itsCameraSource;

  //nub::soft_ref<InputFrameSeries> itsIfs;
  Image<PixRGB<byte> > itsCurrentImg;
  bool itsCurrentImgFwdCam;
  std::string itsCurrentCameraID;

  IceUtil::Mutex         itsImgMutex;

  int lastFrameCount;
};

#endif

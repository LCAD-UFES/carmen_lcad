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
#include <vector>

#ifndef STEREO_VISION_H
#define STEREO_VISION_H

class StereoVision: public RobotBrainComponent {

public:

  StereoVision(int id, OptionManager& mgr,
               const std::string& descrName = "StereoVision",
               const std::string& tagName = "StereoVision");
  ~StereoVision();

  virtual void evolve();
  virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
                             const Ice::Current&);
  virtual void registerTopics();

private:

  nub::ref<OutputFrameSeries> itsOfs;
  nub::ref<OutputFrameSeries> itsOfs2;
  nub::ref<OutputFrameSeries> itsOfs3;
  nub::ref<OutputFrameSeries> itsOfs4;
  int itsFrameCount;
  int lastFrameCount;
  Image<PixRGB<byte> > LFwdImage;
  Image<PixRGB<byte> > RFwdImage;
  Image<PixRGB<byte> > LDownImage;
  Image<PixRGB<byte> > RDownImage;
  IceUtil::Mutex itsImgMutex;
};

#endif


#include "StereoVisionI.H"
#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"
#include "Raster/DeBayer.H"
#include "SIFT/VisualObject.H"
#include "Image/ShapeOps.H"

#ifndef STEREO_VISION_C
#define STEREO_VISION_C

StereoVision::StereoVision(int id, OptionManager& mgr,
                           const std::string& descrName,
                           const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsOfs(new OutputFrameSeries(mgr)),
  itsOfs2(new OutputFrameSeries(mgr)),
  itsOfs3(new OutputFrameSeries(mgr)),
  itsOfs4(new OutputFrameSeries(mgr)),
  itsFrameCount(0),
  lastFrameCount(0)
{
  addSubComponent(itsOfs);
  addSubComponent(itsOfs2);
  addSubComponent(itsOfs3);
  addSubComponent(itsOfs4);
}

StereoVision::~StereoVision() {
  //delete camera;
}

void StereoVision::evolve() {
  itsImgMutex.lock();
  if (itsFrameCount != lastFrameCount) {
    lastFrameCount = itsFrameCount;
    if (LFwdImage.initialized()) itsOfs->writeRGB(LFwdImage, "LeftFwdCamera");
    if (RFwdImage.initialized()) itsOfs2->writeRGB(RFwdImage, "RightFwdCamera");
    if (LDownImage.initialized())  itsOfs3->writeRGB(LDownImage, "LeftDownCamera");
    if (RDownImage.initialized()) itsOfs4->writeRGB(RDownImage, "RightDownCamera");
  }
  itsImgMutex.unlock();
}

void StereoVision::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
                                 const Ice::Current&) {

  if (eMsg->ice_isA("::RobotSimEvents::RetinaMessage")) {
    RobotSimEvents::RetinaMessagePtr msg = RobotSimEvents::RetinaMessagePtr::dynamicCast(eMsg);
    itsImgMutex.lock();
    Image<PixRGB<byte> > image = Ice2Image<PixRGB<byte> >(msg->img);
    if (msg->cameraID == "LFwdCamera")        LFwdImage = image;
    else if (msg->cameraID == "RFwdCamera")   RFwdImage = image;
    else if (msg->cameraID == "LDownCamera")  LDownImage = image;
    else if (msg->cameraID == "RDownCamera")  RDownImage = image;
    itsFrameCount++;
    itsImgMutex.unlock();
  }
}

void StereoVision::registerTopics() {
  LINFO("Registering StereoVision Message");
  this->registerSubscription("RetinaMessageTopic"); 
}

#endif


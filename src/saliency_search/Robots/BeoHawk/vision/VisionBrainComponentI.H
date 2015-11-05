/*
 * Commandline option for the FPS limiting: process-fps
 *
 * So this is the old VisionBrainComponentI with one small twist:
 * We run one timer for each input stream or cameraID that we monitor/receive.
 * Additionally, there is an ModelParam for a desired FPS delivered
 * to the updateFrame method.  The message received immediately after
 * the timer counts down will be delivered and the appropriate timer
 * reset. For FPS -1 (the default) every frame that is received
 * should go be sent on, 0 means that we should send none on, and
 * a positive integer means that we should send that number per second
 * from that camera or as many as possible less than that rate.
 *
 * WHY: Well, we can turn off/rate limit intensive processing to free
 * up the CPU to other tasks so that we don't have a bunch of processing
 * that each slows down the others and results in poor overall performance.
 *
 * One additional change that might happen in the future is that this
 * class listens for a FPSChangeMessage or something from the GUI
 * or another class to tell it to change its FPS setting.  This would
 * be extraordinarily useful if say we only want to perform some
 * image processing technique at a certain stage in the competition or
 * if we only want to process slowly to save resources most of the time,
 * but when something interesting happens elsewhere we should processes
 * faster to analyze the event.  This will save on system resources and
 * allow us to implement more intensive algorithms without overburdening
 * the system.
 *
 * In the future this class will replace the old VBCI class, but
 * (1) I want to thoroughly test out the new changes before checking
 * in over the old one and potentially breaking the trunk and (2) I
 * do not think that this class belongs in the Robots/SeaBeeIII dir
 * anymore, maybe moving it to the Robots/RobotBrain dir would be
 * more appropriate.
 *
 *        --Nick
 */


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
#include "Util/Timer.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/IceImageUtils.H"
#include <IceUtil/Thread.h>

#ifndef VISIONBRAINCOMPONENTI_H
#define VISIONBRAINCOMPONENTI_H

//in milliseconds
#define ZERO_RATE_WAIT_TIME                500

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

  virtual void updateFrame(Image<PixRGB<byte> > img, bool isFwdCamera) {};

  virtual void updateFrame(Image<PixRGB<byte> > img, string cameraID) {};

  std::string getCameraSourceName() { return itsCameraSource.getVal();};

protected:
  int itsFrameCount;
  OModelParam<int> desiredFramerate;

private:

  OModelParam<std::string> itsCameraSource;
  Image<PixRGB<byte> > itsCurrentImg;
  bool itsCurrentImgFwdCam;
  string itsCurrentCameraID;

  Timer timer;

  IceUtil::Mutex         itsImgMutex;

  int lastFrameCount;
};

#endif

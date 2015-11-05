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
#include "Raster/PnmWriter.H"
#include <IceUtil/Thread.h>
#include <fstream>

#ifndef LOGGERMODULE_H
#define LOGGERMODULE_H


class LoggerModule : public RobotBrainComponent
{
public:

  LoggerModule(int id, OptionManager& mgr,
               const std::string& descrName = "LoggerModule",
               const std::string& tagName = "LoggerModule");

  ~LoggerModule();

  virtual void evolve();

  // Get a message
  virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
                             const Ice::Current&);

  virtual void registerTopics();

private:

  OModelParam<string> itsLogPath;
  std::ofstream itsLogFile;

  Image<PixRGB<byte> > itsCurrentFwdImg;
  Image<PixRGB<byte> > itsCurrentDwnImg;
  int itsCurrentHeading;
  int itsCurrentExtPressure;
  int itsCurrentIntPressure;

  IceUtil::Mutex itsDataMutex;

  int itsFwdFrameCount;
  int itsLastFwdFrameCount;
  int itsDwnFrameCount;
  int itsLastDwnFrameCount;


  PnmWriter itsImgWriter;

  bool itsLogOpen;

};

#endif

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "GUI/XWinManaged.H"
#include "GUI/ImageDisplayStream.H"
#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Util/Timer.H"
#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include <IceUtil/Thread.h>

#include <vector>

#include "Devices/Serial.H"

typedef struct {
  double dist,angle;
} sonarData;

class BeoSonar : public RobotBrainComponent
{
public:

  BeoSonar(int id, OptionManager& mgr,
      const std::string& descrName = "BeoSonar",
      const std::string& tagName = "BeoSonar");

  ~BeoSonar();

  virtual void evolve();

  //!Get a message
  virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
      const Ice::Current&);

  virtual void registerTopics();

  void plotDists();

  void start3();

private:
  nub::soft_ref<Serial> itsSerial;

  std::vector<sonarData> itsData;
  nub::soft_ref<OutputFrameSeries> itsOfs;
        Timer itsDisplayTimer;
        float itsDisplayUpdateRate;

        OModelParam<std::string> itsSerialDev;
};

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
#include "Robots/SeaBeeIII/CameraManager.H"
#include "Robots/SeaBeeIII/Camera.H"
#include "Image/ShapeOps.H"
#include "Raster/DeBayer.H"
#include <vector>


class StereoRetinaI : public RobotBrainComponent
{
public:

  StereoRetinaI(int id, OptionManager& mgr,
      const std::string& descrName = "StereoRetina",
      const std::string& tagName = "StereoRetina");

  ~StereoRetinaI();

  virtual void evolve();

  //!Get a message
  virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
      const Ice::Current&);

  virtual void registerTopics();

private:
  bool itsRunning;
  CameraManager cameraManager;
  //std::vector<Camera*> cameras;
  Camera* camera;
};


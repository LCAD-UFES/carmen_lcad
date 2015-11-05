#include "Robots/RobotBrain/RetinaI.H"


// ######################################################################
RetinaI::RetinaI(int id, OptionManager& mgr,
    const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsIfs(new InputFrameSeries(mgr)),
  itsRunning(true)
{
  addSubComponent(itsIfs);
}

// ######################################################################
RetinaI::~RetinaI()
{
}

void RetinaI::registerTopics()
{
  LINFO("Registering Retina Message");
  this->registerPublisher("RetinaMessageTopic");
  this->registerSubscription("CameraConfigTopic");
}

void RetinaI::evolve()
{
  if(itsRunning)
  {
    Image<PixRGB<byte> > img;
    itsIfs->updateNext(); img = itsIfs->readRGB();
    if(img.initialized())
    {
      LINFO("Sending Frame");
      RobotSimEvents::RetinaMessagePtr msg = new RobotSimEvents::RetinaMessage;
      msg->img = Image2Ice(img);
      msg->cameraID = itsIceIdentity.getVal();
      this->publish("RetinaMessageTopic", msg);
    }
    else
    {
      LERROR("Image Not Initialized!");
    }
  }
  printf("Evolve\n");

}

// ######################################################################
void RetinaI::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{
  //Get a retina message
  if(eMsg->ice_isA("::RobotSimEvents::CameraConfigMessage"))
  {
    RobotSimEvents::CameraConfigMessagePtr ccMsg = RobotSimEvents::CameraConfigMessagePtr::dynamicCast(eMsg);

    LINFO("Message is a camera config message: cameraID=%s, active:%d", ccMsg->cameraID.c_str(), ccMsg->active);

    if(ccMsg->cameraID == itsIceIdentity.getVal())
      itsRunning = ccMsg->active;

  }
}



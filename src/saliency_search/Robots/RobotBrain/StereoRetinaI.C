#include "Robots/RobotBrain/StereoRetinaI.H"

// ######################################################################
StereoRetinaI::StereoRetinaI(int id, OptionManager& mgr,
    const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsRunning(true)
{
}

// ######################################################################
StereoRetinaI::~StereoRetinaI()
{
}

void StereoRetinaI::registerTopics()
{
  LINFO("Registering Retina Message");
  this->registerPublisher("RetinaMessageTopic");
}

void StereoRetinaI::evolve()
{
  if (camera) {
    /*ImageIceMod::ImageIce rawImage = camera->getIceImage();
    Image<byte> monoImage = Ice2Image<byte>(rawImage);
    */
    Image<byte> monoImage = camera->getImage();
    Image<PixRGB<byte> > colorImage = deBayer(monoImage, BAYER_RGGB);
    Image<PixRGB<byte> > scaledImage = rescaleNI(colorImage, colorImage.getWidth()/2, colorImage.getHeight()/2);
    RobotSimEvents::RetinaMessagePtr msg = new RobotSimEvents::RetinaMessage;
    msg->img = Image2Ice(scaledImage);

    if (itsIceIdentity.getVal() == "LFwdCamera")       msg->cameraID = "LFwdCamera";
    else if (itsIceIdentity.getVal() == "RFwdCamera")  msg->cameraID = "RFwdCamera";
    else if (itsIceIdentity.getVal() == "LDownCamera")  msg->cameraID = "LDownCamera";
    else if (itsIceIdentity.getVal() == "RDownCamera")  msg->cameraID = "RDownCamera";
    else              msg->cameraID = "Unknown";

    this->publish("RetinaMessageTopic", msg);
  } else {

    std::vector<uint64_t> list = cameraManager.getCameraList();

    if (list.size() > 0 && itsIceIdentity.getVal() == "LFwdCamera") {
      camera = cameraManager.getCamera(list[0]);
    } else if (list.size() > 1 && itsIceIdentity.getVal() == "RFwdCamera") {
      camera = cameraManager.getCamera(list[1]);
    } else if (list.size() > 2 && itsIceIdentity.getVal() == "LDownCamera") {
      camera = cameraManager.getCamera(list[2]);
    } else if (list.size() > 3 && itsIceIdentity.getVal() == "RDownCamera") {
      camera = cameraManager.getCamera(list[3]);
    }
    printf("Cameras Detected: %d\n", list.size());
  }
  /*for (std::vector<Camera*>::size_type i = 0; i < cameras.size(); i++) {
    ImageIceMod::ImageIce rawImage = cameras[i]->getIceImage();
    Image<byte> monoImage = Ice2Image<byte>(rawImage);
    Image<PixRGB<byte> > colorImage = deBayer(monoImage, BAYER_RGGB);
    Image<PixRGB<byte> > scaledImage = rescaleNI(colorImage, colorImage.getWidth()/2, colorImage.getHeight()/2);
    RobotSimEvents::RetinaMessagePtr msg = new RobotSimEvents::RetinaMessage;
    msg->img = Image2Ice(scaledImage);

    if (i == 0)       msg->cameraID = "LFwdCamera";
    else if (i == 1)  msg->cameraID = "RFwdCamera";
    else if (i == 2)  msg->cameraID = "LDownCamera";
    else if (i == 3)  msg->cameraID = "RDownCamera";
    else              msg->cameraID = "Unknown";

    this->publish("RetinaMessageTopic", msg);
  }*/
}

// ######################################################################
void StereoRetinaI::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{
  //Get a retina message
  /*
  if(eMsg->ice_isA("::RobotSimEvents::CameraConfigMessage"))
  {
    RobotSimEvents::CameraConfigMessagePtr ccMsg = RobotSimEvents::CameraConfigMessagePtr::dynamicCast(eMsg);

    LINFO("Message is a camera config message: cameraID=%s, active:%d", ccMsg->cameraID.c_str(), ccMsg->active);

    if(ccMsg->cameraID == itsIceIdentity.getVal())
      itsRunning = ccMsg->active;

  }
  */
}



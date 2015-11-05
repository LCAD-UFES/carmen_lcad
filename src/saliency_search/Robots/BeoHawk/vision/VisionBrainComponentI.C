
#include "Robots/BeoHawk/vision/VisionBrainComponentI.H"
#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"

#ifndef VISIONBRAINCOMPONENTI_C
#define VISIONBRAINCOMPONENTI_C

const ModelOptionCateg MOC_VisionBrainComponent = {
    MOC_SORTPRI_3, "VisionBrainComponent Related Options" };

const ModelOptionDef OPT_CameraSource =
{ MODOPT_ARG(std::string), "CameraSource", &MOC_VisionBrainComponent, OPTEXP_CORE,
    "The string description to be matched against incoming Retina messages\n"
  "when using Ice as an input source",
    "camera-source", '\0', "<string>", "Camera0" };

const ModelOptionDef OPT_DesiredFramerate =
{ MODOPT_ARG(int), "ProcessFPS", &MOC_VisionBrainComponent, OPTEXP_CORE,
	"The desired framerate at which the component processes incoming images",
	"process-fps", '\0', "<int>", "-1" };

// ######################################################################
VisionBrainComponentI::VisionBrainComponentI(OptionManager& mgr,
    const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsFrameCount(0),
  desiredFramerate(&OPT_DesiredFramerate, this, 0),
  itsCameraSource(&OPT_CameraSource, this, 0),
  itsCurrentImgFwdCam(false),
  timer(1000),
  lastFrameCount(-1)
{
}

// ######################################################################
VisionBrainComponentI::~VisionBrainComponentI()
{
}

void VisionBrainComponentI::registerVisionTopics()
{
  LINFO("Registering VisionBrainComponent Messages/Subscriptions");
  this->registerSubscription("RetinaMessageTopic");
}

void VisionBrainComponentI::registerVisionPublisher(const std::string& MessageTopic)
{
  this->registerPublisher(MessageTopic);
  registerVisionTopics();
}

bool VisionBrainComponentI::publishVisionMsg(const::std::string& MessageTopic, RobotSimEvents::EventMessagePtr msg)
{
  return this->publish(MessageTopic,msg);
}

void VisionBrainComponentI::evolve()
{
	int framerate = desiredFramerate.getVal();

	if (framerate == 0)
		usleep(ZERO_RATE_WAIT_TIME*1000);
	else {
		if (itsCurrentImg.initialized() && itsFrameCount != lastFrameCount) {
			lastFrameCount = itsFrameCount;
		    updateFrame(itsCurrentImg, itsCurrentImgFwdCam);
		    updateFrame(itsCurrentImg, itsCurrentCameraID);
		}
		int ms_left = (1000/framerate) - (int) timer.get();
		if (framerate > 0 && ms_left > 0)
			usleep((ms_left)*1000);
	}

	//There used to be an ifs hack here.  It is gone now.  If you want to read from an ifs
	// then just start-up a retina which reads from the stream.  This is now much cleaner
	// and properly structured.  Also, if you want your inherited class to write to an ofs
	// then include the ofs in that class, not here, as many of the classes which inherit
	// this one have no need for an ofs.

}

// ######################################################################
void VisionBrainComponentI::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{
  //Get a retina message
  if(eMsg->ice_isA("::RobotSimEvents::RetinaMessage"))
  {
	  RobotSimEvents::RetinaMessagePtr retinaMessage = RobotSimEvents::RetinaMessagePtr::dynamicCast(eMsg);
	  int framerate = desiredFramerate.getVal();

	  //if our FPS == -1, we always process
	  bool processMessage = (framerate == -1);
	  //if our FPS > -1, then we process only when the timers run out
	  if (framerate > 0 && timer.get() > (unsigned int) (1000/framerate)) {
		  timer.reset();
		  processMessage = true;
	  }

	  if (processMessage)
	  {
		  itsImgMutex.lock();
		  if(retinaMessage->cameraID == itsCameraSource.getVal() || itsCameraSource.getVal() == "all")
		  {
			  Image<byte > retinaImage = Ice2Image<byte >(retinaMessage->img);
			  itsCurrentImg = retinaImage;
			  itsCurrentCameraID = retinaMessage->cameraID;

			  if(retinaMessage->cameraID == "FwdCamera")
				  itsCurrentImgFwdCam = true;
			  else
				  itsCurrentImgFwdCam = false;

			  itsFrameCount++;
		  }
		  itsImgMutex.unlock();
	  }
  }
}

#endif

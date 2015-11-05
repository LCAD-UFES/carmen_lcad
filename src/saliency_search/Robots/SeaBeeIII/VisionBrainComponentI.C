 #include "Robots/SeaBeeIII/VisionBrainComponentI.H"
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

// ######################################################################
VisionBrainComponentI::VisionBrainComponentI(OptionManager& mgr,
    const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
        itsOfs(new OutputFrameSeries(mgr)),
  itsFrameCount(0),
  itsCameraSource(&OPT_CameraSource, this, 0),
  itsCurrentImgFwdCam(false),
  lastFrameCount(-1)
{
  //addSubComponent(itsIfs);
  addSubComponent(itsOfs);
}

// ######################################################################
VisionBrainComponentI::~VisionBrainComponentI()
{
}

void VisionBrainComponentI::registerVisionTopics()
{
  LINFO("Registering VisionBrainComponent Message");
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

//        Image<PixRGB<byte> > img;
//        FrameState fs = itsIfs->updateNext();
//
//        // If using IFS, we do not currently check if img from forward or downward cam
//        if(fs == FRAME_NEXT)
//        {
//                img = itsIfs->readRGB();
//
//                if(img.initialized() && img.size() > 1)
//                {
//                        itsImgMutex.lock();
//                        itsCurrentImg = img;
//                        itsFrameCount++;
//                        itsImgMutex.unlock();
//                }
//        }
//

        itsImgMutex.lock();
        {
                if(itsCurrentImg.initialized() && itsFrameCount != lastFrameCount)
                {
                        lastFrameCount = itsFrameCount;
                        updateFrame(itsCurrentImg, itsCurrentCameraID);
                }
        }
        itsImgMutex.unlock();
                //
}

// ######################################################################
void VisionBrainComponentI::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
                const Ice::Current&)
{
        //Get a retina message
        if(eMsg->ice_isA("::RobotSimEvents::RetinaMessage"))
        {
                RobotSimEvents::RetinaMessagePtr retinaMessage = RobotSimEvents::RetinaMessagePtr::dynamicCast(eMsg);
                itsImgMutex.lock();
                if(retinaMessage->cameraID == itsCameraSource.getVal() || itsCameraSource.getVal() == "all")
                {
                        Image<PixRGB<byte> > retinaImage = Ice2Image<PixRGB<byte> >(retinaMessage->img);
                        itsCurrentImg = retinaImage;
                        itsCurrentCameraID = retinaMessage->cameraID;

                        if(retinaMessage->cameraID == "FwdCamera")
                                itsCurrentImgFwdCam = true;
                        else
                                itsCurrentImgFwdCam = false;

                        itsFrameCount++;

                        //updateFrame(itsCurrentImg, itsCurrentCameraID);
                }
                //      else
                //LINFO("id name does match src %s,%s",retinaMessage->cameraID.c_str(), itsCameraSource.c_str());
                itsImgMutex.unlock();
        }
}

#endif

#include "Robots/SeaBeeIII/SaliencyModuleI.H"
#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"
#include "Image/MathOps.H"

#ifndef SALIENCYMODULEI_C
#define SALIENCYMODULEI_C

const ModelOptionCateg MOC_SaliencyModule = {
  MOC_SORTPRI_3, "SeaBee Saliency Computer Related Options"
};

const ModelOptionDef OPT_ImageDescr =
{ MODOPT_ARG(std::string), "ImageDescr", &MOC_SaliencyModule, OPTEXP_CORE,
  "Description of the image on which we are computing saliency.\n"
    "(Generally, use Camera0, Camera1, etc...)",
  "image-descr", '\0', "<string>", "Camera0" };


// ######################################################################
SaliencyModuleI::SaliencyModuleI(OptionManager& mgr,
    const std::string& descrName, const std::string& tagName) :
  VisionBrainComponentI(mgr, descrName, tagName),
  itsEvc(new EnvVisualCortex(mgr)),
  itsImageDescr(&OPT_ImageDescr, this, 0),
  itsRunning(true)
{
  //Add the visual cortex
  addSubComponent(itsEvc);
}

// ######################################################################
void SaliencyModuleI::registerTopics()
{
  LINFO("Registering Saliency Point Message");
  this->registerPublisher("SalientPointMessageTopic");
  registerVisionTopics();
}

// ######################################################################
void SaliencyModuleI::updateFrame(Image<PixRGB<byte> > img, string cameraId)
{
        bool isFwdCamera = false;
        if(cameraId == "FwdCamera")
                isFwdCamera = true;

        updateFrame(img, isFwdCamera);

}

// ######################################################################
void SaliencyModuleI::updateFrame(Image<PixRGB<byte> > img, bool isFwdCamera)
{
  if(itsRunning)
  {
    if(img.initialized())
    {
      // Input the new image into the envision visual cortex
      itsEvc->input(img);
      // Grab the visual cortex output map (saliency map)
      Image<byte> vcxMap = itsEvc->getVCXmap();
      Point2D<int> pt;
      byte saliency;
      findMax(vcxMap, pt, saliency);
      pt.i = pt.i << itsEvc->getMapLevel();
      pt.j = pt.j << itsEvc->getMapLevel();

      Point2D<int> imgCenter = Point2D<int>(img.getWidth(),img.getHeight());
      Point2D<int> err = pt - imgCenter;

                        if(!itsOfs->isVoid())
                        {
                                drawCircle(img,pt,5,PixRGB<byte>(0,0,255));
                                itsOfs->writeRGB(img, "Saliency Module Image",
                                                FrameInfo("Saliency Module Image", SRC_POS));

                                itsOfs->updateNext();
                        }

      RobotSimEvents::SalientPointMessagePtr msg = new RobotSimEvents::SalientPointMessage;
      msg->x = float(pt.i)/float(img.getWidth());
      msg->y = float(pt.j)/float(img.getHeight());
      msg->saliency = saliency;
      msg->imageDescr = itsImageDescr.getVal();

                        LINFO("Calculated Saliency");
      this->publish("SalientPointMessageTopic", msg);

    }
    else
    {
      LERROR("Image Not Initialized!");
    }

  }
}

#endif


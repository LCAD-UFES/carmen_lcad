#include "Robots/SeaBeeIII/CircleDetectionComponent.H"

#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"

#ifndef CIRCLEDETECTIONCOMPONENT_C
#define CIRCLEDETECTIONCOMPONENT_C

// ######################################################################
CircleDetectionComponent::CircleDetectionComponent(int id, OptionManager& mgr,
    const std::string& descrName, const std::string& tagName) :
  VisionBrainComponentI(mgr, descrName, tagName)
{
}

// ######################################################################
CircleDetectionComponent::~CircleDetectionComponent()
{

}

void CircleDetectionComponent::registerTopics()
{
        //Use Internet Communication Engine (ICE) to send/receive messages
}

void CircleDetectionComponent::updateFrame(Image<PixRGB<byte> > img, bool itsFwdCamera)
{
  LINFO("Image Recieved in the updateFrame");

  if(img.initialized())
    itsOfs->writeRGB(img, "Circle Detection Component Image",
                     FrameInfo("Circle Detection Component", SRC_POS));


}
#endif


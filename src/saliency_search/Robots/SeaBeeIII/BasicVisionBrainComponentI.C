#include "Robots/SeaBeeIII/BasicVisionBrainComponentI.H"

#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"

#ifndef BASICVISIONBRAINCOMPONENTI_C
#define BASICVISIONBRAINCOMPONENTI_C

// ######################################################################
BasicVisionBrainComponentI::BasicVisionBrainComponentI(int id, OptionManager& mgr,
    const std::string& descrName, const std::string& tagName) :
  VisionBrainComponentI(mgr, descrName, tagName)
{
}

// ######################################################################
BasicVisionBrainComponentI::~BasicVisionBrainComponentI()
{
}

void BasicVisionBrainComponentI::registerTopics()
{

        registerVisionTopics();
}

void BasicVisionBrainComponentI::updateFrame(Image<PixRGB<byte> > img, std::string cameraId)
{
  if(img.initialized())
        {
    itsOfs->writeRGB(img, cameraId);
                itsOfs->updateNext();
        }
}

#endif

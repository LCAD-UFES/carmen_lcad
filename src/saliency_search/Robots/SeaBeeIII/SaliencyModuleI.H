#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "GUI/ImageDisplayStream.H"
#include "GUI/XWinManaged.H"
#include "Image/Image.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Media/FrameSeries.H"
#include "Neuro/EnvVisualCortex.H"
#include "Raster/GenericFrame.H"
#include "Transport/FrameInfo.H"
#include "Robots/SeaBeeIII/VisionBrainComponentI.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/IceImageUtils.H"
#include <IceUtil/Thread.h>


#ifndef SALIENCYMODULEI_H
#define SALIENCYMODULEI_H

class SaliencyModuleI : public VisionBrainComponentI
{
public:

  SaliencyModuleI(OptionManager& mgr,
      const std::string& descrName = "SaliencyModule",
      const std::string& tagName = "SaliencyModule");

  void updateFrame(Image<PixRGB<byte> > img, bool isFwdCamera);

        //Get rid of this function! There should only be one update frame...
        void updateFrame(Image<PixRGB<byte> > img, string cameraId);

  virtual void registerTopics();

private:
  nub::soft_ref<EnvVisualCortex>  itsEvc;

  OModelParam<std::string>        itsImageDescr;

  bool itsRunning;

};

#endif


#include "Component/ModelManager.H"
#include "Component/OptionManager.H"

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
#include "Robots/SeaBeeIII/VisionBrainComponentI.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/IceImageUtils.H"
#include <IceUtil/Thread.h>

#ifndef PIXELIZERI_H
#define PIXELIZERI_H


class PixelizerI : public VisionBrainComponentI
{
public:

  PixelizerI(int id, OptionManager& mgr,
      const std::string& descrName = "Pixelizer",
      const std::string& tagName = "Pixelizer");

  ~PixelizerI();

  virtual void updateFrame(Image<PixRGB<byte> > img, bool isFwdCamera);

  PixRGB<byte> getAvgTileColor(int colOffset,
                               int rowOffset,
                               const Image<PixRGB<byte> >& img);
  virtual void registerTopics();

private:


};

#endif

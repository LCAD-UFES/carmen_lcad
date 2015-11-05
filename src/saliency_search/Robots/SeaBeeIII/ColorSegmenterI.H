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
#include "Image/Pixels.H"
#include "Robots/SeaBeeIII/VisionBrainComponentI.H"

#include "VFAT/segmentImage2.H"

#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include "Ice/IceImageUtils.H"
#include <IceUtil/Thread.h>

#define IMG_WIDTH  320
#define IMG_HEIGHT 240

#ifndef COLORSEGEMENTERI_H
#define COLORSEGEMENTERI_H


class ColorSegmenterI : public VisionBrainComponentI
{
public:
	ColorSegmenterI(OptionManager& mgr,
	  const std::string& descrName = "ColorSegmenter",
	  const std::string& tagName = "ColorSegmenter");

	~ColorSegmenterI();

  void updateFrame(Image<PixRGB<byte> > img, bool isFwdCamera);

		//Get rid of this function! There should only be one update frame...
		void updateFrame(Image<PixRGB<byte> > img, string cameraId);

  virtual void registerTopics();
  //virtual void updateMessage(const RobotSimEvents::EventMessagePtr&,
  //              const Ice::Current&);

private:

  bool itsInit;
  bool itsRunning;
  segmentImage2 itsBuoySegmenter;
  segmentImage2 itsPipeSegmenter;
};

#endif

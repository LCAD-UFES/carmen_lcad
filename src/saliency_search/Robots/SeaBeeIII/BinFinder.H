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
#include <opencv/cv.h>
#include <IceUtil/Thread.h>

#ifndef BINFINDER_H
#define BINFINDER_H


class BinFinder : public VisionBrainComponentI
{
public:

  BinFinder(OptionManager& mgr,
      const std::string& descrName = "BinFinder",
      const std::string& tagName = "BinFinder");

  ~BinFinder();

  virtual void updateFrame(Image<PixRGB<byte> > img, std::string cameraId);

  //!Get a message
  //virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
  //const Ice::Current&);

  virtual void registerTopics();

private:
  CvSeq* findSquares4( IplImage* img, CvMemStorage* storage );
  double angle( CvPoint* pt1, CvPoint* pt2, CvPoint* pt0 );

  bool inCorner(CvSeq* result);


  rutz::shared_ptr<XWinManaged>   itsMwin;       // display window
  Image<PixRGB<byte> >            itsDisp;       // display image
  uint itsWidth;
  uint itsHeight;
};

#endif

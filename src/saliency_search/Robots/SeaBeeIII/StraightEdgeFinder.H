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

#include "Image/Point2D.H"
#include "MBARI/Geometry2D.H"
#include "VFAT/segmentImageTrackMC.H"

#ifndef STRAIGHT_EDGE_FINDER_H
#define STRAIGHT_EDGE_FINDER_H


class StraightEdgeFinder : public VisionBrainComponentI
{
public:

  StraightEdgeFinder(OptionManager& mgr,
      const std::string& descrName = "StraightEdgeFinder",
      const std::string& tagName = "StraightEdgeFinder");

  ~StraightEdgeFinder();

  virtual void updateFrame(Image<PixRGB<byte> > img, std::string cameraId);

  virtual void registerTopics();

  enum PipeRecognizeMethod { HOUGH, LINE_BEST_FIT, CONTOUR };

private:

  std::vector<LineSegment2D> getPipeLocation
  (rutz::shared_ptr<Image<byte> > colorSegmentedImage,
   rutz::shared_ptr<Image<PixRGB <byte> > > outputImage,
   PipeRecognizeMethod method);

  std::vector<LineSegment2D> calculateHoughTransform
  (Image<byte>& colorSegmentedImage,
   rutz::shared_ptr<Image<PixRGB<byte> > > outputImage);

  uint calculateLineBestFit
  (Image<byte>  &colorSegmentedImage,
   Image<PixRGB <byte> >  &outputImage,
   Point2D<int> &pipeCenter,
   double &pipeAngle);

  uint calculateContours
  (Image<byte>  &colorSegmentedImage,
   Image<PixRGB <byte> >  &outputImage,
   Point2D<int> &pipeCenter,
   double &pipeAngle);

  std::vector<LineSegment2D> getHoughLines
  ( IplImage cannyImage );

  IplImage getCannyImage
  ( Image<byte> colorSegmentedImage );

  std::vector<LineSegment2D> pruneHoughLines
  (const std::vector<LineSegment2D> lineSegments);

  float isolateOrange3
  (Image< PixRGB<byte> > &inputImage,  Image<byte> &outputImage);

  float isolateOrange4
  (Image< PixRGB<byte> > &inputImage,  Image<byte> &outputImage);

  float isolateOrange5
  (Image< PixRGB<byte> > &inputImage,  Image<byte> &outputImage);

  float normalizeAngle(float angleInRads);

  Image<PixRGB<byte> > itsCurrentImg;
  bool itsCurrentImgFwdCam;

  //  segmentImageTrackMC<float,unsigned int,4> *segmenter;



  Image<PixRGB<byte> >            itsDispImg;       // display image
  uint itsWidth;
  uint itsHeight;
};

#endif

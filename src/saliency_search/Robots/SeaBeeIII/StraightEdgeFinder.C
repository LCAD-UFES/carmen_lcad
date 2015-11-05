#include "Robots/SeaBeeIII/StraightEdgeFinder.H"

#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"
#include "Image/OpenCVUtil.H"
#include "Raster/Raster.H"
#include "Image/Pixels.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"
#include "Image/CutPaste.H"
#include "Image/OpenCVUtil.H"

#include "BeoSub/IsolateColor.H"
#include "Image/DrawOps.H"
#include "Image/ColorOps.H"

#include "GUI/XWinManaged.H"

#include "SeaBee/PipeRecognizer.H"
#include "BeoSub/ColorSegmenter.H"
#include "SeaBee/VisionRecognizer.H"

#include "Image/ColorOps.H"


#include "Media/MediaOpts.H"

#include "VFAT/segmentImageMerge.H"

#include "Util/Types.H"
#include "Util/log.H"


#ifndef STRAIGHT_EDGE_FINDER_C
#define STRAIGHT_EDGE_FINDER_C

using namespace std;

#define MIN_CENTER_DIST  15
#define MIN_AREA         150
#define CORNER_TOLERANCE 4

#define IMG_WIDTH 320
#define IMG_HEIGHT 240
int thresh = 42;
double angle_thresh = 0.3;


// ######################################################################
StraightEdgeFinder::StraightEdgeFinder
( OptionManager& mgr,
  const std::string& descrName,
  const std::string& tagName) :
  VisionBrainComponentI(mgr, descrName, tagName)
{

  itsWidth  = IMG_WIDTH;
  itsHeight = IMG_HEIGHT;
  itsDispImg.resize(2*itsWidth, 2*itsHeight);


  /*int wi = itsWidth/4;
  int hi = itsHeight/4;

  segmenter = new segmentImageTrackMC<float,unsigned int, 4> (itsWidth*itsHeight);

  segmenter->SITsetFrame(&wi,&hi);

  segmenter->SITsetCircleColor(0,255,0);
  segmenter->SITsetBoxColor(255,255,0,0,255,255);
  segmenter->SITsetUseSmoothing(false,10);


  segmenter->SITtoggleCandidateBandPass(false);
  segmenter->SITtoggleColorAdaptation(false);*/
}


// ######################################################################
StraightEdgeFinder::~StraightEdgeFinder()
{
}

// ######################################################################
void StraightEdgeFinder::registerTopics()
{
  LINFO("Registering StraightEdge Message");
  this->registerPublisher("StraightEdgeMessageTopic");
  registerVisionTopics();
}



// ######################################################################
void StraightEdgeFinder::updateFrame(Image<PixRGB<byte> > img, std::string cameraId)
{
        bool isFwdCamera = false;

        if(cameraId == "FwdCamera")
                isFwdCamera = true;

  LINFO("Image Received: %d", itsFrameCount);

      itsDispImg.clear();
      inplacePaste(itsDispImg, img, Point2D<int>(0, 0));

      uint w = itsWidth; uint h = itsHeight;
      rutz::shared_ptr<Image< PixRGB<byte> > >
        outputImg(new Image<PixRGB<byte> >(w,h, ZEROS));

      //get all of the orange pixels in the image
      rutz::shared_ptr<Image<byte> > orangeIsoImage;
      orangeIsoImage.reset(new Image<byte>(w,h, ZEROS));
      orangeIsoImage->resize(w,h);
      float res = isolateOrange(img, *orangeIsoImage);
      LINFO("result: %f", res);

      inplacePaste(itsDispImg, toRGB(*orangeIsoImage), Point2D<int>(w,0));

      //get all the orange lines in the image
      std::vector<LineSegment2D> pipelines =
        getPipeLocation
        (orangeIsoImage, outputImg, StraightEdgeFinder::HOUGH);

      int minY = -1; //minimum midpoint y coordinate found
      int followLineIndex = -1; //index of pipeline with minimum y coordinate

      //iterates through pipelines and finds the topmost one in the image
      for(uint i = 0; i < pipelines.size(); i++)
        {
          if(pipelines[i].isValid())
            {
              LineSegment2D pipeline = pipelines[i];
              Point2D<int> midpoint = (pipeline.point1() + pipeline.point2())/2;

              if(midpoint.j < minY || minY == -1)
                {
                  minY = midpoint.j;
                  followLineIndex = i;
                }
            }
        }

      //if we found a pipeline
      if(followLineIndex != -1)
        {
          LineSegment2D followLine = pipelines[followLineIndex];
          Point2D<int> midpoint = (followLine.point1() + followLine.point2())/2;

          Point2D<int> projPoint;
          projPoint.i = (int)(midpoint.i+30*cos(followLine.angle()));
          projPoint.j = (int)(midpoint.j+30*sin(followLine.angle()));

          drawLine(*outputImg, midpoint, projPoint,
                   PixRGB <byte> (255, 255,0), 3);

          inplacePaste(itsDispImg, *outputImg, Point2D<int>(0,h));

          // publish the result
          RobotSimEvents::StraightEdgeMessagePtr msg =
            new RobotSimEvents::StraightEdgeMessage;

          ImageIceMod::LineIce line;
          line.pt1.i = midpoint.i;
          line.pt1.j = midpoint.j;
          line.pt2.i = projPoint.i;
          line.pt2.j = projPoint.j;

          float radAngle,normalAngle;
          radAngle = followLine.angle();
          normalAngle = normalizeAngle(radAngle);

          LINFO("angle in rads: %1.6f | normalized deg: %1.6f",radAngle,normalAngle);

          line.angle = normalAngle;
          msg->line = line;
          msg->isFwdCamera = true;

          LINFO("SEMessage: line: [%4d,%4d][%4d,%4d]: %10.6f",
                line.pt1.i, line.pt1.j,
                line.pt2.i, line.pt2.j, line.angle);
          this->publish("StraightEdgeMessageTopic", msg);
        }

      itsOfs->writeRGB(itsDispImg, "Straight Edge Finder Image",
                       FrameInfo("Straight Edge Finder Image", SRC_POS));

      itsOfs->updateNext();
}

// ######################################################################
std::vector<LineSegment2D> StraightEdgeFinder::getPipeLocation
(rutz::shared_ptr<Image<byte> > colorSegmentedImage,
 rutz::shared_ptr<Image<PixRGB <byte> > > outputImage,
 PipeRecognizeMethod method)
{
  if(!colorSegmentedImage->initialized())
    return std::vector<LineSegment2D>();

  //  Image<byte> lum = luminance(*colorSegmentedImage);
  Image<byte> lum = *colorSegmentedImage;

  switch(method)
    {
    case HOUGH:
      return calculateHoughTransform(lum,
                                     outputImage);
      break;

     default:
       LERROR("Invalid pipe recognizer method specified");
       return std::vector<LineSegment2D>();
     }
}

// ######################################################################
std::vector<LineSegment2D> StraightEdgeFinder::calculateHoughTransform
(Image<byte>& colorSegmentedImage,
 rutz::shared_ptr<Image<PixRGB<byte> > > outputImage)
{
#ifndef HAVE_OPENCV
  LFATAL("OpenCV must be installed in order to use this function");
#else
  // Do edge detection (canny) on the image.
  IplImage cannyImage = getCannyImage( colorSegmentedImage );

  // Clear output image and set it equal to canny image.
  //  outputImage->clear();
  //rutz::shared_ptr<Image<PixRGB<byte> > > temp
  //(new Image<PixRGB<byte> > ( toRGB( ipl2gray( &cannyImage ) ) );
   // Cannot convert directly to RGB
  //since cannyImage has only 1 channel (black and white).
  //  temp.resize(outputImage->getDims());
  //  *outputImage += temp;

  // Do Hough transform.
  std::vector <LineSegment2D> lineSegments = getHoughLines( cannyImage );

  // Loop through hough lines and draw them to the screen.
  for(uint i = 0; i < lineSegments.size(); i++ )
    {
      Point2D<int> pt1 = lineSegments[i].point1();
      Point2D<int> pt2 = lineSegments[i].point2();

      if(pt1.isValid() && pt2.isValid())
        {
          //draw line segment in output image
          drawLine(*outputImage, pt1, pt2, PixRGB<byte>(255,0,0));
        }
    }

  std::vector <LineSegment2D> prunedHoughLines =
    pruneHoughLines( lineSegments );

  return prunedHoughLines;

  #endif // HAVE_OPENCV
}

// ######################################################################

uint StraightEdgeFinder::calculateLineBestFit
(Image<byte>  &colorSegmentedImage,
 Image<PixRGB <byte> >  &outputImage,
 Point2D<int> &pipeCenter,
 double &pipeAngle)
{return 0;}

uint StraightEdgeFinder::calculateContours
(Image<byte>  &colorSegmentedImage,
 Image<PixRGB <byte> >  &outputImage,
 Point2D<int> &pipeCenter,
 double &pipeAngle)
{return 0;}

// double PipeRecognizer::getOrangePixels(Image<byte> &cameraImage,
//                                                   double &avgX,
//                                                   double &avgY,
//                                                   double &sumX,
//                                                   double &sumY)
// {
//   Timer tim(1000000);

//   std::vector <Point2D<int> > edgePoints;
//   uint w = cameraImage.getWidth();
//   uint h = cameraImage.getHeight();

//   Image<byte> (*colorSegmentedImage)(w,h, ZEROS);

//   (*colorSegmentedImage) = cameraImage;

//   avgX = 0.0;
//   avgY = 0.0;
//   sumX = 0.0;
//   sumY = 0.0;

//   //Isolate the orange pixels in the image
//   tim.reset();

//   // isolateOrange(cameraImage, orangeIsoImage); //, fnb=0;


//   //find all the white edge pixels in the image and store them
//   for(int y = 0; y < orangeIsoImage.getHeight(); y++)
//     {
//       for(int x = 0; x < orangeIsoImage.getWidth(); x++)
//         {
//           if(orangeIsoImage.getVal(x,y) == 255)
//             {
//             // convert the x,y position of the pixel to an x,y position where
//             // the center of the image is the origin as opposed to the top left corner
//             // and store the pixel
//               edgePoints.push_back(Point2D<int>(x, y));

//               sumX += x;
//               sumY += y;
//             }
//         }
//     }

//   avgX = sumX/edgePoints.size();
//   avgY = sumY/edgePoints.size();

//   return getSlope(orangeIsoImage, edgePoints, avgX, avgY, sumX, sumY);
// }

// double PipeRecognizer::getSlope(Image<byte> &cameraImage,
//                std::vector <Point2D<int> > &points,
//                double avgX,
//                double avgY,
//                double sumX,
//                doubley sumY)
// {
//   double top = 0.0;
//   double bottom = 0.0;
//   double top2 = 0.0;
//   double bottom2 = 0.0;
//   double return_value = 0.0;
//   double return_value2 = 0.0;

//   int x = 0;
//   int y = 0;

//   /* loop through all the points in the picture and generate a slope
//      by finding the line of best fit*/
//   for(uint i = 0; i < points.size(); i++)
//     {
//       x = points[i].i;
//       y = points[i].j;

//           top += (x - avgX) * (y - avgY);
//           bottom += (x - avgX) * (x - avgX);

//       int tx =  x- cameraImage.getWidth()/2;
//       int ty =  y- cameraImage.getHeight()/2;
//       x = ty +cameraImage.getHeight()/2;
//       y = -tx + cameraImage.getWidth()/2;

//           top2 += (x - avgX) * (y - avgY);
//           bottom2 += (x - avgX) * (x - avgX);

//     }

//   if( bottom != 0.0 )
//       return_value =  atan2(top,bottom);
//   else
//       return_value = 1.62;  //if the bottom is zero, we have a vertical line,
//                            //so we want to return pi/2

//   if( bottom2 != 0.0 )
//       return_value2 =  (atan2(top2,bottom2)+3.14159/2);
//   else
//       return_value2 = (1.62+3.14159/2);


//   double e1 = 0.0;
//   double e2 = 0.0;
//   for(uint i = 0; i < points.size(); i++)
//     {

//       x = points[i].i;
//       y = points[i].j;

//       e1 =pow(x/bottom*top+avgY-y,2);

//       int tx =  x- cameraImage.getWidth()/2;
//       int ty =  y- cameraImage.getHeight()/2;
//       x = ty +cameraImage.getHeight()/2;
//       y = -tx + cameraImage.getWidth()/2;


//       e2 =pow(x/bottom2*top2+avgY-y,2);
//     }


//   if(e1<e2)
//     return return_value;
//   return return_value2;
// }


// ######################################################################
std::vector<LineSegment2D> StraightEdgeFinder::getHoughLines
( IplImage cannyImage )
{
#ifndef HAVE_OPENCV
  LFATAL("OpenCV must be installed in order to use this function");
#else
  // Storage for use in hough transform.
  CvMemStorage* storage = cvCreateMemStorage(0);

  // Perform hough transform and store hough lines.
  CvSeq* cvLines = cvHoughLines2(&cannyImage, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 30, 20, 10);


  // Storage for hough line segments.
  std::vector <LineSegment2D> lineSegments;

  // Loop through hough lines, store them as line segments, and draw lines in output image.
  for(int i = 0; i < cvLines->total; i++ )
  {
    // Get a line.
    CvPoint* line = (CvPoint*)cvGetSeqElem(cvLines,i);

    // Get line end points.
    Point2D<int> pt1 = Point2D<int>(line[0].x,line[0].y);
    Point2D<int> pt2 = Point2D<int>(line[1].x,line[1].y);

    // Create line segment from endpoints and store.
    lineSegments.push_back(LineSegment2D(pt1,pt2));
  }
  cvReleaseMemStorage( &storage );

  return lineSegments;
#endif // HAVE_OPENCV
}


// ######################################################################
IplImage StraightEdgeFinder::getCannyImage( Image<byte> colorSegmentedImage )
{
#ifndef HAVE_OPENCV
  LFATAL("OpenCV must be installed in order to use this function");
#else
        // Find edges of segmented image using canny.
  IplImage *edge = cvCreateImage( cvGetSize( img2ipl( colorSegmentedImage ) ), 8, 1 );
  cvCanny( img2ipl( luminance( colorSegmentedImage ) ), edge, 100, 150, 3 );//150,200,3

        return *edge;
#endif // HAVE_OPENCV
}


// ######################################################################
std::vector<LineSegment2D> StraightEdgeFinder::pruneHoughLines
(const std::vector<LineSegment2D> lineSegments)
{
  uint numLines = lineSegments.size();
  if(numLines == 0) { LDEBUG("No hough lines to prune"); }

  std::vector< std::vector<LineSegment2D> > pipeLines;

  //Go through all the lines
  for(uint r = 0; r < numLines; r++)
    {
      int lnIndex = -1;

      //check to see if the current lines fits into a bucket
      for(uint c = 0; c < pipeLines.size(); c++)
        {
          LineSegment2D pipeLine = pipeLines[c][0];

          if(pipeLine.isValid() && lineSegments[r].angleBetween(pipeLine) < 5*(M_PI/180))//convert 5 degrees to radians
          {
            lnIndex = c;
            break;
          }
        }

      //if the line fits into a pre-existing bucket, add it to the bucket
      if( lnIndex > 0 )
        {
          pipeLines[lnIndex].push_back(lineSegments[r]);
          //average the old bucket's value with the new line added
          //so as to create a moving bucket
          Point2D<int> newPt1 =
            Point2D<int>(((lineSegments[r].point1().i + pipeLines[lnIndex][0].point1().i)/2),
                         ((lineSegments[r].point1().j + pipeLines[lnIndex][0].point1().j)/2));

          Point2D<int> newPt2 = Point2D<int>(((lineSegments[r].point2().i + pipeLines[lnIndex][0].point2().i)/2),
                            ((lineSegments[r].point2().j + pipeLines[lnIndex][0].point2().j)/2));

          pipeLines[lnIndex][0] = LineSegment2D(newPt1,newPt2);

        }
      //otherwise, create a new bucket
      else
        {
          std::vector<LineSegment2D> newCntrLines;
          newCntrLines.push_back(lineSegments[r]);
          pipeLines.push_back(newCntrLines);
        }
    }

  std::vector<LineSegment2D> centerPipeLines;

  uint pipeLineSize = pipeLines.size();

  for(uint c = 0; c < pipeLineSize; c++)
    {
      centerPipeLines.push_back(pipeLines[c][0]);
    }
//  std::vector<LineSegment2D> centerPipeLines;

//   Point2D<int> two = Point2D<int>(2,2);

//   for(uint c = 0; c < pipeLines.size(); c++)
//     {
//       if(pipeLines[c].size() == 2)
//         {
//           Point2D<int> endPoint1 = Point2D<int>((pipeLines[c][0].point1()+pipeLines[c][1].point1())/two);
//           Point2D<int> endPoint2 = Point2D<int>((pipeLines[c][0].point2()+pipeLines[c][1].point2())/two);

//           centerPipeLines.push_back(LineSegment2D(endPoint1,endPoint2));
//         }
//     }

  return centerPipeLines;
}

// //true test with frame 3498
// float StraightEdgeFinder::isolateOrange
// (Image< PixRGB<byte> > &inputImage,  Image<byte> &outputImage)
// {
//   Image< PixRGB<byte> > tempImage(inputImage);
//   Image<PixH2SV2<float> > h2svImage(tempImage);

//   readConfig colorConf;
//   colorConf.openFile("colortrack.conf", false);

//   int orangeCount = 0;

//   Image<PixRGB<byte> >::iterator iptr = inputImage.beginw();
//   Image<byte>::iterator          optr = outputImage.beginw();
//   Image<PixRGB<byte> >::iterator stop = inputImage.endw();

//   float tR = colorConf.getItemValueF("ORANGE_stdR");//70.0;
//   float tG = colorConf.getItemValueF("ORANGE_stdG");//200.0;
//   float tB = colorConf.getItemValueF("ORANGE_stdB");//128.0;

//   float R = colorConf.getItemValueF("ORANGE_R");//255;
//   float G = colorConf.getItemValueF("ORANGE_G");//198;
//   float B = colorConf.getItemValueF("ORANGE_B");//0;


//   //for average
//   float totalHue = 0.0;
//   int numTotal = 0;

//   //orange as a hue
//   float pure_orange_hue =  60*(((G/255)-(B/255))/((R/255)-(B/255)))+0;
//   float orange_hue = 60*(((tB/255)-(tR/255))/((tG/255) - (tR/255)))+120;
//   //orange saturation (of HSL)
//   float orange_sat = ((200.0/255.0)-(70/255.0))/(2.0-(270.0/255.0));//using tR,tG,tB, R,B,G gives '1'

//   std::cout<<"orange hue is: "<<orange_hue<<std::endl;
//   std::cout<<"orange saturation(purity) is: "<<orange_sat<<std::endl;
//   std::cout<<"orange HSV saturation is: "<<(1.0-70.0/200.0)<<std::endl;
//   //  LINFO("orange values (RGB):(std RGB): %f, %f, %f: %f, %f, %f", R, G, B, tR, tG, tB);

//   while (iptr != stop)
//     {
//       float hue = 0.0;
//       float s = 0.0; //saturation
//       float avgR = (*iptr).red();
//       float avgG = (*iptr).green();
//       float avgB = (*iptr).blue();
//       float r = avgR/255;
//       float g = avgG/255;
//       float b = avgB/255;




//       //do conversion to HSV to find the hue
//       float max = 0;
//       float min = 1;
//       //find max
//       if(r > max) { max = r;}
//       if(g > max) { max = g;}
//       if(b > max) { max = b;}
//       //find min
//       if(r < min){min = r;}
//       if(g < min){min = g;}
//       if(b < min){min = b;}

//       //do conversion to find hue
//       if(max == min) {hue = 0.0;}
//       else if(max == r && g >= b) {hue = 60.0*((g-b)/(max - min)) + 0.0;}
//       else if(max == r && g < b) {hue = 60.0*((g-b)/(max - min)) + 360.0;}
//       else if(max == g) {hue =  60.0*((b-r)/(max-min))+120.0;}
//       else if(max == b) {hue = 60.0*((r-g)/(max-min))+240.0;}


//       //for average calculation
//       totalHue += hue;
//       numTotal++;

//       //find saturation
//       if(max){s = max;}
//       if(max != 0){s = 1 - min/max;}
//       //std::cout<<" "<<hue;
//       if(hue == orange_hue)//result:get spects here and there
//         {
//           //(*optr) = (byte)255;  // orange
//           //orangeCount++;
//         }
//       if(hue == pure_orange_hue)//result:nothing
//         {
//           //(*optr) = (byte)255;  // orange
//           //orangeCount++;
//         }
//         //to reason these numbers 145 is about the value of orange hue
//         //pretty good but with spects, "s != 1" gets rid of specs, but also takes out some of the pipe
//         //value of 120 to 145 seems best
//         //using 130 as min makes it less accurate
//         //using a higher max does not seem to make a difference
//         //probably because of the colors involved here
//       if(!(120<hue && hue<146) &&
//          s != 1)
//         {
//         //std::cout<<" "<<s;
//           (*optr) = (byte)255;  // orange
//           orangeCount++;
//         }

//       //float avg = (avgR+avgG+avgB)/3.0;
//       //float sigma = pow(avgR - avg, 2.) + pow(avgG - avg, 2.) + pow(avgB - avg, 2.);
//       //float stdDev = sqrt( (1./3.) * sigma );

//         //result: pretty good but is confused by highlights
//       if (avgR > R - tR && avgR < R + tR &&
//           avgG > G - tG && avgG < G + tG &&
//           avgB > B - tB && avgB < B + tB   )
//         {
//           (*optr) = (byte)255;  // orange
//           orangeCount++;
//         }
//       //       else
//       //         {
//       //           //if(outputImage.coordsOk(i,j)){
//       //           //(*optr) = (byte)0; //not orange
//       //           //}
//       //         }
//       iptr++; optr++;

//     }

//       //display image to compare to what we get with the color segmenter

//       Image<PixRGB<byte> > Aux;
//       Aux.resize(100,450,true);

//       /******************************************************************/
//       // SEGMENT IMAGE ON EACH INPUT FRAME

//       segmenter->SITtrackImageAny(h2svImage,&inputImage,&Aux,true);

//       /* Retrieve and Draw all our output images */
//       Image<byte> temp = quickInterpolate(segmenter->SITreturnCandidateImage(),4);
//       //display for now for testing purposes
//       //wini->drawImage(display);
//       //wino->drawImage(temp);

//       std::cout<<"average hue was "<<totalHue/numTotal<<std::endl;
//       return float(orangeCount)/float( (inputImage.getHeight() * inputImage.getWidth()));
// }


//true test with frame 3498
float StraightEdgeFinder::isolateOrange3(Image< PixRGB<byte> > &inputImage,  Image<byte> &outputImage)
{

  readConfig colorConf;
  colorConf.openFile("colortrack.conf", false);

  int orangeCount = 0;

  Image<PixRGB<byte> >::iterator iptr = inputImage.beginw();
  Image<byte>::iterator          optr = outputImage.beginw();
  Image<PixRGB<byte> >::iterator stop = inputImage.endw();

//   float tR = colorConf.getItemValueF("ORANGE_stdR");//70.0;
//   float tG = colorConf.getItemValueF("ORANGE_stdG");//200.0;
//   float tB = colorConf.getItemValueF("ORANGE_stdB");//128.0;

//   float R = colorConf.getItemValueF("ORANGE_R");//255;
//   float G = colorConf.getItemValueF("ORANGE_G");//198;
//   float B = colorConf.getItemValueF("ORANGE_B");//0;

  // seem to be unused values
  float tR = 70.0;
  float tG = 200.0;
  float tB = 128.0;

  float R = 255;
  float G = 198;
  float B = 0;




  //for average
  float totalHue = 0.0;
  int numTotal = 0;

  //orange as a hue
  float pure_orange_hue =  60*(((G/255)-(B/255))/((R/255)-(B/255)))+0;
  float orange_hue = 60*(((tB/255)-(tR/255))/((tG/255) - (tR/255)))+120;
  //orange saturation (of HSL)
  float orange_sat = ((200.0/255.0)-(70/255.0))/(2.0-(270.0/255.0));//using tR,tG,tB, R,B,G gives '1'
  std::cout<<"orange hue is: "<<orange_hue<<std::endl;
  std::cout<<"orange saturation(purity) is: "<<orange_sat<<std::endl;
  std::cout<<"orange HSV saturation is: "<<(1.0-70.0/200.0)<<std::endl;
  //  LINFO("orange values (RGB):(std RGB): %f, %f, %f: %f, %f, %f", R, G, B, tR, tG, tB);
  while (iptr != stop)
    {
      float hue = 0.0;
      float s = 0.0; //saturation
      float avgR = (*iptr).red();
      float avgG = (*iptr).green();
      float avgB = (*iptr).blue();
      float r = avgR/255;
      float g = avgG/255;
      float b = avgB/255;




      //do conversion to HSV to find the hue
      float max = 0;
      float min = 1;
      //find max
      if(r > max) { max = r;}
      if(g > max) { max = g;}
      if(b > max) { max = b;}
      //find min
      if(r < min){min = r;}
      if(g < min){min = g;}
      if(b < min){min = b;}

      //do conversion to find hue
      if(max == min) {hue = 0.0;}
      else if(max == r && g >= b) {hue = 60.0*((g-b)/(max - min)) + 0.0;}
      else if(max == r && g < b) {hue = 60.0*((g-b)/(max - min)) + 360.0;}
      else if(max == g) {hue =  60.0*((b-r)/(max-min))+120.0;}
      else if(max == b) {hue = 60.0*((r-g)/(max-min))+240.0;}


      //for average calculation
      totalHue += hue;
      numTotal++;

      //find saturation
      if(max){s = max;}
      if(max != 0){s = 1 - min/max;}
        //std::cout<<" "<<hue;
      if(hue == orange_hue)//result:get spects here and there
        {
          //(*optr) = (byte)255;  // orange
          //orangeCount++;
        }
      if(hue == pure_orange_hue)//result:nothing
        {
          //(*optr) = (byte)255;  // orange
          //orangeCount++;
        }
      //to reason these numbers 145 is about the value of orange hue
      //pretty good but with spects, "s != 1" gets rid of specs, but also takes out some of the pipe
      //value of 120 to 145 seems best
      //using 130 as min makes it less accurate
      //using a higher max does not seem to make a difference
      //probably because of the colors involved here
      if(!(120 <= hue && hue <= 145) &&
         s != 1)
        {
          //std::cout<<" "<<s;
          (*optr) = (byte)255;  // orange
          orangeCount++;
        }

      //float avg = (avgR+avgG+avgB)/3.0;
      //float sigma = pow(avgR - avg, 2.) + pow(avgG - avg, 2.) + pow(avgB - avg, 2.);
      //float stdDev = sqrt( (1./3.) * sigma );

      //result: pretty good but is confused by highlights
      if (avgR > R - tR && avgR < R + tR &&
          avgG > G - tG && avgG < G + tG &&
          avgB > B - tB && avgB < B + tB   )
        {
          (*optr) = (byte)255;  // orange
          orangeCount++;
        }
      //       else
      //         {
//           //if(outputImage.coordsOk(i,j)){
//           //(*optr) = (byte)0; //not orange
//           //}
//         }

      iptr++; optr++;
    }

        std::cout<<"average hue was "<<totalHue/numTotal<<std::endl;
  return float(orangeCount)/float( (inputImage.getHeight() * inputImage.getWidth()));
}


//true test with frame 3498
float StraightEdgeFinder::isolateOrange4(Image< PixRGB<byte> > &inputImage,  Image<byte> &outputImage)
{

//   XWindow wini(Dims(width, height), 0, 0, "test-input window");
//   XWindow wino(Dims(width/4, height/4), 0, 0, "test-output window 2");
//   XWindow winAux(Dims(500, 450), 0, 0, "Channel levels");
  //  Timer tim;
    Image< PixRGB<byte> > display;

  uint width = itsWidth; uint height = itsHeight;
  Image< PixRGB<byte> > ima = inputImage;
  Image< PixRGB<float> > fima;

  //Image< PixRGB<byte> > display;
  //  uint64 t[NAVG]; unsigned int frame = 0;
  Image<PixH2SV2<float> > H2SVimage;

  /****************************************************************************/
  /* create 2 trackers that are bound together (e.g. 2 trackers in the same
     camera input image
  */
 /*
  //! Mean color to track (ideal color)
  std::vector<float> color(3,0.0F);
  color[0] = 20.0F; color[1] = 0.25F; color[2] = 156.0F;

  //! +/- tollerance value on mean for track
  std::vector<float> std(3,0.0F);
  std[0] = 30.0F; std[1] = 0.30F; std[2] = 60.0F;

  //! normalizer over color values (highest value possible)
  std::vector<float> norm(3,0.0F);
  norm[0] = 360.0F; norm[1] = 1.0F; norm[2] = 255.0F;

  //! how many standard deviations out to adapt, higher means less bias
  std::vector<float> adapt(3,0.0F);
  adapt[0] = 3.5F; adapt[1] = 3.5F; adapt[2] = 3.5F;

  //! highest value for color adaptation possible (hard boundary)
  std::vector<float> upperBound(3,0.0F);
  upperBound[0] = 50.0F; upperBound[1] = 0.4F ; upperBound[2] = 180.0F;

  //! lowest value for color adaptation possible (hard boundary)
  std::vector<float> lowerBound(3,0.0F);
  lowerBound[0] = 12.0F; lowerBound[1] = 0.1F; lowerBound[2] = 120.0F;
*/
  /****************************************************************************/
//   //! Mean color to track (ideal color for red feducial)
//   std::vector<float> color(3,0.0F);
//   color[0] = 10.0F; color[1] = 0.80F; color[2] = 156.0F;

//   //! +/- tollerance value on mean for track
//   std::vector<float> std(3,0.0F);
//   std[0] = 30.0F; std[1] = 0.30F; std[2] = 60.0F;

//   //! normalizer over color values (highest value possible)
//   std::vector<float> norm(3,0.0F);
//   norm[0] = 360.0F; norm[1] = 1.0F; norm[2] = 255.0F;

//   //! how many standard deviations out to adapt, higher means less bias
//   std::vector<float> adapt(3,0.0F);
//   adapt[0] = 3.5F; adapt[1] = 3.5F; adapt[2] = 3.5F;

//   //! highest value for color adaptation possible (hard boundry)
//   std::vector<float> upperBound(3,0.0F);
//   upperBound[0] = 50.0F; upperBound[1] = 1.0F ; upperBound[2] = 255.0F;

//   //! lowest value for color adaptation possible (hard boundry)
//   std::vector<float> lowerBound(3,0.0F);
//   lowerBound[0] = 0.0F; lowerBound[1] = 0.1F; lowerBound[2] = 10.0F;

  /****************************************************************************/
  //! Mean color to track (ideal color for blue feducial)


  std::vector<float> color(3,0.0F);
  //"PINK"
  color[0] = 0.0F; color[1] = 0.88F; color[2] = 180.0F;
  //BLUE
  //color[0] = 250.0F; color[1] = 0.50F; color[2] = 156.0F;

  //! +/- tollerance value on mean for track
  std::vector<float> std(3,0.0F);
  std[0] = 60.0F; std[1] = 0.30F; std[2] = 60.0F;

  //! normalizer over color values (highest value possible)
  std::vector<float> norm(3,0.0F);
  norm[0] = 360.0F; norm[1] = 1.0F; norm[2] = 255.0F;

  //! how many standard deviations out to adapt, higher means less bias
  std::vector<float> adapt(3,0.0F);
  adapt[0] = 3.5F; adapt[1] = 3.5F; adapt[2] = 3.5F;

  //! highest value for color adaptation possible (hard boundry)
  std::vector<float> upperBound(3,0.0F);
  upperBound[0] = 360.0F; upperBound[1] = 1.0F ; upperBound[2] = 255.0F;

  //! lowest value for color adaptation possible (hard boundry)
  std::vector<float> lowerBound(3,0.0F);
  lowerBound[0] = 200.0F; lowerBound[1] = 0.1F; lowerBound[2] = 10.0F;


  /****************************************************************************/
  //! extracted signature

  // signature extracted for Nathan's mahogany shirt

  // H1 - H2 - S - V
//   std::vector<float> color(4,0.0F);
//   color[0] = 0.350962; color[1] = 0.645527; color[2] = 0.313523; color[3] = 0.720654;

//   //! +/- tollerance value on mean for track
//   std::vector<float> std(4,0.0F);
//   std[0] = 0.339556; std[1] = 0.368726; std[2] = 0.609608; std[3] = 0.34012;

//   //! normalizer over color values (highest value possible)
//   std::vector<float> norm(4,0.0F);
//   norm[0] = 1.0F; norm[1] = 1.0F; norm[2] = 1.0F; norm[3] = 1.0F;

//   //! how many standard deviations out to adapt, higher means less bias
//   std::vector<float> adapt(4,0.0F);
//   adapt[0] = 3.5F; adapt[1] = 3.5F; adapt[2] = 3.5F; adapt[3] = 3.5F;

//   //! highest value for color adaptation possible (hard boundary)
//   std::vector<float> upperBound(4,0.0F);
//   upperBound[0] = color[0] + 0.45F; upperBound[1] = color[1] + 0.45F;
//   upperBound[2] = color[2] + 0.55F; upperBound[3] = color[3] + 0.55F;

//   //! lowest value for color adaptation possible (hard boundary)
//   std::vector<float> lowerBound(4,0.0F);
//   lowerBound[0] = color[0] - 0.45F; lowerBound[1] = color[1] - 0.45F;
//   lowerBound[2] = color[2] - 0.55F; lowerBound[3] = color[3] - 0.55F;


  //int zero = 0;
  int wi = width/4;
  int hi = height/4;

  segmentImageTrackMC<float,unsigned int, 4> segmenter(wi*hi);

  segmenter.SITsetTrackColor(&color,&std,&norm,&adapt,&upperBound,&lowerBound);

  /* This limits the area of consideration to an area smaller than
     the image size. That is, it creates a boundery in the image
     outside of which it will not consider pixes (i.e. a frame)
  */
  segmenter.SITsetFrame(&wi,&hi);


  /* Set display colors for output of tracking. Strictly asthetic */
  segmenter.SITsetCircleColor(255,255,0);
  segmenter.SITsetBoxColor(255,255,0,0,255,255);
  segmenter.SITsetUseSmoothing(true,10);
  //unsigned long counter = 0;

    Image<PixRGB<byte> > Aux;
    Aux.resize(100,450,true);

    /* Take in the image and color segment it */
    H2SVimage = ima;
    display = ima;

    /******************************************************************/
    // SEGMENT IMAGE ON EACH INPUT FRAME

    segmenter.SITtrackImageAny(H2SVimage,&display,&Aux,true);

    /* Retrieve and Draw all our output images */
    Image<byte> temp = segmenter.SITreturnCandidateImage();


    outputImage = temp;

    inplacePaste(itsDispImg, toRGB(temp), Point2D<int>(itsWidth,itsHeight));

    //wini.drawImage(display);
    //wino.drawImage(temp);
    //winAux.drawImage(Aux);

    /******************************************************************/
    // Uncomment these lines to write each frame to the hard drive
    /******************************************************************/
    /*
    LINFO("COUNT %d",counter);
    // TRACKER DISPLAY
    Raster::WriteRGB(display,sformat("out.display.%d.ppm",counter));
    // BOOL CANDIDATE MAP
    Raster::WriteRGB(temp,sformat("out.temp.%d.ppm",counter));
    // ADAPTIVE THRESHOLDING BARS
    Raster::WriteRGB(Aux,sformat("out.Aux.%d.ppm",counter));
    // BLOB ID MAP
    Image<byte> blobs = segmenter.SITreturnBlobMap();
    inplaceNormalize(blobs, 0,255);
    Raster::WriteRGB(blobs,sformat("out.Blobs.%d.ppm",counter));
    counter++;
    */
    /******************************************************************/

    return 0.0;
}


float StraightEdgeFinder::isolateOrange5
(Image< PixRGB<byte> > &inputImage,  Image<byte> &outputImage)
{
  // variables for segmenting and tracking
  int width = itsWidth, height = itsHeight;
  //  float delay = 0;
  float H,S,V,Hs,Ss,Vs;
  float LOTcount = 0;

  // timer initialization
  Image< PixRGB<byte> > ima = inputImage;
  Image< PixRGB<float> > fima;

  // configure segmenter and tracker
  segmentImage segment(HSV);
  segmentImageTrack track(1000, &segment);
  H = 10; Hs = 200;
  S = .70; Ss = .20;
  V = 150; Vs = 250;
  segment.setHue(H,Hs,0);
  segment.setSat(S,Ss,0);
  segment.setVal(V,Vs,0);
  segment.setHSVavg(15);
  segment.setFrame(0,0,width/4,height/4,width/4,height/4);

  // decimate image to 1/4 size
  fima = decXY(ima);
  fima = decXY(fima);

  // segment image
  segment.segment(fima);
  Image<byte> outputI = segment.returnNormalizedCandidates();
  segment.calcMassCenter();
  track.track(0);

  for(int i = 0; i < segment.numberBlobs(); i++)
    {
      if(track.isCandidate(i) == true)
        {
          segment.getHSVvalueMean(i,&H,&S,&V,&Hs,&Ss,&Vs);
//           int tt = segment.getYmin(i); int bb = segment.getYmax(i);
//           int ll = segment.getXmin(i); int rr = segment.getXmax(i);
//           if((bb != tt) && (ll != rr))
//             drawRect(ima, Rectangle::tlbrI(tt*4,ll*4,bb*4,rr*4),
//                      PixRGB<byte>(255,255,0),1);
//           drawCircle(ima, Point2D<int>((int)segment.getCenterX(i)*4
//                                        ,(int)segment.getCenterY(i)*4)
//                      ,(int)sqrt((double)segment.getMass(i)),
//                      PixRGB<byte>(0,0,255),2);
//           drawCircle(ima, Point2D<int>((int)segment.getCenterX(i)*4
//                                        ,(int)segment.getCenterY(i)*4)
//                      ,2,PixRGB<byte>(255,0,0),2);
      }
      if(track.returnLOT() == true)
      {
        if(LOTcount == 2)
        {
          H = 200; Hs = 20;
          S = .70; Ss = .20;
          V = 150; Vs = 150;
          LOTcount = 0;
        }
        else
        {
          LOTcount++;
        }
      }
      segment.setHue(H,(Hs*3),0);
      segment.setSat(S,(Ss*3),0);
      segment.setVal(V,(Vs*3),0);
    }
//     drawCircle(ima, Point2D<int>((int)track.getObjectX()*4
//                             ,(int)track.getObjectY()*4)
//                ,2,PixRGB<byte>(0,255,0));

//     if(camPause.get() > delay)
//     {
//       LINFO( "Object mass: %d", track.getMass() );
//       int modi = (int)track.getObjectX()*8;
//       int modj = 480-((int)track.getObjectY()*8);
//       if(modi > 0 && modi < 640 && modj > 0 && modj < 480)
//       {
//         if(!track.returnLOT() &&
//            track.getMass() < 2400 && track.getMass() > 30 )
//         {
//           /* // send speed and steer command to Board B
//           if( car->getSpeed() < 0.18 )
//             car->setSpeed( car->getSpeed() + 0.01 );
//           car->setSteering( 1.0f * 1/320 * ( modi - 320 ) );
//           */
//           LINFO( "Steering to %f", 1.0f * 1/320 * ( modi - 320 ) );
//         }
//         else
//           {
//             /* // send speed and steer command to Board B
//             car->setSpeed( 0.0 );
//             car->setSteering( 0.0 );
//             */
//             LINFO("Loss of Track, stopping");
//           }
//       }
//     }

//     // display segment image if option was specified
//     ofs->writeRGB(ima, "input");
//     ofs->writeGray(outputI, "normalizedCandidates");


  outputImage = outputI;

  itsDispImg.clear();
  //  inplacePaste(itsDispImg, inputImage, Point2D<int>(itsWidth,0));
    inplacePaste(itsDispImg, toRGB(outputI), Point2D<int>(itsWidth,itsHeight));


    return 1.0;
}


float StraightEdgeFinder::normalizeAngle(float angleInRads) {
  float angleInDegs;

  angleInDegs = angleInRads * 180 / M_PI;
  if (angleInDegs < 0) angleInDegs += 90;
  else angleInDegs -= 90;

  return angleInDegs;
}


#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

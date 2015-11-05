#include "Robots/SeaBeeIII/VisionRectangle.H"

#include "Component/ModelParam.H"
#include "Component/ModelOptionDef.H"
#include "Image/OpenCVUtil.H"
#include "Raster/Raster.H"
#include "MBARI/Geometry2D.H"
#include "Image/Pixels.H"

#include "Image/ColorOps.H"


#include "Media/MediaOpts.H"


#ifndef VISIONRECTANGLE_C
#define VISIONRECTANGLE_C

using namespace std;

#define MIN_CENTER_DIST  15
#define MIN_AREA         150
#define CORNER_TOLERANCE 4

int thresh = 42;
double angle_thresh = 0.3;


// ######################################################################
VisionRectangle::VisionRectangle
( OptionManager& mgr,
  const std::string& descrName,
  const std::string& tagName) :
  VisionBrainComponentI(mgr, descrName, tagName)
{

  itsWidth  = 320;
  itsHeight = 240;
  //  itsMwin.reset
  //    (new XWinManaged(Dims(2*itsWidth,2*itsHeight), 0, 0, "Master window"));
  itsDisp.resize(2*itsWidth, 2*itsHeight);
}

// ######################################################################
VisionRectangle::~VisionRectangle()
{
}

// ######################################################################
void VisionRectangle::registerTopics()
{
  LINFO("Registering VisionRectangle Message");
  this->registerPublisher("VisionRectangleMessageTopic");
  registerVisionTopics();
}


// ######################################################################
// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
double VisionRectangle::angle( CvPoint* pt1, CvPoint* pt2, CvPoint* pt0 ) {
  double dx1 = pt1->x - pt0->x;
  double dy1 = pt1->y - pt0->y;
  double dx2 = pt2->x - pt0->x;
  double dy2 = pt2->y - pt0->y;
  return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}


// ######################################################################
// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
CvSeq* VisionRectangle::findSquares4( IplImage* img, CvMemStorage* storage ) {
  CvSeq* contours;
  int i, c, l, N = 11;

  CvSize sz = cvSize( img->width & -2, img->height & -2 );

  IplImage* timg = cvCloneImage( img ); // make a copy of input image
  IplImage* gray = cvCreateImage( sz, 8, 1 );
  IplImage* pyr = cvCreateImage( cvSize(sz.width/2, sz.height/2), 8, 3 );
  IplImage* tgray;

  CvSeq* result;

  double s, t;
  // create empty sequence that will contain points -
  // 4 points per square (the square's vertices)
  CvSeq* squares = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvPoint), storage );

  double img_area = sz.width * sz.height;
  double max_area = img_area * 0.5;

  // select the maximum ROI in the image
  // with the width and height divisible by 2
  //cvSetImageROI( timg, cvRect( max_roi_x1, max_roi_y1, max_roi_w, max_roi_h ));
  cvSetImageROI( timg, cvRect( 0, 0, sz.width, sz.height ));

  // down-scale and upscale the image to filter out the noise
  cvPyrDown( timg, pyr, 7 );
  cvPyrUp( pyr, timg, 7 );
  tgray = cvCreateImage( sz, 8, 1 );

  // TAKE OUT LATER
  itsDisp.clear();
  inplacePaste(itsDisp, ipl2rgb(img), Point2D<int>(0, 0));

  // find squares in every color plane of the image
  for( c = 0; c < 3; c++ ) {
    // extract the c-th color plane
    cvSetImageCOI( timg, c+1 );
    cvCopy( timg, tgray, 0 );

    // try several threshold levels
    for( l = 0; l < N; l++ ) {
      // hack: use Canny instead of zero threshold level.
      // Canny helps to catch squares with gradient shading
      if( l == 0 ) {
        // apply Canny. Take the upper threshold from slider
        // and set the lower to 0 (which forces edges merging)
        cvCanny( tgray, gray, 0, thresh, 5 );



//         inplacePaste(itsDisp, Image<PixRGB<byte> >(toRGB(ipl2gray(gray))), Point2D<int>(itsWidth, 0));
//         inplacePaste(itsDisp, Image<PixRGB<byte> >(toRGB(ipl2gray(tgray))), Point2D<int>(0, itsHeight));
//         LINFO("cvC [color: %d, level: %d]", c, l);
//         itsMwin->drawImage(itsDisp,0,0);
//         Raster::waitForKey();



        // dilate canny output to remove potential
        // holes between edge segments
        cvDilate( gray, gray, 0, 1 );


//         inplacePaste(itsDisp, Image<PixRGB<byte> >(toRGB(ipl2gray(gray))), Point2D<int>(itsWidth, 0));
//         inplacePaste(itsDisp, Image<PixRGB<byte> >(toRGB(ipl2gray(tgray))), Point2D<int>(0, itsHeight));
//         LINFO("cvD [color: %d, level: %d]", c, l);
//         itsMwin->drawImage(itsDisp,0,0);
//         Raster::waitForKey();


      } else {
        // apply threshold if l!=0:
        // tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
        cvThreshold( tgray, gray, (l+1)*255/N, 255, CV_THRESH_BINARY );

//         inplacePaste(itsDisp, Image<PixRGB<byte> >(toRGB(ipl2gray(gray))), Point2D<int>(itsWidth, 0));
//         inplacePaste(itsDisp, Image<PixRGB<byte> >(toRGB(ipl2gray(tgray))), Point2D<int>(0, itsHeight));
//         LINFO("cvT [color: %d, level: %d]", c, l);
//         itsMwin->drawImage(itsDisp,0,0);
//         Raster::waitForKey();
      }

      // find contours and store them all as a list
      cvFindContours( gray, storage, &contours, sizeof(CvContour),
                      CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );

      // test each contour
      uint count =0;
      while( contours ) {

        // approximate contour with accuracy proportional
        // to the contour perimeter
        result = cvApproxPoly( contours, sizeof(CvContour), storage,
                               CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.02, 0 );

        // square contours should have 4 vertices after approximation
        // relatively large area (to filter out noisy contours)
        // and be convex.
        // Note: absolute value of an area is used because
        // area may be positive or negative - in accordance with the
        // contour orientation

        //also going to check that the area is smaller than max_area
        double rect_area = fabs(cvContourArea(result,CV_WHOLE_SEQ));

//         if (
//             result->total == 4
//             &&
//             rect_area > 100
//             && rect_area < max_area
//             //            && cvCheckContourConvexity(result)
//             )
//           {

//             Image<PixRGB<byte> > conImg = ipl2rgb(img);
//             Point2D<int> pt1;             Point2D<int> pt2;
//             for(int pti=0; pti < result->total-1; pti++)
//               {
//                 pt1.i = ((CvPoint*)cvGetSeqElem(result, pti))->x;
//                 pt1.j = ((CvPoint*)cvGetSeqElem(result, pti))->y;
//                 LINFO("[%d,%d]", pt1.i, pt1.j);

//                 pt2.i = ((CvPoint*)cvGetSeqElem(result, pti+1))->x;
//                 pt2.j = ((CvPoint*)cvGetSeqElem(result, pti+1))->y;

//                 // draw the quad on the image
//                 drawLine(conImg, pt1, pt2, PixRGB<byte>(255,0,0),1);
//               }
//             LINFO("[%d,%d]", pt2.i, pt2.j);
//             pt1.i = ((CvPoint*)cvGetSeqElem(result, 0))->x;
//             pt1.j = ((CvPoint*)cvGetSeqElem(result, 0))->y;

//             // draw the quad on the image
//             drawLine(conImg,pt2, pt1, PixRGB<byte>(255,0,0),1);

//             inplacePaste(itsDisp, conImg, Point2D<int>(itsWidth, itsHeight));
//             LINFO("con [%d] [color: %d, level: %d]", count, c, l);
//             LINFO("[total: %d: area: %f  conv: %d inCorner: %d",
//                   result->total, rect_area, cvCheckContourConvexity(result), inCorner(result));
//             itsMwin->drawImage(itsDisp,0,0);
//             Raster::waitForKey();
//           }

        if (
            result->total == 4
            && rect_area > MIN_AREA
            && rect_area < max_area
            && cvCheckContourConvexity(result)
            && !inCorner(result)
            )
          {
            s = 0;

            for( i = 0; i < 5; i++ ) {
              // find minimum angle between joint
              // edges (maximum of cosine)
              if( i >= 2 ) {
                t = fabs(angle(
                               (CvPoint*)cvGetSeqElem( result, i ),
                               (CvPoint*)cvGetSeqElem( result, i-2 ),
                             (CvPoint*)cvGetSeqElem( result, i-1 )));
                s = s > t ? s : t;
              }
            }

            // if cosines of all angles are small
            // (all angles are ~90 degree) then write quandrange
            // vertices to resultant sequence
            if( s < angle_thresh)
              {
                //LINFO("RECTANGLE BABY");
                for( i = 0; i < 4; i++ )
                  cvSeqPush( squares, (CvPoint*)cvGetSeqElem( result, i ));
              }
          }

        // take the next contour
        contours = contours->h_next;
        count++;
      }
//       LINFO("Bot[#contours: %d] [color: %d, level: %d]", count, c, l);
//       itsMwin->drawImage(itsDisp,0,0);
//       Raster::waitForKey();
    }
  }


  // release all the temporary images
  cvReleaseImage( &gray );
  cvReleaseImage( &pyr );
  cvReleaseImage( &tgray );
  cvReleaseImage( &timg );

  return squares;
}

// ######################################################################
void VisionRectangle::updateFrame(Image<PixRGB<byte> > img, std::string cameraId)
{
        bool isFwdCamera = false;
        if(cameraId == "FwdCamera")
                isFwdCamera = true;

  LINFO("Image Received: %d", itsFrameCount);

  if(img.initialized())
    {

      CvMemStorage* storage = cvCreateMemStorage(0);
      IplImage* img0 = img2ipl(img);

      CvSeq *cvsquares = findSquares4( img0, storage );

      cvReleaseImage( &img0 );

      RobotSimEvents::QuadrilateralIceVector quadVect;
      ImageIceMod::QuadrilateralIce quad;
      Point2D<int> avgCenter(0,0);
      std::multimap<int,Point2D<int> > tempPoints;

      // iterate over all the quadrilateral points found
      for(int i=0; i < cvsquares->total; i++) {

        // get an individual point
        Point2D<int> quadPoint;
        quadPoint.i = ((CvPoint*)cvGetSeqElem(cvsquares, i))->x;
        quadPoint.j = ((CvPoint*)cvGetSeqElem(cvsquares, i))->y;

        // add current point's position to running average of point positions
        avgCenter += Point2D<int>(quadPoint.i,quadPoint.j);

        // add the point to map, sorted by point Y-axis position
        tempPoints.insert(make_pair(quadPoint.j,quadPoint));

        //LINFO("tempPoints size: %d\n",tempPoints.size());

        // if we have added the 4th point on the quadrilateral
        if (tempPoints.size() == 4)
          {
            std::vector<Point2D<int> > tempVec;

            for(std::map<int,Point2D<int > >::const_iterator it = tempPoints.begin();
                it != tempPoints.end(); ++it)
              {
                tempVec.push_back(it->second);
              }


            // compare first two points to determine which is top left
            // and which is top right
            if(tempVec[0].i < tempVec[1].i)
              {
                quad.tl.i = tempVec[0].i;
                quad.tr.i = tempVec[1].i;

                quad.tl.j = tempVec[0].j;
                quad.tr.j = tempVec[1].j;
              }
            else
              {
                quad.tr.i = tempVec[0].i;
                quad.tl.i = tempVec[1].i;

                quad.tr.j = tempVec[0].j;
                quad.tl.j = tempVec[1].j;
              }

            // compare second two points to determine bottom left and
            // bottom right
            if(tempVec[2].i < tempVec[3].i)
              {
                quad.bl.i = tempVec[2].i;
                quad.br.i = tempVec[3].i;

                quad.bl.j = tempVec[2].j;
                quad.br.j = tempVec[3].j;
              }
            else
              {
                quad.br.i = tempVec[2].i;
                quad.bl.i = tempVec[3].i;

                quad.br.j = tempVec[2].j;
                quad.bl.j = tempVec[3].j;
              }


            // divide by total number of averaged points
            // to get current quad's center position
            avgCenter /= Point2D<int>(4,4);
            quad.center.i = avgCenter.i;
            quad.center.j = avgCenter.j;


            bool isDupe = false;

            // make sure the quad's center is not too close
            // to a prev. quad's center in order to avoid duplicates
            for(uint j = 0; j < quadVect.size(); j++)
              {
                if(avgCenter.distance(Point2D<int>(quadVect[j].center.i,quadVect[j].center.j))
                   < MIN_CENTER_DIST)
                  {
                    isDupe = true;
                  }
              }

            // not dupe so add it to vector
            if(!isDupe)
              {
                LineSegment2D vertLine = LineSegment2D((Point2D<int>(quad.tr.i,quad.tr.j) +
                                                        Point2D<int>(quad.tl.i,quad.tl.j))/2,
                                                       (Point2D<int>(quad.br.i,quad.br.j) +
                                                        Point2D<int>(quad.bl.i,quad.bl.j))/2);

                LineSegment2D horizLine = LineSegment2D((Point2D<int>(quad.tl.i,quad.tl.j) +
                                                         Point2D<int>(quad.bl.i,quad.bl.j))/2,
                                                        (Point2D<int>(quad.tr.i,quad.tr.j) +
                                                         Point2D<int>(quad.br.i,quad.br.j))/2);
                float ratio = 0.0;
                float angle = 0.0;
                if(vertLine.length() > horizLine.length())
                  {
                    if(horizLine.length() > 0)
                      ratio = vertLine.length() / horizLine.length();

                    angle = vertLine.angle();
                  }
                else
                  {
                    if(vertLine.length() > 0)
                      ratio = horizLine.length() / vertLine.length();

                    angle = horizLine.angle();
                  }


                // change angle to degrees
                 angle = angle * (180/M_PI);

                // normalize angle so that zero degrees is facing forawrd
                // turning to the right is [0 -> 90]
                // turning to the left is [0 -> -90]
                if(angle < 0)
                  angle += 90;
                else
                  angle += -90;

                quad.ratio = ratio;
                quad.angle = angle;
                quadVect.push_back(quad);

                // draw the quad on the image
                drawLine(img,Point2D<int>(quad.tr.i,quad.tr.j),
                         Point2D<int>(quad.br.i,quad.br.j),
                         PixRGB<byte>(0,255,0),2);
                drawLine(img,Point2D<int>(quad.br.i,quad.br.j),
                         Point2D<int>(quad.bl.i,quad.bl.j),
                         PixRGB<byte>(0,255,0),2);
                drawLine(img,Point2D<int>(quad.bl.i,quad.bl.j),
                         Point2D<int>(quad.tl.i,quad.tl.j),
                         PixRGB<byte>(0,255,0),2);
                drawLine(img,Point2D<int>(quad.tl.i,quad.tl.j),
                         Point2D<int>(quad.tr.i,quad.tr.j),
                         PixRGB<byte>(0,255,0),2);

                char* str = new char[20];
                sprintf(str,"%1.2f, %2.1f",ratio,angle);
                 writeText(img,Point2D<int>(quad.center.i,quad.center.j),str);
                delete [] str;

              }

            // re-initialize for next quad
            quad = ImageIceMod::QuadrilateralIce();
            avgCenter = Point2D<int>(0,0);
            tempPoints.clear();

          }
      }

      itsOfs->writeRGB(img, "Vision Rectangle Image",
                       FrameInfo("Vision Rectangle Image", SRC_POS));

      itsOfs->updateNext();

      if (quadVect.size() > 0) {
        RobotSimEvents::VisionRectangleMessagePtr msg = new RobotSimEvents::VisionRectangleMessage;
        msg->quads = quadVect;
        msg->isFwdCamera = isFwdCamera;
        this->publish("VisionRectangleMessageTopic", msg);
      }

      cvReleaseMemStorage( &storage );
    }
}


bool VisionRectangle::inCorner(CvSeq* result)
{

  for(int pti=0; pti < result->total; pti++)
    {
      Point2D<uint> pt;
      pt.i = ((CvPoint*)cvGetSeqElem(result, pti))->x;
      pt.j = ((CvPoint*)cvGetSeqElem(result, pti))->y;

      //edge contours are inset by 1
      uint right_edge = itsWidth - 2;
      uint bottom_edge = itsHeight - 2;

      if((pt.i <= CORNER_TOLERANCE && pt.j <= CORNER_TOLERANCE) ||
         (pt.i >= right_edge-CORNER_TOLERANCE && pt.j <= CORNER_TOLERANCE) ||
         (pt.i <= CORNER_TOLERANCE && pt.j >= bottom_edge-CORNER_TOLERANCE) ||
         (pt.i >= right_edge-CORNER_TOLERANCE && pt.j >= bottom_edge-CORNER_TOLERANCE)
         )
        return true;
    }

  return false;
}

#endif

/*!@file Devices/HeliPose.C read HeliPose data  */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/HeliBot/HeliPose.C $
// $Id: HeliPose.C 12962 2010-03-06 02:13:53Z irock $
//

#include "Robots/HeliBot/HeliPose.H"

#include "Component/ParamMap.H"
#include "Util/Assert.H"
#include "Util/Types.H"
#include "Util/log.H"
#include "Image/ColorOps.H"
#include "Image/DrawOps.H"

#include <cmath>
#include <unistd.h>

#ifdef HAVE_OPENCV
namespace
{
  const int thresh = 50;

  // helper function:
  // finds a cosine of angle between vectors
  // from pt0->pt1 and from pt0->pt2
  double angle( CvPoint* pt1, CvPoint* pt2, CvPoint* pt0 )
  {
    double dx1 = pt1->x - pt0->x;
    double dy1 = pt1->y - pt0->y;
    double dx2 = pt2->x - pt0->x;
    double dy2 = pt2->y - pt0->y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
  }

  CvSeq* findSquares(const Image<PixRGB<byte> >& in, CvMemStorage* storage,
                  const int minarea, const int maxarea, const double mincos)
  {
    const int N = 11;

    IplImage* img = img2ipl(in);

    CvSize sz = cvSize( img->width & -2, img->height & -2 );
    IplImage* timg = cvCloneImage( img ); // make a copy of input image
    IplImage* gray = cvCreateImage( sz, 8, 1 );
    IplImage* pyr = cvCreateImage( cvSize(sz.width/2, sz.height/2), 8, 3 );
    // create empty sequence that will contain points -
    // 4 points per square (the square's vertices)
    CvSeq* squares = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvPoint), storage );

    // select the maximum ROI in the image
    // with the width and height divisible by 2
    cvSetImageROI( timg, cvRect( 0, 0, sz.width, sz.height ));

    // down-scale and upscale the image to filter out the noise
    cvPyrDown( timg, pyr, 7 );
    cvPyrUp( pyr, timg, 7 );
    IplImage* tgray = cvCreateImage( sz, 8, 1 );

    // find squares in every color plane of the image
    for (int c = 0; c < 3; ++c)
      {
        // extract the c-th color plane
        cvSetImageCOI( timg, c+1 );
        cvCopy( timg, tgray, 0 );

        // try several threshold levels
        for (int l = 0; l < N; ++l)
          {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
              {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                cvCanny( tgray, gray, 0, thresh, 5 );
                // dilate canny output to remove potential
                // holes between edge segments
                cvDilate( gray, gray, 0, 1 );
              }
            else
              {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                cvThreshold( tgray, gray, (l+1)*255/N, 255, CV_THRESH_BINARY );
              }

            // find contours and store them all as a list
            CvSeq* contours = 0;
            cvFindContours( gray, storage, &contours, sizeof(CvContour),
                            CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );

            // test each contour
            while( contours )
              {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                CvSeq* result =
                  cvApproxPoly( contours, sizeof(CvContour), storage,
                                CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.02, 0 );
                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                const double area = fabs(cvContourArea(result,CV_WHOLE_SEQ));
                if (result->total == 4 &&
                    area >= minarea && area <= maxarea &&
                    cvCheckContourConvexity(result))
                  {
                    double s = 0;

                    for (int i = 0; i < 4; ++i)
                      {
                        // find minimum angle between joint
                        // edges (maximum of cosine)
                        const double t =
                          fabs(angle((CvPoint*)cvGetSeqElem( result, i % 4 ),
                                     (CvPoint*)cvGetSeqElem( result, (i-2) % 4 ),
                                     (CvPoint*)cvGetSeqElem( result, (i-1) % 4 )));
                        s = s > t ? s : t;
                      }


                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrangle
                    // vertices to resultant sequence
                    if (s < mincos)
                    {
                      for (int i = 0; i < 4; ++i)
                        cvSeqPush(squares,
                                  (CvPoint*)cvGetSeqElem( result, i ));
                      //LINFO("area=%f, mincos=%f", area, s);
                    }
                  }

                // take the next contour
                contours = contours->h_next;
              }
          }
      }

    // release all the temporary images
    cvReleaseImage( &gray );
    cvReleaseImage( &pyr );
    cvReleaseImage( &tgray );
    cvReleaseImage( &timg );
    cvReleaseImageHeader( &img );

    return squares;
  }

  // the function draws all the squares in the image
  Image<PixRGB<byte> > drawSquares(const Image<PixRGB<byte> >&in, CvSeq* squares)
  {
    Image<PixRGB<byte> > out(in);

    CvSeqReader reader;

    // initialize reader of the sequence
    cvStartReadSeq(squares, &reader, 0);

    // read 4 sequence elements at a time (all vertices of a square)
    for (int i = 0; i < squares->total; i += 4)
      {
        CvPoint pt[4];

        // read 4 vertices
        CV_READ_SEQ_ELEM( pt[0], reader );
        CV_READ_SEQ_ELEM( pt[1], reader );
        CV_READ_SEQ_ELEM( pt[2], reader );
        CV_READ_SEQ_ELEM( pt[3], reader );

        for (int j = 0; j < 4; ++j)
          drawLine(out,
                   Point2D<int>(pt[j].x, pt[j].y),
                   Point2D<int>(pt[(j+1)%4].x, pt[(j+1)%4].y),
                   PixRGB<byte>(0, 255, 0),
                   2);
      }

    return out;
  }

  std::vector<Point2D<int> > getRectangle(CvSeq* cards)
  {
    CvSeqReader reader;
    // initialize reader of the sequence
    cvStartReadSeq( cards, &reader, 0 );

    std::vector<Point2D<int> > corners;

    if (cards->total > 0)
    {
      CvPoint pt[4];
      // read 4 vertices
      CV_READ_SEQ_ELEM( pt[0], reader );
      CV_READ_SEQ_ELEM( pt[1], reader );
      CV_READ_SEQ_ELEM( pt[2], reader );
      CV_READ_SEQ_ELEM( pt[3], reader );

      corners.push_back(Point2D<int>(pt[0].x, pt[0].y));
      corners.push_back(Point2D<int>(pt[1].x, pt[1].y));
      corners.push_back(Point2D<int>(pt[2].x, pt[2].y));
      corners.push_back(Point2D<int>(pt[3].x, pt[3].y));

    }

    return corners;
  }
}

#endif //HAVE_OPENCV


void* HeliPose_run(void *r0); // will live in a separate thread

// ######################################################################
void* HeliPose_run(void *r0)
{
  HeliPose *r = (HeliPose *)r0;
  //r->run_cameraPose(); return NULL;
  r->run_imuPose(); return NULL;
}

// ######################################################################
HeliPose::HeliPose(OptionManager& mgr,
    nub::ref<InputFrameSeries>& ifs,
    nub::ref<OutputFrameSeries>& ofs,
    const std::string& descrName,
    const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsIfs(ifs),
  itsOfs(ofs),
  itsIMU(new IMU_SFE_Atomic(mgr))
{
  addSubComponent(itsIMU);

  running = false;
  itsDebug = true;

  pthread_mutex_init(&itsPoseLock, NULL);

  //pthread_mutex_init(&itsImgLock, NULL);

  //itsStorage = cvCreateMemStorage(0);
  //itsIntrinsicMatrix = cvCreateMat( 3, 3, CV_32FC1);
  //itsDistortionCoeffs = cvCreateMat( 4, 1, CV_32FC1);
  //itsCameraRotation = cvCreateMat( 1, 3, CV_64FC1);
  //itsCameraTranslation = cvCreateMat( 1, 3, CV_64FC1);


  //cvmSet(itsDistortionCoeffs, 0, 0, -0.1033503  );
  //cvmSet(itsDistortionCoeffs, 1, 0, 0.0095519 );
  //cvmSet(itsDistortionCoeffs, 2, 0, -0.0163438  );
  //cvmSet(itsDistortionCoeffs, 3, 0, -0.0204184  );

  //cvmSet(itsIntrinsicMatrix, 0, 0, 64.10168); cvmSet(itsIntrinsicMatrix, 0, 1, 0); cvmSet(itsIntrinsicMatrix, 0, 2, 159.50000);
  //cvmSet(itsIntrinsicMatrix, 1, 0, 0); cvmSet(itsIntrinsicMatrix, 1, 1, 71.64519 ); cvmSet(itsIntrinsicMatrix, 1, 2, 119.5);
  //cvmSet(itsIntrinsicMatrix, 2, 0, 0); cvmSet(itsIntrinsicMatrix, 2, 1, 0); cvmSet(itsIntrinsicMatrix, 2, 2, 1);

  //cvmSet(itsDistortionCoeffs, 0, 0, -0.4240642  );
  //cvmSet(itsDistortionCoeffs, 1, 0, -1.9482374 );
  //cvmSet(itsDistortionCoeffs, 2, 0, -0.0267558  );
  //cvmSet(itsDistortionCoeffs, 3, 0, -0.0132413  );

  //cvmSet(itsIntrinsicMatrix, 0, 0, 841.03479); cvmSet(itsIntrinsicMatrix, 0, 1, 0); cvmSet(itsIntrinsicMatrix, 0, 2, 159.50000);
  //cvmSet(itsIntrinsicMatrix, 1, 0, 0); cvmSet(itsIntrinsicMatrix, 1, 1, 861.99316 ); cvmSet(itsIntrinsicMatrix, 1, 2, 119.5);
  //cvmSet(itsIntrinsicMatrix, 2, 0, 0); cvmSet(itsIntrinsicMatrix, 2, 1, 0); cvmSet(itsIntrinsicMatrix, 2, 2, 1);

  itsCurrentPose.translation = Point3D<float>(0,0,0);
  itsCurrentPose.velocity = Point3D<float>(0,0,0);
  itsCurrentPose.rotation = Point3D<float>(0,0,0);

  itsVelocityScale.x = 0.1;
  itsVelocityScale.y = 1;
  itsVelocityScale.z = 1;
  itsRotationScale.x = 1;
  itsRotationScale.y = 1;
  itsRotationScale.z = M_PI/((17870-12360)*2); //units per 90 degrees

  itsVelocityBias = Point3D<float>(0,0,0);
  itsRotationBias = Point3D<float>(0,0,0);
  itsVelocitySigma = Point3D<float>(0,0,0);
  itsVelocitySigma = Point3D<float>(0,0,0);


}

// ######################################################################
void HeliPose::start1()
{
}

// ######################################################################
void HeliPose::start2()
{
  // start thread for run():
  pthread_create(&runner, NULL, &HeliPose_run, (void *)this);
}

// ######################################################################
void HeliPose::stop1()
{
  // stop our thread:
  running = false; while(running == false) usleep(5);
  usleep(50); running = false;
}

// ######################################################################
HeliPose::~HeliPose()
{
  cvReleaseMemStorage(&itsStorage);
  pthread_mutex_destroy(&itsPoseLock);
  //pthread_mutex_destroy(&itsImgLock);
}

// ######################################################################
std::vector<Point2D<int> > HeliPose::getExtrinsic(Image<byte>& img)
{
  int rows = 3;
  int cols = 4;
  std::vector<CvPoint2D32f> corners(rows*cols);

  int count = 0;
  int result = cvFindChessboardCorners(img2ipl(img),
      cvSize(rows,cols),
      &corners[0], &count,
      CV_CALIB_CB_ADAPTIVE_THRESH |
      CV_CALIB_CB_NORMALIZE_IMAGE |
      CV_CALIB_CB_FILTER_QUADS);

  // result = 0 if not all corners were found
  // Find corners to an accuracy of 0.1 pixel
  if(result != 0)
  {
    cvFindCornerSubPix(img2ipl(img),
        &corners[0],
        count,
        cvSize(10,10), //win
        cvSize(-1,-1), //zero_zone
        cvTermCriteria(CV_TERMCRIT_ITER,1000,0.01) );
    //cvDrawChessboardCorners(img2ipl(img), cvSize(rows,cols), &corners[0], count, result);

    //Get
    CvMat *image_points_ex = cvCreateMat( corners.size(), 2, CV_64FC1);

    for (uint j = 0; j < corners.size(); j++){
      cvSetReal2D( image_points_ex, j, 0, corners[j].x);
      cvSetReal2D( image_points_ex, j, 1, corners[j].y);
    }

    CvMat *object_points_ex = cvCreateMat( corners.size(), 3, CV_64FC1);
    for (uint j = 0; j < corners.size(); j++){
      cvSetReal2D( object_points_ex, j, 0, ( j % rows) * 50 ); //0.4m
      cvSetReal2D( object_points_ex, j, 1, ( j / rows) * 50 );
      cvSetReal2D( object_points_ex, j, 2, 0.0 );
    }

    cvFindExtrinsicCameraParams2( object_points_ex,
        image_points_ex,
        itsIntrinsicMatrix,
        itsDistortionCoeffs,
        itsCameraRotation,
        itsCameraTranslation);

    cvReleaseMat( &image_points_ex);
    cvReleaseMat( &object_points_ex);
    //return corners;
  }

  //No corners found
  return std::vector<Point2D<int> >();
}

// ######################################################################
std::vector<Point2D<int> > HeliPose::getExtrinsic(Image<PixRGB<byte> >& img)
{
  CvSeq* square = findSquares(itsCurrentImg, itsStorage,
      1500, //minArea
      15000, //maxArea
      0.3 //minCos
      );
  std::vector<Point2D<int> > corners = getRectangle(square);
  if (corners.size() > 0)
  {

    CvMat *image_points_ex = cvCreateMat( corners.size(), 2, CV_64FC1);

    for (uint j = 0; j < corners.size(); j++){
      cvSetReal2D( image_points_ex, j, 0, (float)corners[j].i);
      cvSetReal2D( image_points_ex, j, 1, (float)corners[j].j);
    }

    CvMat *object_points_ex = cvCreateMat( corners.size(), 3, CV_64FC1);

    cvSetReal2D( object_points_ex, 0, 0, 0 ); //0.4m
    cvSetReal2D( object_points_ex, 0, 1, 0 );
    cvSetReal2D( object_points_ex, 0, 2, 0.0 );

    cvSetReal2D( object_points_ex, 1, 0, 0 );
    cvSetReal2D( object_points_ex, 1, 1, 203 );
    cvSetReal2D( object_points_ex, 1, 2, 0.0 );

    cvSetReal2D( object_points_ex, 2, 0, 127) ; //0.4m
    cvSetReal2D( object_points_ex, 2, 1, 203 );
    cvSetReal2D( object_points_ex, 2, 2, 0.0 );

    cvSetReal2D( object_points_ex, 3, 0, 127); //0.4m
    cvSetReal2D( object_points_ex, 3, 1, 0 );
    cvSetReal2D( object_points_ex, 3, 2, 0.0 );

    cvFindExtrinsicCameraParams2( object_points_ex,
        image_points_ex,
        itsIntrinsicMatrix,
        itsDistortionCoeffs,
        itsCameraRotation,
        itsCameraTranslation);

    cvReleaseMat( &image_points_ex);
    cvReleaseMat( &object_points_ex);
    //   return corners;

  }
  return corners;

}

// ######################################################################
void HeliPose::displayExtrinsic(Image<byte>& img)
{
  int  NUM_GRID         = 12; //21
  CvMat *my_3d_point = cvCreateMat( 3, NUM_GRID * NUM_GRID + 1, CV_64FC1);
  CvMat *my_image_point = cvCreateMat( 2, NUM_GRID * NUM_GRID + 1, CV_64FC1);

  for ( int i = 0; i < NUM_GRID; i++){
    for ( int j = 0; j < NUM_GRID; j++){
      cvSetReal2D( my_3d_point, 0, i * NUM_GRID + j, (i * 50));
      cvSetReal2D( my_3d_point, 1, i * NUM_GRID + j, (j * 50));
      cvSetReal2D( my_3d_point, 2, i * NUM_GRID + j, 0.0);
    }
  }

  cvSetReal2D( my_3d_point, 0, NUM_GRID*NUM_GRID, 0.0);
  cvSetReal2D( my_3d_point, 1, NUM_GRID*NUM_GRID, 0.0);
  cvSetReal2D( my_3d_point, 2, NUM_GRID*NUM_GRID, 15);


  cvProjectPoints2( my_3d_point,
      itsCameraRotation,
      itsCameraTranslation,
      itsIntrinsicMatrix,
      itsDistortionCoeffs,
      my_image_point);

  //printf( "Rotation2: %f %f %f\n",
  //    cvGetReal2D(itsCameraRotation, 0, 0),
  //    cvGetReal2D(itsCameraRotation, 0, 1),
  //    cvGetReal2D(itsCameraRotation, 0, 2));
  //printf("%f %f %f\n",
  //    cvGetReal2D(itsCameraTranslation, 0, 0),
  //    cvGetReal2D(itsCameraTranslation, 0, 1),
  //    cvGetReal2D(itsCameraTranslation, 0, 2));

  for ( int i = 0; i < NUM_GRID; i++){
    for ( int j = 0; j < NUM_GRID-1; j++){
      int im_x1 = (int)cvGetReal2D( my_image_point, 0, i * NUM_GRID + j);
      int im_y1 = (int)cvGetReal2D( my_image_point, 1, i * NUM_GRID + j);
      int im_x2 = (int)cvGetReal2D( my_image_point, 0, i * NUM_GRID + j+1);
      int im_y2 = (int)cvGetReal2D( my_image_point, 1, i * NUM_GRID + j+1);

      cvLine( img2ipl(img), cvPoint( im_x1, im_y1), cvPoint( im_x2, im_y2), CV_RGB( 0, 255, 0), 1);
    }
  }
  for ( int j = 0; j < NUM_GRID; j++){
    for ( int i = 0; i < NUM_GRID-1; i++){
      int im_x1 = (int)cvGetReal2D( my_image_point, 0, i * NUM_GRID + j);
      int im_y1 = (int)cvGetReal2D( my_image_point, 1, i * NUM_GRID + j);
      int im_x2 = (int)cvGetReal2D( my_image_point, 0, (i+1) * NUM_GRID + j);
      int im_y2 = (int)cvGetReal2D( my_image_point, 1, (i+1) * NUM_GRID + j);

      cvLine( img2ipl(img), cvPoint( im_x1, im_y1), cvPoint( im_x2, im_y2), CV_RGB( 0, 255, 0), 1);
    }
  }

  int im_x0 = (int)cvGetReal2D( my_image_point, 0, 0);
  int im_y0 = (int)cvGetReal2D( my_image_point, 1, 0);
  int im_x = (int)cvGetReal2D( my_image_point, 0, NUM_GRID*NUM_GRID);
  int im_y = (int)cvGetReal2D( my_image_point, 1, NUM_GRID*NUM_GRID);
  cvLine( img2ipl(img), cvPoint( im_x0, im_y0), cvPoint( im_x, im_y), CV_RGB( 255, 0, 0), 2); //Z axis

  cvReleaseMat( &my_3d_point);
  cvReleaseMat( &my_image_point);

}

HeliPose::Pose HeliPose::getPose()
{
  pthread_mutex_lock(&itsPoseLock);
  Pose pose = itsCurrentPose;
  pthread_mutex_unlock(&itsPoseLock);
  return pose;
}

// ######################################################################
Image<PixRGB<byte> > HeliPose::getImg()
{
  pthread_mutex_lock(&itsImgLock);
  Image<PixRGB<byte> > img = itsCurrentImg;
  pthread_mutex_unlock(&itsImgLock);

  return img;
}

// ######################################################################
void HeliPose::run_cameraPose()
{
  running = true;

  while(running)
  {
    GenericFrame input = itsIfs->readFrame();

    pthread_mutex_lock(&itsImgLock);
    itsCurrentImg = input.asRgb();
    pthread_mutex_unlock(&itsImgLock);

    //Image<byte> grey = luminance(itsCurrentImg);
    CvSeq* square = findSquares(itsCurrentImg, itsStorage,
        1500, //minArea
        15000, //maxArea
        0.3 //minCos
        );
    std::vector<Point2D<int> > corners = getRectangle(square);

    //std::vector<CvPoint2D32f> corners = getExtrinsic(grey);

    //std::vector<Point2D<int> > corners = getExtrinsic(itsCurrentImg);

    Point2D<float> center(0,0);
    for(uint i=0; i<corners.size(); i++)
    {
      center.i += corners[i].i;
      center.j += corners[i].j;
    }
    center /= corners.size();


    if (itsDebug)
    {
      if (corners.size() > 0)
      {
        for(uint i=0; i<corners.size(); i++)
          drawCircle(itsCurrentImg, corners[i], 6, PixRGB<byte>(255,0,0), 4);
        drawCircle(itsCurrentImg, Point2D<int>(center), 6, PixRGB<byte>(0,255,0), 4);
      }

      //displayExtrinsic(grey);
      itsOfs->writeRGB(itsCurrentImg, "HeliPose", FrameInfo("heliPose", SRC_POS));
    }


    pthread_mutex_lock(&itsPoseLock);

    if (corners.size() > 0)
    {
      itsCurrentPose.translation = Point3D<float>(center.i - (320/2), center.j - (240/2), 0);
      itsCurrentPose.valid = true;
    } else {
      itsCurrentPose.valid = false;
    }




    //float x = cvGetReal2D(itsCameraTranslation, 0, 0);
    //float y = cvGetReal2D(itsCameraTranslation, 0, 1);
    //float z  = cvGetReal2D(itsCameraTranslation, 0, 2);

    //float rotx =  cvGetReal2D(itsCameraRotation, 0, 0);
    //float roty =  cvGetReal2D(itsCameraRotation, 0, 1);
    //float rotz =  cvGetReal2D(itsCameraRotation, 0, 2);

    //itsCurrentPose.translation = Point3D<float>(x,y,z);
    //itsCurrentPose.rotation = Point3D<float>(rotx, roty, rotz);
    //if (corners.size() > 0)
    //  itsCurrentPose.valid = true;
    //else
    //  itsCurrentPose.valid = false;

    pthread_mutex_unlock(&itsPoseLock);

    cvClearMemStorage(itsStorage);


    //// sleep a little:
    usleep(10000);
  }

  // we got an order to stop:
  //running = f;  // FIXME: bogus! other thread may unlock too soon
  pthread_exit(0);
}

void HeliPose::run_imuPose()
{
  running = true;

  while(running)
  {
    IMU_SFE_Atomic::IMUData imuData = itsIMU->readIMUData();

    pthread_mutex_lock(&itsPoseLock);
    Pose currentPose = itsCurrentPose; //get the current pose
    pthread_mutex_unlock(&itsPoseLock);


    currentPose.rotation.z += (imuData.yaw*itsRotationScale.z) - itsRotationBias.z;
    currentPose.velocity.x += (imuData.accelX*itsVelocityScale.x) - itsVelocityBias.x;
    currentPose.translation.x += currentPose.velocity.x*itsVelocityScale.x;
    //Update the pose position from the imu


    pthread_mutex_lock(&itsPoseLock);

    itsCurrentPose.valid = true;
    itsCurrentPose.translation = currentPose.translation;
    itsCurrentPose.velocity = currentPose.velocity;
    itsCurrentPose.rotation = currentPose.rotation;
    itsCurrentPose.accelX = imuData.accelX*itsVelocityScale.x;
    itsCurrentPose.accelY = imuData.accelY;
    itsCurrentPose.accelZ = imuData.accelZ;
    itsCurrentPose.roll = imuData.roll;
    itsCurrentPose.pitch = imuData.pitch;
    itsCurrentPose.yaw = imuData.yaw*itsRotationScale.z;

    pthread_mutex_unlock(&itsPoseLock);

    //// sleep a little:
    usleep(10000);
  }

  // we got an order to stop:
  //running = f;  // FIXME: bogus! other thread may unlock too soon
  pthread_exit(0);
}

void HeliPose::getIMUBias()
{

  //Claculate the mean and variance of the imu data

  pthread_mutex_lock(&itsPoseLock);
  for(int i=0; i<50; i++)
  {
    IMU_SFE_Atomic::IMUData imuData = itsIMU->readIMUData();

    float yawVal = imuData.yaw*itsRotationScale.z;
    float xAcc = imuData.accelX*itsVelocityScale.x;

    itsRotationBias.z = onlineMean(itsRotationBias.z, yawVal, i+1);
    itsVelocityBias.x = onlineMean(itsVelocityBias.x, xAcc, i+1);
    LINFO("%f %f",
        itsRotationBias.z, itsVelocityBias.x);
  }

  itsCurrentPose.translation = Point3D<float>(0,0,0);
  itsCurrentPose.rotation = Point3D<float>(0,0,0);
  itsCurrentPose.velocity = Point3D<float>(0,0,0);
  pthread_mutex_unlock(&itsPoseLock);

}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

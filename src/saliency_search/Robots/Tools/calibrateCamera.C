/*!@file Robots/Tools/calibrateCamera.C  */

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/Tools/calibrateCamera.C $
// $Id: calibrateCamera.C 15282 2012-05-07 23:39:26Z kai $
//

#include "Image/OpenCVUtil.H"
#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Image/DrawOps.H"
#include "Image/ColorOps.H"
#include "Image/FilterOps.H"
#include "Image/VisualTracker.H"
#include "GUI/ImageDisplayStream.H"
#include "GUI/XWinManaged.H"
#include "Image/MathOps.H"
#include "Image/Transforms.H"
#include "Image/Layout.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Raster/Raster.H"
#include "GUI/XWinManaged.H"
#include "GUI/DebugWin.H"
#include <pthread.h>

#define KEY_UP 98
#define KEY_DOWN 104
#define KEY_LEFT 100
#define KEY_RIGHT 102

//Camera params
CvMat* itsIntrinsicMatrix;
CvMat* itsDistortionCoeffs;
CvMat* itsCameraRotation;
CvMat* itsCameraTranslation;

//GRID param
#define GRID_SIZE 51.5

int getKey(nub::ref<OutputFrameSeries> &ofs)
{
  const nub::soft_ref<ImageDisplayStream> ids =
    ofs->findFrameDestType<ImageDisplayStream>();

  const rutz::shared_ptr<XWinManaged> uiwin =
    ids.is_valid()
    ? ids->getWindow("Output")
    : rutz::shared_ptr<XWinManaged>();

  if (uiwin.is_valid())
    return uiwin->getLastKeyPress();
  else
    return -1;
}

Point2D<int> getMouseClick(nub::ref<OutputFrameSeries> &ofs, const char* wname)
{
  const nub::soft_ref<ImageDisplayStream> ids =
    ofs->findFrameDestType<ImageDisplayStream>();

  const rutz::shared_ptr<XWinManaged> uiwin =
    ids.is_valid()
    ? ids->getWindow(wname)
    : rutz::shared_ptr<XWinManaged>();

  if (uiwin.is_valid())
    return uiwin->getLastMouseClick();
  else
    return Point2D<int>(-1,-1);
}

#ifdef HAVE_OPENCV2
void projectGrid(Image<PixRGB<byte> > &img)
{

  //draw center point
  drawCircle(img, Point2D<int>(img.getWidth()/2, img.getHeight()/2), 3, PixRGB<byte>(255,0,0));

  CvMat *rot_mat = cvCreateMat( 3, 3, CV_64FC1);
  cvRodrigues2( itsCameraRotation, rot_mat, 0);

  int  NUM_GRID         = 9; //21
  CvMat *my_3d_point = cvCreateMat( 3, NUM_GRID * NUM_GRID + 2, CV_64FC1);
	CvMat *my_image_point = cvCreateMat( 2, NUM_GRID * NUM_GRID + 2, CV_64FC2);

  for ( int i = 0; i < NUM_GRID; i++){
                for ( int j = 0; j < NUM_GRID; j++){
                        cvSetReal2D( my_3d_point, 0, i * NUM_GRID + j,(i * GRID_SIZE));
                        cvSetReal2D( my_3d_point, 1, i * NUM_GRID + j,(j * GRID_SIZE));
                        cvSetReal2D( my_3d_point, 2, i * NUM_GRID + j, 0.0);
                }
        }

  cvSetReal2D( my_3d_point, 0, NUM_GRID*NUM_GRID, 0);
        cvSetReal2D( my_3d_point, 1, NUM_GRID*NUM_GRID, 0);
        cvSetReal2D( my_3d_point, 2, NUM_GRID*NUM_GRID, 0);

  cvSetReal2D( my_3d_point, 0, NUM_GRID*NUM_GRID+1, 0);
        cvSetReal2D( my_3d_point, 1, NUM_GRID*NUM_GRID+1, 0);
        cvSetReal2D( my_3d_point, 2, NUM_GRID*NUM_GRID+1, 30);

  cvProjectPoints2( my_3d_point,
                                        itsCameraRotation,
                                        itsCameraTranslation,
                                        itsIntrinsicMatrix,
                                        itsDistortionCoeffs,
                                        my_image_point);

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

        int im_x0 = (int)cvGetReal2D( my_image_point, 0, NUM_GRID*NUM_GRID);
        int im_y0 = (int)cvGetReal2D( my_image_point, 1, NUM_GRID*NUM_GRID);
        int im_x = (int)cvGetReal2D( my_image_point, 0, NUM_GRID*NUM_GRID+1);
        int im_y = (int)cvGetReal2D( my_image_point, 1, NUM_GRID*NUM_GRID+1);
        cvLine( img2ipl(img), cvPoint( im_x0, im_y0), cvPoint( im_x, im_y), CV_RGB( 255, 0, 0), 2); //Z axis


        cvReleaseMat( &my_3d_point);
        cvReleaseMat( &my_image_point);
  cvReleaseMat( &rot_mat);

}

void projectRect(Image<PixRGB<byte> > &img, float width, float height)
{

  //draw center point
  drawCircle(img, Point2D<int>(img.getWidth()/2, img.getHeight()/2), 3, PixRGB<byte>(255,0,0));

  CvMat *my_3d_point = cvCreateMat( 3, 5, CV_64FC1);
        CvMat *my_image_point = cvCreateMat( 2, 5, CV_64FC2);

  cvSetReal2D( my_3d_point, 0, 0, -width/2);
  cvSetReal2D( my_3d_point, 1, 0, -height/2);
  cvSetReal2D( my_3d_point, 2, 0, 0.0);

  cvSetReal2D( my_3d_point, 0, 1, width/2);
  cvSetReal2D( my_3d_point, 1, 1, -height/2);
  cvSetReal2D( my_3d_point, 2, 1, 0.0);

  cvSetReal2D( my_3d_point, 0, 2, width/2);
  cvSetReal2D( my_3d_point, 1, 2, height/2);
  cvSetReal2D( my_3d_point, 2, 2, 0.0);

  cvSetReal2D( my_3d_point, 0, 3, -width/2);
  cvSetReal2D( my_3d_point, 1, 3, height/2);
  cvSetReal2D( my_3d_point, 2, 3, 0.0);

  cvSetReal2D( my_3d_point, 0, 4, 0);
  cvSetReal2D( my_3d_point, 1, 4, 0);
  cvSetReal2D( my_3d_point, 2, 4, 0.0);


  cvProjectPoints2( my_3d_point,
                                        itsCameraRotation,
                                        itsCameraTranslation,
                                        itsIntrinsicMatrix,
                                        itsDistortionCoeffs,
                                        my_image_point);

  int x1 = (int)cvGetReal2D( my_image_point, 0, 0);
  int y1 = (int)cvGetReal2D( my_image_point, 1, 0);
  int x2 = (int)cvGetReal2D( my_image_point, 0, 1);
  int y2 = (int)cvGetReal2D( my_image_point, 1, 1);
  int x3 = (int)cvGetReal2D( my_image_point, 0, 2);
  int y3 = (int)cvGetReal2D( my_image_point, 1, 2);
  int x4 = (int)cvGetReal2D( my_image_point, 0, 3);
  int y4 = (int)cvGetReal2D( my_image_point, 1, 3);

  int cx = (int)cvGetReal2D( my_image_point, 0, 4);
  int cy = (int)cvGetReal2D( my_image_point, 1, 4);

  drawLine(img,  Point2D<int>(x1,y1),  Point2D<int>(x2,y2), PixRGB<byte>(0, 255,0));
  drawLine(img,  Point2D<int>(x2,y2),  Point2D<int>(x3,y3), PixRGB<byte>(0, 255,0));
  drawLine(img,  Point2D<int>(x3,y3),  Point2D<int>(x4,y4), PixRGB<byte>(0, 255,0));
  drawLine(img,  Point2D<int>(x4,y4),  Point2D<int>(x1,y1), PixRGB<byte>(0, 255,0));

  drawCircle(img,  Point2D<int>(cx,cy), 3, PixRGB<byte>(0,255,0));


        cvReleaseMat( &my_3d_point);
        cvReleaseMat( &my_image_point);

}


void calibrateViews(std::vector<CvPoint2D32f>& corners, int rows, int cols)
{

  int nViews = corners.size()/(rows*cols);
  LINFO("Calibrate: %i views", nViews);

  if (nViews <= 0)
  {
    LINFO("No corners avl");
    return;
  }

  // Set up the object points matrix
  // Squares are size set in defines found in header file.
  CvMat* object_points = cvCreateMat(corners.size(), 3, CV_32FC1);
  for(uint k=0; k < corners.size()/(rows*cols); k++ )
  {
    for(int i=0; i < cols; i++ )
    {
      for(int j=0; j < rows; j++ )
      {
        cvmSet( object_points, k*(rows*cols) + i*rows + j, 0, GRID_SIZE*j );        // x coordinate
        cvmSet( object_points, k*(rows*cols) + i*rows + j, 1, GRID_SIZE*i ); // y coordinate
        cvmSet( object_points, k*(rows*cols) + i*rows + j, 2, 0 ); // z coordinate
      }
    }
  }

  //for (uint j = 0; j < corners.size(); j++){
  //  cvSetReal2D( object_points, j, 0, ( j % rows) * GRID_SIZE );
  //  cvSetReal2D( object_points, j, 1, ( j / rows) * GRID_SIZE );
  //  cvSetReal2D( object_points, j, 2, 0.0 );
        //}

        // Set up the matrix of points per image
        CvMat* point_counts = cvCreateMat(1, nViews, CV_32SC1);
        for(int i=0; i < nViews; i++ )
                cvSetReal1D( point_counts, i, rows*cols );

        // Copy corners found to matrix
        CvMat image_points = cvMat(corners.size(), 2, CV_32FC1, &corners[0]);



  int flags = 0;

        // Initiliazie the intrinsic matrix such that the two focal lengths
        // have a ratio of 1.0
 cvmSet( itsIntrinsicMatrix, 0, 0, 1.0);
 cvmSet( itsIntrinsicMatrix, 1, 1, 1.0);


  //flags = CV_CALIB_FIX_PRINCIPAL_POINT; // | CV_CALIB_USE_INTRINSIC_GUESS;
  //flags =  CV_CALIB_USE_INTRINSIC_GUESS;
  flags =  CV_CALIB_FIX_ASPECT_RATIO;

  cvCalibrateCamera2( object_points, &image_points, point_counts, cvSize(320,240),
      itsIntrinsicMatrix, itsDistortionCoeffs,
      NULL, NULL,
      flags);


  //display results
 // Image<byte> in = itsPCameraCellsInput[0];
 // Image<byte> out(in.getDims(), ZEROS);

 // cvUndistort2( img2ipl(in), img2ipl(out), itsIntrinsicMatrix, itsDistortionCoeffs);

 // //itsDebugImg = out;


  cvReleaseMat( &object_points);
  cvReleaseMat( &point_counts);

}

void findExtrinsic(std::vector<CvPoint2D32f>& corners, int rows, int cols)
{

  CvMat *image_points_ex = cvCreateMat( corners.size(), 2, CV_64FC1);

  for (uint j = 0; j < corners.size(); j++){
    cvSetReal2D( image_points_ex, j, 0, corners[j].x);
    cvSetReal2D( image_points_ex, j, 1, corners[j].y);
  }

  //int views = 1;

  CvMat *object_points_ex = cvCreateMat( corners.size(), 3, CV_64FC1);
  for (uint j = 0; j < corners.size(); j++){
                cvSetReal2D( object_points_ex, j, 0, ( j % rows) * GRID_SIZE );
                cvSetReal2D( object_points_ex, j, 1, ( j / rows) * GRID_SIZE );
                cvSetReal2D( object_points_ex, j, 2, 0.0 );
        }

  //cvSetReal2D( itsCameraTranslation, 0, 2, 782.319961 );
  cvFindExtrinsicCameraParams2( object_points_ex,
      image_points_ex,
      itsIntrinsicMatrix,
      itsDistortionCoeffs,
      itsCameraRotation,
      itsCameraTranslation);

        cvReleaseMat( &image_points_ex);
  cvReleaseMat( &object_points_ex);

}



std::vector<CvPoint2D32f> findCorners(Image<PixRGB<byte> > &img, int rows, int cols)
{

  int count = 0;

  std::vector<CvPoint2D32f> corners(rows*cols);

  Image<byte> in = luminance(img);

  int result = cvFindChessboardCorners(img2ipl(in), cvSize(rows,cols),
      &corners[0], &count,
      CV_CALIB_CB_ADAPTIVE_THRESH |
      CV_CALIB_CB_NORMALIZE_IMAGE |
      CV_CALIB_CB_FILTER_QUADS);

  // result = 0 if not all corners were found
  // Find corners to an accuracy of 0.1 pixel
        if(result != 0)
  {
                cvFindCornerSubPix(img2ipl(in),
        &corners[0],
        count,
        cvSize(10,10), //win
        cvSize(-1,-1), //zero_zone
        cvTermCriteria(CV_TERMCRIT_ITER,1000,0.01) );
    return corners;
  } else {
    return std::vector<CvPoint2D32f>();
  }


}



void processUserInput(nub::ref<OutputFrameSeries> &ofs, bool &drawGrid, bool &saveCorners, bool &calibrate)
{
  static bool moveCamera = false;
  static bool itsChangeRot = false;

  switch(getKey(ofs))
  {
    case KEY_UP:
      if (!moveCamera)
      {
        if (itsChangeRot)
          cvmSet(itsCameraRotation, 0, 0, cvGetReal2D(itsCameraRotation, 0, 0) + M_PI/180);
        else
          cvmSet(itsCameraTranslation, 0, 1, cvGetReal2D(itsCameraTranslation, 0, 1) - 1);
      } else {
        //itsCameraCtrl->movePanTilt(1, 0, true); //move relative
      }

      break;
    case KEY_DOWN:
      if (!moveCamera)
      {
        if (itsChangeRot)
          cvmSet(itsCameraRotation, 0, 0, cvGetReal2D(itsCameraRotation, 0, 0) - M_PI/180);
        else
          cvmSet(itsCameraTranslation, 0, 1, cvGetReal2D(itsCameraTranslation, 0, 1) + 1);
      } else {
        //itsCameraCtrl->movePanTilt(-1, 0, true); //move relative
      }
      break;
    case KEY_LEFT:
      if (!moveCamera)
      {
        if (itsChangeRot)
          cvmSet(itsCameraRotation, 0, 1, cvGetReal2D(itsCameraRotation, 0, 1) + M_PI/180);
        else
          cvmSet(itsCameraTranslation, 0, 0, cvGetReal2D(itsCameraTranslation, 0, 0) - 1);
      } else {
        //itsCameraCtrl->movePanTilt(0, 1, true); //move relative
      }
      break;
    case KEY_RIGHT:
      if (!moveCamera)
      {
        if (itsChangeRot)
          cvmSet(itsCameraRotation, 0, 1, cvGetReal2D(itsCameraRotation, 0, 1) - M_PI/180);
        else
          cvmSet(itsCameraTranslation, 0, 0, cvGetReal2D(itsCameraTranslation, 0, 0) + 1);
      } else {
        //itsCameraCtrl->movePanTilt(0, -1, true); //move relative
      }
      break;
    case 38: //a
      if (!moveCamera)
      {
        if (itsChangeRot)
          cvmSet(itsCameraRotation, 0, 2, cvGetReal2D(itsCameraRotation, 0, 2) + M_PI/180);
        else
          cvmSet(itsCameraTranslation, 0, 2, cvGetReal2D(itsCameraTranslation, 0, 2) + 1);
      } else {
        //itsCameraCtrl->zoom(1, true); //move relative
      }
      break;
    case 52: //z
      if (!moveCamera)
      {
        if (itsChangeRot)
          cvmSet(itsCameraRotation, 0, 2, cvGetReal2D(itsCameraRotation, 0, 2) - M_PI/180);
        else
          cvmSet(itsCameraTranslation, 0, 2, cvGetReal2D(itsCameraTranslation, 0, 2) - 1);
      } else {
        //itsCameraCtrl->zoom(-1, true); //move relative
      }
      break;
    case 39: //s
      saveCorners = true;
      break;
    case 54: //c
      calibrate = true;
      break;
    case 42: //g
      drawGrid = !drawGrid;
      {
        double d0 = cvGetReal2D( itsDistortionCoeffs, 0, 0);
        double d1 = cvGetReal2D( itsDistortionCoeffs, 1, 0);
        double d2 = cvGetReal2D( itsDistortionCoeffs, 2, 0);
        double d3 = cvGetReal2D( itsDistortionCoeffs, 3, 0);
        printf( "distortion_coeffs ( %7.7lf, %7.7lf, %7.7lf, %7.7lf)\n", d0, d1, d2, d3);

        double ir00 = cvGetReal2D( itsIntrinsicMatrix, 0, 0);
        double ir01 = cvGetReal2D( itsIntrinsicMatrix, 0, 1);
        double ir02 = cvGetReal2D( itsIntrinsicMatrix, 0, 2);
        double ir10 = cvGetReal2D( itsIntrinsicMatrix, 1, 0);
        double ir11 = cvGetReal2D( itsIntrinsicMatrix, 1, 1);
        double ir12 = cvGetReal2D( itsIntrinsicMatrix, 1, 2);
        double ir20 = cvGetReal2D( itsIntrinsicMatrix, 2, 0);
        double ir21 = cvGetReal2D( itsIntrinsicMatrix, 2, 1);
        double ir22 = cvGetReal2D( itsIntrinsicMatrix, 2, 2);
        printf( "intrinsics ( %7.5lf, %7.5lf, %7.5lf)\n", ir00, ir01, ir02);
        printf( "           ( %7.5lf, %7.5lf, %7.5lf)\n", ir10, ir11, ir12);
        printf( "           ( %7.5lf, %7.5lf, %7.5lf)\n", ir20, ir21, ir22);

        printf( "Rotation: %f(%0.2fDeg) %f(%0.2fDeg) %f(%0.2fDeg)\n",
            cvGetReal2D(itsCameraRotation, 0, 0),
            cvGetReal2D(itsCameraRotation, 0, 0)*180/M_PI,
            cvGetReal2D(itsCameraRotation, 0, 1),
            cvGetReal2D(itsCameraRotation, 0, 1)*180/M_PI,
            cvGetReal2D(itsCameraRotation, 0, 2),
            cvGetReal2D(itsCameraRotation, 0, 2)*180/M_PI);
        printf( "Translation: %f %f %f\n",
            cvGetReal2D(itsCameraTranslation, 0, 0),
            cvGetReal2D(itsCameraTranslation, 0, 1),
            cvGetReal2D(itsCameraTranslation, 0, 2));

        CvMat *rot_mat = cvCreateMat( 3, 3, CV_64FC1);
        cvRodrigues2( itsCameraRotation, rot_mat, 0);

        printf( "Rotation Rot: %0.2f %0.2f %0.2f\n",
            cvGetReal2D(rot_mat, 0, 0)*180/M_PI,
            cvGetReal2D(rot_mat, 0, 1)*180/M_PI,
            cvGetReal2D(rot_mat, 0, 2)*180/M_PI);
        printf( "              %0.2f %0.2f %0.2f\n",
            cvGetReal2D(rot_mat, 1, 0)*180/M_PI,
            cvGetReal2D(rot_mat, 1, 1)*180/M_PI,
            cvGetReal2D(rot_mat, 1, 2)*180/M_PI);
        printf( "              %0.2f %0.2f %0.2f\n",
            cvGetReal2D(rot_mat, 2, 0)*180/M_PI,
            cvGetReal2D(rot_mat, 2, 1)*180/M_PI,
            cvGetReal2D(rot_mat, 2, 2)*180/M_PI);


        cvReleaseMat( &rot_mat);


      }
      break;
    case 27: //r
      itsChangeRot = !itsChangeRot;
      break;
  }
}

int main(int argc, const char **argv)
{
  // Instantiate a ModelManager:
  ModelManager manager("Test wiimote");

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  nub::ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv, "", 0, 0) == false) return(1);

  manager.start();



  //Init camara params
        itsIntrinsicMatrix = cvCreateMat( 3, 3, CV_32FC1);
        itsDistortionCoeffs = cvCreateMat( 4, 1, CV_32FC1);
  itsCameraRotation = cvCreateMat( 1, 3, CV_64FC1);
  itsCameraTranslation = cvCreateMat( 1, 3, CV_64FC1);

  //cvmSet(itsDistortionCoeffs, 0, 0, -0.2403274);
  //cvmSet(itsDistortionCoeffs, 1, 0, 2.5312502);
  //cvmSet(itsDistortionCoeffs, 2, 0, -0.0439848);
  //cvmSet(itsDistortionCoeffs, 3, 0, -0.0106820);
  cvmSet(itsDistortionCoeffs, 0, 0, 0);
  cvmSet(itsDistortionCoeffs, 1, 0, 0);
  cvmSet(itsDistortionCoeffs, 2, 0, 0);
  cvmSet(itsDistortionCoeffs, 3, 0, 0);

  cvmSet(itsCameraRotation, 0, 0, 2.391102);
  cvmSet(itsCameraRotation, 0, 1, 0);
  cvmSet(itsCameraRotation, 0, 2, 0);

  cvmSet(itsCameraTranslation, 0, 0, 0);
  cvmSet(itsCameraTranslation, 0, 1, 0);
  cvmSet(itsCameraTranslation, 0, 2, 840.954432);


  //cvmSet(itsIntrinsicMatrix, 0, 0, 290.85342); cvmSet(itsIntrinsicMatrix, 0, 1, 0); cvmSet(itsIntrinsicMatrix, 0, 2, 320/2); //159.50000);
  //cvmSet(itsIntrinsicMatrix, 1, 0, 0); cvmSet(itsIntrinsicMatrix, 1, 1, 290.85342 ); cvmSet(itsIntrinsicMatrix, 1, 2, 240/2); // 119.5);
  //cvmSet(itsIntrinsicMatrix, 2, 0, 0); cvmSet(itsIntrinsicMatrix, 2, 1, 0); cvmSet(itsIntrinsicMatrix, 2, 2, 1);

  cvmSet(itsIntrinsicMatrix, 0, 0, 415.5); cvmSet(itsIntrinsicMatrix, 0, 1, 0); cvmSet(itsIntrinsicMatrix, 0, 2, 320/2); //159.50000);
  cvmSet(itsIntrinsicMatrix, 1, 0, 0); cvmSet(itsIntrinsicMatrix, 1, 1, 436 ); cvmSet(itsIntrinsicMatrix, 1, 2, 240/2); // 119.5);
  cvmSet(itsIntrinsicMatrix, 2, 0, 0); cvmSet(itsIntrinsicMatrix, 2, 1, 0); cvmSet(itsIntrinsicMatrix, 2, 2, 1);

  bool drawGrid = true;
  bool saveCorners = false;
  bool calibrate = false;

  std::vector<CvPoint2D32f> allCorners;

  while(1)
  {

    GenericFrame input = ifs->readFrame();
    Image<PixRGB<byte> > img = input.asRgb();


    int rows = 4, cols = 3;

    std::vector<CvPoint2D32f> corners = findCorners(img, rows, cols);

    if (corners.size() == (uint)(rows*cols))
    {
      if (saveCorners)
        for(uint i=0; i<corners.size(); i++)
          allCorners.push_back(corners[i]);
      saveCorners = false;

      cvDrawChessboardCorners(img2ipl(img), cvSize(rows,cols), &corners[0], corners.size(), 1);
    }

    if (calibrate)
    {
      calibrateViews(allCorners, rows, cols);
      if (corners.size() == (uint)(rows*cols))
        findExtrinsic(corners, rows, cols);
      calibrate = false;
    }

    if (drawGrid)
      projectGrid(img);

    projectRect(img, 216.5, 279.5);


    processUserInput(ofs, drawGrid, saveCorners, calibrate);

    ofs->writeRGB(img, "Output", FrameInfo("Output", SRC_POS));

    ofs->updateNext();
  }

  // stop all our ModelComponents
  manager.stop();

  // all done!
  return 0;
}
#else
int main(int argc, const char **argv)
{
	LINFO("No OpenCV");
	return 0;
}

#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

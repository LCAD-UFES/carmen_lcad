/*!@file Robots2/Beobot2/HumanRobotInteraction/FaceDetector.C Ice Module to
detect faces    */
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
// Primary maintainer for this file: Dicky Sihite <sihite@usc.edu>
// $HeadURL: svn://ilab.usc.edu/trunk/saliency/src/Robots/Beobot2/HumanRobotInteaction/FaceDetector.C
// $ $Id: FaceDetector.C 15310 2012-06-01 02:29:24Z itti $
//////////////////////////////////////////////////////////////////////

#include "Robots/Beobot2/HumanRobotInteraction/FaceDetector.H"
#include "Ice/BeobotEvents.ice.H"

#include "Raster/Raster.H"
#include "Util/sformat.H"
#include "Image/Image.H"
#include "Ice/IceImageUtils.H"

#include <sys/stat.h>

//#define  LOG_FOLDER "../data/logs/"

// ######################################################################
FaceDetector::FaceDetector(OptionManager& mgr,
               const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsOfs(new OutputFrameSeries(mgr)),
  itsTimer(1000000),
  itsCurrImgID(-1),
  itsPrevProcImgID(-1)
{
  addSubComponent(itsOfs);
}

// ######################################################################
FaceDetector::~FaceDetector()
{ }

// ######################################################################
void FaceDetector::start1()
{
  // set start time
  itsTimer.reset();
}


// ######################################################################
void FaceDetector::registerTopics()
{
  // subscribe to all sensor data
  this->registerSubscription("CameraMessageTopic");

  this->registerPublisher("FacesMessageTopic");
}

// ######################################################################
void FaceDetector::evolve()
{
  // check if the current image is updated
  its_Curr_Img_mutex.lock();
  bool newImageFlag = (itsPrevProcImgID < itsCurrImgID);
  its_Curr_Img_mutex.unlock();

  // if so, process
  if(newImageFlag)
    {
      itsTimer.reset();

      its_Curr_Img_mutex.lock();
      itsProcImg = itsCurrImg;
      itsPrevProcImgID = itsCurrImgID;
      its_Curr_Img_mutex.unlock();

      // find faces
      Image<PixRGB<byte> > res = findFaces(itsProcImg);

      // display the image
      itsOfs->writeRGB(res, "display");
      itsOfs->updateNext();
      LINFO("time: %15.3f", itsTimer.get()/1000.0);

      // publish FacesMessage
      BeobotEvents::FacesMessagePtr msg =
        new BeobotEvents::FacesMessage;
      msg->RequestID = itsPrevProcImgID;

      its_Curr_Res_mutex.lock();
      LINFO("Number of faces detected : %" ZU , itsCurrentFacesFound.size());
      for(uint i = 0; i < itsCurrentFacesFound.size(); i++)
        {
          ImageIceMod::RectangleIce rect;
          rect.tl.i = itsCurrentFacesFound[i].top();
          rect.tl.j = itsCurrentFacesFound[i].left();
          rect.br.i = itsCurrentFacesFound[i].bottomO();
          rect.br.j = itsCurrentFacesFound[i].rightO();
          msg->faces.push_back(rect);
          LINFO("[%3d,%3d,%3d,%3d]", rect.tl.i,rect.tl.j,rect.br.i,rect.br.j );
        }
      its_Curr_Res_mutex.unlock();

      // publish the message
      LINFO("[%6d] Publishing Faces Message", itsCurrMessageID++);
      publish("FacesMessageTopic", msg);
    }
}

// ######################################################################
void FaceDetector::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{
  // reset the timer
  // itsTimer.reset();
  // camera message
  if(eMsg->ice_isA("::BeobotEvents::CameraMessage"))
  {
    // store the image
    BeobotEvents::CameraMessagePtr cameraMsg =
      BeobotEvents::CameraMessagePtr::dynamicCast(eMsg);

    int currRequestID = cameraMsg->RequestID;
    LINFO("Got a CameraMessage with Request ID = %d",
          currRequestID);
    Image<PixRGB<byte> > ima = Ice2Image<PixRGB<byte> >(cameraMsg->image);

    // this line was to find face, moved up to evolve
    its_Curr_Img_mutex.lock();
    itsCurrImg = ima;
    itsCurrImgID = cameraMsg->RequestID;
    its_Curr_Img_mutex.unlock();

    LINFO("time: %15.3f", itsTimer.get()/1000.0);
  }
}

// ######################################################################
Image<PixRGB<byte> > FaceDetector::findFaces(Image<PixRGB<byte> > ima)
{
  Image<PixRGB<byte> > res;
  IplImage *img = img2ipl(ima);


  int scale = 1;

  // Create a string that contains the cascade name
  const char* cascade_name =
    "haarcascade_frontalface_alt.xml";

  // Load a HaarClassifierCascde
  CvHaarClassifierCascade* cascade =
    (CvHaarClassifierCascade*)cvLoad( cascade_name, 0, 0, 0 );

  // Check whether the cascade has loaded successfully.
  // Else report and error and quit
  if( !cascade )
    LFATAL("ERROR: Could not load classifier cascade");

  // Create a new image based on the input image
  IplImage* temp =
    cvCreateImage( cvSize(img->width/scale,img->height/scale), 8, 3 );

  // Create two points to represent the face locations
  CvPoint pt1, pt2;
  int i;

  // Clear the memory storage which was used before
  CvMemStorage* storage = 0;
  storage = cvCreateMemStorage(0);
  cvClearMemStorage( storage );

  std::vector<Rectangle> tempResults;

  // Find whether the cascade is loaded, to find the faces. If yes, then:
  if( cascade )
    {

      // There can be more than one face in an image.
      // So create a growable sequence of faces.
      // Detect the objects and store them in the sequence
      CvSeq* faces = cvHaarDetectObjects( img, cascade, storage,
                                          1.1, 2, CV_HAAR_DO_CANNY_PRUNING,
                                          cvSize(40, 40) );

      // Loop the number of faces found.
      for( i = 0; i < (faces ? faces->total : 0); i++ )
        {
          // Create a new rectangle for drawing the face
          CvRect* r = (CvRect*)cvGetSeqElem( faces, i );

          // Find the dimensions of the face,and scale it if necessary
          pt1.x = r->x*scale;
          pt2.x = (r->x+r->width)*scale;
          pt1.y = r->y*scale;
          pt2.y = (r->y+r->height)*scale;

          // Draw the rectangle in the input image
          cvRectangle( img, pt1, pt2, CV_RGB(255,0,0), 3, 8, 0 );

          // store rectangle to publish later
          tempResults.push_back
            (Rectangle::tlbrO(pt1.y,pt1.x,pt2.y,pt2.x));
        }
    }

  // Release the temp image created.
  cvReleaseImage( &temp );

  // storing data
  its_Curr_Res_mutex.lock();
  itsCurrentFacesFound.clear();
  itsCurrentFacesFound = tempResults;
  // for (uint j = 0; j < tempResults.size(); j++)
  //   {
  //     //      itsCurrentFacesFound
  //   }
  its_Curr_Res_mutex.unlock();

  res = ipl2rgb(img);
  return res;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

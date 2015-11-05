/*!@file Image/VisualTracker.C Interface to VisualTracker */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/VisualTracker.C $
// $Id: VisualTracker.C 13551 2010-06-10 21:56:32Z itti $
//

#include "Image/VisualTracker.H"

#include "Component/OptionManager.H"
#include "Component/ModelOptionDef.H"
#include "Simulation/SimEventQueue.H"
#include "Image/ColorOps.H"
#include "Image/MathOps.H"

const ModelOptionCateg MOC_VISUALTRACKER = {
  MOC_SORTPRI_2, "Visual Tracker Related Options" };

const ModelOptionDef OPT_TrackWindowSize =
{ MODOPT_ARG(int), "TrackWindowSize", &MOC_VISUALTRACKER, OPTEXP_CORE,
    "Size of the tracking window",
    "trackwindowsize", '\0', "<int>", "30" };

const ModelOptionDef OPT_InitTrackWindowSize =
{ MODOPT_ARG(int), "InitTrackWindowSize", &MOC_VISUALTRACKER, OPTEXP_CORE,
    "Size of the window from which to initialize the tracker",
    "inittrackwindowsize", '\0', "<int>", "30" };

// ######################################################################
VisualTracker::VisualTracker(OptionManager& mgr, const std::string& descrName,
                   const std::string& tagName) :
  SimModule(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventInputFrame),
  SIMCALLBACK_INIT(SimEventSetVisualTracker),
  itsTrackWindowSize(&OPT_TrackWindowSize, this, ALLOW_ONLINE_CHANGES),
  itsInitTrackWindowSize(&OPT_InitTrackWindowSize, this, ALLOW_ONLINE_CHANGES)

{
#ifdef HAVE_OPENCV
    itsCurrentPoints = NULL;
    itsPreviousPoints = NULL;
    itsTrackStatus = NULL;
    itsTrackError = NULL;
    itsCurrentPyramid = NULL;
    itsPreviousPyramid = NULL;
    itsKalman = NULL;
    itsObjectHist = NULL;
    itsBackproject = NULL;
    itsTrackWindow = cvRect(0,0,320,240);
    itsUseKalman = false;
#endif
    itsInitTracker = true;
    itsTracking = false;

}

VisualTracker::~VisualTracker()
{
#ifdef HAVE_OPENCV
  cvFree(&itsCurrentPoints);
  cvFree(&itsPreviousPoints);
  cvFree(&itsTrackStatus);
  cvFree(&itsTrackError);
  cvReleaseImage(&itsCurrentPyramid);
  cvReleaseImage(&itsPreviousPyramid);

  cvReleaseKalman(&itsKalman);
#endif

}

void VisualTracker::onSimEventInputFrame(SimEventQueue& q,
                                  rutz::shared_ptr<SimEventInputFrame>& e)
{
#ifdef HAVE_OPENCV
  // here is the inputs image:
  const Image<PixRGB<byte> > inimg = e->frame().asRgb();

  //Image<float> rg, by;
  //getRGBY(inimg, rg, by, 25.0F);
  //Image<float> input = by + rg + luminance(inimg);
  ////TODO should return 3 seperate channels, for better discrimnation
  //inplaceNormalize(input, 0.0F, 255.0F);
  itsCurrentGreyImg = luminance(inimg);

  itsCurrentTargetLoc = trackObjects(itsCurrentGreyImg);

  if (itsCurrentTargetLoc.isValid())
  {
    q.post(rutz::make_shared(new SimEventVisualTracker(this, itsCurrentTargetLoc, true)));
    LINFO("Tracking %ix%i", itsCurrentTargetLoc.i, itsCurrentTargetLoc.j);
  } else {
    if (!itsTracking)
      q.post(rutz::make_shared(new SimEventVisualTracker(this, itsCurrentTargetLoc, false)));
  }
#endif

}

void VisualTracker::onSimEventSetVisualTracker(SimEventQueue& q,
                                  rutz::shared_ptr<SimEventSetVisualTracker>& e)
{
#ifdef HAVE_OPENCV
  Point2D<int> targetLoc = e->getTargetLoc();
  LINFO("Set visual tracker to %ix%i",
      targetLoc.i,
      targetLoc.j);

  setTargets(itsCurrentGreyImg, targetLoc);

#endif
}



void VisualTracker::start2()
{
}

void VisualTracker::initTracker(Dims imageDims)
{

#ifdef HAVE_OPENCV
  itsMaxNumPoints = 1;
  itsCurrentNumPoints = 0;
  itsCurrentPoints = (CvPoint2D32f*)cvAlloc(itsMaxNumPoints*sizeof(itsCurrentPoints));
  itsPreviousPoints = (CvPoint2D32f*)cvAlloc(itsMaxNumPoints*sizeof(itsPreviousPoints));

  itsPreviousGreyImg = Image<byte>(imageDims, ZEROS);
  itsCurrentPyramid = cvCreateImage( cvSize(imageDims.w(), imageDims.h()), 8, 1 );
  itsPreviousPyramid = cvCreateImage( cvSize(imageDims.w(), imageDims.h()), 8, 1 );

  itsTrackStatus = (char*)cvAlloc(itsMaxNumPoints);
  itsTrackError = (float*)cvAlloc(itsMaxNumPoints);

  if (itsUseKalman)
  {
    itsKalman = cvCreateKalman(4, //Dim of state vector x,y,dx,dy
                               2); //dim of mesurment vector x,y

    //State transition matrix
                    //x  y dx dy
    const float A[] = { 1, 0, 1, 0,
                      0, 1, 0, 1,
                      0, 0, 1, 0,
                      0, 0, 0, 1};

    //Observation matrix
    const float H[] = { 1, 0, 0, 0,
                      0, 1, 0, 0};

    //set the transition and mesurment matrix
    memcpy( itsKalman->transition_matrix->data.fl, A, sizeof(A));
    memcpy( itsKalman->measurement_matrix->data.fl, H, sizeof(H));

    //Set the process and measurment noise
    cvSetIdentity( itsKalman->process_noise_cov, cvRealScalar(1e-5) );
    cvSetIdentity( itsKalman->measurement_noise_cov, cvRealScalar(1e-1) );

    /*posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k) */
    cvSetIdentity( itsKalman->error_cov_post, cvRealScalar(1));

    cvZero(itsKalman->state_post); /* corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k)) */
    cvZero(itsKalman->state_pre); /* predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k) */

  }

//  //camshift
//  int hdims = 16;
//  float hranges_arr[] = {0,180};
//  float* hranges = hranges_arr;
//
//  itsObjectHist = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );
//  itsBackproject = cvCreateImage( cvSize(imageDims.w(), imageDims.h()), 8, 1 );


  itsTrackFlags = 0;
#endif
  itsInitTracker = false;

}


// ######################################################################
void VisualTracker::setTargets(const Image<byte>& grey, const Point2D<int> loc)
{
  if (!loc.isValid())
  {
    itsTracking = false;
    return;
  }

  if (itsInitTracker)
    initTracker(grey.getDims());

#ifdef HAVE_OPENCV
  itsCurrentNumPoints = itsMaxNumPoints;

  IplImage* currentImg = img2ipl(grey);
  itsCurrentPoints[0].x = loc.i;
  itsCurrentPoints[0].y = loc.j;

  //cvFindCornerSubPix(currentImg, itsCurrentPoints, itsCurrentNumPoints,
  //    cvSize(itsInitTrackWindowSize.getVal(),itsInitTrackWindowSize.getVal()),
  //    cvSize(-1,-1),
  //    cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,
  //      20,0.03));
  cvReleaseImageHeader(&currentImg);

  itsPreviousGreyImg = grey;

  itsTrackFlags = 0;
  IplImage *swap_temp;
  CV_SWAP( itsPreviousPyramid, itsCurrentPyramid, swap_temp );

  CvPoint2D32f* swap_points;
  CV_SWAP( itsPreviousPoints, itsCurrentPoints, swap_points );

  if (itsUseKalman)
  {
    itsKalman->state_post->data.fl[0] = loc.i;
    itsKalman->state_post->data.fl[1] = loc.j;
  }

#endif

  itsTracking = true;
}

// ######################################################################
void VisualTracker::setTargets(const Image<byte>& grey, const Image<byte>& target)
{
#ifdef HAVE_OPENCV
  if (itsInitTracker)
    initTracker(grey.getDims());

  IplImage* currentImg = img2ipl(grey);

  cvCalcHist( &currentImg, itsObjectHist );

  float max_val = 0.f;
  cvGetMinMaxHistValue(itsObjectHist, 0, &max_val, 0, 0 );
  cvConvertScale( itsObjectHist->bins, itsObjectHist->bins, max_val ? 255. / max_val : 0., 0 );



  itsTargetTempl = target;
#endif
}

// ######################################################################
Point2D<int> VisualTracker::trackTemplObject(const Image<byte>& grey, double& err)
{

  Point2D<int> targetLoc(-1,-1);

  if (!itsTargetTempl.initialized())
    return targetLoc;


#ifdef HAVE_OPENCV
  IplImage *result = cvCreateImage(
      cvSize(grey.getWidth() - itsTargetTempl.getWidth() + 1, grey.getHeight() - itsTargetTempl.getHeight()+1),
      IPL_DEPTH_32F, 1);

  cvMatchTemplate(img2ipl(grey), img2ipl(itsTargetTempl), result,
      //CV_TM_CCORR);
      CV_TM_SQDIFF_NORMED);

  double minVal, maxVal;
  CvPoint minLoc, maxLoc;
  cvMinMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

  if (true) //minVal < 0.5)
  {
    targetLoc.i = minLoc.x + itsTargetTempl.getWidth()/2;
    targetLoc.j = minLoc.y + itsTargetTempl.getHeight()/2;
    err = minVal;
  } else {
    err = -1;
  }
#endif

  return targetLoc;



}

//track using mean shift
//Point2D<int> VisualTracker::trackTemplObject(const Image<byte>& grey, double& err)
//{
//
//  Point2D<int> targetLoc(-1,-1);
//
//  if (!itsTargetTempl.initialized())
//    return targetLoc;
//
//  IplImage* currentImg = img2ipl(grey);
//
//  cvCalcBackProject( &currentImg, itsBackproject, itsObjectHist );
//
//
//  CvBox2D track_box;
//  CvConnectedComp track_comp;
//
//  cvCamShift( itsBackproject, itsTrackWindow,
//      cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ),
//      &track_comp, &track_box );
//
//  itsTrackWindow = track_comp.rect;
//
//
//  targetLoc.i = (int)track_box.center.x;
//  targetLoc.j = (int)track_box.center.y;
//
//  return targetLoc;
//}

// ######################################################################
Point2D<int> VisualTracker::trackObjects(const Image<byte>& grey)
{
  Point2D<int> targetLoc(-1,-1);

  if (!itsTracking)
    return targetLoc;

#ifdef HAVE_OPENCV
  if (itsCurrentNumPoints > 0)
  {
    IplImage* pGrey = img2ipl(itsPreviousGreyImg);
    IplImage* cGrey = img2ipl(grey);

    //flags = CV_LKFLOW_INITIAL_GUESSES;

    cvCalcOpticalFlowPyrLK(pGrey, cGrey, itsPreviousPyramid, itsCurrentPyramid,
        itsPreviousPoints, itsCurrentPoints,
        itsCurrentNumPoints, //number of feature points
        cvSize(itsTrackWindowSize.getVal(),itsTrackWindowSize.getVal()), //search window size in each pyramid
        3, // maximal pyramid level nummber
        itsTrackStatus,
        itsTrackError,
        cvTermCriteria(CV_TERMCRIT_ITER
          |CV_TERMCRIT_EPS,
          20,0.03), itsTrackFlags);

    itsTrackFlags = CV_LKFLOW_PYR_A_READY | CV_LKFLOW_PYR_B_READY;

    cvReleaseImageHeader(&pGrey);
    cvReleaseImageHeader(&cGrey);


    //show track points
    int k, i;
    for(i = k = 0; i<itsCurrentNumPoints; i++)
    {
      if (!itsTrackStatus[i])
        continue;

      itsCurrentPoints[k++] = itsCurrentPoints[i];
      //LINFO("Error %i: %f", i, itsTrackError[i]);
      if (itsTrackError[i] < 2000)
      {
        targetLoc.i = std::min(grey.getWidth()-1, std::max(0, (int)itsCurrentPoints[i].x));
        targetLoc.j = std::min(grey.getHeight()-1, std::max(0, (int)itsCurrentPoints[i].y));
        ASSERT(grey.coordsOk(targetLoc));
      }
    }
    itsCurrentNumPoints = k;

  }

  IplImage *swap_temp;
  CV_SWAP( itsPreviousPyramid, itsCurrentPyramid, swap_temp );
  CvPoint2D32f* swap_points;
  CV_SWAP( itsPreviousPoints, itsCurrentPoints, swap_points );

  itsPreviousGreyImg = grey;


  if (itsUseKalman && grey.coordsOk(targetLoc))
  {
    float Z[2];
    CvMat Zmat = cvMat(2,1,CV_32F, Z);

    Z[0] = targetLoc.i;
    Z[1] = targetLoc.j;

    cvKalmanCorrect(itsKalman, &Zmat);
    const CvMat* prediction = cvKalmanPredict(itsKalman, 0);

    //generate measurement
    cvMatMulAdd(itsKalman->measurement_matrix, itsKalman->state_pre, NULL, &Zmat);

    targetLoc.i = (int)prediction->data.fl[0];
    targetLoc.j = (int)prediction->data.fl[1];
  }

#endif

  return targetLoc;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

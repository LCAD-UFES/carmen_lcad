//*!@file Image/BlobTracker.C A multi-blob tracker */
//
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
// Primary maintainer for this file: Rand Voorhies <voorhies at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/BlobTracker.C $
// $Id: BlobTracker.C 14376 2011-01-11 02:44:34Z pez $
//

#ifndef BLOBTRACKER_C_DEFINED
#define BLOBTRACKER_C_DEFINED

#include "Image/BlobTracker.H"
#include "GUI/DebugWin.H"


BlobTracker::BlobTracker(OptionManager& mgr,
    const std::string& descrName,
    const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName)
{
  itsTrackerParams.FGTrainFrames = 0;

  //Foreground Detector Module:
  // Adaptive background mixture models for real-time tracking. CVPR1999
  //  itsTrackerParams.pFG = cvCreateFGDetectorBase(CV_BG_MODEL_MOG, NULL);
  // Foreground Object Detection from Videos Containing Complex Background. ACM MM2003.
  itsTrackerParams.pFG = cvCreateFGDetectorBase(CV_BG_MODEL_FGD, NULL);
  //itsTrackerParams.pFG = cvCreateFGDetectorBase(CV_BG_MODEL_FGD_SIMPLE, NULL);

  //Blob Detector Module:
  // Detect new blob by tracking CC of FG mask
  //  itsTrackerParams.pBD = cvCreateBlobDetectorCC();

  // Detect new blob by uniform moving of connected components of FG mask
  itsTrackerParams.pBD = cvCreateBlobDetectorSimple();


  //Blob Tracker Module:
  // Simple connected component tracking
  itsTrackerParams.pBT = cvCreateBlobTrackerCC();
 // itsTrackerParams.pBT = cvCreateBlobTrackerCCMSPF();
  
  // Blob Tracker Tratectory Generation Module
  //itsTrackerParams.pBTGen = cvCreateModuleBlobTrackGen1();
  itsTrackerParams.pBTGen = NULL;

  //Blob Tracker Post Processor:
  // None
  itsTrackerParams.pBTPP = NULL;
  //itsTrackerParams.pBTPP = cvCreateModuleBlobTrackPostProcKalman();

  //Use Post Processing Data:
  // No, don't
  itsTrackerParams.UsePPData = 0;
  //itsTrackerParams.UsePPData = 1;

  //Blob Tracker Trajectory Analysis Module
  // None
  itsTrackerParams.pBTA = NULL;

  itsTracker = cvCreateBlobTrackerAuto1(&itsTrackerParams);

  if(!itsTracker)
    LFATAL("Could Not Create Blob Tracker!");

}

BlobTracker::~BlobTracker()
{
}

void BlobTracker::update(Image<PixRGB<byte> > img)
{
  IplImage* pImg  = img2ipl(img);
  LINFO("Attempting to process image %dx%d",img.getWidth(),img.getHeight());
  //Process the new image
  itsTracker->Process(pImg);

  itsBlobs.clear();

  //Analyze the blobs picked out by the tracker
  for(int i=0; i<itsTracker->GetBlobNum(); i++)
  {
    Blob          newBlob;
    CvBlob*       blob;
    Point2D<int>  blobCenter;
    Dims          blobDims;
    int           blobID;

    //Grab the next blob
    blob = itsTracker->GetBlob(i);

    //Find the blob center
    blobCenter = Point2D<int> (
        blob->x,
        blob->y
        );

    //Find the blob dims
    blobDims = Dims(
        blob->w,
        blob->h
        );

    blobID = CV_BLOB_ID(blob);

    newBlob.confidence = 0;
    newBlob.center = blobCenter;
    newBlob.dims   = blobDims;
    newBlob.id     = blobID;

    itsBlobs.push_back(newBlob);
  }
}




#endif

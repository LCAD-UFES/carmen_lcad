/*!@file Neuro/EnvObjDetection.C */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/EnvObjDetection.C $
// $Id: EnvObjDetection.C 12962 2010-03-06 02:13:53Z irock $
//

#include "Image/OpenCVUtil.H"  // must be first to avoid conflicting defs of int64, uint64

#include "Neuro/EnvObjDetection.H"

#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "GUI/DebugWin.H"
#include "Component/ModelOptionDef.H"
#include "Neuro/NeuroOpts.H"

static const ModelOptionDef OPT_CascadeFilePath =
  { MODOPT_ARG_STRING, "Cascade file path", &MOC_ITC, OPTEXP_CORE,
    "Name of directory containing the description of a trained cascade classifier."
    "Used in making faces salient or any other object.  ",
    "cascade-file", '\0', "<filename>.xml",
    "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt2.xml"};


// ######################################################################
EnvObjDetection::EnvObjDetection(OptionManager& mgr)
  :
  EnvSegmenter(mgr, "Embeddable Object detection",
               "EnvObjDetection"),
  itsCascadeFile(&OPT_CascadeFilePath, this)
#ifdef HAVE_OPENCV
  ,
  itsStorage(cvCreateMemStorage(0))
#endif
{
#ifndef HAVE_OPENCV
  LFATAL("OpenCV must be installed in order to use this function");
#else
  ASSERT(itsStorage != 0);

  itsCascade = (CvHaarClassifierCascade*)cvLoad( itsCascadeFile.getVal().c_str(), 0, 0, 0 );
  if( !itsCascade )
    LFATAL("ERROR: Could not load classifier cascade (%s)\n", itsCascadeFile.getVal().c_str() );

#endif
}

// ######################################################################
EnvObjDetection::~EnvObjDetection()
{
#ifndef HAVE_OPENCV
  LERROR("OpenCV must be installed in order to use this function");
#else
  cvReleaseMemStorage(&itsStorage);
#endif
}

// ######################################################################
Rectangle EnvObjDetection::getFoa(const Image<PixRGB<byte> >& rgbin,
                                           const Point2D<int>& center,
                                           Image<byte>* foamask,
                                           Image<PixRGB<byte> >* segmentdisp) const
{
#ifndef HAVE_OPENCV
  LFATAL("OpenCV must be installed in order to use this function");
  return Rectangle();
#else

  Image<byte> lum = luminance(rgbin);

  double scale = 1.3;
  IplImage* small_img = cvCreateImage(
      cvSize( cvRound (lum.getWidth()/scale), cvRound (lum.getHeight()/scale)),
      8, 1 );

  cvResize( img2ipl(lum), small_img, CV_INTER_LINEAR );
  cvEqualizeHist( small_img, small_img );
  cvClearMemStorage( itsStorage );

  Rectangle result;
  if( itsCascade )
  {
    double t = (double)cvGetTickCount();
    CvSeq* objects = cvHaarDetectObjects( small_img, itsCascade, itsStorage,
        1.1, 2, 0/*CV_HAAR_DO_CANNY_PRUNING*/,
        cvSize(30, 30) );
    t = (double)cvGetTickCount() - t;
    LDEBUG( "detection time = %gms objects=%i\n",
        t/((double)cvGetTickFrequency()*1000.), objects->total );

    if (objects->total > 0)
    {
      int i = 0;
      CvRect* r = (CvRect*)cvGetSeqElem( objects, i );

      result = Rectangle(Point2D<int>((int)(r->x*scale), (int)(r->y*scale)),
          Dims((int)(r->width*scale), (int)(r->height*scale)));
    }

  }
  cvReleaseImage( &small_img );

  return result.getOverlap(rgbin.getBounds());
#endif
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

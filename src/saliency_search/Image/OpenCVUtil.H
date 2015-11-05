/*!@file Image/OpenCVUtil.H OpenCV IPL image conversions
 */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/OpenCVUtil.H $
// $Id: OpenCVUtil.H 15272 2012-05-07 20:40:14Z kai $
//

#ifndef IMAGE_OPENCVUTIL_H_DEFINED
#define IMAGE_OPENCVUTIL_H_DEFINED




// opencv 2.0 (but not earlier versions) defines int64 and uint64 so
// let's not redefine them in Util/Types.H:
#ifdef HAVE_OPENCV2
#   include <opencv2/core/version.hpp>
#   include <opencv2/core/core.hpp>
#   include <opencv2/core/core_c.h>
#   include <opencv2/core/types_c.h>
#   include <opencv2/features2d/features2d.hpp>
#   include <opencv2/objdetect/objdetect.hpp>
#   include <opencv2/imgproc/imgproc.hpp>
#   include <opencv2/imgproc/imgproc_c.h>
#   include <opencv2/video/background_segm.hpp>
#   include <opencv2/highgui/highgui_c.h>
#   include <opencv2/video/tracking.hpp>
#   include <opencv2/calib3d/calib3d.hpp>

#else
#ifdef HAVE_OPENCV1

#ifdef HAVE_OPENCV_CV_H
#   include <cstddef>
#endif //HAVE_OPENCV_CV_H

#   include <opencv/cvver.h>
#   include <opencv/cv.h>
#ifdef HAVE_OPENCVAUX
#   include <opencv/cxtypes.h>
#   include <opencv/highgui.h>
#endif //HAVE_OPENCVAUX
#endif //HAVE_OPENCV
#endif //HAVE_OPENCV2

#if CV_MAJOR_VERSION >= 2
#define INVT_TYPEDEF_INT64
#define INVT_TYPEDEF_UINT64
#endif
#include "Util/Types.H"

#ifdef HAVE_OPENCV

template <class T> class Image;
template <class T> class PixRGB;

//! Convert from Image<PixRGB<byte>> to IplImage
IplImage* img2ipl(const Image< PixRGB<byte> > &img);

//! Convert from Image<byte> to IplImage
IplImage* img2ipl(const Image<byte> &img);

//! Convert from Image<float> to IplImage
IplImage* img2ipl(const Image<float> &img);

//! Convert to Image<byte> from IplImage, which must have depth==8 and nChannels==1
Image<byte> ipl2gray(const IplImage* img);

//! Convert to Image<PixRGB<byte>> from IplImage, which must have depth==8, nChannels==3, and dataOrder==0 (interleaved)
Image<PixRGB<byte> > ipl2rgb(const IplImage* img);

//! Convert to Image<byte> from IplImage, which must have depth==32 and nChannels==1
Image<float> ipl2float(const IplImage* img);

#endif

// ######################################################################
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif

/*!@file Image/OpenCVUtil.C OpenCV IPL image conversions
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/OpenCVUtil.C $
// $Id: OpenCVUtil.C 14605 2011-03-15 02:25:06Z dparks $
//

#ifdef HAVE_OPENCV
#include "Image/OpenCVUtil.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Util/log.H"

// ######################################################################
IplImage* img2ipl(const Image< PixRGB<byte> > &img)
{
  IplImage* ret_img =
    cvCreateImageHeader(cvSize(img.getWidth(), img.getHeight()),
                        IPL_DEPTH_8U, 3);

  ret_img->imageData = (char*)(img.getArrayPtr());

  return ret_img;
}

// ######################################################################
IplImage* img2ipl(const Image<byte> &img)
{
  IplImage* ret_img =
    cvCreateImageHeader(cvSize(img.getWidth(), img.getHeight()),
                        IPL_DEPTH_8U, 1);

  ret_img->imageData = (char*)(img.getArrayPtr());

  return ret_img;
}

// ######################################################################
IplImage* img2ipl(const Image<float> &img)
{
  IplImage* ret_img =
    cvCreateImageHeader(cvSize(img.getWidth(), img.getHeight()),
                        IPL_DEPTH_32F, 1);

  ret_img->imageData = (char*)(img.getArrayPtr());

  return ret_img;
}

// ######################################################################
Image<byte> ipl2gray(const IplImage* img)
{
  if (img->depth != IPL_DEPTH_8U)
    LFATAL("IplImage must have depth==IPL_DEPTH_8U"
           "for conversion to Image<byte>");

  if (img->nChannels != 1)
    LFATAL("IplImage must have nChannels==1"
           "for conversion to Image<byte>");

  return Image<byte>(reinterpret_cast<const byte*>(img->imageData),
                     img->width, img->height);
}

// ######################################################################
Image<PixRGB<byte> > ipl2rgb(const IplImage* img)
{
  if (img->depth != IPL_DEPTH_8U)
    LFATAL("IplImage must have depth==IPL_DEPTH_8U"
           "for conversion to Image<PixRGB<byte>>");

  if (img->nChannels != 3)
    LFATAL("IplImage must have nChannels==3"
           "for conversion to Image<PixRGB<byte>>");

  if (img->dataOrder != 0)
    LFATAL("IplImage must have dataOrder==0 (interleaved)"
           "for conversion to Image<PixRGB<byte>>");

  return Image<PixRGB<byte> >
    (reinterpret_cast<const PixRGB<byte>*>(img->imageData),
    img->width, img->height);
}

// ######################################################################
Image<float> ipl2float(const IplImage* img)
{
  if (img->depth != IPL_DEPTH_32F)
    LFATAL("IplImage must have depth==IPL_DEPTH_32F"
           "for conversion to Image<float>");

  if (img->nChannels != 1)
    LFATAL("IplImage must have nChannels==1"
           "for conversion to Image<float>");

  return Image<float>(reinterpret_cast<const float*>(img->imageData),
                      img->width, img->height);
}

#endif

// ######################################################################
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

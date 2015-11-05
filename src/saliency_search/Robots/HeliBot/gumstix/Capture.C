/*!@file Capture.C  */

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: $
// $Id: $
//
 
#include "Capture.H"

// ######################################################################
Capture::Capture(int width, int height, int fps) :
  itsWidth(width),
  itsHeight(height),
  itsFps(fps)
{

}

// ######################################################################
Capture::~Capture()
{
  stopCapture();
}

// ######################################################################
void Capture::initCapture()
{

  //Requested Camera Resolution
  char* videodevice = "/dev/video0";
  
  int format = V4L2_PIX_FMT_MJPEG;
  int grabmethod = 1;
  char *avifilename = NULL;
  
  //Allocate our video input structure
  itsVideoIn = (struct vdIn *) calloc(1, sizeof(struct vdIn));
  //Initialize our color lookup tables
  initLut();
  //Initialize the video input data structure
  init_videoIn(itsVideoIn, (char *) videodevice, itsWidth, itsHeight, itsFps, format,
      grabmethod, avifilename);

  itsImg = new unsigned char[itsWidth*itsHeight*3];
}

//Perform Packed YUV422 to RGB24 conversion directly on an Image
void Capture::yuv422_to_rgb24(const unsigned char* inPtr, unsigned char* dst,
    unsigned int width, unsigned int height)
{
	const unsigned char *buff = inPtr;

  for (int j=0; j < height; j++)
    for(int i=0; i<width; i+=2)
    {
      const unsigned char Y = buff[0];
      const unsigned char U = buff[1];
      const unsigned char Y1 = buff[2];
      const unsigned char V = buff[3];
      buff += 4;

      *dst++ = R_FROMYV(Y,V);
      *dst++ = G_FROMYUV(Y,U,V);
      *dst++ = B_FROMYU(Y,U);

      *dst++ = R_FROMYV(Y1,V);
      *dst++ = G_FROMYUV(Y1,U,V);
      *dst++ = B_FROMYU(Y1,U);
    }
} 

// ######################################################################
//Grab a single image from an attached USB webcam
unsigned char* Capture::grabFrame() 
{
  //Grab the image from the camera (also does MJPEG->JPEG->YUYV conversion)
  uvcGrab(itsVideoIn);

  yuv422_to_rgb24(itsVideoIn->framebuffer, itsImg, itsWidth, itsHeight);

  return itsImg;
}

// ######################################################################
unsigned char* Capture::grabFrameRaw(unsigned int& frameSize) 
{
  uvcGrabRaw(itsVideoIn);
  frameSize = itsVideoIn->buf.bytesused;
  return itsVideoIn->framebuffer;
}

// ######################################################################
void Capture::stopCapture()
{
  if (itsImg)
    free(itsImg);
  close_v4l2(itsVideoIn);
}
  


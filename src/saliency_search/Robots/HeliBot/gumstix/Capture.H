/*!@file Capture.H */

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


#ifndef Capture_H_DEFINED
#define Capture_H_DEFINED

extern "C" {
#include <captureMjpeg/color.h>
#include <captureMjpeg/v4l2uvc.h>
#include <captureMjpeg/utils.h>
}

class Capture 
{
public:

  Capture();

  //Init the capture with width,hight and frame per second
  Capture(int width, int height, int fps);
    
  ~Capture();

  //! init the v4l drives and start capturing
  void initCapture();

  //! stop capturing
  void stopCapture();

  //! grab raw frame
  unsigned char* grabFrame();

  unsigned char* grabFrameRaw(unsigned int& frameSize );

  int getWidth() { return itsWidth; }
  int getHeight() { return itsHeight; }

  //Perform Packed YUV422 to RGB24 conversion directly on an Image
  void yuv422_to_rgb24(const unsigned char* inPtr, unsigned char* dst,
      unsigned int width, unsigned int height);
  
  
private:
  int itsWidth;
  int itsHeight;
  unsigned char* itsImg;
  int itsFps;
  vdIn* itsVideoIn;
  
};


#endif



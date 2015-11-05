/*!@file Gist/test-Canny.C edge dector using cvCanny algorithm */
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
// Primary maintainer for this file: Chin-Kai Chang <chinkaic@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/test-Canny.C $
// $Id: test-Canny.C 14770 2011-05-19 04:19:46Z kai $
//
//////////////////////////////////////////////////////////////////////////
#include "Image/OpenCVUtil.H"
#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameIstream.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"
#include "Image/CutPaste.H"     		// for inplacePaste()
#include "Image/DrawOps.H" 					//for writeText and SimpleFont
#include "Image/ColorOps.H" 				//for luminance
#include "Image/MathOps.H" 					//for inplaceNormalize() 
#include "Image/Normalize.H" 				//for inplaceNormalize() 
#include "GUI/ImageDisplayStream.H"
#include <cstdio>										//for sprintf
#include "Util/Timer.H"

//######################################################################
//Doing Canny Edge 
Image<PixRGB<byte> > getCannyEdge(Image<PixRGB<byte> > img,Image<PixRGB<byte> > &rawCannyImg){

  //convert iNVT image to cvImage
  IplImage *cvImg = cvCreateImage(cvGetSize(img2ipl(img)),8,1);
  CvMemStorage *s = cvCreateMemStorage(0);
  CvSeq *lines = 0;

  //Doing Canny Edge 
  int lowThreshold = 50;
  int highThreshold = 250;
  int sobelApertureSize = 3;
  cvCanny(img2ipl(luminance(img)),cvImg,lowThreshold,highThreshold,sobelApertureSize);

  //convert cvImage to iNVT image
  Image<PixRGB<byte> > edgeImg = toRGB(ipl2gray(cvImg)); 
  rawCannyImg = edgeImg;

  int threshold = 10;
  int minLineLength = 10;
  int maxGap = 5;
  lines = cvHoughLines2(cvImg,s,CV_HOUGH_PROBABILISTIC,1,CV_PI/180,threshold,minLineLength,maxGap);

  //int lx1 = 0,lx2 = 0,ly1 = 0,ly2 = 0,rx1 = 0,rx2 =0 ,ry1 = 0,ry2 = 0;
  for(int i = 0;i<MIN(lines->total,100);i++)
    {
      LDEBUG("Total Line %d",lines->total);
      CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
      Point2D<int> pt1 = Point2D<int>(line[0].x,line[0].y);
      Point2D<int> pt2 = Point2D<int>(line[1].x,line[1].y);
      int dx = pt2.i - pt1.i;
      int dy = pt2.j - pt1.j;
      int midx = (pt1.i+pt2.i)/2;
      int midy = (pt1.j+pt2.j)/2;
      float slope = 0;
      if(dx*dy != 0)
	slope = dy/dx;	
      PixRGB<byte > line_color;
      if(slope == 0.0)
	line_color = PixRGB<byte>(200,0,0);//Dark Red
      else if(slope < 0.0 && slope > -10.0)	
	{
	  line_color = PixRGB<byte>(128,128,200);//Light blue
	  LDEBUG("Left edge Road!!!! %f",slope);
	}
      else if(slope <= -10.0 )	
	line_color = PixRGB<byte>(0,128,0);//Green
      else if(slope > 0.0 && slope < 10.0)	
	{
	  line_color = PixRGB<byte>(0,255,0);//light green
	}
      else {
	line_color = PixRGB<byte>(0,0,128);//blue
      }
      drawLine(edgeImg,pt1,pt2,line_color,2);
      if(slope != 0.0)
	{
	  drawLine(edgeImg,pt1,pt2,line_color,2);
	  char buffer[128];
	  sprintf(buffer,"%1.2f",slope);
	  writeText(edgeImg,Point2D<int>(midx,midy),buffer,line_color,PixRGB<byte>(0,0,0),SimpleFont::FIXED(8));
	}//end slope != 0.0

    }//end for loop
  return edgeImg;	
}
// ######################################################################
int main(int argc, char **argv)
{
  // instantiate a model manager:
  ModelManager manager("test Canny");

  // Instantiate our various ModelComponents:

  nub::soft_ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);

  nub::soft_ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  manager.exportOptions(MC_RECURSE);

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv, "[image name] ", 0, 4) == false) return(1);
  // let's do it!
  manager.start();

  Timer tim(1000000);

  bool keepGoing = true; uint index = 0;
  while(keepGoing)
    {
      ifs->updateNext();
      Image<PixRGB<byte> > ima = ifs->readRGB();

      if(!ima.initialized()) { keepGoing = false; }
      else
	{
	  uint w = ima.getWidth();
	  uint h = ima.getHeight();

	  //Create Display Image for 2xW and 1xH
	  Image<PixRGB<byte> > dispIma(w*2, h, NO_INIT);

	  Image<PixRGB<byte> > rawCanny(ima.getDims(),ZEROS);

	  tim.reset();
	  Image<PixRGB<byte> > output    = getCannyEdge(ima,rawCanny);
	  LINFO("time: %f", tim.get()/1000.0F);

	  inplacePaste(dispIma, ima, Point2D<int>(0, 0));
	  inplacePaste(dispIma, output, Point2D<int>(w, 0));

	  ofs->writeRGB(dispIma, "Canny");
	  ofs->updateNext();
	  Raster::waitForKey();
			

	}
      index++;
    }
  Raster::waitForKey();
  return 0;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

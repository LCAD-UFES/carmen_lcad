/*!@file Gist/test-KMeans.C color segmentation using cvKMeans2 algorithm */
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/test-KMeans.C $
// $Id: test-KMeans.C 14770 2011-05-19 04:19:46Z kai $
//
//////////////////////////////////////////////////////////////////////////

#include "Gist/Clustering.H"

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
#include "Gist/VanishingPointDetector.H" 
#include "Util/Timer.H"


// // ######################################################################
// //Doing Canny Edge 
// Image<PixRGB<byte> > getCannyEdge(Image<PixRGB<byte> > img,Image<PixRGB<byte> > &rawCannyImg){

//   //convert iNVT image to cvImage
//   IplImage *cvImg = cvCreateImage(cvGetSize(img2ipl(img)),8,1);
//   //CvMemStorage *s = cvCreateMemStorage(0);
//   //CvSeq *lines = 0;

//   //Doing Canny Edge 
//   cvCanny(img2ipl(luminance(img)),cvImg,100,150,3);

//   //convert cvImage to iNVT image
//   Image<PixRGB<byte> > edgeImg = toRGB(ipl2gray(cvImg)); 
//   rawCannyImg = edgeImg;

//   //lines = cvHoughLines2(cvImg,s,CV_HOUGH_PROBABILISTIC,1,CV_PI/180,80,30,10);

//   return edgeImg;	
// }

// ######################################################################
int main(int argc, char **argv)
{

  // instantiate a model manager:
  ModelManager manager("test KMean");

  // Instantiate our various ModelComponents:

  nub::soft_ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);

  nub::soft_ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  manager.exportOptions(MC_RECURSE);

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv, "[image name] [K-size] [Dist-Gain] ", 0, 4) == false) return(1);
  // let's do it!
  manager.start();

  bool keepGoing = true; uint index = 0;

  Timer timer(1000000); float time = 0.0;
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

          Image<PixRGB<byte> >rawCanny(ima);

          time = timer.get()/1000.0F;
          Image<PixRGB<byte> > output = getKMeans(ima, 10,0.0);

          LINFO("Total time for one frame: %f", timer.get()/1000.0F - time);
          time = timer.get()/1000.0F;
			
          inplacePaste(dispIma, output, Point2D<int>(w, 0));
          inplacePaste(dispIma, ima, Point2D<int>(0, 0));

          ofs->writeRGB(dispIma, "K-Mean Segmentation");
          ofs->updateNext();

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

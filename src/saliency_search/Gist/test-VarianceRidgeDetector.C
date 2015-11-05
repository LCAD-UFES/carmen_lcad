/*!@file Gist/test-VarianceRidgeDetector.C 
  testing Variance Ridge boundary detector algorithm                    */
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
//                                                       d               //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/test-VarianceRidgeDetector.C $
// $Id: $
//
//////////////////////////////////////////////////////////////////////////
//
// Implementation of boundary detection algorithm described in:
//
// Real-time texture boundary detection from ridges
// in the standard deviation space
// Ray Hidayat and Richard Green
// BMCV 2009

#ifndef TEST_VARIANCE_RIDGE_DETECTOR
#define TEST_VARIANCE_RIDGE_DETECTOR

#include "Image/OpenCVUtil.H"

#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameIstream.H"

#include "Raster/Raster.H"
#include "Image/CutPaste.H"     // for inplacePaste()
#include "Image/LowPass.H"

#include "Image/ColorOps.H"
#include "Image/MathOps.H"



#include "Image/DrawOps.H"
#include "Image/Kernels.H"
#include "Image/FilterOps.H"

#include "Image/ImageSet.H"
#include "Image/PyramidOps.H"
#include "Raster/Raster.H"

#include "Util/Timer.H"
#include  <cstdio>

#include "Gist/ContourBoundaryDetector.H"

Image<float> getGabor(Image<float> img,
		      float angle, float filter_period,
		      float elongation, int size);

//Image<float> getCanny(Image<byte> img);



int main(int argc, char **argv)
{
  // instantiate a model manager:
  ModelManager manager("test VRD Boundary Detection");

  // Instantiate our various ModelComponents:

  nub::soft_ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);

  nub::soft_ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  rutz::shared_ptr<ContourBoundaryDetector> 
    cbd(new ContourBoundaryDetector());

  manager.exportOptions(MC_RECURSE);

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv, "[window_size] ", 0, 1)
      == false) return(1);

  // get the operation mode
  int r = 8;
  if(manager.numExtraArgs() >  0)
    r = manager.getExtraArgAs<uint>(0);

  // let's do it!
  manager.start();

  ifs->updateNext();
  Image<PixRGB<byte> > ima = ifs->readRGB();
  // ima = Image<PixRGB<byte> >(ima.getDims(),ZEROS);
  // drawFilledRect(ima, 
  // 	   Rectangle(Point2D<int>(180, 100), Dims(100, 100)), 
  // 	   PixRGB<byte>(255,0,0));

  Image<float> fIma(luminance(ima));
  Image<float> tempFIma = fIma;
  inplaceNormalize(tempFIma, 0.0f,255.0f);  
  Image<byte>  bIma(tempFIma); 

  Timer timer(1000000); timer.reset();
  cbd->computeContourBoundary(ima,r);	
  Image<float> boundaryMap = cbd->getVarianceRidgeBoundaryMap();
  LINFO("time: %f ms", timer.get()/1000.0);

  // get non-max suppressed boundary map image
  
  float mVal = 32;
  float bVal = 255 - mVal;
  inplaceNormalize(boundaryMap, 0.0f,bVal);
  Image<byte> dBmapc(boundaryMap);

  Image<byte> dImaR, dImaG, dImaB;
  getComponents(ima, dImaR, dImaG, dImaB);
  inplaceNormalize(dImaR, byte(0), byte(mVal));
  inplaceNormalize(dImaG, byte(0), byte(mVal));
  inplaceNormalize(dImaB, byte(0), byte(mVal));
  Image<PixRGB<byte> > dBmap = toRGB(dBmapc);
  Image<PixRGB<byte> > dIma  = makeRGB(dImaR,dImaG,dImaB);

  // the non-max suppressed boundary map image
  Image<float> dBmapNMStemp = cbd->getNmsBoundaryMap();
  inplaceNormalize(dBmapNMStemp, 0.0F, 255.0F);
  Image<PixRGB<byte> > dBmapNMS = toRGB(Image<byte>(dBmapNMStemp));

  // the contour boundary map image
  Image<float> dCBmapTemp = cbd->getEdgelBoundaryMap();
  inplaceNormalize(dCBmapTemp, 0.0F, bVal);
  Image<PixRGB<byte> > dCBmap = toRGB(Image<byte>(dCBmapTemp));

  // setup the display map
  uint w = ima.getWidth();
  uint h = ima.getHeight();
  Image<PixRGB<byte> > dispIma(4*w,2*h,ZEROS);
  inplacePaste(dispIma, ima, Point2D<int>(0,0));
  inplacePaste(dispIma, Image<PixRGB<byte> >(dBmap), Point2D<int>(w,0));
  //inplacePaste(dispIma, Image<PixRGB<byte> >(dIma+dBmap), Point2D<int>(w,0));
  inplacePaste(dispIma, dBmapNMS, Point2D<int>(0,h));
  inplacePaste(dispIma,  Image<PixRGB<byte> >(dIma+dCBmap), Point2D<int>(w,h));
  inplacePaste(dispIma,  cbd->getContourBoundaryMap(), Point2D<int>(2*w,0));

  // angle at 4th param: 0 degrees
  //Image<float> gaborImg = getGabor(fIma, 0.0, 2.50, 1.0, 5);
  // Image<float> gaborImg = getGabor(fIma,0,7,1,9);
  // --------------
  // Image<float> gaborImg = getCanny(fIma);
  // inplaceNormalize(gaborImg, 0.0F, 255.0F);
  // Image<PixRGB<byte> > dGaborImg = toRGB(Image<byte>(gaborImg));
  // inplacePaste(dispIma, Image<PixRGB<byte> >(dGaborImg), 
  //  	       Point2D<int>(w*2,h));  

  // Image<float> rDir0 = itsRDirMax[0];
  // inplaceNormalize(rDir0, 0.0F, 255.0F);
  // Image<PixRGB<byte> > dRDir0 = toRGB(Image<byte>(rDir0));
  // inplacePaste(dispIma, dRDir0, Point2D<int>(2*w,0));

  // Image<float> rDir1 = itsRDirMax[1];
  // inplaceNormalize(rDir1, 0.0F, 255.0F);
  // Image<PixRGB<byte> > dRDir1 = toRGB(Image<byte>(rDir1));
  // inplacePaste(dispIma, dRDir1, Point2D<int>(3*w,0));

  // Image<float> rDir2 = itsRDirMax[2];
  // inplaceNormalize(rDir2, 0.0F, 255.0F);
  // Image<PixRGB<byte> > dRDir2 = toRGB(Image<byte>(rDir2));
  // inplacePaste(dispIma, dRDir2, Point2D<int>(2*w,h));

  // Image<float> rDir3 = itsRDirMax[3];
  // inplaceNormalize(rDir3, 0.0F, 255.0F);
  // Image<PixRGB<byte> > dRDir3 = toRGB(Image<byte>(rDir3));
  // inplacePaste(dispIma, dRDir3, Point2D<int>(3*w,h));

  // 300, 140
  /*
  drawRect(dispIma, 
	   Rectangle(Point2D<int>(w+156, 68), Dims(8, 8)), 
	   PixRGB<byte>(255,0,0));
  drawRect(dispIma, 
	   Rectangle(Point2D<int>(w+152, 64), Dims(16, 16)), 
	   PixRGB<byte>(255,255,0));

  drawRect(dispIma, 
	   Rectangle(Point2D<int>(156, h+68), Dims(8, 8)), 
	   PixRGB<byte>(255,0,0));
  drawRect(dispIma, 
	   Rectangle(Point2D<int>(152, h+64), Dims(16, 16)), 
	   PixRGB<byte>(255,255,0));
  */
  //ofs->writeRGB(boundaryMap, "VRD Boundary Detection");
  ofs->writeRGB(dispIma, "VRD Boundary Detection");
  ofs->updateNext();
  Raster::waitForKey();

  return 0;
}

// ######################################################################
Image<float> getGabor
(Image<float> img, float angle, float filter_period, 
 float elongation, int size)
{
  const double major_stddev = filter_period / 3.0;
  const double minor_stddev = major_stddev * elongation;

  // We have to add 90 to the angle here when constructing the gabor
  // filter. That's because the angle used to build the gabor filter
  // actually specifies the direction along which the grating
  // varies. This direction is orthogonal to the the direction of the
  // contours that the grating will detect.
  const double theta = angle + 90.0f;

  // In concept, we want to filter with four phases of the filter: odd
  // on+off, and even on+off (that would be xox, oxo, xo, and ox). But
  // since the on version just produces the negation of the off version,
  // we can get the summed output of both by just taking the absolute
  // value of the outputs of one of them. So we only need to convolve
  // with the filter in two phases, then take the absolute value (i.e.,
  // we're doing |xox| and |xo|).

  Image<float> g0 = gaborFilter3(major_stddev, minor_stddev,
                                 filter_period, 0.0f, theta, size);
  Image<float> g90 = gaborFilter3(major_stddev, minor_stddev,
                                  filter_period, 90.0f, theta, size);

  LDEBUG("angle = %.2f, period = %.2f pix, size = %dx%d pix",
         angle, filter_period, g0.getWidth(), g0.getHeight());

  Image<float> f0, f90;
  Image<float> result(img.getDims(), NO_INIT);

  Image<float> temp = energyNorm(img);
  f0 = optConvolve(temp, g0);
  f90 = optConvolve(temp, g90);

  result = f0 + f90;

  return result;
}

// // ######################################################################
// Image<float> getCanny(Image<byte> img)
// {
//   Image<float> ret;
//   IplImage *cvImg = cvCreateImage(cvGetSize(img2ipl(img)),8,1);  
//   int ap = 7;
//   cvCanny(img2ipl(img),cvImg, ap*ap*100, ap*ap*200, 7);
//   return ipl2gray(cvImg);
// }

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

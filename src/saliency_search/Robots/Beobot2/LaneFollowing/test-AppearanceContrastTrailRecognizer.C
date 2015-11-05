/*!@file
   Robots/Beobot2/LaneRecognition/test-AppearanceContrastTrailRecognizer.C
   test Lane recognition using appearance contrast [Rasmussen, etal. 09iros] */
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
// Primary maintainer for this file: Christian Siagian <siagian@caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/test-AppearanceContrastTrailRecognizer.C $
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

#ifndef TEST_APPEARANCE_CONTRAST_TRAIL_RECOGNIZER
#define TEST_APPEARANCE_CONTRAST_TRAIL_RECOGNIZER

#include "Component/ModelManager.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameIstream.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"

#include "Image/CutPaste.H"

#include "Robots/Beobot2/LaneFollowing/AppearanceContrastTrailRecognizer.H"

int main(int argc, char **argv)
{
  // instantiate a model manager:
  ModelManager manager("test Appearance Contrast Trail Recognizer");

  // Instantiate our various ModelComponents:

  nub::soft_ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);

  nub::soft_ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  rutz::shared_ptr<AppearanceContrastTrailRecognizer> 
    actr(new AppearanceContrastTrailRecognizer());

  manager.exportOptions(MC_RECURSE);

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv, "", 0, 0) == false) return(1);

  // get the operation mode
  //int r = 8;
  //if(manager.numExtraArgs() >  0)
  //  r = manager.getExtraArgAs<uint>(0);

  // let's do it!
  manager.start();

  ifs->updateNext();
  Image<PixRGB<byte> > ima = ifs->readRGB();
  actr->computeRoad(ima);
  Image<PixRGB<byte> > rima = actr->getDisplayImage();
  Image<PixRGB<byte> > kima = actr->getKMeansDisplayImage();

  uint w = ima.getWidth();
  uint h = ima.getHeight();
  Image<PixRGB<byte> > dispIma(3*w,h, ZEROS);

  inplacePaste(dispIma, ima,  Point2D<int>(  0,0));  
  inplacePaste(dispIma, rima, Point2D<int>(  w,0));
  inplacePaste(dispIma, kima, Point2D<int>(2*w,0));

  ofs->writeRGB(dispIma, "AppearanceContrastTrailRecognizer");
  ofs->updateNext();
  Raster::waitForKey();

  return 0;
}

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

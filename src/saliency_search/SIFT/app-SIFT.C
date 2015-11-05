/*! @file SIFT/test-SIFT.C Use the iLab SIFT implementation to output a
 *                         .key file just like Lowe's official binary */

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
// Primary maintainer for this file: Randolph Voorhies <voorhies at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/app-SIFT.C $


#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Media/FrameSeries.H"
#include "Util/Timer.H"
#include "GUI/XWinManaged.H"
#include "SIFT/ScaleSpace.H"
#include "SIFT/VisualObject.H"
#include "SIFT/Keypoint.H"
#include "SIFT/VisualObjectDB.H"
#include "Image/CutPaste.H"
#include "Image/ShapeOps.H"
#include "Image/DrawOps.H"
#include "Media/TestImages.H"
#include "Raster/Raster.H"
#include "Transport/FrameInfo.H"
#include "Raster/Raster.H"
#include "Raster/GenericFrame.H"

int main(const int argc, const char **argv)
{

  ModelManager manager("SIFT_Application");

  nub::ref<InputFrameSeries> ifs(new InputFrameSeries(manager));
  manager.addSubComponent(ifs);

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  if (manager.parseCommandLine(
        (const int)argc, (const char**)argv, "<OutputFile.key>", 2, 2) == false)
    return 0;

  manager.start();

  std::string outfileName = manager.getExtraArg(1);

  //grab the image
  while(1)
  {
    Image< PixRGB<byte> > inputImg;
    ifs->updateNext();
    GenericFrame input = ifs->readFrame();
    if (!input.initialized()) LFATAL("Could Not Open Input Image");
    inputImg = input.asRgb();

    //Create the new visual object to generate the keypoints
    VisualObject siftObject("Sift", "Image", inputImg);

    LINFO("Computed %d Keypoints", siftObject.numKeypoints());
  }

}

/*!@file HMAX/test-hmax5.C Test Hmax class and compare to original code */

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
// Primary maintainer for this file: Dan Parks <danielfp@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/HMAX/extractpatches.C $
// $Id: extractpatches.C 14139 2010-10-16 02:11:21Z dparks $
//

#include "Component/ModelManager.H"
#include "GUI/XWindow.H"
#include "HMAX/HmaxFL.H"
#include "HMAX/Hmax.H"
#include "Image/Image.H"
#include "Image/CutPaste.H"
#include "Image/Rectangle.H"
#include "Image/MathOps.H"
#include "Image/Normalize.H"
#include "Image/Transforms.H"
#include "Image/Convolutions.H"
#include "Learn/svm.h"
#include "Media/FrameSeries.H"
#include "nub/ref.h"
#include "Raster/GenericFrame.H"
#include "Raster/Raster.H"
#include "Util/Types.H"
#include "Util/log.H"

#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>
#include <cstdlib>


// number of orientations to use in HmaxFL
#define NORI 4
#define NUM_PATCHES_PER_SIZE 250


int main(const int argc, const char **argv)
{

  MYLOGVERB = LOG_INFO;
  ModelManager *mgr = new ModelManager("Extract Patches for Hmax with Feature Learning");

  mgr->exportOptions(MC_RECURSE);

  // required arguments
  // <c1patchesDir> <trainPosDir>

  if (mgr->parseCommandLine(
                            (const int)argc, (const char**)argv, "<c1patchesDir> <trainPosDir>", 2, 2) == false)
    return 1;

  // Create a temp HmaxFL object to extract C1Patches
  std::vector<int> c1ScaleSS(2);
  c1ScaleSS[0] = 1; c1ScaleSS[1] = 3;
  std::vector<int> c1SpaceSS(2);
  c1SpaceSS[0] = 10; c1SpaceSS[1] = 11;
  // desired frame sizes [11 and 13]
  HmaxFL hmax(NORI,c1SpaceSS,c1ScaleSS,2,true,1.0F,1.0F,0.3F,4.05F,-0.05F,11,2);

  std::string c1PatchesBaseDir;
  std::string trainPosName; // Directory where positive images are

  c1PatchesBaseDir = mgr->getExtraArg(0);
  trainPosName = mgr->getExtraArg(1);

  // Extract random patches from a set of images in a positive training directory
  std::vector<std::string> trainPos = hmax.readDir(trainPosName);
  int posTrainSize = trainPos.size();

  //Image<byte> inputb;

  Image<float> trainPosImage;

  std::cout << "Scanned training and testing images" << std::endl;

  std::vector<int> pS(4);
  pS[0] = 4; pS[1] = 8, pS[2] = 12; pS[3] = 16;

  std::srand(time(0));
  for(int i=0;i<NUM_PATCHES_PER_SIZE;i++){
    // Randomly select an image from the list
    unsigned int imInd = static_cast<unsigned int>(floor((rand()-1.0F)/RAND_MAX*posTrainSize));
    trainPosImage = Raster::ReadFloat(trainPos[imInd]);
    // Learn the appropriate simple S2 patches from the C1 results
    hmax.extractRandC1Patch(c1PatchesBaseDir,trainPosImage,i,pS);
  }

  std::cout << "Completed extraction of C1 Patches" << std::endl;

  return 0;
}




// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

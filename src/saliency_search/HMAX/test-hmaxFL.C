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
// Primary maintainer for this file: Lior Elazary
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/HMAX/test-hmaxFL.C $
// $Id: test-hmaxFL.C 12962 2010-03-06 02:13:53Z irock $
//

#include "Component/ModelManager.H"
#include "GUI/XWindow.H"
#include "HMAX/HmaxFL.H"
#include "HMAX/Hmax.H"
#include "Image/Image.H"
#include "Image/ColorOps.H"
#include "Image/CutPaste.H"
#include "Image/ShapeOps.H"
#include "Image/Rectangle.H"
#include "Image/MathOps.H"
#include "Image/MatrixOps.H"
#include "Image/Transforms.H"
#include "Image/Convolutions.H"
#include "Learn/SVMClassifier.H"
#include "Media/FrameSeries.H"
#include "Media/TestImages.H"
#include "nub/ref.h"
#include "Raster/GenericFrame.H"
#include "Transport/FrameInfo.H"
#include "Raster/Raster.H"
#include "Util/Types.H"
#include "Util/log.H"

#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <unistd.h>
#include <cstdlib>


// number of orientations to use in HmaxFL
#define NORI 4
#define NUM_PATCHES_PER_SIZE 250

int main(const int argc, const char **argv)
{

  MYLOGVERB = LOG_INFO;
  ModelManager *mgr = new ModelManager("Test Hmax with Feature Learning");

  nub::ref<InputFrameSeries> ifs(new InputFrameSeries(*mgr));
  mgr->addSubComponent(ifs);

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(*mgr));
  mgr->addSubComponent(ofs);



  mgr->exportOptions(MC_RECURSE);

  // required arguments
  // <c1patchesDir> <dir|list> <id> <outputfile>
  //
  // <id> is the given id for the given set of images
  // --in only needs to happen if we are loading the patches

  if (mgr->parseCommandLine(
        (const int)argc, (const char**)argv, "<c1patchesDir> <modelFile> <objectName> train=1/classify=0", 4, 4) == false)
    return 1;

  // get an HmaxFL object:
  std::vector<int> scss(9);
  scss[0] = 1; scss[1] = 3; scss[2] = 5; scss[3] = 7; scss[4] = 9;
  scss[5] = 11; scss[6] = 13; scss[7] = 15; scss[8] = 17;
  std::vector<int> spss(8);
  spss[0] = 8; spss[1] = 10; spss[2] = 12; spss[3] = 14;
  spss[4] = 16; spss[5] = 18; spss[6] = 20; spss[7] = 22;
  HmaxFL hmax(NORI, spss, scss);

  std::string c1PatchesBaseDir = mgr->getExtraArg(0);
  std::string modelFile = mgr->getExtraArg(1);
  std::string objName = mgr->getExtraArg(2);
  bool training = atoi(mgr->getExtraArg(3).c_str());

  hmax.readInC1Patches(c1PatchesBaseDir);

  mgr->start();

  ifs->startStream();

  SVMClassifier classifier;

  //Load the svm File
  if (!training)
    classifier.readModel(modelFile);

  while(1)
  {
    Image< PixRGB<byte> > inputImg;
    const FrameState is = ifs->updateNext();
    LINFO("Frame %i\n", ifs->frame());
    if (is == FRAME_COMPLETE)
      break;

    //grab the images
    GenericFrame input = ifs->readFrame();
    if (!input.initialized())
      break;
    inputImg = input.asRgb();

    Image<float> inputf = luminance(inputImg);

    inputf = rescale(inputf, 128, 128);

    std::vector<int> patchSizes = hmax.getC1PatchSizes();

    int objId = 0;
    //Get the metadata and find if we have the object name in the scene
    rutz::shared_ptr<GenericFrame::MetaData>
      metaData = input.getMetaData(std::string("SceneData"));
    if (metaData.get() != 0) {
      rutz::shared_ptr<TestImages::SceneData> sceneData;
      sceneData.dyn_cast_from(metaData);
      for (uint i = 0; i < sceneData->objects.size(); i++) {
        TestImages::ObjData objData = sceneData->objects[i];
        if (objData.name == objName)
          objId = 1;
      }
    }


    //Compute C2 features
    float **c2Res = new float*[patchSizes.size()];
    for(unsigned int i=0;i<patchSizes.size();i++) {
      c2Res[i] = new float[NUM_PATCHES_PER_SIZE];
    }
    hmax.getC2(inputf,c2Res);

    LINFO("C2 Processing Complete.");

    //printf("%i ", objId);
    //for(unsigned int i=0;i<patchSizes.size();i++)
    //  for(int j=0;j<NUM_PATCHES_PER_SIZE;j++)
    //    printf("%i:%f ", (i*NUM_PATCHES_PER_SIZE+j+1),  c2Res[i][j]);
    //printf("\n");


    if (training)
    {
      classifier.train(modelFile,objId,c2Res,patchSizes.size(),NUM_PATCHES_PER_SIZE);
    } else {
      double predObjId = classifier.predict(c2Res,patchSizes.size(),NUM_PATCHES_PER_SIZE);
      printf("Obj id %i predicted %f",
          objId, predObjId);
    }


    for(unsigned int i=0;i<patchSizes.size();i++) {
      delete[] c2Res[i];
    }
    delete [] c2Res;


    ofs->writeRGB(inputImg, "input", FrameInfo("input", SRC_POS));
  }


  return 0;
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

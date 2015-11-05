/*! @file ObjRec/test-ObjRec.C test various obj rec alg */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/test-ObjRec.C $
// $Id: test-ObjRec.C 10794 2009-02-08 06:21:09Z itti $
//


#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Image/Transforms.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "ObjRec/ObjRecSPM.H"
//#include "ObjRec/ObjRecSalBayes.H"
#include "Media/TestImages.H"
#include "GUI/DebugWin.H"

int getObjName(std::vector<std::string> &objNames, const std::string &objName)
{
  //Find the object
  //TODO can use hash function
  uint i=0;
  for(i=0; i<objNames.size(); i++)
    if (objNames[i] == objName)
      return i;

  objNames.push_back(objName);
  return i;
}


int main(const int argc, const char **argv)
{

  MYLOGVERB = LOG_INFO;
  ModelManager *mgr = new ModelManager("Test ObjRec");

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(*mgr));
  mgr->addSubComponent(ofs);

  nub::ref<ObjRecSPM> objRec(new ObjRecSPM(*mgr));
  //nub::ref<ObjRecSalBayes> objRec(new ObjRecSalBayes(*mgr));
  mgr->addSubComponent(objRec);

  mgr->exportOptions(MC_RECURSE);

  if (mgr->parseCommandLine(
        (const int)argc, (const char**)argv, "PathToImageDB", 1, 1) == false)
    return 1;

  mgr->start();

  TestImages testImages(mgr->getExtraArg(0).c_str(), TestImages::CALTECH256,
      1, 1, 1);


  std::vector<std::string> objNames;

  //Train the object rec
  for(uint scene=0; scene<testImages.getNumScenes(TestImages::TRAIN); scene++)
  {
    Image<PixRGB<byte> > sceneImg = testImages.getScene(scene, TestImages::TRAIN);
    //ofs->writeRGB(sceneImg, "Train", FrameInfo("Train", SRC_POS));

    TestImages::SceneData sceneData = testImages.getSceneData(scene, TestImages::TRAIN);
    LINFO("Train %s with Scene %s",
        sceneData.description.c_str(),
        sceneData.filename.c_str());
    objRec->train(sceneImg, sceneData.description);

    //Add the name to the list of objects trained on if not exsists
    getObjName(objNames, sceneData.description);
  }
  objRec->finalizeTraining();

  int totalImages = 0;
  int correctImages = 0;

  //The confusion matrix
  Image<float> confMatrix(objNames.size(), objNames.size(), ZEROS);

  //Test the object rec
  for(uint scene=0; scene<testImages.getNumScenes(TestImages::TEST); scene++)
  {
    Image<PixRGB<byte> > sceneImg = testImages.getScene(scene, TestImages::TEST);
    //ofs->writeRGB(sceneImg, "Object", FrameInfo("Objects", SRC_POS));

    TestImages::SceneData sceneData = testImages.getSceneData(scene, TestImages::TEST);
    std::string classDesc = objRec->predict(sceneImg);

    printf("Test:r:%s p:%s\n",
        sceneData.description.c_str(),
        classDesc.c_str());
    if(classDesc  == sceneData.description)
      correctImages++;
    totalImages++;

    //Update the confusion matrix
    int x = getObjName(objNames, classDesc);
    int y = getObjName(objNames, sceneData.description);

    if (confMatrix.coordsOk(x,y))
    {
      float pVal = confMatrix.getVal(x,y);
      confMatrix.setVal(x,y, pVal+1);
    } else {
      printf("Invalid corrd %ix%i", x, y);
    }

  }

  for(int y=0; y<confMatrix.getHeight(); y++)
  {
    for(int x=0; x<confMatrix.getWidth(); x++)
      printf("%f ", confMatrix.getVal(x,y));
    printf("\n");
  }


  //SHOWIMG(scaleBlock(confMatrix, Dims(256,256)));
  //SHOWIMG(confMatrix);

  printf("Recognition Rate %i/%i=%f (train size=%i)\n",
      correctImages,
      totalImages,
      (float)correctImages/(float)totalImages,
      testImages.getNumScenes(TestImages::TRAIN)
      );

  // stop all our ModelComponents
  mgr->stop();

  return 0;

}


/*! @file ObjRec/test-ObjRec.C test the objrec for the VOC challange */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/test-ObjRecVOC.C $
// $Id: test-ObjRecVOC.C 10794 2009-02-08 06:21:09Z itti $
//


#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Image/Transforms.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "ObjRec/ObjRecBOF.H"
#include "Media/TestImages.H"
#include "GUI/DebugWin.H"
#include <dirent.h>
#include <vector>

struct ObjInfo
{
  std::string name;
  int cls;
};

std::vector<ObjInfo> readImageSet(const char* filename)
{
  std::vector<ObjInfo> imageSet;

  FILE* fp = fopen(filename, "r");
  if (!fp)
    LFATAL("Error reading %s\n", filename);

  char name[255];
  int cls;
  while (fp != NULL)
  {
    if (fscanf(fp, "%s %i", name, &cls) != 2) break;
    ObjInfo objInf;
    objInf.name = std::string(name);
    if (cls == -1)
      objInf.cls = -1;
    else
      objInf.cls = 1;
    imageSet.push_back(objInf);
  }

  return imageSet;
}

int main(const int argc, const char **argv)
{

  MYLOGVERB = LOG_INFO;
  ModelManager *mgr = new ModelManager("Test ObjRec");

  nub::ref<ObjRecBOF> objRec(new ObjRecBOF(*mgr));
  mgr->addSubComponent(objRec);

  mgr->exportOptions(MC_RECURSE);

  if (mgr->parseCommandLine(
        (const int)argc, (const char**)argv, "PathToTrain PathToTest", 2, 2) == false)
    return 1;

  mgr->start();

  std::vector<ObjInfo> trainSet = readImageSet(mgr->getExtraArg(0).c_str());
  for(uint scene=0; scene<trainSet.size(); scene++)
  {
    LINFO("Train %s %i",
        trainSet[scene].name.c_str(),
        trainSet[scene].cls);
    objRec->train(trainSet[scene].name, trainSet[scene].cls);
  }
  objRec->finalizeTraining();

  //Get the code words
  for(uint scene=0; scene<trainSet.size(); scene++)
  {
    LINFO("Train %s %i",
        trainSet[scene].name.c_str(),
        trainSet[scene].cls);
    objRec->getObjCodeWords(trainSet[scene].name);
  }
  std::vector<ObjInfo> testSet = readImageSet(mgr->getExtraArg(1).c_str());
  for(uint scene=0; scene<testSet.size(); scene++)
  {
    LINFO("Test %s %i",
        testSet[scene].name.c_str(),
        testSet[scene].cls);
    objRec->getObjCodeWords(testSet[scene].name);
  }
  //objRec->finalizeTesting();

  /*
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
   */

  // stop all our ModelComponents
  mgr->stop();

  return 0;

}


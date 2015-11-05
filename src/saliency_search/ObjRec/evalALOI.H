/*! @file ObjRec/evalALOI.H get images from the coil data set to evaluate alg */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ObjRec/evalALOI.H $
// $Id: evalALOI.H 9412 2008-03-10 23:10:15Z farhan $
//

#include "Media/SceneGenerator.H"

#define DEBUG 1

#ifdef DEBUG
#include "GUI/DebugWin.H"
#endif

void evalALOI(DescriptorVec &descVec, Bayes &bayesNet, int nobj);
int trainALOI(TestImages &testImages, DescriptorVec &descVec, Bayes &bayesNet, int nobj);
int testALOI(TestImages &testImages, DescriptorVec &descVec, Bayes &bayesNet, int nobj);
int testSceneALOI(DescriptorVec &descVec, Bayes &bayesNet, int nobj);
int testBiasedSceneALOI(DescriptorVec &descVec, Bayes &bayesNet, int nobj, int biasedObj);
int trainTestALOI(TestImages &testImages, DescriptorVec &descVec, Bayes &bayesNet, int nobj);

//Max lum 12, col 24, rot 72
#define MAXLUM 12
#define MAXCOL 24
#define MAXROT 72

void evalALOI(DescriptorVec &descVec, Bayes &bayesNet, int nobj, int biasedObj, int train)
{
  TestImages testImages("/lab/ilab15/tmp/objectsDB/png/png", TestImages::ALOI);

  //////////////////////////////////////////////// Main test Loop //////////////////////////////////////
  printf("Training\n");

  if (train)
    trainALOI(testImages, descVec, bayesNet, nobj);
  //  trainSceneALOI(descVec, bayesNet, nobj);


  //for(uint i=0; i<bayesNet.getNumFeatures(); i++)
  //   LINFO("F:%i: Mean %f sigma:%f", i, bayesNet.getMean(9, i), bayesNet.getStdevSq(9,i));

 // printf("Testing Obj rec\n");
 // getchar();
 //testALOI(testImages, descVec, bayesNet, nobj);
 //

 // printf("Testing Scene rec\n");
 // getchar();
  //testSceneALOI(descVec, bayesNet, nobj);

 if (!train)
  //testBiasedSceneALOI(descVec, bayesNet, nobj, biasedObj);
   testALOI(testImages, descVec, bayesNet, nobj);
 //
 //

  printf("Training while testing");

 // trainTestALOI(testImages, descVec, bayesNet, nobj);
  //bayesNet.save("objRecALOI.net");

}


int trainALOI(TestImages &testImages, DescriptorVec &descVec, Bayes &bayesNet, int nobj)
{

  Dims trainSize(OBJSIZEX, OBJSIZEY);
  descVec.setFoveaSize(trainSize);

  //Train on half the objects veriations
  for(int obj=0; obj<nobj; obj++){ //for each object
    for(uint lum=0; lum<testImages.getMaxLum(); lum+=2 ){
      LINFO("Training on object %i lum: %i", obj, lum);
      Image< PixRGB<byte> > input = testImages.getObject(obj, lum,-2, -2) ;
      input = rescale(input, trainSize); //resize the image
      learnImage(input, obj, descVec, bayesNet);
    }

    for(uint col=0; col<testImages.getMaxCol(); col+=2){
      LINFO("Training on object %i col: %i", obj, col);
      Image< PixRGB<byte> > input = testImages.getObject(obj, -2, col, -2) ;
      input = rescale(input, trainSize); //resize the image
      learnImage(input, obj, descVec, bayesNet);
    }

    for(uint rot=0; rot<testImages.getMaxRot(); rot+=2 ){
      LINFO("Training on object %i rot: %i", obj, rot);
      Image< PixRGB<byte> > input = testImages.getObject(obj, -2,-2, rot) ;
      input = rescale(input, trainSize); //resize the image
      learnImage(input, obj, descVec, bayesNet);
    }
  }

  return 0;
}

int testALOI(TestImages &testImages, DescriptorVec &descVec, Bayes &bayesNet, int nobj)
{

  //Train with only one image
  Dims testSize(OBJSIZEX, OBJSIZEY);
  descVec.setFoveaSize(testSize);
  bool testLum=true, testCol=true, testRot = true, testScale = false;
  int numLumTargetMatches = 0; int totalLumImages = 0;
  int numColTargetMatches = 0; int totalColImages = 0;
  int numRotTargetMatches = 0; int totalRotImages = 0;

  for(int obj=0; obj<nobj; obj++){ //for each object
    if (testLum)
    {
      printf("# Checking obj %i over variations in LUM\n", obj);
      for(uint lum=0; lum<testImages.getMaxLum(); lum++){
        Image< PixRGB<byte> > input = testImages.getObject(obj, lum,-2, -2) ;
        input = rescale(input, testSize); //resize the image

        int cls = classifyImage(input, descVec, bayesNet);
        totalLumImages++;
        if (cls == obj)
          numLumTargetMatches++;
        printf("LUM: Obj %i is class %i\n", obj, cls);
      }
    }

    if (testCol)
    {
      printf("# Checking obj %i over variations in COLOR\n", obj);
      for(uint col=0; col<testImages.getMaxCol(); col++){
        Image< PixRGB<byte> > input = testImages.getObject(obj, -2, col, -2) ;
        input = rescale(input, testSize); //resize the image

        int cls = classifyImage(input, descVec, bayesNet);
        totalColImages++;
        if (cls == obj)
          numColTargetMatches++;
        printf("COL: Obj %i is class %i\n", obj, cls);
      }
    }

    if (testRot)
    {
      printf("# Checking obj %i over variations in ROT\n", obj);
      for(uint rot=0; rot<testImages.getMaxRot(); rot++){
        Image< PixRGB<byte> > input = testImages.getObject(obj, -2,-2, rot) ;
        input = rescale(input, testSize); //resize the image

        int cls = classifyImage(input, descVec, bayesNet);
        totalRotImages++;
        if (cls == obj)
          numRotTargetMatches++;

        printf("Rot: Obj %i is class %i\n", obj, cls);

      }
    }

    if (testScale)
    {
    }

    int numTargetMatches = numLumTargetMatches + numColTargetMatches + numRotTargetMatches;
    int totalImages = totalLumImages + totalColImages + totalRotImages;

    printf("Number Lum of matches %i/%i %0.2f\n",
        numLumTargetMatches, totalLumImages,
        (float)numLumTargetMatches/(float)totalLumImages);
    printf("Number Col of matches %i/%i %0.2f\n",
        numColTargetMatches, totalColImages,
        (float)numColTargetMatches/(float)totalColImages);
    printf("Number Rot of matches %i/%i %0.2f\n",
        numRotTargetMatches, totalRotImages,
        (float)numRotTargetMatches/(float)totalRotImages);
    printf("Number total of matches %i/%i %0.2f\n",
        numTargetMatches, totalImages,
        (float)numTargetMatches/(float)totalImages);

  }

  return 0; //numTargetMatches;
}

int testSceneALOI(DescriptorVec &descVec, Bayes &bayesNet, int nobj)
{
  Dims objSize(80, 80);
  int numOfObjects = nobj;
  int spacing = objSize.w();

  int numTargetMatches = 0; int totalObjects = 0;

  int sceneW = (int)((objSize.w() + spacing) * sqrt(numOfObjects));
  int sceneH = (int)((objSize.h() + spacing) * sqrt(numOfObjects));
  LINFO("Scene size %ix%i", sceneW, sceneH);

  SceneGenerator testScenes(SceneGenerator::ALOI_OBJECTS, sceneW, sceneH);
  testScenes.setObjectSize(objSize.w());

  printf("# Testing over variations in LUM\n");
  for(int lum=0; lum<MAXLUM; lum++)
  {
    Image<PixRGB<byte> > scene = testScenes.getScene(nobj, lum, -2, -2);

    Point2D<int> winner = evolveBrain(scene, descVec); //evolve the biased brain

    for(uint obj=0; obj<testScenes.getNumObj(); obj++)
    {

      Point2D<int> objLoc = testScenes.getObjLocation(obj);
      int objId = testScenes.getObjId(obj);

      int cls = classifyLocation(objLoc, descVec, bayesNet);
      totalObjects++;
      if (cls == objId)
        numTargetMatches++;
      printf("Obj %i is class %i\n", objId, cls);

     // Image<PixRGB<byte> > tmp = scene;
     // drawCircle(tmp, objLoc, objSize.w()/2, PixRGB<byte>(255, 0, 0), 3);
     // SHOWIMG(rescale(tmp, 512, 512));

    }
  }

  printf("# Testing over variations in COL\n");
  for(int col=0; col<MAXCOL; col++)
  {
    Image<PixRGB<byte> > scene = testScenes.getScene(nobj, -2, col, -2);

    Point2D<int> winner = evolveBrain(scene, descVec); //evolve the biased brain

    for(uint obj=0; obj<testScenes.getNumObj(); obj++)
    {

      Point2D<int> objLoc = testScenes.getObjLocation(obj);
      int objId = testScenes.getObjId(obj);

      int cls = classifyLocation(objLoc, descVec, bayesNet);
      totalObjects++;
      if (cls == objId)
        numTargetMatches++;
      printf("Obj %i is class %i\n", objId, cls);

     // Image<PixRGB<byte> > tmp = scene;
     // drawCircle(tmp, objLoc, objSize.w()/2, PixRGB<byte>(255, 0, 0), 3);
     // SHOWIMG(rescale(tmp, 512, 512));

    }
  }

  printf("# Testing over variations in Rot\n");
  for(int rot=0; rot<MAXROT; rot++)
  {
    Image<PixRGB<byte> > scene = testScenes.getScene(nobj, -2, -2, rot);

    Point2D<int> winner = evolveBrain(scene, descVec); //evolve the biased brain

    for(uint obj=0; obj<testScenes.getNumObj(); obj++)
    {

      Point2D<int> objLoc = testScenes.getObjLocation(obj);
      int objId = testScenes.getObjId(obj);

      int cls = classifyLocation(objLoc, descVec, bayesNet);
      totalObjects++;
      if (cls == objId)
        numTargetMatches++;
      printf("Obj %i is class %i\n", objId, cls);

     // Image<PixRGB<byte> > tmp = scene;
     // drawCircle(tmp, objLoc, objSize.w()/2, PixRGB<byte>(255, 0, 0), 3);
     // SHOWIMG(rescale(tmp, 512, 512));

    }
  }

  printf("Number of matches %i/%i %0.2f\n",
      numTargetMatches, totalObjects,
      (float)numTargetMatches/(float)totalObjects);

  return 0;
}

int testBiasedSceneALOI(DescriptorVec &descVec, Bayes &bayesNet, int nobj, int biasedObj)
{
  Dims objSize(80, 80);
  int numOfObjects = nobj;
  int spacing = objSize.w();

  int numTargetMatches = 0; int totalObjects = 0;
  int totalSaccFound = 0; int totalScenes = 0;

  int sceneW = (int)((objSize.w() + spacing) * sqrt(numOfObjects));
  int sceneH = (int)((objSize.h() + spacing) * sqrt(numOfObjects));
  LINFO("Scene size %ix%i", sceneW, sceneH);

  SceneGenerator testScenes(SceneGenerator::ALOI_OBJECTS, sceneW, sceneH);
  testScenes.setObjectSize(objSize.w());
  testScenes.setTargetObject(biasedObj);

  int ii=0;
  printf("# Testing over variations in LUM\n");
 for(int lum=0; lum<MAXLUM; lum++)
  {
    Image<PixRGB<byte> > scene = testScenes.getScene(nobj, lum, -2, -2);

    //SHOWIMG(rescale(scene, 512, 512));
    Point2D<int> winner = evolveBrain(scene, descVec); //evolve the biased brain
    totalScenes++;

    int sacc = 0;
    for(sacc=0; sacc<numOfObjects; sacc++)
    {
      int objId = testScenes.getObjFromPos(winner, objSize);

      int cls = classifyLocation(winner, descVec, bayesNet);
      totalObjects++;
      if (cls == objId)
        numTargetMatches++;
      printf("lum %i Obj %i is class %i BiasedObj %i\n", lum, objId, cls, biasedObj);

    //  Image<PixRGB<byte> > tmp = scene;
    //  drawCircle(tmp, winner, objSize.w()/2, PixRGB<byte>(255, 0, 0), 3);
    //  SHOWIMG(rescale(tmp, 512, 512));
      //

      if (objId == biasedObj)
        break;

      Image<PixRGB<byte> > nullImg;
      winner = evolveBrain(nullImg, descVec); //evolve the biased brain to get a new winner

    }
    totalSaccFound += (sacc+1);
    printf("Found the object in %i saccads (Total %i)\n", sacc+1, totalSaccFound);
  }


  printf("# Testing over variations in COL\n");
  for(int col=2; col<MAXCOL; col++)
  {
    Image<PixRGB<byte> > scene = testScenes.getScene(nobj, -2, col, -2);

    Point2D<int> winner = evolveBrain(scene, descVec); //evolve the biased brain
    totalScenes++;

    int sacc = 0;
    for(sacc=0; sacc<numOfObjects; sacc++)
    {
      int objId = testScenes.getObjFromPos(winner, objSize);

      int cls = classifyLocation(winner, descVec, bayesNet);
      totalObjects++;
      if (cls == objId)
        numTargetMatches++;
      printf("Col: %i Obj %i is class %i\n", col, objId, cls);

   //  Image<PixRGB<byte> > tmp = scene;
   //  drawCircle(tmp, winner, objSize.w()/2, PixRGB<byte>(255, 0, 0), 3);
   //  SHOWIMG(rescale(tmp, 512, 512));

      if (objId == biasedObj)
        break;

      Image<PixRGB<byte> > nullImg;
      winner = evolveBrain(nullImg, descVec); //evolve the biased brain to get a new winner

    }
    totalSaccFound += (sacc+1);
    printf("Found the object in %i saccads (Total %i)\n", sacc+1, totalSaccFound);
  }

  printf("# Testing over variations in ROT\n");
  for(int rot=0; rot<MAXROT; rot++)
  {
    Image<PixRGB<byte> > scene = testScenes.getScene(nobj, -2, -2, rot);

    Point2D<int> winner = evolveBrain(scene, descVec, ii); //evolve the biased brain
    totalScenes++;

    int sacc = 0;
    for(sacc=0; sacc<numOfObjects; sacc++)
    {
      int objId = testScenes.getObjFromPos(winner, objSize);

      int cls = classifyLocation(winner, descVec, bayesNet);
      totalObjects++;
      if (cls == objId)
        numTargetMatches++;
      printf("Obj %i is class %i\n", objId, cls);

     // SHOWIMG(rescale(tmp, 512, 512));

      if (objId == biasedObj) //foudnd the obj we are looking for
        break;

      Image<PixRGB<byte> > nullImg;
      winner = evolveBrain(nullImg, descVec); //evolve the biased brain to get a new winner

    }

    totalSaccFound += (sacc+1);
    printf("Found the object in %i saccads (Total %i)\n", sacc+1, totalSaccFound);
  }

  printf("Number of matches %i/%i %0.2f\n",
      numTargetMatches, totalObjects,
      (float)numTargetMatches/(float)totalObjects);

  printf("Found the object in %i saccads out of %i scenes\n",
      totalSaccFound, totalScenes);

  return 0;
}

int trainTestALOI(TestImages &testImages, DescriptorVec &descVec, Bayes &bayesNet, int nobj)
{

  Dims trainSize(255, 255);
  descVec.setFoveaSize(trainSize);

  float thresh = 0.90;
  float classRate = 0.0;
  int epochs = 0;
  while(classRate < thresh)
  {
    int correctClassify = 0;
    int totalImages = 0;
    for(int obj=0; obj<nobj; obj++){ //for each object

      for(uint lum=0; lum<testImages.getMaxLum(); lum += 2){
        Image< PixRGB<byte> > input = testImages.getObject(obj, lum,-2, -2) ;
        input = rescale(input, trainSize); //resize the image
        totalImages++; //the # of images we seen so far

        //try to classify the image. If we fail, then learn the image
        int cls = classifyImage(input, descVec, bayesNet);
        if (cls != obj)
        {
          printf("Learning obj:%i lum:%i\n", obj, lum);
          learnImage(input, obj, descVec, bayesNet);
        } else {
          correctClassify++;
        }

      }

      for(uint col=0; col<testImages.getMaxCol(); col+=2){
        Image< PixRGB<byte> > input = testImages.getObject(obj, -2, col, -2) ;
        input = rescale(input, trainSize); //resize the image
        totalImages++; //the # of images we seen so far

        //try to classify the image. If we fail, then learn the image
        int cls = classifyImage(input, descVec, bayesNet);
        if (cls != obj)
        {
          printf("Learning obj:%i col:%i\n", obj, col);
          learnImage(input, obj, descVec, bayesNet);
        } else {
          correctClassify++;
        }

      }

      for(uint rot=0; rot<testImages.getMaxRot(); rot += 2 ){
        Image< PixRGB<byte> > input = testImages.getObject(obj, -2,-2, rot) ;
        input = rescale(input, trainSize); //resize the image

        totalImages++; //the # of images we seen so far

        //try to classify the image. If we fail, then learn the image
        int cls = classifyImage(input, descVec, bayesNet);
        if (cls != obj)
        {
          printf("Learning obj:%i rot:%i\n", obj, rot);
          learnImage(input, obj, descVec, bayesNet);
        } else {
          correctClassify++;
        }
      }
    }
    epochs++;
    classRate = (float)correctClassify/(float)totalImages;
    printf("pass %i: correctClassify %i total %i rate:%0.2f\n",
        epochs, correctClassify, totalImages, classRate);
    bayesNet.save("partIncrementalObjRecALOI.net");

  }

  return 0;
}



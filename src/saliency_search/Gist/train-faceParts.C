/*!@file Gist/train-faceParts.C testing Feed Forward Network class
  for use to train any mapping */

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
// Primary maintainer for this file: Christian Siagian <siagian@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Gist/train-faceParts.C $
// $Id: train-faceParts.C 14376 2011-01-11 02:44:34Z pez $
//

// ######################################################################
/*! This class test implementation a simple feedforward network
 with backpropagation. */

#include "Channels/OrientationChannel.H"
#include "Component/ModelManager.H"
#include "GUI/XWinManaged.H"
#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Image/ImageSet.H"
#include "Image/MathOps.H"
#include "Image/PyramidOps.H"
#include "Image/ShapeOps.H"
#include "Image/Transforms.H"
#include "Neuro/StdBrain.H"
#include "Neuro/VisualCortex.H"
#include "Raster/Raster.H"


#include "Gist/FFN.H"
#include "Gist/ModelFace.H"

#define LEARN_RATE                 0.5
#define TRAIN_START_INDEX          277
#define NUM_TRAIN_SAMPLE           200
#define TEST_START_INDEX           234
#define NUM_TEST_SAMPLE            100

CloseButtonListener wList; XWinManaged *imgWin;
Image<byte> targetmask;

Image<float> getPartImage(char* iName, Rectangle pr);
Image<float> getPartImage(Image<float> img, Rectangle pr);

Rectangle getPartRect(char* iName, char* dName, FacePart part, int opp);
Rectangle getPartRect(Image<float> img, char* dName, FacePart part, int opp);

void getFeature(Rectangle r, FacePart part, Image<double> features);
void getEyeFeature  (Rectangle r, Image<double> features);
void getNoseFeature (Rectangle r, Image<double> features);
void getMouthFeature(Rectangle r, Image<double> features);

void train
(FeedForwardNetwork* ffn, FacePart part, nub::soft_ref<StdBrain> brain);
void fillEyeData
( int start, std::vector<Image<double> > pData,
  std::vector<Image<double> >nData, int *pCt, int *nCt);
void fillNoseData
( int start, std::vector<Image<double> > pData,
  std::vector<Image<double> > nData, int *pCt, int *nCt);
void fillMouthData
( int start, std::vector<Image<double> > pData,
  std::vector<Image<double> > nData, int *pCt, int *nCt);

bool within(Rectangle r, int x, int y);

nub::soft_ref<OrientationChannel> oriChan;

// CURRENT IMAGE BEING PROCESS - MINIMIZING PROCESSING
// IN REAL APPLICATION A BRAIN WOULD DO THIS
Image<float> currImg;
ImageSet<float> currGaPyr[NUM_DIR];

void printRegion(Image<float> img,int sX,int eX,int dX, int sY,int eY, int dY);

// testing feed forward network to teach a network to detect an object
int main(const int argc, const char **argv)
{

  // Instantiate a ModelManager:
  ModelManager manager("Face Recognition Model");

  // Instantiate our various ModelComponents:
  nub::soft_ref<StdBrain> brain(new StdBrain(manager));
  manager.addSubComponent(brain);

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv,"", 0, 0) == false)
    return(1);

  // let's get all our ModelComponent instances started:
  manager.start();
  LFATAL("fixme");
  nub::soft_ref<VisualCortex> vc;////// = brain->getVC();
  ///////  dynCastWeakToFrom(oriChan, vc->subChan("orientation"));

  // seed the random number generator
  srand((unsigned)time(0));

  // instantiate a feed forward network for each face part
  FeedForwardNetwork *ffn_e = new FeedForwardNetwork();
  FeedForwardNetwork *ffn_n = new FeedForwardNetwork();
  FeedForwardNetwork *ffn_m = new FeedForwardNetwork();

  // get the weight values for each part
  char h1EName[200],h2EName[200],oEName[200];
  sprintf(h1EName,"%s%s%s",WEIGHT_DATA_FOLDER,FILE_TAGNAME,"hidden1EYE.dat");
  sprintf(h2EName,"%s%s%s",WEIGHT_DATA_FOLDER,FILE_TAGNAME,"hidden2EYE.dat");
  sprintf(oEName ,"%s%s%s",WEIGHT_DATA_FOLDER,FILE_TAGNAME,"outEYE.dat");

  char h1NName[200],h2NName[200],oNName[200];
  sprintf(h1NName,"%s%s%s",WEIGHT_DATA_FOLDER,FILE_TAGNAME,"hidden1NOSE.dat");
  sprintf(h2NName,"%s%s%s",WEIGHT_DATA_FOLDER,FILE_TAGNAME,"hidden2NOSE.dat");
  sprintf(oNName ,"%s%s%s",WEIGHT_DATA_FOLDER,FILE_TAGNAME,"outNOSE.dat");

  char h1MName[200],h2MName[200],oMName[200];
  sprintf(h1MName,"%s%s%s",WEIGHT_DATA_FOLDER,FILE_TAGNAME,"hidden1MOUTH.dat");
  sprintf(h2MName,"%s%s%s",WEIGHT_DATA_FOLDER,FILE_TAGNAME,"hidden2MOUTH.dat");
  sprintf(oMName ,"%s%s%s",WEIGHT_DATA_FOLDER,FILE_TAGNAME,"outMOUTH.dat");

  // initialize the size of network
  // use 3-weight layer network
  ffn_e->init3L(h1EName, h2EName, oEName,
                RE_INPUT, RE_HIDDEN_1, RE_HIDDEN_2, 1, LEARN_RATE, 0.0);
  ffn_n->init3L(h1NName, h2NName, oNName,
                N_INPUT,   N_HIDDEN_1,  N_HIDDEN_2, 1, LEARN_RATE, 0.0);
  ffn_m->init3L(h1MName, h2MName, oMName,
                M_INPUT,   M_HIDDEN_1,  M_HIDDEN_2, 1, LEARN_RATE, 0.0);

  // train each individual face parts
  train(ffn_e, EYE, brain);
  train(ffn_n, NOSE, brain);
  train(ffn_m, MOUTH, brain);

  // save the weight values
  //ffn_e->write3L(h1EName, h2EName, oEName);
  //ffn_n->write3L(h1NName, h2NName, oNName);
  //ffn_m->write3L(h1MName, h2MName, oMName);
}

// ######################################################################
// train the FFN for the part
void train(FeedForwardNetwork *ffn, FacePart part, nub::soft_ref<StdBrain> brain)
{
  // set the appropriate storage size for the specified part
  int pW = 0, pH = 0;
  switch(part)
    {
    case EYE:
      pW = MODEL_EYE_WIDTH;
      pH = MODEL_EYE_HEIGHT;
    break;
    case NOSE:
      pW = MODEL_NOSE_WIDTH;
      pH = MODEL_NOSE_HEIGHT;
    break;
    case MOUTH:
      pW = MODEL_MOUTH_WIDTH;
      pH = MODEL_MOUTH_HEIGHT;
    break;
    default:
      LFATAL("Unknown face part");
    }

  // store positive training samples
  // negative training samples will be generated randomly every 10 epoch
  std::vector<Image<double> > pData(NUM_TRAIN_SAMPLE);
  std::vector<Image<double> > nData(NUM_TRAIN_SAMPLE);

  // the success of the neural net lies in the set of training samples
  // knowing what the boudaries we are looking for
//   int  start = TRAIN_START_INDEX;
  int pCount = 0, nCount = 0;

  // setup the test cases for each part
//   switch(part)
//   {
//   case EYE:
//     fillEyeData  (start, pData, nData, &pCount, &nCount);   break;
//   case NOSE:
//     fillNoseData (start, pData, nData, &pCount, &nCount);   break;
//   case MOUTH:
//     fillMouthData(start, pData, nData, &pCount, &nCount);   break;
//   default:
//     LFATAL("Unknown face part");
//   }

  // train the network
  printf("Start Training\n");
  int nfc = NUM_TRAIN_SAMPLE;
  Image<double> trueout(1,1,ZEROS);
  double errSum = double(2*NUM_TRAIN_SAMPLE);
  int nTrials = 0;
  Image<double> ffnOut;
  while(nTrials < 0) // change to a positive number to train
  {
    errSum = 0.0; nfc = 0; int i = 0;
    while (i < pCount || i < nCount)
    {
      // run the positive sample image
      if(i < pCount)
      {
        ffn->run3L(pData[i]);
        ffnOut = ffn->getOutput();
        errSum += fabs(1.0 - ffnOut[0]);
        if(ffnOut[0] < .5)
        {
          nfc++;
          printf("falsePOS o: %f, err: %f\n",ffnOut[0], 1.0-ffnOut[0]);
        }
        trueout[0] = 1.0; ffn->backprop3L(trueout);
      }

      // run the negative sample image
      if(i < nCount)
      {
        ffn->run3L(nData[i]);
        ffnOut = ffn->getOutput();
        errSum += fabs(ffnOut[0]);
        if(ffnOut[0] > .5)
        {
          nfc++;
          printf("falseNEG: out: %f, err: %f\n",ffnOut[0],ffnOut[0]);
        }
        trueout[0] = 0.0; ffn->backprop3L(trueout);
      }
      i++;
    }
    nTrials++;

    // periodically report progress
    if(nTrials %10 == 0)
      printf("Trial_%04d_Err: %f, nfc: %d \n",
             nTrials,errSum/double(pCount+nCount),nfc);
  }

  // final error count
  printf("Final Trial_%04d_Err: %f, nfc: %d \n",
         nTrials,errSum/double(pCount+nCount),nfc);

  // test the current network on test images.
  printf("Testing Phase\n");

  // traverse through the test cases
//   start = TEST_START_INDEX; errSum = 0.0; nfc = 0;
//   for(int i = start; i < start+NUM_TEST_SAMPLE; i++)
//   {
//     double testData[pSize];
//     char iName[200],dName[200];
//     sprintf(iName, "%s%s%04d%s",IMAGE_FOLDER,FILE_TAGNAME,i,IMAGE_FILE_EXT);
//     sprintf(dName, "%s%s%04d%s",DATA_FOLDER, FILE_TAGNAME,i,DATA_FILE_EXT );

//     // get the part from the image
//     Rectangle pr = getPartRect(iName, dName, part,0);
//     Image<float> *pImg = getPartImage(iName,pr);
//     getFeature(pImg, testData);

//     // run the positive sample image
//     ffn->run3L(testData);
//     errSum += fabs(1.0 - ffn->out[0]);
//     if(ffn->out[0] < .5) nfc++;

//     // get random images from the same file (for negative cases)
//     pr = getPartRect(iName, dName, part,1);
//     pImg = getPartImage(iName,pr);
//     getFeature(pImg,testData);

//     // run the negative sample image
//     ffn->run3L(testData);
//     errSum += fabs(ffn->out[0]);
//     if(ffn->out[0] > .5) nfc++;

//    XWinManaged *eWin = new XWinManaged(Dims(MODEL_EYE_WIDTH*4,MODEL_EYE_WIDTH*4),
//                                        1000,0,"False eye");
//    wList.add(*eWin);
//    XWinManaged *neWin = new XWinManaged(Dims(MODEL_EYE_WIDTH*4,MODEL_EYE_WIDTH*4),
//                                         1000,0,"False non-eye");
//    wList.add(*neWin);
//    eWin->drawImage(zoomXY(eImg[i], 4,-1),0,0);
//    neWin->drawImage(zoomXY(neImg[i], 4,-1),0,0);
//    Raster::waitForKey();
//  }
//  printf("Test_Err: %f, nfc: %d \n", errSum/NUM_TEST_SAMPLE/2.0,nfc);

  // test on an image:
  int tNum = 333;
  char tName[200];
  sprintf(tName, "%s%s%04d%s",IMAGE_FOLDER,FILE_TAGNAME,tNum,IMAGE_FILE_EXT);
  Image<float> tImg = Raster::ReadGray(tName);

    currImg = tImg;
    float cMean  = float(mean(currImg));
    float cStdev = float(stdev(currImg));
    currImg -= cMean;
    currImg /= cStdev;
    for(int i = 0; i< NUM_DIR; i++)
      currGaPyr[i] = buildPyrGabor(currImg,0,4,i*45,7,1,9,0);

  ImageSet<float> tImgPyr = buildPyrGaussian(currImg, 0, 9, 3);
  Image<double> tData;
  Image<float> apa;

  Image<float> res(tImgPyr[1].getWidth(), tImgPyr[1].getHeight(), ZEROS);
  for(int i = 0; i < tImgPyr[1].getWidth() - pW; i++)
    for(int j = 0; j < tImgPyr[1].getHeight() - pH; j++)
    {
      Rectangle a = Rectangle::tlbrI(j,i,j+pH-1,i+pW-1);
      getFeature(a, part, tData);
      ffn->run3L(tData);
      Image<double> ffnOut = ffn->getOutput();
      res.setVal(i+pW/2,j+pH/2,ffnOut[0]);
//       if(ffnOut[0] >.7 & i == 42 && j == 61)
//       {

//         Rectangle r2(a.top()/2, a.left()/2, a.bottom()/2, a.right()/2);
//         printf("t: %d, b: %d, l: %d, r: %d\n",
//        a.top(),a.bottom(),a.left(),a.right());
//         printf("t: %d, b: %d, l: %d, r: %d\n",
//        r2.top(),r2.bottom(),r2.left(),r2.right());

//         printRegion(currGaPyr[0][1],i,i+11,1,j,j+13,1);
//         for(int t = 0; t < 61*4; t++)
//           printf("%2d - %f \n",t, tData[t]);
//         printf("i: %d j: %d out: %f\n",i,j,ffnOut[0]);
//         apa = getPartImage(tImg,a);
//         Raster::waitForKey();
//       }
    }
  XWinManaged *eWin = new XWinManaged(Dims(2*2*tImgPyr[1].getWidth(),
                                           2*tImgPyr[1].getHeight()),
                                      700,0,"TestImage");
  wList.add(*eWin);
  eWin->drawImage(tImg+(zoomXY(res, 2,-1)*255),0,0);
  eWin->drawImage(zoomXY(res, 2,-1),2*tImgPyr[1].getWidth(),0);
  Raster::waitForKey();
}

// ######################################################################
void fillEyeData
(int start,
 std::vector<Image<double> > pData,
 std::vector<Image<double> > nData,
 int *pCt, int  *nCt)
{
  int pW = MODEL_EYE_WIDTH, pH = MODEL_EYE_HEIGHT;
  char iName[200],dName[200];
  int pCount = 0; int nCount = 0;

  // get positive samples of eyes(left and right) from the files
  for(int s = 0; s < NUM_TRAIN_SAMPLE/2; s++)
  {
    sprintf(iName, "%s%s%04d%s",
            IMAGE_FOLDER,FILE_TAGNAME,start+s,IMAGE_FILE_EXT);
    sprintf(dName, "%s%s%04d%s",
            DATA_FOLDER, FILE_TAGNAME,start+s,DATA_FILE_EXT );
    if(s%10 == 0)
      printf("setting up positive image %d\n",s);

    currImg = Raster::ReadGray(iName);
    float cMean  = float(mean(currImg));
    float cStdev = float(stdev(currImg));
    currImg -= cMean;
    currImg /= cStdev;

    for(int i = 0; i< NUM_DIR; i++)
      currGaPyr[i] = buildPyrGabor(currImg,0,4,i*45,7,1,9,0);

    // get the left eye from the image
    getEyeFeature(getPartRect(currImg, dName, L_EYE, 0),
                   pData[pCount]);
    pCount++;

    // get the right eye from the image
    getEyeFeature(getPartRect(currImg, dName, R_EYE, 0),
                   pData[pCount]);
    pCount++;
  }

  // get negative-case images from the last file
  Rectangle ler = getPartRect(currImg, dName, L_EYE, 0);
  Rectangle rer = getPartRect(currImg, dName, R_EYE, 0);
  int w = currImg.getWidth(), h = currImg.getHeight();

  // This code is changed over and over - samples are provided by user
  // from area above the eye
  for (int i = w/5/2+5; i < 2*w/5-10; i+= pW/2)
    for(int j = 15; j < std::min(ler.top(),rer.top())-10;j+= pH)
    {
      getEyeFeature(Rectangle(Point2D<int>(i,j), Dims(pW, pH)),
                     nData[nCount]); nCount++;
    }

  // from the area below the eyes
  for (int i = w/5/2; i < 2*w/5-10; i+= pW/2)
    for(int j = std::max(ler.bottomI(),rer.bottomI())+5; j < h/2-pH; j+= pH)
    {
      getEyeFeature(Rectangle(Point2D<int>(i,j), Dims(pW, pH)),
                     nData[nCount]); nCount++;
    }

  *nCt = nCount;
  *pCt = pCount;
  printf("pCount: %d, nCount: %d \n", pCount, nCount);
}

// ######################################################################
void fillNoseData
(int start,
 std::vector<Image<double> > pData,
 std::vector<Image<double> > nData,
 int *pCt, int  *nCt)
{
  int pW = MODEL_NOSE_WIDTH, pH = MODEL_NOSE_HEIGHT;
  char iName[200],dName[200];
  int pCount = 0; int nCount = 0;

  // get positive samples of noses from the files
  for(int s = 0; s < NUM_TRAIN_SAMPLE; s++)
  {
    sprintf(iName, "%s%s%04d%s",
            IMAGE_FOLDER,FILE_TAGNAME,start+s,IMAGE_FILE_EXT);
    sprintf(dName, "%s%s%04d%s",
            DATA_FOLDER, FILE_TAGNAME,start+s,DATA_FILE_EXT );
    if(s%10 == 0)
      printf("setting up positive image %d\n",s);

    currImg = Raster::ReadGray(iName);
    float cMean  = float(mean(currImg));
    float cStdev = float(stdev(currImg));
    currImg -= cMean;
    currImg /= cStdev;
    for(int i = 0; i< NUM_DIR; i++)
      currGaPyr[i] = buildPyrGabor(currImg,0,4,i*45,7,1,9,0);

    // get the nose from the image
    getNoseFeature(getPartRect(currImg, dName, NOSE, 0),
                    pData[pCount]);
    pCount++;
  }

  // get negative-case images from the last file
  Rectangle nr = getPartRect(currImg, dName, NOSE, 0);
  int w = currImg.getWidth(), h = currImg.getHeight();

  // from the area above the nose
  for (int i = w/5/2; i < 2*w/5; i+= pW/3)
    for(int j = 0; j < nr.top()-pH;j+= pH/2)
    {
      getNoseFeature(Rectangle(Point2D<int>(i,j), Dims(pW, pH)),
                      nData[nCount]); nCount++;
    }

  // from the area below the nose
  for (int i = w/5/2; i < 2*w/5; i+= pW/2)
    for(int j = nr.bottomI(); j < h/2-pH; j+= pH/2)
    {
      getNoseFeature(Rectangle(Point2D<int>(i,j), Dims(pW, pH)),
                      nData[nCount]); nCount++;
    }

  // from the area on the left side of the nose
  for (int i = w/5/2; i < nr.left()-10; i+= pW/2)
    for(int j = nr.top()-pH; j < nr.bottomI()+pH; j+= pH/2)
    {
      getNoseFeature(Rectangle(Point2D<int>(i,j), Dims(pW, pH)),
                      nData[nCount]); nCount++;
    }

  // from the area on the right side of the nose
  for (int i = nr.rightI() + 5; i < 2*w/5; i+= pW/2)
    for(int j = nr.top()-pH; j < nr.bottomI()+pH; j+= pH/2)
    {
      getNoseFeature(Rectangle(Point2D<int>(i,j), Dims(pW, pH)),
                      nData[nCount]); nCount++;
    }
  *nCt = nCount;
  *pCt = pCount;
  printf("pCount: %d, nCount: %d \n",pCount,nCount);
}

// ######################################################################
void fillMouthData
( int start,
  std::vector<Image<double> > pData,
  std::vector<Image<double> > nData,
  int *pCt, int  *nCt)
{
  int pW = MODEL_MOUTH_WIDTH, pH = MODEL_MOUTH_HEIGHT;
  char iName[200],dName[200];
  int pCount = 0; int nCount = 0;

  // get positive samples of mouths from the files
  for(int s = 0; s < NUM_TRAIN_SAMPLE; s++)
  {
    sprintf(iName, "%s%s%04d%s",
            IMAGE_FOLDER,FILE_TAGNAME,start+s,IMAGE_FILE_EXT);
    sprintf(dName, "%s%s%04d%s",
            DATA_FOLDER, FILE_TAGNAME,start+s,DATA_FILE_EXT );
    if(s%10 == 0)
      printf("setting up positive image %d\n",s);

    currImg = Raster::ReadGray(iName);
    float cMean  = float(mean(currImg));
    float cStdev = float(stdev(currImg));
    currImg -= cMean;
    currImg /= cStdev;
    for(int i = 0; i< NUM_DIR; i++)
      currGaPyr[i] = buildPyrGabor(currImg,0,4,i*45,7,1,9,0);

    // get the Mouth from the image
     getMouthFeature(getPartRect(currImg, dName, MOUTH, 0),
                     pData[pCount]);
    pCount++;
  }

  // get negative-case images from the last file
  Rectangle mr = getPartRect(currImg, dName, MOUTH, 0);
  int w = currImg.getWidth(), h = currImg.getHeight();

  // from the area above the mouth
  for (int i = w/5/2; i < 2*w/5; i+= pW/2)
    for(int j = 0; j < mr.top()-pH;j+= pH/2)
    {
      getMouthFeature(Rectangle(Point2D<int>(i,j), Dims(pW, pH)),
                      nData[nCount]); nCount++;
    }

  // from the area below the mouth
  for (int i = w/5/2; i < 2*w/5; i+= pW/2)
    for(int j = mr.bottomI(); j < h/2-pH; j+= pH/2)
    {
      getMouthFeature(Rectangle(Point2D<int>(i,j), Dims(pW, pH)),
                      nData[nCount]); nCount++;
    }

  // from the area on the left side of the mouth
  for (int i = w/5/2; i < mr.left()-10; i+= pW/2)
    for(int j = mr.top(); j < mr.bottomI(); j+= pH/2)
    {
      getMouthFeature(Rectangle(Point2D<int>(i,j), Dims(pW, pH)),
                       nData[nCount]); nCount++;
    }

  // from the area on the right side of the mouth
  for (int i = mr.rightI() + 5; i < 2*w/5; i+= pW/2)
    for(int j = mr.top(); j < mr.bottomI(); j+= pH/2)
    {
      getMouthFeature(Rectangle(Point2D<int>(i,j), Dims(pW, pH)),
                      nData[nCount]); nCount++;
    }

  *nCt = nCount;
  *pCt = pCount;
  printf("pCount: %d, nCount: %d \n",pCount,nCount);
}

// ######################################################################
// get nose features
void getFeature(Rectangle r, FacePart part, Image<double> features)
{
  switch(part)
    {
    case EYE:   getEyeFeature  (r, features);     break;
    case NOSE:  getNoseFeature (r, features);     break;
    case MOUTH: getMouthFeature(r, features);     break;
    default:
      LFATAL("Unknown face part");
    }
}

// ######################################################################
// get nose features
void getEyeFeature(Rectangle r, Image<double> features)
{
  //getPartImage(currImg,r);
  // image -> array to be inputted
  int k = 0;
  Rectangle r2 =
    Rectangle::tlbrI(r.top()/2               , r.left()/2              ,
                     r.top()/2+r.height()/2-1, r.left()/2+r.width()/2-1 );


  // the eye model is 12 by 6
  // we will extract information from specific points only
  // so as to reduce the input numbers

  // for level 1
  // ______________
  // |    XXXX    |
  // |X  X    X  X|
  // |X  X    X  X|
  // |X  X    X  X|
  // |X  X    X   |
  // |____XXXX____|

  int totalF1 = 24;
  int xF1[24] = { 4, 5, 6, 7, 4, 5, 6, 7, 0, 3, 8,11,
                  0, 3, 8,11, 0, 3, 8,11, 0, 3, 8,11 };
  int yF1[24] = { 0, 0, 0, 0, 5, 5, 5, 5, 1, 1, 1, 1,
                  2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4 };

  // for level 2
  // ________
  // |  XX  |
  // |X XX X|
  // |__XX__|

  int totalF2 = 8;
  int xF2[8]  = { 2, 3, 0, 2, 3, 5, 2, 3 };
  int yF2[8]  = { 0, 0, 1, 1, 1, 1, 2, 2 };

  // get feature from all angle of gabor response
  for(int n = 0; n < NUM_DIR; n++)
  {
    for(int i = 0; i< totalF1; i++)
    {
        features[k] = double(currGaPyr[n][1].getVal(
                        r.left()+xF1[i], r.top()+yF1[i]));
        k++;
    }

    for(int i = 0; i< totalF2; i++)
    {
        features[k] = double(currGaPyr[n][2].getVal(
                        r2.left()+xF2[i], r2.top()+yF2[i]));
        k++;
    }
  }
}

// ######################################################################
// get nose features
void getNoseFeature(Rectangle r, Image<double> features)
{
  //getPartImage(currImg,r);
  // image -> array to be inputted
  int k = 0;
  Rectangle r2 =
    Rectangle::tlbrI(r.top()/2               , r.left()/2              ,
                    r.top()/2+r.height()/2-1, r.left()/2+r.width()/2-1 );

  // the Nose model is 20 by 10
  // we will extract information from specific points only
  // so as to reduce the number of inputs

  // for level 1
  // ________________
  // |              |
  // |              |
  // |      XX      |
  // |      XX      |
  // |      XX      |
  // |      XX      |
  // |      XX      |
  // |      XX      |
  // |      XX      |
  // |      XX      |
  // |      XX      |
  // |      XX      |
  // |      XX      |
  // |   XXXXXXXX   |
  // |   X      X   |
  // |   X      X   |
  // |   X      X   |
  // |   X      X   |
  // |   XXXXXXXX   |
  // |              |
  // |______________|

  int totalF1 = 44;
  int xF1[44] = { 6, 7, 6, 7, 6, 7, 6, 7, 6, 7, 6, 7, 6, 7, 6, 7, 6, 7, 6, 7,
                  3, 4, 5, 6, 7, 8, 9,10, 3,10, 3,10, 3,10, 3,10, 3, 4, 5, 6,
                  7, 8, 9,10 };
  int yF1[44] = { 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9,10,10,11,11,
                 12,12,12,12,12,12,12,12,13,13,14,14,15,15,16,16,17,17,17,17,
                 17,17,17,17 };

  // for level 2
  // _________
  // |       |
  // |   X   |
  // |   X   |
  // |   X   |
  // |   X   |
  // |   X   |
  // | XXXXX |
  // | X   X |
  // | XXXXX |
  // |_______|

  int totalF2 = 17;
  int xF2[17]  = { 3, 3, 3, 3, 3, 1, 2, 3, 4, 5, 1, 5, 1, 2, 3, 4, 5 };
  int yF2[17]  = { 1, 2, 3, 4, 5, 6, 6, 6, 6, 6, 7, 7, 8, 8, 8, 8, 8 };

  // get feature from all angle of gabor response
  for(int n = 0; n < NUM_DIR; n++)
  {
    for(int i = 0; i< totalF1; i++)
    {
        features[k] = double(currGaPyr[n][1].getVal(
                        r.left()+xF1[i], r.top()+yF1[i]));
        k++;
    }

    for(int i = 0; i< totalF2; i++)
    {
        features[k] = double(currGaPyr[n][2].getVal(
                        r2.left()+xF2[i], r2.top()+yF2[i]));
        k++;
    }
  }
}

// ######################################################################
// get mouth features
void getMouthFeature(Rectangle r, Image<double> features)
{
  //getPartImage(currImg,r);
  // image -> array to be inputted
  int k = 0;
  Rectangle r2 =
    Rectangle::tlbrI(r.top()/2               , r.left()/2              ,
                    r.top()/2+r.height()/2-1, r.left()/2+r.width()/2-1 );

  // the Mouth model is 20 by 10
  // we will extract information from specific points only
  // so as to reduce the number of inputs

  // for level 1
  // ______________________
  // |                    |
  // |                    |
  // |  X    XXXXXX   X   |
  // |  X      XX     X   |
  // |  X      XX     X   |
  // |  X      XX     X   |
  // |  X      XX     X   |
  // |  X    XXXXXX   X   |
  // |                    |
  // |____________________|

  int totalF1 = 32;
  int xF1[32] = { 2, 7, 8, 9,10,11,12,17, 2, 9,10,17, 2, 9,10,17,
                  2, 9,10,17, 2, 9,10,17, 2, 7, 8, 9,10,11,12,17 };
  int yF1[32] = { 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4,
                  5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7 };

  // for level 2
  // ____________
  // |          |
  // | X XXXX X |
  // | X  XX  X |
  // | X XXXX X |
  // |__________|

  int totalF2 = 16;
  int xF2[16]  = { 1, 3, 4, 5, 6, 8, 1, 4, 5, 8, 1, 3, 4, 5, 6, 8 };
  int yF2[16]  = { 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3 };

  // get feature from all angle of gabor response
  for(int n = 0; n < NUM_DIR; n++)
  {
    for(int i = 0; i< totalF1; i++)
    {
        features[k] = double(currGaPyr[n][1].getVal(
                        r.left()+xF1[i], r.top()+yF1[i]));
        k++;
    }

    for(int i = 0; i< totalF2; i++)
    {
        features[k] = double(currGaPyr[n][2].getVal(
                        r2.left()+xF2[i], r2.top()+yF2[i]));
        k++;
    }
  }
}

// ######################################################################
Rectangle getPartRect(char* iName, char* dName, FacePart part, int opp)
{
  Image<float> img = Raster::ReadGray(iName);
  return getPartRect(img, dName, part, opp);
}

// ######################################################################
Rectangle getPartRect(Image <float> img, char* dName, FacePart part, int opp)
{
  int top = 0, bot = 0, left = 0, right = 0, width = 0, height = 0;
  // set the appropriate index for the specified part
  switch(part)
    {
    case EYE:
      top    = NUM_HEADER_LINE + RE_TOP;
      bot    = NUM_HEADER_LINE + RE_BOTTOM;
      left   = NUM_HEADER_LINE + RE_LEFT;
      right  = NUM_HEADER_LINE + RE_RIGHT;
      width  = MODEL_EYE_WIDTH;
      height = MODEL_EYE_HEIGHT;
      break;
    case NOSE:
      top    = NUM_HEADER_LINE + N_TOP;
      bot    = NUM_HEADER_LINE + N_BOTTOM;
      left   = NUM_HEADER_LINE + N_LEFT;
      right  = NUM_HEADER_LINE + N_RIGHT;
      width  = MODEL_NOSE_WIDTH;
      height = MODEL_NOSE_HEIGHT;
      break;
    case MOUTH:
      top    = NUM_HEADER_LINE + M_TOP;
      bot    = NUM_HEADER_LINE + M_BOTTOM;
      left   = NUM_HEADER_LINE + M_LEFT;
      right  = NUM_HEADER_LINE + M_RIGHT;
      width  = MODEL_MOUTH_WIDTH;
      height = MODEL_MOUTH_HEIGHT;
      break;
    case L_EYE:
      top    = NUM_HEADER_LINE + LE_TOP;
      bot    = NUM_HEADER_LINE + LE_BOTTOM;
      left   = NUM_HEADER_LINE + LE_LEFT;
      right  = NUM_HEADER_LINE + LE_RIGHT;
      width  = MODEL_EYE_WIDTH;
      height = MODEL_EYE_HEIGHT;
      break;
    case R_EYE:
      top    = NUM_HEADER_LINE + RE_TOP;
      bot    = NUM_HEADER_LINE + RE_BOTTOM;
      left   = NUM_HEADER_LINE + RE_LEFT;
      right  = NUM_HEADER_LINE + RE_RIGHT;
      width  = MODEL_EYE_WIDTH;
      height = MODEL_EYE_HEIGHT;
      break;
    default:
      LFATAL("Unknown face part");
    }

  // get the size of the image
  if(img.getWidth() == 0)
    LFATAL("Image not found");
  int w = img.getWidth(), h = img.getHeight();

  // parse the data file to get part
  FILE *fp;  char input[100];

  // if data not found assuming to ask for negative sample
  // size still as specified above
  if((fp = fopen(dName,"rb")) == NULL)
    LFATAL("data file not found");

  int temp, temp2, xT, yT;  int xL, yL;  int xB, yB;  int xR, yR;

  // get the coordinate of top of the part
  fseek (fp, 0, SEEK_SET);
  for(int i = 0; i<= top; i++) if (fgets(input, 1000, fp) == NULL) LFATAL("fgets failed");
  sscanf(input, "point %d = [ %d, %d], index = %d", &temp, &xT, &yT, &temp2);

  // get the coordinate of left of the part
  fseek (fp, 0, SEEK_SET);
  for(int i = 0; i<= left; i++) if (fgets(input, 1000, fp) == NULL) LFATAL("fgets failed");
  sscanf(input, "point %d = [ %d, %d], index = %d", &temp, &xL, &yL, &temp2);

  // get the coordinate of bottom of the part
  fseek (fp, 0, SEEK_SET);
  for(int i = 0; i<= bot; i++) if (fgets(input, 1000, fp) == NULL) LFATAL("fgets failed");
  sscanf(input, "point %d = [ %d, %d], index = %d", &temp, &xB, &yB, &temp2);

  // get the coordinate of right of the part
  fseek (fp, 0, SEEK_SET);
  for(int i = 0; i<= right; i++) if (fgets(input, 1000, fp) == NULL) LFATAL("fgets failed");
  sscanf(input, "point %d = [ %d, %d], index = %d", &temp, &xR, &yR, &temp2);

  fclose(fp);

  // adjust coordinates for nose boundary
  switch(part)
    {
    case EYE: case MOUTH: case L_EYE: case R_EYE:break;
    case NOSE:
      if(xB > xL) xL = xB;
      if(xB < xR) xR = xB;
      if(yB > std::min(yL,yR)) yB = std::min(yL,yR);
      break;
    default:
      LFATAL("Unknown face part");
    }
  //     // for debugging
  //     imgWin = new XWinManaged(img.getDims(),1000,700,"Image"); wList.add(*imgWin);
  //     drawCircle(img, Point2D<int>(xT,h-yT),2,255,2);
  //     drawCircle(img, Point2D<int>(xL,h-yL),2,255,1);
  //     drawCircle(img, Point2D<int>(xR,h-yR),2,255,1);
  //     drawCircle(img, Point2D<int>(xB,h-yB),2,255,1);
  //     imgWin->drawImage(zoomXY(img, 1,-1),0,0);
  //     Raster::waitForKey();

  // set the appropriate bounding rectangle for the specified part
  int t = 0, b = 0, l = 0, r = 0;
  t = (h - yT + h - yB)/4 - height/2    ;
  l = (xL + xR)/4         - width/2     ;
  b = (h - yT + h - yB)/4 + height/2 - 1;
  r = (xL + xR)/4         + width/2  - 1;

  // if actually want part that excludes this rectangle
  if(opp)
  {
    Rectangle box =
      Rectangle::tlbrI(t - height/2, l - width/2, b + height/2, r + width/2);

    // get a random coordinate in range:
    // width : [0 ... w/2 - MODEL_EYE_WIDTH  - 1]
    // height: [0 ... h/2 - MODEL_EYE_HEIGHT - 1]
    // but not close to the eyes
    int xRand, yRand;
    xRand = int(rand()/(RAND_MAX + 1.0) * (w/2 - width  - 1));
    yRand = int(rand()/(RAND_MAX + 1.0) * (h/2 - height - 1));

    Rectangle lFace =
      Rectangle::tlbrI(0,0      ,h/2,w/5/2);
    Rectangle rFace =
      Rectangle::tlbrI(0,4*w/5/2,h/2,w/2  );

    while(within(box  , xRand, yRand)||
          within(lFace, xRand, yRand)||
          within(rFace, xRand, yRand)  )
    {
      xRand = int(rand()/(RAND_MAX + 1.0) * (w/2 - width  - 1));
      yRand = int(rand()/(RAND_MAX + 1.0) * (h/2 - height - 1));

    }

    Rectangle res =
      Rectangle::tlbrI(yRand, xRand, yRand+height - 1, xRand+width - 1);

//     printf("bad case:(%d,%d)\n",xRand,yRand);
//     XWinManaged *gfWin = new XWinManaged(img.getDims(),
//                                          1000,0,"GetFeaures:Img");
//     wList.add(*gfWin);
//     drawRect(img, res,255,1);
//     gfWin->drawImage(zoomXY(img, 1,-1),0,0);
//     Raster::waitForKey();

    return res;
  }

  Rectangle res = Rectangle::tlbrI(t,l,b,r);

//     printf("good case\n");
//     XWinManaged *gfWin = new XWinManaged(img.getDims(),
//                                          1000,0,"GetFeature:Img");
//     wList.add(*gfWin);
//     drawRect(img, res,255,1);
//     gfWin->drawImage(zoomXY(img, 1,-1),0,0);
//     Raster::waitForKey();


  return res;

}

// ######################################################################
bool within(Rectangle r, int x, int y)
{
  //printf("t: %d, b: %d, l: %d, r: %d\n",
  //       r.top(),r.bottom(),r.left(),r.right());
  return (x >= r.left() && x <= r.rightI() &&
          y >= r.top()  && y <= r.bottomI()  );
}

// ######################################################################
Image<float> getPartImage(char* iName, Rectangle pr)
{
  Image<float> img = Raster::ReadGray(iName);
  return getPartImage(img,pr);
}

// ######################################################################
//   XWinManaged *gM[4];
//   XWinManaged *fM[4];
  XWinManaged *iWin;
  XWinManaged *gaWin[NUM_DIR][3];
int start_win = 1;
Image<float> getPartImage(Image<float> img, Rectangle pr)
{
  int w = img.getWidth(), h = img.getHeight();
  ImageSet<float> gPyr = buildPyrGaussian(img, 0, 9, 3);
  Image<float> resMap;
  resMap = crop(gPyr[1],pr);
  // resMap = crop(imgPyr[0][1],pr);

  // for debugging
  if(start_win == 1)
    {
      iWin = new XWinManaged(Dims(w*2,h),700,700,"Image");     wList.add(*iWin);
      //start_win = 0;
    }

  float min, max;
  getMinMax(gPyr[1], min,max);
  drawRect(gPyr[1], pr, max, 1);
  iWin->drawImage(zoomXY(gPyr[1], 2,-1),0,0);
  iWin->drawImage(zoomXY(resMap,8,-1),w,0);
  //Raster::waitForKey();

  // Gabor filter debug
  Image<float> temp;
  for(int i = 0; i < NUM_DIR; i++)
    for(int j = 1; j < 4; j++)
    {
      if(start_win == 1)
        gaWin[i][j-1] = new XWinManaged(img.getDims(),i*w,j*h,"imgL");
      temp = zoomXY(currGaPyr[i][j],int(pow(2.0,j-1)),-1);
      temp = crop(temp,pr);
        //drawRect(temp, pr,100,1);
      gaWin[i][j-1]->drawImage(zoomXY(temp,8,-1),0,0);
    }
  start_win = 0;
  Raster::waitForKey();


//    printf("t: %d, b: %d, l: %d, r: %d\n",
//           pr.top(),pr.bottom(),pr.left(),pr.right());
//   printf("w: %d, h: %d\n",w,h);

//     ImageSet<float> imgPyr[NUM_DIR];
//     for(int i = 0; i < NUM_DIR; i++)
//     imgPyr[i] = buildPyrOriented(img, 0, 2, 9, i*45.0, 10.0);

//   Image<float> tempMap[NUM_DIR];
//   Image<float> resMap2[NUM_DIR];

//   //   tempMap = squared(imgPyr[0][1]);
//   for(int i = 0; i < NUM_DIR; i++)
//     {
//       tempMap[i] = imgPyr[i][1];
//       resMap2[i] = crop(tempMap[i],pr);
//       tempMap[i].drawRect(pr, 100, 1);

//       if(start_win == 1)
//      {
//        gM[i] = new XWinManaged(Dims(w,h),i*w,0,"imgL");     wList.add(*gM[i]);
//        fM[i] = new XWinManaged(Dims(w,h),i*w,h,"imgL");     wList.add(*fM[i]);
//      }
//       gM[i]->drawImage(zoomXY(resMap2[i],4,-1),0,0);
//       fM[i]->drawImage(zoomXY(tempMap[i],2,-1),0,0);

//     }

  return resMap;
}

// ######################################################################
// simple image printing procedure
void printRegion(Image<float> img,int sX,int eX,int dX, int sY,int eY, int dY)
{
  for(int j = sY; j<=eY; j+=dY)
  {
    for(int i = sX; i<=eX; i+=dX)
      printf("%8.3f ", img.getVal(i,j));
    printf(" \n");
  }
  printf("\n");
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
